/*
 *  Copyright (C) 2012, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */
#include "include/ssp.h"

/* SSP -> AP Instruction */
#define MSG2AP_INST_BYPASS_DATA			0x37
#define MSG2AP_INST_LIBRARY_DATA		0x01
#define MSG2AP_INST_SELFTEST_DATA		0x02
#define MSG2AP_INST_DEBUG_DATA			0x03

/* Factory data length */
#define ACCEL_FACTORY_DATA_LENGTH		1
#define GYRO_FACTORY_DATA_LENGTH		36
#define MAGNETIC_FACTORY_DATA_LENGTH		26
#define PRESSURE_FACTORY_DATA_LENGTH		1
#define MCU_FACTORY_DATA_LENGTH			5
#define	GYRO_TEMP_FACTORY_DATA_LENGTH		2
#define	GYRO_DPS_FACTORY_DATA_LENGTH		1
#define TEMPHUMIDITY_FACTORY_DATA_LENGTH	1
#define MCU_SLEEP_FACTORY_DATA_LENGTH		FACTORY_DATA_MAX
#define GESTURE_FACTORY_DATA_LENGTH		4

/*************************************************************************/
/* SSP function                                                          */
/*************************************************************************/
void ssp_enable(struct ssp_data *data, bool enable)
{
	pr_info("%s, enable = %d, old enable = %d\n",
		__func__, enable, data->bSspShutdown);

	if (enable && data->bSspShutdown) {
		data->bSspShutdown = false;
		enable_irq(data->iIrq);
		enable_irq_wake(data->iIrq);
	} else if (!enable && !data->bSspShutdown) {
		data->bSspShutdown = true;
		disable_irq(data->iIrq);
		disable_irq_wake(data->iIrq);
	} else
		pr_err("%s, error / enable = %d, old enable = %d\n",
			__func__, enable, data->bSspShutdown);
}

int ssp_remove_sensor(struct ssp_data *data,
	unsigned int uChangedSensor, unsigned int uNewEnable)
{
	u8 uBuf[2];
	int iRet = 0;
	int64_t dSensorDelay = data->adDelayBuf[uChangedSensor];

	ssp_dbg("[SSP]: %s - remove sensor = %d, current state = %d\n",
		__func__, (1 << uChangedSensor), uNewEnable);

	data->adDelayBuf[uChangedSensor] = DEFUALT_POLLING_DELAY;

	if (data->aiCheckStatus[uChangedSensor] == INITIALIZATION_STATE) {
		data->aiCheckStatus[uChangedSensor] = NO_SENSOR_STATE;
		if (uChangedSensor == ACCELEROMETER_SENSOR)
			accelerometer_open_calibration(data);
		else if (uChangedSensor == GYROSCOPE_SENSOR)
			gyroscope_open_calibration(data);
		else if (uChangedSensor == BAROMETER_SENSOR)
			barometer_open_calibration(data);
		else if (uChangedSensor == PROXIMITY_SENSOR) {
			proximity_open_lcd_ldi(data);
			proximity_open_calibration(data);
		} else if (uChangedSensor == MAGNETIC_SENSOR) {
			iRet = mag_open_hwoffset(data);
			if (iRet < 0)
				pr_err("[SSP]: %s - mag_open_hw_offset"
				" failed, %d\n", __func__, iRet);

			iRet = set_hw_offset(data);
			if (iRet < 0) {
				pr_err("[SSP]: %s - set_hw_offset failed\n",
					__func__);
			}
		}
		return 0;
	} else if (uChangedSensor == MAGNETIC_SENSOR) {
		if (mag_store_hwoffset(data))
			pr_err("mag_store_hwoffset success\n");
	}

	if (atomic_read(&data->aSensorEnable) & (1 << uChangedSensor)) {
		uBuf[1] = (u8)get_msdelay(dSensorDelay);
		uBuf[0] = (u8)get_delay_cmd(uBuf[1]);

		send_instruction(data, REMOVE_SENSOR, uChangedSensor, uBuf, 2);
	}
	data->aiCheckStatus[uChangedSensor] = NO_SENSOR_STATE;
	return 0;
}

unsigned int get_msdelay(int64_t dDelayRate)
{
	if (dDelayRate <= SENSOR_NS_DELAY_FASTEST)
		return SENSOR_MS_DELAY_FASTEST;
	else if (dDelayRate <= SENSOR_NS_DELAY_GAME)
		return SENSOR_MS_DELAY_GAME;
	else if (dDelayRate <= SENSOR_NS_DELAY_UI)
		return SENSOR_MS_DELAY_UI;
	else
		return SENSOR_MS_DELAY_NORMAL;
}

unsigned int get_delay_cmd(u8 uDelayRate)
{
	if (uDelayRate <= SENSOR_MS_DELAY_FASTEST)
		return SENSOR_CMD_DELAY_FASTEST;
	else if (uDelayRate <= SENSOR_MS_DELAY_GAME)
		return SENSOR_CMD_DELAY_GAME;
	else if (uDelayRate <= SENSOR_MS_DELAY_UI)
		return SENSOR_CMD_DELAY_UI;
	else
		return SENSOR_CMD_DELAY_NORMAL;
}

void change_sensor_delay(struct ssp_data *data,
		int iSensorType, int64_t dNewDelay)
{
	u8 uBuf[2];
	unsigned int uNewEnable = 0;
	int64_t dTempDelay = data->adDelayBuf[iSensorType];

	if (!(atomic_read(&data->aSensorEnable) & (1 << iSensorType))) {
		data->aiCheckStatus[iSensorType] = NO_SENSOR_STATE;
		return;
	}

	data->adDelayBuf[iSensorType] = dNewDelay;

	switch (data->aiCheckStatus[iSensorType]) {
	case ADD_SENSOR_STATE:
		ssp_dbg("[SSP]: %s - add %u, New = %lldns\n",
			 __func__, 1 << iSensorType, dNewDelay);

		uBuf[1] = (u8)get_msdelay(dNewDelay);
		uBuf[0] = (u8)get_delay_cmd(uBuf[1]);

		if (send_instruction(data, ADD_SENSOR, iSensorType, uBuf, 2)
			!= SUCCESS) {
			uNewEnable =
				(unsigned int)atomic_read(&data->aSensorEnable)
				& (~(unsigned int)(1 << iSensorType));
			atomic_set(&data->aSensorEnable, uNewEnable);

			data->aiCheckStatus[iSensorType] = NO_SENSOR_STATE;
			data->uMissSensorCnt++;
			break;
		}

		data->aiCheckStatus[iSensorType] = RUNNING_SENSOR_STATE;

		if (iSensorType == PROXIMITY_SENSOR) {
			proximity_open_lcd_ldi(data);
			proximity_open_calibration(data);
		}
		break;
	case RUNNING_SENSOR_STATE:
		if (get_msdelay(dTempDelay)
			== get_msdelay(data->adDelayBuf[iSensorType]))
			break;

		ssp_dbg("[SSP]: %s - Change %u, New = %lldns\n",
			__func__, 1 << iSensorType, dNewDelay);

		uBuf[1] = (u8)get_msdelay(dNewDelay);
		uBuf[0] = (u8)get_delay_cmd(uBuf[1]);
		send_instruction(data, CHANGE_DELAY, iSensorType, uBuf, 2);

		break;
	default:
		data->aiCheckStatus[iSensorType] = ADD_SENSOR_STATE;
	}
}

/*************************************************************************/
/* SSP parsing the dataframe                                             */
/*************************************************************************/
void get_3axis_sensordata(char *pchRcvDataFrame, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	s16 iTemp = 0;

	iTemp = (s16)pchRcvDataFrame[(*iDataIdx)++];
	iTemp <<= 8;
	iTemp += pchRcvDataFrame[(*iDataIdx)++];
	sensorsdata->x = iTemp;

	iTemp = (s16)pchRcvDataFrame[(*iDataIdx)++];
	iTemp <<= 8;
	iTemp += pchRcvDataFrame[(*iDataIdx)++];
	sensorsdata->y = iTemp;

	iTemp = (s16)pchRcvDataFrame[(*iDataIdx)++];
	iTemp <<= 8;
	iTemp += pchRcvDataFrame[(*iDataIdx)++];
	sensorsdata->z = iTemp;
}

static void get_factory_data(struct ssp_data *data, int iSensorData,
	char *pchRcvDataFrame, int *iDataIdx)
{
	int iIdx, iTotalLenth = 0;
	unsigned int uTemp = 0;

	switch (iSensorData) {
	case ACCELEROMETER_FACTORY:
		uTemp = (1 << ACCELEROMETER_FACTORY);
		iTotalLenth = ACCEL_FACTORY_DATA_LENGTH;
		break;
	case GYROSCOPE_FACTORY:
		uTemp = (1 << GYROSCOPE_FACTORY);
		iTotalLenth = GYRO_FACTORY_DATA_LENGTH;
		break;
	case MAGNETIC_FACTORY:
		uTemp = (1 << MAGNETIC_FACTORY);
		iTotalLenth = MAGNETIC_FACTORY_DATA_LENGTH;
		break;
	case PRESSURE_FACTORY:
		uTemp = (1 << PRESSURE_FACTORY);
		iTotalLenth = PRESSURE_FACTORY_DATA_LENGTH;
		break;
	case MCU_FACTORY:
		uTemp = (1 << MCU_FACTORY);
		iTotalLenth = MCU_FACTORY_DATA_LENGTH;
		break;
	case GYROSCOPE_TEMP_FACTORY:
		uTemp = (1 << GYROSCOPE_TEMP_FACTORY);
		iTotalLenth = GYRO_TEMP_FACTORY_DATA_LENGTH;
		break;
	case GYROSCOPE_DPS_FACTORY:
		uTemp = (1 << GYROSCOPE_DPS_FACTORY);
		iTotalLenth = GYRO_DPS_FACTORY_DATA_LENGTH;
		break;
	case MCU_SLEEP_FACTORY:
		uTemp = (1 << MCU_SLEEP_FACTORY);
		iTotalLenth = MCU_SLEEP_FACTORY_DATA_LENGTH;
		break;
	case GESTURE_FACTORY:
		uTemp = (1 << GESTURE_FACTORY);
		iTotalLenth = GESTURE_FACTORY_DATA_LENGTH;
		break;
	case TEMPHUMIDITY_CRC_FACTORY:
		uTemp = (1 << TEMPHUMIDITY_CRC_FACTORY);
		iTotalLenth = TEMPHUMIDITY_FACTORY_DATA_LENGTH;
		break;
	}

	ssp_dbg("[SSP]: %s - Factory test data %d\n", __func__, iSensorData);
	for (iIdx = 0; iIdx < iTotalLenth; iIdx++)
		data->uFactorydata[iIdx] = (u8)pchRcvDataFrame[(*iDataIdx)++];

	data->uFactorydataReady = uTemp;
}

int parse_dataframe(struct ssp_data *data, char *pchRcvDataFrame, int iLength)
{
	int iDataIdx, iSensorData;
	struct sensor_value *sensorsdata;

	sensorsdata = kzalloc(sizeof(*sensorsdata), GFP_KERNEL);
	if (sensorsdata == NULL)
		return ERROR;

	for (iDataIdx = 0; iDataIdx < iLength;) {
		if (pchRcvDataFrame[iDataIdx] == MSG2AP_INST_BYPASS_DATA) {
			iDataIdx++;
			iSensorData = pchRcvDataFrame[iDataIdx++];
			if ((iSensorData < 0) ||
				(iSensorData >= (SENSOR_MAX - 1))) {
				pr_err("[SSP]: %s - Mcu data frame1 error %d\n",
					__func__, iSensorData);
				kfree(sensorsdata);
				return ERROR;
			}

			data->get_sensor_data[iSensorData](pchRcvDataFrame,
				&iDataIdx, sensorsdata);
			data->report_sensor_data[iSensorData](data,
				sensorsdata);
		} else if (pchRcvDataFrame[iDataIdx] ==
			MSG2AP_INST_SELFTEST_DATA) {
			iDataIdx++;
			iSensorData = pchRcvDataFrame[iDataIdx++];
			if ((iSensorData < 0) ||
				(iSensorData >= SENSOR_FACTORY_MAX)) {
				pr_err("[SSP]: %s - Mcu data frame2 error %d\n",
					__func__, iSensorData);
				kfree(sensorsdata);
				return ERROR;
			}
			get_factory_data(data, iSensorData, pchRcvDataFrame,
				&iDataIdx);
		} else if (pchRcvDataFrame[iDataIdx] ==
			MSG2AP_INST_DEBUG_DATA) {
			iSensorData
				= print_mcu_debug(pchRcvDataFrame + iDataIdx+1,
						&iDataIdx, iLength);
			if (iSensorData) {
				pr_err("[SSP]: %s - Mcu data frame3 error %d\n",
					__func__, iSensorData);
				kfree(sensorsdata);
				return ERROR;
			}
		} else if (pchRcvDataFrame[iDataIdx] ==
			MSG2AP_INST_LIBRARY_DATA) {
			ssp_sensorhub_handle_data(data,
					pchRcvDataFrame, iDataIdx, iLength);
			break;
		} else
			iDataIdx++;
	}
	kfree(sensorsdata);
	return SUCCESS;
}
