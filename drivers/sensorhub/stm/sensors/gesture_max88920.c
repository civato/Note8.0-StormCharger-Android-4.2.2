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
#include "../include/ssp.h"

#define	VENDOR		"MAXIM"
#define	CHIP_ID		"MAX88920"
#define MODULE_NAME	"geture_sensor"

/* Gesture Sensor Current */
#define DEFUALT_IR_CURRENT    400 //0xF2

static const u16 SET_CURRENT[2][16] = {
	{0, 25, 50, 75, 100, 125, 150, 175,
	225, 250, 275, 300, 325, 350, 375, 400},
	{2, 28, 34, 50, 66,  82,  98,  114,
	130, 146, 162, 178, 194, 210, 226, 242}
};

static void set_gesture_current(struct ssp_data *data, unsigned char uData)
{
	char chTxBuf[2] = { 0, };
	char chRxBuf = 0;
	int iRet = 0, iRetries = DEFAULT_RETRIES;

	if (waiting_wakeup_mcu(data) < 0 ||
		data->fw_dl_state == FW_DL_STATE_DOWNLOADING) {
		pr_info("[SSP] : %s, skip DL state = %d\n", __func__,
			data->fw_dl_state);
		return;
	}

	chTxBuf[0] = MSG2SSP_AP_SENSOR_GESTURE_CURRENT;
	chTxBuf[1] = uData;

	iRet = ssp_read_data(data, chTxBuf, 2, &chRxBuf, 1, DEFAULT_RETRIES);
	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - SENSOR_GESTURE_CURRENT CMD fail %d\n",
			__func__, iRet);
		return;
	} else if (chRxBuf != MSG_ACK) {
		while (iRetries--) {
			mdelay(10);
			pr_err("[SSP]: %s - MSG2SSP_AP_SENSOR_GESTURE_CURRENT "\
				"CMD retry...\n", __func__);
			iRet = ssp_read_data(data, chTxBuf, 2, &chRxBuf, 1,
				DEFAULT_RETRIES);
			if ((iRet == SUCCESS) && (chRxBuf == MSG_ACK))
				break;
		}

		if (iRetries < 0) {
			data->uInstFailCnt++;
			return;
		}
	}

	data->uInstFailCnt = 0;
	pr_info("[SSP]: Gesture Current Setting - %u\n", uData);
}

void report_gesture_data(struct ssp_data *data, struct sensor_value *ges)
{
	data->buf[GESTURE_SENSOR].data[0] = ges->data[0];
	data->buf[GESTURE_SENSOR].data[1] = ges->data[1];
	data->buf[GESTURE_SENSOR].data[2] = ges->data[2];
	data->buf[GESTURE_SENSOR].data[3] = ges->data[3];
	data->buf[GESTURE_SENSOR].data[4] = data->uTempCount;

	data->buf[GESTURE_SENSOR].data[5] = ges->data[5]; /* a_delta */
	data->buf[GESTURE_SENSOR].data[6] = ges->data[6]; /* b_delta */
	data->buf[GESTURE_SENSOR].data[7] = ges->data[7]; /* c_delta */
	data->buf[GESTURE_SENSOR].data[8] = ges->data[8]; /* d_delta */

	input_report_abs(data->gesture_input_dev,
		ABS_RUDDER, data->buf[GESTURE_SENSOR].data[0]);
	input_report_abs(data->gesture_input_dev,
		ABS_WHEEL, data->buf[GESTURE_SENSOR].data[1]);
	input_report_abs(data->gesture_input_dev,
		ABS_GAS, data->buf[GESTURE_SENSOR].data[2]);
	input_report_abs(data->gesture_input_dev,
		ABS_BRAKE, data->buf[GESTURE_SENSOR].data[3]);
	input_report_abs(data->gesture_input_dev,
		ABS_THROTTLE, data->buf[GESTURE_SENSOR].data[4]);
	input_report_abs(data->gesture_input_dev,
		ABS_X, data->buf[GESTURE_SENSOR].data[5]);
	input_report_abs(data->gesture_input_dev,
		ABS_Y, data->buf[GESTURE_SENSOR].data[6]);
	input_report_abs(data->gesture_input_dev,
		ABS_Z, data->buf[GESTURE_SENSOR].data[7]);
	input_report_abs(data->gesture_input_dev,
		ABS_RX, data->buf[GESTURE_SENSOR].data[8]);
	input_sync(data->gesture_input_dev);

	data->uTempCount++;
}

void get_gesture_sensordata(char *pchRcvDataFrame, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	int iTemp, iCnt;

	for (iCnt = 0; iCnt < 9; iCnt++) {
		iTemp = (int)pchRcvDataFrame[(*iDataIdx)++];
		iTemp <<= 8;
		iTemp += pchRcvDataFrame[(*iDataIdx)++];
		sensorsdata->data[iCnt] = (s16)iTemp;
	}
}

static ssize_t gestrue_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR);
}

static ssize_t gestrue_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_ID);
}

static ssize_t raw_data_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d\n",
		data->buf[GESTURE_SENSOR].data[0],
		data->buf[GESTURE_SENSOR].data[1],
		data->buf[GESTURE_SENSOR].data[2],
		data->buf[GESTURE_SENSOR].data[3]);
}

static ssize_t gesture_get_selftest_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	s16 raw_A = 0, raw_B = 0, raw_C = 0, raw_D = 0;
	int iDelayCnt = 0, iRet = 0;
	char chTempBuf[2] = { 0, 10 };
	struct ssp_data *data = dev_get_drvdata(dev);

	iDelayCnt = 0;
	data->uFactorydataReady = 0;
	memset(data->uFactorydata, 0, sizeof(char) * FACTORY_DATA_MAX);

	iRet = send_instruction(data, FACTORY_MODE, GESTURE_FACTORY,
			chTempBuf, 2);

	while (!(data->uFactorydataReady & (1 << GESTURE_FACTORY))
		&& (iDelayCnt++ < 100)
		&& (iRet == SUCCESS))
		msleep(20);

	if ((iDelayCnt >= 100) || (iRet != SUCCESS)) {
		pr_err("[SSP]: %s - Gesture Selftest Timeout!!\n", __func__);
		goto exit;
	}

	raw_A = data->uFactorydata[0];
	raw_B = data->uFactorydata[1];
	raw_C = data->uFactorydata[2];
	raw_D = data->uFactorydata[3];

	pr_info("[SSP] %s: self test A = %d, B = %d, C = %d, D = %d\n",
		__func__, raw_A, raw_B, raw_C, raw_D);

exit:
	return sprintf(buf, "%d,%d,%d,%d\n",
            raw_A, raw_B, raw_C, raw_D);
}

static ssize_t ir_current_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_dbg("[SSP]: %s - Ir_Current Setting = %d\n",
		__func__, data->uIr_Current);

	return sprintf(buf, "%d\n", data->uIr_Current);
}

static ssize_t ir_current_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	u16 uNewIrCurrent = DEFUALT_IR_CURRENT;
	int iRet = 0;
	u16 current_index = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	iRet = kstrtou16(buf, 10, &uNewIrCurrent);
	if (iRet < 0)
		pr_err("[SSP]: %s - kstrtoint failed.(%d)\n", __func__, iRet);
	else {
		for(current_index = 0; current_index < 16; current_index++)
			if (SET_CURRENT[0][current_index] == uNewIrCurrent)
				data->uIr_Current =
					SET_CURRENT[1][current_index];

		set_gesture_current(data, data->uIr_Current);
		data->uIr_Current = uNewIrCurrent;
	}

	ssp_dbg("[SSP]: %s - new Ir_Current Setting : %d\n",
	        __func__, data->uIr_Current);

	return size;
}

static DEVICE_ATTR(vendor, S_IRUGO, gestrue_vendor_show, NULL);
static DEVICE_ATTR(name, S_IRUGO, gestrue_name_show, NULL);
static DEVICE_ATTR(raw_data, S_IRUGO, raw_data_read, NULL);
static DEVICE_ATTR(selftest, S_IRUGO, gesture_get_selftest_show, NULL);
static DEVICE_ATTR(ir_current, S_IRUGO | S_IWUSR | S_IWGRP,
		ir_current_show, ir_current_store);

static struct device_attribute *gesture_attrs[] = {
	&dev_attr_vendor,
	&dev_attr_name,
	&dev_attr_raw_data,
	&dev_attr_selftest,
	&dev_attr_ir_current,
	NULL,
};

static ssize_t show_gesture_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[GESTURE_SENSOR]);
}

static ssize_t set_gesture_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, GESTURE_SENSOR, dNewDelay);

	return size;
}

static struct device_attribute dev_attr_gesture_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_gesture_delay, set_gesture_delay);

int initialize_gesture_sensor(struct ssp_data *data)
{
	int iRet;
	struct input_dev *input_dev;

	/* allocate input_device */
	input_dev = input_allocate_device();
	if (input_dev == NULL)
		return ERROR;

	input_dev->name = MODULE_NAME;
	input_set_capability(input_dev, EV_ABS, ABS_RUDDER);
	input_set_abs_params(input_dev, ABS_RUDDER, 0, 1024, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_WHEEL);
	input_set_abs_params(input_dev, ABS_WHEEL, 0, 1024, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_GAS);
	input_set_abs_params(input_dev, ABS_GAS, 0, 1024, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_BRAKE);
	input_set_abs_params(input_dev, ABS_BRAKE, 0, 1024, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_THROTTLE);
	input_set_abs_params(input_dev, ABS_THROTTLE, 0, 1024, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_X);
	input_set_abs_params(input_dev, ABS_X, 0, 1024, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_Y);
	input_set_abs_params(input_dev, ABS_Y, 0, 1024, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_Z);
	input_set_abs_params(input_dev, ABS_Z, 0, 1024, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_RX);
	input_set_abs_params(input_dev, ABS_RX, 0, 1024, 0, 0);
	input_set_drvdata(input_dev, data);

	/* register input_device */
	iRet = input_register_device(input_dev);
	if (iRet < 0) {
		input_free_device(input_dev);		
		return ERROR;
	}

	iRet = sensors_create_symlink(&input_dev->dev.kobj, input_dev->name);
	if (iRet < 0)
		goto err_creat_symlink;

	iRet = device_create_file(&input_dev->dev,
		&dev_attr_gesture_poll_delay);
	if (iRet < 0)
		goto err_creat_file;

	iRet = sensors_register(data->ges_device, data,
			gesture_attrs, MODULE_NAME);
	if (iRet < 0)
		goto err_sensor_register;

	data->gesture_input_dev = input_dev;
	data->uIr_Current = DEFUALT_IR_CURRENT;

	return SUCCESS;

err_sensor_register:
	device_remove_file(&input_dev->dev, &dev_attr_gesture_poll_delay);
err_creat_file:
	sensors_remove_symlink(&input_dev->dev.kobj, input_dev->name);
err_creat_symlink:
	input_unregister_device(input_dev);

	pr_err("[SSP]: %s - fail!\n", __func__);
	return ERROR;
}

void remove_gesture_sensor(struct ssp_data *data)
{
	sensors_unregister(data->ges_device, gesture_attrs);
	device_remove_file(&data->gesture_input_dev->dev,
		&dev_attr_gesture_poll_delay);
	sensors_remove_symlink(&data->gesture_input_dev->dev.kobj,
		data->gesture_input_dev->name);
	input_unregister_device(data->gesture_input_dev);	
}
