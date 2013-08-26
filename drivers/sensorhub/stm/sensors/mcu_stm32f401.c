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

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/
#define MODEL_NAME		"STM32F401CCY6B"
#define MODULE_NAME		"ssp_sensor"

static ssize_t mcu_revision_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "ST01%u,ST01%u\n", data->uCurFirmRev,
		get_module_rev(data));
}

static ssize_t mcu_model_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", MODEL_NAME);
}

static ssize_t mcu_update_kernel_bin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool bSuccess = false;
	int iRet = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_dbg("[SSP]: %s - mcu binany update!\n", __func__);

	iRet = forced_to_download_binary(data, UMS_BINARY);
	if (iRet == SUCCESS) {
		bSuccess = true;
		goto out;
	}

	iRet = forced_to_download_binary(data, KERNEL_BINARY);
	if (iRet == SUCCESS)
		bSuccess = true;
	else
		bSuccess = false;
out:
	return sprintf(buf, "%s\n", (bSuccess ? "OK" : "NG"));
}

static ssize_t mcu_update_kernel_crashed_bin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool bSuccess = false;
	int iRet = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_dbg("[SSP]: %s - mcu binany update!\n", __func__);

	iRet = forced_to_download_binary(data, UMS_BINARY);
	if (iRet == SUCCESS) {
		bSuccess = true;
		goto out;
	}

	iRet = forced_to_download_binary(data, KERNEL_CRASHED_BINARY);
	if (iRet == SUCCESS)
		bSuccess = true;
	else
		bSuccess = false;
out:
	return sprintf(buf, "%s\n", (bSuccess ? "OK" : "NG"));
}

static ssize_t mcu_update_ums_bin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool bSuccess = false;
	int iRet = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_dbg("[SSP]: %s - mcu binany update!\n", __func__);

	iRet = forced_to_download_binary(data, UMS_BINARY);
	if (iRet == SUCCESS)
		bSuccess = true;
	else
		bSuccess = false;

	return sprintf(buf, "%s\n", (bSuccess ? "OK" : "NG"));
}

static ssize_t mcu_reset_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	reset_mcu(data);

	return sprintf(buf, "OK\n");
}

static ssize_t mcu_factorytest_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	char chTempBuf[2] = {0, 10};
	int iRet = 0;

	if (sysfs_streq(buf, "1")) {
		data->uFactorydataReady = 0;
		memset(data->uFactorydata, 0, sizeof(char) * FACTORY_DATA_MAX);

		data->bMcuIRQTestSuccessed = false;
		data->uTimeOutCnt = 0;

		iRet = send_instruction(data, FACTORY_MODE,
				MCU_FACTORY, chTempBuf, 2);
		if (data->uTimeOutCnt == 0)
			data->bMcuIRQTestSuccessed = true;
	} else {
		pr_err("[SSP]: %s - invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	ssp_dbg("[SSP]: MCU Factory Test Start! - %d\n", iRet);

	return size;
}

static ssize_t mcu_factorytest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool bMcuTestSuccessed = false;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (data->bSspShutdown == true) {
		ssp_dbg("[SSP]: %s - MCU Bin is crashed\n", __func__);
		return sprintf(buf, "NG,NG,NG\n");
	}

	if (data->uFactorydataReady & (1 << MCU_FACTORY)) {
		ssp_dbg("[SSP] MCU Factory Test Data : %u, %u, %u, %u, %u\n",
			data->uFactorydata[0], data->uFactorydata[1],
			data->uFactorydata[2], data->uFactorydata[3],
			data->uFactorydata[4]);

		/* system clock, RTC, I2C Master, I2C Slave, externel pin */
		if ((data->uFactorydata[0] == SUCCESS)
			&& (data->uFactorydata[1] == SUCCESS)
			&& (data->uFactorydata[2] == SUCCESS)
			&& (data->uFactorydata[3] == SUCCESS)
			&& (data->uFactorydata[4] == SUCCESS))
			bMcuTestSuccessed = true;
	} else {
		pr_err("[SSP]: %s - The Sensorhub is not ready %u\n", __func__,
			data->uFactorydataReady);
	}

	ssp_dbg("[SSP]: MCU Factory Test Result - %s, %s, %s\n", MODEL_NAME,
		(data->bMcuIRQTestSuccessed ? "OK" : "NG"),
		(bMcuTestSuccessed ? "OK" : "NG"));

	return sprintf(buf, "%s,%s,%s\n", MODEL_NAME,
		(data->bMcuIRQTestSuccessed ? "OK" : "NG"),
		(bMcuTestSuccessed ? "OK" : "NG"));
}

static ssize_t mcu_sleep_factorytest_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	char chTempBuf[2] = {0, 10};
	int iRet = 0;

	if (sysfs_streq(buf, "1")) {
		data->uFactorydataReady = 0;
		memset(data->uFactorydata, 0, sizeof(char) * FACTORY_DATA_MAX);

		iRet = send_instruction(data, FACTORY_MODE,
				MCU_SLEEP_FACTORY, chTempBuf, 2);
	} else {
		pr_err("[SSP]: %s - invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	ssp_dbg("[SSP]: MCU Sleep Factory Test Start! - %d\n", iRet);

	return size;
}

static ssize_t mcu_sleep_factorytest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int iDataIdx, iSensorData = 0;
	struct ssp_data *data = dev_get_drvdata(dev);
	struct sensor_value fsb[SENSOR_MAX];

	if (!(data->uFactorydataReady & (1 << MCU_SLEEP_FACTORY))) {
		pr_err("[SSP]: %s - The Sensorhub is not ready\n", __func__);
		goto exit;
	}

	for (iDataIdx = 0; iDataIdx < FACTORY_DATA_MAX;) {
		iSensorData = (int)data->uFactorydata[iDataIdx++];

		if ((iSensorData < 0) ||
			(iSensorData >= (SENSOR_MAX - 1))) {
			pr_err("[SSP]: %s - Mcu data frame error %d\n",
				__func__, iSensorData);
			goto exit;
		}

		data->get_sensor_data[iSensorData]((char *)data->uFactorydata,
			&iDataIdx, &(fsb[iSensorData]));
	}

	convert_accelerometer_data(&fsb[ACCELEROMETER_SENSOR].x);
	convert_accelerometer_data(&fsb[ACCELEROMETER_SENSOR].y);
	convert_accelerometer_data(&fsb[ACCELEROMETER_SENSOR].z);

	fsb[ACCELEROMETER_SENSOR].x -= data->accelcal.x;
	fsb[ACCELEROMETER_SENSOR].y -= data->accelcal.y;
	fsb[ACCELEROMETER_SENSOR].z -= data->accelcal.z;

	fsb[GYROSCOPE_SENSOR].x -= data->gyrocal.x;
	fsb[GYROSCOPE_SENSOR].y -= data->gyrocal.y;
	fsb[GYROSCOPE_SENSOR].z -= data->gyrocal.z;

	fsb[BAROMETER_SENSOR].barometer[0] -= data->iPressureCal;

exit:
	ssp_dbg("[SSP]: %s Result\n"
		"accel %d,%d,%d\n"
		"gyro %d,%d,%d\n"
		"mag %d,%d,%d\n"
		"baro %d,%d\n"
		"ges %d,%d,%d,%d\n"
		"prox %u,%u\n"
		"temp %d,%d,%d\n"
		"light %u,%u,%u,%u\n", __func__,
		fsb[ACCELEROMETER_SENSOR].x, fsb[ACCELEROMETER_SENSOR].y,
		fsb[ACCELEROMETER_SENSOR].z, fsb[GYROSCOPE_SENSOR].x,
		fsb[GYROSCOPE_SENSOR].y, fsb[GYROSCOPE_SENSOR].z,
		fsb[MAGNETIC_SENSOR].x, fsb[MAGNETIC_SENSOR].y,
		fsb[MAGNETIC_SENSOR].z, fsb[BAROMETER_SENSOR].barometer[0],
		fsb[BAROMETER_SENSOR].barometer[1],
		fsb[GESTURE_SENSOR].data[0], fsb[GESTURE_SENSOR].data[1],
		fsb[GESTURE_SENSOR].data[2], fsb[GESTURE_SENSOR].data[3],
		fsb[PROXIMITY_SENSOR].prox[0], fsb[PROXIMITY_SENSOR].prox[1],
		fsb[TEMPERATURE_HUMIDITY_SENSOR].data[0],
		fsb[TEMPERATURE_HUMIDITY_SENSOR].data[1],
		fsb[TEMPERATURE_HUMIDITY_SENSOR].data[2],
		fsb[LIGHT_SENSOR].r, fsb[LIGHT_SENSOR].g, fsb[LIGHT_SENSOR].b,
		fsb[LIGHT_SENSOR].w);

	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,"
		"%u,%u,%u,%u,%d,%d,%d,%d,%d,%d\n",
		fsb[ACCELEROMETER_SENSOR].x, fsb[ACCELEROMETER_SENSOR].y,
		fsb[ACCELEROMETER_SENSOR].z, fsb[GYROSCOPE_SENSOR].x,
		fsb[GYROSCOPE_SENSOR].y, fsb[GYROSCOPE_SENSOR].z,
		fsb[MAGNETIC_SENSOR].x, fsb[MAGNETIC_SENSOR].y,
		fsb[MAGNETIC_SENSOR].z, fsb[BAROMETER_SENSOR].barometer[0],
		fsb[BAROMETER_SENSOR].barometer[1], fsb[PROXIMITY_SENSOR].prox[1],
		fsb[LIGHT_SENSOR].r, fsb[LIGHT_SENSOR].g, fsb[LIGHT_SENSOR].b,
		fsb[LIGHT_SENSOR].w,
		fsb[GESTURE_SENSOR].data[0], fsb[GESTURE_SENSOR].data[1],
		fsb[GESTURE_SENSOR].data[2], fsb[GESTURE_SENSOR].data[3],
		fsb[TEMPERATURE_HUMIDITY_SENSOR].data[0],
		fsb[TEMPERATURE_HUMIDITY_SENSOR].data[1]);
}


/*************************************************************************/
/* ssp Sysfs                                                             */
/*************************************************************************/
static ssize_t show_sensors_enable(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_dbg("[SSP]: %s - cur_enable = %d\n", __func__,
		 atomic_read(&data->aSensorEnable));

	return sprintf(buf, "%9u\n", atomic_read(&data->aSensorEnable));
}

static ssize_t set_sensors_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dTemp;
	unsigned int uNewEnable = 0, uChangedSensor = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dTemp) < 0)
		return -EINVAL;

	uNewEnable = (unsigned int)dTemp;
	ssp_dbg("[SSP]: %s - new_enable = %u, old_enable = %u\n", __func__,
		 uNewEnable, atomic_read(&data->aSensorEnable));

	if (uNewEnable == atomic_read(&data->aSensorEnable))
		return size;

	for (uChangedSensor = 0; uChangedSensor < SENSOR_MAX; uChangedSensor++)
		if ((atomic_read(&data->aSensorEnable) & (1 << uChangedSensor))
			!= (uNewEnable & (1 << uChangedSensor))) {

			if (!(uNewEnable & (1 << uChangedSensor)))
				ssp_remove_sensor(data, uChangedSensor,
					uNewEnable);
			break;
		}

	atomic_set(&data->aSensorEnable, uNewEnable);

	return size;
}

static DEVICE_ATTR(mcu_rev, S_IRUGO, mcu_revision_show, NULL);
static DEVICE_ATTR(mcu_name, S_IRUGO, mcu_model_name_show, NULL);
static DEVICE_ATTR(mcu_update, S_IRUGO, mcu_update_kernel_bin_show, NULL);
static DEVICE_ATTR(mcu_update2, S_IRUGO,
	mcu_update_kernel_crashed_bin_show, NULL);
static DEVICE_ATTR(mcu_update_ums, S_IRUGO, mcu_update_ums_bin_show, NULL);
static DEVICE_ATTR(mcu_reset, S_IRUGO, mcu_reset_show, NULL);

static DEVICE_ATTR(mcu_test, S_IRUGO | S_IWUSR | S_IWGRP,
	mcu_factorytest_show, mcu_factorytest_store);
static DEVICE_ATTR(mcu_sleep_test, S_IRUGO | S_IWUSR | S_IWGRP,
	mcu_sleep_factorytest_show, mcu_sleep_factorytest_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	show_sensors_enable, set_sensors_enable);

static struct device_attribute *mcu_attrs[] = {
	&dev_attr_enable,
	&dev_attr_mcu_rev,
	&dev_attr_mcu_name,
	&dev_attr_mcu_test,
	&dev_attr_mcu_reset,
	&dev_attr_mcu_update,
	&dev_attr_mcu_update2,
	&dev_attr_mcu_update_ums,
	&dev_attr_mcu_sleep_test,
	NULL,
};

int initialize_ssp_mcu(struct ssp_data *data)
{
	int iRet;

	iRet = sensors_register(data->mcu_device, data,
		mcu_attrs, MODULE_NAME);
	if (iRet < 0)
		return ERROR;

	return SUCCESS;
}

void remove_ssp_mcu(struct ssp_data *data)
{
	sensors_unregister(data->mcu_device, mcu_attrs);
	destroy_sensor_class();
}

