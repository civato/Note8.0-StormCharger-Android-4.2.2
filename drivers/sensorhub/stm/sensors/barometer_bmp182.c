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

#define	VENDOR			"BOSCH"
#define	CHIP_ID			"BMP180"
#define MODULE_NAME		"barometer_sensor"

#define CALIBRATION_FILE_PATH	"/efs/FactoryApp/baro_delta"

/* 24 bit 2'compl */
#define	PR_ABS_MAX		8388607
#define	PR_ABS_MIN		-8388608

void report_barometer_data(struct ssp_data *data, struct sensor_value *baro)
{
	data->buf[BAROMETER_SENSOR].barometer[0] =
			baro->barometer[0] - data->iPressureCal;
	data->buf[BAROMETER_SENSOR].barometer[1] = baro->barometer[1];

	/* barometer */
	input_report_rel(data->barometer_input_dev, REL_HWHEEL,
		data->buf[BAROMETER_SENSOR].barometer[0]);
	/* temperature */
	input_report_rel(data->barometer_input_dev, REL_WHEEL,
		data->buf[BAROMETER_SENSOR].barometer[1]);
	input_sync(data->barometer_input_dev);
}

void get_barometer_sensordata(char *pchRcvDataFrame, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	int iTemp = 0;

	iTemp = (int)pchRcvDataFrame[(*iDataIdx)++];
	iTemp <<= 16;
	sensorsdata->barometer[0] = iTemp;

	iTemp = (int)pchRcvDataFrame[(*iDataIdx)++];
	iTemp <<= 8;
	sensorsdata->barometer[0] += iTemp;

	iTemp = (int)pchRcvDataFrame[(*iDataIdx)++];
	sensorsdata->barometer[0] += iTemp;

	iTemp = (int)pchRcvDataFrame[(*iDataIdx)++];
	iTemp <<= 8;
	iTemp += (int)pchRcvDataFrame[(*iDataIdx)++];
	sensorsdata->barometer[1] = (s16)iTemp;
}

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/
static ssize_t sea_level_barometer_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int iNewSeaLevelPressure;

	sscanf(buf, "%d", &iNewSeaLevelPressure);

	if (iNewSeaLevelPressure == 0) {
		pr_info("%s, our->temperature = 0\n", __func__);
		iNewSeaLevelPressure = -1;
	}

	input_report_rel(data->barometer_input_dev, REL_DIAL,
		iNewSeaLevelPressure);
	input_sync(data->barometer_input_dev);

	return size;
}

int barometer_open_calibration(struct ssp_data *data)
{
	char chBuf[10] = {0,};
	int iErr = 0;
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(cal_filp)) {
		iErr = PTR_ERR(cal_filp);
		if (iErr != -ENOENT)
			pr_err("[SSP]: %s - Can't open calibration file(%d)\n",
				__func__, iErr);
		set_fs(old_fs);
		return iErr;
	}
	iErr = cal_filp->f_op->read(cal_filp,
		chBuf, 10 * sizeof(char), &cal_filp->f_pos);
	if (iErr < 0) {
		pr_err("[SSP]: %s - Can't read the cal data from file (%d)\n",
			__func__, iErr);
		return iErr;
	}
	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	iErr = kstrtoint(chBuf, 10, &data->iPressureCal);
	if (iErr < 0) {
		pr_err("[SSP]: %s - kstrtoint failed. %d", __func__, iErr);
		return iErr;
	}

	ssp_dbg("[SSP]: open barometer calibration %d\n", data->iPressureCal);

	if (data->iPressureCal < PR_ABS_MIN || data->iPressureCal > PR_ABS_MAX)
		pr_err("[SSP]: %s - wrong offset value!!!\n", __func__);

	return iErr;
}

static ssize_t barometer_cabratioin_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int iPressureCal = 0, iErr = 0;

	iErr = kstrtoint(buf, 10, &iPressureCal);
	if (iErr < 0) {
		pr_err("[SSP]: %s - kstrtoint failed.(%d)", __func__, iErr);
		return iErr;
	}

	if (iPressureCal < PR_ABS_MIN || iPressureCal > PR_ABS_MAX)
		return -EINVAL;

	data->iPressureCal = (s32)iPressureCal;

	return size;
}

static ssize_t barometer_cabratioin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	barometer_open_calibration(data);

	return sprintf(buf, "%d\n", data->iPressureCal);
}

static ssize_t eeprom_check_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool bSuccess = false;
	char chTempBuf[2] = {0, 10};
	int iRet, iDelayCnt = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	data->uFactorydataReady = 0;
	memset(data->uFactorydata, 0, sizeof(char) * FACTORY_DATA_MAX);

	iRet = send_instruction(data, FACTORY_MODE, PRESSURE_FACTORY,
			chTempBuf, 2);

	while (!(data->uFactorydataReady & (1 << PRESSURE_FACTORY))
		&& (iDelayCnt++ < 150)
		&& (iRet == SUCCESS))
		msleep(20);

	if ((iDelayCnt >= 150) || (iRet != SUCCESS)) {
		pr_err("[SSP]: %s - Pressure Selftest Timeout!!\n",
			__func__);
		goto exit;
	}

	mdelay(5);

	bSuccess = (bool)(!!data->uFactorydata[0]);
	ssp_dbg("[SSP]: %s - %u\n", __func__, bSuccess);

exit:
	return snprintf(buf, PAGE_SIZE, "%d", bSuccess);
}

/* sysfs for vendor & name */
static ssize_t barometer_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VENDOR);
}

static ssize_t barometer_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", CHIP_ID);
}

static DEVICE_ATTR(vendor,  S_IRUGO, barometer_vendor_show, NULL);
static DEVICE_ATTR(name,  S_IRUGO, barometer_name_show, NULL);
static DEVICE_ATTR(eeprom_check, S_IRUGO, eeprom_check_show, NULL);
static DEVICE_ATTR(calibration,  S_IRUGO | S_IWUSR | S_IWGRP,
	barometer_cabratioin_show, barometer_cabratioin_store);
static DEVICE_ATTR(sea_level_barometer, S_IRUGO | S_IWUSR | S_IWGRP,
	NULL, sea_level_barometer_store);

static struct device_attribute *barometer_attrs[] = {
	&dev_attr_vendor,
	&dev_attr_name,
	&dev_attr_calibration,
	&dev_attr_sea_level_barometer,
	&dev_attr_eeprom_check,
	NULL,
};

static ssize_t show_barometer_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[BAROMETER_SENSOR]);
}

static ssize_t set_barometer_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, BAROMETER_SENSOR, dNewDelay);
	return size;
}

static struct device_attribute dev_attr_barometer_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_barometer_delay, set_barometer_delay);

int initialize_barometer_sensor(struct ssp_data *data)
{
	int iRet;
	struct input_dev *input_dev;


	/* allocate input_device */
	input_dev = input_allocate_device();
	if (input_dev == NULL)
		return ERROR;

	input_dev->name = MODULE_NAME;
	input_set_capability(input_dev, EV_REL, REL_HWHEEL);
	input_set_capability(input_dev, EV_REL, REL_DIAL);
	input_set_capability(input_dev, EV_REL, REL_WHEEL);
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

	iRet = device_create_file(&input_dev->dev, &dev_attr_barometer_poll_delay);
	if (iRet < 0)
		goto err_creat_file;

	iRet = sensors_register(data->prs_device, data,
			barometer_attrs, MODULE_NAME);
	if (iRet < 0)
		goto err_sensor_register;

	data->barometer_input_dev = input_dev;
	return SUCCESS;

err_sensor_register:
	device_remove_file(&input_dev->dev, &dev_attr_barometer_poll_delay);
err_creat_file:
	sensors_remove_symlink(&input_dev->dev.kobj, input_dev->name);
err_creat_symlink:
	input_unregister_device(input_dev);

	pr_err("[SSP]: %s - fail!\n", __func__);
	return ERROR;
}

void remove_barometer_sensor(struct ssp_data *data)
{
	sensors_unregister(data->prs_device, barometer_attrs);
	device_remove_file(&data->barometer_input_dev->dev,
		&dev_attr_barometer_poll_delay);
	sensors_remove_symlink(&data->barometer_input_dev->dev.kobj,
		data->barometer_input_dev->name);
	input_unregister_device(data->barometer_input_dev);
}
