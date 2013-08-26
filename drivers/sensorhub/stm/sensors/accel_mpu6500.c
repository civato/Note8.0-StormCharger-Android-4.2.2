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

#define VENDOR				"INVENSENSE"
#define CHIP_ID				"MPU6500"
#define MODULE_NAME			"accelerometer_sensor"

#define CALIBRATION_FILE_PATH		"/efs/calibration_data"
#define CALIBRATION_DATA_AMOUNT		20

/* 16bits */
#define MAX_ACCEL_1G			16384
#define MAX_ACCEL_2G			32767
#define MIN_ACCEL_2G			-32768
#define MAX_ACCEL_4G			65536

void convert_accelerometer_data(s16 *iValue)
{
	if (*iValue > MAX_ACCEL_2G)
		*iValue = ((MAX_ACCEL_4G - *iValue)) * (-1);
}

void report_accelerometer_data(struct ssp_data *data, struct sensor_value *acc)
{
	convert_accelerometer_data(&acc->x);
	convert_accelerometer_data(&acc->y);
	convert_accelerometer_data(&acc->z);

	data->buf[ACCELEROMETER_SENSOR].x = acc->x - data->accelcal.x;
	data->buf[ACCELEROMETER_SENSOR].y = acc->y - data->accelcal.y;
	data->buf[ACCELEROMETER_SENSOR].z = acc->z - data->accelcal.z;

	if (!(data->buf[ACCELEROMETER_SENSOR].x >> 15 == acc->x >> 15) &&\
		!(data->accelcal.x >> 15 == acc->x >> 15)) {
		pr_debug("[SSP] : accel x is overflowed!\n");
		data->buf[ACCELEROMETER_SENSOR].x =
			(data->buf[ACCELEROMETER_SENSOR].x > 0 ?
			MIN_ACCEL_2G : MAX_ACCEL_2G);
	}
	if (!(data->buf[ACCELEROMETER_SENSOR].y >> 15 == acc->y >> 15) &&\
		!(data->accelcal.y >> 15 == acc->y >> 15)) {
		pr_debug("[SSP] : accel y is overflowed!\n");
		data->buf[ACCELEROMETER_SENSOR].y =
			(data->buf[ACCELEROMETER_SENSOR].y > 0 ?
			MIN_ACCEL_2G : MAX_ACCEL_2G);
	}
	if (!(data->buf[ACCELEROMETER_SENSOR].z >> 15 == acc->z >> 15) &&\
		!(data->accelcal.z >> 15 == acc->z >> 15)) {
		pr_debug("[SSP] : accel z is overflowed!\n");
		data->buf[ACCELEROMETER_SENSOR].z =
			(data->buf[ACCELEROMETER_SENSOR].z > 0 ?
			MIN_ACCEL_2G : MAX_ACCEL_2G);
	}

	input_report_rel(data->acc_input_dev, REL_X,
		data->buf[ACCELEROMETER_SENSOR].x);
	input_report_rel(data->acc_input_dev, REL_Y,
		data->buf[ACCELEROMETER_SENSOR].y);
	input_report_rel(data->acc_input_dev, REL_Z,
		data->buf[ACCELEROMETER_SENSOR].z);
	input_sync(data->acc_input_dev);
}

static ssize_t accel_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VENDOR);
}

static ssize_t accel_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", CHIP_ID);
}

int accelerometer_open_calibration(struct ssp_data *data)
{
	int iRet = 0;
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(cal_filp)) {
		set_fs(old_fs);
		iRet = PTR_ERR(cal_filp);

		data->accelcal.x = 0;
		data->accelcal.y = 0;
		data->accelcal.z = 0;

		return iRet;
	}

	iRet = cal_filp->f_op->read(cal_filp, (char *)&data->accelcal,
		3 * sizeof(int), &cal_filp->f_pos);
	if (iRet != 3 * sizeof(int))
		iRet = -EIO;

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	ssp_dbg("[SSP]: open accel calibration %d, %d, %d\n",
		data->accelcal.x, data->accelcal.y, data->accelcal.z);

	if ((data->accelcal.x == 0) && (data->accelcal.y == 0)
		&& (data->accelcal.z == 0))
		return ERROR;

	return iRet;
}

static int enable_accel_for_cal(struct ssp_data *data)
{
	u8 uBuf[2] = {0, 10};

	if (atomic_read(&data->aSensorEnable) & (1 << ACCELEROMETER_SENSOR)) {
		if (get_msdelay(data->adDelayBuf[ACCELEROMETER_SENSOR]) != 10) {
			send_instruction(data, CHANGE_DELAY,
				ACCELEROMETER_SENSOR, uBuf, 2);
			return SUCCESS;
		}
	} else {
		send_instruction(data, ADD_SENSOR,
			ACCELEROMETER_SENSOR, uBuf, 2);
	}

	return FAIL;
}

static void disable_accel_for_cal(struct ssp_data *data, int iDelayChanged)
{
	u8 uBuf[2] = {0, 10};

	if (atomic_read(&data->aSensorEnable) & (1 << ACCELEROMETER_SENSOR)) {
		uBuf[1] = get_msdelay(data->adDelayBuf[ACCELEROMETER_SENSOR]);
		uBuf[0] = get_delay_cmd(uBuf[1]);
		if (iDelayChanged)
			send_instruction(data, CHANGE_DELAY,
				ACCELEROMETER_SENSOR, uBuf, 2);
	} else {
		send_instruction(data, REMOVE_SENSOR,
			ACCELEROMETER_SENSOR, uBuf, 2);
	}
}

static int accel_do_calibrate(struct ssp_data *data, int iEnable)
{
	int iSum[3] = { 0, };
	int iRet = 0, iCount;
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;

	if (iEnable) {
		data->accelcal.x = 0;
		data->accelcal.y = 0;
		data->accelcal.z = 0;

		iRet = enable_accel_for_cal(data);
		msleep(300);

		for (iCount = 0; iCount < CALIBRATION_DATA_AMOUNT; iCount++) {
			iSum[0] += data->buf[ACCELEROMETER_SENSOR].x;
			iSum[1] += data->buf[ACCELEROMETER_SENSOR].y;
			iSum[2] += data->buf[ACCELEROMETER_SENSOR].z;
			mdelay(10);
		}
		disable_accel_for_cal(data, iRet);

		data->accelcal.x = (iSum[0] / CALIBRATION_DATA_AMOUNT);
		data->accelcal.y = (iSum[1] / CALIBRATION_DATA_AMOUNT);
		data->accelcal.z = (iSum[2] / CALIBRATION_DATA_AMOUNT);

		if (data->accelcal.z > 0)
			data->accelcal.z -= MAX_ACCEL_1G;
		else if (data->accelcal.z < 0)
			data->accelcal.z += MAX_ACCEL_1G;
	} else {
		data->accelcal.x = 0;
		data->accelcal.y = 0;
		data->accelcal.z = 0;
	}

	ssp_dbg("[SSP]: do accel calibrate %d, %d, %d\n",
		data->accelcal.x, data->accelcal.y, data->accelcal.z);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_filp)) {
		pr_err("[SSP]: %s - Can't open calibration file\n", __func__);
		set_fs(old_fs);
		iRet = PTR_ERR(cal_filp);
		return iRet;
	}

	iRet = cal_filp->f_op->write(cal_filp, (char *)&data->accelcal,
		3 * sizeof(int), &cal_filp->f_pos);
	if (iRet != 3 * sizeof(int)) {
		pr_err("[SSP]: %s - Can't write the accelcal to file\n",
			__func__);
		iRet = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return iRet;
}

static ssize_t accel_calibration_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int iRet;
	int iCount = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	iRet = accelerometer_open_calibration(data);
	if (iRet < 0)
		pr_err("[SSP]: %s - calibration open failed(%d)\n",
			__func__, iRet);

	ssp_dbg("[SSP] Cal data : %d %d %d - %d\n",
		data->accelcal.x, data->accelcal.y, data->accelcal.z, iRet);

	iCount = sprintf(buf, "%d %d %d %d\n", iRet, data->accelcal.x,
			data->accelcal.y, data->accelcal.z);
	return iCount;
}

static ssize_t accel_calibration_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int iRet;
	int64_t dEnable;
	struct ssp_data *data = dev_get_drvdata(dev);

	iRet = kstrtoll(buf, 10, &dEnable);
	if (iRet < 0)
		return iRet;

	iRet = accel_do_calibrate(data, (int)dEnable);
	if (iRet < 0)
		pr_err("[SSP]: %s - accel_do_calibrate() failed\n", __func__);

	return size;
}

static ssize_t raw_data_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
		data->buf[ACCELEROMETER_SENSOR].x,
		data->buf[ACCELEROMETER_SENSOR].y,
		data->buf[ACCELEROMETER_SENSOR].z);
}

static ssize_t accel_reactive_alert_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	char chTempBuf[2] = {0, 10};
	int iRet, iDelayCnt = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "1"))
		ssp_dbg("[SSP]: %s - on\n", __func__);
	else if (sysfs_streq(buf, "0"))
		ssp_dbg("[SSP]: %s - off\n", __func__);
	else if (sysfs_streq(buf, "2")) {
		ssp_dbg("[SSP]: %s - factory\n", __func__);

		data->uFactorydataReady = 0;
		memset(data->uFactorydata, 0, sizeof(char) * FACTORY_DATA_MAX);

		data->bAccelAlert = false;
		iRet = send_instruction(data, FACTORY_MODE,
				ACCELEROMETER_FACTORY, chTempBuf, 2);

		while (!(data->uFactorydataReady & (1 << ACCELEROMETER_FACTORY))
			&& (iDelayCnt++ < 150)
			&& (iRet == SUCCESS))
			msleep(20);

		if ((iDelayCnt >= 150) || (iRet != SUCCESS)) {
			pr_err("[SSP]: %s - accel Selftest Timeout!!\n",
				__func__);
			goto exit;
		}

		mdelay(5);

		data->bAccelAlert = data->uFactorydata[0];
		ssp_dbg("[SSP]: %s factory test success!\n", __func__);
	} else {
		pr_err("[SSP]: %s - invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
exit:
	return size;
}

static ssize_t accel_reactive_alert_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool bSuccess = false;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (data->bAccelAlert == true)
		bSuccess = true;
	else
		bSuccess = false;

	data->bAccelAlert = false;
	return sprintf(buf, "%u\n", bSuccess);
}

static DEVICE_ATTR(name, S_IRUGO, accel_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, accel_vendor_show, NULL);
static DEVICE_ATTR(calibration, S_IRUGO | S_IWUSR | S_IWGRP,
	accel_calibration_show, accel_calibration_store);
static DEVICE_ATTR(raw_data, S_IRUGO, raw_data_read, NULL);
static DEVICE_ATTR(reactive_alert, S_IRUGO | S_IWUSR | S_IWGRP,
	accel_reactive_alert_show, accel_reactive_alert_store);

static struct device_attribute *acc_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_calibration,
	&dev_attr_raw_data,
	&dev_attr_reactive_alert,
	NULL,
};

static ssize_t show_acc_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[ACCELEROMETER_SENSOR]);
}

static ssize_t set_acc_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, ACCELEROMETER_SENSOR, dNewDelay);
	return size;
}

static struct device_attribute dev_attr_accel_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_acc_delay, set_acc_delay);

int initialize_accelerometer_sensor(struct ssp_data *data)
{
	int iRet;
	struct input_dev *input_dev;

	/* allocate input_device */
	input_dev = input_allocate_device();
	if (input_dev == NULL)
		return ERROR;

	input_dev->name = MODULE_NAME;
	input_set_capability(input_dev, EV_REL, REL_X);
	input_set_capability(input_dev, EV_REL, REL_Y);
	input_set_capability(input_dev, EV_REL, REL_Z);
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

	iRet = device_create_file(&input_dev->dev, &dev_attr_accel_poll_delay);
	if (iRet < 0)
		goto err_creat_file;

	iRet = sensors_register(data->acc_device, data, acc_attrs, MODULE_NAME);
	if (iRet < 0)
		goto err_sensor_register;

	data->acc_input_dev = input_dev;
	return SUCCESS;

err_sensor_register:
	device_remove_file(&input_dev->dev, &dev_attr_accel_poll_delay);
err_creat_file:
	sensors_remove_symlink(&input_dev->dev.kobj, input_dev->name);
err_creat_symlink:
	input_unregister_device(input_dev);

	pr_err("[SSP]: %s - fail!\n", __func__);
	return ERROR;
}

void remove_accelerometer_sensor(struct ssp_data *data)
{
	sensors_unregister(data->acc_device, acc_attrs);
	device_remove_file(&data->acc_input_dev->dev,
		&dev_attr_accel_poll_delay);
	sensors_remove_symlink(&data->acc_input_dev->dev.kobj,
		data->acc_input_dev->name);
	input_unregister_device(data->acc_input_dev);
}
