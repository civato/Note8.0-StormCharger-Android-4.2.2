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
#include <linux/platform_device.h>
#include <plat/adc.h>

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

#define VENDOR		"SENSIRION"
#define CHIP_ID		"SHTC1"
#define MODULE_NAME	"temp_humidity_sensor"

#define CP_THM_ADC_SAMPLING_CNT	7

enum {
	SHTC1_CMD_NONE,
	SHTC1_CMD_RESET,
};

void report_temphumidity_data(struct ssp_data *data, struct sensor_value *th)
{
	data->buf[TEMPERATURE_HUMIDITY_SENSOR].data[0] =
		th->data[0];
	data->buf[TEMPERATURE_HUMIDITY_SENSOR].data[1] =
		th->data[1];
	data->buf[TEMPERATURE_HUMIDITY_SENSOR].data[2] =
		th->data[2];

	/* Temperature */
	input_report_rel(data->temp_humi_input_dev, REL_HWHEEL,
		data->buf[TEMPERATURE_HUMIDITY_SENSOR].data[0]);
	/* Humidity */
	input_report_rel(data->temp_humi_input_dev, REL_DIAL,
		data->buf[TEMPERATURE_HUMIDITY_SENSOR].data[1]);

	/* Compensation engine cmd */
	if (data->comp_engine_cmd != SHTC1_CMD_NONE) {
		input_report_rel(data->temp_humi_input_dev, REL_WHEEL,
			data->comp_engine_cmd);
		data->comp_engine_cmd = SHTC1_CMD_NONE;
	}

	input_sync(data->temp_humi_input_dev);
	if (data->buf[TEMPERATURE_HUMIDITY_SENSOR].data[2])
		wake_lock_timeout(&data->ssp_wake_lock, 2 * HZ);
}

static void get_temphumidity_sensordata(char *pchRcvDataFrame, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	s16 iTemp = 0;

	/* Temperature */
	iTemp = (s16)pchRcvDataFrame[(*iDataIdx)++];
	iTemp <<= 8;
	iTemp += pchRcvDataFrame[(*iDataIdx)++];
	sensorsdata->data[0] = iTemp;
	/* Humidity */
	iTemp = (s16)pchRcvDataFrame[(*iDataIdx)++];
	iTemp <<= 8;
	iTemp += pchRcvDataFrame[(*iDataIdx)++];
	sensorsdata->data[1] = iTemp;
}

static int cp_thm_get_adc_data(struct ssp_data *data)
{
	int adc_data;
	int adc_max = 0;
	int adc_min = 0;
	int adc_total = 0;
	int i;
	int err_value;

	for (i = 0; i < CP_THM_ADC_SAMPLING_CNT; i++) {
		mutex_lock(&data->cp_temp_adc_lock);
		if (data->adc_client)
			adc_data = s3c_adc_read(data->adc_client,
					data->cp_thm_adc_channel);
		else
			adc_data = 0;
		mutex_unlock(&data->cp_temp_adc_lock);

		if (adc_data < 0) {
			pr_err("[SSP] : %s err(%d) returned, skip read\n",
				__func__, adc_data);
			err_value = adc_data;
			goto err;
		}

		if (i != 0) {
			if (adc_data > adc_max)
				adc_max = adc_data;
			else if (adc_data < adc_min)
				adc_min = adc_data;
		} else {
			adc_max = adc_data;
			adc_min = adc_data;
		}
		adc_total += adc_data;
	}

	return (adc_total - adc_max - adc_min) / (CP_THM_ADC_SAMPLING_CNT - 2);
err:
	return err_value;
}

static int convert_adc_to_temp(struct ssp_data *data, unsigned int adc)
{
	u8 low = 0, mid = 0;
	u8 high;

	if (!data->cp_thm_adc_table || !data->cp_thm_adc_arr_size) {
		/* using fake temp */
		return 300;
	}

	high = data->cp_thm_adc_arr_size - 1;

	while (low <= high) {
		mid = (low + high) / 2;
		if (data->cp_thm_adc_table[mid].adc > adc)
			high = mid - 1;
		else if (data->cp_thm_adc_table[mid].adc < adc)
			low = mid + 1;
		else
			break;
	}
	return data->cp_thm_adc_table[mid].temperature;
}

static ssize_t temphumidity_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VENDOR);
}

static ssize_t temphumidity_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", CHIP_ID);
}

static ssize_t engine_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	pr_err("[SSP] %s - engine_ver = %s_%s\n",
		__func__, CONFIG_SENSORS_SSP_SHTC1_VER, data->comp_engine_ver);

	return sprintf(buf, "%s_%s\n",
		CONFIG_SENSORS_SSP_SHTC1_VER, data->comp_engine_ver);
}

ssize_t engine_version_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	kfree(data->comp_engine_ver);
	data->comp_engine_ver =
		    kzalloc(((strlen(buf)+1) * sizeof(char)), GFP_KERNEL);
	strncpy(data->comp_engine_ver, buf, strlen(buf)+1);
	pr_err("[SSP] %s - engine_ver = %s, %s\n",
		__func__, data->comp_engine_ver, buf);

	return size;
}

static ssize_t pam_adc_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int cp_thm_adc = 0;

	if (data->bSspShutdown == false)
		cp_thm_adc = cp_thm_get_adc_data(data);
	else
		pr_info("[SSP] : %s, device is shutting down", __func__);

	return sprintf(buf, "%d\n", cp_thm_adc);
}

static ssize_t pam_temp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int adc, temp;

	adc = cp_thm_get_adc_data(data);
	if (adc < 0) {
		pr_err("[SSP] : %s, reading adc failed.(%d)\n", __func__, adc);
		temp = adc;
	} else
		temp = convert_adc_to_temp(data, adc);

	return sprintf(buf, "%d\n", temp);
}

static ssize_t temphumidity_crc_check(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char chTempBuf[2] = {0, 10};
	int iDelayCnt = 0, iRet;
	struct ssp_data *data = dev_get_drvdata(dev);

	data->uFactorydata[0] = 0xff;
	data->uFactorydataReady = 0;
	iRet = send_instruction(data, FACTORY_MODE,
		TEMPHUMIDITY_CRC_FACTORY, chTempBuf, 2);

	while (!(data->uFactorydataReady &
		(1 << TEMPHUMIDITY_CRC_FACTORY))
		&& (iDelayCnt++ < 50)
		&& (iRet == SUCCESS))
		msleep(20);

	if ((iDelayCnt >= 50) || (iRet != SUCCESS)) {
		pr_err("[SSP]: %s - Temphumidity check crc Timeout!! %d\n",
			__func__, iRet);
			goto exit;
	}

	mdelay(5);

	pr_info("[SSP] : %s -Check_CRC : %d\n", __func__,
			data->uFactorydata[0]);

exit:
	if (data->uFactorydata[0] == 1)
		return sprintf(buf, "%s\n", "OK");
	else if (data->uFactorydata[0] == 2)
		return sprintf(buf, "%s\n","NG_NC");
	else
		return sprintf(buf, "%s\n","NG");
}

static ssize_t temphumidity_compengine_reset(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	data->comp_engine_cmd = SHTC1_CMD_RESET;

	return sprintf(buf, "%d\n", 1);
}

#if 0
#define DONE_CAL	3

ssize_t temphumidity_send_accuracy(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	u8 accuracy;

	if (kstrtou8(buf, 10, &accuracy) < 0) {
		pr_err("[SSP] %s - read buf is fail(%s)\n", __func__, buf);
		return size;
	}

	if (accuracy == DONE_CAL)
		ssp_send_cmd(data, MSG2SSP_AP_TEMPHUMIDITY_CAL_DONE);
	pr_info("[SSP] %s - accuracy = %d\n", __func__, accuracy);

	return size;
}

static DEVICE_ATTR(send_accuracy,  S_IWUSR | S_IWGRP,
	NULL, temphumidity_send_accuracy);*/
#endif

static DEVICE_ATTR(name, S_IRUGO, temphumidity_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, temphumidity_vendor_show, NULL);
static DEVICE_ATTR(engine_ver, S_IRUGO | S_IWUSR | S_IWGRP,
	engine_version_show, engine_version_store);
static DEVICE_ATTR(cp_thm, S_IRUGO,
	pam_adc_show, NULL);
static DEVICE_ATTR(cp_temperature, S_IRUGO,
	pam_temp_show, NULL);
static DEVICE_ATTR(crc_check, S_IRUGO,
	temphumidity_crc_check, NULL);
static DEVICE_ATTR(reset, S_IRUGO,
	temphumidity_compengine_reset, NULL);

static struct device_attribute *temphumidity_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_engine_ver,
	&dev_attr_cp_thm,
	&dev_attr_cp_temperature,
	&dev_attr_crc_check,
	&dev_attr_reset,
	NULL,
};

static ssize_t show_temp_humi_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n",
		data->adDelayBuf[TEMPERATURE_HUMIDITY_SENSOR]);
}

static ssize_t set_temp_humi_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, TEMPERATURE_HUMIDITY_SENSOR, dNewDelay);
	return size;
}

static struct device_attribute dev_attr_temp_humi_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_temp_humi_delay, set_temp_humi_delay);

int initialize_temphumidity_sensor(struct ssp_data *data)
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
			&dev_attr_temp_humi_poll_delay);
	if (iRet < 0)
		goto err_creat_file;

	iRet = sensors_register(data->temphumidity_device, data,
		temphumidity_attrs, MODULE_NAME);
	if (iRet < 0)
		goto err_sensor_register;

	data->temp_humi_input_dev = input_dev;

	/* Get pdata for cp thermister */
	if (pdata->cp_thm_adc_table) {
		data->cp_thm_adc_channel = pdata->cp_thm_adc_channel;
		data->cp_thm_adc_arr_size= pdata->cp_thm_adc_arr_size;
		data->cp_thm_adc_table = pdata->cp_thm_adc_table;
	}
	mutex_init(&data->cp_temp_adc_lock);
	
	/* alloc platform device for adc client */
	data->pdev_pam_temp = platform_device_alloc("pam-temp-adc", -1);
	if (!data->pdev_pam_temp)
		pr_err("%s: could not allocation pam-temp-adc\n", __func__);

	data->adc_client = s3c_adc_register(data->pdev_pam_temp, NULL, NULL, 0);
	if (IS_ERR(data->adc_client))
		pr_err("%s, fail to register pam-temp-adc(%ld)\n",
			__func__, IS_ERR(data->adc_client));

	return SUCCESS;

err_sensor_register:
	device_remove_file(&input_dev->dev, &dev_attr_temp_humi_poll_delay);
err_creat_file:
	sensors_remove_symlink(&input_dev->dev.kobj, input_dev->name);
err_creat_symlink:
	input_unregister_device(input_dev);

	pr_err("[SSP]: %s - fail!\n", __func__);
	return ERROR;
}

void remove_temphumidity_sensor(struct ssp_data *data)
{
	sensors_unregister(data->temphumidity_device, temphumidity_attrs);
	device_remove_file(&data->temphumidity_device->dev,
			&dev_attr_temp_humi_poll_delay);
	sensors_remove_symlink(&data->temp_humi_input_dev->dev.kobj,
		data->temp_humi_input_dev->name);
	input_unregister_device(data->temp_humi_input_dev);

	mutex_destroy(&data->cp_temp_adc_lock);

	if (data->adc_client)
		s3c_adc_release(data->adc_client);
	if (data->pdev_pam_temp)
		platform_device_put(data->pdev_pam_temp);

	kfree(data->comp_engine_ver);
}
