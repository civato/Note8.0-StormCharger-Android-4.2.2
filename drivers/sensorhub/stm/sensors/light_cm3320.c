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

#define	VENDOR		"CAPELLA"
#define	CHIP_ID		"CM3323"
#define MODULE_NAME	"light_sensor"

void report_light_data(struct ssp_data *data, struct sensor_value *light)
{
	data->buf[LIGHT_SENSOR].r = light->r;
	data->buf[LIGHT_SENSOR].g = light->g;
	data->buf[LIGHT_SENSOR].b = light->b;
	data->buf[LIGHT_SENSOR].w = light->w;

	input_report_rel(data->light_input_dev, REL_HWHEEL,
		data->buf[LIGHT_SENSOR].r + 1);
	input_report_rel(data->light_input_dev, REL_DIAL,
		data->buf[LIGHT_SENSOR].g + 1);
	input_report_rel(data->light_input_dev, REL_WHEEL,
		data->buf[LIGHT_SENSOR].b + 1);
	input_report_rel(data->light_input_dev, REL_MISC,
		data->buf[LIGHT_SENSOR].w + 1);
	input_sync(data->light_input_dev);
}

void get_light_sensordata(char *pchRcvDataFrame, int *iDataIdx,
	struct sensor_value *sensorsdata)
{
	s16 iTemp = 0;

	iTemp = (s16)pchRcvDataFrame[(*iDataIdx)++];
	iTemp <<= 8;
	iTemp += pchRcvDataFrame[(*iDataIdx)++];
	sensorsdata->r = iTemp;

	iTemp = (s16)pchRcvDataFrame[(*iDataIdx)++];
	iTemp <<= 8;
	iTemp += pchRcvDataFrame[(*iDataIdx)++];
	sensorsdata->g = iTemp;

	iTemp = (s16)pchRcvDataFrame[(*iDataIdx)++];
	iTemp <<= 8;
	iTemp += pchRcvDataFrame[(*iDataIdx)++];
	sensorsdata->b = iTemp;

	iTemp = (s16)pchRcvDataFrame[(*iDataIdx)++];
	iTemp <<= 8;
	iTemp += pchRcvDataFrame[(*iDataIdx)++];
	sensorsdata->w = iTemp;
}

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/
static ssize_t light_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VENDOR);
}

static ssize_t light_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", CHIP_ID);
}

static ssize_t light_lux_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u,%u,%u,%u\n",
		data->buf[LIGHT_SENSOR].r, data->buf[LIGHT_SENSOR].g,
		data->buf[LIGHT_SENSOR].b, data->buf[LIGHT_SENSOR].w);
}

static ssize_t light_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u,%u,%u,%u\n",
		data->buf[LIGHT_SENSOR].r, data->buf[LIGHT_SENSOR].g,
		data->buf[LIGHT_SENSOR].b, data->buf[LIGHT_SENSOR].w);
}

static DEVICE_ATTR(vendor, S_IRUGO, light_vendor_show, NULL);
static DEVICE_ATTR(name, S_IRUGO, light_name_show, NULL);
static DEVICE_ATTR(lux, S_IRUGO, light_lux_show, NULL);
static DEVICE_ATTR(raw_data, S_IRUGO, light_data_show, NULL);

static struct device_attribute *light_attrs[] = {
	&dev_attr_vendor,
	&dev_attr_name,
	&dev_attr_lux,
	&dev_attr_raw_data,
	NULL,
};

static ssize_t show_light_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[LIGHT_SENSOR]);
}

static ssize_t set_light_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, LIGHT_SENSOR, dNewDelay);
	return size;
}

static struct device_attribute dev_attr_light_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_light_delay, set_light_delay);

int initialize_light_sensor(struct ssp_data *data)
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
	input_set_capability(input_dev, EV_REL, REL_MISC);
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

	iRet = device_create_file(&input_dev->dev, &dev_attr_light_poll_delay);
	if (iRet < 0)
		goto err_creat_file;

	iRet = sensors_register(data->light_device, data, light_attrs, MODULE_NAME);
	if (iRet < 0)
		goto err_sensor_register;

	data->light_input_dev = input_dev;
	return SUCCESS;

err_sensor_register:
	device_remove_file(&input_dev->dev, &dev_attr_light_poll_delay);
err_creat_file:
	sensors_remove_symlink(&input_dev->dev.kobj, input_dev->name);
err_creat_symlink:
	input_unregister_device(input_dev);

	pr_err("[SSP]: %s - fail!\n", __func__);
	return ERROR;
}

void remove_light_sensor(struct ssp_data *data)
{
	sensors_unregister(data->light_device, light_attrs);
	device_remove_file(&data->light_input_dev->dev,
		&dev_attr_light_poll_delay);
	sensors_remove_symlink(&data->light_input_dev->dev.kobj,
		data->light_input_dev->name);
	input_unregister_device(data->light_input_dev);
}
