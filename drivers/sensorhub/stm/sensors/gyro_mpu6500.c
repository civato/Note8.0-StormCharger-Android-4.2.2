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
#include <linux/kernel.h>
#include "../include/ssp.h"

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

#define VENDOR				"INVENSENSE"
#define CHIP_ID				"MPU6500"
#define MODULE_NAME			"gyro_sensor"

#define CALIBRATION_FILE_PATH		"/efs/gyro_cal_data"
#define VERBOSE_OUT			1
#define CALIBRATION_DATA_AMOUNT		20
#define DEF_GYRO_FULLSCALE		2000

#define DEF_GYRO_SENS			16 /* (32768 / DEF_GYRO_FULLSCALE) */
#define DEF_BIAS_LSB_THRESH_SELF	(20 * DEF_GYRO_SENS)
#define DEF_BIAS_LSB_THRESH_SELF_6500	(30 * DEF_GYRO_SENS)
#define DEF_RMS_LSB_TH_SELF		(5 * DEF_GYRO_SENS)
#define DEF_RMS_THRESH			((DEF_RMS_LSB_TH_SELF) * (DEF_RMS_LSB_TH_SELF))
#define DEF_SCALE_FOR_FLOAT		1000
#define DEF_RMS_SCALE_FOR_RMS		10000
#define DEF_SQRT_SCALE_FOR_RMS		100

#define MAX_GYRO			32767
#define MIN_GYRO			-32768

/* Gyroscope DPS */
#define GYROSCOPE_DPS250		250
#define GYROSCOPE_DPS500		500
#define GYROSCOPE_DPS2000		2000

void report_gyroscope_data(struct ssp_data *data, struct sensor_value *gyro)
{
	long lTemp[3] = {0,};

	data->buf[GYROSCOPE_SENSOR].x = gyro->x - data->gyrocal.x;
	data->buf[GYROSCOPE_SENSOR].y = gyro->y - data->gyrocal.y;
	data->buf[GYROSCOPE_SENSOR].z = gyro->z - data->gyrocal.z;

	if (!(data->buf[GYROSCOPE_SENSOR].x >> 15 == gyro->x >> 15) &&\
		!(data->gyrocal.x >> 15 == gyro->x >> 15)) {
		pr_debug("[SSP] : gyro x is overflowed!\n");
		data->buf[GYROSCOPE_SENSOR].x =
			(data->buf[GYROSCOPE_SENSOR].x >= 0 ?
			MIN_GYRO : MAX_GYRO);
	}
	if (!(data->buf[GYROSCOPE_SENSOR].y >> 15 == gyro->y >> 15) &&\
		!(data->gyrocal.y >> 15 == gyro->y >> 15)) {
		pr_debug("[SSP] : gyro y is overflowed!\n");
		data->buf[GYROSCOPE_SENSOR].y =
			(data->buf[GYROSCOPE_SENSOR].y >= 0 ?
			MIN_GYRO : MAX_GYRO);
	}
	if (!(data->buf[GYROSCOPE_SENSOR].z >> 15 == gyro->z >> 15) &&\
		!(data->gyrocal.z >> 15 == gyro->z >> 15)) {
		pr_debug("[SSP] : gyro z is overflowed!\n");
		data->buf[GYROSCOPE_SENSOR].z =
			(data->buf[GYROSCOPE_SENSOR].z >= 0 ?
			MIN_GYRO : MAX_GYRO);
	}

	if (data->uGyroDps == GYROSCOPE_DPS500) {
		lTemp[0] = (long)data->buf[GYROSCOPE_SENSOR].x;
		lTemp[1] = (long)data->buf[GYROSCOPE_SENSOR].y;
		lTemp[2] = (long)data->buf[GYROSCOPE_SENSOR].z;
	} else if (data->uGyroDps == GYROSCOPE_DPS250)	{
		lTemp[0] = (long)data->buf[GYROSCOPE_SENSOR].x >> 1;
		lTemp[1] = (long)data->buf[GYROSCOPE_SENSOR].y >> 1;
		lTemp[2] = (long)data->buf[GYROSCOPE_SENSOR].z >> 1;
	} else if (data->uGyroDps == GYROSCOPE_DPS2000)	{
		lTemp[0] = (long)data->buf[GYROSCOPE_SENSOR].x << 2;
		lTemp[1] = (long)data->buf[GYROSCOPE_SENSOR].y << 2;
		lTemp[2] = (long)data->buf[GYROSCOPE_SENSOR].z << 2;
	} else {
		lTemp[0] = (long)data->buf[GYROSCOPE_SENSOR].x;
		lTemp[1] = (long)data->buf[GYROSCOPE_SENSOR].y;
		lTemp[2] = (long)data->buf[GYROSCOPE_SENSOR].z;
	}

	input_report_rel(data->gyro_input_dev, REL_RX, lTemp[0]);
	input_report_rel(data->gyro_input_dev, REL_RY, lTemp[1]);
	input_report_rel(data->gyro_input_dev, REL_RZ, lTemp[2]);
	input_sync(data->gyro_input_dev);
}

static ssize_t gyro_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VENDOR);
}

static ssize_t gyro_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", CHIP_ID);
}

int gyroscope_open_calibration(struct ssp_data *data)
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

		data->gyrocal.x = 0;
		data->gyrocal.y = 0;
		data->gyrocal.z = 0;

		return iRet;
	}

	iRet = cal_filp->f_op->read(cal_filp, (char *)&data->gyrocal,
		3 * sizeof(int), &cal_filp->f_pos);
	if (iRet != 3 * sizeof(int))
		iRet = -EIO;

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	ssp_dbg("[SSP]: open gyro calibration %d, %d, %d\n",
		data->gyrocal.x, data->gyrocal.y, data->gyrocal.z);
	return iRet;
}

static int save_gyro_caldata(struct ssp_data *data, s16 *iCalData)
{
	int iRet = 0;
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;

	data->gyrocal.x = iCalData[0];
	data->gyrocal.y = iCalData[1];
	data->gyrocal.z = iCalData[2];

	ssp_dbg("[SSP]: do gyro calibrate %d, %d, %d\n",
		data->gyrocal.x, data->gyrocal.y, data->gyrocal.z);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_filp)) {
		pr_err("[SSP]: %s - Can't open calibration file\n", __func__);
		set_fs(old_fs);
		iRet = PTR_ERR(cal_filp);
		return -EIO;
	}

	iRet = cal_filp->f_op->write(cal_filp, (char *)&data->gyrocal,
		3 * sizeof(int), &cal_filp->f_pos);
	if (iRet != 3 * sizeof(int)) {
		pr_err("[SSP]: %s - Can't write gyro cal to file\n", __func__);
		iRet = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return iRet;
}

static ssize_t gyro_power_off(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssp_dbg("[SSP]: %s\n", __func__);

	return sprintf(buf, "%d\n", 1);
}

static ssize_t gyro_power_on(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssp_dbg("[SSP]: %s\n", __func__);

	return sprintf(buf, "%d\n", 1);
}

short mpu6500_gyro_get_temp(struct ssp_data *data)
{
	char chTempBuf[2] = { 0, 10};
	unsigned char reg[2];
	short temperature = 0;
	int iDelayCnt = 0, iRet = 0;

	data->uFactorydataReady = 0;
	memset(data->uFactorydata, 0, sizeof(char) * FACTORY_DATA_MAX);

	iRet = send_instruction(data, FACTORY_MODE, GYROSCOPE_TEMP_FACTORY,
		chTempBuf, 2);

	while (!(data->uFactorydataReady & (1 << GYROSCOPE_TEMP_FACTORY))
		&& (iDelayCnt++ < 150)
		&& (iRet == SUCCESS))
		msleep(20);

	if ((iDelayCnt >= 150) || (iRet != SUCCESS)) {
		pr_err("[SSP]: %s - Gyro Temp Timeout!!\n", __func__);
		goto exit;
	}
	reg[0] = data->uFactorydata[1];
	reg[1] = data->uFactorydata[0];
	temperature = (short) (((reg[0]) << 8) | reg[1]);
	temperature = (((temperature + 521) / 340) + 35);
	ssp_dbg("[SSP]: %s - %d\n", __func__, temperature);
exit:
	return temperature;
}

static ssize_t gyro_get_temp(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	short temperature = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	temperature = mpu6500_gyro_get_temp(data);

	return sprintf(buf, "%d\n", temperature);
}

u32 mpu6050_selftest_sqrt(u32 sqsum)
{
	u32 sq_rt;
	u32 g0, g1, g2, g3, g4;
	u32 seed;
	u32 next;
	u32 step;

	g4 = sqsum / 100000000;
	g3 = (sqsum - g4 * 100000000) / 1000000;
	g2 = (sqsum - g4 * 100000000 - g3 * 1000000) / 10000;
	g1 = (sqsum - g4 * 100000000 - g3 * 1000000 - g2 * 10000) / 100;
	g0 = (sqsum - g4 * 100000000 - g3 * 1000000 - g2 * 10000 - g1 * 100);

	next = g4;
	step = 0;
	seed = 0;
	while (((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = seed * 10000;
	next = (next - (seed * step)) * 100 + g3;

	step = 0;
	seed = 2 * seed * 10;
	while (((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = sq_rt + step * 1000;
	next = (next - seed * step) * 100 + g2;
	seed = (seed + step) * 10;
	step = 0;
	while (((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = sq_rt + step * 100;
	next = (next - seed * step) * 100 + g1;
	seed = (seed + step) * 10;
	step = 0;

	while (((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = sq_rt + step * 10;
	next = (next - seed * step) * 100 + g0;
	seed = (seed + step) * 10;
	step = 0;

	while (((seed + 1) * (step + 1)) <= next) {
		step++;
		seed++;
	}

	sq_rt = sq_rt + step;

	return sq_rt;
}

ssize_t mpu6500_gyro_selftest(char *buf, struct ssp_data *data)
{
	char chTempBuf[2] = { 3, 200};
	u8 initialized = 0;
	s8 hw_result = 0;
	int i = 0, j = 0, total_count = 0, ret_val = 0;
	long avg[3] = {0,}, rms[3] = {0,};
	int gyro_bias[3] = {0,}, gyro_rms[3] = {0,};
	s16 shift_ratio[3] = {0,};
	s16 iCalData[3] = {0,};
	char a_name[3][2] = { "X", "Y", "Z" };
	int iDelayCnt = 0, iRet = 0;
	int dps_rms[3] = { 0, };
	u32 temp = 0;
	int bias_thresh = DEF_BIAS_LSB_THRESH_SELF_6500;

	data->uFactorydataReady = 0;
	memset(data->uFactorydata, 0, sizeof(char) * FACTORY_DATA_MAX);

	iRet = send_instruction(data, FACTORY_MODE, GYROSCOPE_FACTORY,
		chTempBuf, 2);

	while (!(data->uFactorydataReady & (1 << GYROSCOPE_FACTORY))
		&& (iDelayCnt++ < 150)
		&& (iRet == SUCCESS))
		msleep(20);

	if ((iDelayCnt >= 150) || (iRet != SUCCESS)) {
		pr_err("[SSP]: %s - Gyro Selftest Timeout!!\n", __func__);
		goto exit;
	}

	initialized = data->uFactorydata[0];
	shift_ratio[0] = (s16)((data->uFactorydata[2] << 8) +
				data->uFactorydata[1]);
	shift_ratio[1] = (s16)((data->uFactorydata[4] << 8) +
				data->uFactorydata[3]);
	shift_ratio[2] = (s16)((data->uFactorydata[6] << 8) +
				data->uFactorydata[5]);
	hw_result = (s8)data->uFactorydata[7];
	total_count = (int)((data->uFactorydata[11] << 24) +
				(data->uFactorydata[10] << 16) +
				(data->uFactorydata[9] << 8) +
				data->uFactorydata[8]);
	avg[0] = (long)((data->uFactorydata[15] << 24) +
				(data->uFactorydata[14] << 16) +
				(data->uFactorydata[13] << 8) +
				data->uFactorydata[12]);
	avg[1] = (long)((data->uFactorydata[19] << 24) +
				(data->uFactorydata[18] << 16) +
				(data->uFactorydata[17] << 8) +
				data->uFactorydata[16]);
	avg[2] = (long)((data->uFactorydata[23] << 24) +
				(data->uFactorydata[22] << 16) +
				(data->uFactorydata[21] << 8) +
				data->uFactorydata[20]);
	rms[0] = (long)((data->uFactorydata[27] << 24) +
				(data->uFactorydata[26] << 16) +
				(data->uFactorydata[25] << 8) +
				data->uFactorydata[24]);
	rms[1] = (long)((data->uFactorydata[31] << 24) +
				(data->uFactorydata[30] << 16) +
				(data->uFactorydata[29] << 8) +
				data->uFactorydata[28]);
	rms[2] = (long)((data->uFactorydata[35] << 24) +
				(data->uFactorydata[34] << 16) +
				(data->uFactorydata[33] << 8) +
				data->uFactorydata[32]);
	pr_info("[SSP] init: %d, total cnt: %d\n", initialized, total_count);
	pr_info("[SSP] hw_result: %d, %d, %d, %d\n", hw_result,
		shift_ratio[0], shift_ratio[1],	shift_ratio[2]);
	pr_info("[SSP] avg %+8ld %+8ld %+8ld (LSB)\n", avg[0], avg[1], avg[2]);
	pr_info("[SSP] rms %+8ld %+8ld %+8ld (LSB)\n", rms[0], rms[1], rms[2]);

	if (hw_result < 0) {
		pr_err("[SSP] %s - hw selftest fail(%d), sw selftest skip\n",
			__func__, hw_result);
		return sprintf(buf, "-1,0,0,0,0,0,0,%d.%d,%d.%d,%d.%d,0,0,0\n",
			shift_ratio[0] / 10, shift_ratio[0] % 10,
			shift_ratio[1] / 10, shift_ratio[1] % 10,
			shift_ratio[2] / 10, shift_ratio[2] % 10);
	}
	gyro_bias[0] = (avg[0] * DEF_SCALE_FOR_FLOAT) / DEF_GYRO_SENS;
	gyro_bias[1] = (avg[1] * DEF_SCALE_FOR_FLOAT) / DEF_GYRO_SENS;
	gyro_bias[2] = (avg[2] * DEF_SCALE_FOR_FLOAT) / DEF_GYRO_SENS;
	iCalData[0] = (s16)avg[0];
	iCalData[1] = (s16)avg[1];
	iCalData[2] = (s16)avg[2];

	if (VERBOSE_OUT) {
		pr_info("[SSP] abs bias: %+8d.%03d %+8d.%03d %+8d.%03d (dps)\n",
			(int)abs(gyro_bias[0]) / DEF_SCALE_FOR_FLOAT,
			(int)abs(gyro_bias[0]) % DEF_SCALE_FOR_FLOAT,
			(int)abs(gyro_bias[1]) / DEF_SCALE_FOR_FLOAT,
			(int)abs(gyro_bias[1]) % DEF_SCALE_FOR_FLOAT,
			(int)abs(gyro_bias[2]) / DEF_SCALE_FOR_FLOAT,
			(int)abs(gyro_bias[2]) % DEF_SCALE_FOR_FLOAT);
	}

	for (j = 0; j < 3; j++) {
		if (unlikely(abs(avg[j]) > bias_thresh)) {
			pr_err("[SSP] %s-Gyro bias (%ld) exceeded threshold "
				"(threshold = %d LSB)\n", a_name[j],
				avg[j], bias_thresh);
			ret_val |= 1 << (3 + j);
		}
	}
	/* 3rd, check RMS for dead gyros
	   If any of the RMS noise value returns zero,
	   then we might have dead gyro or FIFO/register failure,
	   the part is sleeping, or the part is not responsive */
	if (rms[0] == 0 || rms[1] == 0 || rms[2] == 0)
		ret_val |= 1 << 6;

	if (VERBOSE_OUT) {
		pr_info("[SSP] RMS ^ 2 : %+8ld %+8ld %+8ld\n",
			(long)rms[0] / total_count,
			(long)rms[1] / total_count, (long)rms[2] / total_count);
	}

	for (j = 0; j < 3; j++) {
		if (unlikely(rms[j] / total_count > DEF_RMS_THRESH)) {
			pr_err("[SSP] %s-Gyro rms (%ld) exceeded threshold "
				"(threshold = %d LSB)\n", a_name[j],
				rms[j] / total_count, DEF_RMS_THRESH);
			ret_val |= 1 << (7 + j);
		}
	}

	for (i = 0; i < 3; i++) {
		if (rms[i] > 10000) {
			temp =
			    ((u32) (rms[i] / total_count)) *
			    DEF_RMS_SCALE_FOR_RMS;
		} else {
			temp =
			    ((u32) (rms[i] * DEF_RMS_SCALE_FOR_RMS)) /
			    total_count;
		}
		if (rms[i] < 0)
			temp = 1 << 31;

		dps_rms[i] = mpu6050_selftest_sqrt(temp) / DEF_GYRO_SENS;

		gyro_rms[i] =
		    dps_rms[i] * DEF_SCALE_FOR_FLOAT / DEF_SQRT_SCALE_FOR_RMS;
	}

	pr_info("[SSP] RMS : %+8d.%03d	 %+8d.%03d  %+8d.%03d (dps)\n",
		(int)abs(gyro_rms[0]) / DEF_SCALE_FOR_FLOAT,
		(int)abs(gyro_rms[0]) % DEF_SCALE_FOR_FLOAT,
		(int)abs(gyro_rms[1]) / DEF_SCALE_FOR_FLOAT,
		(int)abs(gyro_rms[1]) % DEF_SCALE_FOR_FLOAT,
		(int)abs(gyro_rms[2]) / DEF_SCALE_FOR_FLOAT,
		(int)abs(gyro_rms[2]) % DEF_SCALE_FOR_FLOAT);

	if (likely(!ret_val)) {
		save_gyro_caldata(data, iCalData);
	} else {
		pr_err("[SSP] ret_val != 0, gyrocal is 0 at all axis\n");
		data->gyrocal.x = 0;
		data->gyrocal.y = 0;
		data->gyrocal.z = 0;
	}

exit:
	ssp_dbg("[SSP]: %s - %d,"
		"%d.%03d,%d.%03d,%d.%03d,"
		"%d.%03d,%d.%03d,%d.%03d,"
		"%d.%d,%d.%d,%d.%d,"
		"%d,%d,%d\n",
		__func__, ret_val,
		(int)abs(gyro_bias[0]/1000),
		(int)abs(gyro_bias[0])%1000,
		(int)abs(gyro_bias[1]/1000),
		(int)abs(gyro_bias[1])%1000,
		(int)abs(gyro_bias[2]/1000),
		(int)abs(gyro_bias[2])%1000,
		gyro_rms[0]/1000,
		(int)abs(gyro_rms[0])%1000,
		gyro_rms[1]/1000,
		(int)abs(gyro_rms[1])%1000,
		gyro_rms[2]/1000,
		(int)abs(gyro_rms[2])%1000,
		shift_ratio[0] / 10, shift_ratio[0] % 10,
		shift_ratio[1] / 10, shift_ratio[1] % 10,
		shift_ratio[2] / 10, shift_ratio[2] % 10,
		(int)(total_count/3),
		(int)(total_count/3),
		(int)(total_count/3));

	return sprintf(buf, "%d,"
		"%d.%03d,%d.%03d,%d.%03d,"
		"%d.%03d,%d.%03d,%d.%03d,"
		"%d.%d,%d.%d,%d.%d,"
		"%d,%d,%d\n",
		ret_val,
		(int)abs(gyro_bias[0]/1000),
		(int)abs(gyro_bias[0])%1000,
		(int)abs(gyro_bias[1]/1000),
		(int)abs(gyro_bias[1])%1000,
		(int)abs(gyro_bias[2]/1000),
		(int)abs(gyro_bias[2])%1000,
		gyro_rms[0]/1000,
		(int)abs(gyro_rms[0])%1000,
		gyro_rms[1]/1000,
		(int)abs(gyro_rms[1])%1000,
		gyro_rms[2]/1000,
		(int)abs(gyro_rms[2])%1000,
		shift_ratio[0] / 10, shift_ratio[0] % 10,
		shift_ratio[1] / 10, shift_ratio[1] % 10,
		shift_ratio[2] / 10, shift_ratio[2] % 10,
		(int)(total_count/3),
		(int)(total_count/3),
		(int)(total_count/3));
}

static ssize_t gyro_selftest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return mpu6500_gyro_selftest(buf, data);
}

static ssize_t gyro_selftest_dps_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int iNewDps = 0;
	int iDelayCnt = 0, iRet = 0;
	char chTempBuf[2] = { 0, 10 };

	struct ssp_data *data = dev_get_drvdata(dev);

	if (!(data->uSensorState & (1 << GYROSCOPE_SENSOR)))
		goto exit;

	sscanf(buf, "%d", &iNewDps);

	if (iNewDps == GYROSCOPE_DPS250)
		chTempBuf[0] = 0;
	else if (iNewDps == GYROSCOPE_DPS500)
		chTempBuf[0] = 1;
	else if (iNewDps == GYROSCOPE_DPS2000)
		chTempBuf[0] = 2;
	else {
		chTempBuf[0] = 1;
		iNewDps = GYROSCOPE_DPS500;
	}

	data->uFactorydataReady = 0;
	memset(data->uFactorydata, 0, sizeof(char) * FACTORY_DATA_MAX);

	iRet = send_instruction(data, FACTORY_MODE, GYROSCOPE_DPS_FACTORY,
		chTempBuf, 2);

	while (!(data->uFactorydataReady & (1 << GYROSCOPE_DPS_FACTORY))
		&& (iDelayCnt++ < 150)
		&& (iRet == SUCCESS))
		msleep(20);

	if ((iDelayCnt >= 150) || (iRet != SUCCESS)) {
		pr_err("[SSP]: %s - Gyro Selftest DPS Timeout!!\n", __func__);
		goto exit;
	}

	mdelay(5);

	if (data->uFactorydata[0] != SUCCESS) {
		pr_err("[SSP]: %s - Gyro Selftest DPS Error!!\n", __func__);
		goto exit;
	}

	data->uGyroDps = (unsigned int)iNewDps;
	pr_err("[SSP]: %s - %u dps stored\n", __func__, data->uGyroDps);
exit:
	return count;
}

static ssize_t gyro_selftest_dps_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->uGyroDps);
}

static DEVICE_ATTR(name, S_IRUGO, gyro_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, gyro_vendor_show, NULL);
static DEVICE_ATTR(power_off, S_IRUGO, gyro_power_off, NULL);
static DEVICE_ATTR(power_on, S_IRUGO, gyro_power_on, NULL);
static DEVICE_ATTR(temperature, S_IRUGO, gyro_get_temp, NULL);
static DEVICE_ATTR(selftest, S_IRUGO, gyro_selftest_show, NULL);
static DEVICE_ATTR(selftest_dps, S_IRUGO | S_IWUSR | S_IWGRP,
	gyro_selftest_dps_show, gyro_selftest_dps_store);

static struct device_attribute *gyro_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_selftest,
	&dev_attr_power_on,
	&dev_attr_power_off,
	&dev_attr_temperature,
	&dev_attr_selftest_dps,
	NULL,
};

static ssize_t show_gyro_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[GYROSCOPE_SENSOR]);
}

static ssize_t set_gyro_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, GYROSCOPE_SENSOR, dNewDelay);
	return size;
}

static struct device_attribute dev_attr_gyro_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_gyro_delay, set_gyro_delay);

int initialize_gyroscope_sensor(struct ssp_data *data)
{
	int iRet;
	struct input_dev *input_dev;

	/* allocate input_device */
	input_dev = input_allocate_device();
	if (input_dev == NULL)
		return ERROR;

	input_dev->name = MODULE_NAME;
	input_set_capability(input_dev, EV_REL, REL_RX);
	input_set_capability(input_dev, EV_REL, REL_RY);
	input_set_capability(input_dev, EV_REL, REL_RZ);
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

	iRet = device_create_file(&input_dev->dev, &dev_attr_gyro_poll_delay);
	if (iRet < 0)
		goto err_creat_file;

	iRet = sensors_register(data->gyro_device, data, gyro_attrs, MODULE_NAME);
	if (iRet < 0)
		goto err_sensor_register;

	data->gyro_input_dev = input_dev;
	data->uGyroDps = GYROSCOPE_DPS500;

	return SUCCESS;

err_sensor_register:
	device_remove_file(&input_dev->dev, &dev_attr_gyro_poll_delay);
err_creat_file:
	sensors_remove_symlink(&input_dev->dev.kobj, input_dev->name);
err_creat_symlink:
	input_unregister_device(input_dev);

	pr_err("[SSP]: %s - fail!\n", __func__);
	return ERROR;
}

void remove_gyroscope_sensor(struct ssp_data *data)
{
	sensors_unregister(data->gyro_device, gyro_attrs);
	device_remove_file(&data->gyro_input_dev->dev,
		&dev_attr_gyro_poll_delay);
	sensors_remove_symlink(&data->gyro_input_dev->dev.kobj,
		data->gyro_input_dev->name);
	input_unregister_device(data->gyro_input_dev);
}
