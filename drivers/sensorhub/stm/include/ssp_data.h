/*
 *  Copyright (C) 2013, Samsung Electronics Co. Ltd. All Rights Reserved.
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

#ifndef __SSP_DATA_H__
#define __SSP_DATA_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/miscdevice.h>
#include <linux/ssp_platformdata.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/timer.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#define SUCCESS		1
#define FAIL		0
#define ERROR		-1

/*
 1. ACCEL : 6 Bytes
 2. GYRO : 6 Bytes
 3. MAGNETIC : 6 Bytes
 4. PRESSURE : 5 Bytes
 5. GESTURE : 4 Bytes
 6. PROXIMITY : 2 Bytes
 7. TEMPHUMI : 5 Bytes
 8. LIGHT : 8 Bytes
 SUM : 42 Bytes
 Total : 42 + 8(index) = 50 Bytes
*/
#define FACTORY_DATA_MAX	63

#define SSP_DBG		1
#if SSP_DBG
#define SSP_FUNC_DBG 1
#define SSP_DATA_DBG 0
#define ssp_dbg(dev, format, ...) do { \
	printk(KERN_INFO dev, format, ##__VA_ARGS__); \
	} while (0)
#else
#define ssp_dbg(dev, format, ...)
#endif

#if SSP_FUNC_DBG
#define func_dbg() do { \
	printk(KERN_INFO "[SSP]: %s\n", __func__); \
	} while (0)
#else
#define func_dbg()
#endif

#if SSP_DATA_DBG
#define data_dbg(dev, format, ...) do { \
	printk(KERN_INFO dev, format, ##__VA_ARGS__); \
	} while (0)
#else
#define data_dbg(dev, format, ...)
#endif

#define SSP_SW_RESET_TIME	3000
#define DEFUALT_POLLING_DELAY	(200 * NSEC_PER_MSEC)
#define DEFAULT_RETRIES		3

/* SSP Binary Type */
enum {
	KERNEL_BINARY = 0,
	KERNEL_CRASHED_BINARY,
	UMS_BINARY,
};

/* Sensor Sampling Time Define */
enum {
	SENSOR_NS_DELAY_FASTEST = 10000000,	/* 10msec */
	SENSOR_NS_DELAY_GAME = 20000000,	/* 20msec */
	SENSOR_NS_DELAY_UI = 66700000,		/* 66.7msec */
	SENSOR_NS_DELAY_NORMAL = 200000000,	/* 200msec */
};

enum {
	SENSOR_MS_DELAY_FASTEST = 10,	/* 10msec */
	SENSOR_MS_DELAY_GAME = 20,	/* 20msec */
	SENSOR_MS_DELAY_UI = 66,	/* 66.7msec */
	SENSOR_MS_DELAY_NORMAL = 200,	/* 200msec */
};

enum {
	SENSOR_CMD_DELAY_FASTEST = 0,	/* 10msec */
	SENSOR_CMD_DELAY_GAME,		/* 20msec */
	SENSOR_CMD_DELAY_UI,		/* 66.7msec */
	SENSOR_CMD_DELAY_NORMAL,	/* 200msec */
};

/*
 * SENSOR_DELAY_SET_STATE
 * Check delay set to avoid sending ADD instruction twice
 */
enum {
	INITIALIZATION_STATE = 0,
	NO_SENSOR_STATE,
	ADD_SENSOR_STATE,
	RUNNING_SENSOR_STATE,
};

/* Firmware download STATE */
enum {
	FW_DL_STATE_FAIL = -1,
	FW_DL_STATE_NONE = 0,
	FW_DL_STATE_NEED_TO_SCHEDULE,
	FW_DL_STATE_SCHEDULED,
	FW_DL_STATE_DOWNLOADING,
	FW_DL_STATE_SYNC,
	FW_DL_STATE_DONE,
};

/* SSP_INSTRUCTION_CMD */
enum {
	REMOVE_SENSOR = 0,
	ADD_SENSOR,
	CHANGE_DELAY,
	GO_SLEEP,
	FACTORY_MODE,
	REMOVE_LIBRARY,
	ADD_LIBRARY,
};

/* SENSOR_TYPE */
enum {
	ACCELEROMETER_SENSOR = 0,
	GYROSCOPE_SENSOR,
	MAGNETIC_SENSOR,
	BAROMETER_SENSOR,
	GESTURE_SENSOR,
	PROXIMITY_SENSOR,
	TEMPERATURE_HUMIDITY_SENSOR,
	LIGHT_SENSOR,
	PROXIMITY_RAW,
	MAGNETIC_RAW,
	SENSOR_MAX,
};

/* SENSOR_FACTORY_MODE_TYPE */
enum {
	ACCELEROMETER_FACTORY = 0,
	GYROSCOPE_FACTORY,
	MAGNETIC_FACTORY,
	PRESSURE_FACTORY,
	MCU_FACTORY,
	GYROSCOPE_TEMP_FACTORY,
	GYROSCOPE_DPS_FACTORY,
	MCU_SLEEP_FACTORY,
	GESTURE_FACTORY,
	TEMPHUMIDITY_CRC_FACTORY,
	SENSOR_FACTORY_MAX,
};

struct sensor_value {
	union {
		struct {
			s8 mx;
			s8 my;
			s8 mz;
		};
		struct {
			s16 x;
			s16 y;
			s16 z;
		};
		struct {
			u16 r;
			u16 g;
			u16 b;
			u16 w;
		};
		u8 prox[4];
		s16 data[9];
		s32 barometer[3];
	};
};

struct ssp_data {
	struct input_dev *acc_input_dev;
	struct input_dev *gyro_input_dev;
	struct input_dev *mag_input_dev;
	struct input_dev *gesture_input_dev;
	struct input_dev *barometer_input_dev;
	struct input_dev *light_input_dev;
	struct input_dev *prox_input_dev;
	struct input_dev *temp_humi_input_dev;

	struct device *mcu_device;
	struct device *acc_device;
	struct device *gyro_device;
	struct device *mag_device;
	struct device *prs_device;
	struct device *prox_device;
	struct device *light_device;
	struct device *ges_device;
	struct device *temphumidity_device;
	struct spi_device *spi;

	struct wake_lock ssp_wake_lock;
	struct miscdevice akmd_device;
	struct timer_list debug_timer;
	struct workqueue_struct *debug_wq;
	struct delayed_work work_firmware;
	struct work_struct work_debug;
	struct sensor_value accelcal;
	struct sensor_value gyrocal;
	struct sensor_value magoffset;
	struct sensor_value buf[SENSOR_MAX];

	bool bSspShutdown;
	bool bMcuIRQTestSuccessed;
	bool bAccelAlert;
	bool bProximityRawEnabled;
	bool bGeomagneticRawEnabled;
	bool bBarcodeEnabled;
	bool bBinaryChashed;
	bool bProbeIsDone;

	unsigned char uProxCanc;
	unsigned char uProxHiThresh;
	unsigned char uProxLoThresh;
	unsigned char uProxHiThresh_default;
	unsigned char uProxLoThresh_default;
	unsigned int uIr_Current;
	unsigned char uFuseRomData[3];
	unsigned char uFactorydata[FACTORY_DATA_MAX];
	char *pchLibraryBuf;
	char chLcdLdi[2];
	int iIrq;
	int iLibraryLength;
	int aiCheckStatus[SENSOR_MAX];

	unsigned int uIrqFailCnt;
	unsigned int uSsdFailCnt;
	unsigned int uResetCnt;
	unsigned int uInstFailCnt;
	unsigned int uTimeOutCnt;
	unsigned int uIrqCnt;
	unsigned int uBusyCnt;
	unsigned int uMissSensorCnt;

	unsigned int uGyroDps;
	unsigned int uSensorState;
	unsigned int uCurFirmRev;
	unsigned int uFactoryProxAvg[4];
	unsigned int uFactorydataReady;
	s32 iPressureCal;
	unsigned int uTempCount;

	atomic_t aSensorEnable;
	int64_t adDelayBuf[SENSOR_MAX];

	int (*wakeup_mcu)(void);
	int (*check_mcu_ready)(void);
	int (*check_mcu_busy)(void);
	int (*set_mcu_reset)(int);
	void (*get_sensor_data[SENSOR_MAX])(char *, int *,
		struct sensor_value *);
	void (*report_sensor_data[SENSOR_MAX])(struct ssp_data *,
		struct sensor_value *);

	struct ssp_sensorhub_data *hub_data;

	int ap_rev;
	int ssp_changes;
	int accel_position;
	int mag_position;
	int fw_dl_state;
#ifdef CONFIG_SENSORS_SSP_SHTC1
	char *comp_engine_ver;
	struct platform_device *pdev_pam_temp;
	struct s3c_adc_client *adc_client;
	u8 cp_thm_adc_channel;
	u8 cp_thm_adc_arr_size;
	struct cp_thm_adc_table *cp_thm_adc_table;
	struct mutex cp_temp_adc_lock;
#endif
	u8 comp_engine_cmd;
	struct mutex comm_mutex;
};

#endif /* __SSP_DATA_H__ */
