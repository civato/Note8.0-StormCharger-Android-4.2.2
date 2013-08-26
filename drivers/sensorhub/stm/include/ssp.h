/*
 *  Copyright (C) 2011, Samsung Electronics Co. Ltd. All Rights Reserved.
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

#ifndef __SSP_PRJ_H__
#define __SSP_PRJ_H__

#include "ssp_data.h"
#include "ssp_accelerometer.h"
#include "ssp_gyroscope.h"
#include "ssp_magnetic.h"
#include "ssp_barometer.h"
#include "ssp_light.h"
#include "ssp_proximity.h"
#include "ssp_gesture.h"
#include "ssp_temphumidity.h"
#include "ssp_sensorhub.h"

/* AP -> SSP Instruction */
#define MSG2SSP_INST_BYPASS_SENSOR_ADD		0xA1
#define MSG2SSP_INST_BYPASS_SENSOR_REMOVE	0xA2
#define MSG2SSP_INST_REMOVE_ALL			0xA3
#define MSG2SSP_INST_CHANGE_DELAY		0xA4
#define MSG2SSP_INST_SENSOR_SELFTEST		0xA8
#define MSG2SSP_INST_LIBRARY_ADD		0xB1
#define MSG2SSP_INST_LIBRARY_REMOVE		0xB2
#define MSG2SSP_INST_LIB_NOTI			0xB4

#define MSG2SSP_AP_STT				0xC8
#define MSG2SSP_AP_STATUS_WAKEUP		0xD1
#define MSG2SSP_AP_STATUS_SLEEP			0xD2
#define MSG2SSP_AP_STATUS_RESUME		0xD3
#define MSG2SSP_AP_STATUS_SUSPEND		0xD4
#define MSG2SSP_AP_STATUS_RESET			0xD5
#define MSG2SSP_AP_STATUS_POW_CONNECTED		0xD6
#define MSG2SSP_AP_STATUS_POW_DISCONNECTED	0xD7
#define MSG2SSP_AP_STATUS_CALL_IDLE		0xD8
#define MSG2SSP_AP_STATUS_CALL_ACTIVE		0xD9
#define MSG2SSP_AP_TEMPHUMIDITY_CAL_DONE	0xDA

#define MSG2SSP_AP_WHOAMI			0x0F
#define MSG2SSP_AP_FIRMWARE_REV			0xF0
#define MSG2SSP_AP_SENSOR_FORMATION		0xF1
#define MSG2SSP_AP_SENSOR_PROXTHRESHOLD		0xF2
#define MSG2SSP_AP_SENSOR_BARCODE_EMUL		0xF3
#define MSG2SSP_AP_SENSOR_SCANNING		0xF4
#define MSG2SSP_AP_SET_MAGNETIC_HWOFFSET	0xF5
#define MSG2SSP_AP_GET_MAGNETIC_HWOFFSET	0xF6
#define MSG2SSP_AP_SENSOR_GESTURE_CURRENT	0xF7

#define MSG2SSP_AP_FUSEROM			0X01

/* AP -> SSP Data Protocol Frame Field */
#define MSG2SSP_SSP_SLEEP	0xC1
#define MSG2SSP_STS		0xC2 /* Start to Send */
#define MSG2SSP_RTS		0xC4 /* Ready to Send */
#define MSG2SSP_STT		0xC8
#define MSG2SSP_SRM		0xCA /* Start to Read MSG */
#define MSG2SSP_SSM		0xCB /* Start to Send MSG */
#define MSG2SSP_SSD		0xCE /* Start to Send Data Type & Length */
#define MSG2SSP_NO_DATA		0xCF /* There is no data to get from MCU */

/* SSP -> AP ACK about write CMD */
#define MSG_ACK			0x80 /* ACK from SSP to AP */
#define MSG_NAK			0x70 /* NAK from SSP to AP */

int initialize_ssp_mcu(struct ssp_data *);
void remove_ssp_mcu(struct ssp_data *);
void ssp_enable(struct ssp_data *, bool);
int ssp_remove_sensor(struct ssp_data *data, unsigned int, unsigned int);
void change_sensor_delay(struct ssp_data *, int, int64_t);
void get_3axis_sensordata(char *pchRcvDataFrame, int *, struct sensor_value *);

int check_fwbl(struct ssp_data *);
void reset_mcu(struct ssp_data *);
void toggle_mcu_reset(struct ssp_data *);
int forced_to_download_binary(struct ssp_data *, int);

int startup_mcu(struct ssp_data *);
int set_sensor_position(struct ssp_data *);
int get_chipid(struct ssp_data *);
unsigned int get_sensor_scanning_info(struct ssp_data *);
unsigned int get_firmware_rev(struct ssp_data *);
unsigned int get_module_rev(struct ssp_data *data);
void sync_sensor_state(struct ssp_data *);

int parse_dataframe(struct ssp_data *, char *, int);
int ssp_read_data(struct ssp_data *, char *, u16, char *, u16, int);
int send_instruction(struct ssp_data *, u8, u8, u8 *, u8);
int ssp_send_cmd(struct ssp_data *, char);
int select_irq_msg(struct ssp_data *);

unsigned int get_delay_cmd(u8);
unsigned int get_msdelay(int64_t);

void enable_debug_timer(struct ssp_data *);
void disable_debug_timer(struct ssp_data *);
int initialize_debug_timer(struct ssp_data *);
int print_mcu_debug(char *, int *, int);

int sensors_create_symlink(struct kobject *, const char *);
void sensors_remove_symlink(struct kobject *target, const char *);
void destroy_sensor_class(void);
int sensors_register(struct device *, void *,
	struct device_attribute*[], char *);
void sensors_unregister(struct device *,
	struct device_attribute*[]);
#endif /* __SSP_PRJ_H__ */
