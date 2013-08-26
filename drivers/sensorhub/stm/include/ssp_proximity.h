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

#ifndef __SSP_PROXIMITY_H__
#define __SSP_PROXIMITY_H__

#ifdef CONFIG_SSP_PROXIMITY
extern int initialize_proximity_sensor(struct ssp_data *);
extern void remove_proximity_sensor(struct ssp_data *);
extern void report_proximity_data(struct ssp_data *, struct sensor_value *);
extern void report_proximity_raw_data(struct ssp_data *,
	struct sensor_value *);
extern int proximity_open_calibration(struct ssp_data *);
extern int proximity_open_lcd_ldi(struct ssp_data *);
extern void set_proximity_barcode_enable(struct ssp_data *, bool);
extern void set_proximity_threshold(struct ssp_data *, unsigned char,
	unsigned char);
extern void get_proximity_sensordata(char *, int *, struct sensor_value *);
extern void get_proximity_rawdata(char *, int *, struct sensor_value *);

#else
static inline int initialize_proximity_sensor(struct ssp_data *data)
{
	return 0;
}

static inline void remove_proximity_sensor(struct ssp_data *data)
{
	return;
}

static inline void report_proximity_data(struct ssp_data *data,
	struct sensor_value *prox)
{
	return;
}

static inline void report_proximity_raw_data(struct ssp_data *data,
	struct sensor_value *prox)
{
	return;
}

static inline int proximity_open_calibration(struct ssp_data *data)
{
	return 0;
}

static inline int proximity_open_lcd_ldi(struct ssp_data *data)
{
	return 0;
}

static inline int set_proximity_barcode_enable(struct ssp_data *data,
	bool bEnable)
{
	return 0;
}

static inline int set_proximity_threshold(struct ssp_data *data,
	unsigned char uData1, unsigned char uData2)
{
	return 0;
}

static inline void get_proximity_sensordata(char *pchRcvDataFrame,
	int *iDataIdx, struct sensor_value *sensorsdata)
{
	return;
}

static inline void get_proximity_rawdata(char *pchRcvDataFrame,
	int *iDataIdx, struct sensor_value *sensorsdata)
{
	return;
}
#endif
#endif /* __SSP_PROXIMITY_H__ */
