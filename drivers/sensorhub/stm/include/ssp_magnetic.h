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

#ifndef __SSP_MAGNETIC_H__
#define __SSP_MAGNETIC_H__

#ifdef CONFIG_SSP_MAGNETIC
extern int initialize_magnetic_sensor(struct ssp_data *);
extern void remove_magnetic_sensor(struct ssp_data *);
extern void report_magnetic_data(struct ssp_data *, struct sensor_value *);
extern void report_magnetic_raw_data(struct ssp_data *,
	struct sensor_value *);
extern int set_hw_offset(struct ssp_data *);
extern int get_hw_offset(struct ssp_data *);
extern int mag_store_hwoffset(struct ssp_data *);
extern int mag_open_hwoffset(struct ssp_data *);

#else
static inline int initialize_magnetic_sensor(struct ssp_data *data)
{
	return 0;
}

static inline void remove_magnetic_sensor(struct ssp_data *data)
{
	return;
}

static inline void report_magnetic_data(struct ssp_data *data,
	struct sensor_value *mag)
{
	return;
}

static inline void report_magnetic_raw_data(struct ssp_data *data,
	struct sensor_value *mag)
{
	return;
}

static inline int set_hw_offset(struct ssp_data *data)
{
	return 0;
}

static inline int get_hw_offset(struct ssp_data *data)
{
	return 0;
}

static inline int mag_store_hwoffset(struct ssp_data *data)
{
	return 0;
}

static inline int mag_open_hwoffset(struct ssp_data *data)
{
	return 0;
}

#endif
#endif /* __SSP_MAGNETIC_H__ */
