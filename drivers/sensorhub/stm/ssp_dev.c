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
#include "include/ssp.h"

/* ssp mcu device ID */
#define DEVICE_ID			0x55

/************************************************************************/
/* interrupt happened due to transition/change of SSP MCU		*/
/************************************************************************/
static irqreturn_t sensordata_irq_thread_fn(int iIrq, void *dev_id)
{
	struct ssp_data *data = dev_id;

	select_irq_msg(data);
	data->uIrqCnt++;

	return IRQ_HANDLED;
}

/*************************************************************************/
/* initialize sensor hub						 */
/*************************************************************************/
static void initialize_function_pointer(struct ssp_data *data)
{
	data->get_sensor_data[ACCELEROMETER_SENSOR] = get_3axis_sensordata;
	data->get_sensor_data[GYROSCOPE_SENSOR] = get_3axis_sensordata;
	data->get_sensor_data[MAGNETIC_SENSOR] = get_3axis_sensordata;
	data->get_sensor_data[MAGNETIC_RAW] = get_3axis_sensordata;
	data->get_sensor_data[BAROMETER_SENSOR] = get_barometer_sensordata;
	data->get_sensor_data[GESTURE_SENSOR] = get_gesture_sensordata;
	data->get_sensor_data[PROXIMITY_SENSOR] = get_proximity_sensordata;
	data->get_sensor_data[PROXIMITY_RAW] = get_proximity_rawdata;
	data->get_sensor_data[LIGHT_SENSOR] = get_light_sensordata;
	data->get_sensor_data[TEMPERATURE_HUMIDITY_SENSOR] =
		get_temphumidity_sensordata;

	data->report_sensor_data[ACCELEROMETER_SENSOR] =
		report_accelerometer_data;
	data->report_sensor_data[GYROSCOPE_SENSOR] =
		report_gyroscope_data;
	data->report_sensor_data[MAGNETIC_SENSOR] = report_magnetic_data;
	data->report_sensor_data[MAGNETIC_RAW] = report_magnetic_raw_data;
	data->report_sensor_data[BAROMETER_SENSOR] = report_barometer_data;
	data->report_sensor_data[GESTURE_SENSOR] = report_gesture_data;
	data->report_sensor_data[PROXIMITY_SENSOR] = report_proximity_data;
	data->report_sensor_data[PROXIMITY_RAW] = report_proximity_raw_data;
	data->report_sensor_data[LIGHT_SENSOR] = report_light_data;
	data->report_sensor_data[TEMPERATURE_HUMIDITY_SENSOR] =
		report_temphumidity_data;
}

static void initialize_variable(struct ssp_data *data)
{
	int iSensorIndex;

	for (iSensorIndex = 0; iSensorIndex < SENSOR_MAX; iSensorIndex++) {
		data->adDelayBuf[iSensorIndex] = DEFUALT_POLLING_DELAY;
		data->aiCheckStatus[iSensorIndex] = INITIALIZATION_STATE;
	}

	memset(data->uFactorydata, 0, sizeof(char) * FACTORY_DATA_MAX);

	atomic_set(&data->aSensorEnable, 0);
	data->iLibraryLength = 0;
	data->uSensorState = 0;
	data->uFactorydataReady = 0;
	data->uFactoryProxAvg[0] = 0;

	data->uResetCnt = 0;
	data->uInstFailCnt = 0;
	data->uTimeOutCnt = 0;
	data->uSsdFailCnt = 0;
	data->uBusyCnt = 0;
	data->uIrqCnt = 0;
	data->uIrqFailCnt = 0;
	data->uMissSensorCnt = 0;

	data->bProbeIsDone = false;
	data->fw_dl_state = FW_DL_STATE_NONE;
	data->bSspShutdown = true;
	data->bProximityRawEnabled = false;
	data->bGeomagneticRawEnabled = false;
	data->bMcuIRQTestSuccessed = false;
	data->bBarcodeEnabled = false;
	data->bAccelAlert = false;

	data->accelcal.x = 0;
	data->accelcal.y = 0;
	data->accelcal.z = 0;

	data->gyrocal.x = 0;
	data->gyrocal.y = 0;
	data->gyrocal.z = 0;

	data->magoffset.mx = 0;
	data->magoffset.my = 0;
	data->magoffset.mz = 0;

	data->iPressureCal = 0;
	data->uProxCanc = 0;
	data->uProxHiThresh = 0;
	data->uProxLoThresh = 0;

	data->mcu_device = NULL;
	data->acc_device = NULL;
	data->gyro_device = NULL;
	data->mag_device = NULL;
	data->prs_device = NULL;
	data->prox_device = NULL;
	data->light_device = NULL;
	data->ges_device = NULL;

	initialize_function_pointer(data);
}

int startup_mcu(struct ssp_data *data)
{
	int iRet = 0;

	iRet = get_chipid(data);
	pr_info("[SSP] MCU device ID = %d, reading ID = %d\n", DEVICE_ID, iRet);
	if (iRet != DEVICE_ID) {
		if (iRet < 0) {
			pr_err("[SSP]: %s - MCU is not working : 0x%x\n",
				__func__, iRet);
		} else {
			pr_err("[SSP]: %s - MCU identification failed\n",
				__func__);
			iRet = -ENODEV;
		}
		goto out;
	}

	iRet = set_sensor_position(data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - set_sensor_position failed\n", __func__);
		goto out;
	}

	data->uSensorState = get_sensor_scanning_info(data);
	if (data->uSensorState == 0) {
		pr_err("[SSP]: %s - get_sensor_scanning_info failed\n",
			__func__);
		iRet = ERROR;
		goto out;
	}

	iRet = SUCCESS;
out:
	return iRet;
}

static int initialize_irq(struct ssp_data *data)
{
	int iRet, iIrq;

	iRet = gpio_request(data->spi->irq, "mpu_ap_int");
	if (iRet < 0) {
		pr_err("[SSP]: %s - gpio %d request failed (%d)\n",
		       __func__, data->spi->irq, iRet);
		return iRet;
	}

	iRet = gpio_direction_input(data->spi->irq);
	if (iRet < 0) {
		pr_err("[SSP]: %s - failed to set gpio %d as input (%d)\n",
		       __func__, data->spi->irq, iRet);
		goto err_irq_direction_input;
	}

	iIrq = gpio_to_irq(data->spi->irq);

	pr_info("[SSP]: requesting IRQ %d, %d\n", iIrq, data->spi->irq);
	iRet = request_threaded_irq(iIrq, NULL, sensordata_irq_thread_fn,
				    IRQF_TRIGGER_FALLING, "SSP_Int", data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - request_irq(%d) failed for gpio %d (%d)\n",
		       __func__, iIrq, iIrq, iRet);
		goto err_request_irq;
	}

	/* start with interrupts disabled */
	data->iIrq = iIrq;
	disable_irq(data->iIrq);
	return 0;

err_request_irq:
err_irq_direction_input:
	gpio_free(data->spi->irq);
	return iRet;
}

static void work_function_firmware_update(struct work_struct *work)
{
	struct ssp_data *data = container_of((struct delayed_work *)work,
						struct ssp_data, work_firmware);
	int iRet = 0;

	pr_info("[SSP] : %s\n", __func__);

	iRet = forced_to_download_binary(data, KERNEL_BINARY);
	if (iRet < 0) {
		ssp_dbg("[SSP]: %s - forced_to_download_binary failed!\n",
			__func__);
		return;
	}

	data->uCurFirmRev = get_firmware_rev(data);
	pr_info("[SSP] MCU Firm Rev : New = %8u\n", data->uCurFirmRev);
}

static int initialize_sensors(struct ssp_data *data)
{
	int iRet;

	iRet = initialize_ssp_mcu(data);
	if (iRet < 0)
		goto err_init_mcu;
	iRet = initialize_accelerometer_sensor(data);
	if (iRet < 0)
		goto err_init_acc;
	iRet = initialize_gyroscope_sensor(data);
	if (iRet < 0)
		goto err_init_gyro;
	iRet = initialize_magnetic_sensor(data);
	if (iRet < 0)
		goto err_init_mag;
	iRet = initialize_barometer_sensor(data);
	if (iRet < 0)
		goto err_init_baro;
	iRet = initialize_proximity_sensor(data);
	if (iRet < 0)
		goto err_init_prox;
	iRet = initialize_light_sensor(data);
	if (iRet < 0)
		goto err_init_light;
	iRet = initialize_gesture_sensor(data);
	if (iRet < 0)
		goto err_init_ges;
	iRet = initialize_temphumidity_sensor(data);
	if (iRet < 0)
		goto err_init_th;

	return SUCCESS;

err_init_th:
	remove_gesture_sensor(data);
err_init_ges:
	remove_light_sensor(data);
err_init_light:
	remove_proximity_sensor(data);
err_init_prox:
	remove_barometer_sensor(data);
err_init_baro:
	remove_magnetic_sensor(data);
err_init_mag:
	remove_gyroscope_sensor(data);
err_init_gyro:
	remove_accelerometer_sensor(data);
err_init_acc:
	remove_ssp_mcu(data);
err_init_mcu:
	return ERROR;
}

static void remove_sensors(struct ssp_data *data)
{
	remove_temphumidity_sensor(data);
	remove_gesture_sensor(data);
	remove_light_sensor(data);
	remove_proximity_sensor(data);
	remove_barometer_sensor(data);
	remove_magnetic_sensor(data);
	remove_gyroscope_sensor(data);
	remove_accelerometer_sensor(data);
	remove_ssp_mcu(data);
}

static int initialize_platformdata(struct ssp_data *data,
	struct ssp_platform_data *pdata)
{
	data->wakeup_mcu = pdata->wakeup_mcu;
	data->check_mcu_ready = pdata->check_mcu_ready;
	data->check_mcu_busy = pdata->check_mcu_busy;
	data->set_mcu_reset = pdata->set_mcu_reset;

	if ((data->wakeup_mcu == NULL)
		|| (data->check_mcu_ready == NULL)
		|| (data->check_mcu_busy == NULL)
		|| (data->set_mcu_reset == NULL)) {

		pr_err("[SSP]: %s - function callback is null\n", __func__);
		return ERROR;
	}

	/* set spi pin */
	if (pdata->set_spi_pin)
		pdata->set_spi_pin();

	/* AP system_rev */
	if (pdata->check_ap_rev)
		data->ap_rev = pdata->check_ap_rev();
	else
		data->ap_rev = 0;

	/* Get sensor positions */
	if (pdata->get_positions) {
		pdata->get_positions(&data->accel_position,
			&data->mag_position);
	} else {
		data->accel_position = 0;
		data->mag_position = 0;
	}

	return SUCCESS;
}

static int ssp_probe(struct spi_device *spi)
{
	int iRet = 0;
	struct ssp_data *data;

	pr_info("\n#####################################################\n");
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		pr_err("[SSP]: %s - failed to allocate memory for data\n",
			__func__);
		iRet = -ENOMEM;
		goto exit;
	}

	iRet = initialize_platformdata(data, spi->dev.platform_data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - failed initialize pdata\n", __func__);
		iRet = -ENOMEM;
		goto err_init_pdata;
	}

	mutex_init(&data->comm_mutex);

	spi->mode = SPI_MODE_1;
	if (spi_setup(spi)) {
		pr_err("failed to setup spi for ssp_spi\n");
		goto err_setup;
	}
	data->spi = spi;
	spi_set_drvdata(spi, data);
	initialize_variable(data);

	INIT_DELAYED_WORK(&data->work_firmware, work_function_firmware_update);

	/* check boot loader binary */
	data->fw_dl_state = check_fwbl(data);
	if (data->fw_dl_state == FW_DL_STATE_NONE) {
		iRet = startup_mcu(data);
		if (iRet == ERROR) {
			data->uResetCnt++;
			toggle_mcu_reset(data);
			msleep(SSP_SW_RESET_TIME);
			iRet = startup_mcu(data);
		} 

		if (iRet != SUCCESS) {
			pr_err("[SSP]: %s - startup_mcu failed\n", __func__);
			goto err_read_reg;
		}
	}

	wake_lock_init(&data->ssp_wake_lock,
		WAKE_LOCK_SUSPEND, "ssp_wake_lock");

	iRet = initialize_debug_timer(data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - could not create workqueue\n", __func__);
		goto err_create_workqueue;
	}

	iRet = initialize_irq(data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - could not create irq\n", __func__);
		goto err_setup_irq;
	}

	iRet = initialize_sensors(data);
	if (iRet < 0) {
		pr_err("[SSP]: %s - could not initialize sensor\n", __func__);
		goto err_init_sensor;
	}

	/* init sensorhub device */
	iRet = ssp_sensorhub_initialize(data);
	if (iRet < 0) {
		pr_err("%s: ssp_sensorhub_initialize err(%d)", __func__, iRet);
		ssp_sensorhub_remove(data);
	}

	ssp_enable(data, true);
	pr_info("[SSP]: %s - probe success!\n", __func__);

	enable_debug_timer(data);

	if (data->fw_dl_state == FW_DL_STATE_NEED_TO_SCHEDULE) {
		pr_info("[SSP]: Firmware update is scheduled\n");
		schedule_delayed_work(&data->work_firmware,
				msecs_to_jiffies(1000));
		data->fw_dl_state = FW_DL_STATE_SCHEDULED;
	} else if (data->fw_dl_state == FW_DL_STATE_FAIL) {
		data->bSspShutdown = true;
	}

	data->bProbeIsDone = true;
	iRet = 0;
	goto exit;

err_init_sensor:
	free_irq(data->iIrq, data);
	gpio_free(data->spi->irq);
err_setup_irq:
	destroy_workqueue(data->debug_wq);
err_create_workqueue:
err_input_register_device:
	wake_lock_destroy(&data->ssp_wake_lock);
err_read_reg:
err_reset_null:
err_setup:
	mutex_destroy(&data->comm_mutex);
err_init_pdata:
	kfree(data);
	pr_err("[SSP]: %s - probe failed!\n", __func__);
exit:
	pr_info("#####################################################\n\n");
	return iRet;
}

static void ssp_shutdown(struct spi_device *spi)
{
	struct ssp_data *data = spi_get_drvdata(spi);

	func_dbg();
	if (data->bProbeIsDone == false)
		goto exit;

	if (data->fw_dl_state >= FW_DL_STATE_SCHEDULED &&
		data->fw_dl_state < FW_DL_STATE_DONE) {
		pr_err("%s, cancel_delayed_work_sync state = %d\n",
			__func__, data->fw_dl_state);
		cancel_delayed_work_sync(&data->work_firmware);
	}

	ssp_enable(data, false);
	disable_debug_timer(data);

	free_irq(data->iIrq, data);
	gpio_free(data->spi->irq);

	remove_sensors(data);
	ssp_sensorhub_remove(data);

	del_timer_sync(&data->debug_timer);
	cancel_work_sync(&data->work_debug);
	destroy_workqueue(data->debug_wq);
	wake_lock_destroy(&data->ssp_wake_lock);
	toggle_mcu_reset(data);
exit:
	kfree(data);
}

static int ssp_suspend(struct device *dev)
{
	func_dbg();

	return 0;
}

static int ssp_resume(struct device *dev)
{
	func_dbg();

	return 0;
}

static const struct dev_pm_ops ssp_pm_ops = {
	.suspend = ssp_suspend,
	.resume = ssp_resume
};

static const struct spi_device_id ssp_id[] = {
	{"ssp-spi", 0},
	{}
};

MODULE_DEVICE_TABLE(spi, ssp_id);

static struct spi_driver ssp_driver = {
	.probe = ssp_probe,
	.shutdown = ssp_shutdown,
	.id_table = ssp_id,
	.driver = {
		.pm = &ssp_pm_ops,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		.name = "ssp-spi"
	},
};

static int __init ssp_stm_spi_init(void)
{
	int ret;

	func_dbg();
	ret = spi_register_driver(&ssp_driver);
	if (ret)
		pr_err("[SSP] failed to register ssp-spi %x\n", ret);

	return ret;
}

static void __exit ssp_stm_spi_exit(void)
{
	spi_unregister_driver(&ssp_driver);
}

module_init(ssp_stm_spi_init);
module_exit(ssp_stm_spi_exit);

MODULE_DESCRIPTION("ssp spi driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
