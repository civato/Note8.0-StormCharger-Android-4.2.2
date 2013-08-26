#
# Makefile for the sensor drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_SENSORS_SSP_STM)		+= ssp_dev.o ssp_core.o ssp_firmware.o ssp_debug.o ssp_spi.o sensors_core.o

obj-$(CONFIG_SENSORS_SSP_MPU6500)	+= sensors/accel_mpu6500.o sensors/gyro_mpu6500.o

obj-$(CONFIG_SENSORS_SSP_CM3320)	+= sensors/light_cm3320.o
obj-$(CONFIG_SENSORS_SSP_MAX88920)	+= sensors/proximity_max88920.o sensors/gesture_max88920.o

obj-$(CONFIG_SENSORS_SSP_YAS532)	+= sensors/magnetic_yas532.o

obj-$(CONFIG_SENSORS_SSP_BMP182)	+= sensors/barometer_bmp182.o
obj-$(CONFIG_SENSORS_SSP_SHTC1)		+= sensors/temphumidity_shtc1.o

obj-$(CONFIG_SENSORS_SSP_ATUC128L5HAR)	+= sensors/mcu_atuc128l5har.o
obj-$(CONFIG_SENSORS_SSP_STM32F401)	+= sensors/mcu_stm32f401.o

obj-$(CONFIG_SENSORS_SSP_SENSORHUB)	+= ssp_sensorhub.o
