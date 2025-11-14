include $(APPDIR)/Make.defs

# IMU System Application

PROGNAME  = $(CONFIG_EXAMPLES_IMU_SYSTEM_PROGNAME)
PRIORITY  = $(CONFIG_EXAMPLES_IMU_SYSTEM_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_IMU_SYSTEM_STACKSIZE)
MODULE    = $(CONFIG_EXAMPLES_IMU_SYSTEM)

# Source files

MAINSRC = imu_system_main.c

CSRCS  = utils/data_queue.c
CSRCS += drivers/spi/spi_manager.c
CSRCS += drivers/icm42688p/icm42688p_driver.c
CSRCS += drivers/bmm150/bmm150_driver.c
CSRCS += drivers/led/led_driver.c
CSRCS += algorithms/sensor_fusion.c
CSRCS += algorithms/orientation.c
CSRCS += tasks/sensor_task.c
CSRCS += tasks/fusion_task.c
CSRCS += tasks/led_task.c

include $(APPDIR)/Application.mk
