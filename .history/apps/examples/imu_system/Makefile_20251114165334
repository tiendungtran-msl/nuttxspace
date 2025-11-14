############################################################################
# apps/examples/imu_system/Makefile
#
# IMU System Application
#
############################################################################

include $(APPDIR)/Make.defs

############################################################################
# Configuration
############################################################################

PROGNAME  = imu_system
PRIORITY  = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE    = $(CONFIG_EXAMPLES_IMU_SYSTEM)

############################################################################
# Source Files
############################################################################

# Main application
MAINSRC = imu_system_main.c

# Drivers
CSRCS += drivers/spi/spi_manager.c
CSRCS += drivers/i2c/i2c_manager.c
CSRCS += drivers/icm42688p/icm42688p_driver.c
CSRCS += drivers/bmm150/bmm150_driver.c
CSRCS += drivers/led/led_driver.c

# Algorithms
CSRCS += algorithms/sensor_fusion.c
CSRCS += algorithms/madgwick.c
CSRCS += algorithms/orientation.c

# Tasks
CSRCS += tasks/sensor_task.c
CSRCS += tasks/fusion_task.c
CSRCS += tasks/led_task.c

# Utils
CSRCS += utils/data_queue.c
CSRCS += utils/math_utils.c

############################################################################
# Compiler Flags
############################################################################

CFLAGS += -O2
CFLAGS += -Wall
CFLAGS += -Wextra

############################################################################
# Build
############################################################################

include $(APPDIR)/Application.mk