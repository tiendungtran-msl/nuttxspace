############################################################################
# apps/uav/lib/Makefile
#
# Shared Libraries Build
#
# CẤU TRÚC:
# - mathlib: Các hàm toán học (quaternion, matrix, etc.)
# - calibration: Sensor calibration library
# - platform: HRT, timebase, platform abstraction
# - utils: Ring buffers, DMA buffers, etc.
# - dsp: Digital signal processing (filters)
# - health: Health monitoring system
# - sensor_processing: IMU fusion, preprocessing
# - drivers_framework: Device ID, base classes
#
############################################################################

include $(APPDIR)/Make.defs

# Subdirectories - thứ tự quan trọng cho dependencies
SUBDIRS  = platform
SUBDIRS += utils
SUBDIRS += mathlib
SUBDIRS += dsp
SUBDIRS += calibration
SUBDIRS += drivers_framework
SUBDIRS += health
SUBDIRS += sensor_processing

ifeq ($(CONFIG_UAV),y)
CONFIGURED_APPS += $(SUBDIRS)
endif

include $(APPDIR)/Directory.mk

