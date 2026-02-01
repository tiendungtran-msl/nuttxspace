############################################################################
# apps/uav/drivers/imu/Makefile
#
# IMU Drivers
############################################################################

include $(APPDIR)/Make.defs

# Subdirectories
SUBDIRS = icm42688p

ifeq ($(CONFIG_UAV),y)
CONFIGURED_APPS += $(SUBDIRS)
endif

include $(APPDIR)/Directory.mk
