############################################################################
# apps/uav/drivers/mag/Makefile
#
# Magnetometer Drivers
############################################################################

include $(APPDIR)/Make.defs

# Subdirectories
SUBDIRS = bmm150

ifeq ($(CONFIG_UAV),y)
CONFIGURED_APPS += $(SUBDIRS)
endif

include $(APPDIR)/Directory.mk
