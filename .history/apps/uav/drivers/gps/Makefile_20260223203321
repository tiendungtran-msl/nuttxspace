############################################################################
# apps/uav/drivers/gps/Makefile
#
# GPS Driver for u-blox M10N
#
############################################################################

include $(APPDIR)/Make.defs

# Override C++ extension (NuttX default is .cxx)
CXXEXT := .cpp

# GPS application name
PROGNAME  = gps_app
PRIORITY  = SCHED_PRIORITY_DEFAULT
STACKSIZE = 2048
MODULE    = $(CONFIG_UAV_DRIVERS_GPS)

# Main source file (includes gps_ubx.cpp via #include)
MAINSRC   = gps_main.cpp

# C++ settings
CXXFLAGS += -I$(APPDIR)
CXXFLAGS += -I$(APPDIR)/uav/uorb
CXXFLAGS += -I$(APPDIR)/uav/drivers/gps

include $(APPDIR)/Application.mk

.PHONY: FORCE
FORCE:

$(PREFIX).built: FORCE
