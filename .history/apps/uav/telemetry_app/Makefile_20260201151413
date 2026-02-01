############################################################################
# apps/uav/telemetry_app/Makefile
#
# UAV Telemetry Application
#
# MỤC ĐÍCH:
# - Gửi telemetry packet qua UART
# - Subscribe uORB và đóng gói binary
#
############################################################################

include $(APPDIR)/Make.defs

# App configuration
PROGNAME  = telemetry
PRIORITY  = $(CONFIG_UAV_TELEMETRY_PRIORITY)
STACKSIZE = $(CONFIG_UAV_TELEMETRY_STACKSIZE)
MODULE    = $(CONFIG_UAV_TELEMETRY_APP)

# Main source
MAINSRC = telemetry_main.cpp

# C sources
CSRCS = telemetry_packet.c

# Include paths
CFLAGS += -I$(APPDIR)
CFLAGS += -I$(APPDIR)/uav
CFLAGS += -I$(APPDIR)/uav/uorb
CFLAGS += -I$(APPDIR)/uav/lib

CXXFLAGS += -I$(APPDIR)
CXXFLAGS += -I$(APPDIR)/uav
CXXFLAGS += -I$(APPDIR)/uav/uorb
CXXFLAGS += -I$(APPDIR)/uav/lib

# C++ configuration
CXXEXT = .cpp
CXXFLAGS += -std=c++17 -fno-rtti -fno-exceptions -fno-threadsafe-statics

include $(APPDIR)/Application.mk
