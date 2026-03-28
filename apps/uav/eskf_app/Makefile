############################################################################
# apps/uav/eskf_app/Makefile
#
# ESKF Estimator Application Build
#
# MỤC ĐÍCH:
# - Build eskf_app thành NSH command "eskf"
# - Sử dụng thư viện ESKF từ lib/eskf/ (header-only)
# - Link với uORB shared library
#
############################################################################

include $(APPDIR)/Make.defs

# Tên ứng dụng (command trên NSH: "eskf start/stop/status")
PROGNAME  = eskf
PRIORITY  = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_UAV_ESKF_STACKSIZE)
MODULE    = $(CONFIG_UAV_ESKF)

# Main entry point
MAINSRC = eskf_main.cpp

# Include paths
CXXFLAGS += -I$(APPDIR)

# C++ configuration
CXXEXT    = .cpp
CXXFLAGS += -std=c++17 -fno-rtti -fno-exceptions -fno-threadsafe-statics

include $(APPDIR)/Application.mk
