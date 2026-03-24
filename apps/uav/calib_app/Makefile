############################################################################
# apps/uav/calib_app/Makefile
#
# Sensor Calibration Application Build
#
# MỤC ĐÍCH:
# - Build calib_app thành NSH command "calib"
# - Chạy interactive calibration cho gyro, accel, mag
#
############################################################################

include $(APPDIR)/Make.defs

# Tên ứng dụng (command trên NSH)
PROGNAME = calib
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_UAV_CALIB_STACKSIZE)
MODULE = $(CONFIG_UAV_CALIB)

# Main entry point
MAINSRC = calib_main.cpp

# Thêm CXXFLAGS
CXXFLAGS += -I$(APPDIR)

# C++ configuration
CXXEXT = .cpp
CXXFLAGS += -std=c++17 -fno-rtti -fno-exceptions -fno-threadsafe-statics

include $(APPDIR)/Application.mk
