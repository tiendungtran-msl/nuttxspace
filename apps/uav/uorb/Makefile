############################################################################
# apps/uav/uorb/Makefile
#
# uORB Message Bus - Build configuration
#
# MỤC ĐÍCH:
# - Build uORB như một library (không phải app)
# - Được link bởi tất cả UAV apps
#
############################################################################

include $(APPDIR)/Make.defs

# Library mode (không tạo executable)
CXXEXT = .cpp

# Source files
CXXSRCS  = uorb.cpp
CXXSRCS += topics.cpp

# Include paths
CXXFLAGS += -I$(APPDIR)

# C++ flags
CXXFLAGS += -std=c++17 -fno-rtti -fno-exceptions

include $(APPDIR)/Application.mk
