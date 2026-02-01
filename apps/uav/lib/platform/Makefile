############################################################################
# apps/uav/lib/platform/Makefile
#
# Platform Abstraction Layer
#
# MỤC ĐÍCH:
# - HRT: High-resolution timer API
# - Timebase: Master tick generation và domain scheduling
# - SPI config: Board-specific SPI configuration
#
############################################################################

include $(APPDIR)/Make.defs

# C sources
CSRCS = hrt.c timebase.c

# C++ sources
CXXSRCS =

# Include paths
CFLAGS += -I$(APPDIR)
CXXFLAGS += -I$(APPDIR)

include $(APPDIR)/Application.mk
