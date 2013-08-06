#
#   Wireless Sensor Networks Laboratory
#   nano-RK
#   
#   Technische Universität München
#   Fachgebiet für Verteilte Multimodale Informationsverarbeitung
#   https://www.vmi.ei.tum.de
#	 
#   copyright (c) 2011-12, Distributed Multimodle Information Processing Group, TUM
#   copyright (c) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
#	
#
    
# 
# Platform name  cc2420DK, firefly, micaZ, firefly2_1, firefly2_2, firefly2_3
# specifiy your platform. We use micaZ
#
PLATFORM = micaZ

# Target file name (without extension).
TARGET = commander

# programmer connected to serial device. For Windows use for example COM23
PROGRAMMING_PORT ?= /dev/ttyUSB0

NODE_ADDR ?= 1

# PROGRAMMING_PORT = `ls /dev/tty.* | grep usb | head -n 1` # programmer connected to serial device

# Set this such that the nano-RK directory is the base path
ROOT_DIR = $(NANORK)

# Set platform specific defines
# The following will be defined based on the PLATFORM variable:
# PROG_TYPE  (e.g. avrdude, or uisp)
# MCU (e.g. atmega32, atmega128, atmega1281)
# RADIO (e.g. cc2420)
include $(ROOT_DIR)/include/platform.mk


SRC = $(TARGET).c
SRC += $(ROOT_DIR)/src/net/bmac/bmac.c

# Add extra source files.
# For example:
# SRC += $(ROOT_DIR)/src/platform/$(PLATFORM_TYPE)/source/my_src1.c
# SRC += $(ROOT_DIR)/src/platform/$(PLATFORM_TYPE)/source/my_src2.c

# Add extra includes files.
# For example:
# EXTRAINCDIRS += $(ROOT_DIR)/src/platform/include
EXTRAINCDIRS += $(ROOT_DIR)/src/net/bmac


#  This is where the final compile and download happens
#
#  change this to common-color if you like a fancy output.
#
include $(ROOT_DIR)/include/platform/$(PLATFORM)/common.mk
#include $(ROOT_DIR)/include/platform/$(PLATFORM)/common-color.mk

