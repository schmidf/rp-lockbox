##
# Copyright (c) 2018, Fabian Schmid
#
# All rights reserved.
#
# (c) Red Pitaya  http://www.redpitaya.com
#
# Library librp project file. To build the library run:
# 'make all'
#
# This project file is written for GNU/Make software. For more details please 
# visit: http://www.gnu.org/software/make/manual/make.html
# GNU Compiler Collection (GCC) tools are used for the compilation and linkage. 
# For the details about the usage and building please visit:
# http://gcc.gnu.org/onlinedocs/gcc/
#

# Versioning system
VERSION ?= 0.00-0000
REVISION ?= devbuild

# Extensions
CPPEXT   = .c
CXXEXT   = .cpp
OBJEXT   = .o

# Directories paths
OBJECTS_DIR = ../obj
INSTALL_DIR ?= .
SOURCE_DIR  = .
OUTPUT_DIR  = ../lib

SHARED_LIB=$(OUTPUT_DIR)/liblockbox.so
STATIC_LIB=$(OUTPUT_DIR)/liblockbox.a

# Library name
TARGET=$(SHARED_LIB) $(STATIC_LIB)

# List of compiled object files
OBJECTS =	common.o \
		kiss_fft/kiss_fft.c \
		kiss_fft/kiss_fftr.c \
		oscilloscope.o \
		acq_handler.o \
		generate.o \
		gen_handler.o \
		calib.o \
		spec_dsp.o \
		spec_fpga.o \
		pid.o \
		limit.o \
		lockbox.o \
		analog_mixed_signals.o

OBJS = $(patsubst %$(OBJEXT), $(OBJECTS_DIR)/%$(OBJEXT), $(OBJECTS))

# GCC compiling & linking flags
CFLAGS  = -std=gnu99 -Wall -Werror -fPIC -Ikiss_fft -Os -s
CFLAGS += -DVERSION=$(VERSION) -DREVISION=$(REVISION)
CFLAGS += -I../include
LDFLAGS=-shared -Wl,--version-script=exportmap

# Additional libraries which needs to be dynamically linked to the executable
# -lm - System math library (used by cos(), sin(), sqrt(), ... functions)
LIBS=-lm -lpthread

# Main GCC executable (used for compiling and linking)
CC=$(CROSS_COMPILE)gcc

AR=$(CROSS_COMPILE)ar

# Main Makefile target 'all' - it iterates over all targets listed in $(TARGET)
# variable.
all: $(TARGET)

# Target with compilation rules to compile object from source files.
# It applies to all files ending with .o. During partial building only new object
# files are created for the source files (.c) which have newer timestamp then 
# objects (.o) files.
$(OBJECTS_DIR)/%.o:$(SOURCE_DIR)/%.c
	@mkdir -p $(@D)
	$(CC) -c $(CFLAGS) $< -o $@

# Makefile target with rules how to link executable for each target from
# $(SHARED_LIB) list.
$(SHARED_LIB): $(OBJS)
	mkdir -p $(OUTPUT_DIR)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS) $(LDFLAGS)

$(STATIC_LIB): $(OBJS)
	mkdir -p $(OUTPUT_DIR)
	$(AR) rcs $@ $^

# Clean target - when called it cleans all object files and executables.
clean:
	rm -f $(TARGET) $(OBJECTS_DIR)/*.o

# Install target - creates 'bin/' sub-directory in $(INSTALL_DIR) and copies all
# executables to that location.
install:
	mkdir -p $(INSTALL_DIR)/lib
	mkdir -p $(INSTALL_DIR)/include
	cp  $(TARGET) $(INSTALL_DIR)/lib
	cp -r ../include/redpitaya $(INSTALL_DIR)/include
