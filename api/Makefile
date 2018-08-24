# Makefile for the Red Pitaya lockbox library (liblockbox).

# Versioning system
# BUILD_NUMBER ?= 0
# REVISION ?= devbuild
# VER:=$(shell cat info/info.json | grep version | sed -e 's/.*:\ *\"//' | sed -e 's/-.*//')

LIBLOCKBOX=lib/liblockbox.so

all: $(LIBLOCKBOX)

$(LIBLOCKBOX):
	$(MAKE) -C src

clean:
	$(MAKE) -C src clean

install:
	$(MAKE) -C src install INSTALL_DIR=$(abspath $(INSTALL_DIR))
