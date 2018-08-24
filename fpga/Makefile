# Copyright (c) 2018 Fabian Schmid
#
# All rights reserved.
#
# Authors: Matej Oblak, Iztok Jeras
# (C) Red Pitaya 2013-2015
#
# Red Pitaya FPGA/SoC Makefile 
#
# Produces:
#   1. FPGA bit file.

PRJ ?= lockbox

# Directory paths
INSTALL_DIR ?= .

# build artefacts
FPGA_BIT    = prj/$(PRJ)/out/red_pitaya.bit

# Vivado from Xilinx provides IP handling, FPGA compilation and is run in batch
# mode with an option to avoid log/journal files
VIVADO = vivado -nojournal -mode batch

.PHONY: all clean project

all: $(FPGA_BIT)

clean:
	rm -rf out .Xil .srcs sdk project
	rm -rf prj/$(PRJ)/out prj/$(PRJ)/.Xil prj/$(PRJ)/.srcs prj/$(PRJ)/sdk prj/$(PRJ)/project

project:
	vivado -source red_pitaya_vivado_project.tcl -tclargs $(PRJ)

$(FPGA_BIT):
	$(VIVADO) -source red_pitaya_vivado.tcl -tclargs $(PRJ)
	./synCheck.sh

install:
	mkdir -p $(INSTALL_DIR)/fpga
	cp $(FPGA_BIT) $(INSTALL_DIR)/fpga/lockbox.bit
