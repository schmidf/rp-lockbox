INSTALL_DIR ?= build

all: api scpi fpga

$(INSTALL_DIR):
	mkdir -p $@

################################################################################
# API library
################################################################################
LIBLOCKBOX_DIR = api

.PHONY: api
api:
	$(MAKE) -C $(LIBLOCKBOX_DIR)

################################################################################
# SCPI server
################################################################################
SCPI_SERVER_DIR = scpi-server

.PHONY: scpi
scpi:
	$(MAKE) -C $(SCPI_SERVER_DIR)

################################################################################
# FPGA bitfile
################################################################################
FPGA_DIR = fpga

.PHONY: fpga
fpga:
	$(MAKE) -C $(FPGA_DIR)

################################################################################
# Copy build products to INSTALL_DIR
################################################################################

.PHONY: install
install:
	$(MAKE) -C $(LIBLOCKBOX_DIR) install INSTALL_DIR=$(abspath $(INSTALL_DIR))
	$(MAKE) -C $(SCPI_SERVER_DIR) install INSTALL_DIR=$(abspath $(INSTALL_DIR))
	$(MAKE) -C $(FPGA_DIR) install INSTALL_DIR=$(abspath $(INSTALL_DIR))

################################################################################
# Cleanup
################################################################################
clean:
	$(MAKE) -C $(LIBLOCKBOX_DIR) clean
	$(MAKE) -C $(SCPI_SERVER_DIR) clean
	$(MAKE) -C $(FPGA_DIR) clean
