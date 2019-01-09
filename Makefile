INSTALL_DIR ?= build
TARBALL = rp-lockbox.tar.gz

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
# Create .tar.xz archive
################################################################################

tarball: $(TARBALL)

$(TARBALL): install
	$(eval TMP := $(shell mktemp -d))
	mkdir -p $(TMP)/rp-lockbox
	cp -r $(INSTALL_DIR)/. $(TMP)/rp-lockbox
	cp -r systemd $(TMP)/rp-lockbox
	cp scripts/install.sh $(TMP)/rp-lockbox
	tar -cf rp-lockbox.tar.gz -C $(TMP) rp-lockbox
	rm -rf $(TMP)

################################################################################
# Cleanup
################################################################################
clean:
	$(MAKE) -C $(LIBLOCKBOX_DIR) clean
	$(MAKE) -C $(SCPI_SERVER_DIR) clean
	$(MAKE) -C $(FPGA_DIR) clean
	rm -rf rp-lockbox.tar.xz
