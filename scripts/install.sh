#!/bin/bash
rw
cp fpga/lockbox.bit /opt/redpitaya/fpga/
cp lib/liblockbox.so /opt/redpitaya/lib/
cp bin/lockbox-server /opt/redpitaya/bin/
cp systemd/lockbox.service /etc/systemd/system/
ro
