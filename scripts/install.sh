#!/bin/bash
rw
cp fpga/lockbox.bit /opt/redpitaya/fpga/
cp lib/liblockbox.so /opt/redpitaya/lib/
cp bin/lockbox-server /opt/redpitaya/bin/
cp -r web-interface /opt/redpitaya/
cp systemd/lockbox.service /etc/systemd/system/
cp systemd/lockbox-web-interface.service /etc/systemd/system/
ro
