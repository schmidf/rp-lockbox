# Copyright (c) 2018, Fabian Schmid, Edward Wang
#
# All rights reserved.
"""GUI for controlling the Red Pitaya lockbox."""

import sys
import logging
import socket
from PyQt5 import QtWidgets, QtGui
import widgets
import rp_lockbox

LOGLEVEL = logging.WARNING
LOG = logging.getLogger(__name__)

FONT_TOP_GROUPS = QtGui.QFont(QtGui.QFont('Arial', 12, QtGui.QFont.Bold))
FONT_INTERMEDIATE_GROUPS = QtGui.QFont('Arial', 10, QtGui.QFont.Bold)
# pylint: disable=R0904
class MainWindow(QtWidgets.QMainWindow):
    """Main application class."""

    def __init__(self):
        super().__init__()

        rp_addr, ok_pressed = QtWidgets.QInputDialog.getText(
            self, "Connect to Red Pitaya", "IP or Hostname:", QtWidgets.QLineEdit.Normal, "")

        while ok_pressed:
            try:
                self.red_pitaya = rp_lockbox.RedPitaya(rp_addr)
            except socket.error as err:
                LOG.error("Could not connect to Red Pitaya at %s. Error: %s", rp_addr, err)
                rp_addr, ok_pressed = QtWidgets.QInputDialog.getText(
                    self, "Connect to Red Pitaya", "Failed to connect, try again.\nIP or Hostname:",
                    QtWidgets.QLineEdit.Normal, "")
            else: # Connection successful
                break

        if not ok_pressed: # Exit when cancel is pressed
            sys.exit()

        self.rp_addr = rp_addr
        self.setWindowTitle('rp-lockbox control')
        left = 20
        top = 50
        width = 640
        height = 480
        self.setGeometry(left, top, width, height)
        button_update_parameters = QtWidgets.QPushButton('Get parameters from device')
        button_update_parameters.clicked.connect(self.update_parameters)

        button_reconnect = QtWidgets.QPushButton('Reconnect')
        button_reconnect.clicked.connect(self.reconnect)

        button_save_parameters = QtWidgets.QPushButton('Save parameters to SD card')
        button_save_parameters.clicked.connect(self.red_pitaya.save_lockbox_config)

        button_load_parameters = QtWidgets.QPushButton('Load parameters from SD card')
        button_load_parameters.clicked.connect(self.load_parameters)

        central_layout = QtWidgets.QGridLayout()
        self.pid_groups = {}
        self.relock_groups = {}
        self.output_groups = {}

        central_layout.addWidget(self._create_lockbox_group(1), 0, 0)
        central_layout.addWidget(self._create_lockbox_group(2), 0, 1)

        self.output_groups[1] = widgets.OutputGroup(1, self.red_pitaya, self.reconnect)
        self.output_groups[2] = widgets.OutputGroup(2, self.red_pitaya, self.reconnect)
        self.output_groups[1].setFont(FONT_TOP_GROUPS)
        self.output_groups[2].setFont(FONT_TOP_GROUPS)
        central_layout.addWidget(self.output_groups[1])
        central_layout.addWidget(self.output_groups[2])

        central_layout.addWidget(button_reconnect, 2, 0)
        central_layout.addWidget(button_update_parameters, 3, 0)
        central_layout.addWidget(button_save_parameters, 2, 1)
        central_layout.addWidget(button_load_parameters, 3, 1)

        central_widget = QtWidgets.QWidget(self)
        self.setCentralWidget(central_widget)
        central_widget.setLayout(central_layout)

    def _create_lockbox_group(self, num_in):
        """Return a QGroupBox containing the UI elements for the lockbox with the specified input
        channel."""
        group_box = QtWidgets.QGroupBox("Lockbox (Input {})".format(num_in))
        group_box.setFont(FONT_TOP_GROUPS)
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self._create_lockbox_output_group(num_in, 1))
        layout.addWidget(self._create_lockbox_output_group(num_in, 2))
        group_box.setLayout(layout)

        return group_box

    def _create_lockbox_output_group(self, num_in, num_out):
        """Return a QGroupBox containing the UI elements for the PID and relock parameters of the
        specified input and output channels."""
        group_box = QtWidgets.QGroupBox("Output {}".format(num_out))
        group_box.setFont(FONT_INTERMEDIATE_GROUPS)
        layout = QtWidgets.QVBoxLayout()
        group_string = "{}{}".format(num_out, num_in)
        self.pid_groups[group_string] = widgets.PIDGroup(num_in, num_out, self.red_pitaya,
                                                         self.reconnect)
        self.relock_groups[group_string] = widgets.RelockGroup(num_in, num_out, self.red_pitaya,
                                                               self.reconnect)
        layout.addWidget(self.pid_groups[group_string])
        layout.addWidget(self.relock_groups[group_string])
        group_box.setLayout(layout)

        return group_box

    def reconnect(self):
        """Try to open a socket connection to the Red Pitaya again. If this fails, ask for a new IP
        address or hostname."""
        try:
            self.red_pitaya.connect()
        except socket.error as err:
            LOG.error("Failed to reconnect to Red Pitaya at %s. Error: %s", self.red_pitaya.host,
                      err)
        else:
            return

        rp_addr, ok_pressed = QtWidgets.QInputDialog.getText(
            self, "Connect to Red Pitaya",
            "Failed to reconnect automatically.\nEnter IP or Hostname:", QtWidgets.QLineEdit.Normal,
            self.red_pitaya.host)

        while ok_pressed:
            try:
                self.red_pitaya.host = rp_addr
                self.red_pitaya.connect()
            except socket.error as err:
                LOG.error("Could not connect to Red Pitaya at %s. Error: %s", rp_addr, err)
                rp_addr, ok_pressed = QtWidgets.QInputDialog.getText(
                    self, "Connect to Red Pitaya", "Failed to connect, try again.\nIP or Hostname:",
                    QtWidgets.QLineEdit.Normal, "")
            else: # Connection successful
                break

    def update_parameters(self):
        """Get parameter values from the red pitaya and display them in the UI elements."""
        for pid_group in self.pid_groups.values():
            pid_group.update_parameters()
        for relock_group in self.relock_groups.values():
            relock_group.update_parameters()
        for output_group in self.output_groups.values():
            output_group.update_parameters()

    def load_parameters(self):
        """Load parameters from the Red Pitaya SD-Card and update the UI elements."""
        self.red_pitaya.load_lockbox_config()
        self.update_parameters()

if __name__ == '__main__':
    logging.basicConfig(level=LOGLEVEL)
    QT_APP = QtWidgets.QApplication(sys.argv)

    MAIN_WINDOW = MainWindow()
    MAIN_WINDOW.show()

    sys.exit(QT_APP.exec_())
