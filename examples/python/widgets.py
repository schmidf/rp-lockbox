# Copyright (c) 2018, Fabian Schmid, Edward Wang
#
# All rights reserved.
"""This module contains QWidgets for the RedPitaya GUI."""

import socket
import logging
from PyQt5 import QtWidgets, QtGui, QtCore

LOG = logging.getLogger(__name__)

FONT_GROUP_HEADER = QtGui.QFont('Arial', 10, QtGui.QFont.Normal)
FONT_GROUP_CONTENT = QtGui.QFont('Arial', 8, QtGui.QFont.Normal)

class PIDGroup(QtWidgets.QGroupBox):
    """Widget for PID controls."""

    def __init__(self, num_in, num_out, red_pitaya, reconnect_func, parent=None):
        """Initialize the widget.

        :num_in: the input channel (1 or 2)
        :num_out: the output channel (1 or 2)
        :red_pitaya: a rp_lockbox.RedPitaya object
        :reconnect_func: function to call when the connection to the red pitaya has failed
        :parent: If parent is None, the new widget becomes a window. If parent is another widget,
                 the widget becomes a child window inside parent. Defaults to None.
        """
        title = "PID"

        super().__init__(title, parent)

        self.num_in = num_in
        self.num_out = num_out
        self.red_pitaya = red_pitaya
        self.reconnect = reconnect_func

        self.setFont(FONT_GROUP_HEADER)

        layout = QtWidgets.QVBoxLayout()

        central_widget = QtWidgets.QWidget()
        central_widget.setFont(FONT_GROUP_CONTENT)
        central_widget_layout = QtWidgets.QVBoxLayout()

        self.spin_box_sp = QtWidgets.QDoubleSpinBox()
        self.spin_box_sp.setKeyboardTracking(False)
        self.spin_box_sp.setDecimals(3)
        self.spin_box_sp.setRange(-1, 1)
        self.spin_box_sp.setSingleStep(0.001)
        self.spin_box_sp.setPrefix("Setpoint = ")
        self.spin_box_sp.setSuffix(" V")
        self.spin_box_sp.valueChanged.connect(self.setpoint)
        central_widget_layout.addWidget(self.spin_box_sp)

        self.spin_box_kp = QtWidgets.QDoubleSpinBox()
        self.spin_box_kp.setKeyboardTracking(False)
        self.spin_box_kp.setDecimals(3)
        self.spin_box_kp.setMaximum(4096)
        self.spin_box_kp.setSingleStep(0.01)
        self.spin_box_kp.valueChanged.connect(self.kp_gain)
        self.spin_box_kp.setPrefix("KP = ")
        central_widget_layout.addWidget(self.spin_box_kp)

        self.spin_box_ki = QtWidgets.QDoubleSpinBox()
        self.spin_box_ki.setKeyboardTracking(False)
        self.spin_box_ki.setMaximum(7812499)
        self.spin_box_ki.valueChanged.connect(self.ki_gain)
        self.spin_box_ki.setPrefix("KI = ")
        self.spin_box_ki.setSuffix(" 1/s")
        central_widget_layout.addWidget(self.spin_box_ki)

        self.spin_box_kd = QtWidgets.QSpinBox()
        self.spin_box_kd.setKeyboardTracking(False)
        self.spin_box_kd.valueChanged.connect(self.kd_gain)
        self.spin_box_kd.setPrefix("KD = ")
        self.spin_box_kd.setMaximum(8191)
        #self.spin_box_kd.setSuffix(" s")
        central_widget_layout.addWidget(self.spin_box_kd)

        self.check_box_inverted = QtWidgets.QCheckBox('Inverted')
        self.check_box_inverted.stateChanged.connect(self.inv_state)
        central_widget_layout.addWidget(self.check_box_inverted)

        self.spin_box_int_reset = QtWidgets.QCheckBox('Integrator reset')
        self.spin_box_int_reset.stateChanged.connect(self.reset_state)
        central_widget_layout.addWidget(self.spin_box_int_reset)

        self.check_box_int_hold = QtWidgets.QCheckBox('Integrator hold')
        self.check_box_int_hold.stateChanged.connect(self.hold_state)
        central_widget_layout.addWidget(self.check_box_int_hold)

        self.check_box_int_auto_reset = QtWidgets.QCheckBox('Automatic integrator reset')
        self.check_box_int_auto_reset.stateChanged.connect(self.auto_state)
        central_widget_layout.addWidget(self.check_box_int_auto_reset)

        self.update_parameters()

        central_widget.setLayout(central_widget_layout)
        layout.addWidget(central_widget)

        self.setLayout(layout)

    def update_parameters(self):
        """Get parameter values from the red pitaya and display them in the UI elements."""
        _blocked = QtCore.QSignalBlocker(self.spin_box_sp)
        self.spin_box_sp.setValue(self.red_pitaya.get_setpoint(self.num_in, self.num_out))
        _blocked = QtCore.QSignalBlocker(self.spin_box_kp)
        self.spin_box_kp.setValue(self.red_pitaya.get_kp(self.num_in, self.num_out))
        _blocked = QtCore.QSignalBlocker(self.spin_box_ki)
        self.spin_box_ki.setValue(self.red_pitaya.get_ki(self.num_in, self.num_out))
        _blocked = QtCore.QSignalBlocker(self.spin_box_kd)
        self.spin_box_kd.setValue(self.red_pitaya.get_kd(self.num_in, self.num_out))
        _blocked = QtCore.QSignalBlocker(self.spin_box_int_reset)
        self.spin_box_int_reset.setChecked(self.red_pitaya.get_int_reset_state(self.num_in,
                                                                               self.num_out))
        _blocked = QtCore.QSignalBlocker(self.check_box_int_hold)
        self.check_box_int_hold.setChecked(self.red_pitaya.get_int_hold_state(self.num_in,
                                                                              self.num_out))
        _blocked = QtCore.QSignalBlocker(self.check_box_int_auto_reset)
        self.check_box_int_auto_reset.setChecked(self.red_pitaya.get_int_auto_state(self.num_in,
                                                                                    self.num_out))
        _blocked = QtCore.QSignalBlocker(self.check_box_inverted)
        self.check_box_inverted.setChecked(self.red_pitaya.get_inv_state(self.num_in, self.num_out))

    def setpoint(self, value):
        """Set the PID setpoint."""
        try:
            self.red_pitaya.set_setpoint(self.num_in, self.num_out, value)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.setpoint(value)

    def kp_gain(self, gain):
        """Set P gain."""
        try:
            self.red_pitaya.set_kp(self.num_in, self.num_out, gain)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.kp_gain(gain)

    def ki_gain(self, gain):
        """Set I gain."""
        try:
            self.red_pitaya.set_ki(self.num_in, self.num_out, gain)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.ki_gain(gain)

    def kd_gain(self, gain):
        """Set D gain."""
        try:
            self.red_pitaya.set_kd(self.num_in, self.num_out, gain)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.kd_gain(gain)

    def reset_state(self, state):
        """Reset the integrator register."""
        try:
            self.red_pitaya.set_int_reset_state(self.num_in, self.num_out, state)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.reset_state(state)

    def hold_state(self, state):
        """Hold the status of the integrator register."""
        try:
            self.red_pitaya.set_int_hold_state(self.num_in, self.num_out, state)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.hold_state(state)

    def auto_state(self, state):
        """If enabled, the integrator register is reset when the PID output hits the configured
        limit."""
        try:
            self.red_pitaya.set_int_auto_state(self.num_in, self.num_out, state)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.auto_state(state)

    def inv_state(self, state):
        """Invert the sign of the PID output."""
        try:
            self.red_pitaya.set_inv_state(self.num_in, self.num_out, state)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.inv_state(state)

    def _warn_and_reconnect(self, err):
        """Log a warning message that sending a command has failed and reconnect to the device.

        :err: the caught exception whose message should be printed
        """
        LOG.error("Failed to send command to Red Pitaya. Error: %s", err)
        self.reconnect()

class RelockGroup(QtWidgets.QGroupBox):
    """Widget for relock controls."""

    def __init__(self, num_in, num_out, red_pitaya, reconnect_func, parent=None):
        """Initialize the widget.

        :num_in: the input channel (1 or 2)
        :num_out: the output channel (1 or 2)
        :red_pitaya: a rp_lockbox.RedPitaya object
        :reconnect_func: function to call when the connection to the red pitaya has failed
        :parent: If parent is None, the new widget becomes a window. If parent is another widget,
                 the widget becomes a child window inside parent. Defaults to None.
        """
        if num_in == 1:
            relock_in = 2
        else:
            relock_in = 1

        title = "Relock (Input {})".format(relock_in)

        super().__init__(title, parent)

        self.num_in = num_in
        self.num_out = num_out
        self.red_pitaya = red_pitaya
        self.reconnect = reconnect_func

        self.setFont(FONT_GROUP_HEADER)

        layout = QtWidgets.QVBoxLayout()

        central_widget = QtWidgets.QWidget()
        central_widget.setFont(FONT_GROUP_CONTENT)
        central_widget_layout = QtWidgets.QVBoxLayout()

        self.check_box_enabled = QtWidgets.QCheckBox('Enabled')
        self.check_box_enabled.stateChanged.connect(self.relock_state)
        central_widget_layout.addWidget(self.check_box_enabled)

        self.spin_box_minimum = QtWidgets.QDoubleSpinBox()
        self.spin_box_minimum.setKeyboardTracking(False)
        self.spin_box_minimum.setDecimals(3)
        self.spin_box_minimum.setRange(-1, 1)
        self.spin_box_minimum.setSingleStep(0.001)
        self.spin_box_minimum.setPrefix("Min = ")
        self.spin_box_minimum.setSuffix(" V")
        self.spin_box_minimum.valueChanged.connect(self.relock_min)
        central_widget_layout.addWidget(self.spin_box_minimum)

        self.spin_box_maximum = QtWidgets.QDoubleSpinBox()
        self.spin_box_maximum.setKeyboardTracking(False)
        self.spin_box_maximum.setDecimals(3)
        self.spin_box_maximum.setRange(-1, 1)
        self.spin_box_maximum.setSingleStep(0.001)
        self.spin_box_maximum.setPrefix("Max = ")
        self.spin_box_maximum.setSuffix(" V")
        self.spin_box_maximum.valueChanged.connect(self.relock_max)
        central_widget_layout.addWidget(self.spin_box_maximum)

        self.spin_box_slew_rate = QtWidgets.QDoubleSpinBox()
        self.spin_box_slew_rate.setKeyboardTracking(False)
        self.spin_box_slew_rate.setMaximum(1E6)
        self.spin_box_slew_rate.setPrefix("Slew Rate = ")
        self.spin_box_slew_rate.setSuffix(" V/s")
        self.spin_box_slew_rate.valueChanged.connect(self.relock_stepsize)
        central_widget_layout.addWidget(self.spin_box_slew_rate)

        self.update_parameters()

        central_widget.setLayout(central_widget_layout)
        layout.addWidget(central_widget)

        self.setLayout(layout)

    def update_parameters(self):
        """Get parameter values from the red pitaya and display them in the UI elements."""
        _blocked = QtCore.QSignalBlocker(self.check_box_enabled)
        self.check_box_enabled.setChecked(self.red_pitaya.get_relock_state(self.num_in,
                                                                           self.num_out))
        _blocked = QtCore.QSignalBlocker(self.spin_box_minimum)
        self.spin_box_minimum.setValue(self.red_pitaya.get_relock_minimum(self.num_in,
                                                                          self.num_out))
        _blocked = QtCore.QSignalBlocker(self.spin_box_maximum)
        self.spin_box_maximum.setValue(self.red_pitaya.get_relock_maximum(self.num_in,
                                                                          self.num_out))
        _blocked = QtCore.QSignalBlocker(self.spin_box_slew_rate)
        self.spin_box_slew_rate.setValue(self.red_pitaya.get_relock_stepsize(self.num_in,
                                                                             self.num_out))

    def relock_state(self, state):
        """Enable or disable the PID relock feature. (See rp_lockbox.py for further details)"""
        try:
            self.red_pitaya.set_relock_state(self.num_in, self.num_out, state)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.relock_state(state)

    def relock_min(self, minimum):
        """Set the minimum input voltage for which the PID is considered locked."""
        try:
            self.red_pitaya.set_relock_minimum(self.num_in, self.num_out, minimum)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.relock_max(minimum)

    def relock_max(self, maximum):
        """Set the maximum input voltage for which the PID is considered locked."""
        try:
            self.red_pitaya.set_relock_maximum(self.num_in, self.num_out, maximum)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.relock_max(maximum)

    def relock_stepsize(self, stepsize):
        """Set the step size (slew rate) of the relock."""
        try:
            self.red_pitaya.set_relock_stepsize(self.num_in, self.num_out, stepsize)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.relock_stepsize(stepsize)

    def _warn_and_reconnect(self, err):
        """Log a warning message that sending a command has failed and reconnect to the device.

        :err: the caught exception whose message should be printed
        """
        LOG.error("Failed to send command to Red Pitaya. Error: %s", err)
        self.reconnect()

class OutputGroup(QtWidgets.QGroupBox):
    """Widget for output controls."""

    def __init__(self, num_out, red_pitaya, reconnect_func, parent=None):
        """Initialize the widget.

        :num_out: the output channel (1 or 2)
        :red_pitaya: a rp_lockbox.RedPitaya object
        :reconnect_func: function to call when the connection to the red pitaya has failed
        :parent: If parent is None, the new widget becomes a window. If parent is another widget,
                 the widget becomes a child window inside parent. Defaults to None.
        """
        title = "Output {}".format(num_out)
        super().__init__(title, parent)

        self.num_out = num_out
        self.reconnect = reconnect_func

        self.setFont(FONT_GROUP_HEADER)
        self.red_pitaya = red_pitaya

        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.create_limit_group())
        layout.addWidget(self.create_generator_group())

        self.update_parameters()

        self.setLayout(layout)

    def create_limit_group(self):
        """Creates output limiting group inside output group."""
        group_box = QtWidgets.QGroupBox("Limit")
        group_box.setFont(FONT_GROUP_HEADER)
        layout = QtWidgets.QVBoxLayout()

        central_widget = QtWidgets.QWidget()
        central_widget.setFont(FONT_GROUP_CONTENT)
        central_widget_layout = QtWidgets.QVBoxLayout()

        self.spin_box_minimum = QtWidgets.QDoubleSpinBox()
        self.spin_box_minimum.setKeyboardTracking(False)
        self.spin_box_minimum.setDecimals(3)
        self.spin_box_minimum.setRange(-1, 1)
        self.spin_box_minimum.setSingleStep(0.001)
        self.spin_box_minimum.setPrefix("Min = ")
        self.spin_box_minimum.setSuffix(" V")
        self.spin_box_minimum.valueChanged.connect(self.output_min)
        central_widget_layout.addWidget(self.spin_box_minimum)

        self.spin_box_maximum = QtWidgets.QDoubleSpinBox()
        self.spin_box_maximum.setKeyboardTracking(False)
        self.spin_box_maximum.setDecimals(3)
        self.spin_box_maximum.setRange(-1, 1)
        self.spin_box_maximum.setSingleStep(0.001)
        self.spin_box_maximum.setPrefix("Max = ")
        self.spin_box_maximum.setSuffix(" V")
        self.spin_box_maximum.valueChanged.connect(self.output_max)
        central_widget_layout.addWidget(self.spin_box_maximum)

        central_widget.setLayout(central_widget_layout)
        layout.addWidget(central_widget)

        group_box.setLayout(layout)

        return group_box


    def create_generator_group(self):
        """Creates signal generator group inside output group"""
        group_box = QtWidgets.QGroupBox("Signal Generator")
        group_box.setFont(FONT_GROUP_HEADER)
        layout = QtWidgets.QVBoxLayout()

        central_widget = QtWidgets.QWidget()
        central_widget.setFont(FONT_GROUP_CONTENT)
        central_widget_layout = QtWidgets.QVBoxLayout()

        self.check_box_output_state = QtWidgets.QCheckBox('Enabled')
        self.check_box_output_state.stateChanged.connect(self.output_state)
        central_widget_layout.addWidget(self.check_box_output_state)

        self.spin_box_frequency = QtWidgets.QDoubleSpinBox()
        self.spin_box_frequency.setKeyboardTracking(False)
        self.spin_box_frequency.setMaximum(62.5e6)
        self.spin_box_frequency.setPrefix("Frequency = ")
        self.spin_box_frequency.setSuffix(" Hz")
        self.spin_box_frequency.valueChanged.connect(self.generator_frequency)
        central_widget_layout.addWidget(self.spin_box_frequency)

        self.combo_box_waveform = QtWidgets.QComboBox()
        self.combo_box_waveform.addItems(["SINE", "SQUARE", "TRIANGLE", "SAWU", "SAWD"])
        self.combo_box_waveform.currentTextChanged.connect(self.generator_waveform)
        central_widget_layout.addWidget(self.combo_box_waveform)

        self.spin_box_amp = QtWidgets.QDoubleSpinBox()
        self.spin_box_amp.setKeyboardTracking(False)
        self.spin_box_amp.setDecimals(3)
        self.spin_box_amp.setRange(-1, 1)
        self.spin_box_amp.setSingleStep(0.001)
        self.spin_box_amp.setPrefix("amplitude = ")
        self.spin_box_amp.setSuffix(" V")
        self.spin_box_amp.valueChanged.connect(self.generator_amplitude)
        central_widget_layout.addWidget(self.spin_box_amp)

        self.spin_box_offset = QtWidgets.QDoubleSpinBox()
        self.spin_box_offset.setKeyboardTracking(False)
        self.spin_box_offset.setDecimals(3)
        self.spin_box_offset.setRange(-1, 1)
        self.spin_box_offset.setSingleStep(0.001)
        self.spin_box_offset.setPrefix("offset = ")
        self.spin_box_offset.setSuffix(" V")
        self.spin_box_offset.valueChanged.connect(self.generator_offset)
        central_widget_layout.addWidget(self.spin_box_offset)

        central_widget.setLayout(central_widget_layout)
        layout.addWidget(central_widget)

        group_box.setLayout(layout)

        return group_box


    def update_parameters(self):
        """Get parameter values from the red pitaya and display them in the UI elements."""
        _blocked = QtCore.QSignalBlocker(self.spin_box_minimum)
        self.spin_box_minimum.setValue(self.red_pitaya.get_output_minimum(self.num_out))
        _blocked = QtCore.QSignalBlocker(self.spin_box_maximum)
        self.spin_box_maximum.setValue(self.red_pitaya.get_output_maximum(self.num_out))
        _blocked = QtCore.QSignalBlocker(self.check_box_output_state)
        self.check_box_output_state.setChecked(self.red_pitaya.get_output_state(self.num_out))
        _blocked = QtCore.QSignalBlocker(self.spin_box_frequency)
        self.spin_box_frequency.setValue(self.red_pitaya.get_generator_frequency(self.num_out))
        _blocked = QtCore.QSignalBlocker(self.combo_box_waveform)
        self.combo_box_waveform.setCurrentText(self.red_pitaya.get_generator_waveform(self.num_out))
        _blocked = QtCore.QSignalBlocker(self.spin_box_amp)
        self.spin_box_amp.setValue(self.red_pitaya.get_generator_amplitude(self.num_out))
        _blocked = QtCore.QSignalBlocker(self.spin_box_offset)
        self.spin_box_offset.setValue(self.red_pitaya.get_generator_offset(self.num_out))

    def output_min(self, minimum):
        """Set the minimum output voltage for the specified channel."""
        try:
            self.red_pitaya.set_output_minimum(self.num_out, minimum)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.output_min(minimum)

    def output_max(self, maximum):
        """Set the maximum output voltage for the specified channel."""
        try:
            self.red_pitaya.set_output_maximum(self.num_out, maximum)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.output_max(maximum)

    def output_state(self, state):
        """Disable or enable fast analog outputs."""
        try:
            self.red_pitaya.set_output_state(self.num_out, state)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.output_state(state)

    def generator_frequency(self, frequency):
        """Set the frequency of fast analog outputs."""
        try:
            self.red_pitaya.set_generator_frequency(self.num_out, frequency)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.generator_frequency(frequency)

    def generator_waveform(self, form):
        """Set the waveform of fast analog outputs."""
        try:
            self.red_pitaya.set_generator_waveform(self.num_out, form)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.generator_waveform(form)

    def generator_amplitude(self, amplitude):
        """Set the amplitude voltage of fast analog outputs."""
        try:
            self.red_pitaya.set_generator_amplitude(self.num_out, amplitude)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.generator_amplitude(amplitude)

    def generator_offset(self, offset):
        """Set the offset voltage of fast analog outputs."""
        try:
            self.red_pitaya.set_generator_offset(self.num_out, offset)
        except socket.error as err:
            self._warn_and_reconnect(err)
            self.generator_offset(offset)

    def _warn_and_reconnect(self, err):
        """Log a warning message that sending a command has failed and reconnect to the device.

        :err: the caught exception whose message should be printed
        """
        LOG.error("Failed to send command to Red Pitaya. Error: %s", err)
        self.reconnect()
