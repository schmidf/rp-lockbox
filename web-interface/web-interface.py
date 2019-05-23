#!/usr/bin/env python
#
# Copyright (c) 2019, Malte Bieringer, Fabian Schmid
#
# All rights reserved.
"""Server-side module for the Red Pitaya lockbox web interface. Uses the bottle micro
web-framework."""
import os
import ctypes
import sys
import json
import logging
from bottle import route, run, request, static_file

logging.basicConfig()
LOG = logging.getLogger(__name__)

BASEDIR = os.path.dirname(__file__)

# Error codes returned by the API
ERROR_CODES = {
    1: "RP_EOED. Failed to Open Memory Device.",
    2: "RP_EOMD. Failed to Close Memory Devic.",
    3: "RP_ECMD. Failed to Map Memory Device.",
    4: "RP_EMMD. Failed to Unmap Memory Device.",
    5: "RP_EUMD. Value Out Of Range.",
    6: "RP_EOOR. LED Input Direction is not valid.",
    7: "RP_ELID. Modifying Read Only field.",
    8: "RP_EMRO. Writing to Input Pin is not valid.",
    9: "RP_EWIP. Invalid Pin number.",
    10: "RP_EPN. Uninitialized Input Argument.",
    11: "RP_UIA. Failed to Find Calibration Parameters.",
    12: "RP_FCA. Failed to Read Calibration Parameters.",
    13: "RP_RCA. Buffer too small.",
    14: "RP_BTS. Invalid parameter value.",
    15: "RP_EIPV. Unsupported Feature.",
    16: "RP_EUF. Data not normalized.",
    17: "RP_ENN. Failed to open bus.",
    18: "RP_EFOB. Failed to close bus.",
    19: "RP_EFCB. Failed to acquire bus access.",
    20: "RP_EABA. Failed to read from the bus.",
    21: "RP_EFRB. Failed to write to the bus.",
    22: "RP_EFWB. Extension module not connected.",
    23: "RP_EMNC. Failed to open config file.",
    24: "RP_EOCF. Incompatible config file version.",
    25: "RP_EICV. Failed to Open EEPROM Devic."}

PID_ID = {
    "PID_11": 0, # Input 1 -> Output 1
    "PID_12": 1, # Input 2 -> Output 1
    "PID_21": 2, # Input 1 -> Output 2
    "PID_22": 3} # Input 2 -> Output 2

AIN_ID = {
    0: "AIN0",
    1: "AIN1",
    2: "AIN2",
    3: "AIN3"}

def init_rp_library():
    """Initialize the Red Pitaya lockbox library. Exit the program on failure."""
    retval = RP_LIB.rp_Init()
    if retval != 0:
        LOG.error("Failed to initialize lockbox library. Error code: %s", ERROR_CODES[retval])
        sys.exit(-1)

@route('/')
def index():
    """Main HTML file."""
    return static_file("index.html", root=BASEDIR)

@route('/jquery-ui.css')
def jquery_ui_css():
    """jQuery UI style file."""
    return static_file("jquery-ui.css", root=os.path.join(BASEDIR, "css"))

@route('/jquery-ui.js')
def jquery_ui_js():
    """jQuery UI library."""
    return static_file("jquery-ui.js", root=os.path.join(BASEDIR, "js"))

@route('/external/jquery/jquery.js')
def jquery_js():
    """jQuery library."""
    return static_file("external/jquery/jquery.js", root=os.path.join(BASEDIR))

@route('/images/<name>')
def images(name):
    """Image files used by jQuery UI."""
    return static_file(name, root=os.path.join(BASEDIR, "images"))

@route("/_set_setpoint", method="POST")
def set_setpoint():
    """Handle POST request for setting the PID setpoint.

    Accepted POST parameters:
    :pid: the PID to adjust
    :setpoint: the value to set in V
    """
    setpoint = request.params.get("setpoint", 0, type=float)
    pid = request.params.get("pid", 1, type=int)
    retval = RP_LIB.rp_PIDSetSetpoint(pid, ctypes.c_float(setpoint))
    if retval != 0:
        LOG.error("Failed to set PID setpoint. Error code: %s", ERROR_CODES[retval])
    LOG.info("setpoint: %f", setpoint)
    LOG.info("PID: %d", pid)

@route("/_set_kp", method="POST")
def set_kp():
    """Handle POST request for setting the PID Kp.

    Accepted POST parameters:
    :pid: the PID to adjust
    :kp: the value to set
    """
    kp = request.params.get("kp", 0, type=float)
    pid = request.params.get("pid", 1, type=int)
    retval = RP_LIB.rp_PIDSetKp(pid, ctypes.c_float(kp))
    if retval != 0:
        LOG.error("Failed to set PID Kp. Error code: %s", ERROR_CODES[retval])
    LOG.info("Kp: %f", kp)
    LOG.info("PID: %d", pid)

@route("/_set_ki", method="POST")
def set_ki():
    """Handle POST request for setting the PID Ki.

    Accepted POST parameters:
    :pid: the PID to adjust
    :ki: the value to set in 1/s
    """
    ki = request.params.get("ki", 0, type=float)
    pid = request.params.get("pid", 1, type=int)
    retval = RP_LIB.rp_PIDSetKi(pid, ctypes.c_float(ki))
    if retval != 0:
        LOG.error("Failed to set PID Ki. Error code: %s", ERROR_CODES[retval])
    LOG.info("Ki: %f", ki)
    LOG.info("PID: %d", pid)

@route("/_set_kd", method="POST")
def set_kd():
    """Handle POST request for setting the PID Kd.

    Accepted POST parameters:
    :pid: the PID to adjust
    :kd: the value to set
    """
    kd = request.params.get("kd", 0, type=int)
    pid = request.params.get("pid", 1, type=int)
    retval = RP_LIB.rp_PIDSetKd(pid, ctypes.c_int(kd))
    if retval != 0:
        LOG.error("Failed to set PID Kd. Error code: %s", ERROR_CODES[retval])
    LOG.info("Kd: %f", kd)
    LOG.info("PID: %d", pid)

@route("/_set_inverted", method="POST")
def set_inverted():
    """Handle POST request for setting the PID feedback sign.

    Accepted POST parameters:
    :pid: the PID to adjust
    :inverted: true for a negative gain sign, false for a positive gain sign
    """
    inverted = request.params.get("inverted", 0) == "true"
    pid = request.params.get("pid", 1, type=int)
    retval = RP_LIB.rp_PIDSetInverted(pid, inverted)
    if retval != 0:
        LOG.error("Failed to set PID feedback sign. Error code: %s", ERROR_CODES[retval])

@route("/_set_hold", method="POST")
def set_hold():
    """Handle POST request for setting to hold internal state.

    Accepted POST parameters:
    :pid: the PID to adjust
    :hold: true if internal state should be held, false if not
    """
    hold = request.params.get("hold", 0) == "true"
    pid = request.params.get("pid", 1, type=int)
    retval = RP_LIB.rp_PIDSetHold(pid, hold)
    if retval != 0:
        LOG.error("Failed to set PID internal state holding. Error code: %s", ERROR_CODES[retval])

@route("/_set_int_reset", method="POST")
def set_int_reset():
    """Handle POST request for resetting the integrator.

    Accepted POST parameters:
    :pid: the PID to adjust
    :int_reset: true if internal state should be reset, false if not
    """
    int_reset = request.params.get("int_reset", 0) == "true"
    pid = request.params.get("pid", 1, type=int)
    retval = RP_LIB.rp_PIDSetIntReset(pid, int_reset)
    if retval != 0:
        LOG.error("Failed to set PID integrator reset. Error code: %s", ERROR_CODES[retval])

@route("/_set_int_auto", method="POST")
def set_int_auto():
    """Handle POST request for resetting the integrator automatically.

    Accepted POST parameters:
    :pid: the PID to adjust
    :int_reset: If true, the integrator register is reset when the PID output hits the configured
                limit
    """
    int_auto = request.params.get("int_auto", 0) == "true"
    pid = request.params.get("pid", 1, type=int)
    retval = RP_LIB.rp_PIDSetResetWhenRailed(pid, int_auto)
    if retval != 0:
        LOG.error("Failed to set PID automatical integrator reset. Error code: %s",
                  ERROR_CODES[retval])

@route("/_set_relock_min", method="POST")
def set_relock_min():
    """Handle POST request for setting the minimum input voltage for which the PID is considered
    locked.

    Accepted POST parameters:
    :pid: the PID to adjust
    :relock_min: the value to set
    """
    relock_min = request.params.get("relock_min", 0, type=float)
    pid = request.params.get("pid", 1, type=int)
    retval = RP_LIB.rp_PIDSetRelockMinimum(pid, ctypes.c_float(relock_min))
    if retval != 0:
        LOG.error("Failed to set PID minimum relock voltage. Error code: %s", ERROR_CODES[retval])
    LOG.info("Minimum relock voltage: %f", relock_min)
    LOG.info("PID: %d", pid)

@route("/_set_relock_max", method="POST")
def set_relock_max():
    """Handle POST request for setting the maximum input voltage for which the PID is considered
    locked.

    Accepted POST parameters:
    :pid: the PID to adjust
    :relock_max: the value to set
    """
    relock_max = request.params.get("relock_max", 0, type=float)
    pid = request.params.get("pid", 1, type=int)
    retval = RP_LIB.rp_PIDSetRelockMaximum(pid, ctypes.c_float(relock_max))
    if retval != 0:
        LOG.error("Failed to set PID maximum relock voltage. Error code: %s", ERROR_CODES[retval])
    LOG.info("Maximum relock voltage: %f", relock_max)
    LOG.info("PID: %d", pid)

@route("/_set_relock_slew_rate", method="POST")
def set_relock_slew_rate():
    """Handle POST request for setting slew rate of the relock in V/s.

    Accepted POST parameters:
    :pid: the PID to adjust
    :relock_slew_rate: the value to set
    """
    relock_slew_rate = request.params.get("relock_slew_rate", 0, type=float)
    pid = request.params.get("pid", 1, type=int)
    retval = RP_LIB.rp_PIDSetRelockStepsize(pid, ctypes.c_float(relock_slew_rate))
    if retval != 0:
        LOG.error("Failed to set PID relock slew rate. Error code: %s", ERROR_CODES[retval])
    LOG.info("Relock slew rate: %f", relock_slew_rate)
    LOG.info("PID: %d", pid)

@route("/_set_relock_enabled", method="POST")
def set_relock_enabled():
    """Handle POST request for enabling the relock group.

    Accepted POST parameters:
    :pid: the PID to adjust
    :relock_enabled: If true, the PID relock feature is enabled.
    """
    relock_enabled = request.params.get("relock_enabled", 0) == "true"
    pid = request.params.get("pid", 1, type=int)
    retval = RP_LIB.rp_PIDSetRelock(pid, relock_enabled)
    if retval != 0:
        LOG.error("Failed to set PID relock enabled. Error code: %s", ERROR_CODES[retval])

@route("/_set_relock_input", method="POST")
def set_relock_input():
    """Handle POST request for setting the analog input to be used for relocking the PID.

    Accepted POST parameters:
    :pid: the PID to adjust
    :ain: number of the analog input to be used for relocking the PID.
    """
    ain = request.params.get("ain", 0, type=int)
    pid = request.params.get("pid", 1, type=int)
    retval = RP_LIB.rp_PIDSetRelockInput(pid, ctypes.c_int(ain))
    if retval != 0:
        LOG.error("Failed to select analog input to be used for relocking the PID. Error code: %s",
                  ERROR_CODES[retval])

@route("/_set_limit_min", method="POST")
def set_limit_min():
    """Handle POST request for setting minimum output value.

    Accepted POST parameters:
    :output: the output channel to adjust
    :limit_min: the value to set.
    """
    limit_min = request.params.get("limit_min", 0, type=float)
    output = request.params.get("output", 1, type=int)
    retval = RP_LIB.rp_LimitMin(output, ctypes.c_float(limit_min))
    if retval != 0:
        LOG.error("Failed to set minimum output voltage. Error code: %s", ERROR_CODES[retval])

@route("/_set_limit_max", method="POST")
def set_limit_max():
    """Handle POST request for setting maximum output value.

    Accepted POST parameters:
    :output: the output channel to adjust
    :limit_max: the value to set.
    """
    limit_max = request.params.get("limit_max", 0, type=float)
    output = request.params.get("output", 1, type=int)
    retval = RP_LIB.rp_LimitMax(output, ctypes.c_float(limit_max))
    if retval != 0:
        LOG.error("Failed to set maximum output voltage. Error code: %s", ERROR_CODES[retval])

@route("/_set_waveform", method="POST")
def set_waveform():
    """Handle POST request for setting waveform of the signal generator.

    Accepted POST parameters:
    :output: the output channel to adjust
    :waveform: number of the selected waveform. (0:SIN, 1:SQUARE, 2:TRIANGLE, 3:SAWU, 4:SAWD)
    """
    waveform = request.params.get("waveform", 0, type=int)
    output = request.params.get("output", 1, type=int)
    retval = RP_LIB.rp_GenWaveform(output, ctypes.c_int(waveform))
    if retval != 0:
        LOG.error("Failed to set waveform of the signal generator. Error code: %s",
                  ERROR_CODES[retval])

@route("/_set_sg_amp", method="POST")
def set_sg_amp():
    """Handle POST request for setting amplitude of the signal generator.

    Accepted POST parameters:
    :output: the output channel to adjust
    :amp: the value to set.
    """
    amp = request.params.get("amp", 0, type=float)
    output = request.params.get("output", 1, type=int)
    retval = RP_LIB.rp_GenAmp(output, ctypes.c_float(amp))
    if retval != 0:
        LOG.error("Failed to set signal generator amplitude. Error code: %s", ERROR_CODES[retval])

@route("/_set_sg_freq", method="POST")
def set_sg_freq():
    """Handle POST request for setting frequency of the signal generator.

    Accepted POST parameters:
    :output: the output channel to adjust
    :freq: the value to set.
    """
    freq = request.params.get("freq", 0, type=float)
    output = request.params.get("output", 1, type=int)
    retval = RP_LIB.rp_GenFreq(output, ctypes.c_float(freq))
    if retval != 0:
        LOG.error("Failed to set signal generator frequency. Error code: %s", ERROR_CODES[retval])

@route("/_set_sg_offset", method="POST")
def set_sg_offset():
    """Handle POST request for setting offset of the signal generator.

    Accepted POST parameters:
    :output: the output channel to adjust
    :offset: the value to set.
    """
    offset = request.params.get("offset", 0, type=float)
    output = request.params.get("output", 1, type=int)
    retval = RP_LIB.rp_GenOffset(output, ctypes.c_float(offset))
    if retval != 0:
        LOG.error("Failed to set signal generator offset. Error code: %s", ERROR_CODES[retval])

@route("/_set_sg_enabled", method="POST")
def set_sg_enabled():
    """Handle POST request for enabling the signal generator.

    Accepted POST parameters:
    :output: the output channel to adjust
    :sg_enabled: true if signal generator enabled, false if not
    """
    sg_enabled = request.params.get("sg_enabled", 0) == "true"
    output = request.params.get("output", 1, type=int)
    if sg_enabled:
        retval = RP_LIB.rp_GenOutEnable(output)
        if retval != 0:
            LOG.error("Failed to enable signal generator. Error code: %s", ERROR_CODES[retval])
    else:
        retval = RP_LIB.rp_GenOutDisable(output)
        if retval != 0:
            LOG.error("Failed to disable signal generator. Error code: %s", ERROR_CODES[retval])

@route("/_save_parameters", method="POST")
def save_parameters():
    """Handle POST request for saving parameters to SD card."""

    retval = RP_LIB.rp_SaveLockboxConfig()
    if retval != 0:
        LOG.error("Failed to save parameters. Error code: %s", ERROR_CODES[retval])

@route("/_load_parameters", method="POST")
def load_parameters():
    """Handle POST request for loading parameters to SD card."""

    retval = RP_LIB.rp_LoadLockboxConfig()
    if retval != 0:
        LOG.error("Failed to load parameters. Error code: %s", ERROR_CODES[retval])

@route("/_get_parameters")
def get_parameters():
    """Return a json string containing the current lockbox parameters."""

    setpoint = [0., 0., 0., 0.]
    kp_param = [0., .0, 0., 0.]
    ki_param = [0., 0., 0., 0.]
    kd_param = [0, 0, 0, 0]
    inverted = [False, False, False, False]
    hold = [False, False, False, False]
    int_reset = [False, False, False, False]
    int_auto = [False, False, False, False]
    relock_min = [0., 0., 0., 0.]
    relock_max = [0., 0., 0., 0.]
    relock_slew_rate = [0., 0., 0., 0.]
    relock_enabled = [False, False, False, False]
    relock_input = [0, 0, 0, 0]
    for i in range(4):
        setpoint[i] = ctypes.c_float()
        retval = RP_LIB.rp_PIDGetSetpoint(i, ctypes.byref(setpoint[i]))
        if retval != 0:
            LOG.error("Failed to get PID setpoint. Error code: %s", ERROR_CODES[retval])

        kp_param[i] = ctypes.c_float()
        retval = RP_LIB.rp_PIDGetKp(i, ctypes.byref(kp_param[i]))
        if retval != 0:
            LOG.error("Failed to get PID Kp parameter. Error code: %s", ERROR_CODES[retval])

        ki_param[i] = ctypes.c_float()
        retval = RP_LIB.rp_PIDGetKi(i, ctypes.byref(ki_param[i]))
        if retval != 0:
            LOG.error("Failed to get PID Ki parameter. Error code: %s", ERROR_CODES[retval])

        kd_param[i] = ctypes.c_int()
        retval = RP_LIB.rp_PIDGetKd(i, ctypes.byref(kd_param[i]))
        if retval != 0:
            LOG.error("Failed to get PID Kd parameter. Error code: %s", ERROR_CODES[retval])

        inverted[i] = ctypes.c_bool()
        retval = RP_LIB.rp_PIDGetInverted(i, ctypes.byref(inverted[i]))
        if retval != 0:
            LOG.error("Failed to get PID feedback sign. Error code: %s", ERROR_CODES[retval])

        hold[i] = ctypes.c_bool()
        retval = RP_LIB.rp_PIDGetHold(i, ctypes.byref(hold[i]))
        if retval != 0:
            LOG.error("Failed to get state of PID internal holding. Error code: %s",
                      ERROR_CODES[retval])

        int_reset[i] = ctypes.c_bool()
        retval = RP_LIB.rp_PIDGetIntReset(i, ctypes.byref(int_reset[i]))
        if retval != 0:
            LOG.error("Failed to get state of PID integrator reset. Error code: %s",
                      ERROR_CODES[retval])

        int_auto[i] = ctypes.c_bool()
        retval = RP_LIB.rp_PIDGetResetWhenRailed(i, ctypes.byref(int_auto[i]))
        if retval != 0:
            LOG.error("Failed to get state of PID automatical integrator reset. Error code: %s",
                      ERROR_CODES[retval])

        relock_min[i] = ctypes.c_float()
        retval = RP_LIB.rp_PIDGetRelockMinimum(i, ctypes.byref(relock_min[i]))
        if retval != 0:
            LOG.error("Failed to get PID minimum relock voltage. Error code: %s",
                      ERROR_CODES[retval])

        relock_max[i] = ctypes.c_float()
        retval = RP_LIB.rp_PIDGetRelockMaximum(i, ctypes.byref(relock_max[i]))
        if retval != 0:
            LOG.error("Failed to get PID maximum relock voltage. Error code: %s",
                      ERROR_CODES[retval])

        relock_slew_rate[i] = ctypes.c_float()
        retval = RP_LIB.rp_PIDGetRelockStepsize(i, ctypes.byref(relock_slew_rate[i]))
        if retval != 0:
            LOG.error("Failed to get PID relock slew rate. Error code: %s",
                      ERROR_CODES[retval])

        relock_enabled[i] = ctypes.c_bool()
        retval = RP_LIB.rp_PIDGetRelock(i, ctypes.byref(relock_enabled[i]))
        if retval != 0:
            LOG.error("Failed to get state of PID relock feature. Error code: %s",
                      ERROR_CODES[retval])

        relock_input[i] = ctypes.c_int()
        retval = RP_LIB.rp_PIDGetRelockInput(i, ctypes.byref(relock_input[i]))
        if retval != 0:
            LOG.error("Failed to get analog input of PID relock. Error code: %s",
                      ERROR_CODES[retval])

    limit_min_1 = ctypes.c_float()
    retval = RP_LIB.rp_LimitGetMin(0, ctypes.byref(limit_min_1))
    if retval != 0:
        LOG.error("Failed to get minimum output limit. Error code: %s", ERROR_CODES[retval])
    limit_min_2 = ctypes.c_float()
    retval = RP_LIB.rp_LimitGetMin(1, ctypes.byref(limit_min_2))
    if retval != 0:
        LOG.error("Failed to get minimum output limit. Error code: %s", ERROR_CODES[retval])

    limit_max_1 = ctypes.c_float()
    retval = RP_LIB.rp_LimitGetMax(0, ctypes.byref(limit_max_1))
    if retval != 0:
        LOG.error("Failed to get maximum output limit. Error code: %s", ERROR_CODES[retval])
    limit_max_2 = ctypes.c_float()
    retval = RP_LIB.rp_LimitGetMax(1, ctypes.byref(limit_max_2))
    if retval != 0:
        LOG.error("Failed to get maximum output limit. Error code: %s", ERROR_CODES[retval])

    sg_1_amp = ctypes.c_float()
    retval = RP_LIB.rp_GenGetAmp(0, ctypes.byref(sg_1_amp))
    if retval != 0:
        LOG.error("Failed to get signal generator amplitude. Error code: %s", ERROR_CODES[retval])
    sg_2_amp = ctypes.c_float()
    retval = RP_LIB.rp_GenGetAmp(1, ctypes.byref(sg_2_amp))
    if retval != 0:
        LOG.error("Failed to get signal generator amplitude. Error code: %s", ERROR_CODES[retval])

    sg_1_freq = ctypes.c_float()
    retval = RP_LIB.rp_GenGetFreq(0, ctypes.byref(sg_1_freq))
    if retval != 0:
        LOG.error("Failed to get signal generator frequency. Error code: %s", ERROR_CODES[retval])
    sg_2_freq = ctypes.c_float()
    retval = RP_LIB.rp_GenGetFreq(1, ctypes.byref(sg_2_freq))
    if retval != 0:
        LOG.error("Failed to get signal generator frequency. Error code: %s", ERROR_CODES[retval])

    sg_1_offset = ctypes.c_float()
    retval = RP_LIB.rp_GenGetOffset(0, ctypes.byref(sg_1_offset))
    if retval != 0:
        LOG.error("Failed to get signal generator offset. Error code: %s", ERROR_CODES[retval])
    sg_2_offset = ctypes.c_float()
    retval = RP_LIB.rp_GenGetOffset(1, ctypes.byref(sg_2_offset))
    if retval != 0:
        LOG.error("Failed to get signal generator offset. Error code: %s", ERROR_CODES[retval])

    sg_1_waveform = ctypes.c_int()
    retval = RP_LIB.rp_GenGetWaveform(0, ctypes.byref(sg_1_waveform))
    if retval != 0:
        LOG.error("Failed to get the waveform of the signal generator. Error code: %s",
                  ERROR_CODES[retval])
    sg_2_waveform = ctypes.c_int()
    retval = RP_LIB.rp_GenGetWaveform(1, ctypes.byref(sg_2_waveform))
    if retval != 0:
        LOG.error("Failed to get the waveform of the signal generator. Error code: %s",
                  ERROR_CODES[retval])

    sg_1_enabled = ctypes.c_bool()
    retval = RP_LIB.rp_GenOutIsEnabled(0, ctypes.byref(sg_1_enabled))
    if retval != 0:
        LOG.error("Failed to get if signal generator is enabled. Error code: %s",
                  ERROR_CODES[retval])
    sg_2_enabled = ctypes.c_bool()
    retval = RP_LIB.rp_GenOutIsEnabled(1, ctypes.byref(sg_2_enabled))
    if retval != 0:
        LOG.error("Failed to get if signal generator is enabled. Error code: %s",
                  ERROR_CODES[retval])

    parameters = {
        "pid_11_setpoint": setpoint[0].value,
        "pid_12_setpoint": setpoint[1].value,
        "pid_21_setpoint": setpoint[2].value,
        "pid_22_setpoint": setpoint[3].value,
        "pid_11_kp": kp_param[0].value,
        "pid_12_kp": kp_param[1].value,
        "pid_21_kp": kp_param[2].value,
        "pid_22_kp": kp_param[3].value,
        "pid_11_ki": ki_param[0].value,
        "pid_12_ki": ki_param[1].value,
        "pid_21_ki": ki_param[2].value,
        "pid_22_ki": ki_param[3].value,
        "pid_11_kd": kd_param[0].value,
        "pid_12_kd": kd_param[1].value,
        "pid_21_kd": kd_param[2].value,
        "pid_22_kd": kd_param[3].value,
        "pid_11_inverted": inverted[0].value,
        "pid_12_inverted": inverted[1].value,
        "pid_21_inverted": inverted[2].value,
        "pid_22_inverted": inverted[3].value,
        "pid_11_hold": hold[0].value,
        "pid_12_hold": hold[1].value,
        "pid_21_hold": hold[2].value,
        "pid_22_hold": hold[3].value,
        "pid_11_int_res": int_reset[0].value,
        "pid_12_int_res": int_reset[1].value,
        "pid_21_int_res": int_reset[2].value,
        "pid_22_int_res": int_reset[3].value,
        "pid_11_int_auto_reset": int_auto[0].value,
        "pid_12_int_auto_reset": int_auto[1].value,
        "pid_21_int_auto_reset": int_auto[2].value,
        "pid_22_int_auto_reset": int_auto[3].value,
        "pid_11_relock_min": relock_min[0].value,
        "pid_12_relock_min": relock_min[1].value,
        "pid_21_relock_min": relock_min[2].value,
        "pid_22_relock_min": relock_min[3].value,
        "pid_11_relock_max": relock_max[0].value,
        "pid_12_relock_max": relock_max[1].value,
        "pid_21_relock_max": relock_max[2].value,
        "pid_22_relock_max": relock_max[3].value,
        "pid_11_relock_slew_rate": relock_slew_rate[0].value,
        "pid_12_relock_slew_rate": relock_slew_rate[1].value,
        "pid_21_relock_slew_rate": relock_slew_rate[2].value,
        "pid_22_relock_slew_rate": relock_slew_rate[3].value,
        "pid_11_relock_enabled": relock_enabled[0].value,
        "pid_12_relock_enabled": relock_enabled[1].value,
        "pid_21_relock_enabled": relock_enabled[2].value,
        "pid_22_relock_enabled": relock_enabled[3].value,
        "pid_11_relock_input": relock_input[0].value,
        "pid_12_relock_input": relock_input[1].value,
        "pid_21_relock_input": relock_input[2].value,
        "pid_22_relock_input": relock_input[3].value,
        "limit_min_1": limit_min_1.value,
        "limit_min_2": limit_min_2.value,
        "limit_max_1": limit_max_1.value,
        "limit_max_2": limit_max_2.value,
        "sg_1_waveform": sg_1_waveform.value,
        "sg_2_waveform": sg_2_waveform.value,
        "sg_1_enabled": sg_1_enabled.value,
        "sg_2_enabled": sg_2_enabled.value,
        "sg_1_amp": sg_1_amp.value,
        "sg_2_amp": sg_2_amp.value,
        "sg_1_freq": sg_1_freq.value,
        "sg_2_freq": sg_2_freq.value,
        "sg_1_offset": sg_1_offset.value,
        "sg_2_offset": sg_2_offset.value
    }
    return json.dumps(parameters)

class MockRPLib():
    """Class that simulates the Red Pitaya lockbox library."""

    def rp_Init(self):
        LOG.debug("rp_Init called")
        return 0

    def rp_PIDSetSetpoint(self, pid, setpoint):
        LOG.debug("pid: %d\t setpoint: %f", pid, setpoint.value)
        return 0

    def rp_PIDSetKp(self, pid, kp):
        LOG.debug("pid: %d\t kp: %f", pid, kp.value)
        return 0

    def rp_PIDSetKi(self, pid, ki):
        LOG.debug("pid: %d\t ki: %f", pid, ki.value)
        return 0

    def rp_PIDSetKd(self, pid, kd):
        LOG.debug("pid: %d\t kd: %d", pid, kd.value)
        return 0

    def rp_PIDSetInverted(self, pid, inverted):
        LOG.debug("pid: %d\t inverted: %s", pid, inverted)
        return 0

    def rp_PIDSetHold(self, pid, hold):
        LOG.debug("pid: %d\t hold: %s", pid, hold)
        return 0

    def rp_PIDSetIntReset(self, pid, int_reset):
        LOG.debug("pid: %d\t int_reset: %s", pid, int_reset)
        return 0

    def rp_PIDSetResetWhenRailed(self, pid, int_auto):
        LOG.debug("pid: %d\t int_auto: %s", pid, int_auto)
        return 0

    def rp_PIDSetRelockMinimum(self, pid, relock_min):
        LOG.debug("pid: %d\t relock_min: %f", pid, relock_min.value)
        return 0

    def rp_PIDSetRelockMaximum(self, pid, relock_max):
        LOG.debug("pid: %d\t relock_max: %f", pid, relock_max.value)
        return 0

    def rp_PIDSetRelockStepsize(self, pid, relock_slew_rate):
        LOG.debug("pid: %d\t relock_slew_rate: %f", pid, relock_slew_rate.value)
        return 0

    def rp_PIDSetRelock(self, pid, relock_enabled):
        LOG.debug("pid: %d\t relock_enabled: %s", pid, relock_enabled)
        return 0

    def rp_PIDSetRelockInput(self, pid, ain):
        LOG.debug("pid: %d\t ain: %d", pid, ain.value)
        return 0

    def rp_LimitMin(self, output, limit_min):
        LOG.debug("output: %d\t limit_min: %f", output, limit_min.value)
        return 0

    def rp_LimitMax(self, output, limit_max):
        LOG.debug("output: %d\t limit_max: %f", output, limit_max.value)
        return 0

    def rp_GenWaveform(self, output, waveform):
        LOG.debug("output: %d\t waveform: %d", output, waveform.value)
        return 0

    def rp_GenAmp(self, output, amp):
        LOG.debug("output: %d\t amp: %f", output, amp.value)
        return 0

    def rp_GenFreq(self, output, freq):
        LOG.debug("output: %d\t freq: %f", output, freq.value)
        return 0

    def rp_GenOffset(self, output, offset):
        LOG.debug("output: %d\t offset: %f", output, offset.value)
        return 0

    def rp_GenOutEnable(self, output):
        LOG.debug("output %d signal generator enabled", output)
        return 0

    def rp_GenOutDisable(self, output):
        LOG.debug("output %d signal generator disabled", output)
        return 0

    def rp_SaveLockboxConfig(self):
        LOG.debug("Lockbox configuration saved")
        return 0

    def rp_LoadLockboxConfig(self):
        LOG.debug("Lockbox configuration loaded")
        return 0

    def rp_PIDGetSetpoint(self, pid, setpoint):
        LOG.debug("rp_PIDGetSetpoint called")
        setpoint._obj.value = 1.0
        return 0

    def rp_PIDGetKp(self, pid, kp):
        LOG.debug("rp_PIDGetKp called")
        kp._obj.value = 0.1
        return 0

    def rp_PIDGetKi(self, pid, ki):
        LOG.debug("rp_PIDGetKi called")
        ki._obj.value = 10.0
        return 0

    def rp_PIDGetKd(self, pid, kd):
        LOG.debug("rp_PIDGetKd called")
        kd._obj.value = 1
        return 0

    def rp_PIDGetInverted(self, pid, inverted):
        LOG.debug("rp_PIDGetInverted called")
        inverted._obj.value = True
        return 0

    def rp_PIDGetHold(self, pid, hold):
        LOG.debug("rp_PIDGetHold called")
        hold._obj.value = True
        return 0

    def rp_PIDGetIntReset(self, pid, int_reset):
        LOG.debug("rp_PIDGetIntReset called")
        int_reset._obj.value = True
        return 0

    def rp_PIDGetResetWhenRailed(self, pid, int_auto):
        LOG.debug("rp_PIDGetResetWhenRailed called")
        int_auto._obj.value = True
        return 0

    def rp_PIDGetRelockMinimum(self, pid, relock_min):
        LOG.debug("rp_PIDGetRelockMinimum called")
        relock_min._obj.value = 0.0
        return 0

    def rp_PIDGetRelockMaximum(self, pid, relock_max):
        LOG.debug("rp_PIDGetRelockMaximum called")
        relock_max._obj.value = 7.0
        return 0

    def rp_PIDGetRelockStepsize(self, pid, relock_slew_rate):
        LOG.debug("rp_PIDGetRelockStepsize called")
        relock_slew_rate._obj.value = 500.0
        return 0

    def rp_PIDGetRelock(self, pid, relock_enabled):
        LOG.debug("rp_PIDGetRelock called")
        relock_enabled._obj.value = True
        return 0

    def rp_PIDGetRelockInput(self, pid, relock_input):
        LOG.debug("rp_PIDGetRelockInput called")
        relock_input._obj.value = 5
        return 0

    def rp_LimitGetMin(self, pid, limit_min):
        LOG.debug("rp_LimitGetMin called")
        limit_min._obj.value = -1.0
        return 0

    def rp_LimitGetMax(self, pid, limit_max):
        LOG.debug("rp_LimitGetMax called")
        limit_max._obj.value = 1.0
        return 0

    def rp_GenGetAmp(self, pid, amp):
        LOG.debug("rp_GenGetAmp called")
        amp._obj.value = 1.0
        return 0

    def rp_GenGetFreq(self, pid, freq):
        LOG.debug("rp_GenGetFreq called")
        freq._obj.value = 1000.0
        return 0

    def rp_GenGetOffset(self, pid, offset):
        LOG.debug("rp_GenGetOffset called")
        offset._obj.value = 0.0
        return 0

    def rp_GenGetWaveform(self, pid, waveform):
        LOG.debug("rp_GenGetWaveform called")
        waveform._obj.value = 0
        return 0

    def rp_GenOutIsEnabled(self, pid, sg_enabled):
        LOG.debug("rp_GenOutIsEnabled called")
        sg_enabled._obj.value = True
        return 0

try:
    RP_LIB = ctypes.CDLL("/opt/redpitaya/lib/liblockbox.so")
except OSError as err:
    LOG.setLevel(logging.DEBUG)
    LOG.error("Failed to load lockbox library. Error: %s", err)
    RP_LIB = MockRPLib()
else:
    init_rp_library()

run(host="0.0.0.0", port=80, quiet=True)
