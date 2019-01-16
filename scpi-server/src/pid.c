/**
 * Copyright (c) 2018, Fabian Schmid
 *
 * All rights reserved.
 *
 * PID SCPI commands implementation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pid.h"
#include "../../api/src/pid.h"

#include "common.h"
#include "scpi/parser.h"
#include "scpi/units.h"

scpi_result_t RP_SaveLockboxConfig(scpi_t *context) {
    int result = rp_SaveLockboxConfig();
    if (result != RP_OK) {
        RP_LOG("LOCKbox:CONFig:SAVE Failed to save config to file: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }
    return SCPI_RES_OK;
}

scpi_result_t RP_LoadLockboxConfig(scpi_t *context) {
    int result = rp_LoadLockboxConfig();
    if (result != RP_OK) {
        RP_LOG("LOCKbox:CONFig:LOAD Failed to load config from file: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }
    return SCPI_RES_OK;
}

/* Parse pid index from SCPI command */
static int RP_ParsePIDArgv(scpi_t *context, rp_pid_t *pid) {
    int32_t inout[2]; // First int: input index (1-2), second int: output index (1-2)

    SCPI_CommandNumbers(context, inout, 2, 1);
    if((inout[0] > 2) || (inout[0] < 1)) {
        RP_LOG(LOG_ERR, "ERROR: Invalid input nr: %d\n", inout[0]);
        return RP_EOOR;
    }
    if((inout[1] > 2) || (inout[1] < 1)) {
        RP_LOG(LOG_ERR, "ERROR: Invalid output nr: %d\n", inout[1]);
        return RP_EOOR;
    }
    if((inout[0] == 1) && (inout[1] == 1))
        *pid = RP_PID_11;
    if((inout[0] == 2) && (inout[1] == 1))
        *pid = RP_PID_12;
    if((inout[0] == 1) && (inout[1] == 2))
        *pid = RP_PID_21;
    if((inout[0] == 2) && (inout[1] == 2))
        *pid = RP_PID_22;

    return RP_OK;
}

scpi_result_t RP_PIDSetpoint(scpi_t *context) {
    int result;
    scpi_number_t setpoint;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:SETPoint Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    /* Parse first parameter (setpoint) */
    if(!SCPI_ParamNumber(context, scpi_special_numbers_def, &setpoint, true)) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:SETPoint Failed to parse first parameter.\n");
        return SCPI_RES_ERR;
    }

    result = rp_PIDSetSetpoint(pid, setpoint.value);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:SETPoint Failed to set setpoint parameter: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:SETPoint Successfully set setpoint.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDSetpointQ(scpi_t *context) {
    int result;
    float setpoint;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:SETPoint? Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    result = rp_PIDGetSetpoint(pid, &setpoint);
    if(result != RP_OK){
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:SETPoint? Failed to get setpoint: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    SCPI_ResultDouble(context, setpoint);

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:SETPoint? Successfully returned setpoint value to client.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDKp(scpi_t *context) {
    int result;
    scpi_number_t kp;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:KP Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    /* Parse first parameter (Kp) */
    if(!SCPI_ParamNumber(context, scpi_special_numbers_def, &kp, true)) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:KP Failed to parse first parameter.\n");
        return SCPI_RES_ERR;
    }

    result = rp_PIDSetKp(pid, kp.value);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:KP Failed to set Kp parameter: %s", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:KP Successfully set Kp.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDKpQ(scpi_t *context) {
    int result;
    float kp;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:KP? Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    result = rp_PIDGetKp(pid, &kp);
    if(result != RP_OK){
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:KP? Failed to get Kp: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    SCPI_ResultDouble(context, kp);

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:KP? Successfully returned Kp value to client.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDKi(scpi_t *context) {
    int result;
    scpi_number_t ki;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:KI Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    /* Parse first parameter (Ki) */
    if(!SCPI_ParamNumber(context, scpi_special_numbers_def, &ki, true)) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:KI Failed to parse first parameter.\n");
        return SCPI_RES_ERR;
    }

    result = rp_PIDSetKi(pid, ki.value);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:KI Failed to set Ki parameter: %s", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:KI Successfully set Ki.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDKiQ(scpi_t *context) {
    int result;
    float ki;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:KI? Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    result = rp_PIDGetKi(pid, &ki);
    if(result != RP_OK){
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:KI? Failed to get Ki: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    SCPI_ResultDouble(context, ki);

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:KI? Successfully returned Ki value to client.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDKd(scpi_t *context) {
    int result;
    uint32_t kd;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:KD Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    /* Parse first parameter (KD) */
    if(!SCPI_ParamUInt32(context, &kd, true)) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:KD Failed to parse first parameter.\n");
        return SCPI_RES_ERR;
    }

    result = rp_PIDSetKd(pid, kd);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:KD Failed to set Kd parameter: %s", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:KD Successfully set Kd.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDKdQ(scpi_t *context) {
    int result;
    uint32_t kd;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:KD? Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    result = rp_PIDGetKd(pid, &kd);
    if(result != RP_OK){
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:KD? Failed to get Kd: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    SCPI_ResultUInt32Base(context, kd, 10);

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:KD? Successfully returned Kd value to client.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDIntReset(scpi_t *context) {
    int result;
    scpi_bool_t enable;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:INTegrator:RESet Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    /* Parse first parameter PID reset (ON,OFF) */
    if(!SCPI_ParamBool(context, &enable, true)) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:INTegrator:RESet Failed to parse first parameter.\n");
        return SCPI_RES_ERR;
    }

    result = rp_PIDSetIntReset(pid, enable);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:INTegrator:RESet Failed to set integrator reset: %s", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:INTegrator:RESet Successfully set integrator reset.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDIntResetQ(scpi_t *context) {
    int result;
    bool enabled;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:INTegrator:RESet? Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    result = rp_PIDGetIntReset(pid, &enabled);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:INTegrator:RESet? Failed to get integrator reset: %s", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    // Return result as string
    SCPI_ResultMnemonic(context, enabled ? "ON": "OFF");

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:INTegrator:RESet? Successfully returned integrator reset.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDInverted(scpi_t *context) {
    int result;
    scpi_bool_t inverted;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:INVerted Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    /* Parse first parameter PID inverted (ON,OFF) */
    if(!SCPI_ParamBool(context, &inverted, true)) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:INVerted Failed to parse first parameter.\n");
        return SCPI_RES_ERR;
    }

    result = rp_PIDSetInverted(pid, inverted);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:INVerted Failed to set feedback sign: %s", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:INVerted Successfully set feedback sign.\n");
    return SCPI_RES_OK;
}
scpi_result_t RP_PIDInvertedQ(scpi_t *context) {
    int result;
    bool inverted;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:INVerted? Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    result = rp_PIDGetInverted(pid, &inverted);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:INVerted? Failed to get feedback sign: %s", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    // Return result as string
    SCPI_ResultMnemonic(context, inverted ? "ON": "OFF");

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:INVerted? Successfully returned feedback sign.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDHold(scpi_t *context) {
    int result;
    scpi_bool_t enabled;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:HOLD Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    /* Parse first parameter PID hold enabled (ON,OFF) */
    if(!SCPI_ParamBool(context, &enabled, true)) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:HOLD Failed to parse first parameter.\n");
        return SCPI_RES_ERR;
    }

    result = rp_PIDSetHold(pid, enabled);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:HOLD Failed to set PID hold: %s", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:HOLD Successfully set PID hold.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDHoldQ(scpi_t *context) {
    int result;
    bool enabled;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:HOLD? Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    result = rp_PIDGetHold(pid, &enabled);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:HOLD? Failed to get PID hold state: %s", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    // Return result as string
    SCPI_ResultMnemonic(context, enabled ? "ON": "OFF");

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:HOLD? Successfully returned PID hold state.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDResetWhenRailed(scpi_t *context) {
    int result;
    scpi_bool_t enabled;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:INTegrator:AUTOreset Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    /* Parse first parameter integrator auto reset (ON,OFF) */
    if(!SCPI_ParamBool(context, &enabled, true)) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:INTegrator:AUTOreset Failed to parse first parameter.\n");
        return SCPI_RES_ERR;
    }

    result = rp_PIDSetResetWhenRailed(pid, enabled);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:INTegrator:AUTOreset Failed to set integrator auto reset: %s", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:INTegrator:AUTOreset Successfully set integrator auto reset.\n");
    return SCPI_RES_OK;
}
scpi_result_t RP_PIDResetWhenRailedQ(scpi_t *context) {
    int result;
    bool enabled;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:INTegrator:AUTOreset? Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    result = rp_PIDGetResetWhenRailed(pid, &enabled);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:INTegrator:AUTOreset Failed to integrator auto reset state: %s", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    // Return result as string
    SCPI_ResultMnemonic(context, enabled ? "ON": "OFF");

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:INTegrator:AUTOreset Successfully returned integrator auto reset state.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDRelock(scpi_t *context) {
    int result;
    scpi_bool_t enabled;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    /* Parse first parameter relock enabled (ON,OFF) */
    if(!SCPI_ParamBool(context, &enabled, true)) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock Failed to parse first parameter.\n");
        return SCPI_RES_ERR;
    }

    result = rp_PIDSetRelock(pid, enabled);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock Failed to set relock state: %s", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:RELock Successfully set relock state.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDRelockQ(scpi_t *context) {
    int result;
    bool enabled;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock? Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    result = rp_PIDGetRelock(pid, &enabled);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock? Failed to get relock state: %s", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    // Return result as string
    SCPI_ResultMnemonic(context, enabled ? "ON": "OFF");

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:RELock? Successfully returned relock state.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDRelockStepsize(scpi_t *context) {
    int result;
    scpi_number_t stepsize;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:STEPsize Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    /* Parse first parameter (stepsize) */
    if(!SCPI_ParamNumber(context, scpi_special_numbers_def, &stepsize, true)) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:STEPsize Failed to parse first parameter.\n");
        return SCPI_RES_ERR;
    }

    result = rp_PIDSetRelockStepsize(pid, stepsize.value);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:STEPsize Failed to set stepsize: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:RELock:STEPsize Successfully set stepsize.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDRelockStepsizeQ(scpi_t *context) {
    int result;
    float stepsize;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:STEPsize? Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    result = rp_PIDGetRelockStepsize(pid, &stepsize);
    if(result != RP_OK){
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:STEPsize? Failed to get stepsize: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    SCPI_ResultDouble(context, stepsize);

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:RELock:STEPsize? Successfully returned stepsize value to client.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDRelockMin(scpi_t *context) {
    int result;
    scpi_number_t minimum;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:MIN Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    /* Parse first parameter (minimum) */
    if(!SCPI_ParamNumber(context, scpi_special_numbers_def, &minimum, true)) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:MIN Failed to parse first parameter.\n");
        return SCPI_RES_ERR;
    }

    result = rp_PIDSetRelockMinimum(pid, minimum.value);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:MIN Failed to set minimum value: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:RELock:MIN Successfully set minimum value.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDRelockMinQ(scpi_t *context) {
    int result;
    float minimum;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:MIN? Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    result = rp_PIDGetRelockMinimum(pid, &minimum);
    if(result != RP_OK){
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:MIN? Failed to get minimum value: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    SCPI_ResultDouble(context, minimum);

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:RELock:MIN? Successfully returned minimum value to client.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDRelockMax(scpi_t *context) {
    int result;
    scpi_number_t maximum;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:MAX Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    /* Parse first parameter (maximum) */
    if(!SCPI_ParamNumber(context, scpi_special_numbers_def, &maximum, true)) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:MAX Failed to parse first parameter.\n");
        return SCPI_RES_ERR;
    }

    result = rp_PIDSetRelockMaximum(pid, maximum.value);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:MAX Failed to set maximum value: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:RELock:MAX Successfully set maximum value.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_PIDRelockMaxQ(scpi_t *context) {
    int result;
    float maximum;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:MAX? Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    result = rp_PIDGetRelockMaximum(pid, &maximum);
    if(result != RP_OK){
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:MAX? Failed to get maximum value: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    SCPI_ResultDouble(context, maximum);

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:RELock:MAX? Successfully returned maximum value to client.\n");
    return SCPI_RES_OK;
}

/* AIN choice def */
const scpi_choice_def_t scpi_RpAinput[] = {
    {"AIN0",  4},  //!< Analog input 0
    {"AIN1",  5},  //!< Analog input 1
    {"AIN2",  6},  //!< Analog input 2
    {"AIN3",  7},  //!< Analog input 3
    SCPI_CHOICE_LIST_END
};

scpi_result_t RP_PIDRelockInput(scpi_t *context) {
    int result;
    int32_t choice;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:INPut Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }
    /* Read first parameter - AIN */
    if (!SCPI_ParamChoice(context, scpi_RpAinput, &choice, true)) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:INPut is missing first parameter.\n");
        return SCPI_RES_ERR;
    }
    rp_apin_t pin = choice;

    result = rp_PIDSetRelockInput(pid, pin);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:INPut Failed to set input pin: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:RELock:INPut Successfully set input pin.\n");
    return SCPI_RES_OK;
}


scpi_result_t RP_PIDRelockInputQ(scpi_t *context) {
    int result;
    const char *pin_name;
    rp_apin_t pin;
    rp_pid_t pid;

    /* Parse PID index */
    result = RP_ParsePIDArgv(context, &pid);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:INPut? Failed to parse input/output choice: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    result = rp_PIDGetRelockInput(pid, &pin);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*PID:IN#:OUT#:RELock:INPut? Failed to get input pin: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    if(!SCPI_ChoiceToName(scpi_RpAinput, pin, &pin_name)) {
        RP_LOG(LOG_ERR,"*PID:IN#:OUT#:RELock:INPut? Failed to get input pin name.\n");
        return SCPI_RES_ERR;
    }

    SCPI_ResultMnemonic(context, pin_name);

    RP_LOG(LOG_INFO, "*PID:IN#:OUT#:RELock:INPut? Successfully returned input pin value to client.\n");
    return SCPI_RES_OK;
}
