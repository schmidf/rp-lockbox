/**
 * Copyright (c) 2018, Fabian Schmid
 *
 * All rights reserved.
 *
 * $Id: $
 *
 * @brief Red Pitaya library pid module implementation
 *
 * @Author Red Pitaya
 *
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in C programming language.
 * Please visit http://en.wikipedia.org/wiki/C_(programming_language)
 * for more details on the language used herein.
 */

#include <stdbool.h>
#include <math.h>
#include "common.h"
#include "pid.h"
#include "calib.h"

// The FPGA register structure for the PID
static volatile pid_control_t *pid_reg = NULL;


/**
 * general
 */

int pid_Init()
{
    cmn_Map(PID_BASE_SIZE, PID_BASE_ADDR, (void**)&pid_reg);
    return RP_OK;
}

int pid_Release()
{
    cmn_Unmap(PID_BASE_SIZE, (void**)&pid_reg);
    return RP_OK;
}

/**
 * PID parameters
 */
int pid_SetPIDSetpoint(rp_pid_t pid, float setpoint)
{
    rp_calib_params_t calib = calib_GetParams();
    uint32_t setpoint_counts;

    if(pid == RP_PID_11 || pid == RP_PID_21)  // Input Channel A
        setpoint_counts = cmn_CnvVToCnt(DATA_BIT_LENGTH, setpoint, SETPOINT_MAX, false,
            calib.fe_ch1_fs_g_hi, calib.fe_ch1_hi_offs, 0);
    else if (pid == RP_PID_22 || pid == RP_PID_12)  // Input Channel B
        setpoint_counts = cmn_CnvVToCnt(DATA_BIT_LENGTH, setpoint, SETPOINT_MAX, false,
            calib.fe_ch2_fs_g_hi, calib.fe_ch2_hi_offs, 0);
    else
        return RP_EPN;

    switch(pid) {
        case RP_PID_11: return cmn_SetValue(&pid_reg->pid11_setpoint, setpoint_counts, PID_SETPOINT_MASK);
        case RP_PID_12: return cmn_SetValue(&pid_reg->pid12_setpoint, setpoint_counts, PID_SETPOINT_MASK);
        case RP_PID_21: return cmn_SetValue(&pid_reg->pid21_setpoint, setpoint_counts, PID_SETPOINT_MASK);
        case RP_PID_22: return cmn_SetValue(&pid_reg->pid22_setpoint, setpoint_counts, PID_SETPOINT_MASK);
        default: return RP_EPN;
    }
}

int pid_GetPIDSetpoint(rp_pid_t pid, float *setpoint)
{
    rp_calib_params_t calib = calib_GetParams();
    uint32_t setpoint_counts;

    switch(pid) {
        case RP_PID_11:
            cmn_GetValue(&pid_reg->pid11_setpoint, &setpoint_counts, PID_SETPOINT_MASK);
            break;
        case RP_PID_12:
            cmn_GetValue(&pid_reg->pid12_setpoint, &setpoint_counts, PID_SETPOINT_MASK);
            break;
        case RP_PID_21:
            cmn_GetValue(&pid_reg->pid21_setpoint, &setpoint_counts, PID_SETPOINT_MASK);
            break;
        case RP_PID_22:
            cmn_GetValue(&pid_reg->pid22_setpoint, &setpoint_counts, PID_SETPOINT_MASK);
            break;
        default: return RP_EPN;
    }

    if(pid == RP_PID_11 || pid == RP_PID_21) { // Input Channel A
        *setpoint = cmn_CnvCntToV(DATA_BIT_LENGTH, setpoint_counts, SETPOINT_MAX,
            calib.fe_ch1_fs_g_hi, calib.fe_ch1_hi_offs, 0);
        return RP_OK;
    }
    else if (pid == RP_PID_22 || pid == RP_PID_12) { // Input Channel B
        *setpoint = cmn_CnvCntToV(DATA_BIT_LENGTH, setpoint_counts, SETPOINT_MAX,
            calib.fe_ch2_fs_g_hi, calib.fe_ch2_hi_offs, 0);
        return RP_OK;
    }
    else
        return RP_EPN;
}

int pid_SetPIDKp(rp_pid_t pid, float kp)
{
    uint32_t kp_integer;

    if(kp < 0) {
        return RP_EIPV;
    }
    
    kp_integer = (int)round(kp * (1 << PID_PSR));
    if(kp_integer > PID_KP_MASK)  // check for integer overflow
        kp_integer = PID_KP_MASK;

    switch(pid) {
        case RP_PID_11: return cmn_SetValue(&pid_reg->pid11_Kp, kp_integer, PID_KP_MASK);
        case RP_PID_12: return cmn_SetValue(&pid_reg->pid12_Kp, kp_integer, PID_KP_MASK);
        case RP_PID_21: return cmn_SetValue(&pid_reg->pid21_Kp, kp_integer, PID_KP_MASK);
        case RP_PID_22: return cmn_SetValue(&pid_reg->pid22_Kp, kp_integer, PID_KP_MASK);
        default: return RP_EPN;
    }
}

int pid_GetPIDKp(rp_pid_t pid, float *kp)
{
    uint32_t kp_integer;
    switch(pid) {
        case RP_PID_11:
            cmn_GetValue(&pid_reg->pid11_Kp, &kp_integer, PID_KP_MASK);
            break;
        case RP_PID_12:
            cmn_GetValue(&pid_reg->pid12_Kp, &kp_integer, PID_KP_MASK);
            break;
        case RP_PID_21:
            cmn_GetValue(&pid_reg->pid21_Kp, &kp_integer, PID_KP_MASK);
            break;
        case RP_PID_22:
            cmn_GetValue(&pid_reg->pid22_Kp, &kp_integer, PID_KP_MASK);
            break;
        default: return RP_EPN;
    }

    *kp = (float)kp_integer/(1 << PID_PSR);
    return RP_OK;
}

int pid_SetPIDKi(rp_pid_t pid, float ki)
{
    uint32_t ki_integer;

    if(ki < 0) {
        return RP_EIPV;
    }

    ki_integer = (int)round(ki * (1 << PID_ISR) * PID_TIMESTEP);
    if(ki_integer > PID_KI_MASK) // check for integer overflow
        ki_integer = PID_KI_MASK;

    switch(pid) {
        case RP_PID_11: return cmn_SetValue(&pid_reg->pid11_Ki, ki_integer, PID_KI_MASK);
        case RP_PID_12: return cmn_SetValue(&pid_reg->pid12_Ki, ki_integer, PID_KI_MASK);
        case RP_PID_21: return cmn_SetValue(&pid_reg->pid21_Ki, ki_integer, PID_KI_MASK);
        case RP_PID_22: return cmn_SetValue(&pid_reg->pid22_Ki, ki_integer, PID_KI_MASK);
        default: return RP_EPN;
    }
}

int pid_GetPIDKi(rp_pid_t pid, float *ki)
{
    uint32_t ki_integer;
    switch(pid) {
        case RP_PID_11:
            cmn_GetValue(&pid_reg->pid11_Ki, &ki_integer, PID_KI_MASK);
            break;
        case RP_PID_12:
            cmn_GetValue(&pid_reg->pid12_Ki, &ki_integer, PID_KI_MASK);
            break;
        case RP_PID_21:
            cmn_GetValue(&pid_reg->pid21_Ki, &ki_integer, PID_KI_MASK);
            break;
        case RP_PID_22:
            cmn_GetValue(&pid_reg->pid22_Ki, &ki_integer, PID_KI_MASK);
            break;
        default: return RP_EPN;
    }

    *ki = (float)ki_integer/(PID_TIMESTEP * (1 << PID_ISR));
    return RP_OK;
}

int pid_SetPIDKd(rp_pid_t pid, uint32_t kd)
{
    switch(pid) {
        case RP_PID_11: return cmn_SetValue(&pid_reg->pid11_Kd, kd, PID_KD_MASK);
        case RP_PID_12: return cmn_SetValue(&pid_reg->pid12_Kd, kd, PID_KD_MASK);
        case RP_PID_21: return cmn_SetValue(&pid_reg->pid21_Kd, kd, PID_KD_MASK);
        case RP_PID_22: return cmn_SetValue(&pid_reg->pid22_Kd, kd, PID_KD_MASK);
        default: return RP_EPN;
    }
}

int pid_GetPIDKd(rp_pid_t pid, uint32_t *kd)
{
    switch(pid) {
        case RP_PID_11: return cmn_GetValue(&pid_reg->pid11_Kd, kd, PID_KD_MASK);
        case RP_PID_12: return cmn_GetValue(&pid_reg->pid12_Kd, kd, PID_KD_MASK);
        case RP_PID_21: return cmn_GetValue(&pid_reg->pid21_Kd, kd, PID_KD_MASK);
        case RP_PID_22: return cmn_GetValue(&pid_reg->pid22_Kd, kd, PID_KD_MASK);
        default: return RP_EPN;
    }
}

int pid_SetPIDIntReset(rp_pid_t pid, bool enable) {
    if(enable) {
        switch(pid) {
            case RP_PID_11: return cmn_SetBits(&pid_reg->conf, 0x1, PID_CONF_MASK);
            case RP_PID_12: return cmn_SetBits(&pid_reg->conf, 0x1 << 1, PID_CONF_MASK);
            case RP_PID_21: return cmn_SetBits(&pid_reg->conf, 0x1 << 2, PID_CONF_MASK);
            case RP_PID_22: return cmn_SetBits(&pid_reg->conf, 0x1 << 3, PID_CONF_MASK);
            default: return RP_EPN;
        }
    }
    else {
        switch(pid) {
            case RP_PID_11: return cmn_UnsetBits(&pid_reg->conf, 0x1, PID_CONF_MASK);
            case RP_PID_12: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 1, PID_CONF_MASK);
            case RP_PID_21: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 2, PID_CONF_MASK);
            case RP_PID_22: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 3, PID_CONF_MASK);
            default: return RP_EPN;
        }
    }
}
int pid_GetPIDIntReset(rp_pid_t pid, bool *enabled) {
    switch(pid) {
        case RP_PID_11: return cmn_AreBitsSet(pid_reg->conf, 0x1, PID_CONF_MASK, enabled);
        case RP_PID_12: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 1, PID_CONF_MASK, enabled);
        case RP_PID_21: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 2, PID_CONF_MASK, enabled);
        case RP_PID_22: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 3, PID_CONF_MASK, enabled);
        default: return RP_EPN;
    }
}

int pid_SetPIDInverted(rp_pid_t pid, bool inverted) {
    if(inverted) {
        switch(pid) {
            case RP_PID_11: return cmn_SetBits(&pid_reg->conf, 0x1 << 4, PID_CONF_MASK);
            case RP_PID_12: return cmn_SetBits(&pid_reg->conf, 0x1 << 5, PID_CONF_MASK);
            case RP_PID_21: return cmn_SetBits(&pid_reg->conf, 0x1 << 6, PID_CONF_MASK);
            case RP_PID_22: return cmn_SetBits(&pid_reg->conf, 0x1 << 7, PID_CONF_MASK);
            default: return RP_EPN;
        }
    }
    else {
        switch(pid) {
            case RP_PID_11: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 4, PID_CONF_MASK);
            case RP_PID_12: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 5, PID_CONF_MASK);
            case RP_PID_21: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 6, PID_CONF_MASK);
            case RP_PID_22: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 7, PID_CONF_MASK);
            default: return RP_EPN;
        }
    }
}

int pid_GetPIDInverted(rp_pid_t pid, bool *inverted) {
    switch(pid) {
        case RP_PID_11: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 4, PID_CONF_MASK, inverted);
        case RP_PID_12: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 5, PID_CONF_MASK, inverted);
        case RP_PID_21: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 6, PID_CONF_MASK, inverted);
        case RP_PID_22: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 7, PID_CONF_MASK, inverted);
        default: return RP_EPN;
    }
}

int pid_SetResetWhenRailed(rp_pid_t pid, bool enable) {
    if(enable) {
        switch(pid) {
            case RP_PID_11: return cmn_SetBits(&pid_reg->conf, 0x1 << 8, PID_CONF_MASK);
            case RP_PID_12: return cmn_SetBits(&pid_reg->conf, 0x1 << 9, PID_CONF_MASK);
            case RP_PID_21: return cmn_SetBits(&pid_reg->conf, 0x1 << 10, PID_CONF_MASK);
            case RP_PID_22: return cmn_SetBits(&pid_reg->conf, 0x1 << 11, PID_CONF_MASK);
            default: return RP_EPN;
        }
    }
    else {
        switch(pid) {
            case RP_PID_11: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 8, PID_CONF_MASK);
            case RP_PID_12: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 9, PID_CONF_MASK);
            case RP_PID_21: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 10, PID_CONF_MASK);
            case RP_PID_22: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 11, PID_CONF_MASK);
            default: return RP_EPN;
        }
    }
}
int pid_GetResetWhenRailed(rp_pid_t pid, bool *enabled) {
    switch(pid) {
        case RP_PID_11: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 8, PID_CONF_MASK, enabled);
        case RP_PID_12: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 9, PID_CONF_MASK, enabled);
        case RP_PID_21: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 10, PID_CONF_MASK, enabled);
        case RP_PID_22: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 11, PID_CONF_MASK, enabled);
        default: return RP_EPN;
    }
}

int pid_SetIntegratorHold(rp_pid_t pid, bool enable) {
    if(enable) {
        switch(pid) {
            case RP_PID_11: return cmn_SetBits(&pid_reg->conf, 0x1 << 12, PID_CONF_MASK);
            case RP_PID_12: return cmn_SetBits(&pid_reg->conf, 0x1 << 13, PID_CONF_MASK);
            case RP_PID_21: return cmn_SetBits(&pid_reg->conf, 0x1 << 14, PID_CONF_MASK);
            case RP_PID_22: return cmn_SetBits(&pid_reg->conf, 0x1 << 15, PID_CONF_MASK);
            default: return RP_EPN;
        }
    }
    else {
        switch(pid) {
            case RP_PID_11: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 12, PID_CONF_MASK);
            case RP_PID_12: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 13, PID_CONF_MASK);
            case RP_PID_21: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 14, PID_CONF_MASK);
            case RP_PID_22: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 15, PID_CONF_MASK);
            default: return RP_EPN;
        }
    }
}

int pid_GetIntegratorHold(rp_pid_t pid, bool *enabled) {
    switch(pid) {
        case RP_PID_11: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 12, PID_CONF_MASK, enabled);
        case RP_PID_12: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 13, PID_CONF_MASK, enabled);
        case RP_PID_21: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 14, PID_CONF_MASK, enabled);
        case RP_PID_22: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 15, PID_CONF_MASK, enabled);
        default: return RP_EPN;
    }
}

int pid_SetPIDRelock(rp_pid_t pid, bool enable) {
    if(enable) {
        switch(pid) {
            case RP_PID_11: return cmn_SetBits(&pid_reg->conf, 0x1 << 16, PID_CONF_MASK);
            case RP_PID_12: return cmn_SetBits(&pid_reg->conf, 0x1 << 17, PID_CONF_MASK);
            case RP_PID_21: return cmn_SetBits(&pid_reg->conf, 0x1 << 18, PID_CONF_MASK);
            case RP_PID_22: return cmn_SetBits(&pid_reg->conf, 0x1 << 19, PID_CONF_MASK);
            default: return RP_EPN;
        }
    }
    else {
        switch(pid) {
            case RP_PID_11: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 16, PID_CONF_MASK);
            case RP_PID_12: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 17, PID_CONF_MASK);
            case RP_PID_21: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 18, PID_CONF_MASK);
            case RP_PID_22: return cmn_UnsetBits(&pid_reg->conf, 0x1 << 19, PID_CONF_MASK);
            default: return RP_EPN;
        }
    }
}

int pid_GetPIDRelock(rp_pid_t pid, bool *enabled) {
    switch(pid) {
        case RP_PID_11: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 16, PID_CONF_MASK, enabled);
        case RP_PID_12: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 17, PID_CONF_MASK, enabled);
        case RP_PID_21: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 18, PID_CONF_MASK, enabled);
        case RP_PID_22: return cmn_AreBitsSet(pid_reg->conf, 0x1 << 19, PID_CONF_MASK, enabled);
        default: return RP_EPN;
    }
}

int pid_SetRelockStepsize(rp_pid_t pid, float stepsize) {
    uint32_t stepsize_integer;

    if(stepsize < 0)
        return RP_EIPV;

    stepsize_integer = (int)round(stepsize * (1 << PID_STEPSR) * PID_TIMESTEP/PID_DACCOUNT);
    if(stepsize_integer > PID_STEPSIZE_MASK) // check for integer overflow
        stepsize_integer = PID_STEPSIZE_MASK;

    switch (pid) {
        case RP_PID_11: return cmn_SetValue(&pid_reg->relock11_stepsize,
                                            stepsize_integer, PID_STEPSIZE_MASK);
        case RP_PID_12: return cmn_SetValue(&pid_reg->relock12_stepsize,
                                            stepsize_integer, PID_STEPSIZE_MASK);
        case RP_PID_21: return cmn_SetValue(&pid_reg->relock21_stepsize,
                                            stepsize_integer, PID_STEPSIZE_MASK);
        case RP_PID_22: return cmn_SetValue(&pid_reg->relock22_stepsize,
                                            stepsize_integer, PID_STEPSIZE_MASK);
        default: return RP_EPN;
    }
}

int pid_GetRelockStepsize(rp_pid_t pid, float *stepsize) {
    uint32_t stepsize_integer;
    switch(pid) {
        case RP_PID_11:
            cmn_GetValue(&pid_reg->relock11_stepsize, &stepsize_integer, PID_STEPSIZE_MASK);
            break;
        case RP_PID_12:
            cmn_GetValue(&pid_reg->relock12_stepsize, &stepsize_integer, PID_STEPSIZE_MASK);
            break;
        case RP_PID_21:
            cmn_GetValue(&pid_reg->relock21_stepsize, &stepsize_integer, PID_STEPSIZE_MASK);
            break;
        case RP_PID_22:
            cmn_GetValue(&pid_reg->relock22_stepsize, &stepsize_integer, PID_STEPSIZE_MASK);
            break;
        default: return RP_EPN;
    }

    *stepsize = (float)stepsize_integer*PID_DACCOUNT/(PID_TIMESTEP * (1 << PID_STEPSR));
    return RP_OK;
}

int pid_SetRelockMinimum(rp_pid_t pid, float minimum) {
    uint32_t minimum_counts;
    minimum_counts = (uint32_t) ((minimum - ANALOG_IN_MIN_VAL) / (ANALOG_IN_MAX_VAL - ANALOG_IN_MIN_VAL) * ANALOG_IN_MAX_VAL_INTEGER);

    switch(pid) {
        case RP_PID_11: return cmn_SetValue(&pid_reg->relock11_minval, minimum_counts, PID_RELOCK_MASK);
        case RP_PID_12: return cmn_SetValue(&pid_reg->relock12_minval, minimum_counts, PID_RELOCK_MASK);
        case RP_PID_21: return cmn_SetValue(&pid_reg->relock21_minval, minimum_counts, PID_RELOCK_MASK);
        case RP_PID_22: return cmn_SetValue(&pid_reg->relock22_minval, minimum_counts, PID_RELOCK_MASK);
        default: return RP_EPN;
    }
}
int pid_GetRelockMinimum(rp_pid_t pid, float *minimum) {
    uint32_t minimum_counts;

    switch(pid) {
        case RP_PID_11:
            cmn_GetValue(&pid_reg->relock11_minval, &minimum_counts, PID_RELOCK_MASK);
            break;
        case RP_PID_12:
            cmn_GetValue(&pid_reg->relock12_minval, &minimum_counts, PID_RELOCK_MASK);
            break;
        case RP_PID_21:
            cmn_GetValue(&pid_reg->relock21_minval, &minimum_counts, PID_RELOCK_MASK);
            break;
        case RP_PID_22:
            cmn_GetValue(&pid_reg->relock22_minval, &minimum_counts, PID_RELOCK_MASK);
            break;
        default: return RP_EPN;
    }

    *minimum = (float)minimum_counts / ANALOG_IN_MAX_VAL_INTEGER * (ANALOG_IN_MAX_VAL - ANALOG_IN_MIN_VAL) + ANALOG_IN_MIN_VAL;
    return RP_OK;
}

int pid_SetRelockMaximum(rp_pid_t pid, float maximum) {
    uint32_t maximum_counts;
    maximum_counts = (uint32_t) ((maximum - ANALOG_IN_MIN_VAL) / (ANALOG_IN_MAX_VAL - ANALOG_IN_MIN_VAL) * ANALOG_IN_MAX_VAL_INTEGER);

    switch(pid) {
        case RP_PID_11: return cmn_SetValue(&pid_reg->relock11_maxval, maximum_counts, PID_RELOCK_MASK);
        case RP_PID_12: return cmn_SetValue(&pid_reg->relock12_maxval, maximum_counts, PID_RELOCK_MASK);
        case RP_PID_21: return cmn_SetValue(&pid_reg->relock21_maxval, maximum_counts, PID_RELOCK_MASK);
        case RP_PID_22: return cmn_SetValue(&pid_reg->relock22_maxval, maximum_counts, PID_RELOCK_MASK);
        default: return RP_EPN;
    }
}

int pid_GetRelockMaximum(rp_pid_t pid, float *maximum) {
    uint32_t maximum_counts;

    switch(pid) {
        case RP_PID_11:
            cmn_GetValue(&pid_reg->relock11_maxval, &maximum_counts, PID_RELOCK_MASK);
            break;
        case RP_PID_12:
            cmn_GetValue(&pid_reg->relock12_maxval, &maximum_counts, PID_RELOCK_MASK);
            break;
        case RP_PID_21:
            cmn_GetValue(&pid_reg->relock21_maxval, &maximum_counts, PID_RELOCK_MASK);
            break;
        case RP_PID_22:
            cmn_GetValue(&pid_reg->relock22_maxval, &maximum_counts, PID_RELOCK_MASK);
            break;
        default: return RP_EPN;
    }
    
    *maximum = (float)maximum_counts / ANALOG_IN_MAX_VAL_INTEGER * (ANALOG_IN_MAX_VAL - ANALOG_IN_MIN_VAL) + ANALOG_IN_MIN_VAL;
    return RP_OK;
}

int pid_SetRelockInput(rp_pid_t pid, rp_apin_t pin) {
    if (pin > RP_AIN3)
        return RP_EPN;
    switch(pid) {
        case RP_PID_11: return cmn_SetValue(&pid_reg->relock11_input, pin-RP_AIN0, PID_RELOCK_INPUT_MASK);
        case RP_PID_12: return cmn_SetValue(&pid_reg->relock12_input, pin-RP_AIN0, PID_RELOCK_INPUT_MASK);
        case RP_PID_21: return cmn_SetValue(&pid_reg->relock21_input, pin-RP_AIN0, PID_RELOCK_INPUT_MASK);
        case RP_PID_22: return cmn_SetValue(&pid_reg->relock22_input, pin-RP_AIN0, PID_RELOCK_INPUT_MASK);
        default: return RP_EPN;
    }
    return RP_OK;
}
int pid_GetRelockInput(rp_pid_t pid, rp_apin_t *pin) {
    rp_apin_t tmp_pin;
    switch(pid) {
        case RP_PID_11:
            cmn_GetValue(&pid_reg->relock11_input, &tmp_pin, PID_RELOCK_INPUT_MASK);
            break;
        case RP_PID_12:
            cmn_GetValue(&pid_reg->relock12_input, &tmp_pin, PID_RELOCK_INPUT_MASK);
            break;
        case RP_PID_21:
            cmn_GetValue(&pid_reg->relock21_input, &tmp_pin, PID_RELOCK_INPUT_MASK);
            break;
        case RP_PID_22:
            cmn_GetValue(&pid_reg->relock22_input, &tmp_pin, PID_RELOCK_INPUT_MASK);
            break;
        default: return RP_EPN;
    }
    *pin = tmp_pin+RP_AIN0;
    return RP_OK;
}
