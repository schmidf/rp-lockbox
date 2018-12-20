/**
 * Copyright (c) 2018, Fabian Schmid
 *
 * All rights reserved.
 *
 * @brief Red Pitaya library PID module interface
 *
 * @Author Red Pitaya
 *
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in C programming language.
 * Please visit http://en.wikipedia.org/wiki/C_(programming_language)
 * for more details on the language used herein.
 */



#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include <stdbool.h>
#include <redpitaya/lockbox.h>
#include "analog_mixed_signals.h"

#define SETPOINT_MAX        1.0     // V
#define DATA_BIT_LENGTH     14      // Used for V->counts conversion

// Base PID address
static const int PID_BASE_ADDR = 0x00300000;
static const int PID_BASE_SIZE = 0x4C;

// PID structure declaration
typedef struct pid_control_s {
    uint32_t conf;
    uint32_t reserved[3];
    uint32_t pid11_setpoint;
    uint32_t pid12_setpoint;
    uint32_t pid21_setpoint;
    uint32_t pid22_setpoint;
    uint32_t pid11_Kp;
    uint32_t pid12_Kp;
    uint32_t pid21_Kp;
    uint32_t pid22_Kp;
    uint32_t pid11_Ki;
    uint32_t pid12_Ki;
    uint32_t pid21_Ki;
    uint32_t pid22_Ki;
    uint32_t pid11_Kd;
    uint32_t pid12_Kd;
    uint32_t pid21_Kd;
    uint32_t pid22_Kd;
    uint32_t relock11_minval;
    uint32_t relock12_minval;
    uint32_t relock21_minval;
    uint32_t relock22_minval;
    uint32_t relock11_maxval;
    uint32_t relock12_maxval;
    uint32_t relock21_maxval;
    uint32_t relock22_maxval;
    uint32_t relock11_stepsize;
    uint32_t relock12_stepsize;
    uint32_t relock21_stepsize;
    uint32_t relock22_stepsize;
    uint32_t relock11_input;
    uint32_t relock12_input;
    uint32_t relock21_input;
    uint32_t relock22_input;
} pid_control_t;

static const uint32_t PID_CONF_MASK = 0xFFFFF; // (20 bits)
static const uint32_t PID_SETPOINT_MASK = 0x3FFF; // (14 bits)
static const uint32_t PID_KP_MASK = 0xFFFFFF; // (24 bits)
static const uint32_t PID_KI_MASK = 0xFFFFFF; // (24 bits)
static const uint32_t PID_KD_MASK = 0x3FFF; // (14 bits)
static const uint32_t PID_STEPSIZE_MASK = 0xFFFFFF; // (24 bits)
static const uint32_t PID_RELOCK_MASK = 0xFFF; // (12 bits)
static const uint32_t PID_RELOCK_INPUT_MASK = 0x3; // (2 bits)

static const float PID_TIMESTEP = 8E-9; // Inverse of the sampling rate
static const float PID_DACCOUNT = 1.221E-4; // DAC count in V = 2V/2**14
static const uint32_t PID_PSR = 12; // P gain = Kp >> PID_PSR
static const uint32_t PID_ISR = 28; // I gain = Ki >> PID_PSR
// Slew rate (in DAC counts/clock cycle) = stepsize >> PID_STEPSR
static const uint32_t PID_STEPSR = 18; 

int pid_Init();
int pid_Release();

int pid_SetPIDSetpoint(rp_pid_t pid, float setpoint);
int pid_GetPIDSetpoint(rp_pid_t pid, float *setpoint);
int pid_SetPIDKp(rp_pid_t pid, float kp);
int pid_GetPIDKp(rp_pid_t pid, float *kp);
int pid_SetPIDKi(rp_pid_t pid, float ki);
int pid_GetPIDKi(rp_pid_t pid, float *ki);
int pid_SetPIDKd(rp_pid_t pid, uint32_t kd);
int pid_GetPIDKd(rp_pid_t pid, uint32_t *kd);
int pid_SetPIDIntReset(rp_pid_t pid, bool enable);
int pid_GetPIDIntReset(rp_pid_t pid, bool *enabled);
int pid_SetPIDInverted(rp_pid_t pid, bool inverted);
int pid_GetPIDInverted(rp_pid_t pid, bool *inverted);
int pid_SetResetWhenRailed(rp_pid_t pid, bool enable);
int pid_GetResetWhenRailed(rp_pid_t pid, bool *enabled);
int pid_SetIntegratorHold(rp_pid_t pid, bool enable);
int pid_GetIntegratorHold(rp_pid_t pid, bool *enabled);
int pid_SetPIDRelock(rp_pid_t pid, bool enable);
int pid_GetPIDRelock(rp_pid_t pid, bool *enabled);
int pid_SetRelockStepsize(rp_pid_t pid, float stepsize);
int pid_GetRelockStepsize(rp_pid_t pid, float *stepsize);
int pid_SetRelockMinimum(rp_pid_t pid, float minimum);
int pid_GetRelockMinimum(rp_pid_t pid, float *minimum);
int pid_SetRelockMaximum(rp_pid_t pid, float maximum);
int pid_GetRelockMaximum(rp_pid_t pid, float *maximum);
int pid_SetRelockInput(rp_pid_t pid, rp_apin_t pin);
int pid_GetRelockInput(rp_pid_t pid, rp_apin_t *pin);

#endif //__PID_H
