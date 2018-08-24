/*
 * Copyright (c) 2018, Fabian Schmid
 *
 * All rights reserved.
 */

#ifndef __LIMIT_H
#define __LIMIT_H

#define LIMIT_MAX         1.0     // V
#define LIMIT_MIN        -1.0     // V
#define DATA_BIT_LENGTH   14

// Base limit address
static const int LIMIT_BASE_ADDR = 0x40600000;
static const int LIMIT_BASE_SIZE = 0x10;

// Limit structure declaration
typedef struct limit_control_s {
    unsigned int ch_a_min   :14;
    unsigned int            :18;
    unsigned int ch_a_max   :14;
    unsigned int            :18;
    unsigned int ch_b_min   :14;
    unsigned int            :18;
    unsigned int ch_b_max   :14;
    unsigned int            :18;
} limit_control_t;

int limit_Init();
int limit_Release();

int limit_LimitMin(rp_channel_t channel, float value);
int limit_LimitMax(rp_channel_t channel, float value);
int limit_LimitGetMin(rp_channel_t channel, float *value);
int limit_LimitGetMax(rp_channel_t channel, float *value);

#endif //__LIMIT_H
