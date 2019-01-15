/**
 * Copyright (c) 2018, Fabian Schmid
 *
 * All rights reserved.
 */

#include "common.h"
#include "analog_mixed_signals.h"

volatile analog_mixed_signals_control_t *ams = NULL;

int ams_Init() {
    cmn_Map(ANALOG_MIXED_SIGNALS_BASE_SIZE, ANALOG_MIXED_SIGNALS_BASE_ADDR, (void**)&ams);
    return RP_OK;
}

int ams_Release() {
    cmn_Unmap(ANALOG_MIXED_SIGNALS_BASE_SIZE, (void**)&ams);
    return RP_OK;
}
