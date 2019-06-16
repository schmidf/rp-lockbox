/**
 * Copyright (c) 2018, Fabian Schmid
 *
 * All rights reserved.
 */

#include <unistd.h>
#include "common.h"
#include <redpitaya/lockbox.h>
#include "calib.h"
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

int ams_GetInVoltage(rp_channel_t channel, float* value) {
    rp_calib_params_t calib = calib_GetParams();

    if (channel == RP_CH_1) {
        *value = cmn_CnvCntToV(DATA_BIT_LENGTH, ams->fadc[0], INPUT_MAX, calib.fe_ch1_fs_g_hi,
                               calib.fe_ch1_hi_offs, 0);
        return RP_OK;
    }
    else if (channel == RP_CH_2) {
        *value = cmn_CnvCntToV(DATA_BIT_LENGTH, ams->fadc[1], INPUT_MAX, calib.fe_ch2_fs_g_hi,
                               calib.fe_ch2_hi_offs, 0);
        return RP_OK;
    }
    else
        return RP_EPN;
}

int ams_GetOutVoltage(rp_channel_t channel, float* value) {
    rp_calib_params_t calib = calib_GetParams();

    if (channel == RP_CH_1) {
        *value = cmn_CnvCntToV(DATA_BIT_LENGTH, ams->fdac[0], OUTPUT_MAX, calib.be_ch1_fs,
                               calib.be_ch1_dc_offs, 0);
        return RP_OK;
    }
    else if (channel == RP_CH_2) {
        *value = cmn_CnvCntToV(DATA_BIT_LENGTH, ams->fdac[1], OUTPUT_MAX, calib.be_ch2_fs,
                               calib.be_ch2_dc_offs, 0);
        return RP_OK;
    }
    else
        return RP_EPN;
}
