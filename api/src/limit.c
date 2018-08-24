/*
 * Copyright (c) 2018, Fabian Schmid
 *
 * All rights reserved.
 */

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <redpitaya/lockbox.h>
#include "common.h"
#include "limit.h"
#include "calib.h"

// The FPGA register structure for the Limiter
static volatile limit_control_t *limit_reg = NULL;
static int fd = 0;

int limit_Init() {
    if (!fd) {
        if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
            return RP_EOMD;
        }
    }
    limit_reg = mmap(NULL, LIMIT_BASE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, LIMIT_BASE_ADDR);
    return RP_OK;
}

int limit_Release() {
    if(fd == -1) {
        return RP_EUMD;
    }

    if((limit_reg == (void*) -1) || (limit_reg == NULL)) {
        return RP_EUMD;
    }

    if(munmap((void *) limit_reg, LIMIT_BASE_SIZE) < 0) {
        return RP_EUMD;
    }
    limit_reg = NULL;

    if(fd) {
        if(close(fd) < 0) {
            return RP_ECMD;
        }
    }
    return RP_OK;
}

int limit_LimitMin(rp_channel_t channel, float value) {
    rp_calib_params_t calib = calib_GetParams();

    if (channel == RP_CH_1) {
        limit_reg->ch_a_min = cmn_CnvVToCnt(DATA_BIT_LENGTH, value, LIMIT_MAX, false,
                                            calib.be_ch1_fs, calib.be_ch1_dc_offs, 0);
        return RP_OK;
    }
    else if (channel == RP_CH_2) {
        limit_reg->ch_b_min = cmn_CnvVToCnt(DATA_BIT_LENGTH, value, LIMIT_MAX, false,
                                            calib.be_ch2_fs, calib.be_ch2_dc_offs, 0);
        return RP_OK;
    }
    else
        return RP_EPN;
}

int limit_LimitMax(rp_channel_t channel, float value) {
    rp_calib_params_t calib = calib_GetParams();

    if (channel == RP_CH_1) {
        limit_reg->ch_a_max = cmn_CnvVToCnt(DATA_BIT_LENGTH, value, LIMIT_MAX, false,
                                            calib.be_ch1_fs, calib.be_ch1_dc_offs, 0);
        return RP_OK;
    }
    else if (channel == RP_CH_2) {
        limit_reg->ch_b_max = cmn_CnvVToCnt(DATA_BIT_LENGTH, value, LIMIT_MAX, false,
                                            calib.be_ch2_fs, calib.be_ch2_dc_offs, 0);
        return RP_OK;
    }

    else
        return RP_EPN;
}

int limit_LimitGetMin(rp_channel_t channel, float *value) {
    rp_calib_params_t calib = calib_GetParams();

    if (channel == RP_CH_1) {
        *value = cmn_CnvCntToV(DATA_BIT_LENGTH, limit_reg->ch_a_min, LIMIT_MAX, calib.be_ch1_fs,
                               calib.be_ch1_dc_offs, 0);
        return RP_OK;
    }
    else if (channel == RP_CH_2) {
        *value = cmn_CnvCntToV(DATA_BIT_LENGTH, limit_reg->ch_b_min, LIMIT_MAX, calib.be_ch2_fs,
                               calib.be_ch2_dc_offs, 0);
        return RP_OK;
    }
    else
        return RP_EPN;
}
int limit_LimitGetMax(rp_channel_t channel, float *value) {
    rp_calib_params_t calib = calib_GetParams();

    if (channel == RP_CH_1) {
        *value = cmn_CnvCntToV(DATA_BIT_LENGTH, limit_reg->ch_a_max, LIMIT_MAX, calib.be_ch1_fs,
                               calib.be_ch1_dc_offs, 0);
        return RP_OK;
    }
    else if (channel == RP_CH_2) {
        *value = cmn_CnvCntToV(DATA_BIT_LENGTH, limit_reg->ch_b_max, LIMIT_MAX, calib.be_ch2_fs,
                               calib.be_ch2_dc_offs, 0);
        return RP_OK;
    }
    else
        return RP_EPN;
}
