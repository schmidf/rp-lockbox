/*
 * Copyright (c) 2018, Fabian Schmid
 *
 * All rights reserved.
 *
 * Output limiting SCPI commands implementation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "limit.h"

#include "common.h"
#include "scpi/parser.h"
#include "scpi/units.h"

scpi_result_t RP_OutputLimitMin(scpi_t *context) {
    int result;
    rp_channel_t channel;
    scpi_number_t limit;

    /* Parse output channel */
    if(RP_ParseChArgv(context, &channel) != RP_OK) {
        return SCPI_RES_ERR;
    }

    /* Parse first parameter (output limit) */
    if(!SCPI_ParamNumber(context, scpi_special_numbers_def, &limit, true)) {
        RP_LOG(LOG_ERR, "*OUTput#:LIMit:MIN Failed to parse first parameter.\n");
        return SCPI_RES_ERR;
    }
    
    result = rp_LimitMin(channel, limit.value);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*OUTput#:LIMit:MIN Failed to set output limit: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*OUTput#:LIMit:MIN Successfully set output limit.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_OutputLimitMinQ(scpi_t *context) {
    int result;
    rp_channel_t channel;
    float limit;

    /* Parse output channel */
    if(RP_ParseChArgv(context, &channel) != RP_OK) {
        return SCPI_RES_ERR;
    }

    result = rp_LimitGetMin(channel, &limit);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*OUTput#:LIMit:MIN? Failed to get limit: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }
    SCPI_ResultDouble(context, limit);

    RP_LOG(LOG_INFO, "*OUTput#:LIMit:MIN? Successfully returned limit value to client.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_OutputLimitMax(scpi_t *context) {
    int result;
    rp_channel_t channel;
    scpi_number_t limit;

    /* Parse output channel */
    if(RP_ParseChArgv(context, &channel) != RP_OK) {
        return SCPI_RES_ERR;
    }

    /* Parse first parameter (output limit) */
    if(!SCPI_ParamNumber(context, scpi_special_numbers_def, &limit, true)) {
        RP_LOG(LOG_ERR, "*OUTput#:LIMit:MAX Failed to parse first parameter.\n");
        return SCPI_RES_ERR;
    }
    
    result = rp_LimitMax(channel, limit.value);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*OUTput#:LIMit:MAX Failed to set output limit: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*OUTput#:LIMit:MAX Successfully set output limit.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_OutputLimitMaxQ(scpi_t *context) {
    int result;
    rp_channel_t channel;
    float limit;

    /* Parse output channel */
    if(RP_ParseChArgv(context, &channel) != RP_OK) {
        return SCPI_RES_ERR;
    }

    result = rp_LimitGetMax(channel, &limit);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*OUTput#:LIMit:MAX? Failed to get limit: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }
    SCPI_ResultDouble(context, limit);

    RP_LOG(LOG_INFO, "*OUTput#:LIMit:MAX? Successfully returned limit value to client.\n");
    return SCPI_RES_OK;
}
