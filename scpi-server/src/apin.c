/**
 * Copyright (c) 2019, Fabian Schmid
 *
 * All rights reserved.
 *
 * @brief Red Pitaya Scpi server apin SCPI commands implementation
 *
 * @Author Red Pitaya
 *
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in C programming language.
 * Please visit http://en.wikipedia.org/wiki/C_(programming_language)
 * for more details on the language used herein.
 */

#include <stdio.h>
#include <string.h>

#include "common.h"
#include "apin.h"
#include "scpi/parser.h"
#include "../../api/src/common.h"

/* Apin choice def */
const scpi_choice_def_t scpi_RpApin[] = {
    {"AOUT0", 0},  //!< Analog output 0
    {"AOUT1", 1},  //!< Analog output 1
    {"AOUT2", 2},  //!< Analog output 2
    {"AOUT3", 3},  //!< Analog output 3
    {"AIN0",  4},  //!< Analog input 0
    {"AIN1",  5},  //!< Analog input 1
    {"AIN2",  6},  //!< Analog input 2
    {"AIN3",  7},  //!< Analog input 3
    SCPI_CHOICE_LIST_END
};

scpi_result_t RP_AnalogPinReset(scpi_t *context) {
    int result = rp_ApinReset();

    if (RP_OK != result) {
        RP_LOG(LOG_ERR, "ANALOG:RST Failed to reset Red "
            "Pitaya analog resources: %s\n" , rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*ANALOG:RST Successfully reset analog pin resources.\n");
    return SCPI_RES_OK;
}

/**
 * Returns Analog Pin value in volts to SCPI context
 * @param context SCPI context
 * @return success or failure
 */
scpi_result_t RP_AnalogPinValueQ(scpi_t * context) {
    
    int32_t choice;

    /* Read first parameter - APIN */
    if (!SCPI_ParamChoice(context, scpi_RpApin, &choice, true)) {
        RP_LOG(LOG_ERR, "*ANALOG:PIN? is missing first parameter.\n");
        return SCPI_RES_ERR;
    }

    // Convert port into pin id
    rp_apin_t pin = choice;

    // Now get the pin value
    float value;
    int result = rp_ApinGetValue(pin, &value);

    if (RP_OK != result){
        RP_LOG(LOG_ERR, "*ANALOG:PIN? Failed to get pin value: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    // Return back result
    SCPI_ResultDouble(context, value);

    RP_LOG(LOG_INFO, "*ANALOG:PIN? Successfully returned port value.\n");
    return SCPI_RES_OK;
}

/**
 * Sets Analog Pin value in volts
 * @param context SCPI context
 * @return success or failure
 */
scpi_result_t RP_AnalogPinValue(scpi_t * context) {
    
    int32_t choice;
    double value;

    /* Read first parameter - APIN */
    if (!SCPI_ParamChoice(context, scpi_RpApin, &choice, true)) {
        RP_LOG(LOG_ERR, "*ANALOG:PIN is missing first parameter.\n");
        return SCPI_RES_ERR;
    }


    /* Read second parameter - VALUE */
    if (!SCPI_ParamDouble(context, &value, true)) {
        RP_LOG(LOG_ERR, "*ANALOG:PIN is missing second parameter.\n");
        return SCPI_RES_ERR;
    }
    // Convert port into pin id
    rp_apin_t pin = choice;

    /* Set pin value */
    int result = rp_ApinSetValue(pin, (float) value);
    if (RP_OK != result){
        RP_LOG(LOG_ERR, "*ANALOG:PIN Failed to set pin value: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }

    RP_LOG(LOG_INFO, "*ANALOG:PIN Successfully set port value.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_AnalogInVoltageQ(scpi_t * context) {
    int result;
    rp_channel_t channel;
    float voltage;

    /* Parse input channel */
    if(RP_ParseChArgv(context, &channel) != RP_OK) {
        return SCPI_RES_ERR;
    }

    result = rp_GetInVoltage(channel, &voltage);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*ANALOG:IN#:VOLT? Failed to get input voltage: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }
    SCPI_ResultDouble(context, voltage);

    RP_LOG(LOG_INFO, "*ANALOG:IN#:VOLT? Successfully returned voltage to client.\n");
    return SCPI_RES_OK;
}

scpi_result_t RP_AnalogOutVoltageQ(scpi_t * context) {
    int result;
    rp_channel_t channel;
    float voltage;

    /* Parse output channel */
    if(RP_ParseChArgv(context, &channel) != RP_OK) {
        return SCPI_RES_ERR;
    }

    result = rp_GetOutVoltage(channel, &voltage);
    if(result != RP_OK) {
        RP_LOG(LOG_ERR, "*ANALOG:OUT#:VOLT? Failed to get output voltage: %s\n", rp_GetError(result));
        return SCPI_RES_ERR;
    }
    SCPI_ResultDouble(context, voltage);

    RP_LOG(LOG_INFO, "*ANALOG:OUT#:VOLT? Successfully returned voltage to client.\n");
    return SCPI_RES_OK;
}
