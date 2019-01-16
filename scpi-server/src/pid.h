/**
 * Copyright (c) 2018, Fabian Schmid
 *
 * All rights reserved.
 *
 * PID SCPI commands implementation
 */

#ifndef PID_H_
#define PID_H_

#include "scpi/types.h"

scpi_result_t RP_PIDSetpoint(scpi_t *context);
scpi_result_t RP_PIDSetpointQ(scpi_t *context);
scpi_result_t RP_PIDKp(scpi_t *context);
scpi_result_t RP_PIDKpQ(scpi_t *context);
scpi_result_t RP_PIDKi(scpi_t *context);
scpi_result_t RP_PIDKiQ(scpi_t *context);
scpi_result_t RP_PIDKd(scpi_t *context);
scpi_result_t RP_PIDKdQ(scpi_t *context);
scpi_result_t RP_PIDIntReset(scpi_t *context);
scpi_result_t RP_PIDIntResetQ(scpi_t *context);
scpi_result_t RP_PIDInverted(scpi_t *context);
scpi_result_t RP_PIDInvertedQ(scpi_t *context);
scpi_result_t RP_PIDHold(scpi_t *context);
scpi_result_t RP_PIDHoldQ(scpi_t *context);
scpi_result_t RP_PIDResetWhenRailed(scpi_t *context);
scpi_result_t RP_PIDResetWhenRailedQ(scpi_t *context);
scpi_result_t RP_PIDRelock(scpi_t *context);
scpi_result_t RP_PIDRelockQ(scpi_t *context);
scpi_result_t RP_PIDRelockStepsize(scpi_t *context);
scpi_result_t RP_PIDRelockStepsizeQ(scpi_t *context);
scpi_result_t RP_PIDRelockMin(scpi_t *context);
scpi_result_t RP_PIDRelockMinQ(scpi_t *context);
scpi_result_t RP_PIDRelockMax(scpi_t *context);
scpi_result_t RP_PIDRelockMaxQ(scpi_t *context);
scpi_result_t RP_PIDRelockInput(scpi_t *context);
scpi_result_t RP_PIDRelockInputQ(scpi_t *context);
scpi_result_t RP_SaveLockboxConfig(scpi_t *context);
scpi_result_t RP_LoadLockboxConfig(scpi_t *context);
#endif /* PID_H_ */
