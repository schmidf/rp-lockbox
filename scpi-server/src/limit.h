/*
 * Copyright (c) 2018, Fabian Schmid
 *
 * All rights reserved.
 *
 * Output limiting SCPI commands implementation
 */

#ifndef LIMIT_H_
#define LIMIT_H_

#include "scpi/types.h"

scpi_result_t RP_OutputLimitMin(scpi_t *context);
scpi_result_t RP_OutputLimitMinQ(scpi_t *context);
scpi_result_t RP_OutputLimitMax(scpi_t *context);
scpi_result_t RP_OutputLimitMaxQ(scpi_t *context);
#endif /* LIMIT_H_ */
