/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _COMMON_H
#define _COMMON_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bmi3.h"

/******************************************************************************/
/*!               User interface functions                                    */

/*!
 *  @brief This function is to select the interface between SPI and I2C.
 *
 *  @param[in] dev       : Structure instance of bmi3_dev
 *  @param[in] intf      : Interface selection parameter
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval < 0 -> Failure Info
 */
int8_t bmi3_interface_init(struct bmi3_dev *dev, int8_t intf);

/*!
 *  @brief This API is used to print the execution status.
 *
 *  @param[in] api_name     : Variable to store API name in string format.
 *  @param[in] rslt         : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void bmi3_error_codes_print_result(const char api_name[], int8_t rslt);

/*!
 *  @brief This API is used to terminate the communication.
 *
 *  @return void.
 */
void bmi3_coines_deinit(void);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _COMMON_H */
