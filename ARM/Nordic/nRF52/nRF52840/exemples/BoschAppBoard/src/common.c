/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bmi3.h"
//#include "coines.h"
#include "idelay.h"
#include "coredev/spi.h"

/******************************************************************************/
/*!                Macro definition                                           */

#define READ_WRITE_LEN  UINT8_C(8)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                Static function definition                                 */

/*!
 * @brief This internal API reads I2C function map to COINES platform
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     len      : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks.
 *
 * @return Status of execution.
 */
static BMI3_INTF_RET_TYPE bmi3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief This internal API writes I2C function map to COINES platform
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     len      : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks.
 *
 * @return Status of execution.
 */
static BMI3_INTF_RET_TYPE bmi3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief This internal API reads SPI function map to COINES platform
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     len      : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks.
 *
 * @return Status of execution.
 */
static BMI3_INTF_RET_TYPE bmi3_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief This internal API writes SPI function map to COINES platform
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     len      : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks.
 *
 * @return Status of execution.
 */
static BMI3_INTF_RET_TYPE bmi3_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief This internal API maps delay function to COINES platform
 *
 * @param[in] period       : The time period in microseconds
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 *
 * @return void.
 */
static void bmi3_delay_us(uint32_t period, void *intf_ptr);

/******************************************************************************/
/*!               User interface functions                                    */

/*!
 * @brief This API prints the execution status
 */
void bmi3_error_codes_print_result(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMI3_OK:

            /*! Do nothing */
            break;

        case BMI3_E_NULL_PTR:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer," " which has been initialized to NULL.\r\n",
                rslt);
            break;

        case BMI3_E_COM_FAIL:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due " "to power failure during communication\r\n",
                rslt);
            break;

        case BMI3_E_DEV_NOT_FOUND:
            printf("%s\t", api_name);
            printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                   rslt);
            break;

        case BMI3_E_INVALID_SENSOR:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the " "available one\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_INT_PIN:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins " "apart from INT1 and INT2\r\n",
                rslt);
            break;

        case BMI3_E_ACC_INVALID_CFG:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration" " register which could be one among range, BW or filter performance in reg address 0x20\r\n",
                rslt);
            break;

        case BMI3_E_GYRO_INVALID_CFG:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration" "register which could be one among range, BW or filter performance in reg address 0x21\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_INPUT:
            printf("%s\t", api_name);
            printf("Error [%d] : Invalid input error. It occurs when the sensor input validity fails\r\n", rslt);
            break;

        case BMI3_E_INVALID_STATUS:
            printf("%s\t", api_name);
            printf("Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails\r\n", rslt);
            break;

        case BMI3_E_DATA_RDY_INT_FAILED:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit " "and data ready status is not updated\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_FOC_POSITION:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong" " axes\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_ST_SELECTION:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Invalid self-test selection error. It occurs when there is an invalid precondition" "settings such as alternate accelerometer and gyroscope enable bits, accelerometer mode and output data rate\r\n",
                rslt);
            break;

        case BMI3_E_OUT_OF_RANGE:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Out of range error. It occurs when the range exceeds the maximum range for accel while performing FOC\r\n",
                rslt);
            break;

        case BMI3_E_FEATURE_ENGINE_STATUS:
            printf("%s\t", api_name);
            printf(
                "Error [%d] : Feature engine status error. It occurs when the feature engine enable mask is not set\r\n",
                rslt);
            break;

        default:
            printf("%s\t", api_name);
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}

/*!
 * @brief This function is to select the interface between SPI and I2C.
 */
int8_t bmi3_interface_init(struct bmi3_dev *dev, int8_t intf)
{
    int8_t rslt = BMI3_OK;

    if (dev != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BMI3_I2C_INTF)
        {
            printf("I2C Interface\n");
            dev_addr = BMI3_ADDR_I2C_PRIM;
            dev->read = bmi3_i2c_read;
            dev->write = bmi3_i2c_write;
            dev->intf = BMI3_I2C_INTF;

            /* SDO pin is made low */
//            (void)coines_set_pin_config(COINES_SHUTTLE_PIN_SDO, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);

//            (void)coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
        }
        /* Bus configuration : SPI */
        else if (intf == BMI3_SPI_INTF)
        {
            printf("SPI Interface\n");
            dev_addr = 0;//COINES_MINI_SHUTTLE_PIN_2_1;
            dev->read = bmi3_spi_read;
            dev->write = bmi3_spi_write;
            dev->intf = BMI3_SPI_INTF;
           // (void)coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_10_MHZ, COINES_SPI_MODE0);
        }

#if 0
        int16_t result = coines_open_comm_intf(COINES_COMM_INTF_USB, NULL);

        if (result < COINES_SUCCESS)
        {
            printf(
                "\n Unable to connect with Application Board ! \n " "1. Check if the board is connected and powered on. \n " "2. Check if Application Board USB driver is installed. \n "
                "3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
            exit(result);
        }

        (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);
        coines_delay_msec(100);

        /* Bus configuration : I2C */
        if (intf == BMI3_I2C_INTF)
        {
            printf("I2C Interface\n");
            dev_addr = BMI3_ADDR_I2C_PRIM;
            dev->read = bmi3_i2c_read;
            dev->write = bmi3_i2c_write;
            dev->intf = BMI3_I2C_INTF;

            /* SDO pin is made low */
            (void)coines_set_pin_config(COINES_SHUTTLE_PIN_SDO, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);

            (void)coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
        }
        /* Bus configuration : SPI */
        else if (intf == BMI3_SPI_INTF)
        {
            printf("SPI Interface\n");
            dev_addr = COINES_MINI_SHUTTLE_PIN_2_1;
            dev->read = bmi3_spi_read;
            dev->write = bmi3_spi_write;
            dev->intf = BMI3_SPI_INTF;
            (void)coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_10_MHZ, COINES_SPI_MODE0);
        }

        (void)coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
        coines_delay_msec(100);
#endif

        /* Configure delay in microseconds */
        dev->delay_us = bmi3_delay_us;

        /* Assign device address to interface pointer */
        dev->intf_ptr = &dev_addr;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        dev->read_write_len = READ_WRITE_LEN;
    }
    else
    {
        rslt = BMI3_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API is used to terminate the communication.
 */
void bmi3_coines_deinit(void)
{
    (void)fflush(stdout);

    //(void)coines_set_shuttleboard_vdd_vddio_config(0, 0);
    //coines_delay_msec(1000);

    /* Coines interface reset */
    //coines_soft_reset();
    //coines_delay_msec(1000);
   // (void)coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);
}

/******************************************************************************/
/*!               Static functions                                            */
#if 0
/*!
 * @brief This internal API reads I2C function map to COINES platform
 */
static BMI3_INTF_RET_TYPE bmi3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;

    return coines_read_i2c(COINES_I2C_BUS_0, device_addr, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * @brief This internal API writes I2C function map to COINES platform
 */
static BMI3_INTF_RET_TYPE bmi3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;

    return coines_write_i2c(COINES_I2C_BUS_0, device_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}

/*!
 * @brief This internal API reads SPI function map to COINES platform
 */
static BMI3_INTF_RET_TYPE bmi3_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;

    return coines_read_spi(COINES_SPI_BUS_0, device_addr, reg_addr, reg_data, (uint16_t)len);
}

/*!
 * @brief This internal API writes SPI function map to COINES platform
 */
static BMI3_INTF_RET_TYPE bmi3_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;

    return coines_write_spi(COINES_SPI_BUS_0, device_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
}
#endif

/*!
 * @breif This internal API maps delay function to COINES platform
 */
static void bmi3_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
//    coines_delay_usec(period);
    usDelay(period);
}

