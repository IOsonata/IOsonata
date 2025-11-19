/***********************************************************************************
* Copyright (c) 2010 - 2018, Motsai
* All rights reserved.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************************/

#include <assert.h>
#include <string.h>

#include "idelay.h"
#include "device/lsm303agr_reg.h"

#include "device/lsm303agr.h"

/**********************************************************************************/

static int32_t LSM303AGR_readXL(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
    LSM303AGR *dev = (LSM303AGR*)handle;

    return dev->ReadXL(ReadAddr, pBuffer, nBytesToRead) > 0 ? 0 : 1;
}

/**********************************************************************************/

static int32_t LSM303AGR_writeXL(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
    LSM303AGR *dev = (LSM303AGR*)handle;

    return dev->WriteXL(WriteAddr, pBuffer, nBytesToWrite) > 0 ? 0 : 1;
}

/**********************************************************************************/

static int32_t LSM303AGR_readMG(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
    LSM303AGR *dev = (LSM303AGR*)handle;

    return dev->ReadMG(ReadAddr, pBuffer, nBytesToRead) > 0 ? 0 : 1;
}

/**********************************************************************************/

static int32_t LSM303AGR_writeMG(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
    LSM303AGR *dev = (LSM303AGR*)handle;

    return dev->WriteMG(WriteAddr, pBuffer, nBytesToWrite) > 0 ? 0 : 1;
}

/**********************************************************************************/

LSM303AGR::LSM303AGR():
    mpDevIntrf( nullptr )
{
    mbSpi = false;
}

/**********************************************************************************/

lsm303agr_error_t LSM303AGR::init( DeviceIntrf* ppInterface, const lsm303agr_config_t& prConfig )
{
    if (ppInterface == NULL)
    {
        return LSM303AGR_ERROR_FATAL;
    }

    mpDevIntrf = ppInterface;
    mConfig = prConfig;
    AccelerometerSensor::init( ppInterface);
    MagnetometerSensor::init( ppInterface);

    mXLDrvHandle.write_reg = LSM303AGR_writeXL;
    mXLDrvHandle.read_reg = LSM303AGR_readXL;
    mXLDrvHandle.handle = (void*)this;

    mMGDrvHandle.write_reg = LSM303AGR_writeMG;
    mMGDrvHandle.read_reg = LSM303AGR_readMG;
    mMGDrvHandle.handle = (void*)this;

    if (prConfig.DevAddrXL == LSM303AGR_I2C_ADD_XL || prConfig.DevAddrMG == LSM303AGR_I2C_ADD_MG)
    {
        mbSpi = false;
    }
    else
    {
        mbSpi = true;
        lsm303agr_xl_spi_mode_set(&mXLDrvHandle, LSM303AGR_SPI_3_WIRE);
    }

    uint32_t d = 0;

    lsm303agr_xl_device_id_get(&mXLDrvHandle, (uint8_t*)&d);
    if (d != LSM303AGR_ID_XL)
    {
        return LSM303AGR_ERROR_FATAL;
    }

    lsm303agr_mag_device_id_get(&mMGDrvHandle, (uint8_t*)&d);
    if (d != LSM303AGR_ID_MG)
    {
        return LSM303AGR_ERROR_FATAL;
    }
    /*
     *  Restore default configuration for magnetometer
     */
    lsm303agr_mag_reset_set(&mMGDrvHandle, PROPERTY_ENABLE);
    do {
       lsm303agr_mag_reset_get(&mMGDrvHandle, (uint8_t*)&d);
    } while (d);

    /*
     *  Enable Block Data Update
     */
    lsm303agr_xl_block_data_update_set(&mXLDrvHandle, PROPERTY_ENABLE);
    lsm303agr_mag_block_data_update_set(&mMGDrvHandle, PROPERTY_ENABLE);

    /*
     * Set Output Data Rate
     */
    setRate( prConfig.XL_Rate );
    setRate( prConfig.MG_Rate );
    setRange( ACCELEROMETER_RANGE_2G );

    /*
     * Set / Reset magnetic sensor mode
     */
    lsm303agr_mag_set_rst_mode_set(&mMGDrvHandle, LSM303AGR_SENS_OFF_CANC_EVERY_ODR);
    /*
     * Enable temperature compensation on mag sensor
     */
    lsm303agr_mag_offset_temp_comp_set(&mMGDrvHandle, PROPERTY_ENABLE);
    /*
     * Enable temperature sensor
     */
    lsm303agr_temperature_meas_set(&mXLDrvHandle, LSM303AGR_TEMP_ENABLE);
    /*
     * Set device in continuous mode
     */
    lsm303agr_xl_operating_mode_set(&mXLDrvHandle, LSM303AGR_LP_8bit);
    /*
     * Set magnetometer in continuous mode
     */
    lsm303agr_mag_operating_mode_set(&mMGDrvHandle, LSM303AGR_CONTINUOUS_MODE);

    return LSM303AGR_ERROR_NONE;
}

/**********************************************************************************/

void LSM303AGR::power( bool pOn )
{
    /// Due to some issue when powering XL back up, this was disable.
    /*if ( pOn ) {
        setRate( ACCELEROMETER_RATE_50 );
        lsm303agr_xl_full_scale_set(&mXLDrvHandle, LSM303AGR_2g);
        lsm303agr_xl_operating_mode_set(&mXLDrvHandle, LSM303AGR_HR_12bit);
        lsm303agr_mag_operating_mode_set( &mMGDrvHandle, LSM303AGR_CONTINUOUS_MODE );
        lsm303agr_temperature_meas_set(&mXLDrvHandle, LSM303AGR_TEMP_ENABLE);
    }
    else {

        setRate( ACCELEROMETER_RATE_0 );
        lsm303agr_mag_operating_mode_set( &mMGDrvHandle, LSM303AGR_POWER_DOWN );
        lsm303agr_temperature_meas_set(&mXLDrvHandle, LSM303AGR_TEMP_DISABLE);
    }*/
}

/**********************************************************************************/

bool LSM303AGR::read( AccelerometerSensorData_t& prData )
{
    axis3bit16_t data;
    lsm303agr_reg_t reg;

    prData.valid = false;

    int32_t res = lsm303agr_xl_status_get(&mXLDrvHandle, &reg.status_reg_a);
    if (res == 0 && reg.status_reg_a.zyxda) {
        res = lsm303agr_acceleration_raw_get(&mXLDrvHandle, data.u8bit);
        if (res == 0) {
            prData.valid = true;
            prData.x = data.i16bit[0];
            prData.y = data.i16bit[1];
            prData.z = data.i16bit[2];
            /*switch ( mXLRange ) {
            case ACCELEROMETER_RANGE_2G:
                prData.x = LSM303AGR_FROM_FS_2g_LP_TO_mg( data.i16bit[0] );
                prData.y = LSM303AGR_FROM_FS_2g_LP_TO_mg( data.i16bit[1] );
                prData.z = LSM303AGR_FROM_FS_2g_LP_TO_mg( data.i16bit[2] );
                break;
            case ACCELEROMETER_RANGE_4G:
                prData.x = LSM303AGR_FROM_FS_4g_LP_TO_mg( data.i16bit[0] );
                prData.y = LSM303AGR_FROM_FS_4g_LP_TO_mg( data.i16bit[1] );
                prData.z = LSM303AGR_FROM_FS_4g_LP_TO_mg( data.i16bit[2] );
                break;
            case ACCELEROMETER_RANGE_8G:
                prData.x = LSM303AGR_FROM_FS_8g_LP_TO_mg( data.i16bit[0] );
                prData.y = LSM303AGR_FROM_FS_8g_LP_TO_mg( data.i16bit[1] );
                prData.z = LSM303AGR_FROM_FS_8g_LP_TO_mg( data.i16bit[2] );
                break;
            case ACCELEROMETER_RANGE_16G:
                prData.x = LSM303AGR_FROM_FS_16g_LP_TO_mg( data.i16bit[0] );
                prData.y = LSM303AGR_FROM_FS_16g_LP_TO_mg( data.i16bit[1] );
                prData.z = LSM303AGR_FROM_FS_16g_LP_TO_mg( data.i16bit[2] );
                break;
            default:
                assert( false );
                break;
            }*/
        }
    }
    return prData.valid;
}

/**********************************************************************************/

bool LSM303AGR::read( MagnetometerSensorData_t& prData )
{
    axis3bit16_t data;
    lsm303agr_reg_t reg;

    prData.valid = false;

    int32_t res = lsm303agr_mag_status_get( &mMGDrvHandle, &reg.status_reg_m );
    if ( res == 0 && reg.status_reg_m.zyxda ) {
        res = lsm303agr_magnetic_raw_get(&mMGDrvHandle, data.u8bit);
        if ( res == 0 ) {
            prData.valid = true;
            prData.x = LSM303AGR_FROM_LSB_TO_mG( data.i16bit[0]);
            prData.y = LSM303AGR_FROM_LSB_TO_mG( data.i16bit[1]);
            prData.z = LSM303AGR_FROM_LSB_TO_mG( data.i16bit[2]);
        }
    }

    return prData.valid;
}

/**********************************************************************************/

bool LSM303AGR::read( TemperatureSensorData_t& prData )
{
    uint8_t lBuffer[2];
    lsm303agr_reg_t reg;

    prData.valid = false;

    int32_t lRes = lsm303agr_temp_data_ready_get( &mXLDrvHandle, &reg.byte );
    if ( lRes == 0 && reg.byte ) {
        lRes = lsm303agr_temperature_raw_get( &mXLDrvHandle, lBuffer );
        if ( lRes == 0 ) {
            prData.valid = true;
            prData.value = lBuffer[1] << 8 | lBuffer[0];
            prData.value = LSM303AGR_FROM_LSB_TO_degC_LP( prData.value );
            //prData.value = (float)((int16_t)prData.value >> 6) / 4.0f + 25.0f;
            prData.value *= 100;
        }
    }

    return prData.valid;
}

/**********************************************************************************/

void LSM303AGR::getRange( AccelerometerRange_t& prRange )
{
    lsm303agr_fs_a_t lVal;
    lsm303agr_xl_full_scale_get( &mXLDrvHandle, &lVal );

    switch ( lVal ) {
    case LSM303AGR_2g:
        prRange = ACCELEROMETER_RANGE_2G;
        break;
    case LSM303AGR_4g:
        prRange = ACCELEROMETER_RANGE_4G;
        break;
    case LSM303AGR_8g:
        prRange = ACCELEROMETER_RANGE_8G;
        break;
    case LSM303AGR_16g:
        prRange = ACCELEROMETER_RANGE_16G;
        break;
    default:
        assert( false );
        break;
    }
}

/**********************************************************************************/

void LSM303AGR::setRange( AccelerometerRange_t pRange )
{
    mXLRange = pRange;

    lsm303agr_fs_a_t fs = LSM303AGR_2g;

    switch (pRange)
    {
        case ACCELEROMETER_RANGE_2G:
            fs = LSM303AGR_2g;
            break;

        case ACCELEROMETER_RANGE_4G:
            fs = LSM303AGR_4g;
            break;

        case ACCELEROMETER_RANGE_8G:
            fs = LSM303AGR_8g;
            break;

        case ACCELEROMETER_RANGE_16G:
        default:
            fs = LSM303AGR_16g;
            break;
    }

    lsm303agr_xl_full_scale_set(&mXLDrvHandle, fs);
}

/**********************************************************************************/

void LSM303AGR::getRange( MagnetometerSensorRange_t& prRange )
{
    // Unimplemented
    assert( false );
}

/**********************************************************************************/

void LSM303AGR::setRange( const MagnetometerSensorRange_t& prRange )
{
    // Unimplemented
    assert( false );
}

/**********************************************************************************/

void LSM303AGR::getRate( AccelerometerRate_t& prRate )
{
    lsm303agr_odr_a_t lRate;
    lsm303agr_xl_data_rate_get( &mXLDrvHandle, &lRate );

    switch ( lRate ) {
    case LSM303AGR_XL_POWER_DOWN:
        prRate = ACCELEROMETER_RATE_0;
        break;
    case LSM303AGR_XL_ODR_1Hz:
        prRate = ACCELEROMETER_RATE_1;
        break;
    case LSM303AGR_XL_ODR_10Hz:
        prRate = ACCELEROMETER_RATE_10;
        break;
    case LSM303AGR_XL_ODR_25Hz:
        prRate = ACCELEROMETER_RATE_25;
        break;
    case LSM303AGR_XL_ODR_50Hz:
        prRate = ACCELEROMETER_RATE_50;
        break;
    case LSM303AGR_XL_ODR_100Hz:
        prRate = ACCELEROMETER_RATE_100;
        break;
    case LSM303AGR_XL_ODR_200Hz:
        prRate = ACCELEROMETER_RATE_200;
        break;
    case LSM303AGR_XL_ODR_400Hz:
        prRate = ACCELEROMETER_RATE_400;
        break;
    default:
        assert( false );
        break;
    }
}

/**********************************************************************************/

void LSM303AGR::setRate( AccelerometerRate_t pRate )
{
    lsm303agr_odr_a_t rate;

    switch ( pRate ) {
    case ACCELEROMETER_RATE_0:
        rate = LSM303AGR_XL_POWER_DOWN;
        break;
    case ACCELEROMETER_RATE_50:
        rate = LSM303AGR_XL_ODR_50Hz;
        break;
    case ACCELEROMETER_RATE_100:
        rate = LSM303AGR_XL_ODR_100Hz;
        break;
    case ACCELEROMETER_RATE_200:
        rate = LSM303AGR_XL_ODR_200Hz;
        break;
    case ACCELEROMETER_RATE_400:
        rate = LSM303AGR_XL_ODR_400Hz;
        break;
    case ACCELEROMETER_RATE_1600:
        rate = LSM303AGR_XL_ODR_1kHz620_LP;
        break;
    default:
        assert( false );
        break;
    }

    lsm303agr_xl_data_rate_set(&mXLDrvHandle, rate);
}

/**********************************************************************************/

void LSM303AGR::getRate( MagnetometerRate_t& prRate )
{
    lsm303agr_mg_odr_m_t lRate;
    lsm303agr_mag_data_rate_get( &mXLDrvHandle, &lRate );

    switch ( lRate ) {
    case LSM303AGR_MG_ODR_10Hz:
        prRate = MAGNETOMETER_RATE_10;
        break;
    case LSM303AGR_MG_ODR_20Hz:
        prRate = MAGNETOMETER_RATE_20;
        break;
    case LSM303AGR_MG_ODR_50Hz:
        prRate = MAGNETOMETER_RATE_50;
        break;
    case LSM303AGR_MG_ODR_100Hz:
        prRate = MAGNETOMETER_RATE_100;
        break;
    default:
        assert( false );
        break;
    }
}

/**********************************************************************************/

void LSM303AGR::setRate( MagnetometerRate_t pRate )
{
    lsm303agr_mg_odr_m_t rate;

    switch ( pRate ) {
    case MAGNETOMETER_RATE_0:
        lsm303agr_mag_operating_mode_set( &mMGDrvHandle, LSM303AGR_POWER_DOWN );
        break;
    case MAGNETOMETER_RATE_50:
        lsm303agr_mag_operating_mode_set( &mMGDrvHandle, LSM303AGR_CONTINUOUS_MODE );
        lsm303agr_mag_data_rate_set( &mXLDrvHandle, LSM303AGR_MG_ODR_50Hz );
        break;
    case MAGNETOMETER_RATE_100:
        lsm303agr_mag_operating_mode_set( &mMGDrvHandle, LSM303AGR_CONTINUOUS_MODE );
        lsm303agr_mag_data_rate_set( &mXLDrvHandle, LSM303AGR_MG_ODR_100Hz );
        break;
    default:
        assert( false );
        break;
    }
}

/**********************************************************************************/

void LSM303AGR::setActivityInterrupt( bool pEnable, uint8_t pThreshold, uint8_t pTimeout )
{
    if ( pEnable ) {
        lsm303agr_act_timeout_set( &mXLDrvHandle, pTimeout );
        lsm303agr_act_threshold_set( &mXLDrvHandle, pThreshold );

        lsm303agr_ctrl_reg6_a_t lCtrlReg6;
        lsm303agr_xl_pin_int2_config_get( &mXLDrvHandle, &lCtrlReg6 );
        lCtrlReg6.p2_act = 1;
        lsm303agr_xl_pin_int2_config_set( &mXLDrvHandle, &lCtrlReg6 );
    }
    else {
        lsm303agr_act_timeout_set( &mXLDrvHandle, 0 );
        lsm303agr_act_threshold_set( &mXLDrvHandle, 0 );

        lsm303agr_ctrl_reg6_a_t lCtrlReg6;
        lsm303agr_xl_pin_int2_config_get( &mXLDrvHandle, &lCtrlReg6 );
        lCtrlReg6.p2_act = 0;
        lsm303agr_xl_pin_int2_config_set( &mXLDrvHandle, &lCtrlReg6 );
    }
}

/**********************************************************************************/

/*#include "iopincfg.h"
const IOPINCFG s_Spi0Pins[] = {
    {0, 11, 1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},    // SCK
    {0, 13, 1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},   // MISO
    {0, 12, 1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},    // MOSI
    {0, 29, 0, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, // CS Accel
    {0, 8, 0, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, // CS Mag
};*/
int LSM303AGR::ReadXL(uint8_t RegAddr, uint8_t *pBuffer, int BuffLen)
{
    int cnt = 0;
    int devaddr = LSM303AGR_I2C_ADD_XL >> 1;

    if (mbSpi == true)
    {
        // SPI

        devaddr = mConfig.DevAddrXL;

        RegAddr |= 0x80;    // Read access

        if (BuffLen > 1)
        {
            RegAddr |= 0x40;
        }

        /*mpDevIntrf->StartRx( devaddr );
        IOPinDisable( 0, s_Spi0Pins[1].PinNo );
        mpDevIntrf->TxData( &RegAddr, 1 );
        //usDelay( 1 );
        IOPinCfg( &s_Spi0Pins[1], 1 );
        IOPinDisable( 0, s_Spi0Pins[2].PinNo );
        cnt += mpDevIntrf->RxData( pBuffer, BuffLen );
        //usDelay( 1 );
        IOPinCfg( &s_Spi0Pins[2], 1 );
        mpDevIntrf->StopRx();*/
    }
    else
    {
        if (BuffLen > 1)
        {
            RegAddr |= 0x80;
        }
    }

    cnt = mpDevIntrf->Read(devaddr, &RegAddr, 1, pBuffer, BuffLen);

    return cnt;
}

/**********************************************************************************/

int LSM303AGR::WriteXL(uint8_t RegAddr, uint8_t *pData, int DataLen)
{
    int devaddr = LSM303AGR_I2C_ADD_XL >> 1;

    if (mbSpi == true)
    {
        // SPI

        devaddr = mConfig.DevAddrXL;

        if (DataLen > 1)
        {
            RegAddr |= 0x40;
        }
    }

    return mpDevIntrf->Write(devaddr, &RegAddr, 1, pData, DataLen);
}

/**********************************************************************************/

int LSM303AGR::ReadMG(uint8_t RegAddr, uint8_t *pBuffer, int BuffLen)
{
    int devaddr = LSM303AGR_I2C_ADD_MG >> 1;

    if (mbSpi == true)
    {
        // SPI

        devaddr = mConfig.DevAddrMG;

        RegAddr |= 0x80;    // Read access
    }

    return mpDevIntrf->Read(devaddr, &RegAddr, 1, pBuffer, BuffLen);
}

/**********************************************************************************/

int LSM303AGR::WriteMG(uint8_t RegAddr, uint8_t *pData, int DataLen)
{
    int devaddr = LSM303AGR_I2C_ADD_MG >> 1;

    if (mbSpi == true)
    {
        // SPI

        devaddr = mConfig.DevAddrMG;
    }

    return mpDevIntrf->Write(devaddr, &RegAddr, 1, pData, DataLen);
}

/**********************************************************************************/
