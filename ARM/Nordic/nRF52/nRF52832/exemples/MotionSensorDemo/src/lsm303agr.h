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

#pragma once

/**********************************************************************************/

#include "device/lsm303agr_reg.h"

#include "accelerometer_sensor.h"
#include "magnetometer_sensor.h"
#include "temperature_sensor.h"

/**********************************************************************************/

typedef enum {
    LSM303AGR_ERROR_NONE,
    LSM303AGR_ERROR_COMMUNICATION,
    LSM303AGR_ERROR_FATAL
} lsm303agr_error_t;

typedef struct {
    int DevAddrXL;          // XL Device address (CS index for SPI)
    int DevAddrMG;          // MG Device address (CS index for SPI)
    AccelerometerRate_t XL_Rate;               // XL ODR
    MagnetometerRate_t MG_Rate;
} lsm303agr_config_t;

/**********************************************************************************/

class DeviceIntrf;

class LSM303AGR : public AccelerometerSensor,
                  public MagnetometerSensor,
                  public TemperatureSensor
{
public:
    LSM303AGR();

    lsm303agr_error_t init( DeviceIntrf* ppInterface, const lsm303agr_config_t& prConfig );

    void power( bool pOn );

    virtual void getRange( AccelerometerRange_t& prRange ) override;
    virtual void setRange( AccelerometerRange_t pRange ) override;

    virtual void getRange( MagnetometerSensorRange_t& prRange ) override;
    virtual void setRange( const MagnetometerSensorRange_t& prRange ) override;

    virtual void getRate( AccelerometerRate_t& prRate ) override;
    virtual void setRate( AccelerometerRate_t pRate ) override;

    virtual void getRate( MagnetometerRate_t& prRate ) override;
    virtual void setRate( MagnetometerRate_t pRate ) override;

    void setActivityInterrupt( bool pEnable, uint8_t pThreshold = 0, uint8_t pTimeout = 0 );

    virtual bool read( AccelerometerSensorData_t& prData ) override;
    virtual bool read( MagnetometerSensorData_t& prData ) override;
    virtual bool read( TemperatureSensorData_t& prData ) override;

    int ReadXL( uint8_t RegAddr, uint8_t *pBuffer, int BuffLen );
    int WriteXL( uint8_t RegAddr, uint8_t *pData, int DataLen );
    int ReadMG( uint8_t RegAddr, uint8_t *pBuffer, int BuffLen );
    int WriteMG( uint8_t RegAddr, uint8_t *pData, int DataLen );



private:
    lsm303agr_ctx_t mXLDrvHandle;   // Accel
    lsm303agr_ctx_t mMGDrvHandle;   // Mag
    DeviceIntrf *mpDevIntrf;
    lsm303agr_config_t mConfig;
    AccelerometerRange_t mXLRange;
    bool mbSpi;
};

/**********************************************************************************/
