

# File device.h

[**File List**](files.md) **>** [**include**](dir_d44c64559bbebec7f509842c48db8b23.md) **>** [**device.h**](device_8h.md)

[Go to the documentation of this file](device_8h.md)


```C++

#ifndef __DEVICE_H__
#define __DEVICE_H__

#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "coredev/iopincfg.h"
#include "coredev/timer.h"
#include "device_intrf.h"

typedef enum __Dev_Interrupt_Polarity {
    DEVINTR_POL_LOW,    
    DEVINTR_POL_HIGH    
} DEVINTR_POL;

typedef enum __Device_Event {
    DEV_EVT_DATA_RDY
} DEV_EVT;

#ifdef __cplusplus

class Device;

typedef void (*DevEvtHandler_t)(Device * const pDev, DEV_EVT Evt);
//typedef DevEvtHandler_t       DEVEVTCB;

class Device {
public:
    Device();
    virtual ~Device() {}

    //
    // *** Require implementations ***
    //

    virtual bool Enable() = 0;

    virtual void Disable() = 0;

    virtual void Reset() = 0;

    //
    // *** Optional implementations ***
    //

    virtual void PowerOff() {}

    virtual void DeviceAddress(uint32_t Addr) { vDevAddr =  Addr; }

    virtual uint32_t DeviceAddress() { return vDevAddr; }

    virtual uint64_t DeviceID() { return vDevId; }

    virtual int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);

    virtual int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);

    virtual uint8_t Read8(uint8_t *pRegAddr, int RegAddrLen) {
        uint8_t val = 0;
        Read(pRegAddr, RegAddrLen, &val, 1);
        return val;
    }

    virtual uint16_t Read16(uint8_t *pRegAddr, int RegAddrLen) {
        uint16_t val = 0;
        Read(pRegAddr, RegAddrLen,(uint8_t*) &val, 2);
        return val;
    }

    virtual uint32_t Read32(uint8_t *pRegAddr, int RegAddrLen) {
        uint32_t val = 0;
        Read(pRegAddr, RegAddrLen, (uint8_t*)&val, 4);
        return val;
    }

    virtual bool Write8(uint8_t *pRegAddr, int RegAddrLen, uint8_t Data) {
        return Write(pRegAddr, RegAddrLen, &Data, 1) > 0;
    }

    virtual bool Write16(uint8_t *pRegAddr, int RegAddrLen, uint16_t Data) {
        return Write(pRegAddr, RegAddrLen, (uint8_t*)&Data, 2) > 1;
    }

    virtual bool Write32(uint8_t *pRegAddr, int RegAddrLen, uint32_t Data) {
        return Write(pRegAddr, RegAddrLen, (uint8_t*)&Data, 1) > 3;
    }

    bool Valid() { return vbValid; }

    DEVINTRF_TYPE InterfaceType() { return vpIntrf != nullptr ? vpIntrf->Type() : DEVINTRF_TYPE_UNKOWN; }

    virtual operator Timer * const () { return vpTimer; }   // Get device interface data (handle)

    void SetEvtHandler(DevEvtHandler_t EvtHandler) { vEvtHandler = EvtHandler; }
    virtual void EvtHandler(DEV_EVT Evt) { if (vEvtHandler) vEvtHandler(this, Evt); }

protected:

    void DeviceID(uint64_t DevId) { vDevId = DevId; }

    void Valid(bool bVal) { vbValid = bVal; }

    void Interface(DeviceIntrf * const pIntrf) { vpIntrf = pIntrf; }

    DeviceIntrf * const Interface() { return vpIntrf; }

    void InterruptEnabled(bool En) { vbIntEn = En; }
    bool InterruptEnabled() { return vbIntEn; }

    bool        vbValid;        
    uint32_t    vDevAddr;       
    DeviceIntrf *vpIntrf;       
    Timer       *vpTimer;       
    uint64_t    vDevId;         
    bool        vbIntEn;        
    DevEvtHandler_t vEvtHandler;    
};

extern "C" {
#endif  // __cplusplus


#ifdef __cplusplus
}

#endif  // __cplusplus

#endif  // __DEVICE_H__
```


