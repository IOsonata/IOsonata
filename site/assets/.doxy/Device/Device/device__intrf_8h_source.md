

# File device\_intrf.h

[**File List**](files.md) **>** [**include**](dir_d44c64559bbebec7f509842c48db8b23.md) **>** [**device\_intrf.h**](device__intrf_8h.md)

[Go to the documentation of this file](device__intrf_8h.md)


```C++

#ifndef __DEVICEINTRF_H__
#define __DEVICEINTRF_H__

#include <stdint.h>

#ifdef __cplusplus
    #include <atomic>
    using namespace std;
#else
#include <stdbool.h>
#include <stdatomic.h>
#endif

typedef enum {
    DEVINTRF_EVT_RX_TIMEOUT,    
    DEVINTRF_EVT_RX_DATA,       
    DEVINTRF_EVT_RX_FIFO_FULL,  
    DEVINTRF_EVT_TX_TIMEOUT,    
    DEVINTRF_EVT_TX_READY,      
    DEVINTRF_EVT_TX_FIFO_EMPTY, 
    DEVINTRF_EVT_STATECHG,      
    DEVINTRF_EVT_READ_RQST,     
    DEVINTRF_EVT_WRITE_RQST,    
    DEVINTRF_EVT_COMPLETED,     
} DEVINTRF_EVT;

typedef enum __Dev_Intrf_Type {
    DEVINTRF_TYPE_UNKOWN,       
    DEVINTRF_TYPE_BT,          
    DEVINTRF_TYPE_ETH,          
    DEVINTRF_TYPE_I2C,          
    DEVINTRF_TYPE_CEL,          
    DEVINTRF_TYPE_SPI,          
    DEVINTRF_TYPE_QSPI,         
    DEVINTRF_TYPE_UART,         
    DEVINTRF_TYPE_USB,          
    DEVINTRF_TYPE_WIFI,         
    DEVINTRF_TYPE_I2S,          
    DEVINTRF_TYPE_PDM,          
    DEVINTRF_TYPE_OSPI,         
} DEVINTRF_TYPE;

typedef struct __device_intrf DevIntrf_t;

typedef int (*DevIntrfEvtHandler_t)(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len);

#pragma pack(push, 4)

struct __device_intrf {
    void *pDevData;             
    int IntPrio;                
    DevIntrfEvtHandler_t EvtCB; 
    atomic_flag bBusy;          
    int MaxRetry;               
    atomic_int EnCnt;           
    DEVINTRF_TYPE Type;         
    bool bDma;                  
    bool bIntEn;                
    atomic_bool bTxReady;       
    atomic_bool bNoStop;        
    // Bellow are all mandatory functions to implement
    // On init, all implementation must fill these function, no NULL allowed
    // If a function is not used. It must be implemented as do nothing function

    void (*Disable)(DevIntrf_t * const pDevIntrf);

    void (*Enable)(DevIntrf_t * const pDevIntrf);

    uint32_t (*GetRate)(DevIntrf_t * const pDevIntrf);

    uint32_t (*SetRate)(DevIntrf_t * const pDevIntrf, uint32_t Rate);

    bool (*StartRx)(DevIntrf_t * const pDevIntrf, uint32_t DevAddr);

    int (*RxData)(DevIntrf_t * const pDevIntrf, uint8_t *pBuff, int BuffLen);

    void (*StopRx)(DevIntrf_t * const pDevIntrf);

    bool (*StartTx)(DevIntrf_t * const pDevIntrf, uint32_t DevAddr);

    int (*TxData)(DevIntrf_t * const pDevIntrf, uint8_t *pData, int DataLen);

    int (*TxSrData)(DevIntrf_t * const pDevIntrf, uint8_t *pData, int DataLen);

    void (*StopTx)(DevIntrf_t * const pDevIntrf);

    void (*Reset)(DevIntrf_t * const pDevIntrf);

    void (*PowerOff)(DevIntrf_t * const pDevIntrf);

    void *(*GetHandle)(DevIntrf_t * const pDevIntrf);
};

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

static inline void DeviceIntrfDisable(DevIntrf_t * const pDev) {
    if (atomic_exchange(&pDev->EnCnt, pDev->EnCnt - 1) < 1) {
//  if (--pDev->EnCnt < 1) {
        pDev->Disable(pDev);
        pDev->EnCnt = 0;
    }
}

static inline void DeviceIntrfEnable(DevIntrf_t * const pDev) {
    if (atomic_exchange(&pDev->EnCnt, pDev->EnCnt + 1) == 1)    {
//  if (++pDev->EnCnt == 1) {
        pDev->Enable(pDev);
    }
}

static inline uint32_t DeviceIntrfGetRate(DevIntrf_t * const pDev) {
    return pDev->GetRate(pDev);
}

static inline uint32_t DeviceIntrfSetRate(DevIntrf_t * const pDev, uint32_t Rate) {
    return pDev->SetRate(pDev, Rate);
}

int DeviceIntrfRx(DevIntrf_t * const pDev, uint32_t DevAddr, uint8_t *pBuff, int BuffLen);

int DeviceIntrfTx(DevIntrf_t * const pDev, uint32_t DevAddr, uint8_t *pData, int DataLen);

void DeviceIntrfTxComplete(DevIntrf_t * const pDev);

int DeviceIntrfRead(DevIntrf_t * const pDev, uint32_t DevAddr, uint8_t *pAdCmd, int AdCmdLen,
                    uint8_t *pRxBuff, int RxLen);

int DeviceIntrfWrite(DevIntrf_t * const pDev, uint32_t DevAddr, uint8_t *pAdCmd, int AdCmdLen,
                     uint8_t *pData, int DataLen);

static inline bool DeviceIntrfStartRx(DevIntrf_t * const pDev, uint32_t DevAddr) {
    if (atomic_flag_test_and_set(&pDev->bBusy))
        return false;

    bool retval = pDev->StartRx(pDev, DevAddr);

    // In case of returned false, app would not call Stop to release busy flag
    // so we need to do that here before returning
    if (retval == false) {
        atomic_flag_clear(&pDev->bBusy);
    }

    return retval;
}

static inline int DeviceIntrfRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen) {
    return pDev->RxData(pDev, pBuff, BuffLen);
}

static inline void DeviceIntrfStopRx(DevIntrf_t * const pDev) {
    pDev->StopRx(pDev);
    atomic_flag_clear(&pDev->bBusy);
}

// Initiate receive
// WARNING this function must be used in pair with StopTx
// Re-entrance protection flag is used
// On success, StopTx must be after transmission is completed to release flag
static inline bool DeviceIntrfStartTx(DevIntrf_t * const pDev, uint32_t DevAddr) {
    if (atomic_flag_test_and_set(&pDev->bBusy))
        return false;

    bool retval =  pDev->StartTx(pDev, DevAddr);

    // In case of returned false, app would not call Stop to release busy flag
    // so we need to do that here before returning
    if (retval == false) {
        atomic_flag_clear(&pDev->bBusy);
    }

    return retval;
}

static inline int DeviceIntrfTxData(DevIntrf_t * const pDev, uint8_t *pData, int DataLen) {
    return pDev->TxData(pDev, pData, DataLen);
}

static inline void DeviceIntrfStopTx(DevIntrf_t * const pDev) {
    pDev->StopTx(pDev);
    atomic_flag_clear(&pDev->bBusy);
}

static inline void DeviceIntrfReset(DevIntrf_t * const pDev) {
    if (pDev->Reset)
        pDev->Reset(pDev);
}

static inline void DeviceIntrfPowerOff(DevIntrf_t * const pDev) {
    if (pDev->PowerOff) pDev->PowerOff(pDev);
}

static inline DEVINTRF_TYPE DeviceIntrfGetType(DevIntrf_t * const pDev) {
    return pDev->Type;
}

static inline void *DeviceIntrfGetHandle(DevIntrf_t * const pDev) {
    return pDev->GetHandle(pDev);
}

#ifdef __cplusplus
}


class DeviceIntrf {
public:

    virtual operator DevIntrf_t * const () = 0; // Get device interface data (handle)

    virtual DEVINTRF_TYPE Type() { return DeviceIntrfGetType(*this); }

    virtual uint32_t Rate(uint32_t DataRate) = 0;

    virtual uint32_t Rate(void) = 0;

    virtual void Disable(void) { DeviceIntrfDisable(*this); }

    virtual void Enable(void) { DeviceIntrfEnable(*this); }

    void PowerOff() { DeviceIntrfPowerOff(*this); }

    virtual int Rx(uint32_t DevAddr, uint8_t *pBuff, int BuffLen) {
        return DeviceIntrfRx(*this,DevAddr, pBuff, BuffLen);
    }

    virtual int Tx(uint32_t DevAddr, uint8_t *pData, int DataLen) {
        return DeviceIntrfTx(*this, DevAddr, pData, DataLen);
    }

    virtual int Read(uint32_t DevAddr, uint8_t *pAdCmd, int AdCmdLen, uint8_t *pBuff, int BuffLen) {
        return DeviceIntrfRead(*this, DevAddr, pAdCmd, AdCmdLen, pBuff, BuffLen);
    }

    virtual int Write(uint32_t DevAddr, uint8_t *pAdCmd, int AdCmdLen, uint8_t *pData, int DataLen) {
        return DeviceIntrfWrite(*this, DevAddr, pAdCmd, AdCmdLen, pData, DataLen);
    }

    // Initiate receive
    // WARNING this function must be used in pair with StopRx
    // Re-entrance protection flag is used
    // On success, StopRx must be after transmission is completed to release flag
    virtual bool StartRx(uint32_t DevAddr) = 0;

    virtual int RxData(uint8_t *pBuff, int BuffLen) = 0;

    virtual void StopRx(void) = 0;

    // Initiate transmit
    // WARNING this function must be used in pair with StopTx
    // Re-entrance protection flag is used
    // On success, StopTx must be after transmission is completed to release flag
    virtual bool StartTx(uint32_t DevAddr) = 0;

    virtual int TxData(uint8_t *pData, int DataLen) = 0;

    // Stop transmit
    // WARNING !!!!!
    // This functions MUST ONLY be called if StartTx returns true.
    virtual void StopTx(void) = 0;

    virtual bool RequestToSend(int NbBytes) { return true; }

    virtual void Reset(void) { DeviceIntrfReset(*this); }
};

#endif

#endif  // __DEVICEINTRF_H__
```


