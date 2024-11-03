

# Group device\_intrf



[**Modules**](modules.md) **>** [**device\_intrf**](group__device__intrf.md)




















## Classes

| Type | Name |
| ---: | :--- |
| struct | [**\_\_device\_intrf**](struct____device__intrf.md) <br>_Device interface data structure._  |


## Public Types

| Type | Name |
| ---: | :--- |
| enum  | [**DEVINTRF\_EVT**](#enum-devintrf_evt)  <br>_Device interface event types._  |
| typedef enum [**\_\_Dev\_Intrf\_Type**](group__device__intrf.md#enum-__dev_intrf_type) | [**DEVINTRF\_TYPE**](#typedef-devintrf_type)  <br>_Enumerating interface types._  |
| typedef int(\* | [**DevIntrfEvtHandler\_t**](#typedef-devintrfevthandler_t)  <br>_Event handler callback._  |
| typedef struct [**\_\_device\_intrf**](struct____device__intrf.md) | [**DevIntrf\_t**](#typedef-devintrf_t)  <br>_Device Interface forward data structure type definition. This structure is the base object. Pointer to an instance of this is passed to all function calls. See structure definition bellow for more details._  |
| enum  | [**\_\_Dev\_Intrf\_Type**](#enum-__dev_intrf_type)  <br>_Enumerating interface types._  |




















## Public Functions

| Type | Name |
| ---: | :--- |
|  int | [**DeviceIntrfRead**](#function-deviceintrfread) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev, uint32\_t DevAddr, uint8\_t \* pAdCmd, int AdCmdLen, uint8\_t \* pRxBuff, int RxLen) <br>_Device read transfer._  |
|  int | [**DeviceIntrfRx**](#function-deviceintrfrx) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev, uint32\_t DevAddr, uint8\_t \* pBuff, int BuffLen) <br>_Full receive data sequence._  |
|  int | [**DeviceIntrfTx**](#function-deviceintrftx) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev, uint32\_t DevAddr, uint8\_t \* pData, int DataLen) <br>_Full transmit data sequence._  |
|  void | [**DeviceIntrfTxComplete**](#function-deviceintrftxcomplete) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev) <br>_Signal Tx transfer completed._  |
|  int | [**DeviceIntrfWrite**](#function-deviceintrfwrite) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev, uint32\_t DevAddr, uint8\_t \* pAdCmd, int AdCmdLen, uint8\_t \* pData, int DataLen) <br>_Device write transfer._  |


## Public Static Functions

| Type | Name |
| ---: | :--- |
|  void | [**DeviceIntrfDisable**](#function-deviceintrfdisable) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev) <br>_Disable interface. Put the interface in lowest power mode._  |
|  void | [**DeviceIntrfEnable**](#function-deviceintrfenable) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev) <br>_Wake up the interface._  |
|  void \* | [**DeviceIntrfGetHandle**](#function-deviceintrfgethandle) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev) <br> |
|  uint32\_t | [**DeviceIntrfGetRate**](#function-deviceintrfgetrate) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev) <br>_Get data rate of the interface in Hertz. This is not a clock frequency but rather the transfer frequency (number of transfers per second). It has meaning base on the implementation as bits/sec or bytes/sec or whatever the case._  |
|  [**DEVINTRF\_TYPE**](group__device__intrf.md#typedef-devintrf_type) | [**DeviceIntrfGetType**](#function-deviceintrfgettype) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev) <br>_Get interface type._  |
|  void | [**DeviceIntrfPowerOff**](#function-deviceintrfpoweroff) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev) <br>_Power off interface completely for power saving._  |
|  void | [**DeviceIntrfReset**](#function-deviceintrfreset) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev) <br>_This function perform a reset of interface._  |
|  int | [**DeviceIntrfRxData**](#function-deviceintrfrxdata) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev, uint8\_t \* pBuff, int BuffLen) <br>_Receive data into pBuff passed in parameter. Assuming StartRx was called prior calling this function to get the actual data._  |
|  uint32\_t | [**DeviceIntrfSetRate**](#function-deviceintrfsetrate) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev, uint32\_t Rate) <br>_Set data rate of the interface in Hertz._  |
|  bool | [**DeviceIntrfStartRx**](#function-deviceintrfstartrx) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev, uint32\_t DevAddr) <br>_Prepare start condition to receive data with subsequence RxData._  |
|  bool | [**DeviceIntrfStartTx**](#function-deviceintrfstarttx) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev, uint32\_t DevAddr) <br>_Prepare start condition to transfer data with subsequence TxData._  |
|  void | [**DeviceIntrfStopRx**](#function-deviceintrfstoprx) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev) <br>_Completion of read data phase._  |
|  void | [**DeviceIntrfStopTx**](#function-deviceintrfstoptx) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev) <br>_Completion of sending data via TxData._  |
|  int | [**DeviceIntrfTxData**](#function-deviceintrftxdata) ([**DevIntrf\_t**](group__device__intrf.md#typedef-devintrf_t) \*const pDev, uint8\_t \* pData, int DataLen) <br>_Transfer data from pData passed in parameter. Assuming StartTx was called prior calling this function to send the actual data._  |


























## Public Types Documentation




### enum DEVINTRF\_EVT 

```
enum DEVINTRF_EVT {
    DEVINTRF_EVT_RX_TIMEOUT,
    DEVINTRF_EVT_RX_DATA,
    DEVINTRF_EVT_RX_FIFO_FULL,
    DEVINTRF_EVT_TX_TIMEOUT,
    DEVINTRF_EVT_TX_READY,
    DEVINTRF_EVT_TX_FIFO_EMPTY,
    DEVINTRF_EVT_STATECHG,
    DEVINTRF_EVT_READ_RQST,
    DEVINTRF_EVT_WRITE_RQST,
    DEVINTRF_EVT_COMPLETED
};
```




<hr>



### typedef DEVINTRF\_TYPE 

```
typedef enum __Dev_Intrf_Type DEVINTRF_TYPE;
```




<hr>



### typedef DevIntrfEvtHandler\_t 

_Event handler callback._ 
```
typedef int(* DevIntrfEvtHandler_t) (DevIntrf_t *const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len);
```



This is normally being called within interrupts, avoid blocking




**Parameters:**


* `pDev` : Device handle 
* `EvtId` : Event code 
* `pBuffer` : In/Out Buffer containing data
 on DEVINTRF\_EVT\_RX\_TIMEOUT & DEVINTRF\_EVT\_RXDATA, pBuffer contains data received. If driver implements CFIFO, this parameter is NULL with BufferLen indicating total data in FIFO.
 on DEVINTRF\_EVT\_TX\_READY, pBuffer contains data to be transmit with max length BufferLen. If driver implements CFIFO, this parameter is NULL and BufferLen indicates amount of data stored in FIFO
 on DEVINTRF\_EVT\_STATECHG, pBuffer contains state data. This is implementation specific for example UART implementation would contains line state info.
* `Len` : Max buffer length. See above description



**Returns:**

Number of bytes processed. Implementation specific.
 in case of FIFO\_FULL events, FIFO will be pushed out if return value is zero 





        

<hr>



### typedef DevIntrf\_t 

```
typedef struct __device_intrf DevIntrf_t;
```




<hr>



### enum \_\_Dev\_Intrf\_Type 

```
enum __Dev_Intrf_Type {
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
    DEVINTRF_TYPE_OSPI
};
```




<hr>
## Public Functions Documentation




### function DeviceIntrfRead 

_Device read transfer._ 
```
int DeviceIntrfRead (
    DevIntrf_t *const pDev,
    uint32_t DevAddr,
    uint8_t * pAdCmd,
    int AdCmdLen,
    uint8_t * pRxBuff,
    int RxLen
) 
```



A device read transfer usually starts with a write of a command or register address. Then follows with a read data results. This function encapsulate that functionality.




**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface 
* `DevAddr` : The device selection id scheme 
* `pAdCmd` : Pointer to buffer containing address or command code to send 
* `AdCmdLen` : Size of addr/Cmd in bytes 
* `pRxBuff` : Pointer to memory area to receive data. 
* `RxLen` : Length of buffer memory in bytes



**Returns:**

Number of bytes read 





        

<hr>



### function DeviceIntrfRx 

_Full receive data sequence._ 
```
int DeviceIntrfRx (
    DevIntrf_t *const pDev,
    uint32_t DevAddr,
    uint8_t * pBuff,
    int BuffLen
) 
```



This function does full receive data sequence by calling StartRx, RxData, StopRx.




**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface 
* `DevAddr` : The device selection id scheme 
* `pBuff` : Pointer to memory area to receive data. 
* `BuffLen` : Length of buffer memory in bytes



**Returns:**

Number of bytes read 





        

<hr>



### function DeviceIntrfTx 

_Full transmit data sequence._ 
```
int DeviceIntrfTx (
    DevIntrf_t *const pDev,
    uint32_t DevAddr,
    uint8_t * pData,
    int DataLen
) 
```



This function does full transmit data sequence by calling StartTx, TxData, StopTx.




**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface 
* `DevAddr` : The device selection id scheme 
* `pData` : Pointer to data to send. 
* `DataLen` : Length of data in bytes



**Returns:**

Number of bytes read 





        

<hr>



### function DeviceIntrfTxComplete 

_Signal Tx transfer completed._ 
```
void DeviceIntrfTxComplete (
    DevIntrf_t *const pDev
) 
```



This is useful for interrupt based transfer




**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface 




        

<hr>



### function DeviceIntrfWrite 

_Device write transfer._ 
```
int DeviceIntrfWrite (
    DevIntrf_t *const pDev,
    uint32_t DevAddr,
    uint8_t * pAdCmd,
    int AdCmdLen,
    uint8_t * pData,
    int DataLen
) 
```



A device write transfer usually starts with a write of a command or register address. Then follows with a write data. This function encapsulate that functionality.




**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface 
* `DevAddr` : The device selection id scheme 
* `pAdCmd` : Pointer to buffer containing address or command code to send 
* `AdCmdLen` : Size of addr/Cmd in bytes 
* `pData` : Pointer to data to send. 
* `DataLen` : Length of data in bytes



**Returns:**

Number of bytes of data sent (not counting the Addr/Cmd). 





        

<hr>
## Public Static Functions Documentation




### function DeviceIntrfDisable 

_Disable interface. Put the interface in lowest power mode._ 
```
static inline void DeviceIntrfDisable (
    DevIntrf_t *const pDev
) 
```



If this is a physical interface, provide a way to turn off for energy saving. Make sure the turn off procedure can be turned back on without going through the full initialization sequence




**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface 




        

<hr>



### function DeviceIntrfEnable 

_Wake up the interface._ 
```
static inline void DeviceIntrfEnable (
    DevIntrf_t *const pDev
) 
```





**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface 




        

<hr>



### function DeviceIntrfGetHandle 

```
static inline void * DeviceIntrfGetHandle (
    DevIntrf_t *const pDev
) 
```




<hr>



### function DeviceIntrfGetRate 

_Get data rate of the interface in Hertz. This is not a clock frequency but rather the transfer frequency (number of transfers per second). It has meaning base on the implementation as bits/sec or bytes/sec or whatever the case._ 
```
static inline uint32_t DeviceIntrfGetRate (
    DevIntrf_t *const pDev
) 
```





**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface



**Returns:**

Transfer rate per second 





        

<hr>



### function DeviceIntrfGetType 

_Get interface type._ 
```
static inline DEVINTRF_TYPE DeviceIntrfGetType (
    DevIntrf_t *const pDev
) 
```





**Returns:**

Interface type 





        

<hr>



### function DeviceIntrfPowerOff 

_Power off interface completely for power saving._ 
```
static inline void DeviceIntrfPowerOff (
    DevIntrf_t *const pDev
) 
```



This function will power off the interface completely. Not all interface provides this type of functionality. Once power off is call, full initialization cycle is required. Therefore there is no PowerOn counter part of this function contrary to the Enable/Disable functions.




**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface 




        

<hr>



### function DeviceIntrfReset 

_This function perform a reset of interface._ 
```
static inline void DeviceIntrfReset (
    DevIntrf_t *const pDev
) 
```





**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface 




        

<hr>



### function DeviceIntrfRxData 

_Receive data into pBuff passed in parameter. Assuming StartRx was called prior calling this function to get the actual data._ 
```
static inline int DeviceIntrfRxData (
    DevIntrf_t *const pDev,
    uint8_t * pBuff,
    int BuffLen
) 
```



Return -1 in case of interrupt based or transfer without waiting for completion. for example I2C where stop condition is handled asynchronously




**Parameters:**


* `pDevIntrf` : Pointer to an instance of the Device Interface 
* `pBuff` : Pointer to memory area to receive data. 
* `BuffLen` : Length of buffer memory in bytes



**Returns:**

Number of bytes read -1 special case for interrupt driven without waiting for completion for example I2C where stop condition is handled asynchronously 





        

<hr>



### function DeviceIntrfSetRate 

_Set data rate of the interface in Hertz._ 
```
static inline uint32_t DeviceIntrfSetRate (
    DevIntrf_t *const pDev,
    uint32_t Rate
) 
```



This is not a clock frequency but rather the transfer frequency (number of transfers per second). It has meaning base on the implementation as bits/sec or bytes/sec or whatever the case




**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface 
* `Rate` : Data rate to be set in Hertz (transfer per second)



**Returns:**

Actual transfer rate per second set. It is the real capable rate closest to rate being requested. 





        

<hr>



### function DeviceIntrfStartRx 

_Prepare start condition to receive data with subsequence RxData._ 
```
static inline bool DeviceIntrfStartRx (
    DevIntrf_t *const pDev,
    uint32_t DevAddr
) 
```



This can be in case such as start condition for I2C or Chip Select for SPI or precondition for DMA transfer or whatever requires it or not This function must check & set the busy state for re-entrancy


NOTE: On success StopRx must be called to release busy flag




**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface 
* `DevAddr` : The device selection id scheme



**Returns:**

true - Success
 false - failed. 





        

<hr>



### function DeviceIntrfStartTx 

_Prepare start condition to transfer data with subsequence TxData._ 
```
static inline bool DeviceIntrfStartTx (
    DevIntrf_t *const pDev,
    uint32_t DevAddr
) 
```



This can be in case such as start condition for I2C or Chip Select for SPI or precondition for DMA transfer or whatever requires it or not This function must check & set the busy state for re-entrancy


On success StopRx must be called to release busy flag




**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface 
* `DevAddr` : The device selection id scheme



**Returns:**

true - Success
 false - failed 





        

<hr>



### function DeviceIntrfStopRx 

_Completion of read data phase._ 
```
static inline void DeviceIntrfStopRx (
    DevIntrf_t *const pDev
) 
```



Do require post processing after data has been received via RxData This function must clear the busy state for re-entrancy




**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface 




        

<hr>



### function DeviceIntrfStopTx 

_Completion of sending data via TxData._ 
```
static inline void DeviceIntrfStopTx (
    DevIntrf_t *const pDev
) 
```



Perform the require post processing after all data was transmitted via TxData. This function must clear the busy state for re-entrancy




**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface 




        

<hr>



### function DeviceIntrfTxData 

_Transfer data from pData passed in parameter. Assuming StartTx was called prior calling this function to send the actual data._ 
```
static inline int DeviceIntrfTxData (
    DevIntrf_t *const pDev,
    uint8_t * pData,
    int DataLen
) 
```





**Parameters:**


* `pDev` : Pointer to an instance of the Device Interface 
* `pData` : Pointer to memory area of data to send. 
* `DataLen` : Length of data memory in bytes



**Returns:**

Number of bytes sent 





        

<hr>

------------------------------


