

# Struct \_\_device\_intrf



[**ClassList**](annotated.md) **>** [**\_\_device\_intrf**](struct____device__intrf.md)



_Device interface data structure._ [More...](#detailed-description)

* `#include <device_intrf.h>`





















## Public Attributes

| Type | Name |
| ---: | :--- |
|  void(\* | [**Disable**](#variable-disable)  <br>_Put the interface to sleep for maximum energy saving._  |
|  atomic\_int | [**EnCnt**](#variable-encnt)  <br> |
|  void(\* | [**Enable**](#variable-enable)  <br>_Wake up the interface._  |
|  [**DevIntrfEvtHandler\_t**](group__device__intrf.md#typedef-devintrfevthandler_t) | [**EvtCB**](#variable-evtcb)  <br>_Interrupt based event callback function pointer. Must be set to NULL if not used._  |
|  void \*(\* | [**GetHandle**](#variable-gethandle)  <br> |
|  uint32\_t(\* | [**GetRate**](#variable-getrate)  <br>_Get data rate of the interface in Hertz. This is not a clock frequency but rather the transfer frequency (number of transfers per second). It has meaning base on the implementation as bits/sec or bytes/sec or whatever the case._  |
|  int | [**IntPrio**](#variable-intprio)  <br>_Interrupt priority. Value is implementation specific._  |
|  int | [**MaxRetry**](#variable-maxretry)  <br>_Max retry when data could not be transfered (Rx/Tx returns zero count)_  |
|  void(\* | [**PowerOff**](#variable-poweroff)  <br>_Power off device for power saving._  |
|  void(\* | [**Reset**](#variable-reset)  <br>_This function perform a reset of interface. Must provide empty function of not used._  |
|  int(\* | [**RxData**](#variable-rxdata)  <br>_Receive data into pBuff passed in parameter. Assuming StartRx was called prior calling this function to get the actual data._  |
|  uint32\_t(\* | [**SetRate**](#variable-setrate)  <br>_Set data rate of the interface in Hertz. This is not a clock frequency but rather the transfer frequency (number of transfers per second). It has meaning base on the implementation as bits/sec or bytes/sec or whatever the case._  |
|  bool(\* | [**StartRx**](#variable-startrx)  <br>_Prepare start condition to receive data with subsequence RxData. This can be in case such as start condition for I2C or Chip Select for SPI or precondition for DMA transfer or whatever requires it or not This function must check & set the busy state for re-entrancy._  |
|  bool(\* | [**StartTx**](#variable-starttx)  <br>_Prepare start condition to transfer data with subsequence TxData. This can be in case such as start condition for I2C or Chip Select for SPI or precondition for DMA transfer or whatever requires it or not This function must check & set the busy state for re-entrancy._  |
|  void(\* | [**StopRx**](#variable-stoprx)  <br>_Completion of read data phase. Do require post processing after data has been received via RxData This function must clear the busy state for reentrancy._  |
|  void(\* | [**StopTx**](#variable-stoptx)  <br>_Completion of sending data via TxData. Do require post processing after all data was transmitted via TxData. This function must clear the busy state for re-entrancy._  |
|  int(\* | [**TxData**](#variable-txdata)  <br>_Transfer data from pData passed in parameter. Assuming StartTx was called prior calling this function to send the actual data._  |
|  int(\* | [**TxSrData**](#variable-txsrdata)  <br>_Transfer data from pData passed in parameter with re-start._  |
|  [**DEVINTRF\_TYPE**](group__device__intrf.md#typedef-devintrf_type) | [**Type**](#variable-type)  <br>_Identify the type of interface._  |
|  atomic\_flag | [**bBusy**](#variable-bbusy)  <br>_Busy flag to be set check and set at start and reset at end of transmission._  |
|  bool | [**bDma**](#variable-bdma)  <br>_Enable DMA transfer support. Not all hardware interface supports this feature._  |
|  bool | [**bIntEn**](#variable-binten)  <br>_Enable interrupt support. Not all hardware interface supports this feature._  |
|  atomic\_bool | [**bNoStop**](#variable-bnostop)  <br> |
|  atomic\_bool | [**bTxReady**](#variable-btxready)  <br>_Flag indicating Tx is ready for transfer._  |
|  void \* | [**pDevData**](#variable-pdevdata)  <br>_Private device interface implementation data._  |












































# Detailed Description


This structure is the actual interface for both C++ & C code It is used to provide C compatibility instead of using C++ interface which is only for C++


This data structure is visible for implementer of interface. It is seen as handle for application to pass to the interface function calls. Application firmware should not access any member of this structure directly. 


    
## Public Attributes Documentation




### variable Disable 

_Put the interface to sleep for maximum energy saving._ 
```C++
void(* __device_intrf::Disable) (DevIntrf_t *const pDevIntrf);
```



If this is a physical interface, provide a way to put the interface to sleep for maximum energy saving possible. This function must be implemented in such a way that the interface can be re-enable without going through full initialization sequence.




**Parameters:**


* `pDevIntrf` : Pointer to an instance of the Device Interface 




        

<hr>



### variable EnCnt 


```C++
atomic_int __device_intrf::EnCnt;
```



Count the number of time device is enabled, this used as ref count where multiple devices are using the same interface. It is to avoid it being disabled while another device is still using it 


        

<hr>



### variable Enable 

_Wake up the interface._ 
```C++
void(* __device_intrf::Enable) (DevIntrf_t *const pDevIntrf);
```





**Parameters:**


* `pDevIntrf` : Pointer to an instance of the Device Interface 




        

<hr>



### variable EvtCB 

```C++
DevIntrfEvtHandler_t __device_intrf::EvtCB;
```




<hr>



### variable GetHandle 

```C++
void *(* __device_intrf::GetHandle) (DevIntrf_t *const pDevIntrf);
```




<hr>



### variable GetRate 

_Get data rate of the interface in Hertz. This is not a clock frequency but rather the transfer frequency (number of transfers per second). It has meaning base on the implementation as bits/sec or bytes/sec or whatever the case._ 
```C++
uint32_t(* __device_intrf::GetRate) (DevIntrf_t *const pDevIntrf);
```





**Parameters:**


* `pDevIntrf` : Pointer to an instance of the Device Interface



**Returns:**

Transfer rate per second 





        

<hr>



### variable IntPrio 

```C++
int __device_intrf::IntPrio;
```




<hr>



### variable MaxRetry 

```C++
int __device_intrf::MaxRetry;
```




<hr>



### variable PowerOff 

_Power off device for power saving._ 
```C++
void(* __device_intrf::PowerOff) (DevIntrf_t *const pDevIntrf);
```



This function will power off device completely. Not all device provide this type of functionality. Once power off is call, full initialization cycle is required. Therefore their is no PowerOn counter part of this function contrary to the Enable/Disable functions.




**Parameters:**


* `pDevIntrf` : Pointer to an instance of the Device Interface 




        

<hr>



### variable Reset 

_This function perform a reset of interface. Must provide empty function of not used._ 
```C++
void(* __device_intrf::Reset) (DevIntrf_t *const pDevIntrf);
```





**Parameters:**


* `pDevIntrf` : Pointer to an instance of the Device Interface 




        

<hr>



### variable RxData 

_Receive data into pBuff passed in parameter. Assuming StartRx was called prior calling this function to get the actual data._ 
```C++
int(* __device_intrf::RxData) (DevIntrf_t *const pDevIntrf, uint8_t *pBuff, int BuffLen);
```



Return -1 in case of interrupt based or transfer without waiting for completion. for example I2C where stop condition is handled asynchronously




**Parameters:**


* `pDevIntrf` : Pointer to an instance of the Device Interface 
* `pBuff` : Pointer to memory area to receive data. 
* `BuffLen` : Length of buffer memory in bytes



**Returns:**

Number of bytes read -1 special case for interrupt driven without waiting for completion for example I2C where stop condition is handled asynchronously 





        

<hr>



### variable SetRate 

_Set data rate of the interface in Hertz. This is not a clock frequency but rather the transfer frequency (number of transfers per second). It has meaning base on the implementation as bits/sec or bytes/sec or whatever the case._ 
```C++
uint32_t(* __device_intrf::SetRate) (DevIntrf_t *const pDevIntrf, uint32_t Rate);
```





**Parameters:**


* `pDevIntrf` : Pointer to an instance of the Device Interface 
* `Rate` : Data rate to be set in Hertz (transfer per second)



**Returns:**

Actual transfer rate per second set. It is the real capable rate closest to rate being requested. 





        

<hr>



### variable StartRx 

_Prepare start condition to receive data with subsequence RxData. This can be in case such as start condition for I2C or Chip Select for SPI or precondition for DMA transfer or whatever requires it or not This function must check & set the busy state for re-entrancy._ 
```C++
bool(* __device_intrf::StartRx) (DevIntrf_t *const pDevIntrf, uint32_t DevAddr);
```





**Parameters:**


* `pDevIntrf` : Pointer to an instance of the Device Interface 
* `DevAddr` : The device selection id scheme



**Returns:**

true - Success
 false - failed. 





        

<hr>



### variable StartTx 

_Prepare start condition to transfer data with subsequence TxData. This can be in case such as start condition for I2C or Chip Select for SPI or precondition for DMA transfer or whatever requires it or not This function must check & set the busy state for re-entrancy._ 
```C++
bool(* __device_intrf::StartTx) (DevIntrf_t *const pDevIntrf, uint32_t DevAddr);
```





**Parameters:**


* `pDevIntrf` : Pointer to an instance of the Device Interface 
* `DevAddr` : The device selection id scheme



**Returns:**

true - Success
 false - failed 





        

<hr>



### variable StopRx 

_Completion of read data phase. Do require post processing after data has been received via RxData This function must clear the busy state for reentrancy._ 
```C++
void(* __device_intrf::StopRx) (DevIntrf_t *const pDevIntrf);
```





**Parameters:**


* `pDevIntrf` : Pointer to an instance of the Device Interface 




        

<hr>



### variable StopTx 

_Completion of sending data via TxData. Do require post processing after all data was transmitted via TxData. This function must clear the busy state for re-entrancy._ 
```C++
void(* __device_intrf::StopTx) (DevIntrf_t *const pDevIntrf);
```





**Parameters:**


* `pDevIntrf` : Pointer to an instance of the Device Interface 




        

<hr>



### variable TxData 

_Transfer data from pData passed in parameter. Assuming StartTx was called prior calling this function to send the actual data._ 
```C++
int(* __device_intrf::TxData) (DevIntrf_t *const pDevIntrf, uint8_t *pData, int DataLen);
```





**Parameters:**


* `pDevIntrf` : Pointer to an instance of the Device Interface 
* `pData` : Pointer to memory area of data to send. 
* `DataLen` : Length of data memory in bytes



**Returns:**

Number of bytes sent 





        

<hr>



### variable TxSrData 

_Transfer data from pData passed in parameter with re-start._ 
```C++
int(* __device_intrf::TxSrData) (DevIntrf_t *const pDevIntrf, uint8_t *pData, int DataLen);
```



Assuming StartTx was called prior calling this function to send the actual data. This is a special function for some I2C devices that requires writing the data into a special register for write-restart-read sequence. One of such MCU is the Atmel SAM series. The data length in this case cannot exceed 4 bytes.




**Parameters:**


* `pDevIntrf` : Pointer to an instance of the Device Interface 
* `pData` : Pointer to memory area of data to send. 
* `DataLen` : Length of data memory in bytes



**Returns:**

Number of bytes sent 





        

<hr>



### variable Type 

```C++
DEVINTRF_TYPE __device_intrf::Type;
```




<hr>



### variable bBusy 

```C++
atomic_flag __device_intrf::bBusy;
```




<hr>



### variable bDma 

```C++
bool __device_intrf::bDma;
```




<hr>



### variable bIntEn 

```C++
bool __device_intrf::bIntEn;
```




<hr>



### variable bNoStop 


```C++
atomic_bool __device_intrf::bNoStop;
```



Flag indicating a continous transfer. Usually used for read/write register value or cmd/response type. This flag is relevant only when interrupt is enabled async transfer 


        

<hr>



### variable bTxReady 

```C++
atomic_bool __device_intrf::bTxReady;
```




<hr>



### variable pDevData 

```C++
void* __device_intrf::pDevData;
```




<hr>

------------------------------
The documentation for this class was generated from the following file `include/device_intrf.h`

