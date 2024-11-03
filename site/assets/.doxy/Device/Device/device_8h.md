

# File device.h



[**FileList**](files.md) **>** [**include**](dir_d44c64559bbebec7f509842c48db8b23.md) **>** [**device.h**](device_8h.md)

[Go to the source code of this file](device_8h_source.md)

_Generic device base class._ [More...](#detailed-description)

* `#include <stdint.h>`
* `#include <stdbool.h>`
* `#include "coredev/iopincfg.h"`
* `#include "coredev/timer.h"`
* `#include "device_intrf.h"`

















## Public Types

| Type | Name |
| ---: | :--- |
| typedef enum [**\_\_Dev\_Interrupt\_Polarity**](device_8h.md#enum-__dev_interrupt_polarity) | [**DEVINTR\_POL**](#typedef-devintr_pol)  <br>_Defines interrupt pin polarity of the device._  |
| typedef enum \_\_Device\_Event | [**DEV\_EVT**](#typedef-dev_evt)  <br> |
| enum  | [**\_\_Dev\_Interrupt\_Polarity**](#enum-__dev_interrupt_polarity)  <br>_Defines interrupt pin polarity of the device._  |
| enum  | [**\_\_Device\_Event**](#enum-__device_event)  <br> |
















































# Detailed Description






This is the base class to implement all sort devices, hardware or software. For example a sensor device or a software audio/video decoder. The device can transfer data via it's DeviceIntrf object.


Important NOTE : For performance, there is no pointer or parameter validation at this low level layer. It is the responsibility of caller to pre-validate all access




**Author:**

Hoang Nguyen Hoan 




**Date:**

Feb. 12, 2017


@license


Copyright (c) 2017, I-SYST inc., all rights reserved


Permission to use, copy, modify, and distribute this software for any purpose with or without fee is hereby granted, provided that the above copyright notice and this permission notice appear in all copies, and none of the names : I-SYST or its contributors may be used to endorse or promote products derived from this software without specific prior written permission.


For info or contributing contact : hnhoan at i-syst dot com


THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS `AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



 


    
## Public Types Documentation




### typedef DEVINTR\_POL 

_Defines interrupt pin polarity of the device._ 
```C++
typedef enum __Dev_Interrupt_Polarity DEVINTR_POL;
```



Many hardware devices can have interrupt pin polarity configurable. 


        

<hr>



### typedef DEV\_EVT 

```C++
typedef enum __Device_Event DEV_EVT;
```




<hr>



### enum \_\_Dev\_Interrupt\_Polarity 

_Defines interrupt pin polarity of the device._ 
```C++
enum __Dev_Interrupt_Polarity {
    DEVINTR_POL_LOW,
    DEVINTR_POL_HIGH
};
```



Many hardware devices can have interrupt pin polarity configurable. 


        

<hr>



### enum \_\_Device\_Event 

```C++
enum __Device_Event {
    DEV_EVT_DATA_RDY
};
```




<hr>

------------------------------
The documentation for this class was generated from the following file `include/device.h`

