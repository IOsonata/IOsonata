![IOsonata Logo](/docs/logo/iosonata_logo_400.png)

# IOsonata 🎶 — Make Your I/Os Sing  
-----

IOsonata is a high-performance, vendor-agnostic, object-oriented C++ embedded framework for building reliable, efficient, and scalable firmware.

It's not another heavyweight RTOS or a simple prototyping tool; it's a complete **professional workshop** for developers who need to build sophisticated custom hardware. It provides a set of high-quality, interoperable drivers and a flexible architecture that lets you choose the right tool for the job.

## 🌟 Why Choose IOsonata? The Professional's Sweet Spot

IOsonata was designed to fill the gap between simple prototyping tools and large, industrial-scale RTOSes. It provides the power needed for professional products without the heavy overhead and steep learning curve.

  * ✅ **Unmatched Architectural Flexibility** You are not locked into one programming model. Choose the best fit for your project:

      * Run in a simple **bare-metal loop** for maximum performance and minimal footprint.
      * Use a non-blocking, **event-driven model** for real-time efficiency.
      * Seamlessly integrate with a third-party RTOS like **FreeRTOS** for full pre-emptive multitasking when you need it.

  * ✅ **Radically Simple Board Portability** Support a new custom board in minutes, not days. With IOsonata, you only need to edit a **single `board.h` file** to define your pinout. No need to create a complex Board Support Package (BSP). A validated MCU port works instantly on any board using that chip.

  * ✅ **Proven in Demanding Industries** IOsonata is not a theoretical framework. It has been successfully deployed in safety-conscious and high-reliability fields, providing a robust and trustworthy foundation for:

      * ⚕️ **Medical Devices (FDA-Approved)**
      * 🔬 **Industrial Chemical Instruments**
      * 🚁 **Avionics Systems**

  * ✅ **Elegant, Object-Oriented Design** A clean, compositional C++ architecture promotes code reuse and maintainability. High-level drivers (`LedPwm`, `Buzzer`) are built from lower-level peripheral drivers (`Pwm`, `SPI`), leading to a more scalable and understandable codebase.

  * ✅ **Dynamic & Simple Configuration** Configure all your peripherals and stacks with powerful C structs directly in your application code. This allows for dynamic, runtime configuration without the need for complex, static build systems like the Device Tree.

  * ✅ **Truly Vendor-Agnostic** A single, unified framework with its own HAL for multiple MCU vendors (**Nordic, ST, NXP,** etc.) and first-class support for desktop OSes (**macOS, Linux, Windows**) for testing and native applications.



## 💾 Installation

Get started by running the setup script in your terminal to install the necessary toolchains and dependencies for your operating system.

### macOS

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/master/Installer/install_iocdevtools_macos.sh)"
```

### Linux

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/IOsonata/IOsonata/master/Installer/install_iocdevtools_linux.sh)"
```

### Windows (PowerShell)

⚠️ Important: You must run PowerShell as an Administrator. Right-click the PowerShell icon in your Start Menu and select "Run as administrator".


```powershell
powershell -ExecutionPolicy Bypass -Command "iex ((New-Object System.Net.WebClient).DownloadString('https://raw.githubusercontent.com/IOsonata/IOsonata/master/Installer/install_iocdevtools_win.ps1'))"
```


## 🚀 Quick Start: The 5-Minute Blinky

IOsonata can be as simple as the most intuitive beginner frameworks. Here’s all you need to blink an LED on your custom board.

**1. Define your board's LED pin in `board.h`:**

```c
// my_custom_board/board.h
#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"

#define LED_PORT      IOPORTB // MCU Port B
#define LED_PIN       12      // MCU Pin 12
#define LED_ACTIVE    1       // LED is active high

#endif // __BOARD_H__
```

**2. Write your application in `main.cpp`:**

```cpp
// main.cpp
#include "idelay.h"
#include "miscdev/led.h"
#include "board.h"

Led g_Led;

int main() {
    // Initialize the LED using the definitions from board.h
    g_Led.Init(LED_PORT, LED_PIN, (LED_LOGIC)LED_ACTIVE);

    while (1) {
        g_Led.Toggle();
        msDelay(500);
    }
    return 0;
}
```

## 🛠️ The IOsonata Ecosystem

  * **Broad IDE Support:** Develop your way with pre-configured projects for Eclipse/GCC, IAR, Keil M-DK, and more.
  * **Middleware Integration:** Comes with built-in support for popular middleware like `fatfs`, `littlefs`, and `lwip`.

## 🤝 Contributing

We welcome contributions\! Please see the [Issues](https://github.com/IOsonata/IOsonata/issues) to report bugs or request features. Pull requests for new drivers and board examples are appreciated.

## 🛡️ License

IOsonata is licensed under the **MIT License**. It is free to use in both open-source and commercial projects.

-----
## 📂 Folder Structure
IOsonata follows a **layered folder structure**:  

- **Generic Layer**: Common APIs and sources (`/include`, `/src`)  
- **Architecture Layer**: ARM Cortex, Linux, OSX, Windows  
- **Vendor Layer**: Nordic, NXP, ST, etc.  
- **Target Layer**: MCU-specific libraries + example projects  

This ensures **clean separation of abstraction and implementation** while keeping portability.  

---

## 🧪 Example: Blink
Each MCU target includes example projects (`Blink`, UART, SPI, etc.) in multiple IDE formats:  
- Eclipse/GCC  
- IAR  
- Keil uVision  
- CrossWorks  

Compile, flash, and run in minutes.  

---

## 🤝 Contributing
We welcome contributions!  
- Report issues → [Issues](https://github.com/IOsonata/IOsonata/issues)  
- Add board support or drivers via PRs  
- Share tutorials/blog posts using IOsonata  

---

## 🛡️ License
MIT License — free to use in open-source and commercial projects.  

---

## 🌍 About I-SYST
IOsonata is maintained by **[I-SYST Inc.](https://www.i-syst.com)**, a Canadian company with 10+ years of embedded experience.  
Our modules and software power **award-winning IoT, industrial, and health tech products** worldwide.  

---

Although this library supports multiple IDE/Compilers, the preferred IDE is still Eclipse/GCC.  GCC is the de facto standard for embedded software development. Eclipse is 100% free and the most flexible IDE.  It could be little overwhelming for newbies at first (like any other IDE if you are new to it anyway).

For desktop PC version of the library, native compiler and IDE are used.  XCode for OSX, Visual Studio for Windows, Eclipse for Linux.

--- 
### IDE limiations :

* Eclipse & GCC : Full C++ support, full file io support.
* IAR : Full C++ support, no system support for file io.  File io only available with semihosting. Bug in IAR : It cannot debug or flash Nordic nRF series using CMSIS-DAP. 
* uVision : Requires compiler version 6. Minimal support for file IO.  However uVision can be configured to GCC instead.
* CrossWorks : GCC C++ is stripped down to bare bone, no file io support, no atomic support and many others. In order to use full GCC C++, CrossWorks must be configured to use an external compiler.
* Segger Stusio : Stripped down version of CrossWorks.  Even less functional. Only supports jlink, cannot be used with any other jtag. SES is not recommended for heavy firmware development. 

--- 
### Required installation of external SDK and libraries :

In order to compile the IOsonata target libraries these external SDK & lib are required. Follow the instructions below to download and install into appropriate folder locations and naming. 
 
 
[nRF5_SDK](https://github.com/IOsonata/nRF5_SDK)  : Nordic nRF5x Bluetooth Low Energy

[nRF5_SDK_Mesh](https://github.com/IOsonata/nRF5_SDK_Mesh) : Nordic nRF5 SDK for Bluetoth Mesh

[nrfx](https://github.com/NordicSemiconductor/nrfx) : Nordic nrfx for nRF series MCU

[ICM-20948 Motion_Driver](https://invensense.tdk.com/developers/software-downloads/) : Create a user at https://invensense.tdk.com/developers/software-downloads/. In the "Development Kits" block, download "DK-20948 SmartMotion eMD 1.1.0". Unzip the downloaded file and navigate to EMD-Core/sources. Copy the folder `Invn` to `external/Invn` as indicated in the folder tree bellow.

[BSEC](https://github.com/boschsensortec/Bosch-BSEC2-Library) : Bosch Sensortec Environmental Cluster (BSEC) Software for #BME680 environmental sensor.  BSEC is needed for calculating Air Quality Index.  Go to https://github.com/boschsensortec/Bosch-BSEC2-Library. Clone it to `external/BSEC` as indicated in the folder tree below.  
 
[LWIP](https://savannah.nongnu.org/projects/lwip/) : A Lightweight TCP/IP stack. This library is required for IoT network connectivity over Ethernet, Wifi, LTE, ... Download it via this link https://download.savannah.nongnu.org/releases/lwip/. Rename the extracted folder as `lwip` and copy it to `external`.

--- 

<p align="center"> 
 
### IDK-BLYST-NANO : nRF52832 Bluetooth 5.2/Bluetooth Mesh development board with builtin IDAP-M CMSIS-DAP Debug JTag  
  
![IDK-BLYST-NANO](https://www.i-syst.com/images/IDK-BLYST-NANO_photo640.png) 

[Buy : IDK-BLYST-NANO (BLYST Nano development board)](https://www.tindie.com/products/hnhoan/bluetooth5mesh-nrf52832-arm-m4f-nano-devkit/).  
 
</p> 



![BLUEIO-TAG-EVIM](https://www.i-syst.com/images/BLUEIO-TAG-EVIM_page.png) 
 
<p align="center"> 
  
[Buy : BLUEIO-TAG-EVIM (BLYST Nano sensor board)](https://www.crowdsupply.com/i-syst/blyst-nano).  
[Nordic Thingy App compatible firmware project](https://github.com/IOsonata/IOsonata/tree/master/ARM/Nordic/nRF52/nRF52832/exemples/BlueIOThingy) 
 
</p> 
 
--- 
### IOsonata folder structure

 
The way the IOsonata folder is structured is simple.  The deeper you go inside the more it is specific to the architecture or platform.  The parent folder contains all that is commonly available to the child folder.  Which means, source file from child folder can access any source in the upper parent folder but not the other way around.  This is the way to keep the abstraction separated from implementation and easier to keep track of things.


```
/your_root     - Development root directory
 |-- external        - Contains downloaded SDKs from silicon vendors
 |   |-- nRF5_SDK        - Latest Nordic SDK (https://developer.nordicsemi.com)
 |   |-- nRF5_SDK_Mesh   - Latest Nordic SDK for Mesh (https://www.nordicsemi.com/eng/nordic/Products/nRF5-SDK-for-Mesh/nRF5-SDK-for-Mesh/62377)
 |   |-- BSEC            - Bosch Sensortec Environmental Cluster (BSEC) Software (https://www.bosch-sensortec.com/bst/products/all_products/bsec) for #BME680
 |   |-- Invn            - Invensense SmartMotion Driver (download https://www.invensense.com/developers) 
 |   |   |-- Devices
 |   |   |...
 |   |-- lwip            - Lightweight TCP/IP stack (download https://download.savannah.nongnu.org/releases/lwip/)
 |   |-- Others as require
 |   |...
 |   |
 |-- IOsonata      - Put IOsonata here
 |   |-- include     - Generic include common to all platforms
 |   |   |-- bluetooth   - Generic definition for Bluetooth
 |   |   |-- converters  - Generic definition for ADV, DAC, etc...
 |   |   |-- coredev     - Generic definition MCU builtin devices such as i2c, uart, spi, timer, etc...
 |   |   |-- miscdev     - Generic definition for other non categorized devices
 |   |   |-- sensors     - Generic definition for all sort of sensors (environmental, motion, etc...)
 |   |   |-- usb         - Generic definition for USB
 |   |   |...
 |   |-- src         - Generic implementation source common to all platforms
 |   |   |-- bluetooth   - Generic source for Bluetooth
 |   |   |-- converters  - Generic source for ADV, DAC, etc...
 |   |   |-- coredev     - Generic source for MCU builtin devices such as i2c, uart, spi, timer, etc...
 |   |   |-- miscdev     - Generic source for other non categorized devices
 |   |   |-- sensors     - Generic source for all sort of sensors (environmental, motion, etc...)
 |   |   |-- usb         - Generic source for USB
 |   |   |...
 |   |    
 |   |-- ARM         - ARM series based MCU
 |   |   |-- include     - Common include for all ARM platform
 |   |   |-- src         - Common source for all ARM platform
 |   |   |-- DbgConfig   - Debugger configuration files.
 |   |   |-- ldscript    - Linker script files
 |   |   |
 |   |   |-- Nordic      - Nordic Semiconductor based  MCU
 |   |   |   |-- nRF51        - nRF51 series MCU
 |   |   |   |   |-- include     - Common include for this target series
 |   |   |   |   |-- src         - Common source for this target series
 |   |   |   |   |-- lib        - IOsonata library for this target
 |   |   |   |   |   |-- Eclipse   - Eclipse project for this lib
 |   |   |   |   |   |-- IAR       - IAR project for this lib
 |   |   |   |   |   |-- CrossWorks- CrossWorks project for this lib
 |   |   |   |   |   |...
 |   |   |   |   |   
 |   |   |   |   |-- exemples   - Example projects for this target
 |   |   |   |   |   |-- Blink     - Blink example
 |   |   |   |   |   |   |-- src      - Source code for this exaple
 |   |   |   |   |   |   |-- Eclipse  - Eclipse project for this example
 |   |   |   |   |   |   |-- IAR      - IAR project for this example
 |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this example
 |   |   |   |   |   |   |   |...
 |   |   |   |   |   |-- Many other examples same
 |   |   |   |   
 |   |   |   |-- nRF52        - nRF52 serie MCU
 |   |   |   |   |-- include     - Common include for this target series
 |   |   |   |   |-- src         - Common source for this target series
 |   |   |   |   |-- nRF52832    - Target MCU
 |   |   |   |   |   |-- lib        - IOsonata library for this target
 |   |   |   |   |   |   |-- Eclipse   - Eclipse project for this lib
 |   |   |   |   |   |   |-- IAR       - IAR project for this lib
 |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this lib
 |   |   |   |   |   |   |...
 |   |   |   |   |   |   
 |   |   |   |   |   |-- exemples   - Example projects for this target
 |   |   |   |   |   |   |-- Blink     - Blink example
 |   |   |   |   |   |   |   |-- src      - Source code for this exaple
 |   |   |   |   |   |   |   |-- Eclipse  - Eclipse project for this example
 |   |   |   |   |   |   |   |-- IAR      - IAR project for this example
 |   |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this example
 |   |   |   |   |   |   |   |...
 |   |   |   |   |   |   |-- Many other examples same
 |   |   |   |   |   |   |
 |   |   |   |   |-- nRF52840    - Target MCU
 |   |   |   |   |   |-- lib        - IOsonata library for this target
 |   |   |   |   |   |   |-- Eclipse   - Eclipse project for this lib
 |   |   |   |   |   |   |-- IAR       - IAR project for this lib
 |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this lib
 |   |   |   |   |   |   |...
 |   |   |   |   |   |   
 |   |   |   |   |   |-- exemples   - Example projects for this target
 |   |   |   |   |   |   |-- Blink     - Blink example
 |   |   |   |   |   |   |   |-- src      - Source code for this exaple
 |   |   |   |   |   |   |   |-- Eclipse  - Eclipse project for this example
 |   |   |   |   |   |   |   |-- IAR      - IAR project for this example
 |   |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this example
 |   |   |   |   |   |   |   |...
 |   |   |   |   |   |   |-- Many other examples same
 |   |   |   |   
 |   |   |   |-- nRF53        - nRF53 series MCU
 |   |   |   |   |-- include     - Common include for this target series
 |   |   |   |   |-- src         - Common source for this target series
 |   |   |   |   |-- nRF5340_App   	- Target MCU
 |   |   |   |   |   |-- lib        - IOsonata library for this target
 |   |   |   |   |   |   |-- Eclipse   - Eclipse project for this lib
 |   |   |   |   |   |   |-- IAR       - IAR project for this lib
 |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this lib
 |   |   |   |   |   |   |...
 |   |   |   |   |   |   
 |   |   |   |   |   |-- exemples   - Example projects for this target
 |   |   |   |   |   |   |-- Blink     - Blink example
 |   |   |   |   |   |   |   |-- src      - Source code for this exaple
 |   |   |   |   |   |   |   |-- Eclipse  - Eclipse project for this example
 |   |   |   |   |   |   |   |-- IAR      - IAR project for this example
 |   |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this example
 |   |   |   |   |   |   |   |...
 |   |   |   |   |   |   |-- Many other examples same
 |   |   |   |   |   |   |
 |   |   |   |   |   
 |   |   |   |   |-- nRF5340_Net   	- Target MCU
 |   |   |   |   |   |-- lib        - IOsonata library for this target
 |   |   |   |   |   |   |-- Eclipse   - Eclipse project for this lib
 |   |   |   |   |   |   |-- IAR       - IAR project for this lib
 |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this lib
 |   |   |   |   |   |   |...
 |   |   |   |   |   |   
 |   |   |   |   |   |-- exemples   - Example projects for this target
 |   |   |   |   |   |   |-- Blink     - Blink example
 |   |   |   |   |   |   |   |-- src      - Source code for this exaple
 |   |   |   |   |   |   |   |-- Eclipse  - Eclipse project for this example
 |   |   |   |   |   |   |   |-- IAR      - IAR project for this example
 |   |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this example
 |   |   |   |   |   |   |   |...
 |   |   |   |   |   |   |-- Many other examples same
 |   |   |   |   |   |   |
 |   |   |   |   
 |   |   |   |-- nRF91        - nRF91 series MCU
 |   |   |   |   |-- include     - Common include for this target series
 |   |   |   |   |-- src         - Common source for this target series
 |   |   |   |   |-- nRF9160     - Target MCU
 |   |   |   |   |   |-- lib        - IOsonata library for this target
 |   |   |   |   |   |   |-- Eclipse   - Eclipse project for this lib
 |   |   |   |   |   |   |-- IAR       - IAR project for this lib
 |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this lib
 |   |   |   |   |   |   |...
 |   |   |   |   |   |   
 |   |   |   |   |   |-- exemples   - Example projects for this target
 |   |   |   |   |   |   |-- Blink     - Blink example
 |   |   |   |   |   |   |   |-- src      - Source code for this exaple
 |   |   |   |   |   |   |   |-- Eclipse  - Eclipse project for this example
 |   |   |   |   |   |   |   |-- IAR      - IAR project for this example
 |   |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this example
 |   |   |   |   |   |   |   |...
 |   |   |   |   |   |   |-- Many other examples same
 |   |   |   |   |   |   |
 |   |   |
 |   |   |-- NXP         - NXP based MCU
 |   |   |   |-- LPC11xx      - LPC11xx series MCU
 |   |   |   |   |-- include     - Common include for this target series
 |   |   |   |   |-- src         - Common source for this target series
 |   |   |   |   |-- LPC11U35    - LPC11U35 target
 |   |   |   |   |   |-- lib        - IOsonata library for this target
 |   |   |   |   |   |   |-- Eclipse   - Eclipse project for this lib
 |   |   |   |   |   |   |-- IAR       - IAR project for this lib
 |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this lib
 |   |   |   |   |   |   |...
 |   |   |   |   |   |   
 |   |   |   |   |   |-- exemples   - Example projects for this target
 |   |   |   |   |   |   |-- Blink     - Blink example
 |   |   |   |   |   |   |   |-- src      - Source code for this exaple
 |   |   |   |   |   |   |   |-- Eclipse  - Eclipse project for this example
 |   |   |   |   |   |   |   |-- IAR      - IAR project for this example
 |   |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this example
 |   |   |   |   |   |   |   |...
 |   |   |   |   |   |   |-- Many other examples same
 |   |   |   |   |   |   |
 |   |   |   |-- LPC17xx      - LPC17xx series MCU
 |   |   |   |   |-- include     - Common include for this target series
 |   |   |   |   |-- src         - Common source for this target series
 |   |   |   |   |-- LPC176x     - LPC176x target
 |   |   |   |   |   |-- lib        - IOsonata library for this target
 |   |   |   |   |   |   |-- Eclipse   - Eclipse project for this lib
 |   |   |   |   |   |   |-- IAR       - IAR project for this lib
 |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this lib
 |   |   |   |   |   |   |...
 |   |   |   |   |   |   
 |   |   |   |   |   |-- exemples   - Example projects for this target
 |   |   |   |   |   |   |-- Blink     - Blink example
 |   |   |   |   |   |   |   |-- src      - Source code for this exaple
 |   |   |   |   |   |   |   |-- Eclipse  - Eclipse project for this example
 |   |   |   |   |   |   |   |-- IAR      - IAR project for this example
 |   |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this example
 |   |   |   |   |   |   |   |...
 |   |   |   |   |   |   |-- Many other examples same
 |   |   |   |   |   |   |
 |   |   |
 |   |   |-- ST          - ST based MCU
 |   |   |   |-- STM32F0xx
 |   |   |   |-- STM32F4xx
 |   |   |   |-- STM32L0xx
 |   |   |   |-- STM32L1xx
 |   |   |   |-- STM32L4xx
 |   |   |   |   |-- include     - Common include for this target series
 |   |   |   |   |-- src         - Common source for this target series
 |   |   |   |   |-- STM32L476      - Target MCU
 |   |   |   |   |   |-- lib        - IOsonata library for this target
 |   |   |   |   |   |   |-- Eclipse   - Eclipse project for this lib
 |   |   |   |   |   |   |-- IAR       - IAR project for this lib
 |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this lib
 |   |   |   |   |   |   |...
 |   |   |   |   |   |   
 |   |   |   |   |   |-- exemples   - Example projects for this target
 |   |   |   |   |   |   |-- Blink     - Blink example
 |   |   |   |   |   |   |   |-- src      - Source code for this exaple
 |   |   |   |   |   |   |   |-- Eclipse  - Eclipse project for this example
 |   |   |   |   |   |   |   |-- IAR      - IAR project for this example
 |   |   |   |   |   |   |   |-- CrossWorks- CrossWorks project for this example
 |   |   |   |   |   |   |   |...
 |   |   |   |   |   |   |-- Many other examples same
 |   |   |   |   |   |   |
 |   |   |
 |   |   |-- Other silicon vendors
 |   |...
 |   |-- Linux
 |   |   |...
 |   |-- OSX
 |   |   |...
 |   |-- Win
 |   |   |...
 | ...
```
 
