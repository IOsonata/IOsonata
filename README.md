![IOsonata Logo](/docs/logo/iosonata_logo_400.png)

# IOsonata 🎶 — Make Your I/Os Sing  
**The high-performance, multi-platform C++ HAL for IoT, wearables, and embedded innovation.**

---

## 🌟 Why IOsonata?
Embedded firmware doesn’t need to be painful.  
IOsonata is an **open-source, cross-vendor hardware abstraction layer (HAL)** that makes microcontroller firmware:  

- 🚀 **Fast** → Optimized drivers proven faster than vendor SDKs.  
- 🔀 **Portable** → Same source code runs on Nordic, ST, NXP, Renesas, and more.  
- 🎯 **Simple** → Swap a `board.h` pin map, recompile, and your app runs on a new board.  
- 💻 **Multi-Platform** → Build and test on macOS, Linux, and Windows before flashing hardware.  

From prototyping to production, IOsonata helps you **spend less time fighting BSPs** and more time building products.

---

## 🚀 Features
- **Cross-Architecture Support**  
  ARM Cortex-M, Nordic nRF, NXP LPC, ST STM32, and beyond.  

- **Driver Coverage**  
  UART, I²C, SPI, PDM, ADC/DAC, Timers, USB, BLE, Sensors, and more.  

- **Performance-Proven**  
  Real-world benchmarks include **streaming uncompressed 16kHz audio over BLE**.  

- **Broad IDE Support**  
  Works with Eclipse/GCC, IAR, Keil uVision, CrossWorks, and native compilers on desktop.  

- **IoT Ready**  
  Integrates with external SDKs like Nordic nRF5, Bosch BSEC, Invensense Motion, and LWIP for connectivity.  

---

## 📚 Learn by Example
- [BLE with just a few lines of code](http://embeddedsoftdev.blogspot.com/2018/01/bluetooth-le-with-nordic-nrf51-nrf52.html)  
- [Nordic nRF51/52 firmware with Eclipse](http://embeddedsoftdev.blogspot.com/p/ehal-nrf51.html)  
- [Eclipse IDE guide](https://www.i-syst.com/article/eclipse-ide-firmware-development-iosonata)  

---

## 🛠️ Requirements
To compile IOsonata target libraries, install the following external SDKs/libs into `/external`:  

- [nRF5_SDK](https://github.com/IOsonata/nRF5_SDK) — Nordic BLE SDK  
- [nRF5_SDK_Mesh](https://github.com/IOsonata/nRF5_SDK_Mesh) — Nordic Mesh SDK  
- [nrfx](https://github.com/NordicSemiconductor/nrfx) — Nordic MCU HAL  
- [Invensense ICM-20948 Motion Driver](https://invensense.tdk.com/developers/software-downloads/)  
- [Bosch BSEC](https://github.com/boschsensortec/Bosch-BSEC2-Library) — Environmental AI/air quality  
- [LWIP](https://savannah.nongnu.org/projects/lwip/) — Lightweight TCP/IP stack  

---

## 🔌 Hardware Dev Kits
- **[IDK-BLYST-NANO](https://www.tindie.com/products/hnhoan/bluetooth5mesh-nrf52832-arm-m4f-nano-devkit/)**  
  nRF52832 Bluetooth 5.2/Mesh dev board with built-in CMSIS-DAP debugger  

- **[BLUEIO-TAG-EVIM](https://www.crowdsupply.com/i-syst/blyst-nano)**  
  Sensor board compatible with Nordic Thingy App  

---

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

💡 **Tagline:** *IOsonata — Fast, portable, and production-ready. Your firmware foundation for the next generation of IoT.*  


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
 
