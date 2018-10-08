## Table of Contents

* [Introduction](#introduction)
* [Navigating the Repository](#navigating-the-repository)
* [Required Tools](#required-tools)
* [Code Examples List](#code-examples-list)
* [References](#references)

# Introduction
This repository contains examples and demos for PSoC 6 MCU family of devices, a single chip solution for the emerging IoT devices. PSoC 6 MCU bridges the gap between expensive, power hungry application processors and low‑performance microcontrollers (MCUs). The ultra‑low‑power, dual-core architecture of PSoC 6 MCU offers the processing performance needed by IoT devices, eliminating the tradeoffs between power and performance.

Cypress provides a wealth of data at [www.cypress.com](http://www.cypress.com/) to help you select the right PSoC device and effectively integrate it into your design. Visit our [PSoC 6 MCU](http://www.cypress.com/products/32-bit-arm-cortex-m4-psoc-6) webpage to explore more about PSoC 6 MCU family of device.
Feel free to explore through the code example source files and let us innovate together!

# Navigating the Repository
This repository provides examples that demonstrates how to develop PSoC 6 MCU based analog designs. These examples help you to use peripherals like ADC, DAC, Comparators etc.
If you are new to developing projects with PSoC 6 MCU, we recommend you to refer the [PSoC 6 Getting Started GitHub](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Getting-Started) page which can help you familiarize with device features and guides you to create a simple PSoC 6 design with PSoC Creator IDE. For other block specific designs please refer to the following GitHub repositories:

#### 1. [Digital Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Digital-Designs)
#### 2. [BLE Connectivity Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-BLE-Connectivity-Designs)
#### 3. [Audio Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Audio-Designs)
#### 4. [Device Related Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Device-Related-Design)
#### 5. [System-Level Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-System-Level-Designs)
#### 6. [PSoC 6 Pioneer Kit Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Pioneer-Kits)
#### 7. [PSoC 6 MCU based RTOS Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-RTOS-Based-Design)

You can use these block level examples to guide you through the development of a system-level design using PSoC 6 MCU. All the code examples in this repository comes with well documented design guidelines to help you understand the design and how to develop it. The code examples and their associated documentation are in the Code Example folder in the repository.

# Required Tools

## Software
### Integrated Development Environment (IDE)
To use the code examples in this repository, please download and install
[PSoC Creator](http://www.cypress.com/products/psoc-creator)

## Hardware
### PSoC 6 MCU Development Kits
* [CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit](http://www.cypress.com/documentation/development-kitsboards/psoc-6-ble-pioneer-kit).

* [CY8CKIT-062 PSoC 6 WiFi-BT Pioneer Kit](http://www.cypress.com/documentation/development-kitsboards/psoc-6-wifi-bt-pioneer-kit). 

**Note** Please refer to the code example documentation for selecting the appropriate kit for testing the project

## Code Examples List
#### 1. CE218129 – PSoC 6 MCU Wakeup from Hibernate Using a Low-Power Comparator
This code example demonstrates how to set the Component options for the LPComp internal reference voltage and how to set the external input from a GPIO using the LPComp driver.
The code example uses one GPIO input to compare the input voltage and internal reference voltage to wake the PSoC 6 MCU from Hibernate mode. The LED indicates the current power mode.
#### 2. CE218472 - PSoC 6 MCU: Comparing External Voltages Using a Low-Power Comparator
This example demonstrates the voltage comparison functionality using the LPComp Component in PSoC 6 MCU.
#### 3. CE220923 - PSoC 6 MCU VDAC Sawtooth Wave Generator
This example generates a sawtooth wave by incrementing through all 4096 unique codes of the Voltage DAC (12-bit)
Component using an interrupt service routine. Both the PSoC Creator™ Voltage DAC (12-bit) Component and underlying lowlevel
Peripheral Driver Library (PDL) Continuous Time DAC (CTDAC) function calls are shown.
#### 4. CE220924 - PSoC 6 MCU VDAC Sine Wave Generator Using DMA
This example generates a sine wave using the Voltage DAC (12-bit) and DMA Components. The DMA Component transfers
data from a lookup table to the DAC value register without any CPU intervention. Other than function calls to initialize and
enable the hardware, there are no other software operations. Both the PSoC Creator™ Voltage DAC (12-bit) Component and
underlying low-level Continuous Time DAC (CTDAC) PDL function calls are shown.
#### 5. CE220925 - PSoC 6 MCU VDAC Sample and Hold
This example demonstrates how to maintain the VDAC output voltage using the sample and hold capacitor while in Deep
Sleep mode. Both Component and low-level Peripheral Driver Library (PDL) function calls are demonstrated. When the
voltage across the capacitor drifts below the internal 1.20-V bandgap voltage, the comparator wakes the device to re-sample
the VDAC output. When the device wakes up, the red LED on the kit is toggled for a visual cue.
#### 6. CE220927 - PSoC 6 MCU OpAmp and Comparator Example
This example demonstrates a simple OpAmp gain stage and a Comparator using VDAC as a programmable reference. Both
Component and low-level PDL function calls are shown to configure and use the OpAmp, Comparator, Comparator interrupt,
VDAC, and internal analog routing.
#### 7. CE220974 - PSoC 6 MCU Multi-Config Scan_ADC Example
This example demonstrates a Scan_ADC with two conversion configurations implemented using direct PDL functions calls.
The example selects between a fast free-running two-channel differential system and a firmware-triggered scan with averaging
of the internal die temperature (DieTemp) sensor. The DieTemp sensor is scanned once per second using a watchdog timer.
Results of all channel readings are transmitted over UART.

## References
#### 1. PSoC 6 MCU
PSoC 6 bridges the gap between expensive, power hungry application processors and low‑performance microcontrollers (MCUs). The ultra‑low‑power PSoC 6 MCU architecture offers the processing performance needed by IoT devices, eliminating the tradeoffs between power and performance. The PSoC 6 MCU contains a dual‑core architecture, with both cores on a single chip. It has an Arm® Cortex®‑M4 for high‑performance tasks, and an Arm® Cortex®‑M0+ for low-power tasks, and with security built-in, your IoT system is protected.
To learn more on the device, please visit our [PSoC 6 MCU](http://www.cypress.com/products/32-bit-arm-cortex-m4-psoc-6) webpage.

####  2. PSoC 6 MCU Learning resource list
##### 2.1 PSoC 6 MCU Datasheets
Device datasheets list the features and electrical specifications of PSoC 6 families of devices: [PSoC 6 MCU Datasheets](http://www.cypress.com/search/all?f%5B0%5D=meta_type%3Atechnical_documents&f%5B1%5D=resource_meta_type%3A575&f%5B2%5D=field_related_products%3A114026)
##### 2.2 PSoC 6 MCU Application Notes
Application notes are available on the Cypress website to assist you with designing your PSoC application: [A list of PSoC 6 MCU ANs](http://www.cypress.com/psoc6an)
##### 2.3 PSoC 6 MCU Component Datasheets
PSoC Creator utilizes "components" as interfaces to functional Hardware (HW). Each component in PSoC Creator has an associated datasheet that describes the functionality, APIs, and electrical specifications for the HW. You can access component datasheets in PSoC Creator by right-clicking a component on the schematic page or by going through the component library listing. You can also access component datasheets from the Cypress website: [PSoC 6 Component Datasheets](http://www.cypress.com/documentation/component-datasheets)
##### 2.4 PSoC 6 MCU Technical Reference Manuals (TRM)
The TRM provides detailed descriptions of the internal architecture of PSoC 6 devices:[PSoC 6 MCU TRMs](http://www.cypress.com/psoc6trm)

## FAQ

### Technical Support
Need support for your design and development questions? Check out the [Cypress Developer Community 3.0](https://community.cypress.com/welcome).  

Interact with technical experts in the embedded design community and receive answers verified by Cypress' very best applications engineers. You'll also have access to robust technical documentation, active conversation threads, and rich multimedia content. 

You can also use the following support resources if you need quick assistance: 
##### Self-help: [Technical Support](http://www.cypress.com/support)
##### Local Sales office locations: [Sales Office](http://www.cypress.com/about-us/sales-offices)
