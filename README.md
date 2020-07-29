# ISP3010-Examples
UWB examples using ISP3010

## Overview

The aim of this archive is to show examples of UWB implementation using the IS3010 module.

The ISP3010 module integrates the DecaWave DW1000 chipset which is an Ultra Wide Band transceiver dedicated to indoor localisation applications. This chipset needs an external processor to operate and Insight SiP decided to integrate the Nordic Semi nRF52 SoC. In addition to the powerful ARM Cortex M4F MCU, this chip also provides a BLE and NFC connectivity for wireless set up and control of the UWB chip.
 
ISP3010 differentiate from other module by integrating a multi band antenna in the package, to support both BLE frequency band at 2.4 GHz and UWB frequency band at 6.5 GHz. This is a new and unique concept developed by Insight SiP within a SIP package. In addition, this module also integrates all passives, crystals and DC/DC converters to optimize the power consumption for longer life time when operating on batteries.

ISP3010 module offers the perfect stand-alone ranging solution for short range security bubble applications, for standard location applications requiring typically 50 meters range from module to module, or even longer distance application when the ISP3010 is used in conjunction with optimized UWB anchor antennas.

See https://www.insightsip.com/products/combo-smart-modules/isp3010.

The following examples are provided:

* **ranging_demo**: Ranging demo provided in the development kit.

* **simple_twr_initiator / simple_twr_responder**: Basic TWR application.

* **ble_lorawan_AT_commands**: Example of LoRaWan AT commands set.

* **simple_uwb_rx**: Set DW1000 in receive mode.

* **simple_uwb_tx**: "Hello world" transmission in UWB.

* **uwb_cw**: Transmits an UWB tone.

## Environment

The examples are ready to use with the Segger Embedded Studio (SES) IDE.

SES provide a free license for nRF52832 development. Therefore it can be used freely for ISP4520 development.
Licenses can be requested at https://license.segger.com/Nordic.cgi

For more information regarding Segger Embedded Studio, please visit https://www.segger.com/products/development-tools/embedded-studio/

## Changelog

### 2020-02-17, v1.1.0

Added uwb frame filtering

### 2019-07-23, v1.0.0

Initial Release.
