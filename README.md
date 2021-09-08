# Tuya IoTOS Embedded Mcu Demo Wifi Ble GD32_LORA_TRANSMIT

[English](./README.md) | [中文](./README_zh.md)

## Introduction  

The Demo uses the GD32E230 analog IIC to read the data of the T/H sensor and the illumination sensor respectively, and then sends the data through the LORA module.  

The implemented features include:

+ Temperature And Humidity Acquisition
+ Illuminance Acquisition
+ LORA TRANSMIT


## Quick start  

### Compile & Burn
+ Download Tuya IoTOS Embeded Code
+ Execute the Project.uvprojx file
+ Click Compile in the software and complete the download


### File introduction 

```
├── Application
│   ├── main.c
│   ├── gd32e23x_it.c
│   ├── systick.c
│   ├── gd32e23x_it.h
│   ├── systick.h
│   ├── gd32e23x_libopt.h
├── GD32E23x_Firmware_Library
│   ├── CMSIS
        ├── Include
           │   ├──gd32e23x.h
           │   ├──system_gd32e23x.h
        ├── Source
           │   ├──startup_gd32e23x.s
           │   ├──system_gd32e23x.h        
│   ├── GD32E23x_standard_peripheral
        ├── Include
        ├── Source
├──User
│   ├── BH1750.c
│   ├── BH1750.h
│   ├──delay.c
│   ├──delay.h
│   ├──sht3x.c
│   ├──sht3x.h
│   ├──soft_i2c.c
│   ├──soft_i2c.h
│   ├──SPI.c
│   ├──SPI.h
│   ├──sx126x_v01.c
│   ├──sx126x_v01.h
│   ├──usart.c
└──────usart.h 
```



### Demo entry

Entry file：main.c

Important functions：main()

+ Initialize and configure MCU IIC，USART，SPI，SHT3X ，BH1750，LORA and other modules , etc. All events are polled and judged in while(1)。

### I/O List  

|    SHT3X     |   BH1750    |          LORA           |  UASRT0  | UASRT1  |
| :----------: | :---------: | :---------------------: | :------: | :-----: |
| PA11 IIC_SCL | PB6 IIC_SCL |         SCK/PA5         | PA9 TXD  | PA2 TXD |
| PA12 IIC_SDA | PB7 IIC_SDA | MISO/PA6       MOSI/PA7 | PA10 RXD | PA3 RXD |

## Related Documents

 Tuya Demo Center: https://developer.tuya.com/demo



## Technical Support

  You can get support for Tuya by using the following methods:

- Developer Center: https://developer.tuya.com
- Help Center: https://support.tuya.com/help
- Technical Support Work Order Center: [https://service.console.tuya.com](https://service.console.tuya.com/) 

