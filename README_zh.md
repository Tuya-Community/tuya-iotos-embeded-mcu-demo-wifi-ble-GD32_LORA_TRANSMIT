# Tuya IoTOS Embedded Mcu Demo Wifi Ble GD32_LORA_TRANSMIT

[English](./README.md) | [中文](./README_zh.md)

## 简介 

本Demo通过GD32E230模拟IIC分别读取温湿度传感器和光照度传感器数据，通过LORA模块发送出去。

已实现功能包括：

+ 温湿度数据采集
+ 光照度数据采集
+ LORA发送




## 快速上手 

### 编译与烧录
+ 下载Tuya IoTOS嵌入式代码

+ 执行Project.uvprojx文件

+ 点击软件中的编译，并完成下载


### 文件介绍 

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



### Demo入口

入口文件：main.c

重要函数：main()

+ 对mcu的USART，IIC，SPI，以及SHT3X ,BH1750，LORA等模块进行初始化配置，所有事件在while(1)中轮询判断。

### I/O 列表 

|    SHT3X     |   BH1750    |          LORA           |  UASRT0  | UASRT1  |
| :----------: | :---------: | :---------------------: | :------: | :-----: |
| PA11 IIC_SCL | PB6 IIC_SCL |         SCK/PA5         | PA9 TXD  | PA2 TXD |
| PA12 IIC_SDA | PB7 IIC_SDA | MISO/PA6       MOSI/PA7 | PA10 RXD | PA3 RXD |

## 相关文档

涂鸦Demo中心：https://developer.tuya.com/demo



## 技术支持

您可以通过以下方法获得涂鸦的支持:

- 开发者中心：https://developer.tuya.com
- 帮助中心: https://support.tuya.com/help
- 技术支持工单中心: [https://service.console.tuya.com](https://service.console.tuya.com/) 