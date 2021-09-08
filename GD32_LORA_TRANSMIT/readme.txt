GD32E230通过模拟IIC分别读取温湿度传感器和光照度传感器数据，通过LORA模块发送出去。
温湿度传感器(SHT30)接口连线：
/* 软件模拟IIC引脚
 * IIC_SCL --> PA11
 * IIC_SDA --> PA12 */
光照度传感器(BH1750)接口连线：
/* 软件模拟IIC引脚
 * IIC_SCL --> PB6
 * IIC_SDA --> PB7*/


DemoGD32E230 reads the data of t/H sensor and illuminance sensor through analog IIC, and sends the data out through LORA module.  
Connection to the T/H sensor (SHT30) Port:  
/* Software simulation IIC pin  
* IIC_SCL --> PA11  
* IIC_SDA --> PA12 */  
Light sensor (BH1750) Interface connection:  
/* Software simulation IIC pin 
* IIC_SCL --> PB6  
* IIC_SDA --> PB7*/  