GD32E230ͨ��ģ��IIC�ֱ��ȡ��ʪ�ȴ������͹��նȴ��������ݣ�ͨ��LORAģ�鷢�ͳ�ȥ��
��ʪ�ȴ�����(SHT30)�ӿ����ߣ�
/* ���ģ��IIC����
 * IIC_SCL --> PA11
 * IIC_SDA --> PA12 */
���նȴ�����(BH1750)�ӿ����ߣ�
/* ���ģ��IIC����
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