#include "SPI.h"
/************************************************
函数名称 ： rcu_config
功    能 ： RCU时钟配置
参    数 ： 无
返 回 值 ： 无
*************************************************/
void rcu_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_SPI0);
}
/************************************************
函数名称 ： gpio_config
功    能 ： SPI映射GPIO初始化
参    数 ： 无
返 回 值 ： 无
*************************************************/
void gpio_config(void)
{
	  rcu_config();
    /* SPI0 GPIO config: SCK/PA5, MISO/PA6, MOSI/PA7 */
    gpio_af_set(GPIOA, GPIO_AF_0, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7);
    gpio_output_options_set(GPIOA,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	  gpio_bit_reset(GPIOA,GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
}
/************************************************
函数名称 ： spi_config
功    能 ： SPI 初始化
参    数 ： 无
返 回 值 ： 无
*************************************************/
void spi_config(void)
{
    spi_parameter_struct spi_init_struct;
    /* deinitilize SPI and the parameters */
    spi_i2s_deinit(SPI0);
    spi_struct_para_init(&spi_init_struct);

    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_8;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI0, &spi_init_struct);
	  /* set crc polynomial */
	  spi_crc_polynomial_set(SPI0, 7);
	  spi_enable(SPI0);
}




