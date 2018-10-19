#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <spi.h>
//#include <mt7688_gpio.h>

struct _spi
{
    uint32_t trans;       //SPI transaction control/status register
    uint32_t op_addr; //SPI opcode/address register
    uint32_t dido0;  //SPI DI/DO data #0 register
    uint32_t dido1;//SPI DI/DO data #1 register
    uint32_t dido2;//SPI DI/DO data #2 register
    uint32_t dido3;//SPI DI/DO data #3 register
    uint32_t dido4;//SPI DI/DO data #4 register
    uint32_t dido5;//SPI DI/DO data #5 register
    uint32_t dido6;//SPI DI/DO data #6 register
    uint32_t dido7;//SPI DI/DO data #7 register
   uint32_t master;//SPI master mode register
   uint32_t more_buf;//SPI more buf control register
   uint32_t queue_ctl;//SPI flash queue control register
   uint32_t status;//SPI controller status register
   uint32_t cs_polar;//SPI chip select polarity
   uint32_t space;//SPI flash space control register
}

struct _gpio_mode
{
    uint32_t mode1;
    uint32_t mode2;
};

struct _gpio_base
{
    uint32_t ctl0;  // gpio0 to gpio31 direction
    uint32_t ctl1; // 32 to 63
    uint32_t ctl2; //64 to 95
    uint32_t reserve0;
    uint32_t pol0; // control polarity. inverted  or no-inverted
    uint32_t pol1;
    uint32_t pol2;
    uint32_t reserve1;
    uint32_t data0;// current gpio data
    uint32_t data1;
    uint32_t data2;
    uint32_t reserve2;
    uint32_t dset0; //set bit int datax register
    uint32_t dset1; 
    uint32_t dset2; 
    uint32_t reserve3;
    uint32_t dclr0; // clear bit int datax register
    uint32_t dclr1; 
    uint32_t dclr2; 
};


struct _gpio_base *gpio;
struct _gpio_mode *gpio_mode;
uint32_t *clkcfg1;
uint32_t *rstctl;
struct _spi *spi;
void spi_init(void)
{
    gpio_mode = ioremap(0x10000060,sizeof(struct _gpio_mode));
    gpio = ioremap(0x10000600,sizeof(struct _gpio_base));
    clkcfg1 = ioremap(0x10000030,4);
    rstctl = ioremap(0x10000034,4);
    spi = ioremap(0x10000B00,sizeof(struct _spi));
    gpio_mode->mode1  &=  ~(0x01<<12);//设置为SPI
    gpio_mode->mode1  &=  ~(0x03<<4);//设置为SPI_CS1
    *clkcfg1 |= 0x01<<18;//使能SPI_CLK
     *rstctl |= 0x01<<18;// reset spi
     
}


void spi_destroy(void)
{
    iounmap(gpio_mode);
    iounmap(gpio);
    iounmap(clkcfg1);
    iounmap(rstctl);
    iounmap(spi);
    
}







