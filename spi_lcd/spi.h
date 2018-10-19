#ifndef __SPI_H
#define __SPI_H


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


void spi_init(void);
void spi_destroy(void);


#endif





