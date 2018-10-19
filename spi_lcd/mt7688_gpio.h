#ifndef __MT7688_GPIO_H
#define __MT7688_GPIO_H


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

#endif


