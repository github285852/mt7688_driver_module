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
#include <linux/ip.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/delay.h>
/////////////////////////////////////////////////////spi.c

#define SPI_INT             27
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
};

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
    uint32_t dset0; //set all bit int datax register
    uint32_t dset1; 
    uint32_t dset2; 
    uint32_t reserve3;
    uint32_t dclr0; // clear all bit int datax register
    uint32_t dclr1; 
    uint32_t dclr2; 
};


struct _gpio_base *gpio;
struct _gpio_mode *gpio_mode;
uint32_t *clkcfg1;
uint32_t *rstctl;
struct _spi *spi;


static irqreturn_t spi_interrupt(int irq, void *dev_id);

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
    // *rstctl |= 0x01<<18;// reset spi
    //设置spi
    spi->master  =  (spi->master &0x1fffffff) |(0x01<<29); // select SPI device 1
    spi->master  &= 0xF000FFFF;//set SPI clk
   spi->master  |= 0x0FFF0000&(8<<16);
 
//    spi->master  &= 0xFFFF07FF;//set cs_dsel_cnt
//    spi->master  |= 0x1F<<11;
    spi->master |= 0x01<<10;//  full duplex mode,全双工模式
    spi->master |= 0x01<<2; //Select  8 words buffer for SPI transaction,max 32 bytes
    spi->master |=0x01<<9; //enable spi int 

   if( request_irq(SPI_INT,spi_interrupt,0,"spi",NULL))//IRQF_SHARED
    {
        printk("request_irq fail \n");
        free_irq(SPI_INT,NULL);
    }
}

void spi_destroy(void)
{
    iounmap(gpio_mode);
    iounmap(gpio);
    iounmap(clkcfg1);
    iounmap(rstctl);
    iounmap(spi);  
    free_irq(SPI_INT,NULL);
}

static irqreturn_t spi_interrupt(int irq, void *dev_id)
{
    printk("int\n");
    if(spi->status&0x01)
   {
            printk("spi int\n");
    }
    return 0;
}

void spi_write_bytes(uint8_t *buf,uint16_t len)
{
    spi->master &= ~(0x01<<10);// half duplex mode,
    spi->master &= ~(0x01<<2); //Select  2 words buffer for SPI transaction,max 32 bytes

 wait0:
    while(spi->trans&0x10000);// wait buys;
    if(spi->trans&0x10000)
        goto wait0;
    
     memcpy(&spi->op_addr,buf,len);
     spi->trans &= 0xfffffff0;
     spi->trans  |= 0x0000000f&len;

     spi->status |= 0x01;  // must clear int ,
wait:
    while(spi->trans&0x10000);// wait buys;
    if((spi->trans&0x10000)==0)
        spi->trans  |= 0x01<<8;//开始传输
    else
        goto wait;
    
    while((spi->status&0x01)==0)
   {
       schedule();
    }
}

uint8_t spi_wr_bytes_max16(uint8_t *buf,uint32_t len)// 全双工
{
    spi->master |= 0x01<<10;// half duplex mode,
    spi->master |= 0x01<<2; //Select  2 words buffer for SPI transaction,max 32 bytes
    if(len>16)
        return 1;
    
 wait0:
    while(spi->trans&0x10000);// wait buys;
    if(spi->trans&0x10000)
        goto wait0;
    
    memcpy(&spi->dido0,buf,len);
    spi->more_buf  &= 0xC0ffffff ;//设置发送bit数 ,不使用cmd
    //spi->more_buf  |= 0x20000000 ;//设置发送cmd bit数
    spi->more_buf  &= 0xfffffE00;  //mosi_bit_cnt
    spi->more_buf  |=  len*8;

    spi->more_buf  &= 0xffE00fff;  //miso_bit_cnt
    spi->more_buf  |=   len*8<<12;
 
    //printk("%X\n", spi->op_addr);
   /*  printk("%X\n", spi->dido0);
     printk("%X\n", spi->dido1);
    printk("%X\n", spi->dido2);
    printk("%X\n", spi->dido3);
    printk("%X\n", spi->dido4);
   printk("%X\n", spi->dido5);
    printk("%X\n", spi->dido6);
    printk("%X\n", spi->dido7);
    */
     spi->status |= 0x01;  // must clear int ,
     
wait:
    while(spi->trans&0x10000);// wait buys;
    if((spi->trans&0x10000)==0)
        spi->trans  |= 0x01<<8;//开始传输
    else
        goto wait;
   uint16_t count = 0;
  while((spi->status&0x01)==0)
   {
      //  schedule();
            count ++;
            if(count>=50000)
                {
                    printk("3");
                    count = 0;
                }
    }
    memcpy(buf,&spi->dido4,len);//接收
    return 0;
}

uint8_t spi_wr_bytes(uint8_t *buf,uint32_t len)
{
    int32_t i;
    for(i=len;i>0;i -= 16)
    {
        if(i>16)
        {
            spi_wr_bytes_max16(buf + len -i,16);
        }
        else
        {
            spi_wr_bytes_max16(buf + len -i,i);
        }
    }
    return 0;
}
//////////////////////////////////////////////////////////////////////xxx9341.c

#define LCD_DC_CLR       gpio->data1 &= ~(0x01<<14)
#define LCD_DC_SET       gpio->data1 |= 0x01<<14

#define LCD_RST_CLR     gpio->data1 &= ~(0x01<<13)
#define LCD_RST_SET      gpio->data1 |= 0x01<<13

void LCD_WR_DATA8(uint8_t data) 
{	
    LCD_DC_SET;
  spi_write_bytes(&data,1);
} 

 void LCD_WR_DATA(uint16_t data)
{
    LCD_DC_SET;
    uint8_t *p = &data;
    spi_write_bytes(p,1);
     spi_write_bytes(p+1,1);
 }

void LCD_WR_REG(uint8_t cmd)	 
{
    LCD_DC_CLR ;
  spi_write_bytes(&cmd,1);
}

 void LCD_WR_REG_DATA(uint8_t reg,uint16_t data)
{	
    LCD_WR_REG(reg);
    LCD_WR_DATA(data);
}

void Address_set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{ 
    LCD_WR_REG(0x2a);
    LCD_WR_DATA8(x1>>8);
    LCD_WR_DATA8(x1);
    LCD_WR_DATA8(x2>>8);
    LCD_WR_DATA8(x2);
  
   LCD_WR_REG(0x2b);
   LCD_WR_DATA8(y1>>8);
   LCD_WR_DATA8(y1);
   LCD_WR_DATA8(y2>>8);
   LCD_WR_DATA8(y2);

   LCD_WR_REG(0x2C);					 						 
}

#define LCD_W 240
#define LCD_H 320

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 
#define BRRED 			 0XFC07 
#define GRAY  			 0X8430 



void LCD_Clear(u16 Color);


void xxx9341_init(void)
{
   // spi_init();
    gpio_mode->mode1 &= ~ (0x01<<25);// uart1 set gpio
    gpio_mode->mode1 |=  0x01<<24;
    
    gpio->ctl1 |= 0x01<<14; //GPIO18设为输出dc
    gpio->ctl1 |= 0x01<<13;//GPIO19设为输出 rst
   LCD_RST_SET;
    mdelay(1);
   LCD_RST_CLR;
    mdelay(10);
   LCD_RST_SET;
   mdelay(120);
   
  //************* Start Initial Sequence **********// 
LCD_WR_REG(0xCF);  
LCD_WR_DATA8(0x00); 
LCD_WR_DATA8(0xD9); 
LCD_WR_DATA8(0X30); 
 
LCD_WR_REG(0xED);  
LCD_WR_DATA8(0x64); 
LCD_WR_DATA8(0x03); 
LCD_WR_DATA8(0X12); 
LCD_WR_DATA8(0X81); 
 
LCD_WR_REG(0xE8);  
LCD_WR_DATA8(0x85); 
LCD_WR_DATA8(0x10); 
LCD_WR_DATA8(0x78); 
 
LCD_WR_REG(0xCB);  
LCD_WR_DATA8(0x39); 
LCD_WR_DATA8(0x2C); 
LCD_WR_DATA8(0x00); 
LCD_WR_DATA8(0x34); 
LCD_WR_DATA8(0x02); 
 
LCD_WR_REG(0xF7);  
LCD_WR_DATA8(0x20); 
 
LCD_WR_REG(0xEA);  
LCD_WR_DATA8(0x00); 
LCD_WR_DATA8(0x00); 
 
LCD_WR_REG(0xC0);    //Power control 
LCD_WR_DATA8(0x21);   //VRH[5:0] 
 
LCD_WR_REG(0xC1);    //Power control 
LCD_WR_DATA8(0x12);   //SAP[2:0];BT[3:0] 
 
LCD_WR_REG(0xC5);    //VCM control 
LCD_WR_DATA8(0x32); 
LCD_WR_DATA8(0x3C); 
 
LCD_WR_REG(0xC7);    //VCM control2 
LCD_WR_DATA8(0XC1); 
 
LCD_WR_REG(0x36);    // Memory Access Control 
LCD_WR_DATA8(0x08); 
 
LCD_WR_REG(0x3A);   
LCD_WR_DATA8(0x55); 

LCD_WR_REG(0xB1);   
LCD_WR_DATA8(0x00);   
LCD_WR_DATA8(0x18); 
 
LCD_WR_REG(0xB6);    // Display Function Control 
LCD_WR_DATA8(0x0A); 
LCD_WR_DATA8(0xA2); 

 
 
LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
LCD_WR_DATA8(0x00); 
 
LCD_WR_REG(0x26);    //Gamma curve selected 
LCD_WR_DATA8(0x01); 
 
LCD_WR_REG(0xE0);    //Set Gamma 
LCD_WR_DATA8(0x0F); 
LCD_WR_DATA8(0x20); 
LCD_WR_DATA8(0x1E); 
LCD_WR_DATA8(0x09); 
LCD_WR_DATA8(0x12); 
LCD_WR_DATA8(0x0B); 
LCD_WR_DATA8(0x50); 
LCD_WR_DATA8(0XBA); 
LCD_WR_DATA8(0x44); 
LCD_WR_DATA8(0x09); 
LCD_WR_DATA8(0x14); 
LCD_WR_DATA8(0x05); 
LCD_WR_DATA8(0x23); 
LCD_WR_DATA8(0x21); 
LCD_WR_DATA8(0x00); 
 
LCD_WR_REG(0XE1);    //Set Gamma 
LCD_WR_DATA8(0x00); 
LCD_WR_DATA8(0x19); 
LCD_WR_DATA8(0x19); 
LCD_WR_DATA8(0x00); 
LCD_WR_DATA8(0x12); 
LCD_WR_DATA8(0x07); 
LCD_WR_DATA8(0x2D); 
LCD_WR_DATA8(0x28); 
LCD_WR_DATA8(0x3F); 
LCD_WR_DATA8(0x02); 
LCD_WR_DATA8(0x0A); 
LCD_WR_DATA8(0x08); 
LCD_WR_DATA8(0x25); 
LCD_WR_DATA8(0x2D); 
LCD_WR_DATA8(0x0F); 
 
LCD_WR_REG(0x11);    //Exit Sleep 
mdelay(120); 
LCD_WR_REG(0x29);    //Display on 

LCD_Clear(RED);
mdelay(500); 
LCD_Clear(BLUE);
mdelay(500); 
LCD_Clear(BRED);
mdelay(500); 
LCD_Clear(GBLUE);
}

void LCD_Clear(u16 Color)
{
    
    uint16_t buf[320];
    uint32_t i,j;  	 
      
     Address_set(0,0,LCD_W-1,LCD_H-1);
     for(i=0;i<LCD_W;i++)
    {
        for (j=0;j<LCD_H;j++)
        {
            LCD_WR_DATA(Color);	 			 
        }
    }
   /*   
    for(i=0;i<319;i++)
        buf[i] = Color;
    Address_set(0,0,LCD_W-1,LCD_H-1);
     LCD_DC_SET;
    for(i=0;i<240;i++)
    {
        spi_wr_bytes(buf,320*2);
    }  
*/
}

//////////////////////////////////////////////////////////////////////sip_lcd.c

#define DEVICE_NAME                 "spi_lcd"
#define SPI_LCD_MAJOR                       231

static struct class *spi_lcd_class;
struct cdev spi_lcd_cdev;
int major = SPI_LCD_MAJOR;
dev_t  devno;

static int spi_lcd_open(struct inode *inode,struct file *file)
{
 //   int minor = MINOR(inode->i_rdev);//获得次设备号
    
    return 0;
}

static ssize_t spi_lcd_write(struct file *filp,const char __user *buff,size_t count,loff_t *offp )
{
    int i,j=1;
//    int minor = MINOR(inode->i_rdev);//获得次设备号
    char buf[100];
    char num_buf[4],temp=0;
    copy_from_user(num_buf,buff,4);
    for(i=0;i<100;i++)
        buf[i] = i;
    i=0;
    while(num_buf[i]!='\0')
    {
        temp += (num_buf[i]-0x30)*j;
        j *= 10;
        i++;
    }
    printk("temp=%d\n",temp);
    spi_wr_bytes(buf,88);
   //spi_write_bytes(buf,4);
    return count;
}

static int spi_lcd_read(struct file *filp,char __user *buff,size_t count,loff_t *offp )
{

  //  int minor = MINOR(inode->i_rdev);//获得次设备号
    return 0;
}

static struct file_operations spi_lcd_fops = 
{
       .owner = THIS_MODULE,
       .open = spi_lcd_open,
        .read = spi_lcd_read,
        .write = spi_lcd_write,
};

static int spi_lcd_init(void)
{
    int ret;
    int minor = 0;
    spi_init();
    xxx9341_init();
    devno = MKDEV(major,minor);
    ret = register_chrdev_region( devno,1,DEVICE_NAME);
    if(ret<0)
    {
        printk(DEVICE_NAME "can't register major number\n");
        return ret;
    }
    cdev_init(&spi_lcd_cdev,&spi_lcd_fops);
    cdev_add(&spi_lcd_cdev,devno,1);
    
    // /dev/
   spi_lcd_class = class_create(THIS_MODULE,"spi_lcd_driver");
   device_create(spi_lcd_class, NULL, MKDEV(major, 0), NULL,"spi_lcd");
    return 0;
}

void spi_lcd_exit(void)
{
    spi_destroy();
    cdev_del(&spi_lcd_cdev);
    device_destroy(spi_lcd_class,MKDEV(major,0));
    class_destroy(spi_lcd_class);
    unregister_chrdev_region(devno,1);
}
module_init(spi_lcd_init);
module_exit(spi_lcd_exit);


MODULE_LICENSE("GPL");

