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


#define DEVICE_NAME                 "gpio"
#define GPIO_MAJOR                       231

#define GPIO_MODE_BASE                0x10000060
#define GPIO_BASE                                0x10000600

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
    uint32_t reserve0[2];
    uint32_t pol0; // control polarity. inverted  or no-inverted
    uint32_t pol1;
    uint32_t pol2;
    uint32_t reserve1[2];
    uint32_t data0;// current gpio data
    uint32_t data1;
    uint32_t data2;
    uint32_t reserve2[2];
    uint32_t dset0; //set bit int datax register
    uint32_t dset1; 
    uint32_t dset2; 
    uint32_t reserve3[2];
    uint32_t dclr0; // clear bit int datax register
    uint32_t dclr1; 
    uint32_t dclr2; 
};

struct _gint_base
{
    uint32_t redge0;// enable the condition of rising edge triggered interrupt
    uint32_t redge1;
    uint32_t redge2;
    uint32_t fedge0;// enable the condition of falling edge triggered interrupt
    uint32_t fedge1;
    uint32_t fedge2;
    uint32_t hlvl0;// enable the condition of high level triggered interrupt.
    uint32_t hlvl1;
    uint32_t Hlvl2;
    uint32_t llvl0;// enable the condition of low level triggered interrupt.
    uint32_t llvl1;
    uint32_t llvl2;
    uint32_t stat0;//ecord the GPIO current interrupt status.
    uint32_t stat1;
    uint32_t stat2;
    uint32_t edge0;// record the GPIO current interrupt's edge status.
    uint32_t edge1;
    uint32_t edge2;
};
struct _gpio_config
{
    uint32_t a_cfg;
    uint32_t n9_int;
    uint32_t n9_mask;
};

struct _gpio_base *gpio;

struct _gpio_mode *gpio_mode;

struct _gpio_config *gpio_config;

static struct class *gpio_class;
struct cdev gpio_cdev;
int major = GPIO_MAJOR;
dev_t  devno;

uint32_t *gpio_ctl0;
uint32_t *gpio_data0;
uint32_t   *gpio_dset0;
uint32_t *gpio_dclr0;

uint32_t *gpio_data1;
uint32_t *gpio_data2;

static int gpio_open(struct inode *inode,struct file *file)
{
 //   int minor = MINOR(inode->i_rdev);//获得次设备号
    
    return 0;
}

static ssize_t gpio_write(struct file *filp,const char __user *buff,size_t count,loff_t *offp )
{
    int i;
//    int minor = MINOR(inode->i_rdev);//获得次设备号
    char buf[4];
    copy_from_user(&buf,buff,4);
  // gpio->ctl0 |= 0x01;
     *gpio_ctl0 |= 0x01;
   if(memcmp(buf,"on",2)==0)
    {
      //gpio->dset0 |= 0x01;
      *gpio_dset0 |= 0x01;
      printk("on\n");
    }
   else
    {
       // gpio->dclr0 |=0x01;
       *gpio_dclr0 |=0x01;
        printk("off\n");
    }
    return count;
}



static int gpio_read(struct file *filp,char __user *buff,size_t count,loff_t *offp )
{

  //  int minor = MINOR(inode->i_rdev);//获得次设备号
  //  char val;
  // gpio->ctl0 &= ~0x01;
     *gpio_ctl0  &= ~0x01;
    printk("gpio_mode->mode1 :%X\n",gpio_mode->mode1);
    printk("gpio->data0 :%X\n",gpio->data0);
    printk("gpio->data1 :%X\n",gpio->data1);
    printk("gpio->data2 :%X\n",gpio->data2);
    printk("gpio_data0=%X\n",*gpio_data0);
    printk("gpio_data1=%X\n",*gpio_data1);
    copy_to_user(buff,&gpio->data0,4);
    return 0;
}

static struct file_operations gpio_fops = 
{
       .owner = THIS_MODULE,
       .open = gpio_open,
        .read = gpio_read,
        .write = gpio_write,
};



static int gpio_init(void)
{
    int ret;
    int minor = 0;
    gpio = (struct _gpio_base *)ioremap(0x10000600,sizeof(struct _gpio_base));
    //printk("sizeof(struct _gpio_base)=%d\n",sizeof(struct _gpio_base));
    gpio_mode =(struct _gpio_mode *) ioremap(GPIO_MODE_BASE,sizeof(struct _gpio_mode));
    gpio_ctl0 = ioremap(0x10000600,4);
    gpio_data0 = ioremap(0x10000620,4);
    gpio_data1 = ioremap(0x10000624,4);
    gpio_dset0 = ioremap(0x10000630,4);
    gpio_dclr0 = ioremap(0x10000640,4);
    if(!gpio)
    {
        return -EIO;
    }
    devno = MKDEV(major,minor);
    ret = register_chrdev_region( devno,1,DEVICE_NAME);
    if(ret<0)
    {
        printk(DEVICE_NAME "can't register major number\n");
        return ret;
    }
    cdev_init(&gpio_cdev,&gpio_fops);
    cdev_add(&gpio_cdev,devno,1);

    //硬件相关操作
    gpio_mode->mode1  = (0x01<<6) |(gpio_mode->mode1 & 0xffffff3f);//设置为GPIO
    
    // /dev/
   gpio_class = class_create(THIS_MODULE,"gpio_driver");
   device_create(gpio_class, NULL, MKDEV(major, 0), NULL,"gpio");
    return 0;
}


void gpio_exit(void)
{
    iounmap(gpio);
    iounmap(gpio_mode);
    iounmap(gpio_ctl0);
    iounmap(gpio_data0);
    iounmap(gpio_data1);
    iounmap(gpio_dset0);
    iounmap(gpio_dclr0);
    cdev_del(&gpio_cdev);
    device_destroy(gpio_class,MKDEV(major,0));
    class_destroy(gpio_class);
    unregister_chrdev_region(devno,1);
}
module_init(gpio_init);
module_exit(gpio_exit);


MODULE_LICENSE("GPL");

