/* å‚è€ƒ:
 * drivers\block\xd.c
 * drivers\block\z2ram.c
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/genhd.h>
#include <linux/hdreg.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/blkdev.h>
#include <linux/blkpg.h>
#include <linux/delay.h>
#include <linux/io.h>
//#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/dma.h>


struct ramblock_dev
{
    unsigned long size;
};


typedef struct request_queue request_queue_t;

static struct gendisk *ramblock_disk;
static request_queue_t *ramblock_queue;

static int major;

static DEFINE_SPINLOCK(ramblock_lock);

#define RAMBLOCK_SIZE (1024*1024)
static unsigned char *ramblock_buf;

static int ramblock_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
    /* =heads*cylinders*sectors*512 */
    geo->heads     = 2;
    geo->cylinders = 32;
    geo->sectors   = RAMBLOCK_SIZE/2/32/512;
    return 0;
}

static int ramblock_open(struct inode *inode,struct file *filp )
{
    struct ramblock_dev *dev = inode->i_bdev->bd_disk->private_data;
    filp->private_data = dev;

    return 0;
}
int ramblock_ioctl(struct inode *inode,struct file *filp,unsigned int cmd,unsigned long arg)
{
    long size;
    struct hd_geometry geo;
    struct ramblock_dev *dev = filp->private_data;
    switch (cmd)
    {
        case HDIO_GETGEO:
            size = dev->size;// *(hardsect_size / KERNEL_SECTOR_SIZE);
            geo.cylinders = (size &~0x3f) >> 6;
            geo.heads = 4;
            geo.sectors = 16;
            geo.start = 4;
            if (copy_to_user((void __user*)arg, &geo, sizeof(geo)))
            {
                return  - EFAULT;
            }
            return 0;
    }

    return  - ENOTTY; //²»ÖªµÀµÄÃüÁî
}

static struct block_device_operations ramblock_fops =
{
    .owner  = THIS_MODULE,
    .getgeo = ramblock_getgeo,
};

static int ramblock_handle_io(unsigned char *block_dev,uint64_t pos,unsigned long len,void *buf,int write)
{
    static int r_cnt = 0;
    static int w_cnt = 0;
    if(write)
    {
  //      printk("do_ramblock_request write %d\n", ++w_cnt);
        memcpy( block_dev+pos,buf, len);
    }
    else
    {
   //      printk("do_ramblock_request read %d\n", ++r_cnt);
        memcpy( buf,block_dev+pos, len);
    }
    return 0;
}

static void do_ramblock_request(struct request_queue * q)
{
    static int cnt = 0;
    struct request *rq;
    uint64_t pos = 0;
    int rv;
    unsigned long len=0;

   // printk("do_ramblock_request %d\n", ++cnt);

    while ((rq = blk_fetch_request(q)) != NULL)//»ñÈ¡I/OÇëÇó¶ÓÁĞÖĞÏÂÒ»¸öÇëÇóº¯Êı
    {
        spin_unlock_irq(q->queue_lock);//Ã»ÄÃµ½Ò»¸örequest£¬¾Í¾ÍÍË³öËø
        if(rq->cmd_type != REQ_TYPE_FS)
        {
            rv = -EIO;//·ÇÎÄ¼şÏµÍ³µÄÇëÇó
            goto skip;
        }
        pos = blk_rq_pos(rq)*512;//·µ»ØÆğÊ¼ÉÈÇøºÅ*512¾ÍÊÇÆ«ÒÆµØÖ·
        len = blk_rq_bytes(rq);//ÇëÇóµÄ×Ö½ÚÊı£¬ÉÈÇø×Ö½ÚµÄÕûÊı±¶
        if((pos+len)>RAMBLOCK_SIZE)
        {
            pr_crit("ramblock:Beyond-end write(%llu %zx)\n",pos,len);
            rv = -EIO;
            goto skip;
        }
        struct bio_vec bvec;//¶¨Òåbio ¶Î½á¹¹Ìå
        struct req_iterator iter;
        void *kaddr = NULL;
        unsigned long test_len=0;
        // Ñ­»·µü´ú±éÀúÒ»¸örequestÀïµÄÃ¿Ò»¸ö¶Î(Ò»¸ö¶Î¶ÔÓ¦Ò»¸öbio)
        //¶ÎµÄÎïÀíµØÖ·ÊÇÁ¬ĞøµÄ£¬µ«¶ÎÓë¶ÎÖ®¼ä²»Ò»¶¨Á¬Ğø
        rq_for_each_segment(bvec,rq,iter)
        {
            //Ó¦ÓÃ³ÌĞòÒª¶ÁĞ´µÄ»º´æ
            kaddr = kmap(bvec.bv_page);
            kaddr += bvec.bv_offset;
            //Ó²¼şÏà¹Ø²Ù×÷
           rv =  ramblock_handle_io(ramblock_buf,pos,bvec.bv_len,kaddr,rq_data_dir(rq));
            if(rv <0)
            {
                goto skip;
            }
            pos += bvec.bv_len;
            test_len += bvec.bv_len;
            kunmap(bvec.bv_page);
        }
        if(test_len !=len)
        {
            printk("test_len !=len\n");
         }
skip:
        blk_end_request_all(rq,rv);
        spin_lock_irq(q->queue_lock);//´¦ÀírequestºóÉÏËø
    }
}

static int ramblock_init(void)
{
    /* 1. ·ÖÅägendisk */
    ramblock_disk = alloc_disk(16); //´ÎÉè±¸ºÅ16

    /* 2.  */
    /* 2.1 ³õÊ¼»¯ÇëÇó¶ÓÁĞ£¬·ÖÅäÇëÇó²¢°ó¶¨ */
    ramblock_queue = blk_init_queue(do_ramblock_request, &ramblock_lock);
    ramblock_disk->queue = ramblock_queue;

    /* 2.2 ×¢²áÉè±¸Çı¶¯ */
    major = register_blkdev(0, "ramblock");  /* cat /proc/devices */
    ramblock_disk->major       = major;
    ramblock_disk->first_minor = 0;
    memcpy(ramblock_disk->disk_name,"ramblock",strlen("ramblock")+1);
   // sprintf(ramblock_disk->disk_name,"%s", "ramblock");
    ramblock_disk->fops        = &ramblock_fops;
    set_capacity(ramblock_disk, RAMBLOCK_SIZE / 512);

    /* 3.  Ó²¼şÏà¹Ø²Ù×÷ */
    ramblock_buf = kzalloc(RAMBLOCK_SIZE, GFP_KERNEL);

    /* 4. Ìí¼Ógendisk device */
    add_disk(ramblock_disk);

    return 0;
}

static void ramblock_exit(void)
{
   // printk("ramblock_exit\n");
    unregister_blkdev(major, "ramblock");
    del_gendisk(ramblock_disk);
    put_disk(ramblock_disk);
    blk_cleanup_queue(ramblock_queue);

    kfree(ramblock_buf);
}

module_init(ramblock_init);
module_exit(ramblock_exit);

MODULE_LICENSE("GPL");
