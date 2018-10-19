#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
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
#include <asm/io.h>
#include <asm/irq.h>


static struct net_device *vnet_dev;

static void emulator_rx_packet(struct sk_buff  *skb,struct net_device *dev)
{
    unsigned char *type;
    struct iphdr *ih;
    __be32 *saddr,*daddr,tmp;
    unsigned char tmp_dev_addr[ETH_ALEN];
    struct ethhdr *ethhdr;
    struct sk_buff *rx_skb;

 //模拟接收到的数据
   //从硬件读出数据、保存数据
   //对调 源目的的MAC地址
   ethhdr = (struct ethhdr *)skb->data;
   memcpy(tmp_dev_addr,ethhdr->h_dest,ETH_ALEN);
   memcpy(ethhdr->h_dest,ethhdr->h_source,ETH_ALEN);
   memcpy(ethhdr->h_source,tmp_dev_addr,ETH_ALEN);

   //对调源目的IP
   ih = (struct iphdr *)(skb->data + sizeof(struct ethhdr));
   saddr = &ih->saddr;
   daddr = &ih->daddr;
  //  printk("step0\n");
    tmp = *saddr;
    *saddr = *daddr;
    *daddr = tmp;

  type = skb->data + sizeof(struct ethhdr) + sizeof(struct iphdr);
  *type = 0; //原来0x08表示ping, 0x00表示reply
  ih->check = 0;
  // printk("ih->ihl:%d\n",ih->ihl);
   if(ih->ihl)
        ih->check = ip_fast_csum(ih,ih->ihl);
  //   printk("step1\n");
   //构造一个sk_buff;
   rx_skb = dev_alloc_skb(skb->len +2);
   skb_reserve(rx_skb,2);/*align IP on 16B boundary*/
   memcpy(skb_put(rx_skb,skb->len),skb->data,skb->len);

   // printk("step2\n");
   /*Write metdata,and then pass to the recevice level*/
   rx_skb->dev = dev;
   rx_skb->protocol = eth_type_trans(rx_skb,dev);
   rx_skb->ip_summed = CHECKSUM_UNNECESSARY; /*do not chect it*/
   dev->stats.rx_packets++;
   dev->stats.rx_bytes += skb->len;
//  printk("step3\n");
   //提交
   netif_rx(rx_skb);
 //  printk("step4\n");
}

static netdev_tx_t  virt_net_send_packet (struct sk_buff *skb,struct net_device *dev)				  
{
    static int cnt =0 ;
    printk("virt_net_send_packet cnt=%d\n",++cnt);
    netif_stop_queue(dev);//停止网卡队列，处理发送数据

    /*构造一个假的sk_buff 上报*/  //模拟接收
    emulator_rx_packet(skb,dev);
  /*
    将skb的数据写入网卡
  */
    dev_kfree_skb(skb); //释放skb
    netif_wake_queue(dev); //数据发送出去之后，继续唤醒网卡的队列
    
    dev->stats.tx_packets++;
    dev->stats.tx_bytes += skb->len;
    printk("virt_net_send_packet end\n");
    return 0;
}

 struct  net_device_ops  ndev_ops ={
    .ndo_start_xmit = virt_net_send_packet,
  };
static int virt_net_init(void)
{
    vnet_dev = alloc_netdev(0,"vnet%d",NET_NAME_ENUM,ether_setup);//分配内存和创建设备
 //  vnet_dev = alloc_etherdev(0);
 
    vnet_dev->netdev_ops = &ndev_ops;

   if(!vnet_dev)
    {
        printk("alloc_netdv fail\n");
   }
   
    if(!vnet_dev->netdev_ops)
    {
        printk("vnet_dev->netdev_ops is null\n");
    }
   
    vnet_dev->dev_addr[0] = 0x89;
    vnet_dev->dev_addr[1] = 0x21;
    vnet_dev->dev_addr[2] = 0x89;
    vnet_dev->dev_addr[3] = 0x45;
    vnet_dev->dev_addr[4] = 0x56;
    vnet_dev->dev_addr[5] = 0x43;
    
    vnet_dev->flags  |= IFF_NOARP;
    vnet_dev->features |= NETIF_F_IP_CSUM_BIT;
   if (register_netdev(vnet_dev))
       printk( "Could not register netdevice\n");
    return 0;
    
}

void virt_net_exit(void)
{
    unregister_netdev(vnet_dev);
    free_netdev(vnet_dev);
}


module_init(virt_net_init);
module_exit(virt_net_exit);



MODULE_AUTHOR("thisway.diy@163.com,17653039@qq.com");
MODULE_LICENSE("GPL");




