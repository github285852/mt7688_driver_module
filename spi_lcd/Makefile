#开始满足这个条件 		
ifeq ($(KERNELRELEASE),)            
PWD=$(shell pwd)  
KERNEL_DIR=/home/ram/openwrt/build_dir/target-mipsel_24kec+dsp_uClibc-0.9.33.2/linux-ramips_mt7688/linux-3.18.45
TOOLCHAIN="/home/ram/openwrt/staging_dir/toolchain-mipsel_24kec+dsp_gcc-4.8-linaro_uClibc-0.9.33.2/bin/mipsel-openwrt-linux-" 

all:  
	make  -C $(KERNEL_DIR)  modules -I /home/ram/include/  M=$(PWD)   ARCH=mips CROSS_COMPILE=$(TOOLCHAIN) 
#-C 指定内核Makefile的路径，可以使用相对路径。  
#-M 指定要编译的文件的路径，同样可以使用相对路径。  
clean:  
	rm -f *.ko  
	rm -f *.o  
	rm -f *.mod.c  
	rm -f *.mod.o  
	rm -f *.order  
	rm -f *.sysvers
    #rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions modules.order Module.symvers  

	
.PHONY: 
	all modules_install clean  
  
#再次读取这个Makefile文件时，下面的执行 
#指定驱动模块的名字
else    
MODULE_NAME := spi_lcd     
# 注意：驱动模块的名字 千万 不能和本文件夹内的任何文件同名！ 
#指定驱动模块的核心文件（有init 和 exit）  
RESMAIN_CORE_OBJS := spi_lcd.o     
#依赖文件  
RESMAIN_GLUE_OBJS := spi.o xxx9341.o  
$(MODULE_NAME)-objs :=  $(RESMAIN_CORE_OBJS)   
#xxx-objs := 指定驱动模块的所有依赖文件  
#最终由xxx-objs链接生成lcd.o，再生成lcd.ko
obj-m := spi_lcd.o     
endif  
# change by myself for test   


