obj-m += globalmem.o

PWD=$(shell pwd)  
KERNEL_DIR=/home/ram/openwrt/build_dir/target-mipsel_24kec+dsp_uClibc-0.9.33.2/linux-ramips_mt7688/linux-3.18.45
TOOLCHAIN="/home/ram/openwrt/staging_dir/toolchain-mipsel_24kec+dsp_gcc-4.8-linaro_uClibc-0.9.33.2/bin/mipsel-openwrt-linux-"  

all:  
	make -C $(KERNEL_DIR)  modules M=$(PWD)   ARCH=mips CROSS_COMPILE=$(TOOLCHAIN)  
      	 
clean:  
	rm -f *.ko  
	rm -f *.o  
	rm -f *.mod.c  
	rm -f *.mod.o  
	rm -f *.order  
	rm -f *.symvers
