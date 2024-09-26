obj-m += mm5d91_driver.o

all: module dt
	echo Builded Device Tree Overlay and kernel module

module:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
dt: mm5d91_overlay.dts
	dtc -@ -I dts -O dtb -o mm5d91_overlay.dtbo mm5d91_overlay.dts
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -rf mm5d91_overlay.dtbo
	
