# MM5D91 driver for RPI4.
* Still under construction. 
* missing message delivery to user.
* unit tests
* add some checks to make more robust
## Wiring of RPI4B to mm5d91
![wiring](images/image.png)
## Install to kernel
* sudo insmod mm5d91_driver.ko
## Remove from kernel
* sudo rmmod mm5d91_driver
## Compile
* make
