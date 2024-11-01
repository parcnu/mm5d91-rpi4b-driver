# MM5D91 driver for RPI4.
* Still under construction. 
* is tested on bullseye and bookworm. 
* in tested in Yocto build scathgap
## Wiring of RPI4B to mm5d91
![wiring](images/image.png)
# Installing driver directly to RPI4B
## Prerequisite
```
sudo apt install raspberrypi-kernel-headers
sudo cp mm5d91_overlay.dtbo /boot/overlays
```
### add "dtoverlay=mm5d91_overlay" to /boot/config.txt
```
sudo vi /boot/config.txt
````
### Run raspi-config
```
sudo raspi-config
```
### Change following to raspi-config:
```
    interfaces->serial->login shell to be accessible over serial -> NO
    interfaces->serial->Serial port hardware enabled -> YES
    reboot raspi
```
## Install driver to kernel
```
sudo insmod mm5d91_driver.ko
```
## Remove driver from kernel
```
sudo rmmod mm5d91_driver
```
## Compile driver
```
make Makefile-rpi
```
# Userapp 
## Compile userapp
```
gcc usertestapp.c -o usertestapp -lpthread
```
## Run user app
```
./usertestapp &
```
* output.log file is created in the same folder. Each time app is started output.log will be over written.
* you may follow file by 
'''
tail -f output.txt
'''

### show the 50 last lines of the output.log:
```
tail -n 50 output.log 
```

## Kill the running app
```
kill -SIGTERM <pid>
```
* pid is printed out when app is starting up.  
## Message path from kernel to user app.
![MSG](images/msg_path.png)
# Istalling to Yocto build
## Prequiresite
* your yocto sources are installed in some folder. 
* see instruction <here>
## Build and Install
* move to your selected <path> where driver is planned to downloaded.
* git clone https://github.com/parcnu/mm5d91-rpi4b-driver.git
* cd <your yocto source path>
* follow the Yocto build instruction given in https://github.com/parcnu/mm5d91-yocto-main/tree/scarthgap
# TODO
* missing unit tests
* add some checks to make more robust (i.e. GOTOs)
