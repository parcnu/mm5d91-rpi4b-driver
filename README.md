# MM5D91 driver for RPI4.
* Still under construction. 
* is tested on bullseye and bookworm. 
## Wiring of RPI4B to mm5d91
![wiring](images/image.png)
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
make
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
## TODO
* missing unit tests
* add some checks to make more robust (i.e. GOTOs)
