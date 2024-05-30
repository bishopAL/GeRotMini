# GeRotMini

you can follow these steps in  to complete raspberry cm4 image burning;
open the raspiberrConfig(The folder contains Instructional Videos, as well as Raspberry Pi burn-in software and instructional videos.)
First, download the two software in the file, and then check your cm4. You will find a small dial on it. Move it to the on position, connect it to the computer with a data cable, use the rpiboot downloaded "boot" to find your cm4, and then use another image-burning software to burn it
by the way, you can visit https://www.raspberrypi.com/software/ to download the latest image file.

Only support Raspbian GNU 10 (buster)

Check your system version with:
```
cat /etc/os-release
```
## Environment configuration
### 1. wiringpi
```
gpio -v
```
If the wiringpi package cannot be installed, try the following method. https://blog.csdn.net/weixin_42194402/article/details/123764537
```
cd /tmp
wget https://project-downloads.drogon.net/wiringpi-latest.deb
sudo dpkg -i wiringpi-latest.deb
```
### 2. ttyAMA0 port 
```
ls -l /dev
```
Check if serial0 is connected to ttyAMA0. Check https://blog.csdn.net/sinat_37939098/article/details/119344651 for more details.

### 3. System interface configuration

raspberry pi configuration-interfaces

（sudo raspi-config）

serial console --disable

serial port --enable

I2C --enable

### 4. Boot option
```
sudo vim /boot/config.txt
```

add these text at the bottom:

```
dtoverlay=pi3-miniuart-bt
```

### 5. Dynamixel API
https://github.com/bishopAL/GeRot/tree/master/API/dynamixel_cpp%20Ver2.0

```
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK/c++/build/linux_sbc
```

E: Makefile:57: *** missing separator (did you mean TAB instead of 8 spaces?)

change the space of Makefile Line:57 to an enter

```
make
sudo make install
```

### 6. eigen
```
sudo apt-get install libeigen3-dev
```
OR

install it directly: http://eigen.tuxfamily.org/index.php?title=Main_Page

put all the header files in	/usr/include/


### 7. qp
```
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
mkdir build && cd build
cmake ..
sudo make
sudo make install
```



