# 一： Program function  
The program running on raspberry will create a file based on the startup time to record GPS and sensor data.
# 二：Compiling environment  
This program can be compiled under raspberry pie or the linux environment of PC.
Compile on linux: make TG=PC
Compile on pi: make TG=pi
# 三：Key Points:
For 3B and 3B +, the /dev/ttyAMA0 used to access UART is now connected to Bluetooth. So,If you want to use the /dev/ttyAMA0 port to    
communicate with other devices,please follow these 5 steps carefully.  
**Step 1 -** run sudo raspi-config,Select Interfacing Options/Serial  
    then specify if you want a Serial console (we select no)   
    ![未找到图片](https://github.com/Aceinna/openrtk/blob/rtk_pi_app_d/RTK_PI_APP_DAI/uart_config_1-1.png)  
    then if you want the Serial Port hardware enabled (we select yes).  
    ![未找到图片](https://github.com/Aceinna/openrtk/blob/rtk_pi_app_d/RTK_PI_APP_DAI/uart_config_1-2.png)  
**Step 2 -** Device Tree settings as below:  
    Add device tree to /boot/config.txt to disable the Raspberry Pi 3 bluetooth.  
    sudo vi /boot/config.txt  
    Add at the end of the file  
   *if you want to change the blutooth to miniuart port(bad)  
    dtoverlay=pi3-miniuart-bt-overlay  
    *if you want to disable the blutooth(good)  
    dtoverlay=pi3-disable-bt  
    Exit the editor saving your changes.    
**Step 3 -** reboot the pi   
    sudo reboot   
**step 4 -**
    a)to disable the Serial Console edit the file using  
    sudo vi /boot/cmdline.txt  
    remove the word phase "console=serial0,115200" or "console=ttyAMA0,115200"  
    Exit and save your changes  
    b)to Enable the Serial Console edit the file using  
    sudo vi /boot/cmdline.txt  
    Change the file to the following:  
    dwc_otg.lpm_enable=0 console=tty1 console=serial0(or ttyAMA0),115200 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline      fsck.repair=yes rootwait
    Exit and save your changes   
**Step 5 -** reboot the pi  
    sudo reboot  


## **The uart_config_file folder can be used as a reference for the files that will be modified above.** 
 
