rtk_esp_bt_app
======================
First, build a cross-compiler environment by reading official documents of esp32

Then make flash step:
    1. make menuconfig.
    2. enter menuconfig "Component config", choose "Bluetooth"
    3. enter menu Bluetooth, choose "Classic Bluetooth" and "SPP Profile"
    4. edit sdkconfig file to set CONFIG_FREERTOS_UNICORE=y   (we use esp32 single-core chips)
    5. if you want to add a new file or folder, please refer to the current makefile and component.mk under the aceinna_app folder

After the program started, we can use bt device connect to esp32 bluetooth named ESP32_BT_KIT or ESP32_BT