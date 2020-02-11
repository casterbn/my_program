# vscode+esp-idf搭建esp32开发环境

## **一. 乐鑫开发框架esp-idf**  
ESP-IDF (Espressif IoT Development Framework)是乐鑫ESP32开发的一套框架，官方搭建教程如下：
> https://docs.espressif.com/projects/esp-idf/en/latest/
## **二. 下载并安装vscode**
配置文件settings.json中添加(根据esp-idf实际路径设置)：
'''
"terminal.integrated.shell.windows": "C:\\msys32\\msys2_shell.cmd",
"terminal.integrated.shellArgs.windows": ["-defterm", "-mingw32", "-no-start", "-here"]
'''

##  **三. vscode中打开esp32中断，输入make help查询编译指令。**

![未找到图片](/media/make_cmd.png)


