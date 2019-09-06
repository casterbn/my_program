#include "com.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
/************打开串口函数****************/ 
int open_port(int com_port) 
{
    int fd; 
    /* 使用普通串口 */ 
    //char *dev[] = {"/dev/ttySAC0","/dev/ttySAC1","/dev/ttySAC2","/dev/ttySAC3"};  
    /* 使用 USB 转串口 */ 
    //char *dev[] = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"}; 
    char *dev[] = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"}; 
    printf("open dev [%s]\n",dev[com_port]);
    fd = open(dev[com_port], O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK); 
    if (fd < 0) 
    { 
        perror("open serial port"); 
        return(-1); 
    } 
    //恢复串口为阻塞状态 
    //非阻塞：fcntl(fd,F_SETFL,FNDELAY)  
    //阻塞：fcntl(fd,F_SETFL,0) 
    if (fcntl(fd, F_SETFL, O_RDWR) < 0) 
    { 
        perror("fcntl F_SETFL\n"); 
    } 
    /*测试是否为终端设备*/ 
    if (isatty(STDIN_FILENO) == 0) 
    { 
        perror("standard input is not a terminal device"); 
    } 
	
#if 0
	  struct termios options ;
  tcgetattr (fd, &options) ;

    cfmakeraw   (&options) ;
    cfsetispeed (&options, myBaud) ;
    cfsetospeed (&options, myBaud) ;

    options.c_cflag |= (CLOCAL | CREAD) ;
    options.c_cflag &= ~PARENB ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS8 ;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    options.c_oflag &= ~OPOST ;

    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)

  tcsetattr (fd, TCSANOW, &options) ;

  ioctl (fd, TIOCMGET, &status);

  status |= TIOCM_DTR ;
  status |= TIOCM_RTS ;

  ioctl (fd, TIOCMSET, &status);

  usleep (10000) ;	// 10mS	
 #endif
    return fd; 
} 

/************串口配置***************/
int set_com_config(int fd,int baud_rate,int data_bits, char parity, int stop_bits) 
{ 
    struct termios opt;
    int speed;
    if(tcgetattr(fd, &opt) != 0)  
    { 
        perror("tcgetattr"); 
        return -1; 
    }
    /* 设置字符大小*/ 
    //opt = opt;
    //cfmakeraw(&opt);//配置为原始模式
    //opt.c_cflag &= ~CSIZE;//c_cflag 控制模式标志
    /*设置波特率*/ 
    switch (baud_rate) 
    { 
        case 2400:  speed = B2400;  break; 
        case 4800:  speed = B4800;  break; 
        case 9600:  speed = B9600;  break; 
        case 19200: speed = B19200; break; 
        case 38400: speed = B38400; break;
        default:    speed = B115200;break;   
    } 
    cfsetispeed(&opt, speed); 
    cfsetospeed(&opt, speed); 
    tcsetattr(fd,TCSANOW,&opt);
    opt.c_cflag &= ~CSIZE;

    /*设置数据位*/ 
    switch (data_bits) 
    { 
        case 7: {opt.c_cflag |= CS7;}break;//7个数据位  
        default:{opt.c_cflag |= CS8;}break;//8个数据位 
    } 

    /*设置奇偶校验位*/ 
    switch (parity) //N
    {   
        case 'n':case 'N': 
        { 
            opt.c_cflag &= ~PARENB;//校验位使能     
            opt.c_iflag &= ~INPCK; //奇偶校验使能  
        }break;
        case 'o':case 'O': 
        { 
            opt.c_cflag |= (PARODD | PARENB);//PARODD使用奇校验而不使用偶校验 
            opt.c_iflag |= INPCK;
        }break; 
        case 'e':case 'E': 
        { 
            opt.c_cflag |= PARENB;   
            opt.c_cflag &= ~PARODD;  
            opt.c_iflag |= INPCK;    
        }break; 
        case 's':case 'S': /*as no parity*/  
        { 
        	opt.c_cflag &= ~PARENB; 
        	opt.c_cflag &= ~CSTOPB; 
        }break;
        default:
        {
            opt.c_cflag &= ~PARENB;//校验位使能     
            opt.c_iflag &= ~INPCK; //奇偶校验使能          	
        }break; 
     }
           
     /*设置停止位*/ 
     switch (stop_bits)
     { 
        case 1: {opt.c_cflag &=  ~CSTOPB;} break;
        case 2: {opt.c_cflag |= CSTOPB;}   break;
        default:{opt.c_cflag &=  ~CSTOPB;} break; 
     }     
    /*处理未接收字符*/ 
    tcflush(fd, TCIFLUSH); 
    /*设置等待时间和最小接收字符*/ 
    //opt.c_cc[VTIME]  = 100;	//11 
    //opt.c_cc[VMIN] = 0;     
	
	
	int status;
    opt.c_cflag |= (CLOCAL | CREAD) ;
    opt.c_cflag &= ~PARENB ;
    opt.c_cflag &= ~CSTOPB ;
    opt.c_cflag &= ~CSIZE ;
    opt.c_cflag |= CS8 ;
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    opt.c_oflag &= ~OPOST ;

    opt.c_cc [VMIN]  =   0 ;
    opt.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)	
    /*关闭串口回显*/
    opt.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHOK|ECHONL|NOFLSH); 
  tcsetattr (fd, TCSANOW, &opt) ;

  ioctl (fd, TIOCMGET, &status);
  
    status |= TIOCM_DTR ;
  status |= TIOCM_RTS ;

  ioctl (fd, TIOCMSET, &status);
    /*激活新配置*/ 
    if((tcsetattr(fd, TCSANOW, &opt)) != 0) 
    { 
        perror("tcsetattr"); 
        return -1; 
    }      
    return 0; 
}
