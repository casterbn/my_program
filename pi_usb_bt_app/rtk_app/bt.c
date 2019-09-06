#include "bt.h"


//#include <bluetooth/tcs.h>

int s,client, bytes_read,result;
char bt_buf[1024] ={0};//,*addr;
int bt_state;
extern int fd;
extern unsigned char buf[1024];
extern int read_buffer_size;
int bt_config()					//蓝牙作为服务端
{
	  printf("start test\n");
    struct sockaddr_rc loc_addr ={0},rem_addr={0};


   	printf("start test1\n");
    int opt = sizeof(rem_addr);
   
    printf("Creating socket...\n");
    s = socket(PF_BLUETOOTH,SOCK_STREAM,BTPROTO_RFCOMM);
    if(s<0)
    {
       perror("create socket error");
       exit(1);
    }
    else
    {
       printf("success!\n");
    }
    loc_addr.rc_family=AF_BLUETOOTH;
    loc_addr.rc_bdaddr=*BDADDR_ANY;
    loc_addr.rc_channel=(uint8_t)1;
   
    printf("Binding socket...\n");
    result=bind(s,(struct sockaddr *)&loc_addr, sizeof(loc_addr));
    if(result<0)
    {
       perror("bind socket error:");
       exit(1);
    }
    else
    {
       printf("success!\n");
    }
   /*result=ba2str(&loc_addr.rc_bdaddr,addr);
   if(result<0)
     {
       perror("addr convert error");
       exit(1);
     }
     printf("local addr is:%s\n",addr);*/
    printf("Listen... ");
    result=listen(s,1);
    if(result<0)
    {
       printf("error:%d\n:",result);
       perror("listen error:");
       exit(1);
    }
    else
    {
       printf("requested!\n");
    }
    printf("Accepting...\n");
    client= accept(s,(struct sockaddr *)&rem_addr,&opt);
    if(client<0)
    {
       perror("accept error");
       exit(1);
    }
    else
    {
       printf("OK!\n");
    }
    ba2str(&rem_addr.rc_bdaddr,bt_buf);
    fprintf(stderr,"accepted connection from %s \n",bt_buf);
   bt_state = 1;
#if 0
      while(1)
    {
      bytes_read = read(client,buf,sizeof(buf));
      if(bytes_read>0){
      printf("received:\n");
      for(int i = 0;i < bytes_read;i++)
      {
         printf("%x",buf[i]);
      }
      if(strcmp(buf,"exit")==0)
         exit(1);
      memset(buf,0,bytes_read);
      }
     } 

   close(client);
   close(s);
#endif
   return 0 ;
}




void *monitor_bt_pthread(void *arg)
{
   printf("bt pthread start\n");
   while(1)
     {
       bytes_read = read(client,bt_buf,sizeof(bt_buf));
       if(bytes_read>0){
       //printf("received[%s]\n",bt_buf);
       //write(client,bt_buf,bytes_read);
       write(fd,bt_buf,bytes_read);
       if(strcmp(bt_buf,"exit")==0)
       exit(1);
       memset(bt_buf,0,bytes_read);

       }
     } 
   close(client);
   close(s);
}



#if 0					//蓝牙作为客户端






int main ( int argc , char **argv )
{
   inquiry_info *ii = NULL;
   int max_rsp, num_rsp;
   int dev_id, sock, len, flags;
   int i;
   char addr [19] = { 0 };
   char name [248] = { 0 };
   dev_id = hci_get_route (NULL);   //得到本地第一个可用的蓝牙设备

   sock = hci_open_dev(dev_id);     //用打开蓝牙设备.
   if( dev_id<0 || sock < 0) {
       perror("opening socket error") ;
       exit(1) ;
   }
   len = 8 ;
   max_rsp = 255 ;
   flags = IREQ_CACHE_FLUSH;
   ii = (inquiry_info*)malloc (max_rsp* sizeof ( inquiry_info)) ;
   
   printf("start search...\n");
#if 0
   num_rsp = hci_inquiry(dev_id , len , max_rsp , NULL, &ii , flags) ;   //检索周围是否有设备
   if ( num_rsp < 0 ) perror ("hci_inquiry error") ;
   for ( i = 0 ; i < num_rsp ; i++) {
       ba2str (&(ii+i)->bdaddr , addr ) ;
       memset (name , 0 , sizeof (name)) ;
       if( hci_read_remote_name ( sock , &( ii+i )->bdaddr , sizeof (name) ,
          name , 0) < 0)   //查询设备的友好设备名
          strcpy (name , "[unknown]") ;
       printf ("%s %s \n", addr , name ) ;
   }
#endif
   printf("end search.\n");
   printf("start connect.\n");
   //tcs_init();
   read_write_bt();
   free(ii);
   close(sock);
   return 0;
}



#if 0			
void read_write_bt()		//从
{
	
    //struct sockaddr_rc addr = { 0 };
    struct sockaddr_l2 addr = { 0 },rem_addr = { 0 };
	int opt = sizeof(rem_addr);
	int s, status, len=0;
    char dest[18] = "98:2C:BC:33:1D:E0";
	//char dest[18] = "20:54:FA:B4:8A:D0";
    char buf[256];
    // allocate a socket
    s = socket(PF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.l2_family = AF_BLUETOOTH;
    addr.l2_psm = 1;
	addr.l2_bdaddr = *BDADDR_ANY;
    str2ba( dest, &addr.l2_bdaddr );

    // connect to server
    //status = connect(s, (struct sockaddr *)&addr, sizeof(addr));
	status = bind(s,(struct sockaddr*)&addr, sizeof(addr));
	   if(status < 0)
     {
       printf("bind socket error...\n");
       close(s);
       exit(1);
     }
   else
     {
       printf("    binding socket success!\n");
     }
	
   printf("(3) listening..\n");
   status = listen(s,1);

   if(status < 0)
     {
       printf("listening error...\n");
       exit(1);
     }
   else
     {
       printf("    requested!\n");
     }
   status = accept(s,(struct sockaddr*)&rem_addr,&opt);//accept(s, (struct sockaddr *)&addr, sizeof(addr));
    if(status < 0)
   {
      printf("accept error\n");
      exit(1);
   }
   else
   {
      printf("accept succes!\n\n");
   }
    do{
        len = read(s, buf, sizeof buf);
    printf("wait\n");
     if( len>0 ) {
         buf[len]=0;
         printf("%s\n",buf);
         write(s, buf, strlen(buf));
     }
    }while(len>0);

    close(s);
    return 0;
}


#else
void read_write_bt()
{
	
    struct sockaddr_rc addr = { 0 };
    int s, status, len=0;
    //char dest[18] = "98:2C:BC:33:1D:E0";	//MP
	//char dest[18] = "20:54:FA:B4:8A:D0";	//PC
	char dest[18] = "24:0A:C4:85:C8:E2";	//esp32
    char buf[256];
    // allocate a socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba(dest, &addr.rc_bdaddr );

    // connect to server
    status = connect(s, (struct sockaddr *)&addr, sizeof(addr));
	
    if(status){
        printf(" failed to connect the device!\n");
        return -1;
    }

    
    do{
        len = read(s, buf, sizeof buf);
    
     if( len>0 ) {
         buf[len]=0;
         printf("%s\n",buf);
         write(s, buf, strlen(buf));
     }
    }while(len>0);
	printf("exit\n");
    close(s);
    return 0;
}
#endif


#endif