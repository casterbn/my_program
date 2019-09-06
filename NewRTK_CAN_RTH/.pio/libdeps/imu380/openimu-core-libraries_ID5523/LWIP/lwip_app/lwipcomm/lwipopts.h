#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#ifndef TCPIP_THREAD_PRIO
#define TCPIP_THREAD_PRIO 5
#endif
#undef DEFAULT_THREAD_PRIO
#define DEFAULT_THREAD_PRIO 2

#define SYS_LIGHTWEIGHT_PROT    1
#define NO_SYS                  0
#define MEM_ALIGNMENT           4

#define MEM_SIZE                16000

#define MEMP_NUM_PBUF           20

#define MEMP_NUM_UDP_PCB        6

#define MEMP_NUM_TCP_PCB        10

#define MEMP_NUM_TCP_PCB_LISTEN 6

#define MEMP_NUM_TCP_SEG        15

#define MEMP_NUM_SYS_TIMEOUT    8


/* ---------- Pbuf ---------- */
#define PBUF_POOL_SIZE          20		
#define PBUF_POOL_BUFSIZE       512	


/* ---------- TCP ---------- */
#define LWIP_TCP                1  		
#define TCP_TTL                 255	

#undef TCP_QUEUE_OOSEQ
#define TCP_QUEUE_OOSEQ         0 		

#undef TCPIP_MBOX_SIZE
#define TCPIP_MBOX_SIZE         MAX_QUEUE_ENTRIES   

#undef DEFAULT_TCP_RECVMBOX_SIZE
#define DEFAULT_TCP_RECVMBOX_SIZE       MAX_QUEUE_ENTRIES  

#undef DEFAULT_ACCEPTMBOX_SIZE
#define DEFAULT_ACCEPTMBOX_SIZE         MAX_QUEUE_ENTRIES  



#define TCP_MSS                 (1500 - 40)	
#define TCP_SND_BUF             (4*TCP_MSS)		
#define TCP_SND_QUEUELEN        (2* TCP_SND_BUF/TCP_MSS) 
#define TCP_WND                 (2*TCP_MSS)


/* ---------- ICMP ---------- */
#define LWIP_ICMP               1 	

/* ---------- DHCP ---------- */
#define LWIP_DHCP               0			

/* ---------- UDP ----------- */ 
#define LWIP_UDP                1 
#define UDP_TTL                 255 

/* ---------- Statistics options ---------- */
#define LWIP_STATS 0
#define LWIP_PROVIDE_ERRNO 1


#define CHECKSUM_BY_HARDWARE

#ifdef CHECKSUM_BY_HARDWARE
  #define CHECKSUM_GEN_IP                 0
  #define CHECKSUM_GEN_UDP                0
  #define CHECKSUM_GEN_TCP                0 
  #define CHECKSUM_CHECK_IP               0
  #define CHECKSUM_CHECK_UDP              0
  #define CHECKSUM_CHECK_TCP              0
  #define CHECKSUM_GEN_ICMP               0
#else
  #define CHECKSUM_GEN_IP                 1
  #define CHECKSUM_GEN_UDP                1
  #define CHECKSUM_GEN_TCP                1
  #define CHECKSUM_CHECK_IP               1
  #define CHECKSUM_CHECK_UDP              1
  #define CHECKSUM_CHECK_TCP              1
  #define CHECKSUM_GEN_ICMP               1
#endif


/*
   -------------------------------------------
   ---------- SequentialAPI----------
   -------------------------------------------
*/

#define LWIP_NETCONN                    1

/*
   ----------------------------------
   ---------- Socket API----------
   ----------------------------------
*/

#define LWIP_SOCKET                     1	
#define LWIP_COMPAT_MUTEX               1
#define LWIP_SO_RCVTIMEO                1 

#define TCPIP_THREAD_STACKSIZE 				1000 
#define DEFAULT_UDP_RECVMBOX_SIZE			2000
#define DEFAULT_THREAD_STACKSIZE 			512

/*
   --------------------------------------
   ---------- Lwip DEBUG----------
   --------------------------------------
*/
#define LWIP_DEBUG			0 								

#define ICMP_DEBUG			LWIP_DBG_OFF 		

#endif /* __LWIPOPTS_H__ */

