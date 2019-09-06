#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__
#include "lwip/err.h"
#include "lwip/netif.h"


#define IFNAME0 'c'
#define IFNAME1 'h'
 

err_t ethernetif_init(struct netif *netif);
err_t ethernetif_input(struct netif *netif);
#endif
