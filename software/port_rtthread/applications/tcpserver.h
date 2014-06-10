#ifndef __TCPSERVER_H
#define __TCPSERVER_H
#include <rtthread.h>
#include "board.h"
#include "drv_wifi.h"

void tcpserver_init(void);
int  tcp_send(const void *buf, size_t len);
#endif 

