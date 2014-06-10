/*
 * File      : drv_wifi.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2014, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-03-29     xiaonong      the first version
 */

#ifndef __DRV_WIFI_H
#define __DRV_WIFI_H
#include "mxchipWNET.h"

void stationModeStart(void);
void softAPModeStart(void);
int wifi_thread_init(void);

/* wifiÍøÂçÏà¹ØAPI*/
 int wifi_socket(int domain, int type, int protocol);
 int wifi_setsockopt(int sockfd, int level, int optname,const void *optval, socklen_t optlen);
 int wifi_bind(int sockfd, const struct sockaddr_t *addr, socklen_t addrlen);
 int wifi_connect(int sockfd, const struct sockaddr_t *addr, socklen_t addrlen);
 int wifi_listen(int sockfd, int backlog);
 int wifi_accept(int sockfd, struct sockaddr_t *addr, socklen_t *addrlen);
 int wifi_select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval_t *timeout);
ssize_t wifi_send(int sockfd, const void *buf, size_t len, int flags);
ssize_t  wifi_sendto(int  sockfd,  const  void  *buf,  size_t  len,  int  flags,const  struct  sockaddr_t  *dest_addr, 
				socklen_t addrlen);
ssize_t wifi_recv(int sockfd, void *buf, size_t len, int flags);
ssize_t wifi_recvfrom(int  sockfd,  void  *buf,  size_t  len,  int  flags,struct  sockaddr_t  *src_addr,  socklen_t 
					*addrlen);
int wifi_read(int sockfd, void *buf, size_t len); 
int wifi_write(int sockfd, void *buf, size_t len); 
int wifi_close(int fd);

#endif
