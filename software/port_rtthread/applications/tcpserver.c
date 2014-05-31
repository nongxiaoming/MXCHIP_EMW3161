#include "drv_wifi.h"

#include "tcpserver.h"

#include <finsh.h>


#define LISTEN_PORT 8888
#define BUF_SIZE 256




void tcpserver_thread_entry(void *p)
{
  int i, j, fd_listen = -1, fd_udp = -1, fd_client = -1;
  char *buf, ip_address[16],ipstr[32];
  int len;
  int con = -1;
	int opt = 0;
  int clientfd[8];
  fd_set readfds, exceptfds;
  struct timeval_t t;
  struct sockaddr_t addr;
  socklen_t addrLen;
  int timeout = 1;
	int bufferSize;
	
	  for(i=0;i<4;i++) 
    clientfd[i] = -1;

  buf = (char*)malloc(1*1024);
	
     t.tv_sec = 0;
  t.tv_usec = 100;
	
  set_tcp_keepalive(3, 60);
	
	while(1){
	/*Establish a TCP server that accept the tcp clients connections*/
		if (fd_listen==-1) {
      fd_listen = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);		
			  bufferSize = 5*1024;
				setsockopt(fd_listen,0,SO_RDBUFLEN,&bufferSize,4);
				bufferSize = 5*1024;
				setsockopt(fd_listen,0,SO_WRBUFLEN,&bufferSize,4);
      addr.s_port = 8080;
      bind(fd_listen, &addr, sizeof(addr));
      listen(fd_listen, 0);
      rt_kprintf("TCP server established at port: %d \r\n", addr.s_port);
    }

		
		/*Check status on erery sockets */
		FD_ZERO(&readfds);
		FD_SET(fd_listen, &readfds);	
		for(i=0;i<4;i++) {
			if (clientfd[i] != -1)
				FD_SET(clientfd[i], &readfds);
		}
		
		select(1, &readfds, NULL, &exceptfds, &t);
    
    /*Check tcp connection requests */
		if(FD_ISSET(fd_listen, &readfds))
		{
			j = accept(fd_listen, &addr, &len);
			if (j > 0) {
			  inet_ntoa(ip_address, addr.s_ip );
			  rt_kprintf("Client %s:%d connected\r\n", ip_address, addr.s_port);
			  for(i=0;i<4;i++) {
				  if (clientfd[i] == -1) {
					  clientfd[i] = j;
					  break;
				  }
			  }
			}
		}
		
    
   /*Read data from tcp clients and send data back */ 
	 for(i=0;i<4;i++) {
      if (clientfd[i] != -1) {
        if (FD_ISSET(clientfd[i], &readfds)) {
          con = wifi_recv(clientfd[i], buf, 1*1024, 0);
          if (con > 0) 
            wifi_send(clientfd[i], buf, con, 0);
          else {
            close(clientfd[i]);
            clientfd[i] = -1;
          }
        }
        else if (FD_ISSET(clientfd[i], &exceptfds))
          clientfd[i] = -1;
      }
    }
		
	}
}
void tcpserver_init(void)
{
  rt_thread_t tcpserver_thread;
    // TCP Server thread.
    tcpserver_thread = rt_thread_create("tcpserver", tcpserver_thread_entry, RT_NULL, 2048, 16, 5);
	  if(tcpserver_thread!=RT_NULL)
    rt_thread_startup(tcpserver_thread);
}












