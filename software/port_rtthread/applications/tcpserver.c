#include "tcpserver.h"

#ifdef RT_USING_FINSH
#include <finsh.h>
#endif

/* TCP Server 监听的端口号 */
#define LISTEN_PORT 8888
/* 允许的最大客户端连接个数 */
#define MAX_CLIENT_NUM 4

int clientfd[MAX_CLIENT_NUM];


void tcpserver_thread_entry(void *p)
{
  int i, new_client, fd_listen = -1;
  char *buf, ip_address[16];
  int count = -1;
  
  fd_set readfds, exceptfds;
  struct timeval_t timeout;
  struct sockaddr_t addr;
  socklen_t addrLen;

	int bufferSize;
	
	  for(i=0;i<MAX_CLIENT_NUM;i++) 
    clientfd[i] = -1;

  buf = (char*)malloc(1*1024);
	
	/*设置select的超时时间*/
  timeout.tv_sec = 0;
  timeout.tv_usec = 100;
	/*设置TCP检测是否断开时间*/
  set_tcp_keepalive(3, 60);
	
	/*建立TCPServer 监听*/
	fd_listen = wifi_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);		
			  bufferSize = 5*1024;
				wifi_setsockopt(fd_listen,0,SO_RDBUFLEN,&bufferSize,4);
				bufferSize = 5*1024;
				wifi_setsockopt(fd_listen,0,SO_WRBUFLEN,&bufferSize,4);
      addr.s_port = LISTEN_PORT;
      wifi_bind(fd_listen, &addr, sizeof(addr));
      wifi_listen(fd_listen, 0);
      rt_kprintf("TCP Server listen at port: %d \r\n", addr.s_port);
			
	while(1){

		/* 检测是否有客户端断开 */
		FD_ZERO(&readfds);
		FD_SET(fd_listen, &readfds);	
		for(i=0;i<4;i++) {
			if (clientfd[i] != -1)
				FD_SET(clientfd[i], &readfds);
		}
		
		wifi_select(1, &readfds, NULL, &exceptfds, &timeout);
    
    /* 检测是否有新的连接连入 */
		if(FD_ISSET(fd_listen, &readfds))
		{
			new_client = wifi_accept(fd_listen, &addr, &addrLen);
			if (new_client > 0) {
			  inet_ntoa(ip_address, addr.s_ip );
			  rt_kprintf("Client %s:%d connected\r\n", ip_address, addr.s_port);
			  for(i=0;i<MAX_CLIENT_NUM;i++) {
				  if (clientfd[i] == -1) {
					  clientfd[i] = new_client;
					  break;
				  }
			  }
			}
		}
		
    
   /* 读出接收到的数据，并回发 */ 
	 for(i=0;i<MAX_CLIENT_NUM;i++) {
      if (clientfd[i] != -1) {
        if (FD_ISSET(clientfd[i], &readfds)) {
          count = wifi_recv(clientfd[i], buf, 1*1024, 0);
          if (count > 0) {
            wifi_send(clientfd[i], buf, count, 0);
					}
          else {
            wifi_close(clientfd[i]);
            clientfd[i] = -1;
          }
        }
        else if (FD_ISSET(clientfd[i], &exceptfds))
          clientfd[i] = -1;
      }
    }
	  rt_thread_delay(5);	
	}
}
int tcp_send(const void *buf, size_t len)
{
	rt_uint8_t i;
	int ret=-1;
   /* 发送数据给连接上的每个客户端 */ 
	 for(i=0;i<MAX_CLIENT_NUM;i++) {
      if (clientfd[i] != -1) {
			 ret=wifi_send(clientfd[i], buf, len, 0);
			}
 }
	 return ret;
}
#ifdef RT_USING_FINSH
FINSH_FUNCTION_EXPORT(tcp_send,send data to client);
#endif
void tcpserver_init(void)
{
  rt_thread_t tcpserver_thread;
    // TCP Server thread.
    tcpserver_thread = rt_thread_create("tcpserver", tcpserver_thread_entry, RT_NULL, 2048, 16, 5);
	  if(tcpserver_thread!=RT_NULL)
    rt_thread_startup(tcpserver_thread);
}
#ifdef RT_USING_FINSH
FINSH_FUNCTION_EXPORT(tcpserver_init,init the tcpserver);
#endif











