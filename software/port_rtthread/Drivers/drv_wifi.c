/*
 * File      : rtc.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2014, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-03-29     xiaonong     the first version
 */

#include <rtthread.h>
#include "drv_wifi.h"
#include "mxchipWNet_TypeDef.h"

#define RT_WIFI_ETHTHREAD_PRIORITY	14
/*
Enable IEEE powersave mode,
*/
#define LowPowerMode
/*
Under this configuration, you can allocate every socket's send/recv buffer for application's requirement.
The more buffer is allocated, faster the speed would be, but the max connection number would be less.
If there is not enough memory for a connection:
	TCP client: function: connect would return fail
	TCP server: Any connection to the server would reset
If the buffer is not setted, stack would use the default size: 2048bytes.
*/
//#define DynamicMemAlloc          
#define AP_NAME           "rtthread_ddwrt"
#define AP_PASSWORD       "rtthread"

network_InitTypeDef_st wNetConfig;
net_para_st para;
static rt_mutex_t wifi_lock;
	
void userWatchDog(void)
{
}

void WifiStatusHandler(int event)
{
  switch (event) {
    case MXCHIP_WIFI_UP:
      rt_kprintf("Station up \r\n");
      break;
    case MXCHIP_WIFI_DOWN:
      rt_kprintf("Station down \r\n");
      break;
    case MXCHIP_UAP_UP:
      rt_kprintf("uAP up \r\n");
      getNetPara(&para, Soft_AP);
      rt_kprintf("Soft AP mode: IP address: %s \r\n", para.ip);
      rt_kprintf("Soft AP mode: NetMask address: %s \r\n", para.mask);
      rt_kprintf("Soft AP mode: MAC address: %s \r\n", para.mac);
      break;
    case MXCHIP_UAP_DOWN:
      rt_kprintf("uAP down \r\n");
      break;
    default:
      break;
  }
  return;
}
void ApListCallback(UwtPara_str *pApList)
{
	int i;
  rt_kprintf("Find %d APs: \r\n", pApList->ApNum);
  for (i=0;i<pApList->ApNum;i++)
    rt_kprintf("    SSID: %s, Signal: %d%%\r\n", pApList->ApList[i].ssid, pApList->ApList[i].ApPower);
}

void NetCallback(net_para_st *pnet)
{
	rt_kprintf("Station mode: IP address: %s \r\n", pnet->ip);
	rt_kprintf("Station mode: NetMask address: %s \r\n", pnet->mask);
	rt_kprintf("Station mode: Gateway address: %s \r\n", pnet->gate);
	rt_kprintf("Station mode: DNS server address: %s \r\n", pnet->dns);
  rt_kprintf("Station mode: MAC address: %s \r\n", pnet->mac);
}
void stationModeStart(void)
{
	int ret;
  memset(&wNetConfig, 0x0, sizeof(network_InitTypeDef_st));
	
	wNetConfig.wifi_mode = Station;
	strcpy((char*)wNetConfig.wifi_ssid, AP_NAME);
	strcpy((char*)wNetConfig.wifi_key, AP_PASSWORD);
	wNetConfig.dhcpMode = DHCP_Client;
	ret = StartNetwork(&wNetConfig);
	rt_kprintf("connect to %s....., return %d\r\n", wNetConfig.wifi_ssid, ret);
}

void softAPModeStart(void)
{
  int ret;
  memset(&wNetConfig, 0x0, sizeof(network_InitTypeDef_st));
	
	wNetConfig.wifi_mode = Soft_AP;
  strcpy((char*)wNetConfig.wifi_ssid, "Soft AP test");
  strcpy((char*)wNetConfig.wifi_key, "bcn224344");
	strcpy((char*)wNetConfig.local_ip_addr, "192.168.0.1");
  strcpy((char*)wNetConfig.net_mask, "255.255.255.0");
	strcpy((char*)wNetConfig.address_pool_start, "192.168.0.10");
	strcpy((char*)wNetConfig.address_pool_end, "192.168.0.177");
	wNetConfig.dhcpMode = DHCP_Server;
	ret = StartNetwork(&wNetConfig);
	rt_kprintf("Setup soft AP: %s, return %d\r\n", wNetConfig.wifi_ssid, ret);
}
void RptConfigmodeRslt(network_InitTypeDef_st *nwkpara)
{
	if(nwkpara == NULL){
		rt_kprintf("Configuration failed\r\n");
	}
	else{
		rt_kprintf("Configuration is successful, SSID:%s, Key:%s\r\n", \
																		nwkpara->wifi_ssid,\
																		nwkpara->wifi_key);
	}
}
 int wifi_socket(int domain, int type, int protocol)
{
  int ret;
	rt_mutex_take(wifi_lock,RT_WAITING_FOREVER);
	ret = socket(domain, type, protocol);
	rt_mutex_release(wifi_lock);
	return ret;
}
 int wifi_setsockopt(int sockfd, int level, int optname,const void *optval, socklen_t optlen)
{
  int ret;
	rt_mutex_take(wifi_lock,RT_WAITING_FOREVER);
	ret = setsockopt(sockfd, level, optname,optval, optlen);
	rt_mutex_release(wifi_lock);
	return ret;
}
 int wifi_bind(int sockfd, const struct sockaddr_t *addr, socklen_t addrlen)
{
  int ret;
	rt_mutex_take(wifi_lock,RT_WAITING_FOREVER);
	ret = bind(sockfd, addr, addrlen);
	rt_mutex_release(wifi_lock);
	return ret;
}
 int wifi_connect(int sockfd, const struct sockaddr_t *addr, socklen_t addrlen)
{
  int ret;
	rt_mutex_take(wifi_lock,RT_WAITING_FOREVER);
	ret = connect(sockfd, addr, addrlen);
	rt_mutex_release(wifi_lock);
	return ret;
}
 int wifi_listen(int sockfd, int backlog)
{
	int ret;
	rt_mutex_take(wifi_lock,RT_WAITING_FOREVER);
	ret = listen(sockfd , backlog);
	rt_mutex_release(wifi_lock);
	return ret;
}
 int wifi_accept(int sockfd, struct sockaddr_t *addr, socklen_t *addrlen)
{
  int ret;
	rt_mutex_take(wifi_lock,RT_WAITING_FOREVER);
	ret = accept(sockfd,addr,addrlen);
	rt_mutex_release(wifi_lock);
	return ret;
}
 int wifi_select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval_t *timeout)
{
int ret;
	rt_mutex_take(wifi_lock,RT_WAITING_FOREVER);
	ret = select(nfds, readfds, writefds, exceptfds, timeout);
	rt_mutex_release(wifi_lock);
	return ret;
}
ssize_t wifi_send(int sockfd, const void *buf, size_t len, int flags)
{
	ssize_t size;
	rt_mutex_take(wifi_lock,RT_WAITING_FOREVER);
	size=send(sockfd, buf, len,flags);
	rt_mutex_release(wifi_lock);
	return size;
}
ssize_t wifi_sendto(int  sockfd,  const  void  *buf,  size_t  len,  int  flags,const  struct  sockaddr_t  *dest_addr, 
				socklen_t addrlen)
{
	ssize_t size;
	rt_mutex_take(wifi_lock,RT_WAITING_FOREVER);
	size=sendto(sockfd, buf, len, flags,dest_addr, addrlen);
	rt_mutex_release(wifi_lock);
	return size;
}
ssize_t wifi_recv(int sockfd, void *buf, size_t len, int flags)
{
	ssize_t size;
	rt_mutex_take(wifi_lock,RT_WAITING_FOREVER);
	size=recv(sockfd, buf, len,flags);
	rt_mutex_release(wifi_lock);
	return size;
}
ssize_t wifi_recvfrom(int  sockfd,  void  *buf,  size_t  len,  int  flags,struct  sockaddr_t  *src_addr,  socklen_t 
					*addrlen)
{
  ssize_t size;
	rt_mutex_take(wifi_lock,RT_WAITING_FOREVER);
	size=recvfrom(sockfd, buf, len,flags,src_addr,addrlen);
	rt_mutex_release(wifi_lock);
	return size;
}
int wifi_read(int sockfd, void *buf, size_t len)
{
  int ret;
	rt_mutex_take(wifi_lock,RT_WAITING_FOREVER);
	ret = read(sockfd,buf,len);
	rt_mutex_release(wifi_lock);
	return ret;
}
int wifi_write(int sockfd, void *buf, size_t len)
{
  int ret;
	rt_mutex_take(wifi_lock,RT_WAITING_FOREVER);
	ret = write(sockfd,buf,len);
	rt_mutex_release(wifi_lock);
	return ret;
}
int wifi_close(int fd)
{
  int ret;
	rt_mutex_take(wifi_lock,RT_WAITING_FOREVER);
	ret = close(fd);
	rt_mutex_release(wifi_lock);
	return ret;
}
//void wifi_thread_entry(void* parameter)
//{
//	mxchipStartScan();
//  stationModeStart();
//  softAPModeStart();
//	while(1){
//		
//	rt_mutex_take(wifi_lock,RT_WAITING_FOREVER);
//		
//  mxchipTick();
//		
//  rt_mutex_release(wifi_lock);
//		
//	rt_thread_delay(5);
//	}
//}

int wifi_thread_init(void)
{
//	rt_thread_t tid;

  wifi_lock=rt_mutex_create("wifi",RT_IPC_FLAG_FIFO);
   	mxchipStartScan();
  stationModeStart();
  softAPModeStart();
//	tid = rt_thread_create("mx_wifi",
//								wifi_thread_entry, RT_NULL,
//								2048,RT_WIFI_ETHTHREAD_PRIORITY, 5);

//	if (tid != RT_NULL)
//		rt_thread_startup(tid);

	return 0;
}
