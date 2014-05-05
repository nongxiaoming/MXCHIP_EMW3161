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
#define AP_NAME           "rtthread_11n"
#define AP_PASSWORD       "rtthread_finsh"
#define WEB_SERVER				"www.baidu.com"
#define APP_INFO          "mxchipWNet Demo: TCP UDP ECHO"

network_InitTypeDef_st wNetConfig;
net_para_st para;

void system_version(char *str, int len)
{
  rt_snprintf( str, len, "%s", APP_INFO);
}
		
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
void wifi_thread_entry(void* parameter)
{
	mxchipStartScan();
  stationModeStart();
  softAPModeStart();
	while(1){
  mxchipTick();
	rt_thread_delay(1);
	}
}

int wifi_thread_init(void)
{
	rt_thread_t tid;

	tid = rt_thread_create("mx_wifi",
								wifi_thread_entry, RT_NULL,
								2048,RT_WIFI_ETHTHREAD_PRIORITY, 16);

	if (tid != RT_NULL)
		rt_thread_startup(tid);

	return 0;
}
