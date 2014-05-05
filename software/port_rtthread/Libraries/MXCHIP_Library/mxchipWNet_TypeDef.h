#ifndef __MXCHIPWNET_TYPEDEF_H__
#define __MXCHIPWNET_TYPEDEF_H__

#define FALSE 0
#define TRUE 1

enum {
	mxDisable,
	mxEnable
} ;

typedef enum {
    WIFI_CHANNEL_1_11 = 0,
    WIFI_CHANNEL_1_13,
    WIFI_CHANNEL_1_14,
} WIFI_CHANNEL;

typedef enum {
  Soft_AP,
	Station
} WiFi_Interface;

typedef enum {
  DHCP_Disable,
  DHCP_Client,
  DHCP_Server
} DHCPOperationMode;


#endif
