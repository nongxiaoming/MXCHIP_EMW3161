#ifndef __FILE_H__
#define __FILE_H__
#include "stdlib.h"
#include "string.h"
#include "stm32f2xx.h"

#define AF_INET 2
#define SOCK_STREAM 1
#define SOCK_DGRM 2 
#define IPPROTO_TCP 6
#define IPPROTO_UDP 17
#define SOL_SOCKET   1
#define INADDR_ANY   0
#define INADDR_BROADCAST 0xFFFFFFFF


#define u8 unsigned char
#define u16 unsigned short
#define u32 unsigned int


#ifndef ssize_t
#define ssize_t unsigned int
#endif

#ifndef size_t
#define size_t unsigned int
#endif

typedef struct  _component_baseinfo_st{ 
  char protocol_ver[32]; 
  char hw_ver[32]; 
  char sw_ver[32]; 
  char product_id[32]; 
  char product_date[32]; 
 } COMPONENT_BASEINFO_st; 
 
typedef struct _component_WIFIinfo_st{ 
  COMPONENT_BASEINFO_st baseinfo; 
  char swlst[32];//eg:softap/wps 
  char securityt[32];//eg:wpa/wep/wpa2 
  char wkmode[32];//eg: b/g/n/a/bg/gn/bgn 
} COMPONENT_WIFIINFO_st; 
 
typedef struct _component_BTinfo_st{ 
  COMPONENT_BASEINFO_st baseinfo; 
  int MFIflg;//if has MFI with CP 1-has ,0-none  
} COMPONENT_BTINFO_st; 

typedef struct _component_NFCinfo_st{ 
    COMPONENT_BASEINFO_st baseinfo;  
    char nfc_model_no[32];//product model number  
    char nfc_id[32];//product id  
    char tag_type;//tag0:0，tag1:1，tag2:2，tag3:3  
    char nfc_mode;//passive：0，active：1，bi_direction：2；  
} COMPONENT_NFCINFO_st; 

struct sockaddr_t {
   u16        s_type;
   u16        s_port;
   u32    	  s_ip;
   u16        s_spares[6];  /* unused in TCP realm */
} ;

typedef struct in_addr {
   __packed u32    s_addr;	//fancpp add 2007.3.14 __packed, align byte
} in_addr_t;


struct timeval_t {
	unsigned long		tv_sec;		/* seconds */
	unsigned long		tv_usec;	/* microseconds */
};

typedef long time_t; /* 时间值time_t 为长整型的别名*/; 

typedef  struct  _timeval_st{ 
    long tv_sec; /*1970年1月1日至今的秒数*/ 
    long tv_hmsec; /* 1970年1月1日至今的百豪秒数*/ 
}TIMEVAL_st; 

#define NULL 0

typedef int socklen_t;

typedef enum {
	SO_REUSEADDR = 2,         /* Socket always support this option */
	SO_BROADCAST = 6,		/* Socket always support this option */

    IP_ADD_MEMBERSHIP = 3, /* Join Multicast group */
    IP_DROP_MEMBERSHIP = 4, /* Leave Multicast group */
    
	SO_BLOCKMODE = 0x1000,  /* set socket as block/non-block mode, default is block mode */
 	SO_SNDTIMEO = 0x1005,	/* send timeout */
	SO_RCVTIMEO =0x1006	,	/* receive timeout */
	SO_CONTIMEO =0x1007	,	/* connect timeout */

    SO_RDBUFLEN = 0x1008,
    SO_WRBUFLEN = 0x1009,

    
} SOCK_OPT_VAL;

typedef struct _net_para {
    char dhcp;
	char ip[16]; // such as string  "192.168.1.1"
	char gate[16];
    char mask[16];
	char dns[16];
    char mac[16]; // such as string "7E0000001111"
    char broadcastip[16];
} net_para_st;

 
typedef  struct  _ApList_str  
{  
    char ssid[32];  
    char ApPower;  // min:0, max:100
}ApList_str; 


typedef  struct  _UwtPara_str  
{  
  char ApNum;       //AP number
  ApList_str * ApList; 
} UwtPara_str;  

typedef  enum {
    SECURITY_TYPE_NONE = 0,
    SECURITY_TYPE_WEP,
    SECURITY_TYPE_WPA_TKIP,
    SECURITY_TYPE_WPA_AES,
    SECURITY_TYPE_WPA2_TKIP,
    SECURITY_TYPE_WPA2_AES,
    SECURITY_TYPE_WPA2_MIXED,
    SECURITY_TYPE_AUTO,
} SECURITY_TYPE_E;

typedef  struct  _ApList_adv  
{  
    char ssid[32];  
    char ApPower;  // min:0, max:100
    char bssid[6];
    char channel;
    SECURITY_TYPE_E security;
}ApList_adv_t; 

typedef  struct  _adv_ap_info  
{  
    char ssid[32];  
    char bssid[6];
    char channel;
    SECURITY_TYPE_E security;
}apinfo_adv_t; 

typedef  struct  _UwtPara_adv  
{  
  char ApNum;       //AP number
  ApList_adv_t * ApList; 
} UwtPara_adv_t;  


typedef struct _uart_get_str 
{ 
    int BaudRate;    //The baud rate 
    char number;     //The number of data bits 
    char parity;     //The parity(0: none, 1:odd, 2:evn, default:0)
    char StopBits;      //The number of stop bits 
    char FlowControl;    //support flow control is 1 
}uart_get_str;

typedef struct _uart_set_str 
{ 
    char UartName[8];    // the name of uart 
    int BaudRate;    //The baud rate 
    char number;     //The number of data bits 
    char parity;     //The parity(default NONE) 
    char StopBits;      //The number of stop bits 
    char FlowControl;    //support flow control is 1 
}uart_set_str;

typedef struct _uart_str
{
	char baudrate;     //The baud rate, 0:9600, 1:19200, 2:38400, 3:57600, 4:115200, 5:230400, 6:460800, 7:921600 
	char databits;      //0:8, 1:9
	char parity;       //The parity(default NONE)  0:none, 1:even parity, 2:odd parity
	char stopbits;       //The number of stop bits ,  0:1, 1:0.5, 2:2, 3:1.5
} uart_str; 

typedef struct _network_InitTypeDef_st 
{ 
    char wifi_mode;    // SoftAp(0)，sta(1)  
    char wifi_ssid[32]; 
    char wifi_key[64]; 
    char local_ip_addr[16]; 
    char net_mask[16]; 
    char gateway_ip_addr[16]; 
    char dnsServer_ip_addr[16]; 
    char dhcpMode;       // disable(0), client mode(1), server mode(2) 
    char address_pool_start[16]; 
    char address_pool_end[16]; 
    int wifi_retry_interval;//sta reconnect interval, ms
} network_InitTypeDef_st; 

typedef struct _network_InitTypeDef_adv_st 
{ 
    apinfo_adv_t ap_info;
    SECURITY_TYPE_E security;
    char key[64];
    int key_len;
    char local_ip_addr[16]; 
    char net_mask[16]; 
    char gateway_ip_addr[16]; 
    char dnsServer_ip_addr[16]; 
    char dhcpMode;       // disable(0), client mode(1), server mode(2) 
    char address_pool_start[16]; 
    char address_pool_end[16]; 
    int wifi_retry_interval;//sta reconnect interval, ms
} network_InitTypeDef_adv_st; 

typedef struct _sta_ap_state{
    int is_connected;
    int wifi_strength;
    u8  ssid[32];
    u8  bssid[6];
    int channel;
}sta_ap_state_t;

struct wifi_InitTypeDef
{
	u8 wifi_mode;		// adhoc mode(1), AP client mode(0), AP mode(2)
	u8 wifi_ssid[32];
	u8 wifi_key[32];
};

enum {
    WIFI_CHANEEL_1_11 = 0,
    WIFI_CHANEEL_1_13,
    WIFI_CHANEEL_1_14,
};

typedef enum {
	MXCHIP_SUCCESS = 0,
	MXCHIP_FAILED = -1,
	MXCHIP_8782_INIT_FAILED = -2,
	MXCHIP_SYS_ILLEGAL = -3,
    MXCHIP_WIFI_JOIN_FAILED = -4,

	MXCHIP_WIFI_UP = 1,
	MXCHIP_WIFI_DOWN,
	

    MXCHIP_UAP_UP,
    MXCHIP_UAP_DOWN,
} 	MxchipStatus;

typedef enum
{
    IRQ_RISING_EDGE  = 0x1, /* Interrupt triggered at input signal's rising edge  */
    IRQ_FALLING_EDGE = 0x2, /* Interrupt triggered at input signal's falling edge */
    IRQ_BOTH_EDGES   = IRQ_RISING_EDGE | IRQ_FALLING_EDGE,
} gpio_irq_trigger_t;

#pragma pack(1)
struct ieee80211_frame
{
    u8 type;
    u8 flags;
    u16 duration;
    u8 address1[6];
    u8 address2[6];
    u8 address3[6];
    u16 ether_type;
    u8 data[1];
};
#pragma pack()

typedef struct
{
    u16     offset;     /**< Offset in bytes to start filtering (referenced to the start of the ethernet packet) */
    u16     mask_size;  /**< Size of the mask in bytes */
    u8*     mask;       /**< Pattern mask bytes to be ANDed with the pattern eg. "\xff00" (must be in network byte order) */
    u8*     pattern;    /**< Pattern bytes used to filter eg. "\x0800"  (must be in network byte order) */
} packet_filter_pattern_t;

/**
 * Enumeration of packet filter rules
 */
typedef enum
{
    POSITIVE_MATCHING  = 0, /**< Specifies that a filter should match a given pattern     */
    NEGATIVE_MATCHING  = 1  /**< Specifies that a filter should NOT match a given pattern */
} packet_filter_rule_e;

typedef int (*sniffer_call_back)(unsigned char * buf, int length, struct ieee80211_frame *wh);

typedef void (*gpio_irq_handler_t)( void* arg );
int gpio_irq_enable ( GPIO_TypeDef* gpio_port, uint8_t gpio_pin_number, gpio_irq_trigger_t trigger, gpio_irq_handler_t handler, void* arg );
int gpio_irq_disable( GPIO_TypeDef* gpio_port, uint8_t gpio_pin_number );

/* Upgrade iamge should save this table to flash */
typedef struct _boot_table_t {
	u32 start_address; // the address of the bin saved on flash.
	u32 length; // file real length
	u8 version[8];
	u8 type; // B:bootloader, P:boot_table, A:application, D: 8782 driver
	u8 upgrade_type; //u:upgrade, 
	u8 reserved[6];
}boot_table_t;

typedef struct _lib_config_t {
    int tcp_buf_dynamic; // TCP socket buffer: 1= dynamic mode, 0=static mode.
    int tcp_max_connection_num; // TCP max connection number, not bigger than 12.
    int tcp_rx_size; // the default TCP read buffer size.
    int tcp_tx_size; // the default TCP write buffer size.

    int hw_watchdog;

    int wifi_channel; // 0:USA(1~11), 1:China(1~13), 2:JP(1~14)
    int nfc_enable; // 1=enable nfc, 0=disable. default disable.
}lib_config_t;

enum {
	SLEEP_UNIT_MS = 0,
	SLEEP_UNIT_BEACON = 1,
};

// upgraded image should saved in here
#define NEW_IMAGE_ADDR 0x08060000

#define FD_UART         1
#define FD_USER_BEGIN   2
#define FD_SETSIZE      1024 // MAX 1024 fd
typedef unsigned long   fd_mask;

#define NBBY    8               /* number of bits in a byte */
#define NFDBITS (sizeof(fd_mask) * NBBY)        /* bits per mask */


#define howmany(x, y)   (((x) + ((y) - 1)) / (y))

typedef struct fd_set {
        fd_mask   fds_bits[howmany(FD_SETSIZE, NFDBITS)];
} fd_set;

#define _fdset_mask(n)   ((fd_mask)1 << ((n) % NFDBITS))
#define FD_SET(n, p)     ((p)->fds_bits[(n)/NFDBITS] |= _fdset_mask(n))
#define FD_CLR(n, p)     ((p)->fds_bits[(n)/NFDBITS] &= ~_fdset_mask(n))
#define FD_ISSET(n, p)  ((p)->fds_bits[(n)/NFDBITS] & _fdset_mask(n))
#define FD_ZERO(p)      memset(p, 0, sizeof(*(p)))

#define MSG_DONTWAIT    0x40    /* Nonblocking io  */

extern int lib_config(lib_config_t* conf);
extern int setSslMaxlen(int len);
extern int getSslMaxlen(void);
extern void set_cert(const char *cert_pem, const char*private_key_pem);

extern int SelectSupport(void);

extern int socket(int domain, int type, int protocol);
extern int setsockopt(int sockfd, int level, int optname,const void *optval, socklen_t optlen);
extern int bind(int sockfd, const struct sockaddr_t *addr, socklen_t addrlen);
extern int connect(int sockfd, const struct sockaddr_t *addr, socklen_t addrlen);
extern int listen(int sockfd, int backlog);
extern int accept(int sockfd, struct sockaddr_t *addr, socklen_t *addrlen);
extern int select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval_t *timeout);
extern ssize_t send(int sockfd, const void *buf, size_t len, int flags);
extern ssize_t  sendto(int  sockfd,  const  void  *buf,  size_t  len,  int  flags,const  struct  sockaddr_t  *dest_addr, 
				socklen_t addrlen);
extern ssize_t recv(int sockfd, void *buf, size_t len, int flags);
extern ssize_t recvfrom(int  sockfd,  void  *buf,  size_t  len,  int  flags,struct  sockaddr_t  *src_addr,  socklen_t 
					*addrlen);
extern int read(int sockfd, void *buf, size_t len); 
extern int write(int sockfd, void *buf, size_t len); 
extern int close(int fd);

extern int GetUartPara (char *uart_name, uart_get_str * uart_para); 
extern int GetUartNum(char *uartname); 
extern int SetUartPara (uart_set_str *puartpara); 
extern int OpenUART(char*  uart_name); 

extern MxchipStatus mxchipInit(void);
extern int StartNetwork(network_InitTypeDef_st* pNetworkInitPara);
extern int StartAdvNetwork(network_InitTypeDef_adv_st* mxconfig);
extern int wifi_get_pmk( char* psk, uint8_t psk_length, char* pmk );
extern void connected_ap_info(apinfo_adv_t *ap_info, char *key, int key_len);

extern void mxchipTick(void);

extern MxchipStatus mxchipStartScan(void);
extern int StartScan(int interval);
extern int StopScan(void);
extern int CheckNetLink(sta_ap_state_t *ap_state);
int getNetPara(net_para_st * pnetpara, int iface);
int ReallocIP(void);
int gethostbyname(const u8 * name, u8 * ip_addr, u8 ipLength);

extern int SetTimer(unsigned long ms, void (*psysTimerHandler)(void));
extern int SetTimer_uniq(unsigned long ms, void (*psysTimerHandler)(void));
extern int UnSetTimer(void (*psysTimerHandler)(void));

/* User provide watch dog callback function */
extern void WatchDog(void);

extern int FlashGetInfo(int *flashadd,int len);
extern int FlashRead(int flashadd,char *pbuf,int len);
extern int FlashWrite(int flashadd,char *pbuf,int len) ;
extern int FlashErase(int flashadd, int erase_bytelen); 
extern MxchipStatus newimage_write(int offset , int len , char *pbuf);

extern int sleep(int seconds);
extern int msleep(int mseconds);

extern u16 ntohs(u16 n);
extern u16 htons(u16 n);
extern u32 ntohl(u32 n);
extern u32 htonl(u32 n);

/* Convert an ip address string in dotted decimal format to  its binary representation.*/
extern u32 inet_addr(char *s);

/* Convert a binary ip address to its dotted decimal format. 
PARAMETER1 's':  location to place the dotted decimal string.  This must be an array of at least 16 bytes.
PARAMETER2 'x':  ip address to convert.

RETURN VALUE:  returns 's' .
*/
extern char *inet_ntoa( char *s, u32 x );

extern void enable_ps_mode(int unit_type, int unitcast_ms, int multicast_ms);

extern void disable_ps_mode(void);

extern void system_reload(void);

/* This is function is used to caclute the md5 value for the input buffer
  * input: the source buffer;
  * len: the length of the source buffer;
  * output: point the output buffer, should be a 16 bytes buffer
  */
extern void md5_hex(u8 *input, u32 len, u8 *output);

int wlan_disconnect(void);
int sta_disconnect(void);
int uap_stop(void);


int OpenConfigmodeWPS(int timeout);
int CloseConfigmodeWPS(void);

int OpenEasylink(int timeout);
int CloseEasylink(void);


int OpenBT(int timeout);
int CloseBT(void);
int SetBTName(char *name);

int SetBTpin(char *pin, int enable);

int SetBTboundleID(char*boundleID);
int SetBTseedID(char* seedID);
int SetBTsdkProto(char*sdkproto);

int GetBTArg(char *name, char *pin, int *enable_pin, char *boundleID);
int SetBT_SearchAppPara(char *boundleID, char *seedId, char *SdkProtocolToken);

/* devlist return mac[4][6], return 4 mac address, last_dev_index is the last connected dev index (0~3)*/
int GetBTDevlist(char *devlist, u8 *last_dev_index);

int CheckComponentBT(COMPONENT_BTINFO_st *pst);


int CheckComponentWIFI(COMPONENT_WIFIINFO_st *pst);

int CheckComponentNFC(COMPONENT_NFCINFO_st * st);
int OpenConfigmodeNFC(int timeout);
int CloseConfigmodeNFC(void);


void ApListCallback(UwtPara_str *pApList);

time_t user_time(TIMEVAL_st *t);

void ps_enable(void); 
void ps_disable(void); 

int rand(void); 

void set_tcp_keepalive(int num, int seconds);
void get_tcp_keepalive(int *num, int *seconds);
int get_tcp_clients(void);
void memory_status(int *total_free, int *max_len);
void get_malloc_list(void (*debug_mem)(u32 p, int len));
void malloc_list_mem(u8 *memory_tbl, int size);

int wifi_power_down(void);

int wifi_power_up(void);

int wifi_disconnect(void);

/* -99 ~ -40 dbm*/
int wifi_roam_trigger(int dbm);

// user provide callback function 
void WatchDog(void);
void WifiStatusHandler(int status);
void RptConfigmodeRslt(network_InitTypeDef_st *nwkpara);
void NetCallback(net_para_st *pnet);
void ApListCallback(UwtPara_str *pApList);
void join_fail(int type);

u32 dns_request(char *hostname);// start a DNS request. return 0=start dns req, 0xffffffff=fail, other=the IP address of this DNS
void dns_ip_set(u8 *name, u32 ip);// user callback function to DNS, return the request DNS's IP address.

int tx_buf_size(int sockfd);// return the tx buffer size


void wlan_sniffer_start(int channel, int rssi, sniffer_call_back pfunc);
void wlan_sniffer_stop(void);
int ath_set_channel(int chann);

int wlan_add_packet_filter(u8 filter_id, packet_filter_pattern_t *patt, packet_filter_rule_e rule);
int wlan_enable_packet_filter(u8 filter_id);
int wlan_disable_pakcet_filter(u8 filter_id);
int wlan_remove_packet_filter(u8 filter_id);

int host_platform_rand( void *inBuffer, int inByteCount );

void NoOS_systick_irq(void); 
void gpio_irq(void); 
void sdio_irq(void); 
void dma_irq(void); 
void bt_uart_irq(void);


int OpenEasylink2(int timeout);
int OpenEasylink2_withdata(int timeout);

int CloseEasylink2(void);

/* return the RF Tx Power, 0~31*/
u8 wlan_get_tx_power(void);
/* Set Tx Power, dBm can be 0~31 and 0xFF, 0xFF use default tx power. */
int wlan_set_tx_power(u8 dBm);

/* Set SoftAP work channel, this API must be called before StartNetwork to start SoftAP. */
void wlan_set_channel(int channel);
int wlan_driver_version( char* version, u8 length );

void wifi_reboot_event(void);// health monitor callback, system unhealth, user should call system_reload to fix it.
int wlan_set_max_station_num(int number);


#endif

