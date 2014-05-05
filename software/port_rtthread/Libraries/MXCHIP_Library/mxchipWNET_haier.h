#ifndef __FILE_H__
#define __FILE_H__
#include "stdlib.h"
#include "string.h"

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

typedef  struct  _haieruhome_timeval_st{ 
    long tv_sec; /*1970年1月1日至今的秒数*/ 
    long tv_hmsec; /* 1970年1月1日至今的百豪秒数*/ 
}HAIERUHOME_TIMEVAL_st; 

#define NULL 0

typedef int socklen_t;

typedef enum {
	SO_REUSEADDR = 2,         /* Socket always support this option */
	SO_BROADCAST = 6,		/* Socket always support this option */
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
    char wifi_key[32]; 
    char local_ip_addr[16]; 
    char net_mask[16]; 
    char gateway_ip_addr[16]; 
    char dnsServer_ip_addr[16]; 
    char dhcpMode;       // disable(0), client mode(1), server mode(2) 
    char address_pool_start[16]; 
    char address_pool_end[16]; 
    int wifi_retry_interval;//sta reconnect interval, ms
} network_InitTypeDef_st; 


struct wifi_InitTypeDef
{
	u8 wifi_mode;		// adhoc mode(1), AP client mode(0), AP mode(2)
	u8 wifi_ssid[32];
	u8 wifi_key[32];
};

typedef enum {
	MXCHIP_SUCCESS = 0,
	MXCHIP_FAILED = -1,
	MXCHIP_8782_INIT_FAILED = -2,
	MXCHIP_SYS_ILLEGAL = -3,
	MXCHIP_WIFI_JOIN_FAILED = -4,

	MXCHIP_WIFI_UP = 1,
	MXCHIP_WIFI_DOWN,
} 	MxchipStatus;

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
}lib_config_t;

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

typedef int (*sniffer_call_back)(unsigned char * buf, int length, struct ieee80211_frame *wh);

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

extern int haieruhome_setSslMaxlen(int len);
extern int haieruhome_getSslMaxlen(void);
extern void haieruhome_set_cert(const char *cert_pem, const char*private_key_pem);

extern int haieruhome_SelectSupport(void);

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

int haieruhome_bind(int sockfd, const struct sockaddr_t *addr, socklen_t addrlen); 
int haieruhome_listen(int sockfd, int backlog); 
int haieruhome_select(int nfds, fd_set *readfds, fd_set *writefds,fd_set *exceptfds, struct timeval_t *timeout); 
void haieruhome_FD_CLR(int fd, fd_set *set); 
int  haieruhome_FD_ISSET(int fd, fd_set *set); 
void haieruhome_FD_SET(int fd, fd_set *set); 
void haieruhome_FD_ZERO(fd_set *set); 
int haieruhome_accept(int sockfd, struct sockaddr_t *addr, int *addrlen); 
int haieruhome_close(int fd); 
int haieruhome_htonl(int hostlong);  
u16 haieruhome_htons( u16 hostshort);  
int haieruhome_ntohl(int netlong);  
u16 haieruhome_ntohs(u16 netshort);  
in_addr_t haieruhome_inet_addr(const char* strptr);  
int haieruhome_read(int handle, void *buf, int nbyte);  
int haieruhome_write(int handle, void *buf, int nbyte);  
ssize_t haieruhome_send(int sockfd, const void *buf, size_t len, int flags); 
ssize_t haieruhome_sendto(int sockfd, const void *buf, size_t len, int flags,const struct sockaddr_t *dest_addr, socklen_t addrlen); 
ssize_t haieruhome_recv(int sockfd, void *buf, size_t len, int flags); 
ssize_t haieruhome_recvfrom(int sockfd, void *buf, size_t len, int flags,struct sockaddr_t *src_addr, socklen_t *addrlen); 
int haieruhome_socket(int domain, int type, int protocol); 
int haieruhome_setsockopt(int sockfd, int level, int optname,const void *optval, socklen_t optlen); 
int haieruhome_connect(int sockfd, const struct sockaddr_t *addr,socklen_t addrlen); 

extern int haieruhome_GetUartPara (char *uart_name, uart_get_str * uart_para); 
extern int haieruhome_GetUartNum(char *uartname); 
extern int haieruhome_SetUartPara (uart_set_str *puartpara); 
extern int haieruhome_OpenUART(char*  uart_name); 

extern MxchipStatus mxchipInit(void);
extern int haieruhome_StartNetwork(network_InitTypeDef_st* pNetworkInitPara);

extern void mxchipTick(void);

extern int haieruhome_StartScan(int interval);
extern int haieruhome_StopScan(void);
extern int haieruhome_CheckNetLink(int *ap_connect,int *wifi_strength);
int haieruhome_getNetPara(net_para_st * pnetpara);
int haieruhome_ReallocIP(void);
int haieruhome_gethostbyname(const u8 * name, u8 * ip_addr, u8 ipLength);

extern int haieruhome_SetTimer(unsigned long ms, void (*phaieruhome_sysTimerHandler)(void));

/* User provide watch dog callback function */
extern void haieruhome_WatchDog(void);

extern int haieruhome_FlashGetInfo(int *flashadd,int len);
extern int haieruhome_FlashRead(int flashadd,char *pbuf,int len);
extern int haieruhome_FlashWrite(int flashadd,char *pbuf,int len) ;
extern int haieruhome_FlashErase(int flashadd, int erase_bytelen); 
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

extern void haieruhome_system_reload(void);

/* This is function is used to caclute the md5 value for the input buffer
  * input: the source buffer;
  * len: the length of the source buffer;
  * output: point the output buffer, should be a 16 bytes buffer
  */
extern void md5_hex(u8 *input, u32 len, u8 *output);

/* This is function is used to caclute the md5 value for the input buffer and format the md5 value as string.
  * input: the source buffer;
  * len: the length of the source buffer;
  * output: point the output buffer, should be a 33 bytes buffer
  */
void haieruhome_md5(char  *input, int len, char *output);


int haieruhome_OpenConfigmodeWPS(int timeout);
int haieruhome_CloseConfigmodeWPS(void);

int haieruhome_OpenEasylink(int timeout);
int haieruhome_CloseEasylink(void);


int haieruhome_OpenBT(int timeout);
int haieruhome_CloseBT(void);
int haieruhome_SetBTName(char *name);

int haieruhome_SetBTpin(char *pin, int enable);

int haieruhome_SetBTboundleID(char*boundleID);
int haieruhome_SetBTseedID(char* seedID);
int haieruhome_SetBTsdkProto(char*sdkproto);

int haieruhome_GetBTArg(char *name, char *pin, int *enable_pin, char *boundleID);
int haieruhome_SetBT_SearchAppPara(char *boundleID, char *seedId, char *SdkProtocolToken);

/* devlist return mac[4][6], return 4 mac address, last_dev_index is the last connected dev index (0~3)*/
int haieruhome_GetBTDevlist(char *devlist, u8 *last_dev_index);

int haieruhome_CheckComponentBT(COMPONENT_BTINFO_st *pst);


int haieruhome_CheckComponentWIFI(COMPONENT_WIFIINFO_st *pst);

int haieruhome_CheckComponentNFC(COMPONENT_NFCINFO_st * st);
int haieruhome_OpenConfigmodeNFC(int timeout);
int haieruhome_CloseConfigmodeNFC(void);

void haieruhome_RptConfigmodeRslt(network_InitTypeDef_st *nwkpara);

int haieruhome_ChgWIFIWorkMode(char flg);
void haieruhome_ApListCallback(UwtPara_str *pApList);

time_t haieruhome_time(HAIERUHOME_TIMEVAL_st *t);
int haieruhome_sleep(int seconds); 
int haieruhome_msleep(int mseconds); 

void haieruhome_ps_enable(void); 
void haieruhome_ps_disable(void); 

int haieruhome_rand(void); 
void haieruhome_srand (unsigned int seed); 
char *haieruhome_inet_ntoa(struct in_addr in);

void haieruhome_set_tcp_keepalive(int num, int seconds);
void haieruhome_get_tcp_keepalive(int *num, int *seconds);
int haieruhome_get_tcp_clients(void);
void haieruhome_memory_status(int *total_free, int *max_len);
void haieruhome_get_malloc_list(void (*debug_mem)(u32 p, int len));
void haieruhome_malloc_list_mem(u8 *memory_tbl, int size);

void wlan_sniffer_start(int channel, int rssi, sniffer_call_back pfunc);
void wlan_sniffer_stop();
int ath_set_channel(int chann);

int wlan_add_packet_filter(u8 filter_id, packet_filter_pattern_t *patt, packet_filter_rule_e rule);
int wlan_enable_packet_filter(u8 filter_id);
int wlan_disable_pakcet_filter(u8 filter_id);
int wlan_remove_packet_filter(u8 filter_id);

#endif

