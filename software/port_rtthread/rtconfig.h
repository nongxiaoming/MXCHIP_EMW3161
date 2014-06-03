/* RT-Thread config file */
#ifndef __RTTHREAD_CFG_H__
#define __RTTHREAD_CFG_H__

/* 最大对象名称长度 */
#define RT_NAME_MAX	8
/* 默认对齐方式 */
#define RT_ALIGN_SIZE	4
/* 最大支持优先级数 */
#define RT_THREAD_PRIORITY_MAX	32
/* 每秒的tick个数 */
#define RT_TICK_PER_SECOND	1000
/* 空闲任务的栈大小 */
#define IDLE_THREAD_STACK_SIZE	512
/* 是否使用内核调试 */
#define RT_DEBUG
/* 是否使用栈溢出检查 */
#define RT_USING_OVERFLOW_CHECK
/* 是否使用钩子函数 */
#define RT_USING_HOOK
/* 是否使用软件timer */
#define RT_USING_TIMER_SOFT
/* timer线程优先级 */
#define RT_TIMER_THREAD_PRIO	4
/* timer线程栈大小 */
#define RT_TIMER_THREAD_STACK_SIZE	512
/* timer每秒的tick个数 */
#define RT_TIMER_TICK_PER_SECOND	100

/* 是否使用信号量 */
#define RT_USING_SEMAPHORE
/* 是否使用互斥量 */
#define RT_USING_MUTEX
/* 是否使用事件 */
#define RT_USING_EVENT
/* 是否使用邮箱 */
#define RT_USING_MAILBOX
/* 是否使用消息队列 */
#define RT_USING_MESSAGEQUEUE

/* 是否使用内存池 */
#define RT_USING_MEMPOOL
/* 是否使用内存管理 */
#define RT_USING_HEAP
/* 是否使用动态内存管理 */
#define RT_USING_MEMHEAP
/* 是否使用优化的小型内存管理 */
#define RT_USING_SMALL_MEM

/* 是否使用设备框架 */
#define RT_USING_DEVICE
/* 是否使用IPC设备框架 */
#define RT_USING_DEVICE_IPC
/* 是否使用串口设备框架 */
#define RT_USING_SERIAL
/* 串口设备接收缓冲区大小 */
#define RT_UART_RX_BUFFER_SIZE    64
/* 是否使用SPI设备框架 */
#define RT_USING_SPI
/* 是否使用I2C设备框架 */
#define RT_USING_I2C
/* 是否使用RTC框架 */
//#define RT_USING_RTC
/* 是否使用输出模块 */
#define RT_USING_CONSOLE
/* 输出缓冲区大小 */
#define RT_CONSOLEBUF_SIZE	128
/* 用于输出的设备名称 */
#define RT_CONSOLE_DEVICE_NAME	"uart2"

/* 是否使用finsh调试组件 */
#define RT_USING_FINSH
/* 是否使用finsh命令表 */
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
/* finsh任务栈大小 */
#define FINSH_THREAD_STACK_SIZE	4096
/* 是否使用MSH模式 */
//#define FINSH_USING_MSH

/* 是否使用arm 标准C库 */
#define RT_USING_ARM_LIBC

/* 是否使用DFS文件系统 */
//#define RT_USING_DFS
/* 是否使用工作目录模式 */
#define DFS_USING_WORKDIR
// <integer name="DFS_FILESYSTEM_TYPES_MAX" description="The maximal number of the supported file system type" default="4" />
#define DFS_FILESYSTEM_TYPES_MAX  4
// <integer name="DFS_FILESYSTEMS_MAX" description="The maximal number of mounted file system" default="4" />
#define DFS_FILESYSTEMS_MAX	4
// <integer name="DFS_FD_MAX" description="The maximal number of opened files" default="4" />
#define DFS_FD_MAX	         16
// <bool name="RT_USING_DFS_ELMFAT" description="Using ELM FatFs" default="true" />
#define RT_USING_DFS_ELMFAT
// <integer name="RT_DFS_ELM_DRIVES" description="The maximal number of drives of FatFs" default="4" />
#define RT_DFS_ELM_DRIVES    4
// <bool name="RT_DFS_ELM_REENTRANT" description="Support reentrant" default="true" />
#define RT_DFS_ELM_REENTRANT
// <integer name="RT_DFS_ELM_USE_LFN" description="Support long file name" default="0">
// <item description="LFN with static LFN working buffer">1</item>
// <item description="LFN with dynamic LFN working buffer on the stack">2</item>
// <item description="LFN with dynamic LFN working buffer on the heap">3</item>
// </integer>
#define RT_DFS_ELM_USE_LFN	3
// <integer name="RT_DFS_ELM_CODE_PAGE" description="OEM code page" default="936">
#define RT_DFS_ELM_CODE_PAGE	936
// <bool name="RT_DFS_ELM_CODE_PAGE_FILE" description="Using OEM code page file" default="false" />
#define RT_DFS_ELM_CODE_PAGE_FILE
// <integer name="RT_DFS_ELM_MAX_LFN" description="Maximal size of file name length" default="256" />
#define RT_DFS_ELM_MAX_LFN	256
// <integer name="RT_DFS_ELM_MAX_SECTOR_SIZE" description="Maximal size of sector" default="512" />
#define RT_DFS_ELM_MAX_SECTOR_SIZE  4096
// <bool name="RT_DFS_ELM_USE_ERASE" description="Enable erase feature for flash" default="true" />
#define RT_DFS_ELM_USE_ERASE
// <bool name="RT_USING_DFS_YAFFS2" description="Using YAFFS2" default="false" />
// #define RT_USING_DFS_YAFFS2
// <bool name="RT_USING_DFS_UFFS" description="Using UFFS" default="false" />
// #define RT_USING_DFS_UFFS
// <bool name="RT_USING_DFS_DEVFS" description="Using devfs for device objects" default="true" />
#define RT_USING_DFS_DEVFS

/* 是否使用NFS */
#define RT_USING_DFS_NFS
/* NFS端口信息 */
#define RT_NFS_HOST_EXPORT  "192.168.1.20:/"
// </section>

// </RDTConfigurator>


#endif
