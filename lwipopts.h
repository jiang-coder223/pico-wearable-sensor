#ifndef LWIPOPTS_H
#define LWIPOPTS_H

#define NO_SYS 1
#define LWIP_SOCKET 0
#define LWIP_NETCONN 0

#define MEM_ALIGNMENT 4

// 🔥 核心：加大 heap
#define MEM_SIZE (20 * 1024)

// TCP
#define TCP_MSS 1460
#define TCP_WND (4 * TCP_MSS)
#define TCP_SND_BUF (4 * TCP_MSS)

// 網路功能
#define LWIP_DHCP 1
#define LWIP_TCP 1
#define LWIP_UDP 1
#define LWIP_ARP 1
#define LWIP_ICMP 1

#define LWIP_NETIF_STATUS_CALLBACK 1

// 🔥 關鍵（解決你 crash）
#define MEMP_NUM_SYS_TIMEOUT 20

// 🔥 建議一起加（避免下一個 crash）
#define MEMP_NUM_TCP_SEG 32
#define MEMP_NUM_TCP_PCB 5

#endif