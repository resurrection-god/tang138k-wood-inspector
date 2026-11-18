/*
 *********************************************************************************************
 * @file      lwipopts.h
 * @author    GowinSemicoductor
 * @device    Gowin_EMPU_M1
 * @brief     LwIP options.
 *********************************************************************************************
 */

#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

/**
 * SYS_LIGHTWEIGHT_PROT==1: if you want inter-task protection for certain
 * critical regions during buffer allocation, deallocation and memory
 * allocation and deallocation.
 */
#define SYS_LIGHTWEIGHT_PROT    1

/**
 * NO_SYS==1: Provides VERY minimal functionality. Otherwise,
 * use lwIP facilities.
 */
#define NO_SYS                  0

#define LWIP_CHKSUM_ALGORITHM   3

/**
 * NO_SYS_NO_TIMERS==1: Drop support for sys_timeout when NO_SYS==1
 * Mainly for compatibility to old versions.
 */
#define NO_SYS_NO_TIMERS        0

/* ---------- Memory options ---------- */
/* MEM_ALIGNMENT: should be set to the alignment of the CPU for which
   lwIP is compiled. 4 byte alignment -> define MEM_ALIGNMENT to 4, 2
   byte alignment -> define MEM_ALIGNMENT to 2. */
#define MEM_ALIGNMENT           4
#define LWIP_DEBUG_TIMERNAMES   0
#define ETH_PAD_SIZE            0
#define LWIP_DECLARE_MEMORY_ALIGNED(variable_name, size) u8_t variable_name[LWIP_MEM_ALIGN_BUFFER(size)] __attribute__((aligned(4)))

/* MEM_SIZE: lwIP heap size.
 *
 * There is no "must fit in 16 bits" limit here: mem.h itself switches
 * mem_size_t to u32_t when MEM_SIZE > 64000 (include/lwip/mem.h:63). The former
 * comment about <64 KB refers to another lwIP build and was misleading.
 *
 * It was 48 KB, and the heap turned out to be the bottleneck: sending copies
 * into PBUF_RAM from this heap, so a long transfer ran into mem_malloc failures
 * and broke the segment lists when the connection closed. Measured on sending
 * from one DDR3 address in 32 KB pieces:
 *
 *     48 KB heap, TCP_SND_BUF 40*MSS : crash after  203 KB
 *     48 KB heap, TCP_SND_BUF 12*MSS : crash after  864 KB
 *
 * The heap lives in DDR3 (see bsp/sag/ae350-ddr.sag: RAM 0x00000000), which has
 * 128 MB, so half a megabyte for it costs nothing. */
#define MEM_SIZE                (512*1024)
#define MEMP_MEM_MALLOC         0

/* MEMP_NUM_PBUF: the number of memp struct pbufs. If the application
   sends a lot of data out of ROM (or other static memory), this
   should be set high. */
/* 05.10.2026: 128 -> 512. Zero-copy frame delivery (lwip_vmsp_write3) holds
   two ROM pbufs (header and row) per chunk in flight, up to ~230 in an
   88*MSS buffer (ERRORS 3.9). */
#define MEMP_NUM_PBUF           512
#define MEMP_NUM_UDP_PCB        16
#define MEMP_NUM_TCP_PCB        32
#define MEMP_NUM_TCP_PCB_LISTEN 8
#define MEMP_NUM_TCP_SEG        512
#define MEMP_NUM_SYS_TIMEOUT    64
#define MEMP_NUM_NETBUF         128
#define MEMP_NUM_NETCONN        64
#define MEMP_NUM_TCPIP_MSG_API  128
#define MEMP_NUM_TCPIP_MSG_INPKT 256
#define MEMP_NUM_ARP_QUEUE      16

#define LWIP_ETHERNET           1
#define LWIP_ARP                1
#define ARP_TABLE_SIZE          10
#define ARP_QUEUEING            0
#define ETHARP_TRUST_IP_MAC     1


/* ---------- Pbuf options ---------- */
/* PBUF_POOL_SIZE: the number of buffers in the pbuf pool. */
#define PBUF_POOL_SIZE          128
/* PBUF_POOL_BUFSIZE: the size of each pbuf in the pbuf pool. */
#define PBUF_POOL_BUFSIZE       LWIP_MEM_ALIGN_SIZE(TCP_MSS+40+PBUF_LINK_ENCAPSULATION_HLEN+PBUF_LINK_HLEN)


/* ---------- TCP options ---------- */
#define LWIP_TCP                1
#define TCP_TTL                 255
#define TCP_MSL                 1000UL /* 2s TIME_WAIT for short local VMSP reconnects */

/* Controls if TCP should queue segments that arrive out of
   order. Define to 0 if your device is low on memory. */
#define TCP_QUEUE_OOSEQ         0

/* TCP Maximum segment size. */
#define TCP_MSS                 (1500 - 40)	  /* TCP_MSS = (Ethernet MTU - IP header size - TCP header size) */

/* TCP sender buffer space (bytes).
 *
 * It was 40*TCP_MSS = 58400 with MEM_SIZE = 48 KB: the stack allowed queuing
 * more bytes for sending than fit in its own heap. Sending copies, i.e. into
 * PBUF_RAM from this very heap, so the queue ran into mem_malloc failures, and
 * tearing down the pcb on connection close walked a corrupted segment list:
 * mcause=4, mtval=0xFFFFFFFF in tcp_output (tcp_out.c:1329), tcp_send_fin
 * (tcp_out.c:1008) and tcp_segs_free (tcp.c:1611).
 *
 * Measured: the board died after ~100...250 KB sent over several connections,
 * and from the same DDR3 address, so the cause was the transmit path, not the
 * memory range and not the Lane 4 writes.
 *
 * 12*TCP_MSS = 17520 leaves the heap almost a threefold margin. On a gigabit
 * link with a round trip under a millisecond that is more than enough:
 * 256 KB went out in 0.26 s.
 *
 * 17.09.2026 (ERRORS former 31.44, 3.4) 44*TCP_MSS = 64240 was tried and does
 * not help: a 1016x1024 frame delivery is the same, 0.254-0.261 s in the fast
 * runs (4.3 MB/s) against 0.27-0.28 s at 12*MSS, and the same slow runs +0.3 s
 * every other time. Returned to the proven 12*MSS.
 *
 * 30.09.2026 experiment C: 32*TCP_MSS = 46720. Reason to repeat it: the
 * 17.09 experiment ran on a driver that disabled receive for the whole of each
 * transmission (ethernetif.c before 30.09), so TCP acknowledgements could not
 * come back earlier. With receive open the result is the same: blocking sends
 * 81 -> 15 per frame, but frame sending 158 -> 172 ms, the whole frame
 * 263 -> 283 ms, lost discovery per 100 cycles 1 -> 3. The window is not the
 * bottleneck. Returned to 12*MSS.
 *
 * 01.10.2026 (ERRORS 1.16): for the profile stream the window is the
 * bottleneck. Two cameras at 450 Hz are 3.9 MB/s of steady stream, and the
 * FPGA ring holds 71 ms. With 12*MSS there are 4 profiles in flight, the task
 * sits in send waiting for the PC's acknowledgements, falls a ring revolution
 * behind and loses 32 profiles at a time. 44*MSS gives 0.2..0.36 % losses over
 * 60 s: the PC window without scaling does not allow more than 64 KB. 88*MSS
 * with window scaling gives 0 losses. The price is frame delivery
 * 263 -> 288 ms (as in experiment C). */
#define TCP_SND_BUF             (88*TCP_MSS)
/* Window scaling: without it the PC window is at most 64 KB, and so is the data
 * in flight. TCP_RCV_SCALE 0: our own receive window is unchanged, only the PC
 * window can be large. */
#define LWIP_WND_SCALE          1
#define TCP_RCV_SCALE           0
/* The default is SND_BUF/2, above lwIP's check limit (65535 - 4*MSS). */
#define TCP_SNDLOWAT            (32*TCP_MSS)

/*  TCP_SND_QUEUELEN: TCP sender buffer space (pbufs). This must be at least
  as much as (2 * TCP_SND_BUF/TCP_MSS) for things to work. */

#define TCP_SND_QUEUELEN        (4* TCP_SND_BUF/TCP_MSS)   /* <= MEMP_NUM_TCP_SEG (512) */

/* TCP receive window. */
#define TCP_WND                 (40*TCP_MSS)

/* TCP timer 50 ms instead of 250 (01.10.2026, ERRORS 1.16): the retransmit tick
 * is 100 ms instead of 500. With RTT < 1 ms lwIP's RTO is 3 ticks: losing the
 * tail of a batch used to stall sending for 1.5 s (20 FPGA ring revolutions),
 * now about 0.3 s. 10 ms since 22.10.2026 (ERRORS 3.20): tick 20 ms, a stall ~60-120 ms,
 * 2 x 1000 Hz on the office network 646-667 -> 810-847 profiles/s per camera. */
#define TCP_TMR_INTERVAL        10


/* ---------- ICMP options ---------- */
#define LWIP_ICMP                       1


/* ---------- DHCP options ---------- */
/* DHCP and link-local addressing (GigE Vision IP configuration, net_config.c, since
   23.10.2026): compiled in, started only when the GEV Persistent IP flag is off; the
   default boot keeps the stored address and never starts them. LLA follows a DHCP that
   got no lease after LWIP_DHCP_AUTOIP_COOP_TRIES discovers (GEV fig. 4-1). */
#define LWIP_DHCP               1
#define LWIP_AUTOIP             1
#define LWIP_DHCP_AUTOIP_COOP   1
#define LWIP_DHCP_AUTOIP_COOP_TRIES 3


/* ---------- UDP options ---------- */
#define LWIP_UDP                1
#define UDP_TTL                 255


/* ---------- Statistics options ---------- */
/* 01.10.2026: enabled for the profile stream (0x0212, ERRORS 1.15): the
 * tcp.drop and link.* counters at the tail of 0x0215. No printing
 * (LWIP_STATS_DISPLAY 0). Checked: WD cmp 0 over 40 s idle (B43AD15A +
 * 7EE09AB6). The WD cmp growth in the first run came from unrelated load on
 * the board, not from the counters. */
#define LWIP_STATS 1
#define LWIP_STATS_DISPLAY 0
#define LWIP_PROVIDE_ERRNO 1

/* ---------- link callback options ---------- */
/* LWIP_NETIF_LINK_CALLBACK==1: Support a callback function from an interface
 * whenever the link changes (i.e., link down)
 */
#define LWIP_NETIF_LINK_CALLBACK        0
/*
   --------------------------------------
   ---------- Checksum options ----------
   --------------------------------------
*/

/* 
The STM32F4x7 allows computing and verifying the IP, UDP, TCP and ICMP checksums by hardware:
 - To use this feature let the following define uncommented.
 - To disable it and process by CPU comment the  the checksum.
*/
//#define CHECKSUM_BY_HARDWARE 


#ifdef CHECKSUM_BY_HARDWARE
  /* CHECKSUM_GEN_IP==0: Generate checksums by hardware for outgoing IP packets.*/
  #define CHECKSUM_GEN_IP                 0
  /* CHECKSUM_GEN_UDP==0: Generate checksums by hardware for outgoing UDP packets.*/
  #define CHECKSUM_GEN_UDP                0
  /* CHECKSUM_GEN_TCP==0: Generate checksums by hardware for outgoing TCP packets.*/
  #define CHECKSUM_GEN_TCP                0 
  /* CHECKSUM_CHECK_IP==0: Check checksums by hardware for incoming IP packets.*/
  #define CHECKSUM_CHECK_IP               0
  /* CHECKSUM_CHECK_UDP==0: Check checksums by hardware for incoming UDP packets.*/
  #define CHECKSUM_CHECK_UDP              0
  /* CHECKSUM_CHECK_TCP==0: Check checksums by hardware for incoming TCP packets.*/
  #define CHECKSUM_CHECK_TCP              0
  /* CHECKSUM_CHECK_ICMP==0: Check checksums by hardware for incoming ICMP packets.*/
  #define CHECKSUM_GEN_ICMP               0
#else
  /* CHECKSUM_GEN_IP==1: Generate checksums in software for outgoing IP packets.*/
  #define CHECKSUM_GEN_IP                 1
  /* CHECKSUM_GEN_UDP==1: Generate checksums in software for outgoing UDP packets.*/
  #define CHECKSUM_GEN_UDP                1
  /* CHECKSUM_GEN_TCP==1: Generate checksums in software for outgoing TCP packets.*/
  #define CHECKSUM_GEN_TCP                1
  /* CHECKSUM_CHECK_IP==1: Check checksums in software for incoming IP packets.*/
  #define CHECKSUM_CHECK_IP               1
  /* CHECKSUM_CHECK_UDP==1: Check checksums in software for incoming UDP packets.*/
  #define CHECKSUM_CHECK_UDP              1
  /* CHECKSUM_CHECK_TCP==1: Check checksums in software for incoming TCP packets.*/
  #define CHECKSUM_CHECK_TCP              1
  /* CHECKSUM_CHECK_ICMP==1: Check checksums by hardware for incoming ICMP packets.*/
  #define CHECKSUM_GEN_ICMP               1
#endif


/*
   ----------------------------------------------
   ---------- Sequential layer options ----------
   ----------------------------------------------
*/
/**
 * LWIP_NETCONN==1: Enable Netconn API (require to use api_lib.c)
 */
#define LWIP_NETCONN                    1

/*
   ------------------------------------
   ---------- Socket options ----------
   ------------------------------------
*/
#define LWIP_SOCKET                     1
#define LWIP_SO_LINGER                   1
/* VMSP sockets (vmsp_service.c accept_loop): a send stalled for 10 s fails, and keepalive
 * drops a peer that vanished while the board waits in recv (ERRORS 3.17). Without these
 * the setsockopt calls failed silently and a lost PC held the port until a reboot. */
#define LWIP_SO_SNDTIMEO                1
#define LWIP_TCP_KEEPALIVE              1

/*
   ---------------------------------
   ---------- OS options ----------
   ---------------------------------
*/


#define DEFAULT_UDP_RECVMBOX_SIZE       64
#define DEFAULT_TCP_RECVMBOX_SIZE       8
#define DEFAULT_ACCEPTMBOX_SIZE         8
#define DEFAULT_THREAD_STACKSIZE        4096


#define TCPIP_THREAD_NAME              "lwip"
#define TCPIP_THREAD_STACKSIZE          4096
#define TCPIP_MBOX_SIZE                 32
#define TCPIP_THREAD_PRIO               4

//#define IPERF_SERVER_THREAD_NAME            "iperf_server"
//#define IPERF_SERVER_THREAD_STACKSIZE        1024
//#define IPERF_SERVER_THREAD_PRIO             0

//#define BLOCK_TIME			                250
//#define BLOCK_TIME_WAITING_FOR_INPUT	( ( portTickType ) 100 )

/*
   ----------------------------------------
   ---------- Lwip Debug options ----------
   ----------------------------------------
*/
//#define LWIP_DEBUG                      1

#endif /* __LWIPOPTS_H__ */
