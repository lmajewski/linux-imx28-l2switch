/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2025 DENX Software Engineering GmbH
 * Lukasz Majewski <lukma@denx.de>
 */

#ifndef __MTIP_L2SWITCH_H_
#define __MTIP_L2SWITCH_H_

#include <linux/clocksource.h>
#include <linux/net_tstamp.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/timecounter.h>

#define PKT_MAXBUF_SIZE         1518
#define PKT_MINBUF_SIZE         64
#define PKT_MAXBLR_SIZE         1536

/* The number of Tx and Rx buffers. These are allocated from the page
 * pool. The code may assume these are power of two, so it is best
 * to keep them that size.
 * We don't need to allocate pages for the transmitter.  We just use
 * the skbuffer directly.
 */
#define MTIP_SWITCH_RX_PAGES       8
#define MTIP_SWITCH_RX_FRSIZE      2048
#define MTIP_SWITCH_RX_FRPPG       (PAGE_SIZE / MTIP_SWITCH_RX_FRSIZE)
#define RX_RING_SIZE            (MTIP_SWITCH_RX_FRPPG * MTIP_SWITCH_RX_PAGES)
#define MTIP_SWITCH_TX_FRSIZE      2048
#define MTIP_SWITCH_TX_FRPPG       (PAGE_SIZE / MTIP_SWITCH_TX_FRSIZE)

#define TX_RING_SIZE            16      /* Must be power of two */
#define TX_RING_MOD_MASK        15      /*   for this to work */

#define SWITCH_EPORT_NUMBER	2
#define SWITCH_HOST_PORT_NUM	0

#if (((RX_RING_SIZE + TX_RING_SIZE) * 8) > PAGE_SIZE)
#error "L2SWITCH: descriptor ring size constants too large"
#endif

#define ESW_REVISION        (0x000)
#define ESW_SCRATCH         (0x004)
#define ESW_PER             (0x008)
#define ESW_VLANV           (0x010)
#define ESW_DBCR            (0x014)
#define ESW_DMCR            (0x018)
#define ESW_BKLR            (0x01C)
#define ESW_BMPC            (0x020)
#define ESW_MODE            (0x024)
#define ESW_VIMSEL          (0x028)
#define ESW_VOMSEL          (0x02C)
#define ESW_VIMEN           (0x030)
#define ESW_VID             (0x034)

#define ESW_MCR             (0x040)
#define ESW_EGMAP           (0x044)
#define ESW_INGMAP          (0x048)
#define ESW_INGSAL          (0x04C)
#define ESW_INGSAH          (0x050)
#define ESW_INGDAL          (0x054)
#define ESW_INGDAH          (0x058)
#define ESW_ENGSAL          (0x05C)
#define ESW_ENGSAH          (0x060)
#define ESW_ENGDAL          (0x064)
#define ESW_ENGDAH          (0x068)
#define ESW_MCVAL           (0x06C)

#define ESW_MMSR            (0x080)
#define ESW_LMT             (0x084)
#define ESW_LFC             (0x088)
#define ESW_PCSR            (0x08C)
#define ESW_IOSR            (0x090)
#define ESW_QWT             (0x094)

#define ESW_P0BCT           (0x09C)

#define ESW_P0FFEN          (0x0BC)

#define ESW_PSNP_BASE       (0x0C0)
#define ESW_PSNP(x)         (ESW_PSNP_BASE + (4 * (x)))

#define ESW_IPSNP_BASE      (0x0EC)
#define ESW_IPSNP(x)        (ESW_IPSNP_BASE + (4 * (x)))

#define ESW_PVRES_BASE      (0x100)
#define ESW_PVRES(x)        (ESW_PVRES_BASE + (4 * (x)))

#define ESW_IPRES           (0x140)

#define ESW_PRES_BASE       (0x180)
#define ESW_PRES(x)         (ESW_PRES_BASE + (4 * (x)))

#define ESW_PID_BASE        (0x200)
#define ESW_PID(x)          (ESW_PID_BASE + (4 * (x)))

#define ESW_VRES_BASE       (0x280)
#define ESW_VRES(x)         (ESW_VRES_BASE + (4 * (x)))

#define ESW_DISCN           (0x300)
#define ESW_DISCB           (0x304)
#define ESW_NDISCN          (0x308)
#define ESW_NDISCB          (0x30C)

#define ESW_ISR             (0x400)
#define ESW_IMR             (0x404)
#define ESW_RDSR            (0x408)
#define ESW_TDSR            (0x40C)
#define ESW_MRBR            (0x410)
#define ESW_RDAR            (0x414)
#define ESW_TDAR            (0x418)

#define ESW_LREC0           (0x500)
#define ESW_LREC1           (0x504)
#define ESW_LSR             (0x508)

struct addr_table64b_entry {
	u32 lo;  /* lower 32 bits */
	u32 hi;  /* upper 32 bits */
};

struct mtip_addr_table {
	struct addr_table64b_entry  mtip_table64b_entry[2048];
};

#define MCF_ESW_LOOKUP_MEM_OFFSET     0x4000
#ifdef CONFIG_SOC_IMX28
#define MCF_ESW_ENET_PORT_OFFSET      0x4000
#define ENET_SWI_PHYS_ADDR_OFFSET     0x8000
#endif

#ifdef CONFIG_SOC_VF610
/* Memory layout vf610 for ENET + MTIP L2 SW
 *
 * 0x400d0000 ENET0 (size 0x1000)
 * 0x400e0000 ENET1 (size 0x1000)
 *
 * 0x400e8000 ESW (switch) (size 0x4000)
 * 0x400eC000 ESW MAC Table (switch) (size 0x4000)
 */
#define MCF_ESW_ENET_PORT_OFFSET      0x1000
#endif

#define MCF_ESW_PER	(0x08)
#define MCF_ESW_DBCR	(0x14)
#define MCF_ESW_IMR	(0x404)

#define MCF_FEC_BASE_ADDR	(fep->enet_addr)
#define MCF_FEC_EIR		(0x04)
#define MCF_FEC_EIMR		(0x08)
#define MCF_FEC_MMFR		(0x40)
#define MCF_FEC_MSCR		(0x44)

#define MCF_FEC_RCR		(0x84)
#define MCF_FEC_TCR		(0xC4)
#define MCF_FEC_ECR		(0x24)

#define MCF_FEC_PALR          (0xE4)
#define MCF_FEC_PAUR          (0xE8)

#define MCF_FEC_ERDSR         (0x180)
#define MCF_FEC_ETDSR         (0x184)

#define MCF_FEC_IAUR          (0x118)
#define MCF_FEC_IALR          (0x11C)

#define MCF_FEC_GAUR          (0x120)
#define MCF_FEC_GALR          (0x124)

#define MCF_FEC_EMRBR         (0x188)

#define MCF_FEC_RCR_DRT	  BIT(1)
#define MCF_FEC_RCR_MII_MODE      BIT(2)
#define MCF_FEC_RCR_PROM          BIT(3)
#define MCF_FEC_RCR_FCE	  BIT(5)
#define MCF_FEC_RCR_RMII_MODE     BIT(8)
#define MCF_FEC_RCR_RMII_10BASET  BIT(9)
#define MCF_FEC_RCR_MAX_FL(x)     (((x) & 0x00003FFF) << 16)
#define MCF_FEC_RCR_CRC_FWD       BIT(14)
#define MCF_FEC_RCR_NO_LGTH_CHECK BIT(30)
#define MCF_FEC_TCR_FDEN          BIT(2)

#define MCF_FEC_ECR_RESET      BIT(0)
#define MCF_FEC_ECR_ETHER_EN   BIT(1)
#define MCF_FEC_ECR_MAGIC_ENA  BIT(2)
#define MCF_FEC_ECR_ENA_1588   BIT(4)
#define MCF_FEC_ECR_BYTESWP    BIT(8)

#define MTIP_ALIGNMENT   0xf
#define MCF_ENET_MII	BIT(23)

/* FEC MII MMFR bits definition */
#define FEC_MMFR_ST             BIT(30)
#define FEC_MMFR_OP_READ        BIT(29)
#define FEC_MMFR_OP_WRITE       BIT(28)
#define FEC_MMFR_PA(v)          (((v) & 0x1F) << 23)
#define FEC_MMFR_RA(v)          (((v) & 0x1F) << 18)
#define FEC_MMFR_TA             (2 << 16)
#define FEC_MMFR_DATA(v)        ((v) & 0xffff)

/* Port 0 backpressure congestion threshold */
#define P0BC_THRESHOLD		0x40
#define LEARNING_AGING_INTERVAL 100
/* Info received from Hardware Learning FIFO,
 * holding MAC address and corresponding Hash Value and
 * port number where the frame was received (disassembled).
 */
struct mtip_port_info {
	/* MAC lower 32 bits (first byte is 7:0). */
	u32 maclo;
	/* MAC upper 16 bits (47:32). */
	u32 machi;
	/* the hash value for this MAC address. */
	u32 hash;
	/* the port number this MAC address is associated with. */
	u32 port;
};

/* Define the buffer descriptor structure. */
struct cbd_t {
	u16 cbd_datlen;		/* Data length */
	u16 cbd_sc;			/* Control and status info */
	u32 cbd_bufaddr;		/* Buffer address */
};

/* The switch buffer descriptors track the ring buffers. The rx_bd_base and
 * tx_bd_base always point to the base of the buffer descriptors.  The
 * cur_rx and cur_tx point to the currently available buffer.
 * The dirty_tx tracks the current buffer that is being sent by the
 * controller. The cur_tx and dirty_tx are equal under both completely
 * empty and completely full conditions.  The empty/ready indicator in
 * the buffer descriptor determines the actual condition.
 */
struct switch_enet_private {
	/* Base addresses for HW registers of the switch device */
	void __iomem *hwp_enet, *hwp, *enet_addr;
	struct mtip_addr_table __iomem *hwentry;

	struct platform_device *pdev;

	/* Switch internals */
	struct mtip_port_info g_info;

	/* Clocks */
	struct clk_bulk_data *clks;
	int clk_num;
	struct clk *clk_ipg;
	struct clk *clk_ptp;

	/* PTP clk */
	bool ptp_clk_on;
	struct mutex ptp_clk_mutex;

	/* skbuff */
	unsigned char *tx_bounce[TX_RING_SIZE];
	struct sk_buff *tx_skbuff[TX_RING_SIZE];
	struct sk_buff *rx_skbuff[RX_RING_SIZE];
	ushort skb_cur;
	ushort skb_dirty;

	/* DMA */
	dma_addr_t bd_dma;
	struct cbd_t *rx_bd_base;	/* Address of Rx and Tx buffers. */
	struct cbd_t *tx_bd_base;
	struct cbd_t *cur_rx, *cur_tx;	/* The next free ring entry */
	struct cbd_t *dirty_tx;      /* The ring entries to be free()ed. */
	uint tx_full;

	/* Locking */
	spinlock_t hw_lock; /* Lock for HW configuration */
	spinlock_t learn_lock; /* Lock for learning DB adjustments */

	/* NAPI support */
	struct napi_struct napi;

	/* PTP */
	struct ptp_clock *ptp_clock;
	struct ptp_clock_info ptp_caps;
	unsigned long last_overflow_check;
	spinlock_t tmreg_lock;
	struct cyclecounter cc;
	struct timecounter tc;
	int rx_hwtstamp_filter;
	u32 base_incval;
	u32 cycle_speed;
	int hwts_rx_en;
	int hwts_tx_en;
	struct delayed_work time_keep;

	/* ptp clock period in ns*/
	unsigned int ptp_inc;

	/* pps  */
	int pps_channel;
	unsigned int reload_period;
	int pps_enable;
	unsigned int next_counter;
	bool perout_enable;
	struct hrtimer perout_timer;
	u64 perout_stime;

	struct {
		int pps_enable;
		u64 ns_sys, ns_phc;
		u32 at_corr;
		u8 at_inc_corr;
	} ptp_saved_state;

	/* Timer for Aging */
	struct timer_list timer_aging;
	struct task_struct *task;
	int at_block_overflows;
	int at_curr_entries;
	int curr_time;

	/* PHY and MDIO */
	struct mii_bus *mii_bus[SWITCH_EPORT_NUMBER];
	struct phy_device *phy_dev[SWITCH_EPORT_NUMBER];
	uint phy_speed;
	int link[SWITCH_EPORT_NUMBER];
	int full_duplex[SWITCH_EPORT_NUMBER];
	phy_interface_t phy_interface[SWITCH_EPORT_NUMBER];
	struct device_node *phy_np[SWITCH_EPORT_NUMBER];

	/* IRQ number */
	int irq;

	/* lan[01] ports */
	int n_ports;
	const char *ndev_name[SWITCH_EPORT_NUMBER];
	struct net_device *ndev[SWITCH_EPORT_NUMBER];
	unsigned char mac[SWITCH_EPORT_NUMBER][ETH_ALEN];

	/* Switch state */
	u8 br_members; /* Bit field with active members */
	u8 br_offload; /* Bridge in-HW offloading flag */
	int usage_count; /* Number of configured ports */

	/* Driver related */
	u32 quirks;
};

struct mtip_ndev_priv {
	int portnum;
	struct net_device *dev;
	struct net_device_stats stats;
	struct net_device *master_dev;
	struct switch_enet_private *fep;
};

#define MCF_FEC_MII_DATA	0x040 /* MII manage frame reg */
#define MCF_FEC_GRP_HASH_TABLE_HIGH	0x120 /* High 32bits hash table */
#define MCF_FEC_GRP_HASH_TABLE_LOW	0x124 /* Low 32bits hash table */

#define BD_SC_EMPTY     ((ushort)0x8000) /* Receive is empty */
#define BD_SC_READY     ((ushort)0x8000) /* Transmit is ready */
#define BD_SC_WRAP      ((ushort)0x2000) /* Last buffer descriptor */
#define BD_SC_INTRPT    ((ushort)0x1000) /* Interrupt on change */
#define BD_SC_CM        ((ushort)0x0200) /* Continuous mode */
#define BD_SC_ID        ((ushort)0x0100) /* Rec'd too many idles */
#define BD_SC_P         ((ushort)0x0100) /* xmt preamble */
#define BD_SC_BR        ((ushort)0x0020) /* Break received */
#define BD_SC_FR        ((ushort)0x0010) /* Framing error */
#define BD_SC_PR        ((ushort)0x0008) /* Parity error */
#define BD_SC_OV        ((ushort)0x0002) /* Overrun */
#define BD_SC_CD        ((ushort)0x0001)

/* Buffer descriptor control/status used by Ethernet receive. */
#define BD_ENET_RX_EMPTY        ((ushort)0x8000)
#define BD_ENET_RX_WRAP         ((ushort)0x2000)
#define BD_ENET_RX_INTR         ((ushort)0x1000)
#define BD_ENET_RX_LAST         ((ushort)0x0800)
#define BD_ENET_RX_FIRST        ((ushort)0x0400)
#define BD_ENET_RX_MISS         ((ushort)0x0100)
#define BD_ENET_RX_LG           ((ushort)0x0020)
#define BD_ENET_RX_NO           ((ushort)0x0010)
#define BD_ENET_RX_SH           ((ushort)0x0008)
#define BD_ENET_RX_CR           ((ushort)0x0004)
#define BD_ENET_RX_OV           ((ushort)0x0002)
#define BD_ENET_RX_CL           ((ushort)0x0001)
/* All status bits */
#define BD_ENET_RX_STATS        ((ushort)0x013f)

/* Buffer descriptor control/status used by Ethernet transmit.*/
#define BD_ENET_TX_READY        ((ushort)0x8000)
#define BD_ENET_TX_PAD          ((ushort)0x4000)
#define BD_ENET_TX_WRAP         ((ushort)0x2000)
#define BD_ENET_TX_INTR         ((ushort)0x1000)
#define BD_ENET_TX_LAST         ((ushort)0x0800)
#define BD_ENET_TX_TC           ((ushort)0x0400)
#define BD_ENET_TX_DEF          ((ushort)0x0200)
#define BD_ENET_TX_HB           ((ushort)0x0100)
#define BD_ENET_TX_LC           ((ushort)0x0080)
#define BD_ENET_TX_RL           ((ushort)0x0040)
#define BD_ENET_TX_RCMASK       ((ushort)0x003c)
#define BD_ENET_TX_UN           ((ushort)0x0002)
#define BD_ENET_TX_CSL          ((ushort)0x0001)
/* All status bits */
#define BD_ENET_TX_STATS        ((ushort)0x03ff)

/* Copy from validation code */
#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 256

#define TX_BD_R                 BIT(15)
#define TX_BD_TO1               BIT(14)
#define TX_BD_W                 BIT(13)
#define TX_BD_TO2               BIT(12)
#define TX_BD_L                 BIT(11)
#define TX_BD_TC                BIT(10)

#define TX_BD_INT       BIT(30)
#define TX_BD_TS        BIT(29)
#define TX_BD_PINS      BIT(28)
#define TX_BD_IINS      BIT(27)
#define TX_BD_TXE       BIT(15)
#define TX_BD_UE        BIT(13)
#define TX_BD_EE        BIT(12)
#define TX_BD_FE        BIT(11)
#define TX_BD_LCE       BIT(10)
#define TX_BD_OE        BIT(9)
#define TX_BD_TSE       BIT(8)
#define TX_BD_BDU       BIT(31)

#define RX_BD_E                 BIT(15)
#define RX_BD_R01               BIT(14)
#define RX_BD_W                 BIT(13)
#define RX_BD_R02               BIT(12)
#define RX_BD_L                 BIT(11)
#define RX_BD_M                 BIT(8)
#define RX_BD_BC                BIT(7)
#define RX_BD_MC                BIT(6)
#define RX_BD_LG                BIT(5)
#define RX_BD_NO                BIT(4)
#define RX_BD_CR                BIT(2)
#define RX_BD_OV                BIT(1)
#define RX_BD_TR                BIT(0)

#define RX_BD_ME               BIT(31)
#define RX_BD_PE               0x04000000
#define RX_BD_CE               0x02000000
#define RX_BD_UC               0x01000000
#define RX_BD_INT              0x00800000
#define RX_BD_ICE              BIT(5)
#define RX_BD_PCR              BIT(4)
#define RX_BD_VLAN             BIT(2)
#define RX_BD_IPV6             BIT(1)
#define RX_BD_FRAG             BIT(0)
#define RX_BD_BDU              BIT(31)
/****************************************************************************/

/* Address Table size in bytes(2048 64bit entry ) */
#define MTIP_ATABLE_MEM_SIZE         (2048 * 8)
/* How many 64-bit elements fit in the address table */
#define MTIP_ATABLE_MEM_NUM_ENTRIES  (2048)
/* Address Table Maximum number of entries in each Slot */
#define ATABLE_ENTRY_PER_SLOT 8
/* log2(ATABLE_ENTRY_PER_SLOT)*/
#define ATABLE_ENTRY_PER_SLOT_bits 3
/* entry size in byte */
#define ATABLE_ENTRY_SIZE     8
/*  slot size in byte */
#define ATABLE_SLOT_SIZE    (ATABLE_ENTRY_PER_SLOT * ATABLE_ENTRY_SIZE)
/* width of timestamp variable (bits) within address table entry */
#define AT_DENTRY_TIMESTAMP_WIDTH    10
/* number of bits for port number storage */
#define AT_DENTRY_PORT_WIDTH     4
/* number of bits for port bitmask number storage */
#define AT_SENTRY_PORT_WIDTH     11
/* address table static entry port bitmask start address bit */
#define AT_SENTRY_PORTMASK_shift     21
/* address table static entry priority start address bit */
#define AT_SENTRY_PRIO_shift     18
/* address table dynamic entry port start address bit */
#define AT_DENTRY_PORT_shift     28
/* address table dynamic entry timestamp start address bit */
#define AT_DENTRY_TIME_shift     18
/* address table entry record type start address bit */
#define AT_ENTRY_TYPE_shift     17
/* address table entry record type bit: 1 static, 0 dynamic */
#define AT_ENTRY_TYPE_STATIC      1
#define AT_ENTRY_TYPE_DYNAMIC     0
/* address table entry record valid start address bit */
#define AT_ENTRY_VALID_shift     16
#define AT_ENTRY_RECORD_VALID     1

/* return block corresponding to the 8 bit hash value calculated */
#define GET_BLOCK_PTR(hash)  ((hash) << 3)
#define AT_EXTRACT_TIMESTAMP(x) \
	(((x) >> AT_DENTRY_TIME_shift) & ((1 << AT_DENTRY_TIMESTAMP_WIDTH) - 1))
#define AT_EXTRACT_PORT(x)   \
	(((x) >> AT_DENTRY_PORT_shift) & ((1 << AT_DENTRY_PORT_WIDTH) - 1))
#define TIMEDELTA(newtime, oldtime) \
	(((newtime) - (oldtime)) & \
	  ((1 << AT_DENTRY_TIMESTAMP_WIDTH) - 1))

/* increment time value respecting modulo. */
static inline int mtip_timeincrement(int time)
{
	return (time + 1) & ((1 << AT_DENTRY_TIMESTAMP_WIDTH) - 1);
}

/* ------------------------------------------------------------------------- */
/* Bit definitions and macros for MCF_ESW_REVISION */
#define MCF_MTIP_REVISION_CORE_REVISION(x)      ((x) & 0x0000FFFF)
#define MCF_MTIP_REVISION_CUSTOMER_REVISION(x)  (((x) & 0xFFFF0000) >> 16)

/* Bit definitions and macros for MCF_ESW_PER */
#define MCF_ESW_PER_TE0                        BIT(0)
#define MCF_ESW_PER_TE1                        BIT(1)
#define MCF_ESW_PER_TE2                        BIT(2)
#define MCF_ESW_PER_RE0                        BIT(16)
#define MCF_ESW_PER_RE1                        BIT(17)
#define MCF_ESW_PER_RE2                        BIT(18)

/* Bit definitions and macros for MCF_ESW_VLANV */
#define MCF_ESW_VLANV_VV0                      BIT(0)
#define MCF_ESW_VLANV_VV1                      BIT(1)
#define MCF_ESW_VLANV_VV2                      BIT(2)
#define MCF_ESW_VLANV_DU0                      BIT(16)
#define MCF_ESW_VLANV_DU1                      BIT(17)
#define MCF_ESW_VLANV_DU2                      BIT(18)

/* Bit definitions and macros for MCF_ESW_DBCR */
#define MCF_ESW_DBCR_P0                        BIT(0)
#define MCF_ESW_DBCR_P1                        BIT(1)
#define MCF_ESW_DBCR_P2                        BIT(2)

/* Bit definitions and macros for MCF_ESW_DMCR */
#define MCF_ESW_DMCR_P0                        BIT(0)
#define MCF_ESW_DMCR_P1                        BIT(1)
#define MCF_ESW_DMCR_P2                        BIT(2)

/* Bit definitions and macros for MCF_ESW_BKLR */
#define MCF_ESW_BKLR_BE0                       BIT(0)
#define MCF_ESW_BKLR_BE1                       BIT(1)
#define MCF_ESW_BKLR_BE2                       BIT(2)
#define MCF_ESW_BKLR_LD0                       BIT(16)
#define MCF_ESW_BKLR_LD1                       BIT(17)
#define MCF_ESW_BKLR_LD2                       BIT(18)

/* Bit definitions and macros for MCF_ESW_BMPC */
#define MCF_ESW_BMPC_PORT(x)                   (((x) & 0x0000000F) << 0)
#define MCF_ESW_BMPC_MSG_TX                    BIT(5)
#define MCF_ESW_BMPC_EN                        BIT(6)
#define MCF_ESW_BMPC_DIS                       BIT(7)
#define MCF_ESW_BMPC_PRIORITY(x)               (((x) & 0x00000007) << 13)
#define MCF_ESW_BMPC_PORTMASK(x)               (((x) & 0x00000007) << 16)

/* Bit definitions and macros for MCF_ESW_MODE */
#define MCF_ESW_MODE_SW_RST                    BIT(0)
#define MCF_ESW_MODE_SW_EN                     BIT(1)
#define MCF_ESW_MODE_STOP                      BIT(7)
#define MCF_ESW_MODE_CRC_TRAN                  BIT(8)
#define MCF_ESW_MODE_P0CT                      BIT(9)
#define MCF_ESW_MODE_STATRST                   BIT(31)

/* Bit definitions and macros for MCF_ESW_VIMSEL */
#define MCF_ESW_VIMSEL_IM0(x)                  (((x) & 0x00000003) << 0)
#define MCF_ESW_VIMSEL_IM1(x)                  (((x) & 0x00000003) << 2)
#define MCF_ESW_VIMSEL_IM2(x)                  (((x) & 0x00000003) << 4)

/* Bit definitions and macros for MCF_ESW_VOMSEL */
#define MCF_ESW_VOMSEL_OM0(x)                  (((x) & 0x00000003) << 0)
#define MCF_ESW_VOMSEL_OM1(x)                  (((x) & 0x00000003) << 2)
#define MCF_ESW_VOMSEL_OM2(x)                  (((x) & 0x00000003) << 4)

/* Bit definitions and macros for MCF_ESW_VIMEN */
#define MCF_ESW_VIMEN_EN0                      BIT(0)
#define MCF_ESW_VIMEN_EN1                      BIT(1)
#define MCF_ESW_VIMEN_EN2                      BIT(2)

/* Bit definitions and macros for MCF_ESW_VID */
#define MCF_ESW_VID_TAG(x)                     (((x) & 0xFFFFFFFF) << 0)

/* Bit definitions and macros for MCF_ESW_MCR */
#define MCF_ESW_MCR_PORT(x)                    (((x) & 0x0000000F) << 0)
#define MCF_ESW_MCR_MEN                        BIT(4)
#define MCF_ESW_MCR_INGMAP                     BIT(5)
#define MCF_ESW_MCR_EGMAP                      BIT(6)
#define MCF_ESW_MCR_INGSA                      BIT(7)
#define MCF_ESW_MCR_INGDA                      BIT(8)
#define MCF_ESW_MCR_EGSA                       BIT(9)
#define MCF_ESW_MCR_EGDA                       BIT(10)

/* Bit definitions and macros for MCF_ESW_EGMAP */
#define MCF_ESW_EGMAP_EG0                      BIT(0)
#define MCF_ESW_EGMAP_EG1                      BIT(1)
#define MCF_ESW_EGMAP_EG2                      BIT(2)

/* Bit definitions and macros for MCF_ESW_INGMAP */
#define MCF_ESW_INGMAP_ING0                    BIT(0)
#define MCF_ESW_INGMAP_ING1                    BIT(1)
#define MCF_ESW_INGMAP_ING2                    BIT(2)

/* Bit definitions and macros for MCF_ESW_INGSAL */
#define MCF_ESW_INGSAL_ADDLOW(x)               (((x) & 0xFFFFFFFF) << 0)

/* Bit definitions and macros for MCF_ESW_INGSAH */
#define MCF_ESW_INGSAH_ADDHIGH(x)              (((x) & 0x0000FFFF) << 0)

/* Bit definitions and macros for MCF_ESW_INGDAL */
#define MCF_ESW_INGDAL_ADDLOW(x)               (((x) & 0xFFFFFFFF) << 0)

/* Bit definitions and macros for MCF_ESW_INGDAH */
#define MCF_ESW_INGDAH_ADDHIGH(x)              (((x) & 0x0000FFFF) << 0)

/* Bit definitions and macros for MCF_ESW_ENGSAL */
#define MCF_ESW_ENGSAL_ADDLOW(x)               (((x) & 0xFFFFFFFF) << 0)

/* Bit definitions and macros for MCF_ESW_ENGSAH */
#define MCF_ESW_ENGSAH_ADDHIGH(x)              (((x) & 0x0000FFFF) << 0)

/* Bit definitions and macros for MCF_ESW_ENGDAL */
#define MCF_ESW_ENGDAL_ADDLOW(x)               (((x) & 0xFFFFFFFF) << 0)

/* Bit definitions and macros for MCF_ESW_ENGDAH */
#define MCF_ESW_ENGDAH_ADDHIGH(x)              (((x) & 0x0000FFFF) << 0)

/* Bit definitions and macros for MCF_ESW_MCVAL */
#define MCF_ESW_MCVAL_COUNT(x)                 (((x) & 0x000000FF) << 0)

/* Bit definitions and macros for MCF_ESW_MMSR */
#define MCF_ESW_MMSR_BUSY                      BIT(0)
#define MCF_ESW_MMSR_NOCELL                    BIT(1)
#define MCF_ESW_MMSR_MEMFULL                   BIT(2)
#define MCF_ESW_MMSR_MFLATCH                   BIT(3)
#define MCF_ESW_MMSR_DQ_GRNT                   BIT(6)
#define MCF_ESW_MMSR_CELLS_AVAIL(x)            (((x) & 0x000000FF) << 16)

/* Bit definitions and macros for MCF_ESW_LMT */
#define MCF_ESW_LMT_THRESH(x)                  (((x) & 0x000000FF) << 0)

/* Bit definitions and macros for MCF_ESW_LFC */
#define MCF_ESW_LFC_COUNT(x)                   (((x) & 0xFFFFFFFF) << 0)

/* Bit definitions and macros for MCF_ESW_PCSR */
#define MCF_ESW_PCSR_PC0                       BIT(0)
#define MCF_ESW_PCSR_PC1                       BIT(1)
#define MCF_ESW_PCSR_PC2                       BIT(2)

/* Bit definitions and macros for MCF_ESW_IOSR */
#define MCF_ESW_IOSR_OR0                       BIT(0)
#define MCF_ESW_IOSR_OR1                       BIT(1)
#define MCF_ESW_IOSR_OR2                       BIT(2)

/* Bit definitions and macros for MCF_ESW_QWT */
#define MCF_ESW_QWT_Q0WT(x)                    (((x) & 0x0000001F) << 0)
#define MCF_ESW_QWT_Q1WT(x)                    (((x) & 0x0000001F) << 8)
#define MCF_ESW_QWT_Q2WT(x)                    (((x) & 0x0000001F) << 16)
#define MCF_ESW_QWT_Q3WT(x)                    (((x) & 0x0000001F) << 24)

/* Bit definitions and macros for MCF_ESW_P0BCT */
#define MCF_ESW_P0BCT_THRESH(x)                (((x) & 0x000000FF) << 0)

/* Bit definitions and macros for MCF_ESW_P0FFEN */
#define MCF_ESW_P0FFEN_FEN                     BIT(0)
#define MCF_ESW_P0FFEN_FD(x)                   (((x) & 0x00000003) << 2)

/* Bit definitions and macros for MCF_ESW_PSNP */
#define MCF_ESW_PSNP_EN                        BIT(0)
#define MCF_ESW_PSNP_MODE(x)                   (((x) & 0x00000003) << 1)
#define MCF_ESW_PSNP_CD                        BIT(3)
#define MCF_ESW_PSNP_CS                        BIT(4)
#define MCF_ESW_PSNP_PORT_COMPARE(x)           (((x) & 0x0000FFFF) << 16)

/* Bit definitions and macros for MCF_ESW_IPSNP */
#define MCF_ESW_IPSNP_EN                       BIT(0)
#define MCF_ESW_IPSNP_MODE(x)                  (((x) & 0x00000003) << 1)
#define MCF_ESW_IPSNP_PROTOCOL(x)              (((x) & 0x000000FF) << 8)

/* Bit definitions and macros for MCF_ESW_PVRES */
#define MCF_ESW_PVRES_PRI0(x)                  (((x) & 0x00000007) << 0)
#define MCF_ESW_PVRES_PRI1(x)                  (((x) & 0x00000007) << 3)
#define MCF_ESW_PVRES_PRI2(x)                  (((x) & 0x00000007) << 6)
#define MCF_ESW_PVRES_PRI3(x)                  (((x) & 0x00000007) << 9)
#define MCF_ESW_PVRES_PRI4(x)                  (((x) & 0x00000007) << 12)
#define MCF_ESW_PVRES_PRI5(x)                  (((x) & 0x00000007) << 15)
#define MCF_ESW_PVRES_PRI6(x)                  (((x) & 0x00000007) << 18)
#define MCF_ESW_PVRES_PRI7(x)                  (((x) & 0x00000007) << 21)

/* Bit definitions and macros for MCF_ESW_IPRES */
#define MCF_ESW_IPRES_ADDRESS(x)               (((x) & 0x000000FF) << 0)
#define MCF_ESW_IPRES_IPV4SEL                  BIT(8)
#define MCF_ESW_IPRES_PRI0(x)                  (((x) & 0x00000003) << 9)
#define MCF_ESW_IPRES_PRI1(x)                  (((x) & 0x00000003) << 11)
#define MCF_ESW_IPRES_PRI2(x)                   (((x) & 0x00000003) << 13)
#define MCF_ESW_IPRES_READ                     BIT(31)

/* Bit definitions and macros for MCF_ESW_PRES */
#define MCF_ESW_PRES_VLAN                      BIT(0)
#define MCF_ESW_PRES_IP                        BIT(1)
#define MCF_ESW_PRES_MAC                       BIT(2)
#define MCF_ESW_PRES_DFLT_PRI(x)               (((x) & 0x00000007) << 4)

/* Bit definitions and macros for MCF_ESW_PID */
#define MCF_ESW_PID_VLANID(x)                  (((x) & 0x0000FFFF) << 0)

/* Bit definitions and macros for MCF_ESW_VRES */
#define MCF_ESW_VRES_P0                        BIT(0)
#define MCF_ESW_VRES_P1                        BIT(1)
#define MCF_ESW_VRES_P2                        BIT(2)
#define MCF_ESW_VRES_VLANID(x)                 (((x) & 0x00000FFF) << 3)

/* Bit definitions and macros for MCF_ESW_DISCN */
#define MCF_ESW_DISCN_COUNT(x)                 (((x) & 0xFFFFFFFF) << 0)

/* Bit definitions and macros for MCF_ESW_DISCB */
#define MCF_ESW_DISCB_COUNT(x)                 (((x) & 0xFFFFFFFF) << 0)

/* Bit definitions and macros for MCF_ESW_NDISCN */
#define MCF_ESW_NDISCN_COUNT(x)                (((x) & 0xFFFFFFFF) << 0)

/* Bit definitions and macros for MCF_ESW_NDISCB */
#define MCF_ESW_NDISCB_COUNT(x)                (((x) & 0xFFFFFFFF) << 0)

/* Bit definitions and macros for MCF_ESW_POQC */
#define MCF_ESW_POQC_COUNT(x)                  (((x) & 0xFFFFFFFF) << 0)

/* Bit definitions and macros for MCF_ESW_PMVID */
#define MCF_ESW_PMVID_COUNT(x)                 (((x) & 0xFFFFFFFF) << 0)

/* Bit definitions and macros for MCF_ESW_PMVTAG */
#define MCF_ESW_PMVTAG_COUNT(x)                (((x) & 0xFFFFFFFF) << 0)

/* Bit definitions and macros for MCF_ESW_PBL */
#define MCF_ESW_PBL_COUNT(x)                   (((x) & 0xFFFFFFFF) << 0)

/* Bit definitions and macros for MCF_ESW_ISR */
#define MCF_ESW_ISR_EBERR                      BIT(0)
#define MCF_ESW_ISR_RXB                        BIT(1)
#define MCF_ESW_ISR_RXF                        BIT(2)
#define MCF_ESW_ISR_TXB                        BIT(3)
#define MCF_ESW_ISR_TXF                        BIT(4)
#define MCF_ESW_ISR_QM                         BIT(5)
#define MCF_ESW_ISR_OD0                        BIT(6)
#define MCF_ESW_ISR_OD1                        BIT(7)
#define MCF_ESW_ISR_OD2                        BIT(8)
#define MCF_ESW_ISR_LRN                        BIT(9)

/* Bit definitions and macros for MCF_ESW_IMR */
#define MCF_ESW_IMR_EBERR                      BIT(0)
#define MCF_ESW_IMR_RXB                        BIT(1)
#define MCF_ESW_IMR_RXF                        BIT(2)
#define MCF_ESW_IMR_TXB                        BIT(3)
#define MCF_ESW_IMR_TXF                        BIT(4)
#define MCF_ESW_IMR_QM                         BIT(5)
#define MCF_ESW_IMR_OD0                        BIT(6)
#define MCF_ESW_IMR_OD1                        BIT(7)
#define MCF_ESW_IMR_OD2                        BIT(8)
#define MCF_ESW_IMR_LRN                        BIT(9)

/* Bit definitions and macros for MCF_ESW_RDSR */
#define MCF_ESW_RDSR_ADDRESS(x)                (((x) & 0x3FFFFFFF) << 2)

/* Bit definitions and macros for MCF_ESW_TDSR */
#define MCF_ESW_TDSR_ADDRESS(x)                (((x) & 0x3FFFFFFF) << 2)

/* Bit definitions and macros for MCF_ESW_MRBR */
#define MCF_ESW_MRBR_SIZE(x)                   (((x) & 0x000003FF) << 4)

/* Bit definitions and macros for MCF_ESW_RDAR */
#define MCF_ESW_RDAR_R_DES_ACTIVE              BIT(24)

/* Bit definitions and macros for MCF_ESW_TDAR */
#define MCF_ESW_TDAR_X_DES_ACTIVE              BIT(24)

/* Bit definitions and macros for MCF_ESW_LREC0 */
#define MCF_ESW_LREC0_MACADDR0(x)              (((x) & 0xFFFFFFFF) << 0)

/* Bit definitions and macros for MCF_ESW_LREC1 */
#define MCF_ESW_LREC1_MACADDR1(x)              (((x) & 0x0000FFFF) << 0)
#define MCF_ESW_LREC1_HASH(x)                  (((x) & 0x000000FF) << 16)
#define MCF_ESW_LREC1_SWPORT(x)                (((x) & 0x00000003) << 24)

/* Bit definitions and macros for MCF_ESW_LSR */
#define MCF_ESW_LSR_DA                         BIT(0)

/* QUIRKS */
/* Controller needs driver to swap frame */
#define FEC_QUIRK_SWAP_FRAME		BIT(1)
/* ENET Block Guide/ Chapter for the iMX6SX (PELE) address one issue:
 * After set ENET_ATCR[Capture], there need some time cycles before the counter
 * value is capture in the register clock domain.
 * The wait-time-cycles is at least 6 clock cycles of the slower clock between
 * the register clock and the 1588 clock. The 1588 ts_clk is fixed to 25Mhz,
 * register clock is 66Mhz, so the wait-time-cycles must be greater than 240ns
 * (40ns * 6).
 */
#define FEC_QUIRK_BUG_CAPTURE		BIT(10)
/* Controller has only one MDIO bus */
#define FEC_QUIRK_SINGLE_MDIO		BIT(11)

#define MTIP_PORT_FORWARDING_INIT 0xFF

/* Switch Management functions */
int mtip_vlan_input_process(struct switch_enet_private *fep,
			    int port, int mode, unsigned short port_vlanid,
			    int vlan_verify_en, int vlan_domain_num,
			    int vlan_domain_port);
int mtip_set_vlan_verification(struct switch_enet_private *fep, int port,
			       int vlan_domain_verify_en,
			       int vlan_discard_unknown_en);
int mtip_port_multicast_config(struct switch_enet_private *fep, int port,
			       bool enable);
int mtip_vlan_output_process(struct switch_enet_private *fep, int port,
			     int mode);
void mtip_switch_en_port_separation(struct switch_enet_private *fep);
void mtip_switch_dis_port_separation(struct switch_enet_private *fep);
int mtip_port_broadcast_config(struct switch_enet_private *fep,
			       int port, bool enable);
int mtip_forced_forward(struct switch_enet_private *fep, int port, bool enable);
int mtip_port_learning_config(struct switch_enet_private *fep, int port,
			      bool disable, bool irq_adj);
int mtip_port_blocking_config(struct switch_enet_private *fep, int port,
			      bool enable);
bool mtip_is_switch_netdev_port(const struct net_device *ndev);
int mtip_bridge_register_notifiers(struct switch_enet_private *fep);
void mtip_bridge_unregister_notifiers(struct switch_enet_private *fep);
int mtip_set_static_table_entry(unsigned char *mac_addr, unsigned int port,
                                struct switch_enet_private *fep);
int mtip_clear_static_table_entry(unsigned char *mac_addr, unsigned int port,
                                  struct switch_enet_private *fep);
int mtip_switchdev_register_notifiers(struct switch_enet_private *fep);
void mtip_switchdev_unregister_notifiers(struct switch_enet_private *fep);
int mtip_port_enable_config(struct switch_enet_private *fep, int port,
			    bool tx_en, bool rx_en);
void mtip_clear_atable(struct switch_enet_private *fep);
void mtip_clear_atable_dynamic_entries(struct switch_enet_private *fep);
/* PTP */
void fec_ptp_init(struct platform_device *pdev, int irq_idx);
void fec_ptp_stop(struct platform_device *pdev);
#endif /* __MTIP_L2SWITCH_H_ */
