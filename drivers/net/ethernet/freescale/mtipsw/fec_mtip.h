// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 DENX Software Engineering GmbH
 * Lukasz Majewski <lukma@denx.de>
 */

#ifndef __MTIP_L2SWITCH_H_
#define __MTIP_L2SWITCH_H_

/* Bit definitions and macros for MCF_ESW_MODE */
#define MCF_ESW_MODE_SW_RST BIT(0)
#define MCF_ESW_MODE_SW_EN  BIT(1)
#define MCF_ESW_MODE_STATRST BIT(31)

/* Port 0 backpressure congestion threshold */
#define P0BC_THRESHOLD		0x40

struct esw_output_queue_status {
	unsigned long ESW_MMSR;
	unsigned long ESW_LMT;
	unsigned long ESW_LFC;
	unsigned long ESW_PCSR;
	unsigned long ESW_IOSR;
	unsigned long ESW_QWT;
	unsigned long esw_reserved;
	unsigned long ESW_P0BCT;
};
struct esw_statistics_status {
	/*
	 * Total number of incoming frames processed
	 * but discarded in switch
	 */
	unsigned long ESW_DISCN;
	/*Sum of bytes of frames counted in ESW_DISCN*/
	unsigned long ESW_DISCB;
	/*
	 * Total number of incoming frames processed
	 * but not discarded in switch
	 */
	unsigned long ESW_NDISCN;
	/*Sum of bytes of frames counted in ESW_NDISCN*/
	unsigned long ESW_NDISCB;
};

struct esw_port_statistics_status {
	/*outgoing frames discarded due to transmit queue congestion*/
	unsigned long MCF_ESW_POQC;
	/*incoming frames discarded due to VLAN domain mismatch*/
	unsigned long MCF_ESW_PMVID;
	/*incoming frames discarded due to untagged discard*/
	unsigned long MCF_ESW_PMVTAG;
	/*incoming frames discarded due port is in blocking state*/
	unsigned long MCF_ESW_PBL;
};

struct l2switch_t {
	unsigned long ESW_REVISION;
	unsigned long ESW_SCRATCH;
	unsigned long ESW_PER;
	unsigned long reserved0[1];
	unsigned long ESW_VLANV;
	unsigned long ESW_DBCR;
	unsigned long ESW_DMCR;
	unsigned long ESW_BKLR;
	unsigned long ESW_BMPC;
	unsigned long ESW_MODE;
	unsigned long ESW_VIMSEL;
	unsigned long ESW_VOMSEL;
	unsigned long ESW_VIMEN;
	unsigned long ESW_VID;/*0x34*/
	/*from 0x38 0x3C*/
	unsigned long esw_reserved0[2];
	unsigned long ESW_MCR;/*0x40*/
	unsigned long ESW_EGMAP;
	unsigned long ESW_INGMAP;
	unsigned long ESW_INGSAL;
	unsigned long ESW_INGSAH;
	unsigned long ESW_INGDAL;
	unsigned long ESW_INGDAH;
	unsigned long ESW_ENGSAL;
	unsigned long ESW_ENGSAH;
	unsigned long ESW_ENGDAL;
	unsigned long ESW_ENGDAH;
	unsigned long ESW_MCVAL;/*0x6C*/
	/*from 0x70--0x7C*/
	unsigned long esw_reserved1[4];
	unsigned long ESW_MMSR;/*0x80*/
	unsigned long ESW_LMT;
	unsigned long ESW_LFC;
	unsigned long ESW_PCSR;
	unsigned long ESW_IOSR;
	unsigned long ESW_QWT;/*0x94*/
	unsigned long esw_reserved2[1];/*0x98*/
	unsigned long ESW_P0BCT;/*0x9C*/
	/*from 0xA0-0xB8*/
	unsigned long esw_reserved3[7];
	unsigned long ESW_P0FFEN;/*0xBC*/
	unsigned long ESW_PSNP[8];
	unsigned long ESW_IPSNP[8];
	unsigned long ESW_PVRES[3];
	/*from 0x10C-0x13C*/
	unsigned long esw_reserved4[13];
	unsigned long ESW_IPRES;/*0x140*/
	/*from 0x144-0x17C*/
	unsigned long esw_reserved5[15];
	unsigned long ESW_PRES[3];
	/*from 0x18C-0x1FC*/
	unsigned long esw_reserved6[29];
	unsigned long ESW_PID[3];
	/*from 0x20C-0x27C*/
	unsigned long esw_reserved7[29];
	unsigned long ESW_VRES[32];
	unsigned long ESW_DISCN;/*0x300*/
	unsigned long ESW_DISCB;
	unsigned long ESW_NDISCN;
	unsigned long ESW_NDISCB;/*0xFC0DC30C*/
	struct esw_port_statistics_status port_statistics_status[3];
	/*from 0x340-0x400*/
	unsigned long esw_reserved8[48];

	/*0xFC0DC400---0xFC0DC418*/
	/*unsigned long MCF_ESW_ISR;*/
	unsigned long   switch_ievent;             /* Interrupt event reg */
	/*unsigned long MCF_ESW_IMR;*/
	unsigned long   switch_imask;              /* Interrupt mask reg */
	/*unsigned long MCF_ESW_RDSR;*/
	unsigned long   fec_r_des_start;        /* Receive descriptor ring */
	/*unsigned long MCF_ESW_TDSR;*/
	unsigned long   fec_x_des_start;        /* Transmit descriptor ring */
	/*unsigned long MCF_ESW_MRBR;*/
	unsigned long   fec_r_buff_size;        /* Maximum receive buff size */
	/*unsigned long MCF_ESW_RDAR;*/
	unsigned long   fec_r_des_active;       /* Receive descriptor reg */
	/*unsigned long MCF_ESW_TDAR;*/
	unsigned long   fec_x_des_active;       /* Transmit descriptor reg */
	/*from 0x420-0x4FC*/
	unsigned long esw_reserved9[57];

	/*0xFC0DC500---0xFC0DC508*/
	unsigned long ESW_LREC0;
	unsigned long ESW_LREC1;
	unsigned long ESW_LSR;
};

struct  AddrTable64bEntry {
	unsigned int lo;  /* lower 32 bits */
	unsigned int hi;  /* upper 32 bits */
};

struct  eswAddrTable_t {
	struct AddrTable64bEntry  eswTable64bEntry[2048];
};

struct mtipl2sw_priv {
	struct fec_enet_private *fep[2];
	void __iomem *hwpsw;
	struct device *dev;
	struct net_device *br_ndev;

	/*
	 * Flag to indicate if L2 switch IP block is initialized and
	 * running.
	 */
	bool l2sw_on;

	/* Number of ports */
	int n_ports;

	/* Bit field with active members */
	u8 br_members;

	/* Registers to configure/setup L2 switch IP block */
	struct l2switch_t *fecp;

	/* Look-up MAC address table start from 0x800FC000 */
	struct eswAddrTable_t *hwentry;
};

#define MCF_ESW_RDAR_R_DES_ACTIVE BIT(24)

/* HW_ENET_SWI_PORT_ENA */
#define MCF_ESW_ENA_TRANSMIT_0 BIT(0)
#define MCF_ESW_ENA_TRANSMIT_1 BIT(1)
#define MCF_ESW_ENA_TRANSMIT_2 BIT(2)

#define MCF_ESW_ENA_RECEIVE_0 BIT(16)
#define MCF_ESW_ENA_RECEIVE_1 BIT(17)
#define MCF_ESW_ENA_RECEIVE_2 BIT(18)

#define MCF_ESW_ENA_PORT_0 (MCF_ESW_ENA_TRANSMIT_0 | MCF_ESW_ENA_RECEIVE_0)
#define MCF_ESW_ENA_PORT_1 (MCF_ESW_ENA_TRANSMIT_1 | MCF_ESW_ENA_RECEIVE_1)
#define MCF_ESW_ENA_PORT_2 (MCF_ESW_ENA_TRANSMIT_2 | MCF_ESW_ENA_RECEIVE_2)

/* L2 switch Maximum receive buff size */
#define MCF_ESW_R_BUFF_SIZE	0x14
/* L2 switch Receive Descriptor Active Register */
#define MCF_ESW_R_RDAR          0x18

/*
 * MCF_FEC_EMRBR corresponds to FEC_R_BUFF_SIZE_0 in fec main driver.
 * It is duplicated as for L2 switch FEC_R_BUFF_SIZE_0 is aliased
 * to different offset in switch IC. Hence the need to introduce new
 * #define.
 */
#define MCF_FEC_EMRBR          (0x188)

#define MCF_FEC_ERDSR(x)       ((x) << 2)

#define L2SW_PKT_MAXBLR_SIZE         1520

#define L2SW_CTRL_REG_OFFSET         0x4

#endif /* __MTIP_L2SWITCH_H_ */
