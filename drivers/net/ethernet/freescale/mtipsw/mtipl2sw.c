// SPDX-License-Identifier: GPL-2.0-only
/*
 *  L2 switch Controller (Ethernet L2 switch) driver for MTIP block.
 *
 *  Copyright (C) 2025 DENX Software Engineering GmbH
 *  Lukasz Majewski <lukma@denx.de>
 *
 *  Based on a previous work by:
 *
 *  Copyright 2010-2012 Freescale Semiconductor, Inc.
 *  Alison Wang (b18965@freescale.com)
 *  Jason Jin (Jason.jin@freescale.com)
 *
 *  Copyright (C) 2010-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *  Shrek Wu (B16972@freescale.com)
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/rtnetlink.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <net/page_pool/helpers.h>
#include <net/netlink.h>

#include "mtipl2sw.h"

static void swap_buffer(void *bufaddr, int len)
{
	unsigned int *buf = bufaddr;
	int i;

	for (i = 0; i < len; i += 4, buf++)
		swab32s(buf);
}


/* Set the last buffer to wrap */
static void mtip_set_last_buf_to_wrap(struct cbd_t *bdp)
{
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;
}

static struct clk_bulk_data imx28_clocks[] = {
	{ .id = "ahb" },
	{ .id = "enet_out" },
};

static struct clk_bulk_data vf_clocks[] = {
	{ .id = "esw" },
	{ .id = "esw_tab0" },
	{ .id = "esw_tab1" },
	{ .id = "esw_tab2" },
	{ .id = "esw_tab3" },
	{ .id = "ahb" },
	{ .id = "ipg1" },
	{ .id = "ahb1" },
};

struct mtip_bulk_clk {
	struct clk_bulk_data *c;
	int size;
};

struct mtip_devinfo {
	u32 quirks;
	struct mtip_bulk_clk clk;
};

struct mtip_dump_ctx {
	struct net_device *dev;
	struct sk_buff *skb;
	struct netlink_callback *cb;
	int idx;
};

static void mtip_enet_init(struct switch_enet_private *fep, int port)
{
	void __iomem *enet_addr = fep->enet_addr;
	u32 mii_speed, holdtime, reg;

	if (port == 2)
		enet_addr += MCF_ESW_ENET_PORT_OFFSET;

	reg = MCF_FEC_RCR_PROM | MCF_FEC_RCR_MII_MODE |
		FIELD_PREP(MCF_FEC_RCR_MAX_FL_MASK, 1522);

	if (fep->phy_interface[port - 1] == PHY_INTERFACE_MODE_RMII)
		reg |= MCF_FEC_RCR_RMII_MODE;

	writel(reg, enet_addr + MCF_FEC_RCR);

	writel(MCF_FEC_TCR_FDEN, enet_addr + MCF_FEC_TCR);
	writel(MCF_FEC_ECR_BYTESWP | MCF_FEC_ECR_ETHER_EN,
	       enet_addr + MCF_FEC_ECR);

	mii_speed = DIV_ROUND_UP(clk_get_rate(fep->clk_ipg), 5000000);
	mii_speed--;

	holdtime = DIV_ROUND_UP(clk_get_rate(fep->clk_ipg), 100000000) - 1;

	fep->phy_speed = mii_speed << 1 | holdtime << 8;

	writel(fep->phy_speed, enet_addr + MCF_FEC_MSCR);
}

static void mtip_setup_mac(struct net_device *dev)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	unsigned char *iap, mac_addr[ETH_ALEN];

	/* Use MAC address from DTS */
	iap = &fep->mac[priv->portnum - 1][0];

	/* Use MAC address set by bootloader */
	if (!is_valid_ether_addr(iap)) {
		*((__be32 *)&mac_addr[0]) =
			cpu_to_be32(readl(fep->enet_addr + MCF_FEC_PALR));
		*((__be16 *)&mac_addr[4]) =
			cpu_to_be16(readl(fep->enet_addr +
					  MCF_FEC_PAUR) >> 16);
		iap = &mac_addr[0];
	}

	/* Use random MAC address */
	if (!is_valid_ether_addr(iap)) {
		eth_hw_addr_random(dev);
		dev_info(&fep->pdev->dev, "Using random MAC address: %pM\n",
			 dev->dev_addr);
		iap = (unsigned char *)dev->dev_addr;
	}

	/* Adjust MAC if using macaddr (and increment if needed) */
	eth_hw_addr_gen(dev, iap, priv->portnum - 1);
}

/**
 * crc8_calc - calculate CRC for MAC storage
 *
 * @pmacaddress: A 6-byte array with the MAC address. The first byte is
 *               the first byte transmitted.
 *
 * Calculate Galois Field Arithmetic CRC for Polynom x^8+x^2+x+1.
 * It omits the final shift in of 8 zeroes a "normal" CRC would do
 * (getting the remainder).
 *
 *  Examples (hexadecimal values):<br>
 *   10-11-12-13-14-15  => CRC=0xc2
 *   10-11-cc-dd-ee-00  => CRC=0xe6
 *
 * Return: The 8-bit CRC in bits 7:0
 */
static int crc8_calc(unsigned char *pmacaddress)
{
	int byt; /* byte index */
	int bit; /* bit index */
	int crc = 0x12;
	int inval;

	for (byt = 0; byt < ETH_ALEN; byt++) {
		inval = (((int)pmacaddress[byt]) & 0xFF);
		/* shift bit 0 to bit 8 so all our bits
		 * travel through bit 8
		 * (simplifies below calc)
		 */
		inval <<= 8;

		for (bit = 0; bit < 8; bit++) {
			/* next input bit comes into d7 after shift */
			crc |= inval & 0x100;
			if (crc & 0x01)
				/* before shift  */
				crc ^= 0x1C0;

			crc >>= 1;
			inval >>= 1;
		}
	}
	/* upper bits are clean as we shifted in zeroes! */
	return crc;
}

static void mtip_read_atable(struct switch_enet_private *fep, int index,
			     u32 *read_lo, u32 *read_hi)
{
	struct addr_table64b_entry __iomem *atable_base =
		fep->hwentry->mtip_table64b_entry;

	*read_lo = readl(&atable_base[index].lo);
	*read_hi = readl(&atable_base[index].hi);
}

static void mtip_write_atable(struct switch_enet_private *fep, int index,
			      u32 write_lo, u32 write_hi)
{
	struct addr_table64b_entry __iomem *atable_base =
		fep->hwentry->mtip_table64b_entry;

	writel(write_lo, &atable_base[index].lo);
	writel(write_hi, &atable_base[index].hi);
}

/**
 * mtip_portinfofifo_read - Read element from receive FIFO
 *
 * @fep: Structure describing switch
 *
 * Read one element from the HW receive FIFO (Queue)
 * if available and return it.
 *
 * Return: mtip_port_info or NULL if no data is available.
 */
static
struct mtip_port_info *mtip_portinfofifo_read(struct switch_enet_private *fep)
{
	struct mtip_port_info *info = &fep->g_info;
	u32 reg;

	reg = readl(fep->hwp + ESW_LSR);
	if (reg == 0) {
		dev_dbg(&fep->pdev->dev, "%s: ESW_LSR = 0x%x\n", __func__, reg);
		return NULL;
	}

	/* read word from FIFO */
	info->maclo = readl(fep->hwp + ESW_LREC0);
	if (info->maclo == 0) {
		dev_dbg(&fep->pdev->dev, "%s: mac lo 0x%x\n", __func__,
			info->maclo);
		return NULL;
	}

	/* read 2nd word from FIFO */
	reg = readl(fep->hwp + ESW_LREC1);
	info->machi = reg & 0xFFFF;
	info->hash  = (reg >> 16) & 0xFF;
	info->port  = (reg >> 24) & 0xF;

	return info;
}

static void mtip_atable_get_entry_port_number(struct switch_enet_private *fep,
					      unsigned char *mac_addr, u8 *port)
{
	int block_index, block_index_end, entry;
	u32 mac_addr_lo, mac_addr_hi;
	u32 read_lo, read_hi;

	mac_addr_lo = (u32)((mac_addr[3] << 24) | (mac_addr[2] << 16) |
			    (mac_addr[1] << 8) | mac_addr[0]);
	mac_addr_hi = (u32)((mac_addr[5] << 8) | (mac_addr[4]));

	block_index = GET_BLOCK_PTR(crc8_calc(mac_addr));
	block_index_end = block_index + ATABLE_ENTRY_PER_SLOT;

	/* now search all the entries in the selected block */
	for (entry = block_index; entry < block_index_end; entry++) {
		mtip_read_atable(fep, entry, &read_lo, &read_hi);
		*port = MTIP_PORT_FORWARDING_INIT;

		if (read_lo == mac_addr_lo &&
		    ((read_hi & 0x0000FFFF) ==
		     (mac_addr_hi & 0x0000FFFF))) {
			/* found the correct address */
			if ((read_hi & (1 << 16)) && (!(read_hi & (1 << 17))))
				*port = FIELD_GET(AT_PORT_MASK, read_hi);
			break;
		}
	}

	dev_dbg(&fep->pdev->dev, "%s: MAC: %pM PORT: 0x%x\n", __func__,
		mac_addr, *port);
}

/* Clear complete MAC Look Up Table */
void mtip_clear_atable(struct switch_enet_private *fep)
{
	int index;

	for (index = 0; index < MTIP_ATABLE_MEM_NUM_ENTRIES; index++)
		mtip_write_atable(fep, index, 0, 0);
}

static int mtip_port_fdb_do_dump(const unsigned char *addr, bool is_static,
                                 void *data)
{
	struct mtip_dump_ctx *dump = data;
	u32 portid = NETLINK_CB(dump->cb->skb).portid;
	u32 seq = dump->cb->nlh->nlmsg_seq;
	struct nlmsghdr *nlh;
	struct ndmsg *ndm;

	if (dump->idx < dump->cb->args[2])
		goto skip;

	nlh = nlmsg_put(dump->skb, portid, seq, RTM_NEWNEIGH,
			sizeof(*ndm), NLM_F_MULTI);
	if (!nlh)
		return -EMSGSIZE;

	ndm = nlmsg_data(nlh);
	ndm->ndm_family  = AF_BRIDGE;
	ndm->ndm_pad1    = 0;
	ndm->ndm_pad2    = 0;
	ndm->ndm_flags   = NTF_SELF;
	ndm->ndm_type    = 0;
	ndm->ndm_ifindex = dump->dev->ifindex;
	ndm->ndm_state   = is_static ? NUD_NOARP : NUD_REACHABLE;

	if (nla_put(dump->skb, NDA_LLADDR, ETH_ALEN, addr))
		goto nla_put_failure;

	nlmsg_end(dump->skb, nlh);

skip:
	dump->idx++;
	return 0;

nla_put_failure:
	nlmsg_cancel(dump->skb, nlh);
	return -EMSGSIZE;
}

static int mtip_dump_mac_table(struct switch_enet_private *fep,
                               int port, void *data)
{
	unsigned char mac[ETH_ALEN], tport, is_static, prio, is_valid;
	int index, time, err = 0;
	u32 read_lo, read_hi;

	for (index = 0; index < MTIP_ATABLE_MEM_NUM_ENTRIES; index++) {
		mtip_read_atable(fep, index, &read_lo, &read_hi);

		mac[0] = read_lo & GENMASK(7,0);
		mac[1] = (read_lo >> 8)  & GENMASK(7,0);
		mac[2] = (read_lo >> 16) & GENMASK(7,0);
		mac[3] = (read_lo >> 24) & GENMASK(7,0);
		mac[4] = (read_hi) & GENMASK(7,0);
		mac[5] = (read_hi >> 8) & GENMASK(7,0);

		if (is_valid_ether_addr(mac)) {
			is_static = (read_hi >> AT_ENTRY_TYPE_shift) & 0x1;
			is_valid = (read_hi >> AT_ENTRY_VALID_shift) & 0x1;
			if (is_static == 1) {
				prio = (read_hi >> AT_SENTRY_PRIO_shift) &
					GENMASK(2, 0);
				tport = (read_hi >> AT_SENTRY_PORTMASK_shift) &
					GENMASK(2, 0);
				pr_debug("MAC TAB: S i: %04i %pM V: %d P: %d Prio: %d\n",
				         index, mac, is_valid, tport, prio);

				if (is_valid && (tport & BIT(port)))
					err = mtip_port_fdb_do_dump(mac, is_static,
					                            data);
			} else {
				time = (read_hi >> AT_DENTRY_TIME_shift) &
					GENMASK(9, 0);
				tport = (read_hi >> AT_DENTRY_PORT_shift) &
					GENMASK(3, 0);
				pr_debug("MAC TAB: D i: %04i %pM V: %d P: %d T: %d\n",
				         index, mac, is_valid, tport, time);

				if (is_valid && (port == tport))
					err = mtip_port_fdb_do_dump(mac, is_static,
					                            data);
			}
			if (err)
				break;
		}
	}

	return err;
}

static int mtip_port_fdb_dump(struct sk_buff *skb,
                              struct netlink_callback *cb,
                              struct net_device *dev,
                              struct net_device *filter_dev, int *idx)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	struct mtip_dump_ctx dump = {
		.dev = dev,
		.skb = skb,
		.cb = cb,
		.idx = *idx,
	};
	int port = priv->portnum;
	int ret = mtip_dump_mac_table(fep, port, &dump);

	*idx = dump.idx;
        return ret;
}

static int __mtip_update_atable_static(unsigned char *mac_addr, unsigned int port,
                                       unsigned int priority,
                                       struct switch_enet_private *fep, bool clear)
{
	unsigned long block_index, entry, index_end;
	u32 write_lo, write_hi, read_lo, read_hi;

	write_lo = (u32)((mac_addr[3] << 24) | (mac_addr[2] << 16) |
			 (mac_addr[1] << 8) | mac_addr[0]);
	write_hi = (u32)(0 | (port << AT_SENTRY_PORTMASK_shift) |
			 (priority << AT_SENTRY_PRIO_shift) |
			 (AT_ENTRY_TYPE_STATIC << AT_ENTRY_TYPE_shift) |
			 (AT_ENTRY_RECORD_VALID << AT_ENTRY_VALID_shift) |
			 (mac_addr[5] << 8) | (mac_addr[4]));

	block_index = GET_BLOCK_PTR(crc8_calc(mac_addr));
	index_end = block_index + ATABLE_ENTRY_PER_SLOT;
	/* Now search all the entries in the selected block */
	for (entry = block_index; entry < index_end; entry++) {
		mtip_read_atable(fep, entry, &read_lo, &read_hi);
		/* MAC address matched, so update the
		 * existing entry
		 * even if its a dynamic one
		 */
		if (read_lo == write_lo &&
		    ((read_hi & 0x0000FFFF) ==
		     (write_hi & 0x0000FFFF))) {
			clear ? mtip_write_atable(fep, entry, 0, 0) :
				mtip_write_atable(fep, entry, write_lo, write_hi);
			return 0;
		} else if (!(read_hi & (1 << 16))) {
			/* Fill this empty slot (valid bit zero),
			 * assuming no holes in the block
			 */
			clear ? mtip_write_atable(fep, entry, 0, 0) :
				mtip_write_atable(fep, entry, write_lo, write_hi);
			return 0;
		}
	}

	/* No space available for this static entry */
	return -ENOSPC;
}

/**
 * mtip_update_atable_static - Update switch static address table
 *
 * @mac_addr: Pointer to the array containing MAC address to
 *            be put as static entry
 * @port:     Port bitmask numbers to be added in static entry,
 *            valid values are 1-7
 * @priority: The priority for the static entry in table
 *
 * @fep:      Pointer to the structure describing the switch
 *
 * Updates MAC address lookup table with a static entry.
 *
 * Searches if the MAC address is already there in the block and replaces
 * the older entry with the new one. If MAC address is not there then puts
 * a new entry in the first empty slot available in the block.
 *
 * Return: 0 for a successful update else -ENOSPC when no slot available
 */
static int mtip_update_atable_static(unsigned char *mac_addr, unsigned int port,
				     unsigned int priority,
				     struct switch_enet_private *fep)
{
	return __mtip_update_atable_static(mac_addr, port, priority, fep, false);
}

int mtip_set_static_table_entry(unsigned char *mac_addr, unsigned int port,
                                struct switch_enet_private *fep)
{
	return __mtip_update_atable_static(mac_addr, BIT(port), 7, fep, false);
}

int mtip_clear_static_table_entry(unsigned char *mac_addr, unsigned int port,
                                  struct switch_enet_private *fep)
{
	return __mtip_update_atable_static(mac_addr, BIT(port), 7, fep, true);
}

static bool mtip_update_atable_dynamic1(u32 write_lo, u32 write_hi,
					int block_index, unsigned int port,
					unsigned int curr_time,
					struct switch_enet_private *fep)
{
	unsigned long entry, index_end;
	int time, timeold, indexold;
	u32 read_lo, read_hi;
	unsigned long conf;

	/* prepare update port and timestamp */
	conf = AT_ENTRY_RECORD_VALID << AT_ENTRY_VALID_shift;
	conf |= AT_ENTRY_TYPE_DYNAMIC << AT_ENTRY_TYPE_shift;
	conf |= curr_time << AT_DENTRY_TIME_shift;
	conf |= port << AT_DENTRY_PORT_shift;
	conf |= write_hi;

	/* linear search through all slot
	 * entries and update if found
	 */
	index_end = block_index + ATABLE_ENTRY_PER_SLOT;
	/* Now search all the entries in the selected block */
	for (entry = block_index; entry < index_end; entry++) {
		mtip_read_atable(fep, entry, &read_lo, &read_hi);
		if (read_lo == write_lo &&
		    ((read_hi & 0x0000FFFF) ==
		     (write_hi & 0x0000FFFF))) {
			/* found correct address,
			 * update timestamp.
			 */
			mtip_write_atable(fep, entry, write_lo, conf);

			return false;
		} else if (!(read_hi & (1 << 16))) {
			/* slot is empty, then use it
			 * for new entry
			 * Note: There are no holes,
			 * therefore cannot be any
			 * more that need to be compared.
			 */
			mtip_write_atable(fep, entry, write_lo, conf);
			return true;
		}
	}

	/* No more entry available in block overwrite oldest */
	timeold = 0;
	indexold = 0;
	for (entry = block_index; entry < index_end; entry++) {
		mtip_read_atable(fep, entry, &read_lo, &read_hi);
		time = FIELD_GET(AT_TIMESTAMP_MASK, read_hi);
		dev_dbg(&fep->pdev->dev, "%s : time %x currtime %x\n",
			__func__, time, curr_time);
		time = mtip_timedelta(curr_time, time);
		if (time > timeold) {
			/* is it older ? */
			timeold = time;
			indexold = entry;
		}
	}

	mtip_write_atable(fep, indexold, write_lo, conf);

	/* Statistics (do it inbetween writing to .lo and .hi */
	fep->at_block_overflows++;
	dev_err(&fep->pdev->dev, "%s update time, at_block_overflows %x\n",
		__func__, fep->at_block_overflows);
	/* newly inserted */
	return true;
}

/* dynamicms MAC address table learn and migration */
static void
mtip_atable_dynamicms_learn_migration(struct switch_enet_private *fep,
				      int curr_time, unsigned char *mac,
				      u8 *rx_port)
{
	u8 port = MTIP_PORT_FORWARDING_INIT;
	struct mtip_port_info *port_info;
	u32 rx_mac_lo = 0, rx_mac_hi = 0;
	int index;

	spin_lock(&fep->learn_lock);

	if (mac && is_valid_ether_addr(mac)) {
		rx_mac_lo = (u32)((mac[3] << 24) | (mac[2] << 16) |
				  (mac[1] << 8) | mac[0]);
		rx_mac_hi = (u32)((mac[5] << 8) | (mac[4]));
	}

	port_info = mtip_portinfofifo_read(fep);
	while (port_info) {
		/* get block index from lookup table */
		index = GET_BLOCK_PTR(port_info->hash);
		mtip_update_atable_dynamic1(port_info->maclo, port_info->machi,
					    index, port_info->port,
					    curr_time, fep);

		if (mac && is_valid_ether_addr(mac) &&
		    port == MTIP_PORT_FORWARDING_INIT) {
			if (rx_mac_lo == port_info->maclo &&
			    rx_mac_hi == port_info->machi) {
				/* The newly learned MAC is the source of
				 * our filtered frame.
				 */
				port = (u8)port_info->port;
			}
		}
		port_info = mtip_portinfofifo_read(fep);
	}

	if (rx_port)
		*rx_port = port;

	spin_unlock(&fep->learn_lock);
}

static void mtip_mgnt_timer(struct timer_list *t)
{
	struct switch_enet_private *fep = from_timer(fep, t, timer_mgnt);

	mtip_atable_dynamicms_learn_migration(fep, mtip_get_time(),
					      NULL, NULL);
	mod_timer(&fep->timer_mgnt,
		  jiffies + msecs_to_jiffies(LEARNING_AGING_INTERVAL));
}

static void esw_mac_addr_static(struct switch_enet_private *fep)
{
	int i;

	for (i = 0; i < SWITCH_EPORT_NUMBER; i++)
		mtip_update_atable_static((unsigned char *)
					  fep->ndev[i]->dev_addr, 7, 7, fep);
}

static void mtip_config_switch(struct switch_enet_private *fep)
{
	esw_mac_addr_static(fep);

	writel(0, fep->hwp + ESW_BKLR);

	/* Do NOT disable learning */
	mtip_port_learning_config(fep, 0, 0, 0);
	mtip_port_learning_config(fep, 1, 0, 0);
	mtip_port_learning_config(fep, 2, 0, 0);

	/* Disable blocking */
	mtip_port_blocking_config(fep, 0, 0);
	mtip_port_blocking_config(fep, 1, 0);
	mtip_port_blocking_config(fep, 2, 0);

	writel(MCF_ESW_IMR_TXF | MCF_ESW_IMR_RXF,
	       fep->hwp + ESW_IMR);

	mtip_port_enable_config(fep, 0, 1, 1);
	mtip_port_enable_config(fep, 1, 1, 1);
	mtip_port_enable_config(fep, 2, 1, 1);

	mtip_port_broadcast_config(fep, 0, 1);
	mtip_port_broadcast_config(fep, 1, 1);
	mtip_port_broadcast_config(fep, 2, 1);

	/* Disable multicast receive on port 0 (MGNT) */
	mtip_port_multicast_config(fep, 0, 0);
	mtip_port_multicast_config(fep, 1, 1);
	mtip_port_multicast_config(fep, 2, 1);

	/* Setup VLANs to provide port separation */
	if (!fep->br_offload)
		mtip_switch_en_port_separation(fep);
}

static netdev_tx_t mtip_start_xmit_port(struct sk_buff *skb,
					struct net_device *dev, int port)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	unsigned short status;
	struct cbd_t *bdp;
	void *bufaddr;

	spin_lock_bh(&fep->hw_lock);

	if (!fep->link[0] && !fep->link[1]) {
		/* Link is down or autonegotiation is in progress. */
		netif_stop_queue(dev);
		spin_unlock_bh(&fep->hw_lock);
		return NETDEV_TX_BUSY;
	}

	/* Fill in a Tx ring entry */
	bdp = fep->cur_tx;
	status = bdp->cbd_sc;

	if (status & BD_ENET_TX_READY) {
		/* All transmit buffers are full. Bail out.
		 * This should not happen, since dev->tbusy should be set.
		 */
		netif_stop_queue(dev);
		spin_unlock_bh(&fep->hw_lock);
		dev_err_ratelimited(&fep->pdev->dev, "%s: tx queue full!.\n",
				    dev->name);
		return NETDEV_TX_BUSY;
	}

	/* Clear all of the status flags */
	status &= ~BD_ENET_TX_STATS;

	/* Set buffer length and buffer pointer */
	bufaddr = skb->data;
	bdp->cbd_datlen = skb->len;

	/* On some FEC implementations data must be aligned on
	 * 4-byte boundaries. Use bounce buffers to copy data
	 * and get it aligned.
	 */
	if ((unsigned long)bufaddr & MTIP_ALIGNMENT ||
	    fep->quirks & FEC_QUIRK_SWAP_FRAME) {
		unsigned int index;

		index = bdp - fep->tx_bd_base;
		memcpy(fep->tx_bounce[index], skb->data, skb->len);
		bufaddr = fep->tx_bounce[index];

		if (fep->quirks & FEC_QUIRK_SWAP_FRAME)
			swap_buffer(bufaddr, skb->len);

	}

	/* Push the data cache so the CPM does not get stale memory
	 * data.
	 */
	bdp->cbd_bufaddr = dma_map_single(&fep->pdev->dev, bufaddr,
					  MTIP_SWITCH_TX_FRSIZE,
					  DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(&fep->pdev->dev, bdp->cbd_bufaddr))) {
		dev_err(&fep->pdev->dev,
			"Failed to map descriptor tx buffer\n");
		dev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		goto err;
	}

	/* Save skb pointer. */
	fep->tx_skbuff[fep->skb_cur] = skb;
	fep->skb_cur = (fep->skb_cur + 1) & TX_RING_MOD_MASK;

	/* Send it on its way.  Tell FEC it's ready, interrupt when done,
	 * it's the last BD of the frame, and to put the CRC on the end.
	 */

	status |= (BD_ENET_TX_READY | BD_ENET_TX_INTR | BD_ENET_TX_LAST |
		   BD_ENET_TX_TC);

	/* Synchronize all descriptor writes */
	wmb();
	bdp->cbd_sc = status;

	skb_tx_timestamp(skb);

	/* For port separation - force sending via specified port */
	if (!fep->br_offload && port != 0)
		mtip_forced_forward(fep, port, 1);

	/* Trigger transmission start */
	writel(MCF_ESW_TDAR_X_DES_ACTIVE, fep->hwp + ESW_TDAR);

	dev->stats.tx_bytes += skb->len;
	/* If this was the last BD in the ring,
	 * start at the beginning again.
	 */
	if (status & BD_ENET_TX_WRAP)
		bdp = fep->tx_bd_base;
	else
		bdp++;

	if (bdp == fep->dirty_tx) {
		fep->tx_full = 1;
		netif_stop_queue(dev);
	}

	fep->cur_tx = bdp;
 err:
	spin_unlock_bh(&fep->hw_lock);

	return NETDEV_TX_OK;
}

static netdev_tx_t mtip_start_xmit(struct sk_buff *skb,
				   struct net_device *dev)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);

	return mtip_start_xmit_port(skb, dev, priv->portnum);
}

static void mtip_configure_enet_mii(struct switch_enet_private *fep, int port)
{
	struct phy_device *phydev = fep->phy_dev[port - 1];
	struct net_device *dev = fep->ndev[port - 1];
	void __iomem *enet_addr = fep->enet_addr;
	int duplex = fep->full_duplex[port - 1];
	u32 rcr;

	if (port == 2)
		enet_addr += MCF_ESW_ENET_PORT_OFFSET;

	/* ECR */
	writel(MCF_FEC_ECR_BYTESWP | MCF_FEC_ECR_MAGIC_ENA,
	       enet_addr + MCF_FEC_ECR);

	/* EMRBR */
	writel(PKT_MAXBLR_SIZE, enet_addr + MCF_FEC_EMRBR);

	/* set the receive and transmit BDs ring base to
	 * hardware registers(ERDSR & ETDSR)
	 */
	writel(fep->bd_dma, enet_addr + MCF_FEC_ERDSR);
	writel((unsigned long)fep->bd_dma + sizeof(struct cbd_t) * RX_RING_SIZE,
	       enet_addr + MCF_FEC_ETDSR);

	writel(fep->phy_speed, enet_addr + MCF_FEC_MSCR);

	/* EIR */
	writel(0, enet_addr + MCF_FEC_EIR);

	/* IAUR */
	writel(0, enet_addr + MCF_FEC_IAUR);

	/* IALR */
	writel(0, enet_addr + MCF_FEC_IALR);

	/* GAUR */
	writel(0, enet_addr + MCF_FEC_GAUR);

	/* GALR */
	writel(0, enet_addr + MCF_FEC_GALR);

	/* EMRBR */
	writel(PKT_MAXBLR_SIZE, enet_addr + MCF_FEC_EMRBR);

	/* EIMR */
	writel(0, enet_addr + MCF_FEC_EIMR);

	/* PALR PAUR */
	/* Set the station address for the ENET Adapter */
	writel(dev->dev_addr[3] |
	       dev->dev_addr[2] << 8 |
	       dev->dev_addr[1] << 16 |
	       dev->dev_addr[0] << 24, enet_addr + MCF_FEC_PALR);
	writel(dev->dev_addr[5] << 16 |
	       (dev->dev_addr[4] + (unsigned char)(0)) << 24,
	       enet_addr + MCF_FEC_PAUR);

	/* RCR */
	rcr = readl(enet_addr + MCF_FEC_RCR);
	if (phydev && phydev->speed == SPEED_100)
		rcr &= ~MCF_FEC_RCR_RMII_10BASET;
	else
		rcr |= MCF_FEC_RCR_RMII_10BASET;

	if (duplex == DUPLEX_FULL)
		rcr &= ~MCF_FEC_RCR_DRT;
	else
		rcr |= MCF_FEC_RCR_DRT;

	writel(rcr, enet_addr + MCF_FEC_RCR);

	/* TCR */
	if (duplex == DUPLEX_FULL)
		writel(0x1C, enet_addr + MCF_FEC_TCR);
	else
		writel(0x18, enet_addr + MCF_FEC_TCR);

	/* ECR */
	writel(readl(enet_addr + MCF_FEC_ECR) | MCF_FEC_ECR_ETHER_EN,
	       enet_addr + MCF_FEC_ECR);
}

/* This function is called to start or restart the FEC during a link
 * change. This only happens when switching between half and full
 * duplex.
 */
static void mtip_switch_restart(struct net_device *dev, int duplex0,
				int duplex1)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	int i;

	 /* Perform a reset. We should wait for this. */
	writel(MCF_ESW_MODE_SW_RST, fep->hwp + ESW_MODE);

	/* Delay of 10us specified in the documentation to perform
	 * SW reset by the switch internally.
	 */
	udelay(10);
	writel(MCF_ESW_MODE_STATRST, fep->hwp + ESW_MODE);
	writel(MCF_ESW_MODE_SW_EN, fep->hwp + ESW_MODE);

	/* Management port configuration,
	 * make port 0 as management port
	 */
	writel(0, fep->hwp + ESW_BMPC);

	/* Clear any outstanding interrupt */
	writel(0xFFFFFFFF, fep->hwp + ESW_ISR);

	/* Set backpressure threshold to minimize discarded frames
	 * during due to congestion.
	 */
	writel(P0BC_THRESHOLD, fep->hwp + ESW_P0BCT);

	/* Set maximum receive buffer size */
	writel(PKT_MAXBLR_SIZE, fep->hwp + ESW_MRBR);

	/* Set receive and transmit descriptor base */
	writel(fep->bd_dma, fep->hwp + ESW_RDSR);
	writel((unsigned long)fep->bd_dma
		+ sizeof(struct cbd_t) * RX_RING_SIZE,
		fep->hwp + ESW_TDSR);

	fep->cur_tx = fep->tx_bd_base;
	fep->cur_rx = fep->rx_bd_base;
	fep->dirty_tx = fep->cur_tx;

	/* Reset SKB transmit buffers */
	fep->skb_cur = 0;
	fep->skb_dirty = 0;
	for (i = 0; i <= TX_RING_MOD_MASK; i++) {
		if (fep->tx_skbuff[i]) {
			dev_kfree_skb_any(fep->tx_skbuff[i]);
			fep->tx_skbuff[i] = NULL;
		}
	}

	fep->full_duplex[0] = duplex0;
	fep->full_duplex[1] = duplex1;

	mtip_configure_enet_mii(fep, 1);
	mtip_configure_enet_mii(fep, 2);
	mtip_clear_atable(fep);

	/* And last, enable the transmit and receive processing */
	writel(MCF_ESW_RDAR_R_DES_ACTIVE, fep->hwp + ESW_RDAR);

	/* Enable interrupts we wish to service */
	writel(0xFFFFFFFF, fep->hwp + ESW_ISR);
	writel(MCF_ESW_IMR_TXF | MCF_ESW_IMR_RXF,
	       fep->hwp + ESW_IMR);

	mtip_config_switch(fep);
}

static void mtip_print_hw_state(struct net_device *dev)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	struct cbd_t *bdp;
	int i;

	spin_lock_bh(&fep->hw_lock);
	dev_info(&dev->dev, "%s: transmit timed out.\n", dev->name);
	dev_info(&dev->dev,
		 "Ring data: cur_tx 0x%p%s, dirty_tx 0x%p cur_rx: 0x%p\n",
		 fep->cur_tx, fep->tx_full ? " (full)" : "", fep->dirty_tx,
		 fep->cur_rx);

	bdp = fep->tx_bd_base;
	dev_info(&dev->dev, " tx: %u buffers\n", TX_RING_SIZE);
	for (i = 0; i < TX_RING_SIZE; i++) {
		dev_info(&dev->dev, "  0x%p: %04x %04x %08x\n",
			 bdp, bdp->cbd_sc, bdp->cbd_datlen,
			 (int)bdp->cbd_bufaddr);
		bdp++;
	}

	bdp = fep->rx_bd_base;
	dev_info(&dev->dev, " rx: %lu buffers\n", RX_RING_SIZE);
	for (i = 0 ; i < RX_RING_SIZE; i++) {
		dev_info(&dev->dev, "  0x%p: %04x %04x %08x\n",
			 bdp, bdp->cbd_sc, bdp->cbd_datlen,
			 (int)bdp->cbd_bufaddr);
		bdp++;
	}
	spin_unlock_bh(&fep->hw_lock);
}

static void mtip_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);

	dev->stats.tx_errors++;
	DO_ONCE(mtip_print_hw_state, dev);

	schedule_work(&priv->tx_timeout_work);
}

static void mtip_timeout_work(struct work_struct *work)
{
	struct mtip_ndev_priv *priv =
		container_of(work, struct mtip_ndev_priv, tx_timeout_work);
	struct switch_enet_private *fep = priv->fep;
	struct net_device *dev = priv->dev;

	rtnl_lock();
	if (netif_device_present(dev) || netif_running(dev)) {
		napi_disable(&fep->napi);
		netif_tx_lock_bh(dev);
		mtip_switch_restart(dev, fep->full_duplex[0],
				    fep->full_duplex[1]);
		netif_tx_wake_all_queues(dev);
		netif_tx_unlock_bh(dev);
		napi_enable(&fep->napi);
	}
	rtnl_unlock();
}

static irqreturn_t mtip_interrupt(int irq, void *ptr_fep)
{
	struct switch_enet_private *fep = ptr_fep;
	irqreturn_t ret = IRQ_NONE;
	u32 int_events, int_imask;

	/* Get the interrupt events that caused us to be here */
	int_events = readl(fep->hwp + ESW_ISR);
	writel(int_events, fep->hwp + ESW_ISR);

	if (int_events & (MCF_ESW_ISR_RXF | MCF_ESW_ISR_TXF)) {
		ret = IRQ_HANDLED;
		/* Disable the RX interrupt */
		if (napi_schedule_prep(&fep->napi)) {
			int_imask = readl(fep->hwp + ESW_IMR);
			int_imask &= ~MCF_ESW_IMR_RXF;
			writel(int_imask, fep->hwp + ESW_IMR);
			__napi_schedule(&fep->napi);
		}
	}

	return ret;
}

static void mtip_switch_tx(struct net_device *dev)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	unsigned short status;
	struct sk_buff *skb;
	struct cbd_t *bdp;

	spin_lock_bh(&fep->hw_lock);
	bdp = fep->dirty_tx;

	while (((status = bdp->cbd_sc) & BD_ENET_TX_READY) == 0) {
		if (bdp == fep->cur_tx && fep->tx_full == 0)
			break;

		dma_unmap_single(&fep->pdev->dev, bdp->cbd_bufaddr,
				 MTIP_SWITCH_TX_FRSIZE, DMA_TO_DEVICE);
		bdp->cbd_bufaddr = 0;
		skb = fep->tx_skbuff[fep->skb_dirty];
		/* Check for errors */
		if (status & (BD_ENET_TX_HB | BD_ENET_TX_LC |
				   BD_ENET_TX_RL | BD_ENET_TX_UN |
				   BD_ENET_TX_CSL)) {
			dev->stats.tx_errors++;
			if (status & BD_ENET_TX_HB)  /* No heartbeat */
				dev->stats.tx_heartbeat_errors++;
			if (status & BD_ENET_TX_LC)  /* Late collision */
				dev->stats.tx_window_errors++;
			if (status & BD_ENET_TX_RL)  /* Retrans limit */
				dev->stats.tx_aborted_errors++;
			if (status & BD_ENET_TX_UN)  /* Underrun */
				dev->stats.tx_fifo_errors++;
			if (status & BD_ENET_TX_CSL) /* Carrier lost */
				dev->stats.tx_carrier_errors++;
		} else {
			dev->stats.tx_packets++;
		}

		if (status & BD_ENET_TX_READY)
			dev_err_ratelimited(&fep->pdev->dev,
				"Enet xmit interrupt and TX_READY.\n");

		/* Deferred means some collisions occurred during transmit,
		 * but we eventually sent the packet OK.
		 */
		if (status & BD_ENET_TX_DEF)
			dev->stats.collisions++;

		/* Free the sk buffer associated with this last transmit */
		dev_consume_skb_any(skb);
		fep->tx_skbuff[fep->skb_dirty] = NULL;
		fep->skb_dirty = (fep->skb_dirty + 1) & TX_RING_MOD_MASK;

		/* Update pointer to next buffer descriptor to be transmitted */
		if (status & BD_ENET_TX_WRAP)
			bdp = fep->tx_bd_base;
		else
			bdp++;

		/* Since we have freed up a buffer, the ring is no longer
		 * full.
		 */
		if (fep->tx_full) {
			fep->tx_full = 0;
			if (netif_queue_stopped(dev))
				netif_wake_queue(dev);
		}
	}
	fep->dirty_tx = bdp;
	spin_unlock_bh(&fep->hw_lock);
}

static int mtip_update_cbd(struct switch_enet_private *fep, struct cbd_t *bdp,
			   int index)
{
	struct page *new_page;

	new_page = page_pool_dev_alloc_pages(fep->page_pool);
	if (unlikely(!new_page))
		return -ENOMEM;

	fep->page[index] = new_page;
	bdp->cbd_bufaddr = page_pool_get_dma_addr(new_page);

	return 0;
}

/* During a receive, the cur_rx points to the current incoming buffer.
 * When we update through the ring, if the next incoming buffer has
 * not been given to the system, we just set the empty indicator,
 * effectively tossing the packet.
 */
static int mtip_switch_rx(struct net_device *dev, int budget, int *port)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	u8 *data, rx_port = MTIP_PORT_FORWARDING_INIT;
	struct switch_enet_private *fep = priv->fep;
	unsigned short status, pkt_len;
	struct net_device *pndev;
	struct ethhdr *eth_hdr;
	int pkt_received = 0;
	struct sk_buff *skb;
	struct cbd_t *bdp;
	struct page *page;
	int index;

	/* First, grab all of the stats for the incoming packet.
	 * These get messed up if we get called due to a busy condition.
	 */
	bdp = fep->cur_rx;

	while (!((status = bdp->cbd_sc) & BD_ENET_RX_EMPTY)) {
		if (pkt_received >= budget)
			break;

		pkt_received++;

		writel(MCF_ESW_IMR_RXF, fep->hwp + ESW_ISR);
		if (!fep->usage_count)
			goto rx_processing_done;

		status ^= BD_ENET_RX_LAST;
		/* Check for errors. */
		if (status & (BD_ENET_RX_LG | BD_ENET_RX_SH | BD_ENET_RX_NO |
			      BD_ENET_RX_CR | BD_ENET_RX_OV | BD_ENET_RX_LAST |
			      BD_ENET_RX_CL)) {
			dev->stats.rx_errors++;
			if (status & BD_ENET_RX_OV) {
				/* FIFO overrun */
				dev->stats.rx_fifo_errors++;
				goto rx_processing_done;
			}
			if (status & (BD_ENET_RX_LG | BD_ENET_RX_SH
				      | BD_ENET_RX_LAST)) {
				/* Frame too long or too short. */
				dev->stats.rx_length_errors++;
				if (status & BD_ENET_RX_LAST)
					netdev_err(dev, "rcv is not +last\n");
			}
			if (status & BD_ENET_RX_CR)	/* CRC Error */
				dev->stats.rx_crc_errors++;

			/* Report late collisions as a frame error. */
			if (status & (BD_ENET_RX_NO | BD_ENET_RX_CL))
				dev->stats.rx_frame_errors++;
			goto rx_processing_done;
		}

		/* Get correct RX page */
		index = bdp - fep->rx_bd_base;
		page = fep->page[index];
		/* Process the incoming frame */
		pkt_len = bdp->cbd_datlen;

		dma_sync_single_for_cpu(&fep->pdev->dev, bdp->cbd_bufaddr,
					pkt_len, DMA_FROM_DEVICE);
		net_prefetch(page_address(page));
		data = page_address(page);

		if (fep->quirks & FEC_QUIRK_SWAP_FRAME)
			swap_buffer(data, pkt_len);

		eth_hdr = (struct ethhdr *)data;
		mtip_atable_get_entry_port_number(fep, eth_hdr->h_source,
						  &rx_port);
		if (rx_port == MTIP_PORT_FORWARDING_INIT)
			mtip_atable_dynamicms_learn_migration(fep,
							      mtip_get_time(),
							      eth_hdr->h_source,
							      &rx_port);

		if ((rx_port == 1 || rx_port == 2) && fep->ndev[rx_port - 1])
			pndev = fep->ndev[rx_port - 1];
		else
			pndev = dev;

		*port = rx_port;

		if (mtip_update_cbd(fep, bdp, index)) {
			pndev->stats.rx_dropped++;
			goto rx_processing_done;
		}

		/* The packet length includes FCS, but we don't want to
		 * include that when passing upstream as it messes up
		 * bridging applications.
		 */
		skb = build_skb(page_address(page), PAGE_SIZE);
		if (unlikely(!skb)) {
			page_pool_recycle_direct(fep->page_pool, page);
			pndev->stats.rx_dropped++;

			netdev_err_once(pndev, "build_skb failed!\n");
			goto rx_processing_done;
		}

		skb_put(skb, pkt_len);      /* Make room */
		skb_mark_for_recycle(skb);
		skb->protocol = eth_type_trans(skb, pndev);
		skb->offload_fwd_mark = fep->br_offload;
		napi_gro_receive(&fep->napi, skb);

		pndev->stats.rx_packets++;
		pndev->stats.rx_bytes += pkt_len;

 rx_processing_done:
		/* Clear the status flags for this buffer */
		status &= ~BD_ENET_RX_STATS;

		/* Mark the buffer empty */
		status |= BD_ENET_RX_EMPTY;
		/* Make sure that updates to the descriptor are performed */
		wmb();
		bdp->cbd_sc = status;

		/* Update BD pointer to next entry */
		if (status & BD_ENET_RX_WRAP)
			bdp = fep->rx_bd_base;
		else
			bdp++;

		/* Doing this here will keep the FEC running while we process
		 * incoming frames.  On a heavily loaded network, we should be
		 * able to keep up at the expense of system resources.
		 */
		writel(MCF_ESW_RDAR_R_DES_ACTIVE, fep->hwp + ESW_RDAR);
	} /* while (!((status = bdp->cbd_sc) & BD_ENET_RX_EMPTY)) */

	fep->cur_rx = bdp;

	return pkt_received;
}

static void mtip_adjust_link(struct net_device *dev)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	struct phy_device *phy_dev;
	int status_change = 0, idx;

	idx = priv->portnum - 1;
	phy_dev = fep->phy_dev[idx];

	/* Duplex link change */
	if (phy_dev->link && fep->full_duplex[idx] != phy_dev->duplex) {
		netif_stop_queue(dev);
		if (idx == 0)
			mtip_switch_restart(dev, phy_dev->duplex,
					    fep->full_duplex[!idx]);
		else
			mtip_switch_restart(dev, fep->full_duplex[!idx],
					    phy_dev->duplex);
		status_change = 1;
	}

	/* Link on or off change */
	if (phy_dev->link != fep->link[idx]) {
		fep->link[idx] = phy_dev->link;
		if (phy_dev->link) {
			netif_stop_queue(dev);
			if (idx == 0)
				mtip_switch_restart(dev, phy_dev->duplex,
						    fep->full_duplex[!idx]);
			else
				mtip_switch_restart(dev, fep->full_duplex[!idx],
						    phy_dev->duplex);
			/* if link becomes up and tx be stopped, start it */
			if (netif_queue_stopped(dev)) {
				netif_start_queue(dev);
				netif_wake_queue(dev);
			}
		}
		status_change = 1;
	}

	if (status_change)
		phy_print_status(phy_dev);
}

static int mtip_mdio_wait(struct switch_enet_private *fep, int bus_id)
{
	void __iomem *enet_addr = fep->enet_addr;
	uint ievent = 0;
	int ret;

	if (bus_id == 1)
		enet_addr += MCF_ESW_ENET_PORT_OFFSET;

	ret = readl_poll_timeout_atomic(enet_addr + MCF_FEC_EIR, ievent,
					ievent & MCF_ENET_MII, 2, 30000);
	if (!ret)
		writel(MCF_ENET_MII, enet_addr + MCF_FEC_EIR);

	return ret;
}

static int mtip_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct switch_enet_private *fep = bus->priv;
	void __iomem *enet_addr = fep->enet_addr;
	int bus_id = 0;
	int ret;

	if(fep->mii_bus[1] && fep->mii_bus[1] == bus)
		bus_id = 1;

	if (bus_id == 1)
		enet_addr += MCF_ESW_ENET_PORT_OFFSET;

	/* start a read op */
	writel(FEC_MMFR_ST | FEC_MMFR_OP_READ |
	       FIELD_PREP(FEC_MMFR_PA_MASK, mii_id) |
	       FIELD_PREP(FEC_MMFR_RA_MASK, regnum) |
	       FEC_MMFR_TA, enet_addr + MCF_FEC_MII_DATA);

	/* wait for end of transfer */
	ret = mtip_mdio_wait(fep, bus_id);
	if (ret) {
		dev_err(&fep->pdev->dev, "MTIP: MDIO (%s:%d) read timeout\n",
			bus->id, mii_id);
		return ret;
	}

	/* return value */
	return FIELD_GET(FEC_MMFR_DATA_MASK,
			 readl(enet_addr + MCF_FEC_MII_DATA));
}

static int mtip_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
			   u16 value)
{
	struct switch_enet_private *fep = bus->priv;
	void __iomem *enet_addr = fep->enet_addr;
	int bus_id = 0;
	int ret;

	if(fep->mii_bus[1] && fep->mii_bus[1] == bus)
		bus_id = 1;

	if (bus_id == 1)
		enet_addr += MCF_ESW_ENET_PORT_OFFSET;

	/* start a write op */
	writel(FEC_MMFR_ST | FEC_MMFR_OP_WRITE |
	       FIELD_PREP(FEC_MMFR_PA_MASK, mii_id) |
	       FIELD_PREP(FEC_MMFR_RA_MASK, regnum) |
	       FEC_MMFR_TA | FIELD_PREP(FEC_MMFR_DATA_MASK, value),
	       enet_addr + MCF_FEC_MII_DATA);

	/* wait for end of transfer */
	ret = mtip_mdio_wait(fep, bus_id);
	if (ret)
		dev_err(&fep->pdev->dev, "MTIP: MDIO (%s:%d) write timeout\n",
			bus->id, mii_id);

	return ret;
}

static int mtip_mii_probe(struct net_device *dev)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	struct phy_device *phy_dev = NULL;
	int port_idx = priv->portnum - 1;

	if (fep->phy_np[port_idx]) {
		phy_dev = of_phy_connect(dev, fep->phy_np[port_idx],
					 &mtip_adjust_link, 0,
					 fep->phy_interface[port_idx]);
		if (!phy_dev) {
			netdev_err(dev, "Unable to connect to phy\n");
			return -ENODEV;
		}
	}

	phy_set_max_speed(phy_dev, 100);
	fep->phy_dev[port_idx] = phy_dev;
	fep->link[port_idx] = 0;
	fep->full_duplex[port_idx] = 0;

	dev_dbg(&dev->dev,
		"MTIP PHY driver [%s] (mii_bus:phy_addr=%s, irq=%d)\n",
		fep->phy_dev[port_idx]->drv->name,
		phydev_name(fep->phy_dev[port_idx]),
		fep->phy_dev[port_idx]->irq);

	return 0;
}

static int mtip_mdiobus_reset(struct mii_bus *bus)
{
	/* Not all PHY devices require gpio reset defined, as some of
	 * them may use the HW (i.e. with a separate reset IC) generated
	 * reset.
	 */
	if (!bus->reset_gpiod)
		return 0;

	gpiod_set_value_cansleep(bus->reset_gpiod, 0);

	/* Extra time to allow:
	 * 1. GPIO RESET pin go high to prevent situation where its value is
	 *    "LOW" as it is NOT configured.
	 * 2. The ENET CLK to stabilize before GPIO RESET is asserted
	 */
	usleep_range(200, 300);

	gpiod_set_value_cansleep(bus->reset_gpiod, 1);
	usleep_range(bus->reset_delay_us, bus->reset_delay_us + 1000);
	gpiod_set_value_cansleep(bus->reset_gpiod, 0);

	if (bus->reset_post_delay_us > 0)
		usleep_range(bus->reset_post_delay_us,
			     bus->reset_post_delay_us + 1000);

	return 0;
}

static int __mtip_mii_init(struct switch_enet_private *fep,
                           struct platform_device *pdev,
                           struct device_node *node, int idx)
{
	void __iomem *enet_addr = fep->enet_addr;
	int err = -ENXIO;

	if (!node)
		return -EINVAL;

	if (idx == 1)
		enet_addr += MCF_ESW_ENET_PORT_OFFSET;

	/* Clear MMFR to avoid to generate MII event by writing MSCR.
	 * MII event generation condition:
	 * - writing MSCR:
	 *      - mmfr[31:0]_not_zero & mscr[7:0]_is_zero &
	 *        mscr_reg_data_in[7:0] != 0
	 * - writing MMFR:
	 *      - mscr[7:0]_not_zero
	 */
	writel(0, fep->hwp + MCF_FEC_MII_DATA);
	/* Clear any pending transaction complete indication */
	writel(MCF_ENET_MII, enet_addr + MCF_FEC_EIR);

	fep->mii_bus[idx] = mdiobus_alloc();
	if (!fep->mii_bus[idx]) {
		err = -ENOMEM;
		goto err_out;
	}

	fep->mii_bus[idx]->name = "mtip_mii_bus";
	fep->mii_bus[idx]->read = mtip_mdio_read;
	fep->mii_bus[idx]->write = mtip_mdio_write;
	fep->mii_bus[idx]->reset = mtip_mdiobus_reset;
	snprintf(fep->mii_bus[idx]->id, MII_BUS_ID_SIZE, "l2sw_mdio%x", idx);
	fep->mii_bus[idx]->priv = fep;
	fep->mii_bus[idx]->parent = &pdev->dev;

	err = of_mdiobus_register(fep->mii_bus[idx], node);
	if (err)
		goto err_out_free_mdiobus;

	return 0;

err_out_free_mdiobus:
	mdiobus_free(fep->mii_bus[idx]);
err_out:
	return err;

}

static int mtip_mii_init(struct switch_enet_private *fep,
			 struct platform_device *pdev)
{
	struct device_node *child;
	int err, i = 0;

	for_each_child_of_node(pdev->dev.of_node, child) {
		if (of_node_name_eq(child, "mdio")) {
			err = __mtip_mii_init(fep, pdev, child, i);
			if (child)
				of_node_put(child);
			if (err) {
				dev_err(&fep->pdev->dev, "MII init failed!\n");
				break;
			}

			dev_dbg(&fep->pdev->dev, "%s: %s[%d]\n", __func__,
			        child->name, i);

			if (++i == SWITCH_EPORT_NUMBER)
				break;
		}
	}

	return err;
}

static void mtip_mii_remove(struct switch_enet_private *fep)
{
	int i;

	for (i = 0; i < SWITCH_EPORT_NUMBER; i++) {
		if (fep->phy_np[i])
			of_node_put(fep->phy_np[i]);

		if (fep->phy_dev[i])
			phy_disconnect(fep->phy_dev[i]);

		if (fep->mii_bus[i]) {
			mdiobus_unregister(fep->mii_bus[i]);
			mdiobus_free(fep->mii_bus[i]);
		}
	}
}

static void mtip_get_drvinfo(struct net_device *dev,
			     struct ethtool_drvinfo *info)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;

	strscpy(info->driver, fep->pdev->dev.driver->name,
		sizeof(info->driver));
	strscpy(info->bus_info, dev_name(&dev->dev),
		sizeof(info->bus_info));
}

static void mtip_free_buffers(struct net_device *dev)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	int i;

	for (i = 0; i < RX_RING_SIZE; i++) {
		page_pool_put_full_page(fep->page_pool,
					fep->page[i], false);
		fep->page[i] = NULL;
	}

	page_pool_destroy(fep->page_pool);
	fep->page_pool = NULL;

	for (i = 0; i < TX_RING_SIZE; i++)
		kfree(fep->tx_bounce[i]);
}

static int mtip_create_page_pool(struct switch_enet_private *fep, int size)
{
	struct page_pool_params pp_params = {
		.order = 0,
		.flags = PP_FLAG_DMA_MAP | PP_FLAG_DMA_SYNC_DEV,
		.pool_size = size,
		.nid = dev_to_node(&fep->pdev->dev),
		.dev = &fep->pdev->dev,
		.dma_dir = DMA_FROM_DEVICE,
		.offset = 0,
		.max_len = MTIP_SWITCH_RX_FRSIZE,
	};
	int ret = 0;

	fep->page_pool = page_pool_create(&pp_params);
	if (IS_ERR(fep->page_pool)) {
		ret = PTR_ERR(fep->page_pool);
		fep->page_pool = NULL;
	}

	return ret;
}

static int mtip_alloc_buffers(struct net_device *dev)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	struct cbd_t *bdp;
	struct page *page;
	int i, ret;

	ret = mtip_create_page_pool(fep, RX_RING_SIZE);
	if (ret < 0) {
		dev_err(&fep->pdev->dev, "Failed to create page pool\n");
		return ret;
	}

	bdp = fep->rx_bd_base;
	for (i = 0; i < RX_RING_SIZE; i++) {
		page = page_pool_dev_alloc_pages(fep->page_pool);
		if (!page) {
			dev_err(&fep->pdev->dev,
				"Failed to allocate page for rx buffer\n");
			goto err;
		}

		bdp->cbd_bufaddr = page_pool_get_dma_addr(page);
		fep->page[i] = page;

		bdp->cbd_sc = BD_ENET_RX_EMPTY;
		bdp++;
	}

	mtip_set_last_buf_to_wrap(bdp);

	bdp = fep->tx_bd_base;
	for (i = 0; i < TX_RING_SIZE; i++) {
		fep->tx_bounce[i] = kmalloc(MTIP_SWITCH_TX_FRSIZE, GFP_KERNEL);
		if (!fep->tx_bounce[i])
			goto err;

		bdp->cbd_sc = 0;
		bdp->cbd_bufaddr = 0;
		bdp++;
	}

	mtip_set_last_buf_to_wrap(bdp);

	return 0;

 err:
	mtip_free_buffers(dev);
	return -ENOMEM;
}

static int mtip_rx_napi(struct napi_struct *napi, int budget)
{
	struct mtip_ndev_priv *priv = netdev_priv(napi->dev);
	struct switch_enet_private *fep = priv->fep;
	int pkts, port;

	pkts = mtip_switch_rx(napi->dev, budget, &port);
	if (pkts == -ENOMEM) {
		napi_complete(napi);
		/* Set default interrupt mask for L2 switch */
		writel(MCF_ESW_IMR_RXF | MCF_ESW_IMR_TXF,
		       fep->hwp + ESW_IMR);
		return 0;
	}

	if ((port == 1 || port == 2) && fep->ndev[port - 1])
		mtip_switch_tx(fep->ndev[port - 1]);
	else
		mtip_switch_tx(napi->dev);

	if (pkts < budget) {
		if (likely(napi_complete_done(napi, pkts)))
			/* Set default interrupt mask for L2 switch */
			writel(MCF_ESW_IMR_RXF | MCF_ESW_IMR_TXF,
			       fep->hwp + ESW_IMR);
	}
	return pkts;
}

static int mtip_open(struct net_device *dev)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	int ret, port_idx = priv->portnum - 1;

	if (fep->usage_count == 0) {
		ret = clk_enable(fep->clk_ipg);
		if (ret) {
			dev_err(&fep->pdev->dev,
				"Cannot enable switch IPG clock\n");
			return ret;
		}

		netif_napi_add(dev, &fep->napi, mtip_rx_napi);

		ret = mtip_alloc_buffers(dev);
		if (ret)
			goto mtip_alloc_buffers_err;
	}

	fep->link[port_idx] = 0;

	/* Probe and connect to PHY when open the interface, if already
	 * NOT done in the switch driver probe (or when the device is
	 * re-opened).
	 */
	ret = mtip_mii_probe(dev);
	if (ret)
		goto mtip_mii_probe_err;

	phy_start(fep->phy_dev[port_idx]);

	if (fep->usage_count == 0) {
		napi_enable(&fep->napi);
		mtip_switch_restart(dev, 1, 1);

		netif_start_queue(dev);
	}

	fep->usage_count++;
	return 0;

 mtip_mii_probe_err:
	mtip_free_buffers(dev);
 mtip_alloc_buffers_err:
	if (fep->usage_count == 0) {
		netif_napi_del(&fep->napi);
		clk_disable(fep->clk_ipg);
	}
	return ret;
};

static int mtip_close(struct net_device *dev)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	int idx = priv->portnum - 1;

	fep->link[idx] = 0;

	if (fep->phy_dev[idx]) {
		phy_stop(fep->phy_dev[idx]);
		netif_stop_queue(dev);
		phy_disconnect(fep->phy_dev[idx]);
		fep->phy_dev[idx] = NULL;
	}

	if (fep->usage_count == 1) {
		napi_disable(&fep->napi);
		netif_napi_del(&fep->napi);
		mtip_free_buffers(dev);
		clk_disable(fep->clk_ipg);
	}

	fep->usage_count--;

	return 0;
}

#define FEC_HASH_BITS	6		/* #bits in hash */
static void mtip_set_multicast_list(struct net_device *dev)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	unsigned int hash_high = 0, hash_low = 0, crc;
	struct switch_enet_private *fep = priv->fep;
	void __iomem *enet_addr = fep->enet_addr;
	struct netdev_hw_addr *ha;
	unsigned char hash;

	if (priv->portnum == 2)
		enet_addr += MCF_ESW_ENET_PORT_OFFSET;

	if (dev->flags & IFF_PROMISC) {
		/* Promisc mode is required for switch - it is
		 * already enabled during driver's probe.
		 */
		dev_dbg(&dev->dev, "%s: IFF_PROMISC\n", __func__);
		return;
	}

	if (dev->flags & IFF_ALLMULTI) {
		dev_dbg(&dev->dev, "%s: IFF_ALLMULTI\n", __func__);

		/* Allow all multicast addresses */
		writel(0xFFFFFFFF, enet_addr + MCF_FEC_GRP_HASH_TABLE_HIGH);
		writel(0xFFFFFFFF, enet_addr + MCF_FEC_GRP_HASH_TABLE_LOW);

		return;
	}

	netdev_for_each_mc_addr(ha, dev) {
		/* Calculate crc32 value of mac address */
		crc = ether_crc_le(dev->addr_len, ha->addr);

		/* Only upper 6 bits (FEC_HASH_BITS) are used
		 * which point to specific bit in the hash registers
		 */
		hash = (crc >> (32 - FEC_HASH_BITS)) & 0x3F;

		if (hash > 31)
			hash_high |= 1 << (hash - 32);
		else
			hash_low |= 1 << hash;
	}

	writel(hash_high, enet_addr + MCF_FEC_GRP_HASH_TABLE_HIGH);
	writel(hash_low, enet_addr + MCF_FEC_GRP_HASH_TABLE_LOW);
}

static int mtip_set_mac_address(struct net_device *dev, void *p)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	void __iomem *enet_addr = fep->enet_addr;
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;
	eth_hw_addr_set(dev, addr->sa_data);

	if (priv->portnum == 2)
		enet_addr += MCF_ESW_ENET_PORT_OFFSET;

	writel(dev->dev_addr[3] | (dev->dev_addr[2] << 8) |
	       (dev->dev_addr[1] << 16) | (dev->dev_addr[0] << 24),
	       enet_addr + MCF_FEC_PALR);
	writel((dev->dev_addr[5] << 16) | (dev->dev_addr[4] << 24),
	       enet_addr + MCF_FEC_PAUR);

	return mtip_update_atable_static((unsigned char *)dev->dev_addr,
					 7, 7, fep);
}

static int mtip_get_port_parent_id(struct net_device *ndev,
				   struct netdev_phys_item_id *ppid)
{
	struct mtip_ndev_priv *priv = netdev_priv(ndev);
	struct switch_enet_private *fep = priv->fep;

	ppid->id_len = sizeof(fep->mac[0]);
	memcpy(&ppid->id, &fep->mac[0], ppid->id_len);

	return 0;
}

static const struct ethtool_ops mtip_ethtool_ops = {
	.get_link_ksettings     = phy_ethtool_get_link_ksettings,
	.set_link_ksettings     = phy_ethtool_set_link_ksettings,
	.get_drvinfo            = mtip_get_drvinfo,
	.get_link               = ethtool_op_get_link,
	.get_ts_info		= ethtool_op_get_ts_info,
};

static const struct net_device_ops mtip_netdev_ops = {
	.ndo_open		= mtip_open,
	.ndo_stop		= mtip_close,
	.ndo_start_xmit	= mtip_start_xmit,
	.ndo_set_rx_mode	= mtip_set_multicast_list,
	.ndo_tx_timeout	= mtip_timeout,
	.ndo_set_mac_address	= mtip_set_mac_address,
	.ndo_get_port_parent_id	= mtip_get_port_parent_id,
	.ndo_fdb_dump		= mtip_port_fdb_dump,
};

bool mtip_is_switch_netdev_port(const struct net_device *ndev)
{
	return ndev->netdev_ops == &mtip_netdev_ops;
}

static int mtip_switch_dma_init(struct switch_enet_private *fep)
{
	struct cbd_t *bdp, *cbd_base;
	int ret, i;

	/* Check mask of the streaming and coherent API */
	ret = dma_set_mask_and_coherent(&fep->pdev->dev, DMA_BIT_MASK(32));
	if (ret < 0) {
		dev_err(&fep->pdev->dev, "No suitable DMA available\n");
		return ret;
	}

	/* Allocate memory for buffer descriptors */
	cbd_base = dma_alloc_coherent(&fep->pdev->dev, PAGE_SIZE, &fep->bd_dma,
				      GFP_KERNEL);
	if (!cbd_base)
		return -ENOMEM;

	/* Set receive and transmit descriptor base */
	fep->rx_bd_base = cbd_base;
	fep->tx_bd_base = cbd_base + RX_RING_SIZE;

	/* Initialize the receive buffer descriptors */
	bdp = fep->rx_bd_base;
	for (i = 0; i < RX_RING_SIZE; i++) {
		bdp->cbd_sc = 0;
		bdp++;
	}

	mtip_set_last_buf_to_wrap(bdp);
	/* ...and the same for transmit */
	bdp = fep->tx_bd_base;
	for (i = 0; i < TX_RING_SIZE; i++) {
		/* Initialize the BD for every fragment in the page */
		bdp->cbd_sc = 0;
		bdp->cbd_bufaddr = 0;
		bdp++;
	}

	mtip_set_last_buf_to_wrap(bdp);
	return 0;
}

static void mtip_ndev_cleanup(struct switch_enet_private *fep)
{
	struct mtip_ndev_priv *priv;
	int i;

	for (i = 0; i < SWITCH_EPORT_NUMBER; i++) {
		if (fep->ndev[i]) {
			priv = netdev_priv(fep->ndev[i]);
			cancel_work_sync(&priv->tx_timeout_work);

			unregister_netdev(fep->ndev[i]);
			free_netdev(fep->ndev[i]);
			fep->ndev[i] = NULL;
		}
	}
}

static int mtip_ndev_init(struct switch_enet_private *fep,
			  struct platform_device *pdev)
{
	struct mtip_ndev_priv *priv;
	int i, ret = 0;

	for (i = 0; i < SWITCH_EPORT_NUMBER; i++) {
		fep->ndev[i] = alloc_netdev(sizeof(struct mtip_ndev_priv),
					    fep->ndev_name[i], NET_NAME_USER,
					    ether_setup);
		if (!fep->ndev[i]) {
			ret = -ENOMEM;
			goto cleanup_created_ndev;
		}

		fep->ndev[i]->ethtool_ops = &mtip_ethtool_ops;
		fep->ndev[i]->netdev_ops = &mtip_netdev_ops;
		SET_NETDEV_DEV(fep->ndev[i], &pdev->dev);

		priv = netdev_priv(fep->ndev[i]);
		priv->dev = fep->ndev[i];
		priv->fep = fep;
		priv->portnum = i + 1;
		fep->ndev[i]->irq = fep->irq;

		mtip_setup_mac(fep->ndev[i]);

		ret = register_netdev(fep->ndev[i]);
		if (ret) {
			dev_err(&fep->ndev[i]->dev,
				"%s: ndev %s register err: %d\n", __func__,
				fep->ndev[i]->name, ret);
			free_netdev(fep->ndev[i]);
			fep->ndev[i] = NULL;
			goto cleanup_created_ndev;
		}

		INIT_WORK(&priv->tx_timeout_work, mtip_timeout_work);

		dev_dbg(&fep->ndev[i]->dev, "%s: MTIP eth L2 switch %pM\n",
			fep->ndev[i]->name, fep->ndev[i]->dev_addr);
	}

	return 0;

 cleanup_created_ndev:
	if (i == SWITCH_EPORT_NUMBER - 1)
		mtip_ndev_cleanup(fep);

	return ret;
}

static int mtip_parse_of(struct switch_enet_private *fep,
			 struct device_node *np)
{
	struct device_node *p;
	unsigned int port_num;
	int ret = 0;

	p = of_get_child_by_name(np, "ethernet-ports");

	for_each_available_child_of_node_scoped(p, port) {
		if (of_property_read_u32(port, "reg", &port_num))
			continue;

		if (port_num > SWITCH_EPORT_NUMBER) {
			dev_err(&fep->pdev->dev,
				"%s: The switch supports up to %d ports!\n",
				__func__, SWITCH_EPORT_NUMBER);
			goto of_get_err;
		}

		fep->n_ports = port_num;
		ret = of_get_mac_address(port, &fep->mac[port_num - 1][0]);
		if (ret)
			dev_dbg(&fep->pdev->dev,
				"of_get_mac_address(%pOF) failed (%d)!\n",
				port, ret);

		ret = of_property_read_string(port, "label",
					      &fep->ndev_name[port_num - 1]);
		if (ret < 0) {
			dev_err(&fep->pdev->dev,
				"%s: Cannot get ethernet port name (%d)!\n",
				__func__, ret);
			goto of_get_err;
		}

		ret = of_get_phy_mode(port, &fep->phy_interface[port_num - 1]);
		if (ret < 0) {
			dev_err(&fep->pdev->dev,
				"%s: Cannot get PHY mode (%d)!\n", __func__,
				ret);
			goto of_get_err;
		}

		fep->phy_np[port_num - 1] = of_parse_phandle(port,
							     "phy-handle", 0);
	}

 of_get_err:
	of_node_put(p);

	return ret;
}

static const struct mtip_devinfo mtip_imx28_l2switch_info = {
	.quirks = FEC_QUIRK_BUG_CAPTURE | FEC_QUIRK_SINGLE_MDIO |
		  FEC_QUIRK_SWAP_FRAME,
	.clk.c = imx28_clocks,
	.clk.size = ARRAY_SIZE(imx28_clocks),
};

static const struct mtip_devinfo mtip_vf610_l2switch_info = {
	.quirks = 0,
	.clk.c = vf_clocks,
	.clk.size = ARRAY_SIZE(vf_clocks),
};

static const struct of_device_id mtipl2_of_match[] = {
	{ .compatible = "nxp,imx28-mtip-switch",
	  .data = &mtip_imx28_l2switch_info},
	{ .compatible = "nxp,vf610-mtip-switch",
	  .data = &mtip_vf610_l2switch_info},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtipl2_of_match);

static int mtip_sw_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct mtip_devinfo *dev_info;
	struct switch_enet_private *fep;
	int ret;

	fep = devm_kzalloc(&pdev->dev, sizeof(*fep), GFP_KERNEL);
	if (!fep)
		return -ENOMEM;

	dev_info = of_device_get_match_data(&pdev->dev);
	if (dev_info) {
		fep->quirks = dev_info->quirks;
		fep->clks = dev_info->clk.c;
		fep->clk_num = dev_info->clk.size;
	}

	fep->pdev = pdev;
	platform_set_drvdata(pdev, fep);

	fep->hwp = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(fep->hwp))
		return PTR_ERR(fep->hwp);

	fep->enet_addr = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(fep->enet_addr))
		return PTR_ERR(fep->enet_addr);

	fep->irq = platform_get_irq_byname(pdev, "enet_switch");
	if (fep->irq < 0)
		return fep->irq;

	ret = mtip_parse_of(fep, np);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret,
				     "OF parse error\n");

	/* Create an Ethernet device instance.
	 * The switch lookup address memory starts at 0x800FC000
	 */
	fep->hwp_enet = fep->enet_addr;
	fep->hwentry = (struct mtip_addr_table __iomem *)
		(fep->hwp + MCF_ESW_LOOKUP_MEM_OFFSET);

	ret = devm_regulator_get_enable_optional(&pdev->dev, "phy");
	if (ret < 0 && ret != -ENODEV)
		return dev_err_probe(&pdev->dev, ret,
				     "Unable to get and enable 'phy'\n");

	fep->clk_ipg = devm_clk_get_enabled(&pdev->dev, "ipg");
	if (IS_ERR(fep->clk_ipg))
		return dev_err_probe(&pdev->dev, PTR_ERR(fep->clk_ipg),
				     "Unable to acquire 'ipg' clock\n");

	ret = devm_clk_bulk_get(&pdev->dev, fep->clk_num, fep->clks);
	if (ret)
		return ret;

	ret = clk_bulk_prepare_enable(fep->clk_num, fep->clks);
	if (ret)
		return ret;

	fep->clk_ptp = devm_clk_get_optional_enabled(&pdev->dev, "ptp");
	if (IS_ERR(fep->clk_ptp))
		return dev_err_probe(&pdev->dev, PTR_ERR(fep->clk_ptp),
				     "Unable to acquire 'ptp' clock\n");

	pr_err("%s: ESW IP REV: 0x%x\n", __func__, readl(fep->hwp + ESW_REVISION));

	/* setup MII interface for external switch ports */
	mtip_enet_init(fep, 1);
	mtip_enet_init(fep, 2);

	spin_lock_init(&fep->learn_lock);
	spin_lock_init(&fep->hw_lock);

	ret = devm_request_irq(&pdev->dev, fep->irq, mtip_interrupt, 0,
			       dev_name(&pdev->dev), fep);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Could not alloc IRQ\n");

	ret = mtip_bridge_register_notifiers(fep);
	if (ret)
		goto disable_clk_bulk;

	ret = mtip_switchdev_register_notifiers(fep);
	if (ret)
		goto unregister_bridge_notifiers;

	ret = mtip_switch_dma_init(fep);
	if (ret) {
		dev_err(&pdev->dev, "%s: ethernet switch init fail (%d)!\n",
			__func__, ret);
		goto unregister_switchdev_notifiers;
	}

	ret = mtip_mii_init(fep, pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s: Cannot init phy bus (%d)!\n", __func__,
			ret);
		goto dma_free_coherent_memory;
	}

	ret = mtip_ndev_init(fep, pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s: Failed to create virtual ndev (%d)\n",
			__func__, ret);
		goto mdiobus_free_memory;
	}

	/* setup timer for learning aging function */
	timer_setup(&fep->timer_mgnt, mtip_mgnt_timer, 0);
	mod_timer(&fep->timer_mgnt,
		  jiffies + msecs_to_jiffies(LEARNING_AGING_INTERVAL));

	return 0;

 mdiobus_free_memory:
	mdiobus_free(fep->mii_bus);
 dma_free_coherent_memory:
	dma_free_coherent(&fep->pdev->dev, PAGE_SIZE, fep->rx_bd_base,
			  fep->bd_dma);
	fep->rx_bd_base = NULL;
	fep->tx_bd_base = NULL;
 unregister_switchdev_notifiers:
	mtip_switchdev_unregister_notifiers(fep);
 unregister_bridge_notifiers:
	mtip_bridge_unregister_notifiers(fep);
 disable_clk_bulk:
        clk_bulk_disable_unprepare(fep->clk_num, fep->clks);

	return ret;
}

static int mtip_sw_remove(struct platform_device *pdev)
{
	struct switch_enet_private *fep = platform_get_drvdata(pdev);

	mtip_switchdev_unregister_notifiers(fep);
	mtip_bridge_unregister_notifiers(fep);

	mtip_ndev_cleanup(fep);

	mtip_mii_remove(fep);

	timer_delete_sync(&fep->timer_mgnt);
	clk_bulk_disable_unprepare(fep->clk_num, fep->clks);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver mtipl2plat_driver = {
	.driver         = {
		.name   = "mtipl2sw",
		.of_match_table = mtipl2_of_match,
		.suppress_bind_attrs = true,
	},
	.probe          = mtip_sw_probe,
	.remove         = mtip_sw_remove,
};

module_platform_driver(mtipl2plat_driver);

MODULE_AUTHOR("Lukasz Majewski <lukma@denx.de>");
MODULE_DESCRIPTION("Driver for MTIP L2 on SOC switch");
MODULE_LICENSE("GPL");
