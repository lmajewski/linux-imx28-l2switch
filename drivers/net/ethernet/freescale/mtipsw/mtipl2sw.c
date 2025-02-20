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

#include "mtipl2sw.h"

static void swap_buffer(void *bufaddr, int len)
{
	int i;
	unsigned int *buf = bufaddr;

	for (i = 0; i < len; i += 4, buf++)
		swab32s(buf);
}

struct mtip_devinfo {
	u32 quirks;
};

static void mtip_enet_init(struct switch_enet_private *fep, int port)
{
	void __iomem *enet_addr = fep->enet_addr;
	u32 mii_speed, holdtime, reg;

	if (port == 2)
		enet_addr += MCF_ESW_ENET_PORT_OFFSET;

	reg = MCF_FEC_RCR_PROM | MCF_FEC_RCR_MII_MODE |
		MCF_FEC_RCR_MAX_FL(1522);

	if (fep->phy_interface[port - 1]  == PHY_INTERFACE_MODE_RMII)
		reg |= MCF_FEC_RCR_RMII_MODE;

	writel(reg, enet_addr + MCF_FEC_RCR);

	writel(MCF_FEC_TCR_FDEN, enet_addr + MCF_FEC_TCR);
	writel(MCF_FEC_ECR_ETHER_EN, enet_addr + MCF_FEC_ECR);

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





				*port = AT_EXTRACT_PORT(read_hi);
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
			mtip_write_atable(fep, entry, write_lo, write_hi);
			return 0;
		} else if (!(read_hi & (1 << 16))) {
			/* Fill this empty slot (valid bit zero),
			 * assuming no holes in the block
			 */
			mtip_write_atable(fep, entry, write_lo, write_hi);
			fep->at_curr_entries++;
			return 0;
		}
	}

	/* No space available for this static entry */
	return -ENOSPC;
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
			/* statistics (we do it between writing
			 *  .hi an .lo due to
			 * hardware limitation...
			 */
			fep->at_curr_entries++;
			/* newly inserted */

			return true;
		}
	}

	/* No more entry available in block overwrite oldest */
	timeold = 0;
	indexold = 0;
	for (entry = block_index; entry < index_end; entry++) {
		mtip_read_atable(fep, entry, &read_lo, &read_hi);
		time = AT_EXTRACT_TIMESTAMP(read_hi);
		dev_dbg(&fep->pdev->dev, "%s : time %x currtime %x\n",
			__func__, time, curr_time);
		time = TIMEDELTA(curr_time, time);
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
	u32 rx_mac_lo, rx_mac_hi;
	unsigned long flags;
	int index;

	spin_lock_irqsave(&fep->learn_lock, flags);

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

	spin_unlock_irqrestore(&fep->learn_lock, flags);
}

static void mtip_aging_timer(struct timer_list *t)
{
	struct switch_enet_private *fep = from_timer(fep, t, timer_aging);

	fep->curr_time = mtip_timeincrement(fep->curr_time);

	mod_timer(&fep->timer_aging,
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
	unsigned short	status;
	unsigned long flags;
	struct cbd_t *bdp;
	void *bufaddr;

	spin_lock_irqsave(&fep->hw_lock, flags);

	if (!fep->link[0] && !fep->link[1]) {
		/* Link is down or autonegotiation is in progress. */
		netif_stop_queue(dev);
		spin_unlock_irqrestore(&fep->hw_lock, flags);
		return NETDEV_TX_BUSY;
	}

	/* Fill in a Tx ring entry */
	bdp = fep->cur_tx;

	status = bdp->cbd_sc;

	if (status & BD_ENET_TX_READY) {
		/* All transmit buffers are full. Bail out.
		 * This should not happen, since dev->tbusy should be set.
		 */
		dev_err(&fep->pdev->dev, "%s: tx queue full!.\n", dev->name);
		spin_unlock_irqrestore(&fep->hw_lock, flags);
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
	if ((unsigned long)bufaddr & MTIP_ALIGNMENT) {
		unsigned int index;

		index = bdp - fep->tx_bd_base;
		memcpy(fep->tx_bounce[index],
		       (void *)skb->data, skb->len);
		bufaddr = fep->tx_bounce[index];
	}

	if (fep->quirks & FEC_QUIRK_SWAP_FRAME)
		swap_buffer(bufaddr, skb->len);

	/* Save skb pointer. */
	fep->tx_skbuff[fep->skb_cur] = skb;

	dev->stats.tx_bytes += skb->len;
	fep->skb_cur = (fep->skb_cur + 1) & TX_RING_MOD_MASK;

	/* Push the data cache so the CPM does not get stale memory
	 * data.
	 */
	bdp->cbd_bufaddr = dma_map_single(&fep->pdev->dev, bufaddr,
					  MTIP_SWITCH_TX_FRSIZE,
					  DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(&fep->pdev->dev, bdp->cbd_bufaddr))) {
		dev_err(&fep->pdev->dev,
			"Failed to map descriptor tx buffer\n");
		dev->stats.tx_errors++;
		dev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		goto err;
	}

	/* Send it on its way.  Tell FEC it's ready, interrupt when done,
	 * it's the last BD of the frame, and to put the CRC on the end.
	 */

	status |= (BD_ENET_TX_READY | BD_ENET_TX_INTR
			| BD_ENET_TX_LAST | BD_ENET_TX_TC);
	bdp->cbd_sc = status;

	netif_trans_update(dev);
	skb_tx_timestamp(skb);

	/* For port separation - force sending via specified port */
	if (!fep->br_offload && port != 0)
		mtip_forced_forward(fep, port, 1);

	/* Trigger transmission start */
	writel(MCF_ESW_TDAR_X_DES_ACTIVE, fep->hwp + ESW_TDAR);

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
	spin_unlock_irqrestore(&fep->hw_lock, flags);

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
	writel(MCF_FEC_ECR_MAGIC_ENA, enet_addr + MCF_FEC_ECR);

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

static void mtip_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	struct cbd_t *bdp;
	int i;

	dev->stats.tx_errors++;

	if (IS_ENABLED(CONFIG_SWITCH_DEBUG)) {
		dev_info(&dev->dev, "%s: transmit timed out.\n", dev->name);
		dev_info(&dev->dev,
			 "Ring data: cur_tx %lx%s, dirty_tx %lx cur_rx: %lx\n",
			 (unsigned long)fep->cur_tx,
			 fep->tx_full ? " (full)" : "",
			 (unsigned long)fep->dirty_tx,
			 (unsigned long)fep->cur_rx);

		bdp = fep->tx_bd_base;
		dev_info(&dev->dev, " tx: %u buffers\n", TX_RING_SIZE);
		for (i = 0; i < TX_RING_SIZE; i++) {
			dev_info(&dev->dev, "  %08lx: %04x %04x %08x\n",
				 (kernel_ulong_t)bdp, bdp->cbd_sc,
				 bdp->cbd_datlen, (int)bdp->cbd_bufaddr);
			bdp++;
		}

		bdp = fep->rx_bd_base;
		dev_info(&dev->dev, " rx: %lu buffers\n",
			 (unsigned long)RX_RING_SIZE);
		for (i = 0 ; i < RX_RING_SIZE; i++) {
			dev_info(&dev->dev, "  %08lx: %04x %04x %08x\n",
				 (kernel_ulong_t)bdp,
				 bdp->cbd_sc, bdp->cbd_datlen,
				 (int)bdp->cbd_bufaddr);
			bdp++;
		}
	}

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
	unsigned long flags;
	struct cbd_t *bdp;

	spin_lock_irqsave(&fep->hw_lock, flags);
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
			dev_err(&fep->pdev->dev,
				"Enet xmit interrupt and TX_READY.\n");

		/* Deferred means some collisions occurred during transmit,
		 * but we eventually sent the packet OK.
		 */
		if (status & BD_ENET_TX_DEF)
			dev->stats.collisions++;

		/* Free the sk buffer associated with this last transmit */
		dev_consume_skb_irq(skb);
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
	spin_unlock_irqrestore(&fep->hw_lock, flags);
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
	unsigned long flags;
	struct cbd_t *bdp;

	spin_lock_irqsave(&fep->hw_lock, flags);

	/* First, grab all of the stats for the incoming packet.
	 * These get messed up if we get called due to a busy condition.
	 */
	bdp = fep->cur_rx;

	while (!((status = bdp->cbd_sc) & BD_ENET_RX_EMPTY)) {
		if (pkt_received >= budget)
			break;

		pkt_received++;
		/* Since we have allocated space to hold a complete frame,
		 * the last indicator should be set.
		 */
		if ((status & BD_ENET_RX_LAST) == 0)
			dev_warn_ratelimited(&dev->dev,
					     "SWITCH ENET: rcv is not +last\n");

		if (!fep->usage_count)
			goto rx_processing_done;

		/* Check for errors. */
		if (status & (BD_ENET_RX_LG | BD_ENET_RX_SH | BD_ENET_RX_NO |
			      BD_ENET_RX_CR | BD_ENET_RX_OV)) {
			dev->stats.rx_errors++;
			if (status & (BD_ENET_RX_LG | BD_ENET_RX_SH)) {
				/* Frame too long or too short. */
				dev->stats.rx_length_errors++;
			}
			if (status & BD_ENET_RX_NO)	/* Frame alignment */
				dev->stats.rx_frame_errors++;
			if (status & BD_ENET_RX_CR)	/* CRC Error */
				dev->stats.rx_crc_errors++;
			if (status & BD_ENET_RX_OV)	/* FIFO overrun */
				dev->stats.rx_fifo_errors++;
		}

		/* Report late collisions as a frame error.
		 * On this error, the BD is closed, but we don't know what we
		 * have in the buffer.  So, just drop this frame on the floor.
		 */
		if (status & BD_ENET_RX_CL) {
			dev->stats.rx_errors++;
			dev->stats.rx_frame_errors++;
			goto rx_processing_done;
		}

		/* Process the incoming frame */
		pkt_len = bdp->cbd_datlen;
		data = (__u8 *)__va(bdp->cbd_bufaddr);

		dma_unmap_single(&fep->pdev->dev, bdp->cbd_bufaddr,
				 bdp->cbd_datlen, DMA_FROM_DEVICE);

		if (fep->quirks & FEC_QUIRK_SWAP_FRAME)
			swap_buffer(data, pkt_len);

		if (data) {
			eth_hdr = (struct ethhdr *)data;
			mtip_atable_get_entry_port_number(fep,
							  eth_hdr->h_source,
							  &rx_port);
			if (rx_port == MTIP_PORT_FORWARDING_INIT)
				mtip_atable_dynamicms_learn_migration(fep,
								      fep->curr_time,
								      eth_hdr->h_source,
								      &rx_port);
		}

		if (!fep->br_offload && (rx_port == 1 || rx_port == 2))
			pndev = fep->ndev[rx_port - 1];
		else
			pndev = dev;

		*port = rx_port;
		pndev->stats.rx_packets++;
		pndev->stats.rx_bytes += pkt_len;

		/* This does 16 byte alignment, exactly what we need.
		 * The packet length includes FCS, but we don't want to
		 * include that when passing upstream as it messes up
		 * bridging applications.
		 */
		skb = netdev_alloc_skb(pndev, pkt_len + NET_IP_ALIGN);
		if (unlikely(!skb)) {
			dev_dbg(&fep->pdev->dev,
				"%s: Memory squeeze, dropping packet.\n",
				pndev->name);
			pndev->stats.rx_dropped++;
			goto err_mem;
		} else {
			skb_reserve(skb, NET_IP_ALIGN);
			skb_put(skb, pkt_len);      /* Make room */
			skb_copy_to_linear_data(skb, data, pkt_len);
			skb->protocol = eth_type_trans(skb, pndev);
			napi_gro_receive(&fep->napi, skb);
		}

		bdp->cbd_bufaddr = dma_map_single(&fep->pdev->dev, data,
						  bdp->cbd_datlen,
						  DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(&fep->pdev->dev,
					       bdp->cbd_bufaddr))) {
			dev_err(&fep->pdev->dev,
				"Failed to map descriptor rx buffer\n");
			pndev->stats.rx_errors++;
			pndev->stats.rx_dropped++;
			dev_kfree_skb_any(skb);
			goto err_mem;
		}

 rx_processing_done:
		/* Clear the status flags for this buffer */
		status &= ~BD_ENET_RX_STATS;

		/* Mark the buffer empty */
		status |= BD_ENET_RX_EMPTY;
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
	spin_unlock_irqrestore(&fep->hw_lock, flags);

	return pkt_received;

 err_mem:
	spin_unlock_irqrestore(&fep->hw_lock, flags);
	return -ENOMEM;
}

static void mtip_adjust_link(struct net_device *dev)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	struct phy_device *phy_dev;
	int status_change = 0, idx;
	unsigned long flags;

	spin_lock_irqsave(&fep->hw_lock, flags);

	idx = priv->portnum - 1;
	phy_dev = fep->phy_dev[idx];

	/* Duplex link change */
	if (phy_dev->link && fep->full_duplex[idx] != phy_dev->duplex) {
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

	spin_unlock_irqrestore(&fep->hw_lock, flags);

	if (status_change)
		phy_print_status(phy_dev);
}

static int mtip_mdio_wait(struct switch_enet_private *fep)
{
	uint ievent = 0;
	int ret;

	ret = readl_poll_timeout_atomic(fep->enet_addr + MCF_FEC_EIR, ievent,
					ievent & MCF_ENET_MII, 2, 30000);
	if (!ret)
		writel(MCF_ENET_MII, fep->enet_addr + MCF_FEC_EIR);

	return ret;
}

static int mtip_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct switch_enet_private *fep = bus->priv;
	int ret;

	/* start a read op */
	writel(FEC_MMFR_ST | FEC_MMFR_OP_READ |
		FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(regnum) |
		FEC_MMFR_TA, fep->enet_addr + MCF_FEC_MII_DATA);

	/* wait for end of transfer */
	ret = mtip_mdio_wait(fep);
	if (ret) {
		dev_err(&fep->pdev->dev, "MTIP: MDIO (%s:%d) read timeout\n",
			bus->id, mii_id);
		return ret;
	}

	/* return value */
	return FEC_MMFR_DATA(readl(fep->enet_addr + MCF_FEC_MII_DATA));
}

static int mtip_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
			   u16 value)
{
	struct switch_enet_private *fep = bus->priv;
	int ret;

	/* start a write op */
	writel(FEC_MMFR_ST | FEC_MMFR_OP_WRITE |
	       FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(regnum) |
	       FEC_MMFR_TA | FEC_MMFR_DATA(value),
	       fep->enet_addr + MCF_FEC_MII_DATA);

	/* wait for end of transfer */
	ret = mtip_mdio_wait(fep);
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
	if (!bus || !bus->reset_gpiod) {
		dev_err(&bus->dev, "Reset GPIO pin not provided!\n");
		return -EINVAL;
	}

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

static int mtip_mii_init(struct switch_enet_private *fep,
			 struct platform_device *pdev)
{
	struct device_node *node;
	int err = -ENXIO;

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
	writel(MCF_ENET_MII, fep->enet_addr + MCF_FEC_EIR);

	fep->mii_bus = mdiobus_alloc();
	if (!fep->mii_bus) {
		err = -ENOMEM;
		goto err_out;
	}

	fep->mii_bus->name = "mtip_mii_bus";
	fep->mii_bus->read = mtip_mdio_read;
	fep->mii_bus->write = mtip_mdio_write;
	fep->mii_bus->reset = mtip_mdiobus_reset;
	snprintf(fep->mii_bus->id, MII_BUS_ID_SIZE, "%x", 0);
	fep->mii_bus->priv = fep;
	fep->mii_bus->parent = &pdev->dev;

	node = of_get_child_by_name(pdev->dev.of_node, "mdio");
	if (node)
		dev_err(&fep->pdev->dev, "%s: PHY name: %s\n",
			__func__, node->name);

	err = of_mdiobus_register(fep->mii_bus, node);
	if (node)
		of_node_put(node);
	if (err)
		goto err_out_free_mdiobus;

	return 0;

err_out_free_mdiobus:
	mdiobus_free(fep->mii_bus);
err_out:
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
	}

	mdiobus_unregister(fep->mii_bus);
	mdiobus_free(fep->mii_bus);
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
	struct sk_buff *skb;
	struct cbd_t *bdp;
	int i;

	bdp = fep->rx_bd_base;
	for (i = 0; i < RX_RING_SIZE; i++) {
		skb = fep->rx_skbuff[i];

		if (bdp->cbd_bufaddr)
			dma_unmap_single(&fep->pdev->dev, bdp->cbd_bufaddr,
					 MTIP_SWITCH_RX_FRSIZE,
					 DMA_FROM_DEVICE);
		if (skb)
			dev_kfree_skb(skb);
		bdp++;
	}

	bdp = fep->tx_bd_base;
	for (i = 0; i < TX_RING_SIZE; i++)
		kfree(fep->tx_bounce[i]);
}

static int mtip_alloc_buffers(struct net_device *dev)
{
	struct mtip_ndev_priv *priv = netdev_priv(dev);
	struct switch_enet_private *fep = priv->fep;
	struct sk_buff *skb;
	struct cbd_t *bdp;
	int i;

	bdp = fep->rx_bd_base;
	for (i = 0; i < RX_RING_SIZE; i++) {
		skb = netdev_alloc_skb(dev, MTIP_SWITCH_RX_FRSIZE);
		if (!skb)
			goto err;

		fep->rx_skbuff[i] = skb;

		bdp->cbd_bufaddr = dma_map_single(&fep->pdev->dev, skb->data,
						  MTIP_SWITCH_RX_FRSIZE,
						  DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(&fep->pdev->dev,
					       bdp->cbd_bufaddr))) {
			dev_err(&fep->pdev->dev,
				"Failed to map descriptor rx buffer\n");
			dev_kfree_skb_any(skb);
			goto err;
		}

		bdp->cbd_sc = BD_ENET_RX_EMPTY;
		bdp++;
	}

	/* Set the last buffer to wrap. */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	bdp = fep->tx_bd_base;
	for (i = 0; i < TX_RING_SIZE; i++) {
		fep->tx_bounce[i] = kmalloc(MTIP_SWITCH_TX_FRSIZE, GFP_KERNEL);

		bdp->cbd_sc = 0;
		bdp->cbd_bufaddr = 0;
		bdp++;
	}

	/* Set the last buffer to wrap. */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

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
		return 0;
	}

	if (!fep->br_offload &&
	    (port == 1 || port == 2) && fep->ndev[port - 1])
		mtip_switch_tx(fep->ndev[port - 1]);
	else
		mtip_switch_tx(napi->dev);

	if (pkts < budget) {
		napi_complete_done(napi, pkts);
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

		fep->curr_time = 0;
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
};

bool mtip_is_switch_netdev_port(const struct net_device *ndev)
{
	return ndev->netdev_ops == &mtip_netdev_ops;
}

static int __init mtip_switch_dma_init(struct switch_enet_private *fep)
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

	/* Set the last buffer to wrap */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	/* ...and the same for transmit */
	bdp = fep->tx_bd_base;
	for (i = 0; i < TX_RING_SIZE; i++) {
		/* Initialize the BD for every fragment in the page */
		bdp->cbd_sc = 0;
		bdp->cbd_bufaddr = 0;
		bdp++;
	}

	/* Set the last buffer to wrap */
	bdp--;
	bdp->cbd_sc |= BD_SC_WRAP;

	return 0;
}

static void mtip_ndev_cleanup(struct switch_enet_private *fep)
{
	int i;

	for (i = 0; i < SWITCH_EPORT_NUMBER; i++) {
		if (fep->ndev[i]) {
			unregister_netdev(fep->ndev[i]);
			free_netdev(fep->ndev[i]);
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
			break;
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
			break;
		}
		dev_dbg(&fep->ndev[i]->dev, "%s: MTIP eth L2 switch %pM\n",
			fep->ndev[i]->name, fep->ndev[i]->dev_addr);
	}

	if (ret)
		mtip_ndev_cleanup(fep);

	return ret;
}

static int mtip_parse_of(struct switch_enet_private *fep,
			 struct device_node *np)
{
	struct device_node *p;
	unsigned int port_num;
	int ret = 0;

	p = of_find_node_by_name(np, "ethernet-ports");

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

static int mtip_sw_learning(void *arg)
{
	struct switch_enet_private *fep = arg;

	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);
		/* check learning record valid */
		mtip_atable_dynamicms_learn_migration(fep, fep->curr_time,
						      NULL, NULL);
		schedule_timeout(HZ / 100);
	}

	return 0;
}

static void mtip_mii_unregister(struct switch_enet_private *fep)
{
	mdiobus_unregister(fep->mii_bus);
	mdiobus_free(fep->mii_bus);
}

static const struct mtip_devinfo mtip_imx28_l2switch_info = {
	.quirks = FEC_QUIRK_BUG_CAPTURE | FEC_QUIRK_SINGLE_MDIO |
		  FEC_QUIRK_SWAP_FRAME,
};

static const struct of_device_id mtipl2_of_match[] = {
	{ .compatible = "nxp,imx28-mtip-switch",
	  .data = &mtip_imx28_l2switch_info},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtipl2_of_match);

static int mtip_sw_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id;
	struct switch_enet_private *fep;
	struct mtip_devinfo *dev_info;
	int ret;

	fep = devm_kzalloc(&pdev->dev, sizeof(*fep), GFP_KERNEL);
	if (!fep)
		return -ENOMEM;

	of_id = of_match_node(mtipl2_of_match, pdev->dev.of_node);
	if (of_id) {
		dev_info = (struct mtip_devinfo *)of_id->data;
		if (dev_info)
			fep->quirks = dev_info->quirks;
	}

	fep->pdev = pdev;
	platform_set_drvdata(pdev, fep);

	fep->enet_addr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(fep->enet_addr))
		return PTR_ERR(fep->enet_addr);

	fep->irq = platform_get_irq_byname(pdev, "mtipl2sw");
	if (fep->irq < 0)
		return fep->irq;

	ret = mtip_parse_of(fep, np);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: OF parse error (%d)!\n", __func__,
			ret);
		return ret;
	}

	/* Create an Ethernet device instance.
	 * The switch lookup address memory starts at 0x800FC000
	 */
	fep->hwp_enet = fep->enet_addr;
	fep->hwp = fep->enet_addr + ENET_SWI_PHYS_ADDR_OFFSET;
	fep->hwentry = (struct mtip_addr_table __iomem *)
		(fep->hwp + MCF_ESW_LOOKUP_MEM_OFFSET);

	ret = devm_regulator_get_enable_optional(&pdev->dev, "phy");
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Unable to get and enable 'phy'\n");

	fep->clk_ipg = devm_clk_get_enabled(&pdev->dev, "ipg");
	if (IS_ERR(fep->clk_ipg))
		return dev_err_probe(&pdev->dev, PTR_ERR(fep->clk_ipg),
				     "Unable to acquire 'ipg' clock\n");

	fep->clk_ahb = devm_clk_get_enabled(&pdev->dev, "ahb");
	if (IS_ERR(fep->clk_ahb))
		return dev_err_probe(&pdev->dev, PTR_ERR(fep->clk_ahb),
				     "Unable to acquire 'ahb' clock\n");

	fep->clk_enet_out = devm_clk_get_optional_enabled(&pdev->dev,
							  "enet_out");
	if (IS_ERR(fep->clk_enet_out))
		return dev_err_probe(&pdev->dev, PTR_ERR(fep->clk_enet_out),
				     "Unable to acquire 'enet_out' clock\n");

	/* setup MII interface for external switch ports */
	mtip_enet_init(fep, 1);
	mtip_enet_init(fep, 2);

	spin_lock_init(&fep->learn_lock);
	spin_lock_init(&fep->hw_lock);
	spin_lock_init(&fep->mii_lock);

	ret = devm_request_irq(&pdev->dev, fep->irq, mtip_interrupt, 0,
			       dev_name(&pdev->dev), fep);
	if (ret)
		return dev_err_probe(&pdev->dev, fep->irq,
				     "Could not alloc IRQ\n");

	ret = mtip_register_notifiers(fep);
	if (ret)
		return ret;

	ret = mtip_ndev_init(fep, pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s: Failed to create virtual ndev (%d)\n",
			__func__, ret);
		goto ndev_init_err;
	}

	ret = mtip_switch_dma_init(fep);
	if (ret) {
		dev_err(&pdev->dev, "%s: ethernet switch init fail (%d)!\n",
			__func__, ret);
		goto dma_init_err;
	}

	ret = mtip_mii_init(fep, pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s: Cannot init phy bus (%d)!\n", __func__,
			ret);
		goto mii_init_err;
	}
	/* setup timer for learning aging function */
	timer_setup(&fep->timer_aging, mtip_aging_timer, 0);
	mod_timer(&fep->timer_aging,
		  jiffies + msecs_to_jiffies(LEARNING_AGING_INTERVAL));

	fep->task = kthread_run(mtip_sw_learning, fep, "mtip_l2sw_learning");
	if (IS_ERR(fep->task)) {
		ret = PTR_ERR(fep->task);
		dev_err(&pdev->dev, "%s: learning kthread_run error (%d)!\n",
			__func__, ret);
		goto task_learning_err;
	}

	return 0;

 task_learning_err:
	del_timer(&fep->timer_aging);
	mtip_mii_unregister(fep);
 mii_init_err:
 dma_init_err:
	mtip_ndev_cleanup(fep);
 ndev_init_err:
	mtip_unregister_notifiers(fep);

	return ret;
}

static void mtip_sw_remove(struct platform_device *pdev)
{
	struct switch_enet_private *fep = platform_get_drvdata(pdev);

	mtip_unregister_notifiers(fep);
	mtip_ndev_cleanup(fep);

	mtip_mii_remove(fep);

	kthread_stop(fep->task);
	del_timer(&fep->timer_aging);
	platform_set_drvdata(pdev, NULL);

	kfree(fep);
}

static struct platform_driver mtipl2plat_driver = {
	.driver         = {
		.name   = "mtipl2sw",
		.of_match_table = mtipl2_of_match,
		.suppress_bind_attrs = true,
	},
	.probe          = mtip_sw_probe,
	.remove_new     = mtip_sw_remove,
};

module_platform_driver(mtipl2plat_driver);

MODULE_AUTHOR("Lukasz Majewski <lukma@denx.de>");
MODULE_DESCRIPTION("Driver for MTIP L2 on SOC switch");
MODULE_LICENSE("GPL");
