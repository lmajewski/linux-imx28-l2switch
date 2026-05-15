// SPDX-License-Identifier: GPL-2.0-only
/*
 *  L2 switch Controller driver for MTIP block - switch MGNT
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

#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>

#include "mtipl2sw.h"

int mtip_vlan_input_process(struct switch_enet_private *fep,
			    int port, int mode, unsigned short port_vlanid,
			    int vlan_verify_en, int vlan_domain_num,
			    int vlan_domain_port)
{
	/* Only modes from 1 to 4 are valid*/
	if (mode < 0 || mode > 4) {
		dev_err(&fep->pdev->dev,
			"%s: VLAN input processing mode (%d) not supported\n",
			__func__, mode);
		return -EINVAL;
	}

	if (port < 0 || port > 2) {
		dev_err(&fep->pdev->dev, "%s: Port (%d) not supported!\n",
			__func__, port);
		return -EINVAL;
	}

	if (vlan_verify_en == 1 &&
	    (vlan_domain_num < 0 || vlan_domain_num > 32)) {
		dev_err(&fep->pdev->dev, "%s: Domain out of range\n", __func__);
		return -EINVAL;
	}

	writel(MCF_ESW_PID_VLANID(port_vlanid), fep->hwp + ESW_PID(port));
	if (port == 0) {
		if (vlan_verify_en == 1)
			writel(MCF_ESW_VRES_VLANID(port_vlanid) |
			       MCF_ESW_VRES_P0,
			       fep->hwp + ESW_VRES(vlan_domain_num));

		writel(readl(fep->hwp + ESW_VIMEN) | MCF_ESW_VIMEN_EN0,
		       fep->hwp + ESW_VIMEN);
		writel(readl(fep->hwp + ESW_VIMSEL) | MCF_ESW_VIMSEL_IM0(mode),
		       fep->hwp + ESW_VIMSEL);
	} else if (port == 1) {
		if (vlan_verify_en == 1)
			writel(MCF_ESW_VRES_VLANID(port_vlanid) |
			       MCF_ESW_VRES_P1,
			       fep->hwp + ESW_VRES(vlan_domain_num));

		writel(readl(fep->hwp + ESW_VIMEN) | MCF_ESW_VIMEN_EN1,
		       fep->hwp + ESW_VIMEN);
		writel(readl(fep->hwp + ESW_VIMSEL) | MCF_ESW_VIMSEL_IM1(mode),
		       fep->hwp + ESW_VIMSEL);
	} else if (port == 2) {
		if (vlan_verify_en == 1)
			writel(MCF_ESW_VRES_VLANID(port_vlanid) |
			       MCF_ESW_VRES_P2,
			       fep->hwp + ESW_VRES(vlan_domain_num));

		writel(readl(fep->hwp + ESW_VIMEN) | MCF_ESW_VIMEN_EN2,
		       fep->hwp + ESW_VIMEN);
		writel(readl(fep->hwp + ESW_VIMSEL) | MCF_ESW_VIMSEL_IM2(mode),
		       fep->hwp + ESW_VIMSEL);
	}

	return 0;
}

int mtip_vlan_output_process(struct switch_enet_private *fep, int port,
			     int mode)
{
	if (port < 0 || port > 2) {
		dev_err(&fep->pdev->dev, "%s: Port (%d) not supported!\n",
			__func__, port);
		return -EINVAL;
	}

	if (port == 0) {
		writel(readl(fep->hwp + ESW_VOMSEL) | MCF_ESW_VOMSEL_OM0(mode),
		       fep->hwp + ESW_VOMSEL);
	} else if (port == 1) {
		writel(readl(fep->hwp + ESW_VOMSEL) | MCF_ESW_VOMSEL_OM1(mode),
		       fep->hwp + ESW_VOMSEL);
	} else if (port == 2) {
		writel(readl(fep->hwp + ESW_VOMSEL) | MCF_ESW_VOMSEL_OM2(mode),
		       fep->hwp + ESW_VOMSEL);
	}

	return 0;
}

int mtip_set_vlan_verification(struct switch_enet_private *fep, int port,
			       int vlan_domain_verify_en,
			       int vlan_discard_unknown_en)
{
	if (port < 0 || port > 2) {
		dev_err(&fep->pdev->dev, "%s: Port (%d) not supported!\n",
			__func__, port);
		return -EINVAL;
	}

	if (vlan_domain_verify_en == 1) {
		if (port == 0)
			writel(readl(fep->hwp + ESW_VLANV) | MCF_ESW_VLANV_VV0,
			       fep->hwp + ESW_VLANV);
		else if (port == 1)
			writel(readl(fep->hwp + ESW_VLANV) | MCF_ESW_VLANV_VV1,
			       fep->hwp + ESW_VLANV);
		else if (port == 2)
			writel(readl(fep->hwp + ESW_VLANV) | MCF_ESW_VLANV_VV2,
			       fep->hwp + ESW_VLANV);
	} else if (vlan_domain_verify_en == 0) {
		if (port == 0)
			writel(readl(fep->hwp + ESW_VLANV) & ~MCF_ESW_VLANV_VV0,
			       fep->hwp + ESW_VLANV);
		else if (port == 1)
			writel(readl(fep->hwp + ESW_VLANV) & ~MCF_ESW_VLANV_VV1,
			       fep->hwp + ESW_VLANV);
		else if (port == 2)
			writel(readl(fep->hwp + ESW_VLANV) & ~MCF_ESW_VLANV_VV2,
			       fep->hwp + ESW_VLANV);
	}

	if (vlan_discard_unknown_en == 1) {
		if (port == 0)
			writel(readl(fep->hwp + ESW_VLANV) | MCF_ESW_VLANV_DU0,
			       fep->hwp + ESW_VLANV);
		else if (port == 1)
			writel(readl(fep->hwp + ESW_VLANV) | MCF_ESW_VLANV_DU1,
			       fep->hwp + ESW_VLANV);
		else if (port == 2)
			writel(readl(fep->hwp + ESW_VLANV) | MCF_ESW_VLANV_DU2,
			       fep->hwp + ESW_VLANV);
	} else if (vlan_discard_unknown_en == 0) {
		if (port == 0)
			writel(readl(fep->hwp + ESW_VLANV) & ~MCF_ESW_VLANV_DU0,
			       fep->hwp + ESW_VLANV);
		else if (port == 1)
			writel(readl(fep->hwp + ESW_VLANV) & ~MCF_ESW_VLANV_DU1,
			       fep->hwp + ESW_VLANV);
		else if (port == 2)
			writel(readl(fep->hwp + ESW_VLANV) & ~MCF_ESW_VLANV_DU2,
			       fep->hwp + ESW_VLANV);
	}

	dev_dbg(&fep->pdev->dev, "%s: ESW_VLANV %#x\n", __func__,
		readl(fep->hwp + ESW_VLANV));

	return 0;
}

int mtip_port_multicast_config(struct switch_enet_private *fep,
			       int port, bool enable)
{
	u32 reg = 0;

	if (port < 0 || port > 2) {
		dev_err(&fep->pdev->dev, "%s: Port (%d) not supported\n",
			__func__, port);
		return -EINVAL;
	}

	reg = readl(fep->hwp + ESW_DMCR);
	if (enable) {
		if (port == 0)
			reg |= MCF_ESW_DMCR_P0;
		else if (port == 1)
			reg |= MCF_ESW_DMCR_P1;
		else if (port == 2)
			reg |= MCF_ESW_DMCR_P2;
	} else {
		if (port == 0)
			reg &= ~MCF_ESW_DMCR_P0;
		else if (port == 1)
			reg &= ~MCF_ESW_DMCR_P1;
		else if (port == 2)
			reg &= ~MCF_ESW_DMCR_P2;
	}

	writel(reg, fep->hwp + ESW_DMCR);
	return 0;
}

/* enable or disable port n tx or rx
 * tx_en 0 disable port n tx
 * tx_en 1 enable  port n tx
 * rx_en 0 disable port n rx
 * rx_en 1 enable  port n rx
 */
int mtip_port_enable_config(struct switch_enet_private *fep, int port,
			    bool tx_en, bool rx_en)
{
	u32 reg = 0;

	if (port < 0 || port > 2) {
		dev_err(&fep->pdev->dev, "%s: Port (%d) not supported\n",
			__func__, port);
		return -EINVAL;
	}

	reg = readl(fep->hwp + ESW_PER);
	if (tx_en) {
		if (port == 0)
			reg |= MCF_ESW_PER_TE0;
		else if (port == 1)
			reg |= MCF_ESW_PER_TE1;
		else if (port == 2)
			reg |= MCF_ESW_PER_TE2;
	} else {
		if (port == 0)
			reg &= (~MCF_ESW_PER_TE0);
		else if (port == 1)
			reg &= (~MCF_ESW_PER_TE1);
		else if (port == 2)
			reg &= (~MCF_ESW_PER_TE2);
	}

	if (rx_en) {
		if (port == 0)
			reg |= MCF_ESW_PER_RE0;
		else if (port == 1)
			reg |= MCF_ESW_PER_RE1;
		else if (port == 2)
			reg |= MCF_ESW_PER_RE2;
	} else {
		if (port == 0)
			reg &= (~MCF_ESW_PER_RE0);
		else if (port == 1)
			reg &= (~MCF_ESW_PER_RE1);
		else if (port == 2)
			reg &= (~MCF_ESW_PER_RE2);
	}

	writel(reg, fep->hwp + ESW_PER);
	return 0;
}

void mtip_switch_en_port_separation(struct switch_enet_private *fep)
{
	u32 reg;

	mtip_vlan_input_process(fep, 0, 3, 0x10, 1, 0, 0);
	mtip_vlan_input_process(fep, 1, 3, 0x11, 1, 1, 0);
	mtip_vlan_input_process(fep, 2, 3, 0x12, 1, 2, 0);

	reg = readl(fep->hwp + ESW_VRES(0));
	writel(reg | MCF_ESW_VRES_P1 | MCF_ESW_VRES_P2,
	       fep->hwp + ESW_VRES(0));

	reg = readl(fep->hwp + ESW_VRES(1));
	writel(reg | MCF_ESW_VRES_P0, fep->hwp + ESW_VRES(1));

	reg = readl(fep->hwp + ESW_VRES(2));
	writel(reg | MCF_ESW_VRES_P0, fep->hwp + ESW_VRES(2));

	dev_dbg(&fep->pdev->dev, "%s: VRES0: 0x%x\n",
		__func__, readl(fep->hwp + ESW_VRES(0)));
	dev_dbg(&fep->pdev->dev, "%s: VRES1: 0x%x\n", __func__,
		readl(fep->hwp + ESW_VRES(1)));
	dev_dbg(&fep->pdev->dev, "%s: VRES2: 0x%x\n", __func__,
		readl(fep->hwp + ESW_VRES(2)));

	mtip_set_vlan_verification(fep, 0, 1, 0);
	mtip_set_vlan_verification(fep, 1, 1, 0);
	mtip_set_vlan_verification(fep, 2, 1, 0);

	mtip_vlan_output_process(fep, 0, 2);
	mtip_vlan_output_process(fep, 1, 2);
	mtip_vlan_output_process(fep, 2, 2);
}

void mtip_switch_dis_port_separation(struct switch_enet_private *fep)
{
	writel(0, fep->hwp + ESW_PID(0));
	writel(0, fep->hwp + ESW_PID(1));
	writel(0, fep->hwp + ESW_PID(2));

	writel(0, fep->hwp + ESW_VRES(0));
	writel(0, fep->hwp + ESW_VRES(1));
	writel(0, fep->hwp + ESW_VRES(2));

	writel(0, fep->hwp + ESW_VIMEN);
	writel(0, fep->hwp + ESW_VIMSEL);
	writel(0, fep->hwp + ESW_VLANV);
	writel(0, fep->hwp + ESW_VOMSEL);
}

int mtip_switch_bridge_vlan_init(struct switch_enet_private *fep,
                 int input_mode,
                 int output_mode)
{
    u32 reg;
    /* Only input_modes from 0 to 4 are valid*/
    if (input_mode < 0 || input_mode > 4) {
		dev_err(&fep->pdev->dev,
			"%s: VLAN input processing mode (%d) not supported\n",
			__func__, input_mode);
		return -EINVAL;
	}

	/* Only output_modes from 0 to 4 are valid*/
    if (output_mode < 0 || output_mode > 4) {
		dev_err(&fep->pdev->dev,
			"%s: VLAN output processing mode (%d) not supported\n",
			__func__, output_mode);
		return -EINVAL;
	}

    /* Set a default Port VLAN ID (PVID) for each port. */
    writel(MCF_ESW_PID_VLANID(0x10), fep->hwp + ESW_PID(0));
    writel(MCF_ESW_PID_VLANID(0x11), fep->hwp + ESW_PID(1));
    writel(MCF_ESW_PID_VLANID(0x12), fep->hwp + ESW_PID(2));

    /* Enable VLAN input manipulation on all ports */
    writel(MCF_ESW_VIMEN_EN0 |
           MCF_ESW_VIMEN_EN1 |
           MCF_ESW_VIMEN_EN2,
           fep->hwp + ESW_VIMEN);

    /* Configure the VLAN input manipulation mode for all ports */
    reg = 0;
    reg |= MCF_ESW_VIMSEL_IM0(input_mode);
    reg |= MCF_ESW_VIMSEL_IM1(input_mode);
    reg |= MCF_ESW_VIMSEL_IM2(input_mode);
    writel(reg, fep->hwp + ESW_VIMSEL);

    /* Configure the VLAN output manipulation mode for all ports */
    reg = 0;
    reg |= MCF_ESW_VOMSEL_OM0(output_mode);
    reg |= MCF_ESW_VOMSEL_OM1(output_mode);
    reg |= MCF_ESW_VOMSEL_OM2(output_mode);
    writel(reg, fep->hwp + ESW_VOMSEL);

    /* Allow VLAN ID 0 (priority-tagged frames) on all ports */
    writel(MCF_ESW_VRES_VLANID(0) |
           MCF_ESW_VRES_P0 |
           MCF_ESW_VRES_P1 |
           MCF_ESW_VRES_P2,
           fep->hwp + ESW_VRES(3));

    dev_err(&fep->pdev->dev,
        "basic VLAN init done: VIMEN=0x%08x VIMSEL=0x%08x "
        "VOMSEL=0x%08x VLANV=0x%08x\n",
        readl(fep->hwp + ESW_VIMEN),
        readl(fep->hwp + ESW_VIMSEL),
        readl(fep->hwp + ESW_VOMSEL),
        readl(fep->hwp + ESW_VLANV));

    dev_err(&fep->pdev->dev,
        "DBCR=0x%08x DMCR=0x%08x VRES3=0x%08x\n",
        readl(fep->hwp + ESW_DBCR),
        readl(fep->hwp + ESW_DMCR),
        readl(fep->hwp + ESW_VRES(3)));
	return 0;
}

int mtip_port_broadcast_config(struct switch_enet_private *fep,
			       int port, bool enable)
{
	u32 reg = 0;

	if (port < 0 || port > 2) {
		dev_err(&fep->pdev->dev, "%s: Port (%d) not supported\n",
			__func__, port);
		return -EINVAL;
	}

	reg = readl(fep->hwp + ESW_DBCR);
	if (enable) {
		if (port == 0)
			reg |= MCF_ESW_DBCR_P0;
		else if (port == 1)
			reg |= MCF_ESW_DBCR_P1;
		else if (port == 2)
			reg |= MCF_ESW_DBCR_P2;
	} else {
		if (port == 0)
			reg &= ~MCF_ESW_DBCR_P0;
		else if (port == 1)
			reg &= ~MCF_ESW_DBCR_P1;
		else if (port == 2)
			reg &= ~MCF_ESW_DBCR_P2;
	}

	writel(reg, fep->hwp + ESW_DBCR);
	return 0;
}

/* The frame is forwarded to the forced destination ports.
 * It only replace the MAC lookup function,
 * all other filtering(eg.VLAN verification) act as normal
 */
int mtip_forced_forward(struct switch_enet_private *fep, int port, bool enable)
{
	u32 reg = 0;

	if (port & ~GENMASK(1, 0)) {
		dev_err(&fep->pdev->dev,
			"%s: Forced forward for port(s): 0x%x not supported!\n",
			__func__, port);
		return -EINVAL;
	}

	/* Enable Forced forwarding for port(s) */
	reg |= MCF_ESW_P0FFEN_FD(port & GENMASK(1, 0));

	if (enable)
		reg |= MCF_ESW_P0FFEN_FEN;
	else
		reg &= ~MCF_ESW_P0FFEN_FEN;

	writel(reg, fep->hwp + ESW_P0FFEN);
	return 0;
}

int mtip_port_learning_config(struct switch_enet_private *fep, int port,
			      bool disable, bool irq_adj)
{
	u32 reg = 0;

	if (port < 0 || port > 2) {
		dev_err(&fep->pdev->dev, "%s: Port (%d) not supported\n",
			__func__, port);
		return -EINVAL;
	}

	reg = readl(fep->hwp + ESW_BKLR);
	if (disable) {
		if (irq_adj)
			writel(readl(fep->hwp + ESW_IMR) & ~MCF_ESW_IMR_LRN,
			       fep->hwp + ESW_IMR);

		if (port == 0)
			reg |= MCF_ESW_BKLR_LD0;
		else if (port == 1)
			reg |= MCF_ESW_BKLR_LD1;
		else if (port == 2)
			reg |= MCF_ESW_BKLR_LD2;
	} else {
		if (irq_adj)
			writel(readl(fep->hwp + ESW_IMR) | MCF_ESW_IMR_LRN,
			       fep->hwp + ESW_IMR);

		if (port == 0)
			reg &= ~MCF_ESW_BKLR_LD0;
		else if (port == 1)
			reg &= ~MCF_ESW_BKLR_LD1;
		else if (port == 2)
			reg &= ~MCF_ESW_BKLR_LD2;
	}

	writel(reg, fep->hwp + ESW_BKLR);
	dev_dbg(&fep->pdev->dev, "%s ESW_BKLR %#x, ESW_IMR %#x\n", __func__,
		readl(fep->hwp + ESW_BKLR), readl(fep->hwp + ESW_IMR));

	return 0;
}

int mtip_port_blocking_config(struct switch_enet_private *fep, int port,
			      bool enable)
{
	u32 reg = 0;

	if (port < 0 || port > 2) {
		dev_err(&fep->pdev->dev, "%s: Port (%d) not supported\n",
			__func__, port);
		return -EINVAL;
	}

	reg = readl(fep->hwp + ESW_BKLR);
	if (enable) {
		if (port == 0)
			reg |= MCF_ESW_BKLR_BE0;
		else if (port == 1)
			reg |= MCF_ESW_BKLR_BE1;
		else if (port == 2)
			reg |= MCF_ESW_BKLR_BE2;
	} else {
		if (port == 0)
			reg &= ~MCF_ESW_BKLR_BE0;
		else if (port == 1)
			reg &= ~MCF_ESW_BKLR_BE1;
		else if (port == 2)
			reg &= ~MCF_ESW_BKLR_BE2;
	}

	writel(reg, fep->hwp + ESW_BKLR);
	return 0;
}
