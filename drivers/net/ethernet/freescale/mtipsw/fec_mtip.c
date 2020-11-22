// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Lukasz Majewski <lukma@denx.de>
 * Lukasz Majewski <lukma@denx.de>
 */

#include <linux/module.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/if_bridge.h>
#include <linux/mdio.h>
#include <linux/of_mdio.h>
#include <linux/etherdevice.h>

#include "fec_mtip.h"
#include "../fec.h"

static void mtipl2_setup_desc_active(struct mtipl2sw_priv *priv, int id)
{
	struct fec_enet_private *fec = priv->fep[id];

	fec->rx_queue[0]->bd.reg_desc_active =
		fec_hwp(fec) + fec_offset_des_active_rxq(fec, 0);

	fec->tx_queue[0]->bd.reg_desc_active =
		fec_hwp(fec) + fec_offset_des_active_txq(fec, 0);
}

static int mtipl2_en_rx(struct mtipl2sw_priv *priv)
{
	writel(MCF_ESW_RDAR_R_DES_ACTIVE, priv->hwpsw + MCF_ESW_R_RDAR);

	return 0;
}

static void read_atable(struct mtipl2sw_priv *priv, int index,
	unsigned long *read_lo, unsigned long *read_hi)
{
	unsigned long atable_base = (unsigned long)priv->hwentry;

	*read_lo = readl((const volatile void*)atable_base + (index << 3));
	*read_hi = readl((const volatile void*)atable_base + (index << 3) + 4);
}

static void write_atable(struct mtipl2sw_priv *priv, int index,
	unsigned long write_lo, unsigned long write_hi)
{
	unsigned long atable_base = (unsigned long)priv->hwentry;

	writel(write_lo, (volatile void *)atable_base + (index << 3));
	writel(write_hi, (volatile void *)atable_base + (index << 3) + 4);
}

/*
 * Clear complete MAC Look Up Table
 */
static void esw_clear_atable(struct mtipl2sw_priv *priv)
{
	int index;
	for (index = 0; index < 2048; index++)
		write_atable(priv, index, 0, 0);
}

static int mtipl2_sw_enable(struct mtipl2sw_priv *priv)
{
	/*
	 * L2 switch - reset
	 */
	writel(MCF_ESW_MODE_SW_RST, &priv->fecp->ESW_MODE);
	udelay(10);

	/* Configure switch*/
	writel(MCF_ESW_MODE_STATRST, &priv->fecp->ESW_MODE);
	writel(MCF_ESW_MODE_SW_EN, &priv->fecp->ESW_MODE);

	/* Management port configuration, make port 0 as management port */
	writel(0, &priv->fecp->ESW_BMPC);

	/*
	 * Set backpressure threshold to minimize discarded frames
	 * during due to congestion.
	 */
	writel(P0BC_THRESHOLD, &priv->fecp->ESW_P0BCT);

	/* Set the max rx buffer size */
	writel(L2SW_PKT_MAXBLR_SIZE, priv->hwpsw + MCF_ESW_R_BUFF_SIZE);
	/* Enable broadcast on all ports */
	writel(0x7, &priv->fecp->ESW_DBCR);

	/* Enable multicast on all ports */
	writel(0x7, &priv->fecp->ESW_DMCR);

	esw_clear_atable(priv);

	/* Clear all pending interrupts */
	writel(0xffffffff, priv->hwpsw + FEC_IEVENT);

	/* Enable interrupts we wish to service */
	writel(FEC_MTIP_DEFAULT_IMASK, priv->hwpsw + FEC_IMASK);
	priv->l2sw_on = true;

	return 0;
}

static void mtipl2_sw_disable(struct mtipl2sw_priv *priv)
{
	writel(0, &priv->fecp->ESW_MODE);
}

static int mtipl2_port_enable (struct mtipl2sw_priv *priv, int port)
{
	u32 l2_ports_en;

	pr_err("%s: PORT ENABLE %d\n", __func__, port);

	/* Enable tx/rx on L2 switch ports */
	l2_ports_en = readl(&priv->fecp->ESW_PER);
	if (!(l2_ports_en & MCF_ESW_ENA_PORT_0))
		l2_ports_en = MCF_ESW_ENA_PORT_0;

	if (port == 0 && !(l2_ports_en & MCF_ESW_ENA_PORT_1))
		l2_ports_en |= MCF_ESW_ENA_PORT_1;

	if (port == 1 && !(l2_ports_en & MCF_ESW_ENA_PORT_2))
		l2_ports_en |= MCF_ESW_ENA_PORT_2;

	writel(l2_ports_en, &priv->fecp->ESW_PER);

	/*
	 * Check if MAC IP block is already enabled (after switch initializtion)
	 * or if we need to enable it after mtipl2_port_disable was called.
	 */

	return 0;
}

static void mtipl2_port_disable (struct mtipl2sw_priv *priv, int port)
{
	u32 l2_ports_en;

	pr_err(" %s: PORT DISABLE %d\n", __func__, port);

	l2_ports_en = readl(&priv->fecp->ESW_PER);
	if (port == 0)
		l2_ports_en &= ~MCF_ESW_ENA_PORT_1;

	if (port == 1)
		l2_ports_en &= ~MCF_ESW_ENA_PORT_2;

	/* Finally disable tx/rx on port 0 */
	if (!(l2_ports_en & MCF_ESW_ENA_PORT_1) &&
	    !(l2_ports_en & MCF_ESW_ENA_PORT_2))
		l2_ports_en &= ~MCF_ESW_ENA_PORT_0;

	writel(l2_ports_en, &priv->fecp->ESW_PER);
}

irqreturn_t
mtip_l2sw_interrupt_handler(int irq, void *dev_id)
{
	struct mtipl2sw_priv *priv = dev_id;
	struct fec_enet_private *fep = priv->fep[0];
	irqreturn_t ret = IRQ_NONE;
	u32 int_events, int_imask;

	int_events = readl(fec_hwp(fep) + FEC_IEVENT);
	writel(int_events, fec_hwp(fep) + FEC_IEVENT);

	if ((int_events & FEC_MTIP_DEFAULT_IMASK) && fep->link) {
		ret = IRQ_HANDLED;

		if (napi_schedule_prep(&fep->napi)) {
			/* Disable RX interrupts */
			int_imask = readl(fec_hwp(fep) + FEC_IMASK);
			int_imask &= ~FEC_MTIP_ENET_RXF;
			writel(int_imask, fec_hwp(fep) + FEC_IMASK);
			__napi_schedule(&fep->napi);
		}
	}

	return ret;
}

static int mtipl2_parse_of(struct mtipl2sw_priv *priv, struct device_node *np)
{
	struct device_node *port, *p, *fep_np;
	struct platform_device *pdev;
	struct net_device *ndev;
	unsigned int port_num;

	p = of_find_node_by_name(np, "ports");

	for_each_available_child_of_node(p, port) {
		if (of_property_read_u32(port, "reg", &port_num))
			continue;

		priv->n_ports = port_num;

		fep_np = of_parse_phandle(port, "phy-handle", 0);
		pdev = of_find_device_by_node(fep_np);
		ndev = platform_get_drvdata(pdev);
		priv->fep[port_num - 1] = netdev_priv(ndev);
		put_device(&pdev->dev);
	}

	of_node_put(p);

	return 0;
}

int mtipl2_ndev_port_link(struct net_device *ndev,
				 struct net_device *br_ndev)
{
	struct fec_enet_private *fec = netdev_priv(ndev);
	struct mtipl2sw_priv *priv = fec->mtipl2;

	pr_err("%s: ndev: %s br: %s fec: 0x%x 0x%x\n", __func__, ndev->name,
	       br_ndev->name, (unsigned int) fec, fec->dev_id);

	/* Check if MTIP switch is already enabled */
	if(!priv->l2sw_on) {
		/*
		 * Close running network connections - to safely enable
		 * support for mtip L2 switching.
		 */
		if (netif_oper_up(priv->fep[0]->netdev))
			fec_enet_close(priv->fep[0]->netdev);

		if (netif_oper_up(priv->fep[1]->netdev))
			fec_enet_close(priv->fep[1]->netdev);

		/* Configure and enable the L2 switch IP block */
		mtipl2_sw_enable(priv);

		if(!priv->br_ndev)
			priv->br_ndev = br_ndev;
	}

	priv->br_members |= BIT(fec->dev_id);

	/* Enable internal switch port */
	mtipl2_port_enable(fec->mtipl2, fec->dev_id);

	if (fec->dev_id == 1)
		return NOTIFY_DONE;

	/*
	 * Set addresses for DMA0 proper operation to point to L2 switch
	 * IP block.
	 *
	 * The eth1 FEC driver is now only used for controlling the PHY device.
	 */
	fec->mtip_l2sw = true;

	mtipl2_setup_desc_active(priv, fec->dev_id);
	fec_enet_open(priv->fep[fec->dev_id]->netdev);

	mtipl2_en_rx(fec->mtipl2);

	return NOTIFY_DONE;
}

static void mtipl2_netdevice_port_unlink(struct net_device *ndev)
{
	struct fec_enet_private *fec = netdev_priv(ndev);
	struct mtipl2sw_priv *priv = fec->mtipl2;

	pr_err("%s: ndev: %s id: %d\n", __func__, ndev->name, fec->dev_id);

	/* Disable internal switch port */
	mtipl2_port_disable(fec->mtipl2, fec->dev_id);

	if (netif_oper_up(priv->fep[fec->dev_id]->netdev))
		fec_enet_close(priv->fep[fec->dev_id]->netdev);

	priv->br_members &= ~BIT(fec->dev_id);

	fec->mtip_l2sw = false;
	priv->br_ndev = NULL;

	mtipl2_setup_desc_active(priv, fec->dev_id);
	fec_enet_open(priv->fep[fec->dev_id]->netdev);

	if (!priv->br_members) {
		mtipl2_sw_disable(priv);
		priv->l2sw_on = false;
	}
}

bool mtipl2_port_dev_check(const struct net_device *ndev)
{
	if(!fec_get_priv(ndev))
		return false;

	return true;
}

/* netdev notifier */
static int mtipl2_netdevice_event(struct notifier_block *unused,
				  unsigned long event, void *ptr)
{
	struct net_device *ndev = netdev_notifier_info_to_dev(ptr);
	struct netdev_notifier_changeupper_info *info;
	int ret = NOTIFY_DONE;

	if (!mtipl2_port_dev_check(ndev))
		return NOTIFY_DONE;

	pr_err("%s: ndev: %s event: 0x%lx\n", __func__, ndev->name, event);

	switch (event) {
	case NETDEV_CHANGEUPPER:
		info = ptr;

		if (netif_is_bridge_master(info->upper_dev)) {
			if (info->linking)
				ret = mtipl2_ndev_port_link(ndev,
							    info->upper_dev);
			else
				mtipl2_netdevice_port_unlink(ndev);
		}
		break;
	default:
		return NOTIFY_DONE;
	}

	return notifier_from_errno(ret);
}

static struct notifier_block mtipl2_netdevice_nb __read_mostly = {
	.notifier_call = mtipl2_netdevice_event,
};

static int mtipl2_register_notifiers(struct mtipl2sw_priv *priv)
{
	int ret = 0;

	ret = register_netdevice_notifier(&mtipl2_netdevice_nb);
	if (ret) {
		dev_err(priv->dev, "can't register netdevice notifier\n");
		return ret;
	}

	return ret;
}

static void mtipl2_unregister_notifiers(struct mtipl2sw_priv *priv)
{
	unregister_netdevice_notifier(&mtipl2_netdevice_nb);
}

static int mtipl2_sw_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mtipl2sw_priv *priv;
	struct resource *r;
	int ret;

	pr_err("fec: %s\n", __func__);

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mtipl2_parse_of(priv, np);

	/* Wait till eth[01] interfaces are up and running */
	if (!priv->fep[0] || !priv->fep[1] ||
	    !netif_device_present(priv->fep[0]->netdev) ||
	    !netif_device_present(priv->fep[1]->netdev))
		return -EPROBE_DEFER;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->fecp = devm_ioremap_resource(&pdev->dev, r);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	priv->hwentry = devm_ioremap_resource(&pdev->dev, r);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	priv->hwpsw = devm_ioremap_resource(&pdev->dev, r);

	/*
	 * MAC{01} interrupt and descriptors registers have 4 bytes
	 * offset when compared to L2 switch IP block.
	 *
	 * When L2 switch is added "between" ENET (eth0) and MAC{01}
	 * the code for interrupts and setting up descriptors is
	 * reused.
	 *
	 * As for example FEC_IMASK are used also on MAC{01} to
	 * perform MII transmission it is better to subtract the
	 * offset from the outset and reuse defines.
	 */
	priv->hwpsw -= L2SW_CTRL_REG_OFFSET;

	priv->fep[0]->hwpsw = priv->hwpsw;
	priv->fep[1]->hwpsw = priv->hwpsw;

	priv->fep[0]->mtipl2 = priv;
	priv->fep[1]->mtipl2 = priv;

	ret = devm_request_irq(&pdev->dev, platform_get_irq(pdev, 0),
			       mtip_l2sw_interrupt_handler,
			       0, "mtip_l2sw", priv);

	ret = mtipl2_register_notifiers(priv);
	if (ret)
		goto clean_unregister_netdev;

	return 0;

 clean_unregister_netdev:
	mtipl2_unregister_notifiers(priv);

	return ret;
}

static const struct of_device_id mtipl2_of_match[] = {
	{ .compatible = "imx,mtip-l2switch" },
	{ /* sentinel */ },
};

static struct platform_driver mtipl2plat_driver = {
	.probe  = mtipl2_sw_probe,
	.driver = {
		.name = "mtipl2sw",
		.of_match_table = mtipl2_of_match,
	},
};

module_platform_driver(mtipl2plat_driver);

MODULE_AUTHOR("Lukasz Majewski <lukma@denx.de>");
MODULE_DESCRIPTION("Driver for MTIP L2 on SOC switch");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mtipl2sw");
