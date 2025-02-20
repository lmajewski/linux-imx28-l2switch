// SPDX-License-Identifier: GPL-2.0-only
/*
 *  L2 switch Controller driver for MTIP block - bridge network interface
 *
 *  Copyright (C) 2025 DENX Software Engineering GmbH
 *  Lukasz Majewski <lukma@denx.de>
 */

#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>

#include "mtipl2sw.h"

static int mtip_ndev_port_link(struct net_device *ndev,
			       struct net_device *br_ndev,
			       struct netlink_ext_ack *extack)
{
	struct mtip_ndev_priv *priv = netdev_priv(ndev), *other_priv;
	struct switch_enet_private *fep = priv->fep;
	struct net_device *other_ndev;

	/* Check if one port of MTIP switch is already bridged */
	if (fep->br_members && !fep->br_offload) {
		/* Get the second bridge ndev */
		other_ndev = fep->ndev[fep->br_members - 1];
		other_priv = netdev_priv(other_ndev);
		if (other_priv->master_dev != br_ndev) {
			NL_SET_ERR_MSG_MOD(extack,
					   "L2 offloading only possible for the same bridge!");
			return notifier_from_errno(-EOPNOTSUPP);
		}

		fep->br_offload = 1;
		mtip_switch_dis_port_separation(fep);
		mtip_clear_atable(fep);
	}

	if (!priv->master_dev)
		priv->master_dev = br_ndev;

	fep->br_members |= BIT(priv->portnum - 1);

	dev_dbg(&ndev->dev,
		"%s: ndev: %s br: %s fep: %p members: 0x%x offload: %d\n",
		__func__, ndev->name,  br_ndev->name, fep, fep->br_members,
		fep->br_offload);

	return NOTIFY_DONE;
}

static void mtip_netdevice_port_unlink(struct net_device *ndev)
{
	struct mtip_ndev_priv *priv = netdev_priv(ndev);
	struct switch_enet_private *fep = priv->fep;

	dev_dbg(&ndev->dev, "%s: ndev: %s members: 0x%x\n", __func__,
		ndev->name, fep->br_members);

	fep->br_members &= ~BIT(priv->portnum - 1);
	priv->master_dev = NULL;

	if (fep->br_members && fep->br_offload) {
		fep->br_offload = 0;
		mtip_switch_en_port_separation(fep);
		mtip_clear_atable(fep);
	}
}

/* netdev notifier */
static int mtip_netdevice_event(struct notifier_block *unused,
				unsigned long event, void *ptr)
{
	struct net_device *ndev = netdev_notifier_info_to_dev(ptr);
	struct netdev_notifier_changeupper_info *info = ptr;
	struct netlink_ext_ack *extack;
	int ret = NOTIFY_DONE;

	if (!mtip_is_switch_netdev_port(ndev))
		return NOTIFY_DONE;

	extack = netdev_notifier_info_to_extack(&info->info);

	switch (event) {
	case NETDEV_CHANGEUPPER:
		if (!netif_is_bridge_master(info->upper_dev))
			break;

		if (info->linking)
			ret = mtip_ndev_port_link(ndev, info->upper_dev,
						  extack);
		else
			mtip_netdevice_port_unlink(ndev);

		break;
	default:
		return NOTIFY_DONE;
	}

	return notifier_from_errno(ret);
}

static struct notifier_block mtip_netdevice_nb __read_mostly = {
	.notifier_call = mtip_netdevice_event,
};

int mtip_register_notifiers(struct switch_enet_private *fep)
{
	int ret = register_netdevice_notifier(&mtip_netdevice_nb);

	if (ret)
		dev_err(&fep->pdev->dev, "can't register netdevice notifier\n");

	return ret;
}

void mtip_unregister_notifiers(struct switch_enet_private *fep)
{
	unregister_netdevice_notifier(&mtip_netdevice_nb);
}
