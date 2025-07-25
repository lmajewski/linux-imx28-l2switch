// SPDX-License-Identifier: GPL-2.0-only
/*
 *  L2 switch Controller driver for MTIP block - switchdev
 *
 *  Copyright (C) 2025 DENX Software Engineering GmbH
 *  Lukasz Majewski <lukma@denx.de>
 */

#include <linux/etherdevice.h>
#include <linux/if_bridge.h>
#include <linux/netdevice.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <net/switchdev.h>

#include "mtipl2sw.h"

struct mtip_switchdev_event_work {
	struct work_struct work;
	struct switchdev_notifier_fdb_info fdb_info;
	struct mtip_ndev_priv *priv;
	unsigned long event;
};

static int mtip_port_attr_br_flags_pre_set(struct net_device *ndev,
					   struct switchdev_brport_flags flags)
{
	if (flags.mask & ~BR_LEARNING)
		return -EINVAL;

	return 0;
}

static int mtip_port_stp_state_set(struct mtip_ndev_priv *priv, u8 state)
{
	struct switch_enet_private *fep = priv->fep;
	int port = priv->portnum;
	int ret = 0;

	switch (state) {
	case BR_STATE_FORWARDING:
		ret = mtip_port_enable_config(fep, port, 1, 1);
		if (ret)
			break;
		ret = mtip_port_blocking_config(fep, port, 0);
		break;
	case BR_STATE_LEARNING:
		ret = mtip_port_learning_config(fep, port, 0, 0);
		break;
	case BR_STATE_DISABLED:
		ret = mtip_port_learning_config(fep, port, 1, 0);
		if (ret)
			break;
		ret = mtip_port_enable_config(fep, port, 0, 0);
		break;
	case BR_STATE_LISTENING:
	case BR_STATE_BLOCKING:
		ret = mtip_port_blocking_config(fep, port, 1);
		break;
	default:
		return -EOPNOTSUPP;
	}

	dev_dbg(&fep->pdev->dev, " state: %u\n", state);

	return ret;
}

static int mtip_port_attr_br_flags_set(struct mtip_ndev_priv *priv,
				       struct net_device *orig_dev,
				       struct switchdev_brport_flags flags)
{
	return 0;
}

static int mtip_port_attr_set(struct net_device *ndev, const void *ctx,
			      const struct switchdev_attr *attr,
			      struct netlink_ext_ack *extack)
{
	struct mtip_ndev_priv *priv = netdev_priv(ndev);
	struct switch_enet_private *fep = priv->fep;
	int ret;

	dev_dbg(&fep->pdev->dev, "attr: id %u port: %u\n", attr->id,
	        priv->portnum);

	switch (attr->id) {
	case SWITCHDEV_ATTR_ID_PORT_PRE_BRIDGE_FLAGS:
		ret = mtip_port_attr_br_flags_pre_set(ndev,
						      attr->u.brport_flags);
		break;
	case SWITCHDEV_ATTR_ID_PORT_STP_STATE:
		ret = mtip_port_stp_state_set(priv, attr->u.stp_state);
		dev_dbg(&fep->pdev->dev, "stp state: %u\n", attr->u.stp_state);
		break;
	case SWITCHDEV_ATTR_ID_PORT_BRIDGE_FLAGS:
		ret = mtip_port_attr_br_flags_set(priv, attr->orig_dev,
						  attr->u.brport_flags);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static void mtip_fdb_offload_notify(struct net_device *ndev,
				    struct switchdev_notifier_fdb_info *rcv)
{
	struct switchdev_notifier_fdb_info info = {};

	info.addr = rcv->addr;
	info.vid = rcv->vid;
	info.offloaded = true;
	call_switchdev_notifiers(SWITCHDEV_FDB_OFFLOADED,
				 ndev, &info.info, NULL);
}

static void mtip_switchdev_event_work(struct work_struct *work)
{
	struct mtip_switchdev_event_work *switchdev_work =
		container_of(work, struct mtip_switchdev_event_work, work);
	struct mtip_ndev_priv *priv = switchdev_work->priv;
	struct switch_enet_private *fep = priv->fep;
	struct switchdev_notifier_fdb_info *fdb;
	int ret, port = priv->portnum;

	rtnl_lock();
	switch (switchdev_work->event) {
	case SWITCHDEV_FDB_ADD_TO_DEVICE:
		fdb = &switchdev_work->fdb_info;

		dev_dbg(&fep->pdev->dev,
		        "mtip_fdb_add: MACID = %pM vid = %u flags = %u %u -- port %d\n",
			fdb->addr, fdb->vid, fdb->added_by_user,
			fdb->offloaded, port);

		if (!fdb->added_by_user || fdb->is_local)
			break;
		if (ether_addr_equal(&fep->mac[port - 1][0], (u8 *)fdb->addr))
			port = SWITCH_HOST_PORT_NUM;

		ret = mtip_set_static_table_entry((u8 *)fdb->addr, port, fep);
		if (ret)
			dev_err(&fep->pdev->dev,
			        "mtip_fdb_add: Cannot write entry MACID = %pM port %d\n",
			        fdb->addr, port);

		mtip_fdb_offload_notify(priv->dev, fdb);
		break;
	case SWITCHDEV_FDB_DEL_TO_DEVICE:
		fdb = &switchdev_work->fdb_info;

		dev_dbg(&fep->pdev->dev,
		        "mtip_fdb_del: MACID = %pM vid = %u flags = %u %u -- port %d\n",
			fdb->addr, fdb->vid, fdb->added_by_user,
			fdb->offloaded, port);

		if (!fdb->added_by_user || fdb->is_local)
			break;
		if (ether_addr_equal(&fep->mac[port - 1][0], (u8 *)fdb->addr))
			port = SWITCH_HOST_PORT_NUM;

		ret = mtip_clear_static_table_entry((u8 *)fdb->addr, port, fep);
		if (ret)
			dev_err(&fep->pdev->dev,
			        "mtip_fdb_add: Cannot clear entry MACID = %pM port %d\n",
			        fdb->addr, port);
		break;
	default:
		break;
	}
	rtnl_unlock();

	kfree(switchdev_work->fdb_info.addr);
	kfree(switchdev_work);
	dev_put(priv->dev);
}

/* called under rcu_read_lock() */
static int mtip_switchdev_event(struct notifier_block *unused,
				unsigned long event, void *ptr)
{
	struct net_device *ndev = switchdev_notifier_info_to_dev(ptr);
	struct switchdev_notifier_fdb_info *fdb_info = ptr;
	struct mtip_switchdev_event_work *switchdev_work;
	struct mtip_ndev_priv *priv = netdev_priv(ndev);
	int err;

	if (event == SWITCHDEV_PORT_ATTR_SET) {
		err = switchdev_handle_port_attr_set(ndev, ptr,
						     mtip_is_switch_netdev_port,
						     mtip_port_attr_set);
		return notifier_from_errno(err);
	}

	if (!mtip_is_switch_netdev_port(ndev))
		return NOTIFY_DONE;

	switchdev_work = kzalloc(sizeof(*switchdev_work), GFP_ATOMIC);
	if (WARN_ON(!switchdev_work))
		return NOTIFY_BAD;

	INIT_WORK(&switchdev_work->work, mtip_switchdev_event_work);
	switchdev_work->priv = priv;
	switchdev_work->event = event;

	switch (event) {
	case SWITCHDEV_FDB_ADD_TO_DEVICE:
	case SWITCHDEV_FDB_DEL_TO_DEVICE:
		memcpy(&switchdev_work->fdb_info, ptr,
		       sizeof(switchdev_work->fdb_info));
		switchdev_work->fdb_info.addr = kzalloc(ETH_ALEN, GFP_ATOMIC);
		if (!switchdev_work->fdb_info.addr)
			goto err_addr_alloc;
		ether_addr_copy((u8 *)switchdev_work->fdb_info.addr,
				fdb_info->addr);
		dev_hold(ndev);
		break;
	default:
		kfree(switchdev_work);
		return NOTIFY_DONE;
	}

	queue_work(system_long_wq, &switchdev_work->work);

	return NOTIFY_DONE;

err_addr_alloc:
	kfree(switchdev_work);
	return NOTIFY_BAD;
}

static struct notifier_block mtip_switchdev_notifier = {
	.notifier_call = mtip_switchdev_event,
};

int mtip_switchdev_register_notifiers(struct switch_enet_private *fep)
{
	int ret = 0;

	ret = register_switchdev_notifier(&mtip_switchdev_notifier);
	if (ret) {
		dev_err(&fep->pdev->dev,
		        "register switchdev notifier fail ret:%d\n", ret);
		return ret;
	}

	return 0;
}

void mtip_switchdev_unregister_notifiers(struct switch_enet_private *fep)
{
	unregister_switchdev_notifier(&mtip_switchdev_notifier);
}
