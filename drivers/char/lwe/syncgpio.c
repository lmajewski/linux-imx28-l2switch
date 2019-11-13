/*
 * (C) Copyright 2017
 * Stefano Babic, DENX Software Engineering, sbabic@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/hrtimer.h>
#include <linux/cdev.h>

#define DRV_NAME	"syncgpio"

/*
 * Data passed at the initialization
 * Value are read from DT
 */
struct platform_syncgpio_data {
	unsigned int period;
	unsigned int gpio;
	unsigned int max_deviation;
};

struct syncgpio_data {
	struct platform_device	*pdev;
	struct device		*dev;
	unsigned int		period;
	bool			enabled;
	struct hrtimer		timer;
	struct hrtimer		timer_from_past;
	unsigned long		lastjiffies;
	unsigned int		sync;
	dev_t			devt;
	struct cdev		cdev;
	struct class		*class;
	spinlock_t		lock;
	unsigned int		gpio;
	unsigned int		accepted_deviation;
	unsigned int		restart_cnt;
	unsigned int		gpiostatus;
	ktime_t			lastcheck;
};


static unsigned int allowed_periods[] = { 500000, 1000000, 2000000, 5000000, 10000000 };
static ktime_t syncgpio_start_timer(struct syncgpio_data *srs);

/*
 * Functions for sysfs interface
 */
static void syncgpio_enable(struct syncgpio_data *srs, bool on)
{
	ktime_t checktimer;
	if (!on) {
		hrtimer_cancel(&srs->timer_from_past);
		hrtimer_cancel(&srs->timer);
		srs->gpiostatus = 0;
		gpio_set_value(srs->gpio, srs->gpiostatus);
		srs->enabled = false;
	} else {
		srs->enabled = true;
		srs->gpiostatus = 0;
		checktimer = syncgpio_start_timer(srs);
		hrtimer_start(&srs->timer_from_past, checktimer, HRTIMER_MODE_REL);
	}
}

enum hrtimer_restart syncgpio_check(struct hrtimer *hr_timer);

static ssize_t syncgpio_show_period(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct syncgpio_data *srs = dev_get_drvdata(dev);

	if (!srs)
		return -EFAULT;

	return snprintf(buf, PAGE_SIZE, "%u\n", srs->period);

}

static ssize_t syncgpio_store_period(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct syncgpio_data *srs = dev_get_drvdata(dev);
	unsigned long value;
	int i;

	sscanf(buf, "%lu", &value);
	if (!value)
		return -EINVAL;

	/*
	 * Just a few values are allowed:
	 * 500uS, 1mS, 2mS, 5ms and 10 mS
	 */
	for (i = 0; i < ARRAY_SIZE(allowed_periods); i++) {
		if (value == allowed_periods[i])
			break;
	}

	if (i == ARRAY_SIZE(allowed_periods))
		return -EINVAL;

	if (value != srs->period) {
		srs->period = value;
	}

	return count;
}

static DEVICE_ATTR(period, S_IRUGO | S_IWUSR, syncgpio_show_period, syncgpio_store_period);

static ssize_t syncgpio_show_accepted_deviation(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct syncgpio_data *srs = dev_get_drvdata(dev);

	if (!srs)
		return -EFAULT;

	return snprintf(buf, PAGE_SIZE, "%u\n", srs->accepted_deviation);
}

static ssize_t syncgpio_store_accepted_deviation(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct syncgpio_data *srs = dev_get_drvdata(dev);
	unsigned long flags;
	unsigned long value;

	sscanf(buf, "%lu", &value);
	if (!value || value > 100)
		return -EINVAL;

	spin_lock_irqsave(&srs->lock, flags);
	srs->accepted_deviation = value;
	spin_unlock_irqrestore(&srs->lock, flags);

	return count;
}

static DEVICE_ATTR(max_deviation, S_IRUGO | S_IWUSR,
		syncgpio_show_accepted_deviation,
		syncgpio_store_accepted_deviation);

static ssize_t syncgpio_show_enable(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct syncgpio_data *srs = dev_get_drvdata(dev);

	if (!srs)
		return -EFAULT;

	return snprintf(buf, PAGE_SIZE, "%u\n", srs->enabled);
}

static ssize_t syncgpio_store_enable(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct syncgpio_data *srs = dev_get_drvdata(dev);
	unsigned long value;

	sscanf(buf, "%lu", &value);
	if (value != 0 && value != 1)
		return -EINVAL;

	if (value == srs->enabled)
		return count;

	syncgpio_enable(srs, value);

	return count;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
		syncgpio_show_enable,
		syncgpio_store_enable);

static ssize_t syncgpio_show_restart_cnt(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct syncgpio_data *srs = dev_get_drvdata(dev);

	if (!srs)
		return -EFAULT;

	return snprintf(buf, PAGE_SIZE, "%u\n", srs->restart_cnt);
}
static DEVICE_ATTR(restart_cnt, S_IRUGO, syncgpio_show_restart_cnt, NULL);

static struct attribute *syncgpio_sysfs_entries[] = {
	&dev_attr_period.attr,
	&dev_attr_max_deviation.attr,
	&dev_attr_restart_cnt.attr,
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute_group syncgpio_attribute_group = {
	.name = NULL,		/* put in device directory */
	.attrs = syncgpio_sysfs_entries,
};

static ktime_t next_trigger(struct syncgpio_data *srs)
{
	unsigned long half_period = srs->period / 2;

	return ns_to_ktime(half_period);

}

/**
 * @return relative timestamp for (re)start of controlling monotonic timer
 */
static unsigned long next_check_tim(struct syncgpio_data *p)
{
	unsigned long half_period = p->period / 2;

	return (half_period) + ((half_period / 100) * p->accepted_deviation);
}

static int counter = 0;

/** (re)start timer for gpio toggle
 *
 * @return relative timestamp for (re)start of controlling monotonic
 *         timer, (only useful) after (re)start of absolute timer
 */
static ktime_t syncgpio_start_timer(struct syncgpio_data *srs)
{
	struct timespec now;
	ktime_t next;
	u32 remainder;
	/* greatest allowed timeslice */
	u32 timeslice = allowed_periods[ARRAY_SIZE(allowed_periods) - 1];

	now = ktime_to_timespec(ktime_get_real()); /* read (wall-) time */

	/* calculate in 1/100s => align start value to realtime clock according
	 * to max allowed timeslice */
	remainder = now.tv_nsec / timeslice;
	/* do not mess with this, at this point you cannot know,
	 * if the actual cycle is nearly over, or not: therfore move the
	 * start point always in the next cycle */
	remainder += 2;

	/* consider turnover */
	if (remainder > 99) {
		now.tv_sec = now.tv_sec + 1;
		remainder %= 100;
	}

	/* recalculate in ns and setup start time */
	now.tv_nsec = remainder * timeslice;

	next = timespec_to_ktime(now);
	hrtimer_start(&srs->timer, next, HRTIMER_MODE_ABS);

	/* reset last check time */
	srs->lastcheck = ktime_set(0, 0);
	counter = 0;

	/* return timestamp for start of controlling monotonic timer:
	 * respect worst case for first run: 2 * period + 1ms */
	next = ktime_set(0, (timeslice * 2) + timeslice);

	return (ktime_add_ns(next, next_check_tim(srs)));
}

/*
 * This is called when time is set to the past
 */
enum hrtimer_restart syncgpio_check_monotonic(struct hrtimer *hr_timer)
{
	struct syncgpio_data *p = container_of(hr_timer,
			struct syncgpio_data, timer_from_past);
	ktime_t next;

	/* stop timer */
	hrtimer_cancel(&p->timer);

	/* reset gpiostatus and reset gpio,
	 * always start with rising edge */
	p->gpiostatus = 0;
	gpio_set_value(p->gpio, p->gpiostatus);
	p->restart_cnt++;

	printk("deviation violation detected => restart sync signal\n");

	/* restart timer with next regular period */
	next = syncgpio_start_timer(p);
	hrtimer_forward_now(&p->timer_from_past, next);

	return HRTIMER_RESTART;
}


enum hrtimer_restart syncgpio_check(struct hrtimer *hr_timer)
{
	struct syncgpio_data *p = container_of(hr_timer,
			struct syncgpio_data, timer);
	s64 diff_ns = 0;
	s64 deviation_ns = 0;
	ktime_t now;

	now = ktime_get_real(); /* read (wall-) time */

	/* check if lastcheck is valid */
	if (ktime_to_ns(p->lastcheck) > 0) {

		/* check deviation */
		diff_ns = ktime_to_ns(ktime_sub(now, p->lastcheck));

		if ((p->period / 2) > diff_ns) {
			deviation_ns = (p->period / 2) - diff_ns;
		} else {
			deviation_ns = diff_ns - (p->period / 2);
		}

		/* violation found */
		if (deviation_ns > (((p->period / 2) / 100) * p->accepted_deviation)) {
			/* Wait for monothonic => restart timer with next regular period */
			return HRTIMER_NORESTART;
		}
	}

	/* memorize actual timestamp */
	p->lastcheck = now;

	/* toggle signal if a correct trigger is established */
	hrtimer_cancel(&p->timer_from_past);

	p->gpiostatus = p->gpiostatus ? 0 : 1;
	gpio_set_value(p->gpio, p->gpiostatus);

	hrtimer_forward_now(&p->timer, next_trigger(p));
	hrtimer_start(&p->timer_from_past, ktime_set(0,next_check_tim(p)), HRTIMER_MODE_REL);

	return HRTIMER_RESTART;
}

#ifdef CONFIG_OF
static const struct of_device_id of_syncgpio_match[] = {
	{ .compatible = DRV_NAME, },
	{},
};
MODULE_DEVICE_TABLE(of, of_syncgpio_match);

static int syncgpio_parse_dt(struct device *dev,
			  struct platform_syncgpio_data *data)
{
	struct device_node *node = dev->of_node;
	int ret;

	if (!node)
		return -ENODEV;

	if (!data)
		return -EINVAL;

	memset(data, 0, sizeof(*data));

	/* find default values */
	ret = of_property_read_u32(node, "period", &data->period);
	if (ret < 0)
		return ret;

	ret = of_property_read_u32(node, "max-deviation", &data->max_deviation);
	if (ret < 0 || data->max_deviation > 100) {
		dev_info(dev, "max-deviation not found in DTS, set to 100\n");
		data->max_deviation = 100;
	}

	data->gpio = of_get_gpio(node, 0);

	dev_info(dev, "Period %u, max-deviation %d\n",
			data->period, data->max_deviation);

	return 0;
}

#else
static int syncgpio_parse_dt(struct device *dev,
				  struct platform_syncgpio_data *data)
{
	return -ENODEV;
}
#endif

static int syncgpio_probe(struct platform_device *pdev)
{
	struct platform_syncgpio_data *data = dev_get_platdata(&pdev->dev);
	struct platform_syncgpio_data defdata;
	struct syncgpio_data *srs;
	int ret;

	if (!data) {
		ret = syncgpio_parse_dt(&pdev->dev, &defdata);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to find platform data\n");
			return ret;
		}

		data = &defdata;
	}

	if (!gpio_is_valid(data->gpio)) {
		dev_info(&pdev->dev, "Wrong GPIO for system time: %d\n",
				data->gpio);
		return -ENODEV;
	}
       
	srs = devm_kzalloc(&pdev->dev, sizeof(*srs), GFP_KERNEL);
	if (!srs) {
		return -ENOMEM;
	}

	/* Take PWM setup from DT */
	srs->pdev = pdev;
	srs->period = data->period;
	srs->gpio = data->gpio;
	srs->enabled = false;
	srs->gpiostatus = 0;
	srs->lastcheck = ktime_set(0, 0);

	dev_info(&pdev->dev, "Enabling GPIO for system time: %d\n",
			srs->gpio);
	devm_gpio_request(&pdev->dev, srs->gpio, "syncgpio-gpio-test");
	gpio_direction_output(srs->gpio, srs->gpiostatus);

	/*
	 * When the deviation is greater as this value
	 * (in percent), it is restarted
	 */
	srs->accepted_deviation = data->max_deviation;

	spin_lock_init(&srs->lock);

	/* Associated private structure to device */
	platform_set_drvdata(pdev, srs);

	/* Create char device and class */
	srs->class = class_create(THIS_MODULE, "syncgpio");
	if (!srs->class)
		return -ENODEV;

	ret = sysfs_create_group(&pdev->dev.kobj, &syncgpio_attribute_group);
	if (ret) {
		dev_err(&pdev->dev, "failed to create sysfs device attributes\n");
		goto undo_class_create;
	}

	/*
	 * First timer is set synchrounous with the system time
	 * if the time is set to the past, the timer
	 * is never hit
	 */
	hrtimer_init(&srs->timer, CLOCK_REALTIME, HRTIMER_MODE_ABS);
	srs->timer.function = &syncgpio_check;
	srs->lastjiffies = jiffies;

	hrtimer_init(&srs->timer_from_past, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	srs->timer_from_past.function = &syncgpio_check_monotonic;

	syncgpio_enable(srs, true);

	return 0;

undo_class_create:
	class_destroy(srs->class);

	return ret;

}

static int syncgpio_remove(struct platform_device *pdev)
{
	struct syncgpio_data *srs;
	srs = dev_get_drvdata(&pdev->dev);

	sysfs_remove_group(&pdev->dev.kobj, &syncgpio_attribute_group);
	hrtimer_cancel(&srs->timer);
	hrtimer_cancel(&srs->timer_from_past);

	class_destroy(srs->class);

	if (gpio_is_valid(srs->gpio))
		gpio_free(srs->gpio);
	return 0;
}

static struct platform_driver syncgpio_driver = {
	.probe		= syncgpio_probe,
	.remove		= syncgpio_remove,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_syncgpio_match),
	},
};

module_platform_driver(syncgpio_driver);

MODULE_AUTHOR("Stefano Babic <sbabic@denx.de");
MODULE_DESCRIPTION("SRS synchron tick generator");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:syncgpio");
