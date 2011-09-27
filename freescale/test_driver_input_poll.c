/*************************************************************************
 * Author: huangxinghua
 * Email: <nishizawa23@gmail.com>
 * Web: http://www.nishizawa23.com
 * File: my_key.c
 * Create Date: 2011-05-16 01:38:35
 *************************************************************************/

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/input-polldev.h>

#define POLL_INTERVAL_MIN	1
#define POLL_INTERVAL_MAX	500
#define POLL_INTERVAL		100 /* msecs */

#define MY_CONFIG_PM 1 /*if want use .pm = &gpio_keys_pm_ops, set to 1*/

#define MX53_SMD_KEY_SOFT_LEFT		    (4*32 + 4)	/* GPIO5_4 */  //hxh
#define MX53_SMD_KEY_BACK		    (1*32 + 14)	/* GPIO2_14 */  //hxh
#define MX53_SMD_KEY_MENU		    (1*32 + 15)	/* GPIO2_15 */  //hxh

#define SOFT_LEFT		109
#define KEYCODE_BACK            158
#define KEYCODE_MENU		139

static int state;
static int old_state;

static struct input_polled_dev *input_poll;

struct test_button_dataa{
	int gpio;
	int code;
	unsigned int type;
	char *test_key_desc;
	struct input_polled_dev *input_poll;
};

static struct test_button_dataa test_button_data =
	{ MX53_SMD_KEY_MENU,KEYCODE_MENU, EV_KEY, "test_key_gpio_poll"};

static void test_dev_poll(struct input_polled_dev *dev)
{
	//struct test_button_dataa *bp = container_of(&dev, struct test_button_dataa, input_poll);
	struct test_button_dataa *bp = &test_button_data;
	struct input_dev *input = bp->input_poll->input;
	unsigned int type = bp->type ?: EV_KEY;
	state = (gpio_get_value(bp->gpio) ? 0 : 1);

	if(old_state != state )
	{
		input_event(input, type, bp->code, !!state);
		input_sync(input);
		old_state = state;
	}
}


static int setup_key(struct test_button_dataa *dev_data)
{
	int error;
	int gpio = dev_data -> gpio;
	char * desc = dev_data->test_key_desc;

	error = gpio_request(gpio, desc);
	if (error < 0) {
		pr_err("failed to request GPIO %d, error %d\n",
			gpio, error);
	}

	error = gpio_direction_input(gpio);
	if (error < 0) {
		pr_err("failed to configure"
			" direction for GPIO %d, error %d\n",
			gpio, error);
	}

	return 0;
}


static int __devinit my_key_probe(struct platform_device *pdev)
{
	int error;
	struct test_button_dataa *dev_data = pdev -> dev.platform_data;
	struct input_dev *idev;

	input_poll = input_allocate_polled_device();

	input_poll->poll = test_dev_poll;
	input_poll->poll_interval = POLL_INTERVAL;
	input_poll->poll_interval_min = POLL_INTERVAL_MIN;
	input_poll->poll_interval_max = POLL_INTERVAL_MAX;

	dev_data->input_poll = input_poll;

//	platform_set_drvdata(pdev, dev_data);
	idev = input_poll->input;
	idev->name = "test_poll_key";
	idev->dev.parent = &pdev->dev;

	idev->id.bustype = BUS_HOST;
	idev->id.vendor = 0x0011;
	idev->id.product = 0x0011;
	idev->id.version = 0x0101;

	setup_key(dev_data);

	input_set_capability(idev, dev_data->type, dev_data->code);

	error = input_register_polled_device(input_poll);

	if (error) {
		pr_err("Unable to register poll input device, error: %d\n",
			error);
	}

//	gpio_keys_report_event(dev_data);
//	input_sync(input);

//	device_init_wakeup(&pdev->dev, 1);

	return 0;

}

static int my_key_remove(struct platform_device *pdev)
{
	pr_info("function is  %s, line is %d",__func__,__LINE__);
	return 0;
}

#ifdef MY_CONFIG_PM
static int gpio_keys_resume(struct device *dev)
{
	pr_info("\n@@@@@@@@@@@@@@@@@at %s\n",__func__);

	return 0;
}

static int gpio_keys_suspend(struct device *dev)
{
	pr_info("\n@@@@@@@@@@@@@@@@at %s\n",__func__);

	return 0;
}

static const struct dev_pm_ops gpio_keys_pm_ops = {
	.suspend	= gpio_keys_suspend,
	.resume		= gpio_keys_resume,
};
#endif

static void my_early_suspend(struct early_suspend *h)
{
	pr_info("\n~~~~~~~~~~~~~~~~~at %s\n",__func__);
}

static void my_early_resume(struct early_suspend *h)
{
	pr_info("\n~~~~~~~~~~~~~~~~~at %s\n",__func__);
}

static struct early_suspend my_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
	.suspend = my_early_suspend,
	.resume = my_early_resume,
};

static int my_pm_event(struct notifier_block *this, unsigned long event,
				void *ptr)
{
	switch (event) {
		case PM_HIBERNATION_PREPARE:
			break;
		case PM_POST_HIBERNATION:
			break;
		case PM_SUSPEND_PREPARE:
			//mma8451_pm_status = MMA8451_PM_SUSPEND;
			pr_info("\n**********************at %s\n",__func__);
			break;
		case PM_POST_SUSPEND:
			break;
		case PM_RESTORE_PREPARE:
			break;
		case PM_POST_RESTORE:
			break;
		default:
			break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block my_power_notifier = {
	.notifier_call = my_pm_event,
};

static struct platform_driver my_key_driver = {
    .probe    = my_key_probe,
    .remove   = __devexit_p(my_key_remove),
    .driver   = {
		.name	= "my_key_ts_poll",
		.owner	= THIS_MODULE,
#ifdef MY_CONFIG_PM
		.pm		= &gpio_keys_pm_ops,
#endif
    }
};

static struct platform_device my_key_device = {
	.name = "my_key_ts_poll",
	.id = -1,
	.num_resources = 0,
	.dev = {
		.platform_data = &test_button_data,
	}
};

static int __devinit my_key_init(void)
{
	int ret = 0;

    ret = platform_device_register(&my_key_device);

	if(ret){
		pr_err("Fail to platform_device_register error:%d\n",ret);
		return ret;
	}

	ret = platform_driver_register(&my_key_driver);

	if(ret){
		pr_err("Fail to platform_driver_register error:%d\n",ret);
		return ret;
	}

	register_early_suspend(&my_early_suspend_desc);

	ret = register_pm_notifier(&my_power_notifier);
	if (ret) {
		printk("my_test_driver pm register failed.\n");
	}

	return ret;

}

static void __exit my_key_exit(void)
{
	platform_device_unregister(&my_key_device);

	platform_driver_unregister(&my_key_driver);

	unregister_early_suspend(&my_early_suspend_desc);

	unregister_pm_notifier(&my_power_notifier);

}

module_init(my_key_init);
module_exit(my_key_exit);

MODULE_AUTHOR("pete");
MODULE_DESCRIPTION("Module my_key");
MODULE_LICENSE("GPL");
MODULE_ALIAS("my_key");
