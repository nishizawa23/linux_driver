/*************************************************************************
 * Author: huangxinghua
 * Email: <nishizawa23@gmail.com>
 * Web: http://www.nishizawa23.com
 * File: test.c
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
#include <linux/suspend.h>
#include <linux/notifier.h>

#define MY_CONFIG_PM /*if want use .pm = &gpio_keys_pm_ops*/
//#define ANDROID_KERNEL 1
#define DRIVER_IS_MODULE

struct test_data_s{
	int a;
	int b;
} test_data;

static int __devinit test_probe(struct platform_device *pdev)
{
	pr_info("function is  %s, line is %d",__func__,__LINE__);

	return 0;

}

static int test_remove(struct platform_device *pdev)
{
	pr_info("function is  %s, line is %d",__func__,__LINE__);
	return 0;
}

#ifdef MY_CONFIG_PM
static int test_resume(struct device *dev)
{
	pr_info("at %s line %d\n",__func__,__LINE__);

	return 0;
}

static int test_suspend(struct device *dev)
{
	pr_info("at %s line %d\n",__func__,__LINE__);

	return 0;
}

static const struct dev_pm_ops test_pm_ops = {
	.suspend	= test_suspend,
	.resume		= test_resume,
};
#endif

#ifdef ANDROID_KERNEL
static void test_early_suspend(struct early_suspend *h)
{
	pr_info("at %s line %d\n",__func__,__LINE__);
}

static void test_early_resume(struct early_suspend *h)
{
	pr_info("at %s line %d\n",__func__,__LINE__);
}

static struct early_suspend my_early_suspend_desc = {
	.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
	.suspend = test_early_suspend,
	.resume = test_early_resume,
};
#endif

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

static struct platform_driver test_driver = {
    .probe    = test_probe,
    .remove   = __devexit_p(test_remove),
    .driver   = {
		.name	= "my_test",
		.owner	= THIS_MODULE,
#ifdef MY_CONFIG_PM
		.pm		= &test_pm_ops,
#endif
    }
};

#ifdef DRIVER_IS_MODULE
static void platform_test__release(struct device *dev)
{
	
	pr_info("at %s line %d\n",__func__,__LINE__);
	return;
}
#endif

static struct platform_device test_device = {
	.name = "my_test",
	.id = -1,
	.num_resources = 0,
	.dev = {
		.platform_data = &test_data,
#ifdef DRIVER_IS_MODULE
		.release = platform_test__release,
#endif
	}
};

static int __devinit test_init(void)
{
	int ret = 0;

    ret = platform_device_register(&test_device);

	if(ret){
		pr_err("Fail to platform_device_register error:%d\n",ret);
		return ret;
	}

	ret = platform_driver_register(&test_driver);

	if(ret){
		pr_err("Fail to platform_driver_register error:%d\n",ret);
		return ret;
	}

#ifdef ANDROID_KERNEL
	register_early_suspend(&my_early_suspend_desc);
#endif

	ret = register_pm_notifier(&my_power_notifier);
	if (ret) {
		printk("my_test_driver pm register failed.\n");
	}

	return ret;

}

static void __exit test_exit(void)
{
	platform_device_unregister(&test_device);

	platform_driver_unregister(&test_driver);

#ifdef ANDROID_KERNEL
	unregister_early_suspend(&my_early_suspend_desc);
#endif

	unregister_pm_notifier(&my_power_notifier);
}

module_init(test_init);
module_exit(test_exit);

MODULE_AUTHOR("pete");
MODULE_DESCRIPTION("Module test");
MODULE_LICENSE("GPL");
MODULE_ALIAS("test");
