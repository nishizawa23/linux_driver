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

#define MY_CONFIG_PM 1 /*if want use .pm = &gpio_keys_pm_ops, set to 1*/

#define MX53_SMD_KEY_SOFT_LEFT		    (4*32 + 4)	/* GPIO5_4 */  //hxh
#define MX53_SMD_KEY_BACK		    (1*32 + 14)	/* GPIO2_14 */  //hxh
#define MX53_SMD_KEY_MENU		    (1*32 + 15)	/* GPIO2_15 */  //hxh

#define SOFT_LEFT		109
#define KEYCODE_BACK            158
#define KEYCODE_MENU		139

struct test_button_dataa{
	int gpio;
	int code;
	unsigned int type;
	char *test_key_desc;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
};

static struct test_button_dataa test_button_data =
	{ MX53_SMD_KEY_BACK,KEYCODE_BACK, EV_KEY, "test_key_gpio"};

static void gpio_keys_report_event(struct test_button_dataa *bdata)
{
	struct input_dev *input = bdata->input;
	unsigned int type = bdata->type ?: EV_KEY;
	int state = (gpio_get_value(bdata->gpio) ? 0 : 1);

	pr_info("state is %d,tpye is %d,code is %d\n",!!state,type,bdata->code);

	input_event(input, type, bdata->code, !!state);
	input_sync(input);
}

static void gpio_keys_work_func(struct work_struct *work)
{
	struct test_button_dataa *bdata =
		container_of(work, struct test_button_dataa, work);

	pr_info("at %s,jiffies is %lu now\n",__func__,jiffies);

	gpio_keys_report_event(bdata);
}

static void gpio_keys_timer(unsigned long _data)
{
	struct test_button_dataa *data = (struct test_button_dataa*)_data;

	schedule_work(&data->work);
}

static irqreturn_t gpio_keys_isr(int irq, void *dev_id)
{
	int i;

	struct test_button_dataa *bdata = dev_id;

	i = mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(100));

	pr_info("\nhelloo irq %d,jiffies is %lu mod_timer return %d\n",
			irq,jiffies,i);

	return IRQ_HANDLED;
}

static int setup_key(struct test_button_dataa *dev_data)
{
	int error;
	int irq;
	int irqflags;
	int code = dev_data -> code;
	int gpio = dev_data -> gpio;
	char * desc = dev_data->test_key_desc;

	setup_timer(&dev_data->timer, gpio_keys_timer, (unsigned long)dev_data);
	INIT_WORK(&dev_data->work, gpio_keys_work_func);

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

	irq = gpio_to_irq(gpio);
	if (irq < 0) {
		error = irq;
		pr_err("Unable to get irq number for GPIO %d, error %d\n",
			gpio, error);
	}

	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	error = request_irq(irq, gpio_keys_isr, irqflags, desc, dev_data);
	if (error) {
		pr_err("Unable to claim irq %d; error %d\n",
			irq, error);
	}

	return 0;
}

static int __devinit my_key_probe(struct platform_device *pdev)
{
	int error;
	struct test_button_dataa *dev_data = pdev -> dev.platform_data;
	struct input_dev *input;

	input = input_allocate_device();
	dev_data->input = input;

//	platform_set_drvdata(pdev, dev_data);

	input->name = "test_key";
	input->phys = "test_keys/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	setup_key(dev_data);

	input_set_capability(input, dev_data->type, dev_data->code);

	error = input_register_device(input);

	if (error) {
		pr_err("Unable to register input device, error: %d\n",
			error);
	}

//	gpio_keys_report_event(dev_data);
//	input_sync(input);

	device_init_wakeup(&pdev->dev, 1);

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
		.name	= "my_key_ts",
		.owner	= THIS_MODULE,
#ifdef MY_CONFIG_PM
		.pm		= &gpio_keys_pm_ops,
#endif
    }
};

static struct platform_device my_key_device = {
	.name = "my_key_ts",
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

	return unregister_pm_notifier(&my_power_notifier);
}

module_init(my_key_init);
module_exit(my_key_exit);

MODULE_AUTHOR("pete");
MODULE_DESCRIPTION("Module my_key");
MODULE_LICENSE("GPL");
MODULE_ALIAS("my_key");
