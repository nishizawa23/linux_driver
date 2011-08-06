/*************************************************************************
 * Author: huangxinghua
 * Email: <nishizawa23@gmail.com>
 * Web: http://www.nishizawa23.com
 * File: kpp.c
 * Create Date: 2011-05-16 01:38:35
 *************************************************************************/

#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/interrupt.h>

static ssize_t show_scan(struct device *dev,struct device_attribute *attr, char *buf){

	printk(KERN_ERR "show %s\n\n",buf);
	return snprintf(buf, PAGE_SIZE, "%d\n", 111);
};

static DEVICE_ATTR(scan, 0777, show_scan, NULL);

static struct platform_device phone_device = {
	.name = "phone_test",
	.id = -1,
};

static int __devinit phone_init(void)
{
    platform_device_register(&phone_device);
	return device_create_file(&(phone_device.dev),&dev_attr_scan);
}

static void __exit phone_exit(void)
{
	device_remove_file(&(phone_device.dev),&dev_attr_scan);
	return platform_device_unregister(&phone_device);

}

module_init(phone_init);
module_exit(phone_exit);

MODULE_AUTHOR("pete");
MODULE_DESCRIPTION("Module phone");
MODULE_LICENSE("GPL");
MODULE_ALIAS("phone");
