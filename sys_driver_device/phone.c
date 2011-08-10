/*************************************************************************
 * Author: huangxinghua
 * Email: <nishizawa23@gmail.com>
 * Web: http://www.nishizawa23.com
 * File: phone.c
 * Create Date: 2011-05-16 01:38:35
 *************************************************************************/

#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>

static ssize_t show_scan_device(struct device *dev,struct device_attribute *attr, char *buf){

	printk(KERN_ERR "show %s\n\n",buf);
	return snprintf(buf, PAGE_SIZE, "%d\n", 111);
};

static ssize_t store_scan_driver(struct device_driver *driver, const char *buf,size_t count){

	printk(KERN_ERR "helll0 %s\n\n",buf);
	return 0;
};

static ssize_t show_scan_drvier(struct device_driver *driver, char *buf){

	printk(KERN_ERR "show %s\n\n",buf);
	return snprintf(buf, PAGE_SIZE, "%d\n", 111);
};

static ssize_t store_scan_bus(struct bus_type *bus, const char *buf,size_t count){

	printk(KERN_ERR "helll0 %s\n\n",buf);
	return 0;
};

static ssize_t show_scan_bus(struct bus_type *bus, char *buf){

	printk(KERN_ERR "show %s\n\n",buf);
	return snprintf(buf, PAGE_SIZE, "%d\n", 111);
};

static DEVICE_ATTR(scan_dev, 0777, show_scan_device, NULL);

static DRIVER_ATTR(scan_driver, 0777, show_scan_drvier, store_scan_driver);

static BUS_ATTR(scan_bus, 0777, show_scan_bus, store_scan_bus);

static int __init phone_probe(struct platform_device *pdev)
{
        printk(KERN_ERR "fail to request_irq");
		//pr_debug("fail\n");
		return 0;
}

static int phone_remove(struct platform_device *pdev)
{
	        return 0;
}

static struct platform_driver phone_driver = {
    .probe    = phone_probe,
    .remove   = __devexit_p(phone_remove),
    .driver   = {
    .name     = "phone_ts",
    .owner    = THIS_MODULE,
    },
};

static struct platform_device phone_device = {
	.name = "phone_ts",
	.id = -1,
};

static int __devinit phone_init(void)
{
	int ret = 0;

    ret = platform_device_register(&phone_device);

	if(ret){
		printk( KERN_ERR "Fail to platform_device_register error:%d\n",ret);
		return ret;
	}

	ret = platform_driver_register(&phone_driver);

	if(ret){
		printk( KERN_ERR "Fail to platform_driver_register error:%d\n",ret);
		return ret;
	}

	if( device_create_file(&(phone_device.dev),&dev_attr_scan_dev)){

		printk( KERN_ERR "Fail to device_create_file at %s\n",__func__);
	}

	if( bus_create_file(phone_driver.driver.bus,&bus_attr_scan_bus)){

		printk( KERN_ERR "Fail to bus_create_file at %s\n",__func__);
	}

	if( driver_create_file(&(phone_driver.driver),&driver_attr_scan_driver)){

		printk( KERN_ERR "Fail to driver_create_file at %s\n",__func__);
	}
	
	return ret;

}

static void __exit phone_exit(void)
{
	driver_remove_file(&(phone_driver.driver),&driver_attr_scan_driver);

	device_remove_file(&(phone_device.dev),&dev_attr_scan_dev);

	platform_device_unregister(&phone_device);

	return platform_driver_unregister(&phone_driver);
}

module_init(phone_init);
module_exit(phone_exit);

MODULE_AUTHOR("pete");
MODULE_DESCRIPTION("Module phone");
MODULE_LICENSE("GPL");
MODULE_ALIAS("phone");
