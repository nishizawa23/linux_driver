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
//#include <include/linux/device.h>


//static ssize_t store_scan(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){
static ssize_t store_scan(struct device_driver *driver, const char *buf,size_t count){

	printk(KERN_ERR "helll0 %s\n\n",buf);
	return 0;
};

static ssize_t show_scan(struct device_driver *driver, char *buf){

	printk(KERN_ERR "show %s\n\n",buf);
	return snprintf(buf, PAGE_SIZE, "%d\n", 111);
};

//static DRIVER_ATTR(scan, S_IWUSR, NULL, store_scan);
static DRIVER_ATTR(scan, 0777, show_scan, store_scan);

static int __init kpp_probe(struct platform_device *pdev)
{
//		driver_create_file(struct device_driver *, const struct driver_attribute *);
        printk(KERN_ERR "fail to request_irq");
		return 0;
}


static int kpp_remove(struct platform_device *pdev)
{
	        return 0;
}

static struct platform_driver kpp_driver = {
    .probe    = kpp_probe,
    .remove   = kpp_remove,
    .driver   = {
    .name     = "kpp_ts",
    .owner    = THIS_MODULE,
    },
};


static int __devinit kpp_init(void)
{
    platform_driver_register(&kpp_driver);
	driver_create_file(&(kpp_driver.driver),&driver_attr_scan);
	return 0;
}

static void __exit kpp_exit(void)
{
	driver_remove_file(&(kpp_driver.driver),&driver_attr_scan);
	platform_driver_unregister(&kpp_driver);
}

module_init(kpp_init);
module_exit(kpp_exit);

MODULE_AUTHOR("nishizawa23");
MODULE_DESCRIPTION("Module kpp");
MODULE_LICENSE("GPL");
MODULE_ALIAS("kpp");
