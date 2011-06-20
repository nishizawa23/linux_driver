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
#include <asm-arm/arch-s3c2410/map.h>
#include <linux/interrupt.h>

#define MX53_SMD_KEY_VOL_UP 0
#define MX53_SMD_KEY_VOL_DOWN 1
/*
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake)    \
{                               \
	    .gpio       = gpio_num,             \
	    .type       = EV_KEY,               \
	    .code       = ev_code,              \
	    .active_low = act_low,              \
	    .desc       = "btn " descr,             \
	    .wakeup     = wake,                 \
}

static struct gpio_keys_button smd_buttons[] = {
    GPIO_BUTTON(MX53_SMD_KEY_VOL_UP, KEY_VOLUMEUP, 1, "volume-up", 0),
    GPIO_BUTTON(MX53_SMD_KEY_VOL_DOWN, KEY_VOLUMEDOWN, 1, "volume-down", 0),
};

static struct gpio_keys_platform_data smd_button_data = {
   .buttons    = smd_buttons,
   .nbuttons   = ARRAY_SIZE(smd_buttons),
};

truct platform_device mxc_keypad_device = {
	    .name = "mxc_keypad",
		    .id = 0,
			    .num_resources = ARRAY_SIZE(mxc_kpp_resources),
				    .resource = mxc_kpp_resources,
					    .dev = {
							        .release = mxc_nop_release,
									        .platform_data = &keypad_plat_data,
											        },
};  
*/
#define  MX25_INT_KPP 0
static struct resource mxc_kpp_resources[] = {
    [0] = {
         .start = MX25_INT_KPP,
         .end = MX25_INT_KPP,
         .flags = IORESOURCE_IRQ,
    },
	/*
    [1] = {
        .start = S3C2410_PA_GPIO,
        .end = S3C2410_PA_GPIO + S3C24XX_SZ_GPIO - 1,
        .flags = IORESOURCE_MEM,
	*/
};

static struct platform_device kpp_device = {
    .name = "kpp_ts",
    .id = -1,
    .num_resources = ARRAY_SIZE(mxc_kpp_resources),
    .resource = mxc_kpp_resources,
	/*
	.dev = {
		.platform_data = &smd_button_data,
    }
	*/
};
//#endif

static irqreturn_t mxc_kpp_interrupt(int irq, void *dev_id){
	        printk(KERN_ERR
					               "init irq no is %d\n",irq);
			return 0;

}

static int __init kpp_probe(struct platform_device *pdev)
{
		int irq;
		int retval,error;
		irq = platform_get_irq(pdev, 0);
        printk(KERN_ERR
               "init irq no is %d\n",irq);
		retval = request_irq(irq, mxc_kpp_interrupt,IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING |
        IRQF_TRIGGER_FALLING, "hello", NULL);
/*

        irq = gpio_to_irq(button->gpio);
        if (irq < 0) {
        error = irq;
        pr_err("gpio-keys: Unable to get irq number"
               " for GPIO %d, error %d\n",
       // button->gpio, error);
        19, error);
        gpio_free(button->gpio);
//        goto fail;
        }

        error = request_irq(irq,mxc_kpp_interrupt ,
        IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING |
        IRQF_TRIGGER_FALLING,
       // button->desc ? button->desc : "gpio_keys",
        "init_KPP",
        NULL);

        if (error) {
        pr_err("gpio-keys: Unable to claim irq %d; error %d\n",
        irq, error);
        //gpio_free(button->gpio);
       // goto fail;
        }
		*/
		int err = 0;
		return err;
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
	platform_device_register(&kpp_device);        
    return platform_driver_register(&kpp_driver);
}

static void __exit kpp_exit(void)
{
//    platform_device_unregister(&kpp_device);
//   platform_driver_unregister(&kpp_driver);
}

module_init(kpp_init);
module_exit(kpp_exit);

MODULE_AUTHOR("nishizawa23");
MODULE_DESCRIPTION("Module kpp");
MODULE_LICENSE("GPL");
MODULE_ALIAS("kpp");
