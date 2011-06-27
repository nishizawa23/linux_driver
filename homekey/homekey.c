
/*
 ===================================================
 Name            : my2440_buttons.c
 Author          : Huang Gang
 Date            : 09/11/2009
 Copyright       : GPL
 Description     : my2440 buttons driver
 ===================================================
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <mach/regs-gpio.h>
#include <mach/hardware.h>

#define DEVICE_NAME         "my2440_buttons"    //设备名称
#define DEVICE_MAJOR        232                 //主设备号                
#define KEY_TIMER_DELAY1    (HZ/50)             //按键按下去抖延时20毫秒        
#define KEY_TIMER_DELAY2    (HZ/10)             //按键抬起去抖延时100毫秒
#define KEY_DOWN            0                   //按键按下                    
#define KEY_UP              1                   //按键抬起                
#define KEY_UNCERTAIN       2                   //按键不确定                    
#define KEY_COUNT           6                   //6个按键                    

static volatile int ev_press = 0;                //按键按下产生标志
static volatile int key_status[KEY_COUNT];       //记录6个按键的状态    
static struct timer_list key_timers[KEY_COUNT];  //定义6个按键去抖动定时器
static DECLARE_WAIT_QUEUE_HEAD(button_waitq);    //定义并初始化等待队列

//组织硬件资源结构体
struct button_irq_desc    
{
    int irq;            //中断号
    int pin;            //引脚
    int pin_setting;    //引脚配置
    char *name;         //按键名称，注意这个名称，在后面的一个现象中会出现
};

//定义6个按键资源结构体数组
static struct button_irq_desc button_irqs[] = 
{
    {IRQ_EINT8 , S3C2410_GPG0 , S3C2410_GPG0_EINT8 , "KEY0"},
    {IRQ_EINT11, S3C2410_GPG3 , S3C2410_GPG3_EINT11 , "KEY1"},
    {IRQ_EINT13, S3C2410_GPG5 , S3C2410_GPG5_EINT13 , "KEY2"},
    {IRQ_EINT14, S3C2410_GPG6 , S3C2410_GPG6_EINT14 , "KEY3"},
    {IRQ_EINT15, S3C2410_GPG7 , S3C2410_GPG7_EINT15 , "KEY4"},
    {IRQ_EINT19, S3C2410_GPG11, S3C2410_GPG11_EINT19, "KEY5"},
};

static irqreturn_t buttons_interrupt(int irq, void *dev_id)
{
    //获取当前按键资源的索引
    int key = (int)dev_id;

    if(key_status[key] == KEY_UP)
    {
        //设置当前按键的状态为不确定
        key_status[key] = KEY_UNCERTAIN;

        //设置当前按键按下去抖定时器的延时并启动定时器
        key_timers[key].expires = jiffies + KEY_TIMER_DELAY1;
        add_timer(&key_timers[key]);
    }

    return IRQ_RETVAL(IRQ_HANDLED);
}

static void buttons_timer(unsigned long arg)
{
    //获取当前按键资源的索引
    int key = arg;

    //获取当前按键引脚上的电平值来判断按键是按下还是抬起
    int up = s3c2410_gpio_getpin(button_irqs[key].pin);

    if(!up)//低电平，按键按下
    {
        if(key_status[key] == KEY_UNCERTAIN)
        {
            //标识当前按键状态为按下
            key_status[key] = KEY_DOWN;

            //标识当前按键已按下并唤醒等待队列
            ev_press = 1;
            wake_up_interruptible(&button_waitq);
        }

        //设置当前按键抬起去抖定时器的延时并启动定时器
        key_timers[key].expires = jiffies + KEY_TIMER_DELAY2;
        add_timer(&key_timers[key]);
    }
    else//高电平，按键抬起
    {
        //标识当前按键状态为抬起
        key_status[key] = KEY_UP;
    }
}

static int buttons_open(struct inode *inode, struct file *file)
{
    int i;
    int ret;

    for(i = 0; i < KEY_COUNT; i++)
    {
        //设置6个IO口为中断触发方式
        s3c2410_gpio_cfgpin(button_irqs[i].pin, button_irqs[i].pin_setting);

        //设置中断下降沿为有效触发
        set_irq_type(button_irqs[i].irq, IRQ_TYPE_EDGE_FALLING);
        
        //申请中断(类型为快速中断，中断服务时屏蔽所有外部中断？)
        ret = request_irq(button_irqs[i].irq, buttons_interrupt, IRQF_DISABLED, button_irqs[i].name, (void *)i);

        if(ret)
        {
            break;
        }

        //初始化6个按键的状态为抬起
        key_status[i] = KEY_UP;

        //初始化并设置6个去抖定时器
        setup_timer(&key_timers[i], buttons_timer, i);
    }

    if(ret)
    {
        //中断申请失败处理
        i--;

        for(; i >= 0; i--)
        {
            //释放已注册成功的中断
            disable_irq(button_irqs[i].irq);
            free_irq(button_irqs[i].irq, (void *)i);
        }

        return -EBUSY;
    }

    return 0;
}

static int buttons_close(struct inode *inode, struct file *file)
{
    int i;

    //释放6个定时器和中断
    for(i = 0; i < KEY_COUNT; i++)
    {
        del_timer(&key_timers[i]);

        disable_irq(button_irqs[i].irq);
        free_irq(button_irqs[i].irq, (void *)i);
    }

    return 0;
}

static int buttons_read(struct file *file, char __user *buf, size_t count, loff_t *offp)
{
    unsigned long ret;

    if(!ev_press)//判断按键按下产生标识，0没有产生

    {
        if(file->f_flags & O_NONBLOCK)
        {
            //应用程序若采用非阻塞方式读取则返回错误

            return -EAGAIN;
        }
        else
        {
            //以阻塞方式读取且按键按下没有产生，让等待队列进入睡眠
            wait_event_interruptible(button_waitq, ev_press);
        }
    }

    //1为按键按下产生，并清除标识为0，准备给下一次判断用
    ev_press = 0;

    //将内核中的按键状态数据拷贝到用户空间给应用程序使用
    ret = copy_to_user(buf, (void *)key_status, min(sizeof(key_status), count));

    return ret ? -EFAULT : min(sizeof(key_status), count);
}

//驱动程序中的轮询，用于应用程序中的轮询查询是否可对设备进行访问
static int buttons_poll(struct file *file, struct poll_table_struct *wait)
{
    unsigned int mask = 0;

    //添加等待队列到等待队列表中(poll_table)
    poll_wait(file, &button_waitq, wait);

    if(ev_press)
    {
        //标识数据可以获得
        mask |= POLLIN | POLLRDNORM;
    }

    return mask;
}

//设备操作列表
static struct file_operations buttons_fops = 
{
    .owner        = THIS_MODULE,
    .open         = buttons_open,
    .release      = buttons_close,
    .read         = buttons_read,
    .poll         = buttons_poll,
};

static int __init button_init(void)
{
    int ret;

    //注册字符设备
    ret = register_chrdev(DEVICE_MAJOR, DEVICE_NAME, &buttons_fops);

    if(ret < 0)
    {
        printk(DEVICE_NAME " register faild!\n");
        return ret;
    }

    return 0;
}

static void __exit button_exit(void)
{
    //注销字符设备
    unregister_chrdev(DEVICE_MAJOR, DEVICE_NAME);
}

module_init(button_init);
module_exit(button_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Huang Gang");
MODULE_DESCRIPTION("My2440 button driver");