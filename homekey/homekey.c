
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

#define DEVICE_NAME         "my2440_buttons"    //�豸����
#define DEVICE_MAJOR        232                 //���豸��                
#define KEY_TIMER_DELAY1    (HZ/50)             //��������ȥ����ʱ20����        
#define KEY_TIMER_DELAY2    (HZ/10)             //����̧��ȥ����ʱ100����
#define KEY_DOWN            0                   //��������                    
#define KEY_UP              1                   //����̧��                
#define KEY_UNCERTAIN       2                   //������ȷ��                    
#define KEY_COUNT           6                   //6������                    

static volatile int ev_press = 0;                //�������²�����־
static volatile int key_status[KEY_COUNT];       //��¼6��������״̬    
static struct timer_list key_timers[KEY_COUNT];  //����6������ȥ������ʱ��
static DECLARE_WAIT_QUEUE_HEAD(button_waitq);    //���岢��ʼ���ȴ�����

//��֯Ӳ����Դ�ṹ��
struct button_irq_desc    
{
    int irq;            //�жϺ�
    int pin;            //����
    int pin_setting;    //��������
    char *name;         //�������ƣ�ע��������ƣ��ں����һ�������л����
};

//����6��������Դ�ṹ������
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
    //��ȡ��ǰ������Դ������
    int key = (int)dev_id;

    if(key_status[key] == KEY_UP)
    {
        //���õ�ǰ������״̬Ϊ��ȷ��
        key_status[key] = KEY_UNCERTAIN;

        //���õ�ǰ��������ȥ����ʱ������ʱ��������ʱ��
        key_timers[key].expires = jiffies + KEY_TIMER_DELAY1;
        add_timer(&key_timers[key]);
    }

    return IRQ_RETVAL(IRQ_HANDLED);
}

static void buttons_timer(unsigned long arg)
{
    //��ȡ��ǰ������Դ������
    int key = arg;

    //��ȡ��ǰ���������ϵĵ�ƽֵ���жϰ����ǰ��»���̧��
    int up = s3c2410_gpio_getpin(button_irqs[key].pin);

    if(!up)//�͵�ƽ����������
    {
        if(key_status[key] == KEY_UNCERTAIN)
        {
            //��ʶ��ǰ����״̬Ϊ����
            key_status[key] = KEY_DOWN;

            //��ʶ��ǰ�����Ѱ��²����ѵȴ�����
            ev_press = 1;
            wake_up_interruptible(&button_waitq);
        }

        //���õ�ǰ����̧��ȥ����ʱ������ʱ��������ʱ��
        key_timers[key].expires = jiffies + KEY_TIMER_DELAY2;
        add_timer(&key_timers[key]);
    }
    else//�ߵ�ƽ������̧��
    {
        //��ʶ��ǰ����״̬Ϊ̧��
        key_status[key] = KEY_UP;
    }
}

static int buttons_open(struct inode *inode, struct file *file)
{
    int i;
    int ret;

    for(i = 0; i < KEY_COUNT; i++)
    {
        //����6��IO��Ϊ�жϴ�����ʽ
        s3c2410_gpio_cfgpin(button_irqs[i].pin, button_irqs[i].pin_setting);

        //�����ж��½���Ϊ��Ч����
        set_irq_type(button_irqs[i].irq, IRQ_TYPE_EDGE_FALLING);
        
        //�����ж�(����Ϊ�����жϣ��жϷ���ʱ���������ⲿ�жϣ�)
        ret = request_irq(button_irqs[i].irq, buttons_interrupt, IRQF_DISABLED, button_irqs[i].name, (void *)i);

        if(ret)
        {
            break;
        }

        //��ʼ��6��������״̬Ϊ̧��
        key_status[i] = KEY_UP;

        //��ʼ��������6��ȥ����ʱ��
        setup_timer(&key_timers[i], buttons_timer, i);
    }

    if(ret)
    {
        //�ж�����ʧ�ܴ���
        i--;

        for(; i >= 0; i--)
        {
            //�ͷ���ע��ɹ����ж�
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

    //�ͷ�6����ʱ�����ж�
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

    if(!ev_press)//�жϰ������²�����ʶ��0û�в���

    {
        if(file->f_flags & O_NONBLOCK)
        {
            //Ӧ�ó��������÷�������ʽ��ȡ�򷵻ش���

            return -EAGAIN;
        }
        else
        {
            //��������ʽ��ȡ�Ұ�������û�в������õȴ����н���˯��
            wait_event_interruptible(button_waitq, ev_press);
        }
    }

    //1Ϊ�������²������������ʶΪ0��׼������һ���ж���
    ev_press = 0;

    //���ں��еİ���״̬���ݿ������û��ռ��Ӧ�ó���ʹ��
    ret = copy_to_user(buf, (void *)key_status, min(sizeof(key_status), count));

    return ret ? -EFAULT : min(sizeof(key_status), count);
}

//���������е���ѯ������Ӧ�ó����е���ѯ��ѯ�Ƿ�ɶ��豸���з���
static int buttons_poll(struct file *file, struct poll_table_struct *wait)
{
    unsigned int mask = 0;

    //��ӵȴ����е��ȴ����б���(poll_table)
    poll_wait(file, &button_waitq, wait);

    if(ev_press)
    {
        //��ʶ���ݿ��Ի��
        mask |= POLLIN | POLLRDNORM;
    }

    return mask;
}

//�豸�����б�
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

    //ע���ַ��豸
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
    //ע���ַ��豸
    unregister_chrdev(DEVICE_MAJOR, DEVICE_NAME);
}

module_init(button_init);
module_exit(button_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Huang Gang");
MODULE_DESCRIPTION("My2440 button driver");