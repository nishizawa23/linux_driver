/*************************************************************************
 * Author: huangxinghua
 * Email: <nishizawa23@gmail.com>
 * Web: http://www.nishizawa23.com
 * File: mxc_my_8451.c
 * Create Date: 2011-09-13 09:49:55
 *************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/input-polldev.h>
#include <linux/earlysuspend.h>
#include <linux/notifier.h>
#include <linux/suspend.h>

#define MMA8451_I2C_ADDR	0x1C
#define MMA8451_ID		0x2A

#define POLL_INTERVAL_MIN	1
#define POLL_INTERVAL_MAX	500
#define POLL_INTERVAL		100 /* msecs */
#define INPUT_FUZZ		32
#define INPUT_FLAT		32
#define MODE_CHANGE_DELAY_MS	100

#define MMA8451_STATUS_ZYXDR	0x08
#define MMA8451_BUF_SIZE	7

#define MMA8451_PM_NORMAL 0
#define MMA8451_PM_SUSPEND 1
#define MMA8451_PM_EARLY_SUSPEND 2

/* The sensitivity is represented in counts/g. In 2g mode the
sensitivity is 1024 counts/g. In 4g mode the sensitivity is 512
counts/g and in 8g mode the sensitivity is 256 counts/g.
 */
enum {
	MODE_2G = 0,
	MODE_4G,
	MODE_8G,
};

struct mma8451_status {
	u8 mode;
	u8 ctl_reg1;
};

static uint debug_level;

static struct i2c_client *mma8451_i2c_client;
static int mma8451_pm_status = MMA8451_PM_NORMAL;
static struct mma8451_status mma_status;
static struct input_polled_dev *mma8451_idev;
static int senstive_mode = MODE_2G;
static struct device *hwmon_dev;

static const struct i2c_device_id mma8451_id[] = {
	{"mma8451", 0},
};
MODULE_DEVICE_TABLE(i2c, mma8451_id);

static DEFINE_MUTEX(mma8451_lock);

/* register enum for mma8451 registers */
enum {
	MMA8451_STATUS = 0x00,
	MMA8451_OUT_X_MSB,
	MMA8451_OUT_X_LSB,
	MMA8451_OUT_Y_MSB,
	MMA8451_OUT_Y_LSB,
	MMA8451_OUT_Z_MSB,
	MMA8451_OUT_Z_LSB,

	MMA8451_F_SETUP = 0x09,
	MMA8451_TRIG_CFG,
	MMA8451_SYSMOD,
	MMA8451_INT_SOURCE,
	MMA8451_WHO_AM_I,
	MMA8451_XYZ_DATA_CFG,
	MMA8451_HP_FILTER_CUTOFF,

	MMA8451_PL_STATUS,
	MMA8451_PL_CFG,
	MMA8451_PL_COUNT,
	MMA8451_PL_BF_ZCOMP,
	MMA8451_P_L_THS_REG,

	MMA8451_FF_MT_CFG,
	MMA8451_FF_MT_SRC,
	MMA8451_FF_MT_THS,
	MMA8451_FF_MT_COUNT,

	MMA8451_TRANSIENT_CFG = 0x1D,
	MMA8451_TRANSIENT_SRC,
	MMA8451_TRANSIENT_THS,
	MMA8451_TRANSIENT_COUNT,

	MMA8451_PULSE_CFG,
	MMA8451_PULSE_SRC,
	MMA8451_PULSE_THSX,
	MMA8451_PULSE_THSY,
	MMA8451_PULSE_THSZ,
	MMA8451_PULSE_TMLT,
	MMA8451_PULSE_LTCY,
	MMA8451_PULSE_WIND,

	MMA8451_ASLP_COUNT,
	MMA8451_CTRL_REG1,
	MMA8451_CTRL_REG2,
	MMA8451_CTRL_REG3,
	MMA8451_CTRL_REG4,
	MMA8451_CTRL_REG5,

	MMA8451_OFF_X,
	MMA8451_OFF_Y,
	MMA8451_OFF_Z,

	MMA8451_REG_END,
};

static int mma8451_change_mode(struct i2c_client *client, int mode)
{
	int result;
	u8 tmp_data = 0;

	mma_status.ctl_reg1 = 0;
	result = i2c_smbus_write_byte_data(client, MMA8451_CTRL_REG1, 0);
	if (result < 0)
		goto out;

	//open-drain selection on interrupt pad
	tmp_data = 0x01;
	result = i2c_smbus_write_byte_data(client, MMA8451_CTRL_REG3,tmp_data);
	if (result < 0)
		goto out;

	mma_status.mode = mode;
	result = i2c_smbus_write_byte_data(client, MMA8451_XYZ_DATA_CFG,
					   mma_status.mode);
	if (result < 0)
		goto out;

	mma_status.ctl_reg1 |= 0x01;
	result = i2c_smbus_write_byte_data(client, MMA8451_CTRL_REG1,
					   mma_status.ctl_reg1);
	if (result < 0)
		goto out;
	mdelay(MODE_CHANGE_DELAY_MS);

	return 0;
out:
	dev_err(&client->dev, "error when init mma8451:(%d)", result);
	return result;
}

static int mma8451_read_data(short *x, short *y, short *z)
{
	u8 tmp_data[MMA8451_BUF_SIZE];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(mma8451_i2c_client,
					    MMA8451_OUT_X_MSB, 7, tmp_data);
	if (ret < MMA8451_BUF_SIZE) {
		dev_err(&mma8451_i2c_client->dev, "i2c block read failed\n");
		return -EIO;
	}

	*x = ((tmp_data[0] << 8) & 0xff00) | tmp_data[1];
	*y = ((tmp_data[2] << 8) & 0xff00) | tmp_data[3];
	*z = ((tmp_data[4] << 8) & 0xff00) | tmp_data[5];

	*x = (short)(*x) >> 2;
	*y = (short)(*y) >> 2;
	*z = (short)(*z) >> 2;

	if (mma_status.mode == MODE_4G) {
		(*x) = (*x) << 1;
		(*y) = (*y) << 1;
		(*z) = (*z) << 1;
	} else if (mma_status.mode == MODE_8G) {
		(*x) = (*x) << 2;
		(*y) = (*y) << 2;
		(*z) = (*z) << 2;
	}

	return 0;
}

static void report_abs(void)
{
	short x, y, z;
	int result;

	mutex_lock(&mma8451_lock);
	/* wait for the data ready */
	do {
		result = i2c_smbus_read_byte_data(mma8451_i2c_client,
					     MMA8451_STATUS);
	} while (!(result & MMA8451_STATUS_ZYXDR));

	if (mma8451_read_data(&x, &y, &z) != 0)
		goto out;
	if (debug_level == 1)
		printk("x=%d,y=%d,z=%d...\n",x,y,z);

	x = -x;
	input_report_abs(mma8451_idev->input, ABS_X, x);
	input_report_abs(mma8451_idev->input, ABS_Y, y);
	input_report_abs(mma8451_idev->input, ABS_Z, z);
	input_sync(mma8451_idev->input);
out:
	mutex_unlock(&mma8451_lock);
}

static int mma8451_stop_chip(struct i2c_client *client)
{
	mma_status.ctl_reg1 = i2c_smbus_read_byte_data(client,
						       MMA8451_CTRL_REG1);
	return i2c_smbus_write_byte_data(client, MMA8451_CTRL_REG1,
					   mma_status.ctl_reg1 & 0xFE);
}

static void mma8451_dev_poll(struct input_polled_dev *dev)
{
	if (mma8451_pm_status == MMA8451_PM_NORMAL)
		report_abs();
}

static int mma8451_pm_event(struct notifier_block *this, unsigned long event,
			     void *ptr)
{
	switch (event) {
	case PM_HIBERNATION_PREPARE:
		break;	
	case PM_POST_HIBERNATION:
		break;
	case PM_SUSPEND_PREPARE:
		mma8451_pm_status = MMA8451_PM_SUSPEND;
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

static struct notifier_block mma8451_power_notifier = {
	.notifier_call = mma8451_pm_event,
};

static void mma8451_early_suspend(struct early_suspend *h)
{
	if ( mma8451_i2c_client != NULL) {
		mma8451_pm_status = MMA8451_PM_EARLY_SUSPEND;
		mma8451_stop_chip(mma8451_i2c_client);
	}
}

static void mma8451_early_resume(struct early_suspend *h)
{
	if ( mma8451_i2c_client != NULL) {

		if (mma8451_pm_status == MMA8451_PM_SUSPEND) {
			mma8451_change_mode(mma8451_i2c_client, senstive_mode);
		} else if (mma8451_pm_status == MMA8451_PM_EARLY_SUSPEND) {
			i2c_smbus_write_byte_data(mma8451_i2c_client, MMA8451_CTRL_REG1, mma_status.ctl_reg1);
		}

		mma8451_pm_status = MMA8451_PM_NORMAL;
	}
}

static struct early_suspend mma8451_early_suspend_desc = {
    .level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
    .suspend = mma8451_early_suspend,
    .resume = mma8451_early_resume,
};

static int __devinit mma8451_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int result;
	struct input_dev *idev;
	struct i2c_adapter *adapter;

	debug_level = 0;
	mma8451_i2c_client = client;
	adapter = to_i2c_adapter(client->dev.parent);
	result = i2c_check_functionality(adapter,
					 I2C_FUNC_SMBUS_BYTE |
					 I2C_FUNC_SMBUS_BYTE_DATA);
	if (!result)
		goto err_out;

	result = i2c_smbus_read_byte_data(client, MMA8451_WHO_AM_I);

	if (result != MMA8451_ID) {
		dev_err(&client->dev,
			"read chip ID 0x%x is not equal to 0x%x!\n", result,
			MMA8451_ID);
		result = -EINVAL;
		goto err_out;
	}

	/* Initialize the MMA8451 chip */
	result = mma8451_change_mode(client, senstive_mode);
	if (result) {
		dev_err(&client->dev,
			"error when init mma8451 chip:(%d)\n", result);
		goto err_out;
	}

	hwmon_dev = hwmon_device_register(&client->dev);
	if (!hwmon_dev) {
		result = -ENOMEM;
		dev_err(&client->dev,
			"error when register hwmon device\n");
		goto err_out;
	}

	mma8451_idev = input_allocate_polled_device();
	if (!mma8451_idev) {
		result = -ENOMEM;
		dev_err(&client->dev, "alloc poll device failed!\n");
		goto err_alloc_poll_device;
	}
	mma8451_idev->poll = mma8451_dev_poll;
	mma8451_idev->poll_interval = POLL_INTERVAL;
	mma8451_idev->poll_interval_min = POLL_INTERVAL_MIN;
	mma8451_idev->poll_interval_max = POLL_INTERVAL_MAX;
	idev = mma8451_idev->input;
	idev->name = "mma8451";
	idev->id.bustype = BUS_I2C;
	idev->evbit[0] = BIT_MASK(EV_ABS);

	input_set_abs_params(idev, ABS_X, -8192, 8191, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Y, -8192, 8191, INPUT_FUZZ, INPUT_FLAT);
	input_set_abs_params(idev, ABS_Z, -8192, 8191, INPUT_FUZZ, INPUT_FLAT);

	result = input_register_polled_device(mma8451_idev);
	if (result) {
		dev_err(&client->dev, "register poll device failed!\n");
		goto err_register_polled_device;
	}

	return 0;
err_register_polled_device:
	input_free_polled_device(mma8451_idev);
err_alloc_poll_device:
	hwmon_device_unregister(&client->dev);
err_out:
	return result;
}

static int __devexit mma8451_remove(struct i2c_client *client)
{
	int ret;
	ret = mma8451_stop_chip(client);
	hwmon_device_unregister(hwmon_dev);

	return ret;
}

static struct i2c_driver mma8451_driver = {
	.driver = {
		   .name = "mma8451",
		   .owner = THIS_MODULE,
		   //.pm = &mma8451_pm_ops,
		   },
	.probe = mma8451_probe,
	.remove = __devexit_p(mma8451_remove),
	.id_table = mma8451_id,
};

static int __init mma8451_init(void)
{
	/* register driver */
	int res;

	res = i2c_add_driver(&mma8451_driver);
	if (res < 0) {
		printk(KERN_INFO "add mma8451 i2c driver failed\n");
		return -ENODEV;
	}

	register_early_suspend(&mma8451_early_suspend_desc);

	res = register_pm_notifier(&mma8451_power_notifier);
	if (res) {
		printk("mma8415 pm register failed.\n");
	}

	return res;
}

static void __exit mma8451_exit(void)
{
	i2c_del_driver(&mma8451_driver);
	unregister_early_suspend(&mma8451_early_suspend_desc);
	unregister_pm_notifier(&mma8451_power_notifier);
}

MODULE_AUTHOR("pete_sz, Inc.");
MODULE_DESCRIPTION("MMA8451 3-Axis Orientation/Motion Detection Sensor driver");
MODULE_LICENSE("GPL");

module_init(mma8451_init);
module_exit(mma8451_exit);
