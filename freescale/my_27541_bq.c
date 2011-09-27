/*************************************************************************
 * Author: huangxinghua
 * Email: <nishizawa23@gmail.com>
 * Web: http://www.nishizawa23.com
 * File: my_27541.c
 * Create Date: 2011-09-20 17:10:38
 *************************************************************************/

/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <asm/unaligned.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <mach/hardware.h>
#include <linux/kthread.h>
#include <linux/pmic_external.h>
#include <linux/slab.h>
#include <mach/arc_otg.h>
//#include <linux/pmic_battery.h>
//#include "../arch/arm/mach-mx5/mx51_pins.h"
//#include "../arch/arm/plat-mxc/include/mach/iomux-mx51.h"


#define bq27541CMD_CNTL_LSB  0x00		/*Control( )*/
#define bq27541CMD_CNTL_MSB  0x01
#define bq27541CMD_AR_LSB    0x02		/*AtRate( )*/
#define bq27541CMD_AR_MSB    0x03
#define bq27541CMD_ARTTE_LSB 0x04		/*AtRateTimeToEmpty( )*/
#define bq27541CMD_ARTTE_MSB 0x05
#define bq27541CMD_TEMP_LSB  0x06		/*Temperature( )*/
#define bq27541CMD_TEMP_MSB  0x07
#define bq27541CMD_VOLT_LSB  0x08		/*Voltage( )*/
#define bq27541CMD_VOLT_MSB  0x09
#define bq27541CMD_FLAGS_LSB 0x0A		/*Flags( )*/
#define bq27541CMD_FLAGS_MSB 0x0B
#define bq27541CMD_NAC_LSB   0x0C		/*NominalAvailableCapacity( )*/
#define bq27541CMD_NAC_MSB   0x0D
#define bq27541CMD_FAC_LSB   0x0E		/*FullAvailableCapacity( )*/
#define bq27541CMD_FAC_MSB   0x0F
#define bq27541CMD_RM_LSB    0x10		/*RemainingCapacity( )*/
#define bq27541CMD_RM_MSB    0x11
#define bq27541CMD_FCC_LSB   0x12		/*FullChargeCapacity( )*/
#define bq27541CMD_FCC_MSB   0x13
#define bq27541CMD_AI_LSB    0x14		/*AverageCurrent( )*/
#define bq27541CMD_AI_MSB    0x15
#define bq27541CMD_TTE_LSB   0x16		/*TimeToEmpty( )*/
#define bq27541CMD_TTE_MSB   0x17
#define bq27541CMD_TTF_LSB   0x18		/*TimeToFull( )*/
#define bq27541CMD_TTF_MSB   0x19
#define bq27541CMD_SI_LSB    0x1A		/*StandbyCurrent( )*/
#define bq27541CMD_SI_MSB    0x1B
#define bq27541CMD_STTE_LSB  0x1C		/*StandbyTimeToEmpty( )*/
#define bq27541CMD_STTE_MSB  0x1D
#define bq27541CMD_MLI_LSB   0x1E		/*MaxLoadCurrent( )*/
#define bq27541CMD_MLI_MSB   0x1F
#define bq27541CMD_MLTTE_LSB 0x20		/*MaxLoadTimeToEmpty( )*/
#define bq27541CMD_MLTTE_MSB 0x21
#define bq27541CMD_AE_LSB    0x22		/*AvailableEnergy( )*/
#define bq27541CMD_AE_MSB    0x23
#define bq27541CMD_AP_LSB    0x24		/*AveragePower( )*/
#define bq27541CMD_AP_MSB    0x25
#define bq27541CMD_TTECP_LSB 0x26		/*TTEatConstantPower( )*/
#define bq27541CMD_TTECP_MSB 0x27
#define bq27541CMD_RSVD_LSB  0x28		/*Reserved*/
#define bq27541CMD_RSVD_MSB  0x29
#define bq27541CMD_CC_LSB    0x2A		/*CycleCount( )*/
#define bq27541CMD_CC_MSB    0x2B
#define bq27541CMD_SOC_LSB   0x2C		/*StateOfCharge( )*/
#define bq27541CMD_SOC_MSB   0x2D


/*Control() Subcommands*/

#define bq27541CMD_CONTORL_STATUS	0x0000		/*reports the status of DFchecksum,Hibernate,IT,etc.*/
#define bq27541CMD_DEVICE_TYPE		0x0001		/*reports the device type(0x0541)*/
#define bq27541CMD_FW_VERSION		0x0002		/*Reports the firmware version on the device type*/
#define bq27541CMD_HW_VERSION		0x0003		/*Reports the hardware version of the device type*/
#define bq27541CMD_DF_CHECKSUM		0x0004		/*Enables a data flash checksum to be generated and reports on a read*/
#define bq27541CMD_RESET_DATA		0x0005		/*Returns reset data*/
#define bq27541CMD_Reserved		0x0006		/*Not to be used*/
#define bq27541CMD_PREV_MACWRITE	0x0007		/*Returns previous MAC command code*/
#define bq27541CMD_CHEM_ID		0x0008		/*Reports the chemical identifier of the Impedance Track? configuration*/
#define bq27541CMD_SET_FULLSLEEP	0x0010		/*Set the [FullSleep] bit in Control Status register to 1*/
#define bq27541CMD_SET_HIBERNATE	0x0011		/*Forces CONTROL_STATUS [HIBERNATE] to 1*/
#define bq27541CMD_CLEAR_HIBERNATE	0x0012		/*Forces CONTROL_STATUS [HIBERNATE] to 0*/
#define bq27541CMD_SET_SHUTDOWN		0x0013		/*Enables the SE pin to change state*/
#define bq27541CMD_CLEAR_SHUTDOWN	0x0014		/*Disables the SE pin from changing state*/
#define bq27541CMD_SEALED		0x0020		/*Places the bq27541 is SEALED access mode*/
#define bq27541CMD_IT_ENABLE		0x0021		/*Enables the Impedance Track? algorithm*/
#define bq27541CMD_CAL_MODE		0x0040		/*Places the bq27541 in calibration mode*/
#define bq27541CMD_RESET		0x0041		/*Forces a full reset of the bq27541*/

/*Extended Commands*/

#define bq27541CMD_RSVD	     0x34		/*Reserved*/
#define bq27541CMD_DCAP_LSB  0x3C		/*DesignCapacity( )*/
#define bq27541CMD_DCAP_MSB  0x3D
#define bq27541CMD_DFCLS     0x3E		/*DataFlashClass( )*/
#define bq27541CMD_DFBLK     0x3F		/*DataFlashBlock( )*/
#define bq27541CMD_DFD       0x40		/*Block data( )*/
#define bq27541CMD_DFDCKS    0x60		/*BlockDataCheckSum( )*/
#define bq27541CMD_DFDCNTL   0x61		/*BlockDataControl( )*/
#define bq27541CMD_DNAMELEN  0x62		/*DeviceNameLength( )*/
#define bq27541CMD_DNAME     0x63		/*DeviceName( )*/
//#define bq27541CMD_RSVD	     0x6A		/*Reserved*/

#define DEISGN_CAPACITY_LSB  0x0a
#define DEISGN_CAPACITY_MSB  0x0b

//#define NORMALPASSWORD_LSB  0x19e//0414
//#define NORMALPASSWORD_MSB  //3672
#define NORMALPASSWORD  0xe5819e//0414

#define I2CSLAVEADDR         0x55           // 7-bit slave address
#define DEVICE_NAME		    "bq27541"

struct bq27541_dev_info {
	struct device *dev;

	unsigned short voltage_raw;
	int voltage_uV;
	unsigned short current_raw;
	int current_uA;
	int battery_status;
	int charger_online;
	int charger_voltage_uV;
	int accum_current_uAh;
	int old_soc;

	int irq;

	struct power_supply bat;
	struct power_supply charger;

	struct i2c_client	*client;
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
};

static struct task_struct *usbdc_task;
bool bFastCharge = 0;
bool bBq27541Supend = false;
bool bq27541_flag_exchg = false;
struct bq27541_dev_info *di = NULL;
static struct mxc_bq27541_platform_data *plat_data;

static enum power_supply_property bq27541_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_MANUFACTURER,
	#ifdef BQ27541_MORE
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_TEMP,
	#endif
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
};

static  int bq27x00_battery_volt(struct bq27541_dev_info *di)
{
    /*
    int ret;

    ret = i2c_smbus_read_word_data(di->client, bq27541CMD_VOLT_LSB);
    return ret;
*/
	int ret = 0;
	unsigned char sbuffer[2] = {0,0};
	unsigned char rbuffer[2] = {0,0};
	sbuffer[0] = bq27541CMD_VOLT_LSB;
	ret = i2c_master_send(di->client, sbuffer, 1);
	if( ret < 0)
		return ret;
	ret = i2c_master_recv(di->client, rbuffer, 2);
	if( ret < 0)
		return ret;
	//printk("%s rbuffer[%d][%d] ret(%d)\n\n",__func__,rbuffer[0],rbuffer[1], ((rbuffer[1] << 8) | rbuffer[0]));
	return ((rbuffer[1] << 8) | rbuffer[0]);
}

static short bq27x00_battery_ai(struct bq27541_dev_info *di)
{
    /*
    short ret;

    ret = -5;
    ret = i2c_smbus_read_word_data(di->client, bq27541CMD_AI_LSB);
    return ret;
    */
	short ret = 0;
	unsigned char sbuffer[2] = {0,0};
	unsigned char rbuffer[2] = {0,0};
	sbuffer[0] = bq27541CMD_AI_LSB;
	ret = i2c_master_send(di->client, sbuffer, 1);
	if( ret < 0)
		return ret;
	ret = i2c_master_recv(di->client, rbuffer, 2);
	if( ret < 0)
		return ret;
	//printk("%s rbuffer[%d][%d] ret(%d)\n\n",__func__,rbuffer[0],rbuffer[1], ((rbuffer[1] << 8) | rbuffer[0]));
	ret = (rbuffer[1] << 8) | rbuffer[0];
	return ret;
}

static int bq27x00_battery_soc(struct bq27541_dev_info *di)
{
   /*
   	int ret;
   	ret = i2c_smbus_read_word_data(di->client, bq27541CMD_SOC_LSB);
    	return ret;
*/
 	int ret = 0;
	unsigned char sbuffer[2] = {0,0};
	unsigned char rbuffer[2] = {0,0};
	sbuffer[0] = bq27541CMD_SOC_LSB;
	ret = i2c_master_send(di->client, sbuffer, 1);
	if( ret < 0)
		return ret;
	ret = i2c_master_recv(di->client, rbuffer, 2);
	if( ret < 0)
		return ret;
	//printk("%s rbuffer[%d][%d] ret(%d)\n\n",__func__,rbuffer[0],rbuffer[1], ((rbuffer[1] << 8) | rbuffer[0]));
	return ((rbuffer[1] << 8) | rbuffer[0]);
}

static int bq27x00_battery_fac(struct bq27541_dev_info *di)
{
    /*
    int ret;

    ret = i2c_smbus_read_word_data(di->client, bq27541CMD_FAC_LSB);
    return ret;
*/
	int ret = 0;
	unsigned char sbuffer[2] = {0,0};
	unsigned char rbuffer[2] = {0,0};
	sbuffer[0] = bq27541CMD_FAC_LSB;
	ret = i2c_master_send(di->client, sbuffer, 1);
	if( ret < 0)
		return ret;
	ret = i2c_master_recv(di->client, rbuffer, 2);
	if( ret < 0)
		return ret;
	//printk("%s rbuffer[%d][%d] ret(%d)\n\n",__func__,rbuffer[0],rbuffer[1], ((rbuffer[1] << 8) | rbuffer[0]));
	return ((rbuffer[1] << 8) | rbuffer[0]);
}

static int bq27x00_battery_fcc(struct bq27541_dev_info *di)
{
    /*
    int ret;

    ret = i2c_smbus_read_word_data(di->client, bq27541CMD_FCC_LSB);
    return ret;
*/
	int ret = 0;
	unsigned char sbuffer[2] = {0,0};
	unsigned char rbuffer[2] = {0,0};
	sbuffer[0] = bq27541CMD_FCC_LSB;
	ret = i2c_master_send(di->client, sbuffer, 1);
	if( ret < 0)
		return ret;
	ret = i2c_master_recv(di->client, rbuffer, 2);
	if( ret < 0)
		return ret;
	//printk("%s rbuffer[%d][%d] ret(%d)\n\n",__func__,rbuffer[0],rbuffer[1], ((rbuffer[1] << 8) | rbuffer[0]));
	return ((rbuffer[1] << 8) | rbuffer[0]);
}

static int bq27x00_battery_rm(struct bq27541_dev_info *di)
{
    /*
    int ret;

    ret = i2c_smbus_read_word_data(di->client, bq27541CMD_RM_LSB);
    return ret;
*/
	int ret = 0;
	unsigned char sbuffer[2] = {0,0};
	unsigned char rbuffer[2] = {0,0};
	sbuffer[0] = bq27541CMD_RM_LSB;
	ret = i2c_master_send(di->client, sbuffer, 1);
	if( ret < 0)
		return ret;
	ret = i2c_master_recv(di->client, rbuffer, 2);
	if( ret < 0)
		return ret;
	//printk("%s rbuffer[%d][%d] ret(%d)\n\n",__func__,rbuffer[0],rbuffer[1], ((rbuffer[1] << 8) | rbuffer[0]));
	return ((rbuffer[1] << 8) | rbuffer[0]);
}

static int bq27x00_battery_cc(struct bq27541_dev_info *di)
{

  /*  int ret;

    ret = i2c_smbus_read_word_data(di->client, bq27541CMD_CC_LSB);
	printk("bq27541_battery_cc =%d \n",ret);
    return ret;
   */

	int ret = 0;
	unsigned char sbuffer[2] = {0,0};
	unsigned char rbuffer[2] = {0,0};
	sbuffer[0] = bq27541CMD_CC_LSB;
	ret = i2c_master_send(di->client, sbuffer, 1);
	if( ret < 0)
		return ret;
	ret = i2c_master_recv(di->client, rbuffer, 2);
	if( ret < 0)
		return ret;
    //printk("%s rbuffer[%d][%d] ret(%d)\n\n",__func__,rbuffer[0],rbuffer[1], ((rbuffer[1] << 8) | rbuffer[0]));
	return ((rbuffer[1] << 8) | rbuffer[0]);
}

static int bq27x00_battery_control_qen(struct bq27541_dev_info *di)
{
    /*
    int ret;
    int flag;
 //   i2c_smbus_read_word_data(di->client, bq27541CMD_CNTL_LSB);
	ret = i2c_smbus_read_word_data(di->client, bq27541CMD_CONTORL_STATUS);
	flag = ret  & 0x1;
    return flag;
    */
	int ret = 0;
	unsigned char sbuffer[2] = {0,0};
	unsigned char rbuffer[2] = {0,0};
	sbuffer[0] = bq27541CMD_CONTORL_STATUS;
	ret = i2c_master_send(di->client, sbuffer, 1);
	if( ret < 0)
		return ret;
	ret = i2c_master_recv(di->client, rbuffer, 2);
	if( ret < 0)
		return ret;
	//printk("%s rbuffer[%d][%d] ret(%d)\n\n",__func__,rbuffer[0],rbuffer[1], ((rbuffer[1] << 8) | rbuffer[0]));
	ret = 0x01 & ((rbuffer[1] << 8) | rbuffer[0]);
	return ret;
}

#define to_bq27541_dev_info(x) container_of((x), \
				struct bq27541_dev_info, bat);

static int bq27541_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq27541_dev_info *di = to_bq27541_dev_info(psy);

    //printk("%s\n",__func__);
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq27x00_battery_volt(di);
		if (psp == POWER_SUPPLY_PROP_PRESENT)
			val->intval = val->intval <= 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq27x00_battery_ai(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq27x00_battery_soc(di);
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		val->intval = bq27x00_battery_fac(di);
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		val->intval = bq27x00_battery_fcc(di);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		val->intval = bq27x00_battery_rm(di);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = bq27x00_battery_cc(di);
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = bq27x00_battery_control_qen(di)?"OK":"false";
		break;
#ifdef BQ27541_MORE
	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = bq27x00_battery_rm(di);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq27x00_battery_temp(di);
		break;
#endif
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
	case POWER_SUPPLY_PROP_HEALTH:
		//bq27x00_battery_test(di);
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->battery_status;

		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq27x00_battery_status(struct bq27541_dev_info *di)
{
	int ret;
	if (bq27x00_battery_ai(di) > 0)
		ret = POWER_SUPPLY_STATUS_CHARGING;
	else
		ret = POWER_SUPPLY_STATUS_DISCHARGING;
	if (bq27x00_battery_soc(di) == 100)
		ret = POWER_SUPPLY_STATUS_FULL;
	
	return ret;
}

void set_extcharger(bool enable);

static void bq27541_battery_update_status(struct bq27541_dev_info *di)
{
	int soc,bat_status;
	unsigned int value;
	//bq27541_battery_read_status(di);
	soc = bq27x00_battery_soc(di);
	//bq27x00_battery_cntl(di);
	bat_status = bq27x00_battery_status(di);
	if (di->old_soc!=soc || di->battery_status !=bat_status)
	{
		di->old_soc = soc;
		di->battery_status = bat_status;
		power_supply_changed(&di->bat);
	}
	if (soc == 100)
	{
        set_extcharger(1);//turn off charger led
        /*
		pmic_read_reg(REG_CHARGE, &value, 0xffffff);
		value &= ~0x40000;
		pmic_write_reg(REG_CHARGE, value, 0xffffff);

		//pmic_write_reg(REG_CHARGE, 1<<21, ((1U<<1)-1)<<21);
		//pmic_write_reg(REG_CHARGE, 0<<18, ((1U<<1)-1)<<18);
		*/
	}
}

void set_charging_current(unsigned short currt);

static void set_charge_current(void)
{
    if(bFastCharge) {
        return;
    } else if (fsl_usb_battery_charger == CHARGER_DCP) {
        set_charging_current(1100);
        set_extcharger(0);//enable exchager
        bFastCharge = 1;
    } else if(fsl_usb_battery_charger == CHARGER_ACTIVE_HOST){
        set_charging_current(3);
        set_extcharger(0);//enable exchager
        bFastCharge = 1;
    } else {
        set_charging_current(480);
        set_extcharger(0);//enable exchager
    }
}

static void bq27541_battery_work(struct work_struct *work)
{
	unsigned int value;
	struct bq27541_dev_info *di = container_of(work,
						     struct bq27541_dev_info,
						     monitor_work.work);
	const int interval = HZ * 10;
	pr_info("at %s\n",__func__);
	//bq27541_battery_update_status(di);
    if(!bBq27541Supend)
    {
	   /*if ((di->battery_status == POWER_SUPPLY_STATUS_CHARGING)
	   	&& (bq27x00_battery_ai(di)<0)) //something wrong with mc23892
	   {
    	   	pmic_read_reg(REG_CHARGE, &value, 0xffffff);
    	   	value |= 0x100000;
    	   	pmic_write_reg(REG_CHARGE, value, 0xffffff);
	   }*/
	   if(plat_data->ch_status()){
            set_charge_current();
        }
    	bq27541_battery_update_status(di);
    }
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}

static int usbdc_thread(void *unused)
{

	while(1)
	{
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
		
		msleep(200);

        	bFastCharge = 0;
		if (plat_data != NULL)
		{
			if (bq27x00_battery_ai(di) > 0) {
				di->battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
			} else {
				di->battery_status = POWER_SUPPLY_STATUS_CHARGING;
                		set_charge_current();
			}
		}

		if ( di != NULL)
		{
			power_supply_changed(&di->bat);
		}
	}
    return 0;
}

static irqreturn_t bq27541_interrupt(int irq, void *dev_id)
{
	wake_up_process(usbdc_task);

	return IRQ_RETVAL(IRQ_HANDLED);
}

static int bq27541_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int retval = 0;
	//struct bq27541_dev_info *di;
	int ret,ch_status;

	bBq27541Supend = false;

	plat_data = (struct mxc_bq27541_platform_data *)client->dev.platform_data;
	if (plat_data == NULL)
	{
		dev_err(&client->dev, "lack of platform data!\n");
		return -ENODEV;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		retval = -ENOMEM;
		goto di_alloc_failed;
	}

    	di->client = client;
	di->dev	= &client->dev;
	di->bat.name	= "bq27541";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27541_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27541_battery_props);
	di->bat.get_property = bq27541_battery_get_property;
	di->bat.use_for_apm = 1;

	di->old_soc = 0;

	di->battery_status = POWER_SUPPLY_STATUS_UNKNOWN;

	/*ret = bq27x00_battery_cc(di);
	if (ret < 0)
	{
		printk("bq27541 not found\n");
		return -ENODEV;
	}*/

	retval = power_supply_register(&client->dev, &di->bat);
	if (retval) {
		dev_err(di->dev, "failed to register battery\n");
		goto batt_failed;
	}
	INIT_DELAYED_WORK(&di->monitor_work, bq27541_battery_work);
	di->monitor_wqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!di->monitor_wqueue) {
		retval = -ESRCH;
		goto workqueue_failed;
	}
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ * 2);

	/* configure gpio as input for interrupt monitor */
	ret = request_irq(plat_data->irq, bq27541_interrupt,
		  IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, DEVICE_NAME, di);
	if (ret) {
		printk("bq27541 request_irq(%d) returned error %d\n",plat_data->irq, ret);
		goto workqueue_failed;
	}
	ch_status = plat_data->ch_status();
	if(ch_status) {
		printk("\n\nusb charger online\n");
		set_charging_current(480);
		set_extcharger(0);//enable exchager
	}
	//enable_irq(client->irq);

	usbdc_task = kthread_run(usbdc_thread, NULL, "usb_dc");
	if (IS_ERR(usbdc_task)) {
		pr_err("%s: request task failed\n", __func__);
		goto workqueue_failed;
	}
/*
	ret = request_irq(plat_data->irq_chg, bq27541_excharger_full_interrupt,IRQF_TRIGGER_FALLING, "excharger", NULL);
	if (ret) {
		printk("bq27541 request_irq(%d) returned error %d\n",plat_data->irq_chg, ret);
		goto di_alloc_failed;
	}
	enable_irq_wake(plat_data->irq_chg);
	*/
	goto success;
workqueue_failed:
	power_supply_unregister(&di->bat);
batt_failed:
	kfree(di);
di_alloc_failed:
success:
	printk("%s battery probed!\n", __func__);
	return retval;
}


static int bq27541_battery_remove(struct i2c_client *client)
{

   /*
	struct bq27x00_device_info *di = i2c_get_clientdata(client);
	power_supply_unregister(&di->bat);
	kfree(di->bat.name);
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);
	kfree(di);
*/
	return 0;
}

static int bq27541_battery_suspend(struct i2c_client *client, pm_message_t mesg)
{
	bq27541_flag_exchg = false;
	bBq27541Supend = true;
	return 0;
}

static int bq27541_battery_resume(struct i2c_client *client)
{
    	printk("%s\n",__func__);
	bBq27541Supend = false;
	power_supply_changed(&di->bat);
	return 0;
}
/*
* Module stuff
*/
static const struct i2c_device_id bq27541_id[] = {
	{ "bq27541", 0 },
	{},
};

static struct i2c_driver bq27541_battery_driver = {
	.driver = {
		.name = "bq27541",
	},
	.probe = bq27541_battery_probe,
	.remove = bq27541_battery_remove,
	.id_table = bq27541_id,
	.suspend = bq27541_battery_suspend,
	.resume = bq27541_battery_resume,
};

static int __init bq27x00_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27541_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27541 driver\n");
	return ret;
}

//module_init(bq27x00_battery_init);
late_initcall(bq27x00_battery_init);

static void __exit bq27x00_battery_exit(void)
{
	i2c_del_driver(&bq27541_battery_driver);

	kthread_stop(usbdc_task);
}

module_exit(bq27x00_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ272541 battery monitor driver");
MODULE_LICENSE("GPL");

