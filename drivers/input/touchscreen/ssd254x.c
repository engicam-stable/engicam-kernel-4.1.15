/*
 * Copyright (C) 2018 Matteo Lisi, <matteo.lisi@engicam.com>
  *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * This is a simple implementation of driver for the ssd254x touch controller.
 *
 */

#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>

#define SCREEN_MAX_X                  1024 //800
#define SCREEN_MAX_Y                  600 //480
#define FINGERNO		              4
#define DEVICE_ID_REG                 2
#define VERSION_ID_REG                3
#define EVENT_STATUS                  0x79
#define FINGER01_REG                  0x7c
#define MAX_AREA	0xff

struct ChipSetting {
	char No;
	char Reg;
	char Data1;
	char Data2;
};

struct ChipSetting ssd254x_cfgTable[] = {
{2,0x06,0x19,0x0F},
{2,0x07,0x00,0xE0},
{2,0x08,0x00,0xE1},
{2,0x09,0x00,0xE2},
{2,0x0A,0x00,0xE3},
{2,0x0B,0x00,0xE4},
{2,0x0C,0x00,0xE5},
{2,0x0D,0x00,0xE6},
{2,0x0E,0x00,0xE7},
{2,0x0F,0x00,0xE8},
{2,0x10,0x00,0xE9},
{2,0x11,0x00,0xEA},
{2,0x12,0x00,0xEB},
{2,0x13,0x00,0xEC},
{2,0x14,0x00,0xED},
{2,0x15,0x00,0xEE},
{2,0x16,0x00,0xEF},
{2,0x17,0x00,0xF0},
{2,0x18,0x00,0xF1},
{2,0x19,0x00,0xF2},
{2,0x1A,0x00,0xF3},
{2,0x1B,0x00,0xF4},
{2,0x28,0x00,0x14},

{2,0x30,0x08,0x0F},
{2,0xD7,0x00,0x03},
{2,0xD8,0x00,0x06},
{2,0xDB,0x00,0x03},

{2,0x33,0x00,0x01},
{2,0x34,0xC6,0x60},
{2,0x36,0x00,0x20},
{2,0x37,0x07,0xC4},

{2,0x40,0x10,0xC8},
{2,0x41,0x00,0x30},
{2,0x42,0x00,0x50},
{2,0x43,0x00,0x30},
{2,0x44,0x00,0x50},
{2,0x45,0x00,0x00},
{2,0x46,0x10,0x1F},

{2,0x56,0x80,0x10},
{2,0x59,0x80,0x10},

{2,0x65,0x00,0x05},
{2,0x66,0x25,0x80},
{2,0x67,0x27,0x60},

{2,0x7A,0xFF,0xFF},
{2,0x7B,0x00,0x03},

{2,0x25,0x00,0x0C},
};


struct ChipSetting Reset[]={
{ 2, 0x01, 0x00, 0x00},
};

struct ChipSetting Resume[]={
{ 2, 0x04, 0x00, 0x00},
{ 2, 0x25, 0x00, 0x06},
};

struct ChipSetting Suspend[] ={
{ 2, 0x25, 0x00, 0x00},
{ 2, 0x05, 0x00, 0x00},
};

struct ssd254x_platform_data {
	int irq_pin;
	int reset_pin;
};



struct ssd254x_ts_data {
	struct i2c_client *client;
	struct input_dev *input;

	int reset_pin;
	int irq_pin;
	int wake_pin;

	struct mutex mutex;	
	int product;
	int version;

	char name[10];
    
    int FingerX[FINGERNO];
	int FingerY[FINGERNO];
	int FingerP[FINGERNO];

	int EventStatus;
};



static int ssd254x_ts_readwrite(struct i2c_client *client,
				   u16 wr_len, u8 *wr_buf,
				   u16 rd_len, u8 *rd_buf)
{
	struct i2c_msg wrmsg[2];
	int i = 0;
	int ret;

	if (wr_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = 0;
		wrmsg[i].len = wr_len;
		wrmsg[i].buf = wr_buf;
		i++;
	}
	if (rd_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = I2C_M_RD;
		wrmsg[i].len = rd_len;
		wrmsg[i].buf = rd_buf;
		i++;
	}

	ret = i2c_transfer(client->adapter, wrmsg, i);
	if (ret < 0)
		return ret;
	if (ret != i)
		return -EIO;

	return 0;
}


static int ssd254x_registers_read(struct ssd254x_ts_data *tsdata,
				    u8 addr,u8 len)
{
	u8 wrbuf[2], rdbuf[4];
	int error;
	
	memset(rdbuf,0,sizeof(rdbuf));
	wrbuf[0] = addr;
	error = ssd254x_ts_readwrite(tsdata->client, 1,
					wrbuf, len, rdbuf);
	if (error)	
		return error;

	return (int)((unsigned int)rdbuf[0]<<0)|
				((unsigned int)rdbuf[1]<<8)|
				((unsigned int)rdbuf[2]<<16)|
				(rdbuf[3]<<24);
}


static int ssd254x_swap_registers_read(struct ssd254x_ts_data *tsdata,
				    u8 addr,u8 len)
{
	u8 wrbuf[2], rdbuf[4];
	int error;
	
	memset(rdbuf,0,sizeof(rdbuf));
	wrbuf[0] = addr;
	error = ssd254x_ts_readwrite(tsdata->client, 1,
					wrbuf, len, rdbuf);
	if (error)	
		return error;

	return (int)((unsigned int)rdbuf[3]<<0)|
				((unsigned int)rdbuf[2]<<8)|
				((unsigned int)rdbuf[1]<<16)|
				(rdbuf[0]<<24);
}



static irqreturn_t ssd254x_ts_isr(int irq, void *dev_id)
{
    int i;
	unsigned short xpos=0, ypos=0, width=0;
	unsigned int FingerInfo;
	int EventStatus;
	int FingerX[FINGERNO];
	int FingerY[FINGERNO];
	int FingerP[FINGERNO];
	int FingerDetect=0;
    struct ssd254x_ts_data *tsdata = dev_id;
       
	EventStatus = ssd254x_registers_read(tsdata,EVENT_STATUS,2)>>12;


	for(i=0;i<FINGERNO;i++) {
		if((EventStatus>>i)&0x1) {
            FingerInfo=ssd254x_swap_registers_read(tsdata,FINGER01_REG+i,4);
    
			ypos = ((FingerInfo>>0)&0xF00)|((FingerInfo>>16)&0xFF);
			xpos = ((FingerInfo>>4)&0xF00)|((FingerInfo>>24)&0xFF);
			width= FingerInfo & 0x0FF;				

			if(xpos!=0xFFF) {
				FingerDetect++;
			}	else {
				EventStatus=EventStatus&~(1<<i);
			}
		} else {
			xpos=ypos=0xFFF;
			width=0;
		}

        FingerX[i]=xpos;
        FingerY[i]=ypos;
        FingerP[i]=width;
	}

	for(i=0;i<FINGERNO;i++)	{
		xpos=FingerX[i];
		ypos=FingerY[i];
		width=FingerP[i];

        input_mt_slot(tsdata->input, i);
        
		if(xpos!=0xFFF)	{			
            input_mt_report_slot_state(tsdata->input, MT_TOOL_FINGER, 1);
            input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, width);
            input_report_abs(tsdata->input, ABS_MT_POSITION_X, xpos);
            input_report_abs(tsdata->input, ABS_MT_POSITION_Y, ypos);
            
		} else {
            input_mt_report_slot_state(tsdata->input, MT_TOOL_FINGER, 0);
        }
        
        tsdata->FingerX[i]=FingerX[i];
		tsdata->FingerY[i]=FingerY[i];
		tsdata->FingerP[i]=width;
	}

    input_mt_report_pointer_emulation(tsdata->input, true);
	input_sync(tsdata->input);
    
	return IRQ_HANDLED;
}


static unsigned int ssd254x_ts_identify(struct i2c_client *client,
					struct ssd254x_ts_data *tsdata)
{

	tsdata->product = ssd254x_registers_read(tsdata,DEVICE_ID_REG,1);
	tsdata->version = ssd254x_registers_read(tsdata,VERSION_ID_REG,1);

	printk("[ML] ssd254x Touchscreen Device ID  : 0x%04X\n",tsdata->product);
	printk("[ML] ssd254x Touchscreen Version ID : 0x%04X\n",tsdata->version);

	return 0;
}

#ifdef CONFIG_OF
static int ssd254x_i2c_ts_probe_dt(struct device *dev,
				struct ssd254x_ts_data *tsdata)
{
	struct device_node *np = dev->of_node;
	
	printk("[ML] %s %d\n",__FUNCTION__,__LINE__);

    tsdata->irq_pin = -EINVAL;
	tsdata->reset_pin = of_get_named_gpio(np, "reset-gpios", 0);
	tsdata->wake_pin = of_get_named_gpio(np, "wake-gpios", 0);

	return 0;
}
#else
static inline int ssd254x_i2c_ts_probe_dt(struct device *dev,
					struct ssd254x_ts_data *tsdata)
{
	return -ENODEV;
}
#endif



static int sssd254x_device_sw_reset(struct i2c_client *client)
{
	int i;
	u8 number;
	u8 wrbuf[4];
        
    
	for(i=0;i<sizeof(Reset)/sizeof(Reset[0]);i++)	{		
			number=Reset[i].No+1;			
			wrbuf[0] = Reset[i].Reg ;
			wrbuf[1] = Reset[i].Data1;
			wrbuf[2] = Reset[i].Data2;
			ssd254x_ts_readwrite(client, number,wrbuf, 0, NULL);				
			udelay(500);
	}
	
	
	udelay(2000);
    //udelay(2000);
    //udelay(2000);
    
    return 0;
}

static int ssd254x_device_init(struct i2c_client *client)
{
	int i;
	u8 number;
	u8 wrbuf[4];
        
    
	for(i=0;i<sizeof(ssd254x_cfgTable)/sizeof(ssd254x_cfgTable[0]);i++)	{		
			number=ssd254x_cfgTable[i].No+1;			
			wrbuf[0] = ssd254x_cfgTable[i].Reg ;
			wrbuf[1] = ssd254x_cfgTable[i].Data1;
			wrbuf[2] = ssd254x_cfgTable[i].Data2;
			ssd254x_ts_readwrite(client, number,wrbuf, 0, NULL);				
			udelay(500);
	}

    return 0;
}

static int ssd254x_ts_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{

	const struct ssd254x_platform_data *pdata =
						dev_get_platdata(&client->dev);
	struct ssd254x_ts_data *tsdata;
	struct input_dev *input;
	int error;
	
	dev_dbg(&client->dev, "probing for ssd254 I2C\n");
	
	tsdata = devm_kzalloc(&client->dev, sizeof(*tsdata), GFP_KERNEL);
    
	if (!tsdata) {
		dev_err(&client->dev, "failed to allocate driver data.\n");
		return -ENOMEM;
	}

	if (!pdata) {
		error = ssd254x_i2c_ts_probe_dt(&client->dev, tsdata);
		if (error) {
			dev_err(&client->dev,
				"DT probe failed and no platform data present\n");
			return error;
		}
	} else {
		tsdata->reset_pin = pdata->reset_pin;
		tsdata->irq_pin = pdata->irq_pin;
		tsdata->wake_pin = -EINVAL;
	}

	
	error = sssd254x_device_sw_reset(client);

	if (error)
		return error;

	
	error = ssd254x_device_init(client);
    
	if (error)
		return error;

	if (gpio_is_valid(tsdata->irq_pin)) {
		error = devm_gpio_request_one(&client->dev, tsdata->irq_pin,
					GPIOF_IN, "ssd254 irq");
		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d, error %d\n",
				tsdata->irq_pin, error);
			return error;
		}
	}

	input = devm_input_allocate_device(&client->dev);
	if (!input) {
		dev_err(&client->dev, "failed to allocate input device.\n");
		return -ENOMEM;
	}

	mutex_init(&tsdata->mutex);
	tsdata->client = client;
	tsdata->input = input;
	
	error = ssd254x_ts_identify(client, tsdata);
	if (error) {
		dev_err(&client->dev, "touchscreen probe failed\n");
		return error;
	}

	sprintf(tsdata->name,"ssd254x");
	input->name = tsdata->name;
	
	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	
        /* Single touch */
    input_set_abs_params(input, ABS_X, 0, ABS_MT_POSITION_X, 0, 0);
    input_set_abs_params(input, ABS_Y, 0, ABS_MT_POSITION_Y, 0, 0);

        /* Multi touch */
	input_mt_init_slots(input, FINGERNO,0);
 	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, MAX_AREA, 0, 0);	
    input_set_abs_params(input, ABS_MT_POSITION_X,  0,SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y,  0,SCREEN_MAX_Y, 0, 0);

    input_set_drvdata(input, tsdata);
	
	error = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					ssd254x_ts_isr,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					client->name, tsdata);
	if (error) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		return error;
	}


	error = input_register_device(input);
	if (error)
		return error;

    i2c_set_clientdata(client, tsdata);
	device_init_wakeup(&client->dev, 1);

	return 0;
}

static int ssd254x_ts_remove(struct i2c_client *client)
{
	return 0;
}

static int __maybe_unused ssd254x_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int __maybe_unused ssd254x_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(client->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(ssd254x_ts_pm_ops,
			 ssd254x_ts_suspend, ssd254x_ts_resume);

static const struct i2c_device_id ssd254x_ts_id[] = {
	{ "ssd254x", 0, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ssd254x_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id ssd254x_of_match[] = {
	{ .compatible = "ssd254x", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ssd254x_of_match);
#endif

static struct i2c_driver ssd254x_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ssd254x",
		.of_match_table = of_match_ptr(ssd254x_of_match),
		.pm = &ssd254x_ts_pm_ops,
	},
	.id_table = ssd254x_ts_id,
	.probe    = ssd254x_ts_probe,
	.remove   = ssd254x_ts_remove,
};

module_i2c_driver(ssd254x_ts_driver);

MODULE_AUTHOR("Matteo Lisi <matteo.lisi@engicam.com>");
MODULE_DESCRIPTION("ssd254 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
