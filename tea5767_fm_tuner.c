/***
 * Copyright (C) 2020 Mustafa Ozcelikors <mozcelikors@gmail.com>
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU General Public License
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/
 *
 * Description:
 *     Linux I2C Device Driver (Kernel Module) for TEA5767 Radio Tuner Module
 *
 **/

/* To be able to use 
   modprobe i2c_dev, 
   and then e.g.:
   cat /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/mute
   cat /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/station
   cat /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/stationlist
   echo "1" > /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/mute
   echo "0" > /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/mute
   echo 97200 > /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/station
*/

/* Add to your devicetree as such:
   diff --git a/arch/arm/boot/dts/bcm2711-rpi-4-b.dts b/arch/arm/boot/dts/bcm2711-rpi-4-b.dts
   index 18c0f9599d3a..bff7c0830af3 100644
   --- a/arch/arm/boot/dts/bcm2711-rpi-4-b.dts
   +++ b/arch/arm/boot/dts/bcm2711-rpi-4-b.dts
   @@ -314,6 +314,12 @@
    	pinctrl-names = "default";
    	pinctrl-0 = <&i2c1_pins>;
    	clock-frequency = <100000>;
   +        status = "okay";
   +
   +        tea5767@60 {
   +            compatible = "mozcelikors,tea5767";
   +            reg = <0x60>;
   +            minstation = <88000>;
   +            maxstation = <108000>;
   +        };
    };
*/

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

#define READY_WAIT_TIME 15000
#define HCC_DEFAULT 1
#define SNC_DEFAULT 1
#define SEARCH_MODE_DEFAULT 2
#define FORCED_MONO 0
#define MAX_STATION 50

#define TEA5767_DEBUG 1

/* Driver struct */
struct tea5767_dev {
	struct i2c_client * client;
	char name[8]; // tea5767XX

	int frequency;
	int mute;
        int station_list[MAX_STATION];
	int station_count;

	u32 minstation; //min possible frequency, default: 88000
	u32 maxstation; //max possible frequency, default: 108000

        struct task_struct *scan_task; 	/* kthread task_struct */
        u8 scan_task_flag;
        spinlock_t station_list_lock;
};

/* Function prototypes */
/* Utility functions - Scan functions, Not Essential to the driver */
static int tea5767_get_wrapped_frequency (struct i2c_client * client, int freq);
static int tea5767_get_device_frequency (struct i2c_client * client);
static int tea5767_search (struct i2c_client * client, int dir, int mode, int forced_mono);
static int tea5767_wait_ready (struct i2c_client * client);
static int tea5767_get_status (struct i2c_client * client, int *freq, unsigned char *mode, unsigned char *level_adc);
static int tea5767_scan_frequencies (struct i2c_client * client, int mode, int forced_mono);
/* Kthread task function for frequency scanning*/
static int scan_kthread_task (void *data);
/* Utility functions - Essential functions */
static int tea5767_set_freq (struct i2c_client * client, int freq, int hcc, int snc, unsigned char forcd_mono, int mute, int standby);
/* Sysfs functions and attrs */
static ssize_t sysfs_store_station_callback (struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t sysfs_show_station_callback (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t sysfs_store_mute_callback (struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t sysfs_show_mute_callback (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t sysfs_show_stationlist_callback (struct device *dev, struct device_attribute *attr, char *buf);
/* Module enter and exit */
static int tea5767_probe(struct i2c_client * client, const struct i2c_device_id * id);
static int tea5767_remove(struct i2c_client * client);

/* Utility functions - Scan functions, Not essential, but nice to have */
static int tea5767_get_wrapped_frequency (struct i2c_client * client, int freq)
{
	struct tea5767_dev * tea5767;
	tea5767 = i2c_get_clientdata(client);

	if (freq < tea5767->minstation || freq > tea5767->maxstation)
	{
		if (freq > tea5767->maxstation)
			freq = tea5767->minstation;
		else if (freq < tea5767->minstation)
			freq= tea5767->maxstation;
	}

	return freq;
}

static int tea5767_get_device_frequency (struct i2c_client * client)
{
	struct tea5767_dev * tea5767;
	tea5767 = i2c_get_clientdata(client);

#ifdef TEA5767_DEBUG
	dev_info(&client->dev, "tea5767_get_device_frequency is entered \n");
#endif
	unsigned char radio[5];
	int freq;

	if(-1 == i2c_master_recv(client, radio, 5))
	{
		dev_err(&client->dev, "tea5767_get_device_frequency i2c_master_recv failed 1\n");
		return -1;
	}
	freq = ((((radio[0] & 0x3F) << 8) + radio[1]) * 32768 / 4 - 225000) / 10000;
	freq = freq * 10;

#ifdef TEA5767_DEBUG
	dev_info(&client->dev, "tea5767_get_device_frequency returned with %d \n", freq);
#endif
	return freq;
}

static int tea5767_search (struct i2c_client * client, int dir, int mode, int forced_mono)
{
	struct tea5767_dev * tea5767;
	tea5767 = i2c_get_clientdata(client);

#ifdef TEA5767_DEBUG
	dev_info(&client->dev, "tea5767_search is entered \n");
#endif
	int freq;

	freq = tea5767_get_device_frequency(client);
#ifdef TEA5767_DEBUG
	dev_info(&client->dev, "tea5767_search device frequency %d \n", freq);
#endif
	if (dir) freq += 100;
	else freq -= 100;

	freq = tea5767_get_wrapped_frequency(client, freq);
#ifdef TEA5767_DEBUG
	dev_info(&client->dev, "tea5767_search wrapped and incremented frequency %d \n", freq);
#endif
	unsigned char radio[5];     // Radio registers
	unsigned char frequencyH;   // Frequency High portion
	unsigned char frequencyL;   // Frequency Low portion
	unsigned int frequencyB;
	frequencyB = 4 * (freq * 1000 + 225000) / 32768; //calculating PLL word
	frequencyH = frequencyB >> 8;
	frequencyH = frequencyH | 0x40; // triggers search
	frequencyL = frequencyB & 0xFF;

	//nothing to set in array #2 as up is the normal search direction
	radio[0] = frequencyH; //FREQUENCY H
	radio[0] |= 0x80; // MUTE
	radio[1] = frequencyL; //FREQUENCY L
	radio[2] = 0x10; // high side LO injection is on,.
	radio[2] |= (mode & 0x03) << 5; // high side LO injection is on,.
	if (dir) radio[2] |= 0x80;	// search up/down
	if (forced_mono) radio[2] |= 0x08;
	radio[3] = 0x10; // Xtal is 32.768 kHz
	//if (freq < 87.5) radio[3] |= 0x20;
	if (HCC_DEFAULT) radio[3] |= 0x04;
	if (SNC_DEFAULT) radio[3] |= 0x02;
	radio[4] = 0x40; // deemphasis is 75us in Korea and US

	/* Write to i2c */
        if(-1 == i2c_master_send(client, radio, 5))
        {
		return -1;
        }

	tea5767_wait_ready(client);

	return 0;
}

static int tea5767_wait_ready (struct i2c_client * client)
{
	struct tea5767_dev * tea5767;
	tea5767 = i2c_get_clientdata(client);

#ifdef TEA5767_DEBUG
	dev_info(&client->dev, "tea5767_wait_ready is entered \n");
#endif
	unsigned char radio[5];
	int loop;

	loop = 0;
	if(-1 == i2c_master_recv(client, radio, 5))
	{
		dev_err(&client->dev, "tea5767_wait_ready i2c_master_recv failed 1\n");
		return -1;
	}

	while((radio[0] & 0x80) == 0 && loop < 2000000/READY_WAIT_TIME)  // max 2 sec
	{
		usleep_range(READY_WAIT_TIME, READY_WAIT_TIME+100);
		if(-1 == i2c_master_recv(client, radio, 5))
		{
			dev_err(&client->dev, "tea5767_wait_ready i2c_master_recv failed 2\n");
			return -1;
		}
		loop++;
	}

	if ((radio[0] & 0x80) == 0) // Ready fail
	{
		return -1;
	}
	else if (radio[0] & 0x40) // Band limit reached
	{
		return -2;
	}

#ifdef TEA5767_DEBUG
	dev_info(&client->dev, "tea5767_wait_ready is exited \n");
#endif
	return 0;
}

static int tea5767_get_status (struct i2c_client * client, int *freq, unsigned char *mode, unsigned char *level_adc)
{
	struct tea5767_dev * tea5767;
	tea5767 = i2c_get_clientdata(client);

#ifdef TEA5767_DEBUG
	dev_info(&client->dev, "tea5767_get_status is entered \n");
#endif
	unsigned char radio[5];

	if(-1 == i2c_master_recv(client, radio, 5))
	{
		dev_err(&client->dev, "tea5767_get_status i2c_master_recv failed\n");
		return -1;
	}
	*freq = ((((radio[0] & 0x3F) << 8) + radio[1]) * 32768 / 4 - 225000) / 10000;
	*freq = *freq * 10;

	*mode = radio[2] & 0x80;
	*level_adc = radio[3] >> 4;

	return 0;
}

static int tea5767_scan_frequencies (struct i2c_client * client, int mode, int forced_mono)
{
	struct tea5767_dev * tea5767;
	tea5767 = i2c_get_clientdata(client);

#ifdef TEA5767_DEBUG
	dev_info(&client->dev, "tea5767_scan_frequencies is entered \n");
#endif

	/* Start kthread for scanning frequency - nonblocking scan via kthread */
	if (tea5767->scan_task_flag == 0)
	{
		tea5767->scan_task = kthread_run(scan_kthread_task, client, "scan_kthread_task");
		if(IS_ERR(tea5767->scan_task))
		{
			dev_info(&client->dev, "Failed to create the task\n");
			return PTR_ERR(tea5767->scan_task);
		}
	}
	else
		return -EBUSY;

	tea5767->scan_task_flag = 1;

	return 1;
}

/* Kthread task function for frequency scanning*/
static int scan_kthread_task (void *data)
{
	struct tea5767_dev * tea5767;
	struct i2c_client *client = data;
	tea5767 = i2c_get_clientdata(client);

	int freq;
	freq = tea5767->minstation;
	unsigned char stereo, level_adc;
	int count;
        count = 0;
 
#ifdef TEA5767_DEBUG
	dev_info(&client->dev, "scan_kthread_task started\n");
#endif
	// Mute
	tea5767_set_freq(client, freq, HCC_DEFAULT, SNC_DEFAULT, FORCED_MONO, 1, 0);
	tea5767_wait_ready(client);

	do {
		if (tea5767_search(client, 1, SEARCH_MODE_DEFAULT, FORCED_MONO)) break;

		if (-1 == tea5767_get_status(client, &freq, &stereo, &level_adc))
		{
			dev_err(&client->dev, "tea5767_scan_frequencies Unable to get status\n");
		}
		if (freq >= tea5767->maxstation) break;
	
		// Add found station frequency to the station list
		// Protect the variable (CS) that we write to in kthread.
		spin_lock(&tea5767->station_list_lock);
		tea5767->station_list[count] = freq;
		spin_unlock(&tea5767->station_list_lock);
#ifdef TEA5767_DEBUG
		dev_info(&client->dev, "Scanned frequency Nr %d: %d \n", count, freq);
#endif
		count++;
	} 
	while(freq < tea5767->maxstation && count < MAX_STATION);

	// Protect the variable (CS) that we write to in kthread.
	// Use spin_lock_irqsave/spin_lock_irqrestore if the variable is read/written in atomic context (ISR)
	// If not, you can use spin_lock or mutex_lock for process context variables. mutex_lock is more preferred for process context.
	// spin_lock wastes CPU cycles, preventing process to go sleep, so more real-time oriented than mutex.
	spin_lock(&tea5767->station_list_lock);
	tea5767->station_count = count;
	spin_unlock(&tea5767->station_list_lock);

	// We are exiting the kthread, adjusting flag accordingly.
	tea5767->scan_task_flag = 0;

	// Restore the latest station
	tea5767_set_freq(client, tea5767->frequency, 1, 1, 0, tea5767->mute, 0);

	dev_info(&client->dev, "scan_kthread_task completed\n");
	return 0;
}

/* Utility functions - Essential functions */
static int tea5767_set_freq (struct i2c_client * client, int freq, int hcc, int snc, unsigned char forcd_mono, int mute, int standby)
{
#ifdef TEA5767_DEBUG
        dev_info(&client->dev, "tea5767_set_freq is entered \n");
#endif
	/* Calculate register contents */
	unsigned char radio[5];     // Radio registers
	unsigned char frequencyH;   // Frequency High portion
	unsigned char frequencyL;   // Frequency Low portion
	unsigned int frequencyB;
	frequencyH = 0;
	frequencyL = 0;
	frequencyB = 0;

        frequencyB = 4 * (freq * 1000 + 225000) / 32768; //calculating PLL word
        frequencyH = frequencyB >> 8;
        frequencyL = frequencyB & 0xFF;

        radio[0] = frequencyH; //FREQUENCY H
        if (mute) radio[0] |= 0x80;
        radio[1] = frequencyL; //FREQUENCY L
        radio[2] = 0xB0; //3 byte (0xB0): high side LO injection is on,.
        if (forcd_mono) radio[2] |= 0x08;
        radio[3] = 0x10; // Xtal is 32.768 kHz
        if (freq < 87500) radio[3] |= 0x20;
        if (hcc) radio[3] |= 0x04;
        if (snc) radio[3] |= 0x02;
        if (standby) radio[3] |= 0x40;
        radio[4] = 0x40; // deemphasis is 75us in Korea and US

	/* Write to i2c */
        if(-1 == i2c_master_send(client, radio, 5))
        {
		return -1;
        }

#ifdef TEA5767_DEBUG
	dev_info(&client->dev, "tea5767_set_freq is exited.\n");
#endif
	return 0;
}

/* Sysfs functions and attrs */
/* 
   sysfs_store_station_callback is called when sysfs station attribute is written to with e.g.:
   echo 97200 > /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/station
*/
static ssize_t sysfs_store_station_callback (struct device *dev, struct device_attribute *attr,
			                      const char *buf, size_t count)
{
	struct tea5767_dev * tea5767;
	struct i2c_client *client;
	char *buffer;
	unsigned long val;
	int ret;

	client = to_i2c_client(dev);

	/* Get device structure from bus device context */	
	tea5767 = i2c_get_clientdata(client);

	// Prepare and terminate buffer string
	buffer = buf;
	*(buffer+(count-1)) = '\0';

	// convert the string to an unsigned long
	ret = kstrtoul(buffer, 0, &val);
	if (ret)
	{
		dev_err(&client->dev, "Bad station value. Use an integer between 88000 and 108000.\n");
		return -EINVAL;
	}
	else
	{
		dev_info(&client->dev, "Setting station to %d kHz.\n",(int) val);
	}

	/* Set frequency */
	tea5767->frequency = (int) val;

	/* Tune into frequency */
	tea5767_set_freq(client, tea5767->frequency, 1, 1, 0, tea5767->mute, 0);

	return count;
}

/* 
   sysfs_show_station_callback is called when sysfs station attribute is read e.g.:
   cat /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/station
*/
static ssize_t sysfs_show_station_callback (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tea5767_dev * tea5767;

	struct i2c_client *client;

	client = to_i2c_client(dev);

	/* Get device structure from bus device context */	
	tea5767 = i2c_get_clientdata(client);

	dev_info(&client->dev, "sysfs_show_station_callback: Userspace client is reading from sysfs\n");
	return sprintf(buf, "%d", tea5767->frequency);
}


/* 
   sysfs_store_mute_callback is called when sysfs mute attribute is written to with e.g.:
   echo 1 > /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/mute
*/
static ssize_t sysfs_store_mute_callback (struct device *dev, struct device_attribute *attr,
			                      const char *buf, size_t count)
{
	struct tea5767_dev * tea5767;
	struct i2c_client *client = to_i2c_client(dev);

	/* Get device structure from bus device context */	
	tea5767 = i2c_get_clientdata(client);

	char *buffer;
	buffer = buf;

	*(buffer+(count-1)) = '\0';

	if(!strcmp(buffer, "1"))
	{
		tea5767->mute = 1;
		tea5767_set_freq(client, tea5767->frequency, 1, 1, 0, tea5767->mute, 0);
		// Sleep example:
		//usleep_range(100, 200);
	}
	else if (!strcmp(buffer, "0"))
	{
		tea5767->mute = 0;
		tea5767_set_freq(client, tea5767->frequency, 1, 1, 0, tea5767->mute, 0);
	}
	else
	{
		dev_err(&client->dev, "Bad mute value. Use 1 or 0.\n");
		return -EINVAL;
	}

	return count;
}

/* 
   sysfs_show_mute_callback is called when sysfs station attribute is read e.g.:
   cat /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/mute
*/
static ssize_t sysfs_show_mute_callback (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tea5767_dev * tea5767;
	struct i2c_client *client;

	client = to_i2c_client(dev);
	/* Get device structure from bus device context */	
	tea5767 = i2c_get_clientdata(client);

	dev_info(&client->dev, "sysfs_show_mute_callback: Userspace client is reading from sysfs.\n");
	return sprintf(buf, "%d", tea5767->mute);
}

/* 
   sysfs_show_stationlist_callback is called when sysfs stationlist attribute is read e.g.:
   cat /sys/class/i2c-dev/i2c-1/device/1-0060/tea5767/stationlist
*/
static ssize_t sysfs_show_stationlist_callback (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tea5767_dev * tea5767;
	struct i2c_client *client;

	int buf_len;
	buf_len = 0;

	client = to_i2c_client(dev);
	/* Get device structure from bus device context */	
	tea5767 = i2c_get_clientdata(client);

	dev_info(&client->dev, "sysfs_show_stationlist_callback: Userspace client is reading from sysfs.\n");

	// Use spin_lock_irqsave/spin_lock_irqrestore if the variable is read/written in atomic context (ISR)
	// If not, you can use spin_lock or mutex_lock for process context variables. mutex_lock is more preferred for process context.
	// spin_lock wastes CPU cycles, preventing process to go sleep, so more real-time oriented than mutex.
	spin_lock(&tea5767->station_list_lock);

	if (tea5767->station_count > 0)
	{
		int i;
		for (i=0; i < tea5767->station_count; i++)
		{
			if (i == 0)
				buf_len = sprintf(buf, "%d,", tea5767->station_list[i]);
			else if (i == tea5767->station_count-1)
				buf_len = sprintf(buf, "%s%d", buf, tea5767->station_list[i]);  // Concat , without the comma
			else
				buf_len = sprintf(buf, "%s%d,", buf, tea5767->station_list[i]); // Concat , with the comma
		}
		spin_unlock(&tea5767->station_list_lock);
		return buf_len;
	}
	else
		spin_unlock(&tea5767->station_list_lock);
		return 0;
}

/* Following creates sysfs attributes */
// S_IRUGO -> show permission
// S_IWUSR -> store permission
static DEVICE_ATTR(station, S_IWUSR | S_IRUGO, sysfs_show_station_callback, sysfs_store_station_callback); // Create sysfs attribute "station"

static DEVICE_ATTR(mute, S_IWUSR | S_IRUGO, sysfs_show_mute_callback, sysfs_store_mute_callback); // Create sysfs attribute "mute"

static DEVICE_ATTR(stationlist, S_IRUGO, sysfs_show_stationlist_callback, NULL); // Create sysfs attribute "stationlist"

static struct attribute *tea5767_sysfs_attrs[] = {
	&dev_attr_station.attr,
	&dev_attr_mute.attr,
	&dev_attr_stationlist.attr,
	NULL,
};

/* Following creates sysfs group that will be registered and deregistered in module enter and exit. */
static struct attribute_group tea5767_sysfs_group = {
	.name = "tea5767",
	.attrs = tea5767_sysfs_attrs,
};

/* Module enter and exit */
static int tea5767_probe(struct i2c_client * client,
		const struct i2c_device_id * id)
{
	int ret = -1;
	unsigned long val;

	struct tea5767_dev * tea5767;
	struct device *dev = &client->dev; // Here only for reference

	/* Allocate new structure representing device */
	tea5767 = devm_kzalloc(&client->dev, sizeof(struct tea5767_dev), GFP_KERNEL);

	/* Store pointer to the device-structure in bus device context */
	i2c_set_clientdata(client,tea5767);

	/* Store pointer to I2C device/client */
	tea5767->client = client;

	/* Parse properties of the node - an example of how to parse devicetree properties */
	/* More ref: https://elixir.bootlin.com/linux/v4.0/source/include/linux/property.h */
	/* Use of_property_read if you want to use device node descriptor instead of device object for accessing,
	   for that declare: struct device_node *np = dev->of_node
	*/
	u32 minstation_val;
	ret = device_property_read_u32(dev, "minstation", &minstation_val);
	if (ret < 0)
	{
		dev_err(&client->dev, "Couldn't read minstation property from devicetree\n");
	}
	else
	{
		tea5767->minstation = minstation_val;
		dev_info(&client->dev, "Read minstation devicetree property as %d\n", tea5767->minstation);
	}

	u32 maxstation_val;
	ret = device_property_read_u32(dev, "maxstation", &maxstation_val);
	if (ret < 0)
	{
		dev_err(&client->dev, "Couldn't read maxstation property from devicetree\n");
	}
	else
	{
		tea5767->maxstation = maxstation_val;
		dev_info(&client->dev, "Read maxstation devicetree property as %d\n", tea5767->maxstation);
	}

	/* Register sysfs hooks to i2c_client object. You can hook it to different objects as you desire!*/
	ret = sysfs_create_group(&client->dev.kobj, &tea5767_sysfs_group);
	if (ret < 0)
	{
		dev_err(&client->dev, "Couldn't register sysfs group\n");
		return ret;
	}

	/* Scan frequencies */
	tea5767_scan_frequencies (client, SEARCH_MODE_DEFAULT, FORCED_MONO);

	/* Set initial station values - will be set after scan_kthread_task completes!*/
	tea5767->frequency = 97200;
	tea5767->mute = 0;

	dev_info(&client->dev, 
		 "tea5767_probe is exited on %s\n", tea5767->name);

	return 0;
}

static int tea5767_remove(struct i2c_client * client)
{
	struct tea5767_dev * tea5767;

	/* Get device structure from bus device context */	
	tea5767 = i2c_get_clientdata(client);

	dev_info(&client->dev, 
		 "tea5767_remove is entered on %s\n", tea5767->name);

	/* Stop kthread if its running */
	if (tea5767->scan_task_flag == 1) {
		kthread_stop(tea5767->scan_task);
		tea5767->scan_task_flag = 0;
	}

	/* Deregister sysfs hooks */
	sysfs_remove_group(&client->dev.kobj, &tea5767_sysfs_group);

	dev_info(&client->dev, 
		 "tea5767_remove is exited on %s\n", tea5767->name);

	return 0;
}

/* Platform device matching by compatible string and i2c ids */

static const struct of_device_id tea5767_dt_ids[] = {
	{ .compatible = "mozcelikors,tea5767", },
	{ }
};
MODULE_DEVICE_TABLE(of, tea5767_dt_ids);

static const struct i2c_device_id i2c_ids[] = {
	{ .name = "tea5767", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2c_ids);

/* I2C driver and module definitions */

static struct i2c_driver tea5767_driver = {
	.driver = {
		.name = "tea5767",
		.owner = THIS_MODULE,
		.of_match_table = tea5767_dt_ids,
	},
	.probe = tea5767_probe,
	.remove = tea5767_remove,
	.id_table = i2c_ids,
};

module_i2c_driver(tea5767_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mustafa Ozcelikors <mozcelikors@gmail.com>");
MODULE_DESCRIPTION("Linux I2C Device Driver for TEA5767 Radio Tuner Module using sysfs");

