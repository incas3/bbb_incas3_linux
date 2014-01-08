/* tmp401.c
 *
 * Copyright (C) 2007,2008 Hans de Goede <hdegoede@redhat.com>
 * Preliminary tmp411 support by:
 * Gabriel Konat, Sander Leget, Wouter Willems
 * Copyright (C) 2009 Andre Prendel <andre.prendel@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * Driver for the Texas Instruments TMP401 SMBUS temperature sensor IC.
 *
 * Note this IC is in some aspect similar to the LM90, but it has quite a
 * few differences too, for example the local temp has a higher resolution
 * and thus has 16 bits registers for its value and limit instead of 8 bits.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include "i2c-compat.h"

/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x44, I2C_CLIENT_END };

enum chips { bbboard};

/*
 * The TMP401 registers, note some registers have different addresses for
 * reading and writing
 */
#define BBBOARD_STATUS				0x02
#define BBBOARD_CONFIG_READ			0x10
#define BBBOARD_CONFIG_WRITE			0x09
#define BBBOARD_TEMP_CONVERSION_RATE		0.00183108262
#define BBBOARD_TEMP_CRIT_HYST			0x21
#define BBBOARD_MANUFACTURER_ID_REG		0xFE
#define BBBOARD_DEVICE_ID_REG			0xFF
#define BBBOARD_MANUFACTURER_ID   0x01
#define BBBOARD_DEVICE_ID         0x02

#define READ_TEMP_CMD         0x20
#define READ_HUMID_CMD        0x30
#define READ_DAC_CMD          0x40
#define READ_ADC_CMD          0x50
#define READ_GPS_LAT_LOW_CMD      0x60
#define READ_GPS_LAT_HIGH_CMD      0x61
#define READ_GPS_LON_LOW_CMD      0x62
#define READ_GPS_LON_HIGH_CMD      0x63
#define WRITE_DAC_CMD         0x44

#define AM33XX_CONTROL_BASE		0x44e10000

#define DRV_NAME           "bbboard"



struct bbboard_gps_data {
   u16 irq;
   u16 irq_pin, gpio_pin;
   u8 irq_fired, irq_enabled;
};

static struct bbboard_gps_data bbb_data;

static int setup_pinmux(void);



static const u8 BBBOARD_TEMP_MSB_READ[6][1] = {
	{ 0x20 },	/* temp */
	{ 0x30 },	/* humidity */
	{ 0x40 },	/* dac */
	{ 0x20 },	/* therm (crit) limit */
	{ 0x30 },	/* lowest */
	{ 0x32 },	/* highest */
};

static const u8 BBBOARD_TEMP_MSB_WRITE[6][1] = {
	{ 0x00 },	/* temp  (unused) */
	{ 0x0C },	/* humidit (unused) */
	{ 0x0B },	/* dac */
	{ 0x20 },	/* therm (crit) limit */
	{ 0x30 },	/* lowest */
	{ 0x32 },	/* highest */
};

static const u8 BBBOARD_TEMP_LSB[6][1] = {
	{ 0x21 },	/* temp */
	{ 0x31 },	/* humidity */
	{ 0x16 },	/* dac */
	{ 0x00 },	/* therm (crit) limit (unused) */
	{ 0x31 },	/* lowest */
	{ 0x33 },	/* highest */
};

/*
 * Driver data (common to all clients)
 */

static const struct i2c_device_id bbboard_id[] = {
	{ "bbboard", bbboard },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bbboard_id);

/*
 * Client data (each client gets its own)
 */

struct bbboard_data {
	struct device *hwmon_dev;
	struct mutex update_lock;
	char valid; /* zero until following fields are valid */
	unsigned long last_updated; /* in jiffies */
	enum chips kind;

	int update_interval;	/* in milliseconds */

	/* register values */
	u8 status[4];
	u8 config;
	u16 temp[6][3];
	u8 temp_crit_hyst;
  u16 dac_val;
};


/*
 * Sysfs attr show / store functions
 */



static ssize_t show_temp(struct device *dev,
			 struct device_attribute *devattr, char *buf)
{
  int16_t temp;
  struct i2c_client *client = to_i2c_client(dev);
	//int nr = to_sensor_dev_attr_2(devattr)->nr;
	//int index = to_sensor_dev_attr_2(devattr)->index;
	//struct bbboard_data *data = bbboard_update_device(dev);

	//if (IS_ERR(data))
	//	return PTR_ERR(data);
  
  temp = i2c_smbus_read_word_data(client, READ_TEMP_CMD)&0xffff;

	return sprintf(buf, "%d\n",temp*10);
		///bbboard_register_to_temp(data->temp[nr][index], data->config));
}

static ssize_t show_humid(struct device *dev,
			 struct device_attribute *devattr, char *buf)
{
  //u8 val;
  u16 humid;
	//int nr = to_sensor_dev_attr_2(devattr)->nr;
	//int index = to_sensor_dev_attr_2(devattr)->index;
	//struct bbboard_data *data = bbboard_update_device(dev);
  struct i2c_client *client = to_i2c_client(dev);

	//if (IS_ERR(data))
	//	return PTR_ERR(data);
	//reg = BBBOARD_TEMP_MSB_READ[1][0];
	humid = i2c_smbus_read_word_data(client, READ_HUMID_CMD);
  //humid = val << 8;
	//reg = BBBOARD_TEMP_LSB[1][0];
	//val = i2c_smbus_read_byte_data(client, READ_HUMID_LSB_CMD);
  //humid += val;

	return sprintf(buf, "%d\n",humid*10);
}


static ssize_t show_gps(struct device *dev,
			 struct device_attribute *devattr, char *buf)
{
  int32_t lat,lon;
  struct i2c_client *client = to_i2c_client(dev);
  
  lat = (i2c_smbus_read_word_data(client, READ_GPS_LAT_LOW_CMD)<<16)+
        (i2c_smbus_read_word_data(client, READ_GPS_LAT_HIGH_CMD)&0xffff);
  
  lon = (i2c_smbus_read_word_data(client, READ_GPS_LON_LOW_CMD)<<16)+
        (i2c_smbus_read_word_data(client, READ_GPS_LON_HIGH_CMD)&0xffff);

	return sprintf(buf, "lat: %d lon: %d\n",lat,lon);
}

static ssize_t show_dac(struct device *dev,
			 struct device_attribute *devattr, char *buf)
{
  u16 dac_val;
  struct i2c_client *client = to_i2c_client(dev);

  dac_val = i2c_smbus_read_word_data(client, READ_DAC_CMD);

  return sprintf(buf, "%d\n",dac_val);
}

static ssize_t show_adc(struct device *dev,
			 struct device_attribute *devattr, char *buf)
{
  u16 dac_val;
	//int nr = to_sensor_dev_attr_2(devattr)->nr;
	//int index = to_sensor_dev_attr_2(devattr)->index;
	//struct bbboard_data *data = bbboard_update_device(dev);
  struct i2c_client *client = to_i2c_client(dev);

	//if (IS_ERR(data))
	//	return PTR_ERR(data);
	//reg = BBBOARD_TEMP_MSB_READ[2][0];
	dac_val = i2c_smbus_read_word_data(client, READ_ADC_CMD);
  //dac_val = htons(val);

	return sprintf(buf, "%d\n",dac_val);
}

static ssize_t store_dac(struct device *dev, struct device_attribute
	*devattr, const char *buf, size_t count)
{
	//int temp, index = to_sensor_dev_attr(devattr)->index;
	//struct bbboard_data *data = bbboard_update_device(dev);
  struct i2c_client *client = to_i2c_client(dev);
  struct bbboard_data *data = i2c_get_clientdata(client);
	long val;
	//u8 reg;

	//if (IS_ERR(data))
	//	return PTR_ERR(data);

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	val = clamp_val(val, 0, 4096);

	mutex_lock(&data->update_lock);

	i2c_smbus_write_word_data(to_i2c_client(dev), WRITE_DAC_CMD,
				  val);
	//i2c_smbus_write_byte_data(to_i2c_client(dev), WRITE_DAC_LSB_CMD,
	//			  val & 0xff);

	data->dac_val = val;

	mutex_unlock(&data->update_lock);

	return count;
}

static ssize_t show_voltage_label(struct device *dev,
				  struct device_attribute *devattr,
				  char *buf)
{
  struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
  
  switch(attr->index) {
    case 0:
      return sprintf(buf, "DAC setpoint voltage\n");
    break;
    case 1:
      return sprintf(buf, "ADC Voltage\n");
    break;
    
    default:
    return sprintf(buf, "\n");
  }

	
}

static SENSOR_DEVICE_ATTR_2(temp1_input, S_IRUGO, show_temp, NULL, 0, 0);
static SENSOR_DEVICE_ATTR_2(humidity1_input, S_IRUGO, show_humid, NULL, 0, 0);
static SENSOR_DEVICE_ATTR(in1_input, S_IWUSR | S_IRUGO, show_dac, store_dac, 0);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, show_adc, NULL, 0);
static SENSOR_DEVICE_ATTR(gps_data, S_IRUGO, show_gps, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_label, S_IRUGO, show_voltage_label, NULL, 0);
static SENSOR_DEVICE_ATTR(in2_label, S_IRUGO, show_voltage_label, NULL, 1);

static struct attribute *bbboard_attributes[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
  &sensor_dev_attr_humidity1_input.dev_attr.attr,
  &sensor_dev_attr_in1_input.dev_attr.attr,
  &sensor_dev_attr_in2_input.dev_attr.attr,
  &sensor_dev_attr_gps_data.dev_attr.attr,
  &sensor_dev_attr_in1_label.dev_attr.attr,
  &sensor_dev_attr_in2_label.dev_attr.attr,
	NULL
};

static const struct attribute_group bbboard_group = {
	.attrs = bbboard_attributes,
};


static int
setup_pinmux(void)
{
   int i;
   static u32 pins[] = {
      AM33XX_CONTROL_BASE + 0x834,   // irq pin (13): gpio1_13 (beaglebone p8/11)
      0x7 | (2 << 3) | (1 << 5),     //       mode 7 (gpio), PULLUP, INPUT
   };

   for (i=0; i<2; i+=2) {
      void* addr = ioremap(pins[i], 4);

      if (NULL == addr)
         return -EBUSY;

      iowrite32(pins[i+1], addr);
      iounmap(addr);
   }

   bbb_data.irq_pin = 45;

   return 0;
}

static irqreturn_t
bbboard_gps_interrupt_handler(int irq, void* dev_id)
{
   struct bbboard_gps_data* data = (struct bbboard_gps_data*)dev_id;
   
   //getnstimeofday(&data->irq_time);
   data->irq_fired = 1;
   printk(KERN_ALERT DRV_NAME " : Interrupt received\n");
   //gpio_set_value(data->gpio_pin, 1);
   
   return IRQ_HANDLED;
}

/*
 * Begin non sysfs callback code (aka Real code)
 */

static void bbboard_init_client(struct i2c_client *client)
{
  
	int config;//, config_orig;
	struct bbboard_data *data = i2c_get_clientdata(client);
  int err;
  printk("BBBOARD init client");
	/* Set the conversion rate to 2 Hz */
	//i2c_smbus_write_byte_data(client, BBBOARD_CONVERSION_RATE_WRITE, 5);
	data->update_interval = 500;

	/* Start conversions (disable shutdown if necessary) */
	config = i2c_smbus_read_byte_data(client, BBBOARD_CONFIG_READ);
	if (config < 0) {
		dev_warn(&client->dev, "Initialization failed!\n");
		return;
	}
  
  printk("BBBOARD init irq pin");
   

   err = setup_pinmux();
   if (err < 0) {
      printk(KERN_ALERT DRV_NAME " : failed to apply pinmux settings.\n");
      goto err_return;
   }
   
   err = gpio_request_one(bbb_data.irq_pin, GPIOF_IN, DRV_NAME " irq");
   if (err < 0) {
      printk(KERN_ALERT DRV_NAME " : failed to request IRQ pin %d.\n",
         bbb_data.irq_pin);
      goto err_free_gpio_return;
   }

   err = gpio_to_irq(bbb_data.irq_pin);
   if (err < 0) {
      printk(KERN_ALERT DRV_NAME " : failed to get IRQ for pin %d.\n",
         bbb_data.irq_pin);
      goto err_free_irq_return;
   } else {
      bbb_data.irq = (u16)err;
      err = 0;
   }

   err = request_any_context_irq(
      bbb_data.irq,
      bbboard_gps_interrupt_handler,
      IRQF_TRIGGER_FALLING | IRQF_DISABLED,
      DRV_NAME,
      (void*)&bbb_data
   );
   if (err < 0) {
      printk(KERN_ALERT DRV_NAME " : failed to enable IRQ %d for pin %d.\n",
         bbb_data.irq, bbb_data.irq_pin);
      goto err_free_irq_return;
   } else
      bbb_data.irq_enabled = 1;

   printk(KERN_INFO DRV_NAME
      " : Module loaded.\n");
   
   return;

err_free_irq_return:
   gpio_free(bbb_data.irq_pin);
err_free_gpio_return:
   gpio_free(bbb_data.gpio_pin);
err_return:
   return;
  /*
	config_orig = config;
	config &= ~BBBOARD_CONFIG_SHUTDOWN;

	if (config != config_orig)
		i2c_smbus_write_byte_data(client, BBBOARD_CONFIG_WRITE, config);*/
}

static int bbboard_detect(struct i2c_client *client,
			 struct i2c_board_info *info)
{
	enum chips kind;
	struct i2c_adapter *adapter = client->adapter;
	u8 reg;
  dev_info(&client->dev, "Checking adapter capabilities\n");
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;
  dev_info(&client->dev, "byte data..ok\n");
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;
  dev_info(&client->dev, "word data..ok\n");
	//if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_BLOCK_DATA))
	//	return -ENODEV;
  //dev_info(&client->dev, "block data..ok\n");

  dev_info(&client->dev, "Detecting BBBoard\n");
	/* Detect and identify the chip */
	reg = i2c_smbus_read_byte_data(client, BBBOARD_MANUFACTURER_ID_REG);
  dev_info(&client->dev, "Found manufacturer ID %d\n",reg);
	if (reg != BBBOARD_MANUFACTURER_ID)
		return -ENODEV;

	reg = i2c_smbus_read_byte_data(client, BBBOARD_DEVICE_ID_REG);
  dev_info(&client->dev, "Found device ID %d\n",reg);
	switch (reg) {
	case BBBOARD_DEVICE_ID:
		if (client->addr != 0x44) {
		  dev_warn(&client->dev, "Client addr check failed! (%d)\n",client->addr);
      return -ENODEV;
		}
			
		kind = bbboard;
		break;
	default:
		return -ENODEV;
	}

	reg = i2c_smbus_read_byte_data(client, BBBOARD_CONFIG_READ);
	//if (reg != 0x01)
	//	return -ENODEV;

	//reg = i2c_smbus_read_byte_data(client, BBBOARD_CONVERSION_RATE_READ);
	/* Datasheet says: 0x1-0x6 */
	//if (reg > 15)
	//	return -ENODEV;

	strlcpy(info->type, bbboard_id[kind].name, I2C_NAME_SIZE);

	return 0;
}

static int bbboard_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct bbboard_data *data = i2c_get_clientdata(client);

	if (data->hwmon_dev)
		hwmon_device_unregister(data->hwmon_dev);

	sysfs_remove_group(&dev->kobj, &bbboard_group);
  free_irq(bbb_data.irq, (void*)&bbb_data);

  gpio_free(bbb_data.irq_pin);
  //gpio_free(bbb_data.gpio_pin);

  printk(KERN_INFO DRV_NAME " : unloaded BBBoard.\n");
	return 0;
}

static int bbboard_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int err;
	struct bbboard_data *data;
	const char *names[] = { "BBBOARD" };

	data = devm_kzalloc(&client->dev, sizeof(struct bbboard_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);
	data->kind = id->driver_data;

	/* Initialize the TMP401 chip */
	bbboard_init_client(client);

	/* Register sysfs hooks */
	err = sysfs_create_group(&dev->kobj, &bbboard_group);
	if (err)
		return err;

	data->hwmon_dev = hwmon_device_register(dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		data->hwmon_dev = NULL;
		goto exit_remove;
	}

	dev_info(dev, "Detected INCAS3 %s chip\n", names[data->kind]);

	return 0;

exit_remove:
	bbboard_remove(client);
	return err;
}

static struct i2c_driver bbboard_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "bbboard",
	},
	.probe		= bbboard_probe,
	.remove		= bbboard_remove,
	.id_table	= bbboard_id,
	.detect		= bbboard_detect,
	.address_list	= normal_i2c,
};

module_i2c_driver(bbboard_driver);

MODULE_AUTHOR("Erik Kallen <erikkallen@incas3.eu>");
MODULE_DESCRIPTION("INCAS3 BBBoard temperature / humidity sensor driver");
MODULE_LICENSE("GPL");
