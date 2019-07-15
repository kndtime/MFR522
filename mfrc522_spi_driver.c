// SPDX-License-Identifier: GPL-2.0-only
/*
 * MFRC522 -  SPI driver for mfrc522 device
 *
 *  Copyright (C) 2019 Banal Axel <axel.banal@epita.fr>
 *  Copyright (C) 2019 Antoine Lebeury <antoine.lebeury@epita.fr>
 */
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/uaccess.h>
#include <asm/uaccess.h> /* copy_{from,to}_user() */
#include <linux/fs.h>   /* file_operations */
#include <linux/init.h> /* module_{init,exit}() */
#include <linux/interrupt.h> /* request_irq etc */
#include <linux/kernel.h>    /* printk() */
#include <linux/list.h>      /* list_*() */
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h> /* kmalloc()/kfree() */
#include <linux/spi/spi.h>
#include <linux/ioctl.h>

#define WR_VALUE _IOW('a','a',int32_t*)
#define RD_VALUE _IOR('a','b',int32_t*)

int32_t value = 0;

struct spi_device *mfrc522_spi;

static ssize_t mfrc522_read(struct file *file, char *buf, size_t count,
                            loff_t *ppos)
{
        return 0;
}


static int mfrc522_open(struct inode *inode, struct file *file)
{
        return 0;
}

static ssize_t mfrc522_write (struct file *filp, const char *buf, size_t count,
                            loff_t *f_pos)
{
        return 0;
}

static int mfrc522_release(struct inode *inode, struct file *file)
{
        printk(KERN_DEBUG "mfrc522_release()\n");
        file->private_data = NULL;
        printk(KERN_DEBUG "mfrc522: released!\n");
        return 0;
}

static int mfrc522_remove(struct spi_device *spi)
{
        return 0;
}

static int mfrc522_probe(struct spi_device *spi)
{
        printk(KERN_DEBUG "mfrc522: %s\n", __func__);
        mfrc522_spi = spi;
        return 0;
};

static long mfrc522_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
         switch(cmd) {
                case WR_VALUE:
                        _copy_from_user(&value ,(int32_t*) arg, sizeof(value));
                        printk(KERN_INFO "Value = %d\n", value);
                        break;
                case RD_VALUE:
                        _copy_to_user((int32_t*) arg, &value, sizeof(value));
                        break;
        }
        return 0;
}

static struct spi_driver mfrc522_driver = {
	.probe = mfrc522_probe,
	.remove = mfrc522_remove,
	.driver = {
		.name = "mfrc522_rfid",
	},
};

static struct file_operations mfrc522_fops =
{
        .owner = THIS_MODULE,
        .read = mfrc522_read,
        .open = mfrc522_open,
        .write = mfrc522_write,
        .release = mfrc522_release,
        .unlocked_ioctl = mfrc522_ioctl,
};

static struct miscdevice mfrc522_misc_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "mfrc522_rfid_dev",
        .fops = &mfrc522_fops,
};


static int __init mfrc522_init(void)
{
        int res;
        printk(KERN_INFO "Hello, card reader!\n");
        /* Register the character device */
        res = misc_register(&mfrc522_misc_device);
        if (res < 0) {
          printk(KERN_DEBUG "mfrc522: device register failed with error %d.\n",
                 res);
          return res;
        }
        /* Register the SPI device */
        res = spi_register_driver(&mfrc522_driver);
        if (res < 0) {
          printk(KERN_DEBUG "mfrc522: device spi register failed with %s\n",
                 __FUNCTION__);
          return res;
        }
        return 0;
}

static void __exit mfrc522_exit(void)
{
        printk(KERN_INFO "Goodbye, card reader!\n");
        printk(KERN_DEBUG "mfrc522: driver exited\n");
        spi_unregister_driver(&mfrc522_driver);
        misc_deregister(&mfrc522_misc_device);
}

module_init(mfrc522_init);
module_exit(mfrc522_exit);



MODULE_AUTHOR("Axel Banal <axel.banal@epita.fr>");
MODULE_AUTHOR("Antoine Lebeury <antoine.lebeury@epita.fr>");
MODULE_DESCRIPTION("SPI driver for mfrc522 device");
MODULE_LICENSE("GPL v2");
