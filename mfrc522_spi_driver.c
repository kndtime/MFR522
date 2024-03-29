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
#include <linux/delay.h>
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
#include "mfrc522_spi_driver.h"

#define READ_CARD     1
#define GET_ID 	      2

int32_t value = 0;

uint KuaiN;
static unchar PassWd[6]={0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static unsigned char MLastSelectedSnr[4];
static unsigned char buffer[16];

static unsigned char RevBuffer[30];
static unsigned char Read_Data[16]={0x00};
static unsigned char read_data_buff[16];

struct spi_device *mfrc522_spi;

/* sent a value over SPI
 * Bit 7 is set to 0 indicating write operation
 * Bits 6 to 1 define the address
 * Bit 0 is set to 0
 */
uint8_t mfrc522_write_raw_rc(uint8_t addr, uint8_t value)
{
        struct spi_transfer st[2];
        struct spi_message  msg;
        unsigned char ucAddr;
        WAIT_FOR;
        ucAddr = ((addr<<1)&0x7E);

        spi_message_init( &msg );
        memset( st, 0, sizeof(st) );

        st[ 0 ].tx_buf = &ucAddr;
        st[ 0 ].len = 1;
        spi_message_add_tail( &st[0], &msg );

        st[ 1 ].tx_buf = &value;
        st[ 1 ].len = 1;
        spi_message_add_tail( &st[1], &msg );
        spi_sync( mfrc522_spi, &msg );

        WAIT_FOR;
        return 0;
}

unsigned char mfrc522_read_raw_rc(unsigned char addr)
{
	      unsigned char ucAddr;
	      unsigned char ucResult=0;
        int ret;
	      WAIT_FOR;
	      ucAddr = ((addr<<1)&0x7E)|0x80;

	      ret = spi_write_then_read(mfrc522_spi, &ucAddr, 1, &ucResult, 1);
	      if(ret != 0)
        {
		            printk("spi_write_then_read err = %d\n", ret);
	      }
	      WAIT_FOR;
	      return ucResult;
}

void mfrc522_bit_mask(unsigned char reg,unsigned char mask)
{
	       char tmp = 0x0;
	       tmp = mfrc522_read_raw_rc(reg);
	       mfrc522_write_raw_rc(reg,tmp | mask);  // set bit mask
}

void mfrc522_clear_bit_mask(unsigned char reg,unsigned char mask)
{
	       char tmp = 0x0;
	       tmp = mfrc522_read_raw_rc(reg);
	       mfrc522_write_raw_rc(reg, tmp & ~mask);  // clear bit mask
}

char mfrc522_communicate(unsigned char Command, unsigned char *pInData,
		             unsigned char InLenByte, unsigned char *pOutData,
		             unsigned int *pOutLenBit)
{
        	char status = MI_ERR;
        	unsigned char irqEn   = 0x00;
        	unsigned char waitFor = 0x00;
        	unsigned char lastBits;
        	unsigned char n;
        	unsigned int i;
        	switch (Command)
        	{
        		case PCD_AUTHENT:
        			irqEn   = 0x12;
        			waitFor = 0x10;
        			break;
        		case PCD_TRANSCEIVE:
        			irqEn   = 0x77;
        			waitFor = 0x30;
        			break;
        		default:
        			break;
        	}

        	mfrc522_write_raw_rc(ComIEnReg,irqEn|0x80);
        	mfrc522_clear_bit_mask(ComIrqReg,0x80);
        	mfrc522_write_raw_rc(CommandReg,PCD_IDLE);
        	mfrc522_bit_mask(FIFOLevelReg,0x80);

        	for (i=0; i<InLenByte; i++)
        	         mfrc522_write_raw_rc(FIFODataReg, pInData[i]);
        	mfrc522_write_raw_rc(CommandReg, Command);


        	if (Command == PCD_TRANSCEIVE)
        	         mfrc522_bit_mask(BitFramingReg,0x80);


        	i = 2000;
        	do
        	{
        		      n = mfrc522_read_raw_rc(ComIrqReg);
        		      i--;
        	}
        	while ((i!=0) && !(n&0x01) && !(n&waitFor));
        	       mfrc522_clear_bit_mask(BitFramingReg,0x80);

        	if (i!=0)
        	{
              		if(!(mfrc522_read_raw_rc(ErrorReg)&0x1B))
              		{
              			       status = MI_OK;
              			       if (n & irqEn & 0x01)
              			             status = MI_NOTAGERR;
              			       if (Command == PCD_TRANSCEIVE)
                      		 {
                      			     n = mfrc522_read_raw_rc(FIFOLevelReg);
                      			    lastBits = mfrc522_read_raw_rc(ControlReg) & 0x07;
                      			    if (lastBits)
                      			           *pOutLenBit = (n - 1) * 8 + lastBits;
                      				  else
                      				         *pOutLenBit = n * 8;
                      				  if (n == 0)
                      				         n = 1;
                      				  if (n > MAXRLEN)
                      				         n = MAXRLEN;
                      				for (i=0; i<n; i++)
                      				        pOutData[i] = mfrc522_read_raw_rc(FIFODataReg);
                      		}
              		}
              		else
              		        status = MI_ERR;

        	}
        	mfrc522_bit_mask(ControlReg,0x80);           // stop timer now
        	mfrc522_write_raw_rc(CommandReg,PCD_IDLE);
        	return status;
}

char mfrc522_auth_state(unsigned char auth_mode,unsigned char addr,
                        unsigned char *pKey,unsigned char *pSnr)
{
      	char status;
      	unsigned int unLen;
      	unsigned char i,ucComMF522Buf[MAXRLEN];

      	ucComMF522Buf[0] = auth_mode;
      	ucComMF522Buf[1] = addr;
      	for (i=0; i<6; i++)
      	         ucComMF522Buf[i+2] = *( pKey + i );
      	for (i=0; i<6; i++)
      	         ucComMF522Buf[i+8] = *( pSnr + i );

      	status = mfrc522_communicate(PCD_AUTHENT, ucComMF522Buf, 12, ucComMF522Buf, &unLen);
      	if ((status != MI_OK) || (!(mfrc522_read_raw_rc(Status2Reg) & 0x08)))
      	         status = MI_ERR;
      	return status;
}

void mfrc522_calulate_CRC(unsigned char *pIndata, unsigned char len,
                          unsigned char *pOutData)
{
      	unsigned char i,n;
      	mfrc522_clear_bit_mask(DivIrqReg,0x04);
      	mfrc522_write_raw_rc(CommandReg,PCD_IDLE);
      	mfrc522_bit_mask(FIFOLevelReg,0x80);
      	for (i=0; i<len; i++)
      	       mfrc522_write_raw_rc(FIFODataReg, *(pIndata+i));
      	mfrc522_write_raw_rc(CommandReg, PCD_CALCCRC);
      	i = 0xFF;
      	do
      	{
            		n = mfrc522_read_raw_rc(DivIrqReg);
            		i--;
      	}
      	while ((i!=0) && !(n&0x04));
      	pOutData[0] = mfrc522_read_raw_rc(CRCResultRegL);
      	pOutData[1] = mfrc522_read_raw_rc(CRCResultRegM);
}

char mfrc522_read_addr(unsigned char addr,unsigned char *pData)
{
      	char status;
      	unsigned int unLen;
      	unsigned char i, ucComMF522Buf[MAXRLEN];

      	ucComMF522Buf[0] = PICC_READ;
      	ucComMF522Buf[1] = addr;
      	mfrc522_calulate_CRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

      	status = mfrc522_communicate(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);
      	if ((status == MI_OK) && (unLen == 0x90))
      	{
            		for (i=0; i<16; i++)
            		        *(pData+i) = ucComMF522Buf[i];
      	}
      	else
                status = MI_ERR;

      	return status;
}

char mfrc522_halt(void)
{
      	char status;
      	unsigned int unLen;
      	unsigned char ucComMF522Buf[MAXRLEN];

      	ucComMF522Buf[0] = PICC_HALT;
      	ucComMF522Buf[1] = 0;
      	mfrc522_calulate_CRC(ucComMF522Buf, 2, &ucComMF522Buf[2]);

      	status = mfrc522_communicate(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, &unLen);

      	return MI_OK;
}

void mfrc522_antenna_on(void)
{
        unsigned char i;
        i = mfrc522_read_raw_rc(TxControlReg); // Check if Antenna is ON
        if (!(i & 0x03)) {
                mfrc522_bit_mask(TxControlReg, 0x03); // Set REG to ON
        }
}

void mfrc522_antenna_off(void)
{
        mfrc522_clear_bit_mask(TxControlReg, 0x03); // CLEAR REG
}

void mfrc522_antenna_reset(void)
{
        mfrc522_antenna_off();
        mfrc522_antenna_on();
}

void mfrc522_iso_config(void)
{
        mfrc522_clear_bit_mask(Status2Reg, 0x08);
        mfrc522_write_raw_rc(ModeReg, 0x3D);
        mfrc522_write_raw_rc(RxSelReg, 0x86);
        mfrc522_write_raw_rc(RFCfgReg, 0x7F);
        mfrc522_write_raw_rc(TReloadRegL, 30);
        mfrc522_write_raw_rc(TReloadRegH, 0);
        mfrc522_write_raw_rc(TModeReg, 0x8D);
        mfrc522_write_raw_rc(TPrescalerReg, 0x3E);
        ndelay(1000);
        mfrc522_antenna_on();
}

char mfrc522_reset(void)
{
        WAIT_FOR;
        ndelay(10);
        WAIT_FOR;
        ndelay(10);
        WAIT_FOR;
        ndelay(10);
        mfrc522_write_raw_rc(CommandReg, PCD_RESETPHASE);
        ndelay(10);

        mfrc522_write_raw_rc(ModeReg, 0x3D);
        mfrc522_write_raw_rc(TReloadRegL, 30);
        mfrc522_write_raw_rc(TReloadRegH, 0);
        mfrc522_write_raw_rc(TModeReg, 0x8D);
        mfrc522_write_raw_rc(TPrescalerReg, 0x3E);
        mfrc522_write_raw_rc(TxASKReg, 0x40);

        return MI_OK;
}

char mfrc522_request(unsigned char req_code,unsigned char *pTagType)
{
	char status;
	unsigned int unLen;
	unsigned char ucComMF522Buf[MAXRLEN];

	mfrc522_clear_bit_mask(Status2Reg,0x08);
	mfrc522_write_raw_rc(BitFramingReg,0x07);
	mfrc522_bit_mask(TxControlReg,0x03);

	ucComMF522Buf[0] = req_code;

	status = mfrc522_communicate(PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf, &unLen);

	if ((status == MI_OK) && (unLen == 0x10))
	{
      		*pTagType     = ucComMF522Buf[0];
      		*(pTagType+1) = ucComMF522Buf[1];
	}
	else
      	 status = MI_ERR;

	return status;
}

char mfrc522_anticol(unsigned char *pSnr)
{
	char status;
	unsigned char i,snr_check=0;
	unsigned int unLen;
	unsigned char ucComMF522Buf[MAXRLEN];


	mfrc522_clear_bit_mask(Status2Reg,0x08);
	mfrc522_write_raw_rc(BitFramingReg,0x00);
	mfrc522_clear_bit_mask(CollReg,0x80);

	ucComMF522Buf[0] = PICC_ANTICOLL1;
	ucComMF522Buf[1] = 0x20;

	status = mfrc522_communicate(PCD_TRANSCEIVE, ucComMF522Buf, 2, ucComMF522Buf, &unLen);

	if (status == MI_OK)
	{
      		for (i=0; i<4; i++)
      		{
            			*(pSnr+i)  = ucComMF522Buf[i];
            			snr_check ^= ucComMF522Buf[i];
      		}
      		if (snr_check != ucComMF522Buf[i])
            		  status = MI_ERR;
	}

	mfrc522_bit_mask(CollReg, 0x80);
	return status;
}

char mfrc522_select(unsigned char *pSnr)
{
  char status;
  unsigned char i;
  unsigned int len;
  unsigned char buffer[MAXRLEN];

  buffer[0] = PICC_ANTICOLL1;
  buffer[1] = 0x70;
  buffer[6] = 0;
  for (i = 0; i < 4; i++) {
          buffer[i + 2] = *(pSnr + i);
          buffer[6] ^= *(pSnr + i);
  }

  mfrc522_calulate_CRC(buffer, 7, &buffer[7]);
  mfrc522_clear_bit_mask(Status2Reg, 0x08);

  status = mfrc522_communicate(PCD_TRANSCEIVE, buffer, 9, buffer, &len);

  if ((status == MI_OK) && (len == 0x18))
    status = MI_OK;
  else
    status = MI_ERR;

  return status;
}

int mfrc522_initialization(void)
{
        unsigned char read_res;
        // Reset
        mfrc522_reset();
        // Try a read
        read_res = mfrc522_read_raw_rc(TReloadRegL);
        if (read_res != 30)
        {
                printk(KERN_DEBUG "mfrc522: no device detected read error %d\n", read_res);
                return -ENODEV;
        }
        else
                printk(KERN_DEBUG "mfrc522: device detected\n");
        // Put the antenna on and off to reset
        mfrc522_antenna_reset();
        // Write ISO14443_A headers
        mfrc522_iso_config();
        return 0;
}

static char mfrc522_state(unchar a)
{
        char *pdata = buffer;
        char status;
        mfrc522_reset();
      	status = mfrc522_request(PICC_REQIDL, &RevBuffer[0]);
      	if(status != MI_OK)
      	{
            		printk(KERN_DEBUG "mfrc522: no search card, no card found\n");
            		return -EFAULT;
      	}
      	status = mfrc522_anticol(&RevBuffer[2]);
      	if(status != MI_OK)
      	{
            		printk(KERN_DEBUG "mfrc522: get card nu, no number\n");
            		return -EFAULT;
      	}
      	memcpy(MLastSelectedSnr, &RevBuffer[2], 4);

      	status = mfrc522_select(MLastSelectedSnr);
      	if(status!=MI_OK)
      	{
            		printk(KERN_DEBUG "mfrc522: no card selected\n");
            		return -EFAULT;
      	}


        if (a == GET_ID)
        {
                mfrc522_halt();
                return 0;
        }
        else
        {
              status = mfrc522_auth_state(PICC_AUTHENT1A, KuaiN, PassWd,
                                          MLastSelectedSnr);
          		if(status != MI_OK)
          		{
                      printk(KERN_DEBUG "mfrc522: error while reading card\n");
          			      return -EFAULT;
          		}
          		status = mfrc522_read_addr(KuaiN, Read_Data);
          		if(status != MI_OK)
          		{
          			     printk(KERN_DEBUG"read card err\n");
          			     return -EFAULT;
          		}
              else
              {
          			     int i;
          			     memcpy(pdata, Read_Data, sizeof(Read_Data));
          			     printk(KERN_DEBUG"read block %d info:", KuaiN);
          			     for(i = 0; i < 16; i++)
          				          printk(KERN_DEBUG"%2.2X", pdata[i]);
          			     printk(KERN_DEBUG"\n");
          		}
      }
      mfrc522_halt();
      return MI_OK;
}

static ssize_t mfrc522_read(struct file *file, char *buf, size_t count,
                            loff_t *ppos)
{
        if(mfrc522_state(READ_CARD))
                return 0;
        printk(KERN_DEBUG"mfrc522: card info:%2.2X\n", Read_Data[0]);
        if (copy_to_user(buf, read_data_buff, sizeof(read_data_buff)))
        {
              printk(KERN_DEBUG
                     "mfrc522: error while copying card number to userspace\n");
              return 0;
        }
        return sizeof(read_data_buff);
}

static int mfrc522_open(struct inode *inode, struct file *file)
{
        printk(KERN_DEBUG "mfrc522_open()\n");
        return mfrc522_initialization();
}

static ssize_t mfrc522_write (struct file *filp, const char *buf, size_t count,
                            loff_t *f_pos)
{
        //TODO
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
        printk(KERN_DEBUG "mfrc522: driver removed\n");
        return 0;
}

static int mfrc522_probe(struct spi_device *spi)
{
        KuaiN = 1;
        printk(KERN_DEBUG "mfrc522: %s\n", __func__);
        mfrc522_spi = spi;
        return 0;
};

static long mfrc522_ioctl(struct file *file, unsigned int state, unsigned long arg)
{
        switch (state)
        {
                case READ_CARD:
                      //NOPE
                break;
                case GET_ID:
                      if(!mfrc522_state(GET_ID))
                      {
                                if (copy_to_user((char *)arg, MLastSelectedSnr,4))
                                {
                                      printk(KERN_DEBUG
                                      "mfrc522: error while copying to userland %s,[line %d].\n",
                                      __FILE__, __LINE__);
                                      return -EFAULT;
                                }
                      }
                break;
        }
        return 0;
}

static struct spi_driver mfrc522_driver =
{
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

static struct miscdevice mfrc522_misc_device =
{
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
