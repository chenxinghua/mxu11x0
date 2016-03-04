/*
 * TI 3410/5052 USB Serial Driver
 *
 * Copyright (C) 2004 Texas Instruments
 *
 * This driver is based on the Linux io_ti driver, which is
 *   Copyright (C) 2000-2002 Inside Out Networks
 *   Copyright (C) 2001-2002 Greg Kroah-Hartman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * For questions or problems with this driver, contact Texas Instruments
 * technical support, or Al Borchers <alborchers@steinerpoint.com>, or
 * Peter Berger <pberger@brimson.com>.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/ioctl.h>
#include <linux/serial.h>
#include <linux/kfifo.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

/* Configuration ids */
#define TI_BOOT_CONFIG			1
#define TI_ACTIVE_CONFIG		2

/* Vendor and product ids */
#define TI_VENDOR_ID			0x0451
#define IBM_VENDOR_ID			0x04b3
#define TI_3410_PRODUCT_ID		0x3410
#define IBM_4543_PRODUCT_ID		0x4543
#define IBM_454B_PRODUCT_ID		0x454b
#define IBM_454C_PRODUCT_ID		0x454c
#define TI_3410_EZ430_ID		0xF430  /* TI ez430 development tool */
#define TI_5052_BOOT_PRODUCT_ID		0x5052	/* no EEPROM, no firmware */
#define TI_5152_BOOT_PRODUCT_ID		0x5152	/* no EEPROM, no firmware */
#define TI_5052_EEPROM_PRODUCT_ID	0x505A	/* EEPROM, no firmware */
#define TI_5052_FIRMWARE_PRODUCT_ID	0x505F	/* firmware is running */
#define FRI2_PRODUCT_ID			0x5053  /* Fish River Island II */

/* Multi-Tech vendor and product ids */
#define MTS_VENDOR_ID			0x06E0
#define MTS_GSM_NO_FW_PRODUCT_ID	0xF108
#define MTS_CDMA_NO_FW_PRODUCT_ID	0xF109
#define MTS_CDMA_PRODUCT_ID		0xF110
#define MTS_GSM_PRODUCT_ID		0xF111
#define MTS_EDGE_PRODUCT_ID		0xF112
#define MTS_MT9234MU_PRODUCT_ID		0xF114
#define MTS_MT9234ZBA_PRODUCT_ID	0xF115
#define MTS_MT9234ZBAOLD_PRODUCT_ID	0x0319

/* Abbott Diabetics vendor and product ids */
#define ABBOTT_VENDOR_ID		0x1a61
#define ABBOTT_STEREO_PLUG_ID		0x3410
#define ABBOTT_PRODUCT_ID		ABBOTT_STEREO_PLUG_ID
#define ABBOTT_STRIP_PORT_ID		0x3420

/* Honeywell vendor and product IDs */
#define HONEYWELL_VENDOR_ID		0x10ac
#define HONEYWELL_HGI80_PRODUCT_ID	0x0102  /* Honeywell HGI80 */

/* Moxa UPORT 11x0 vendor and product IDs */
#define MXU1_VENDOR_ID				0x110a
#define MXU1_1110_PRODUCT_ID			0x1110
#define MXU1_1130_PRODUCT_ID			0x1130
#define MXU1_1150_PRODUCT_ID			0x1150
#define MXU1_1151_PRODUCT_ID			0x1151
#define MXU1_1131_PRODUCT_ID			0x1131

/* Commands */
#define TI_GET_VERSION			0x01
#define TI_GET_PORT_STATUS		0x02
#define TI_GET_PORT_DEV_INFO		0x03
#define TI_GET_CONFIG			0x04
#define TI_SET_CONFIG			0x05
#define TI_OPEN_PORT			0x06
#define TI_CLOSE_PORT			0x07
#define TI_START_PORT			0x08
#define TI_STOP_PORT			0x09
#define TI_TEST_PORT			0x0A
#define TI_PURGE_PORT			0x0B
#define TI_RESET_EXT_DEVICE		0x0C
#define TI_WRITE_DATA			0x80
#define TI_READ_DATA			0x81
#define TI_REQ_TYPE_CLASS		0x82

/* Module identifiers */
#define TI_I2C_PORT			0x01
#define TI_IEEE1284_PORT		0x02
#define TI_UART1_PORT			0x03
#define TI_UART2_PORT			0x04
#define TI_RAM_PORT			0x05

/* Modem status */
#define TI_MSR_DELTA_CTS		0x01
#define TI_MSR_DELTA_DSR		0x02
#define TI_MSR_DELTA_RI			0x04
#define TI_MSR_DELTA_CD			0x08
#define TI_MSR_CTS			0x10
#define TI_MSR_DSR			0x20
#define TI_MSR_RI			0x40
#define TI_MSR_CD			0x80
#define TI_MSR_DELTA_MASK		0x0F
#define TI_MSR_MASK			0xF0

/* Line status */
#define TI_LSR_OVERRUN_ERROR		0x01
#define TI_LSR_PARITY_ERROR		0x02
#define TI_LSR_FRAMING_ERROR		0x04
#define TI_LSR_BREAK			0x08
#define TI_LSR_ERROR			0x0F
#define TI_LSR_RX_FULL			0x10
#define TI_LSR_TX_EMPTY			0x20

/* Line control */
#define TI_LCR_BREAK			0x40

/* Modem control */
#define TI_MCR_LOOP			0x04
#define TI_MCR_DTR			0x10
#define TI_MCR_RTS			0x20

/* Mask settings */
#define TI_UART_ENABLE_RTS_IN		0x0001
#define TI_UART_DISABLE_RTS		0x0002
#define TI_UART_ENABLE_PARITY_CHECKING	0x0008
#define TI_UART_ENABLE_DSR_OUT		0x0010
#define TI_UART_ENABLE_CTS_OUT		0x0020
#define TI_UART_ENABLE_X_OUT		0x0040
#define TI_UART_ENABLE_XA_OUT		0x0080
#define TI_UART_ENABLE_X_IN		0x0100
#define TI_UART_ENABLE_DTR_IN		0x0800
#define TI_UART_DISABLE_DTR		0x1000
#define TI_UART_ENABLE_MS_INTS		0x2000
#define TI_UART_ENABLE_AUTO_START_DMA	0x4000

/* Parity */
#define TI_UART_NO_PARITY		0x00
#define TI_UART_ODD_PARITY		0x01
#define TI_UART_EVEN_PARITY		0x02
#define TI_UART_MARK_PARITY		0x03
#define TI_UART_SPACE_PARITY		0x04

/* Stop bits */
#define TI_UART_1_STOP_BITS		0x00
#define TI_UART_1_5_STOP_BITS		0x01
#define TI_UART_2_STOP_BITS		0x02

/* Bits per character */
#define TI_UART_5_DATA_BITS		0x00
#define TI_UART_6_DATA_BITS		0x01
#define TI_UART_7_DATA_BITS		0x02
#define TI_UART_8_DATA_BITS		0x03

/* 232/485 modes */
#define TI_UART_232			0x00
#define TI_UART_485_RECEIVER_DISABLED	0x01
#define TI_UART_485_RECEIVER_ENABLED	0x02

/* Pipe transfer mode and timeout */
#define TI_PIPE_MODE_CONTINOUS		0x01
#define TI_PIPE_MODE_MASK		0x03
#define TI_PIPE_TIMEOUT_MASK		0x7C
#define TI_PIPE_TIMEOUT_ENABLE		0x80

/* Config struct */
struct ti_uart_config {
	__be16	wBaudRate;
	__be16	wFlags;
	u8	bDataBits;
	u8	bParity;
	u8	bStopBits;
	char	cXon;
	char	cXoff;
	u8	bUartMode;
} __packed;

/* Get port status */
struct ti_port_status {
	u8 bCmdCode;
	u8 bModuleId;
	u8 bErrorCode;
	u8 bMSR;
	u8 bLSR;
} __packed;

/* Purge modes */
#define TI_PURGE_OUTPUT			0x00
#define TI_PURGE_INPUT			0x80

/* Read/Write data */
#define TI_RW_DATA_ADDR_SFR		0x10
#define TI_RW_DATA_ADDR_IDATA		0x20
#define TI_RW_DATA_ADDR_XDATA		0x30
#define TI_RW_DATA_ADDR_CODE		0x40
#define TI_RW_DATA_ADDR_GPIO		0x50
#define TI_RW_DATA_ADDR_I2C		0x60
#define TI_RW_DATA_ADDR_FLASH		0x70
#define TI_RW_DATA_ADDR_DSP		0x80

#define TI_RW_DATA_UNSPECIFIED		0x00
#define TI_RW_DATA_BYTE			0x01
#define TI_RW_DATA_WORD			0x02
#define TI_RW_DATA_DOUBLE_WORD		0x04

struct ti_write_data_bytes {
	u8	bAddrType;
	u8	bDataType;
	u8	bDataCounter;
	__be16	wBaseAddrHi;
	__be16	wBaseAddrLo;
	u8	bData[0];
} __packed;

/* Interrupt codes */
static inline int ti_get_port_from_code(unsigned char code)
{
	return (code >> 4) - 3;
}

static inline int ti_get_func_from_code(unsigned char code)
{
	return code & 0x0f;
}

#define TI_CODE_HARDWARE_ERROR		0xFF
#define TI_CODE_DATA_ERROR		0x03
#define TI_CODE_MODEM_STATUS		0x04

/* Download firmware max packet size */
#define TI_DOWNLOAD_MAX_PACKET_SIZE	64

/* Firmware image header */
struct ti_firmware_header {
	__le16	wLength;
	u8	bCheckSum;
} __packed;

/* UART addresses */
#define TI_UART1_BASE_ADDR		0xFFA0	/* UART 1 base address */
#define TI_UART2_BASE_ADDR		0xFFB0	/* UART 2 base address */
#define TI_UART_OFFSET_LCR		0x0002	/* UART MCR register offset */
#define TI_UART_OFFSET_MCR		0x0004	/* UART MCR register offset */

#define TI_DRIVER_AUTHOR	"Al Borchers <alborchers@steinerpoint.com>"
#define TI_DRIVER_DESC		"TI USB 3410/5052 Serial Driver"

#define TI_3410_BAUD_BASE       923077
#define TI_5052_BAUD_BASE       461538

#define TI_FIRMWARE_BUF_SIZE	16284
#define TI_TRANSFER_TIMEOUT	2
#define TI_DOWNLOAD_TIMEOUT	1000
#define TI_DEFAULT_CLOSING_WAIT	4000		/* in .01 secs */

#define TI_EXTRA_VID_PID_COUNT	5

struct ti_port {
	u8			tp_msr;
	u8			tp_shadow_mcr;
	u8			tp_uart_mode;	/* 232 or 485 modes */
	unsigned int		tp_uart_base_addr;
	struct ti_device	*tp_tdev;
	struct usb_serial_port	*tp_port;
	spinlock_t		tp_lock;
};

struct ti_device {
	struct mutex		td_open_close_lock;
	int			td_open_port_count;
	int			td_is_3410;
	int			td_model;
};

static int ti_startup(struct usb_serial *serial);
static void ti_release(struct usb_serial *serial);
static int ti_port_probe(struct usb_serial_port *port);
static int ti_port_remove(struct usb_serial_port *port);
static int ti_open(struct tty_struct *tty, struct usb_serial_port *port);
static void ti_close(struct usb_serial_port *port);
static bool ti_tx_empty(struct usb_serial_port *port);
static int ti_ioctl(struct tty_struct *tty,
		unsigned int cmd, unsigned long arg);
static void ti_set_termios(struct tty_struct *tty,
		struct usb_serial_port *port, struct ktermios *old_termios);
static int ti_tiocmget(struct tty_struct *tty);
static int ti_tiocmset(struct tty_struct *tty,
		unsigned int set, unsigned int clear);
static void ti_break(struct tty_struct *tty, int break_state);
static void ti_interrupt_callback(struct urb *urb);

static int ti_set_mcr(struct ti_port *tport, unsigned int mcr);
static int ti_get_lsr(struct ti_port *tport, u8 *lsr);
static int ti_get_serial_info(struct ti_port *tport,
	struct serial_struct __user *ret_arg);
static int ti_set_serial_info(struct tty_struct *tty, struct ti_port *tport,
	struct serial_struct __user *new_arg);
static void ti_handle_new_msr(struct usb_serial_port *port, u8 msr);
static int ti_download_firmware(struct usb_serial *serial);

static const struct usb_device_id ti_id_table_3410[] = {
	{ USB_DEVICE(TI_VENDOR_ID, TI_3410_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_3410_EZ430_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_GSM_NO_FW_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_CDMA_NO_FW_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_CDMA_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_GSM_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_EDGE_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_MT9234MU_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_MT9234ZBA_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_MT9234ZBAOLD_PRODUCT_ID) },
	{ USB_DEVICE(IBM_VENDOR_ID, IBM_4543_PRODUCT_ID) },
	{ USB_DEVICE(IBM_VENDOR_ID, IBM_454B_PRODUCT_ID) },
	{ USB_DEVICE(IBM_VENDOR_ID, IBM_454C_PRODUCT_ID) },
	{ USB_DEVICE(ABBOTT_VENDOR_ID, ABBOTT_STEREO_PLUG_ID) },
	{ USB_DEVICE(ABBOTT_VENDOR_ID, ABBOTT_STRIP_PORT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, FRI2_PRODUCT_ID) },
	{ USB_DEVICE(HONEYWELL_VENDOR_ID, HONEYWELL_HGI80_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1110_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1130_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1150_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1151_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1131_PRODUCT_ID) },
	{ }
};

static const struct usb_device_id ti_id_table_5052[] = {
	{ USB_DEVICE(TI_VENDOR_ID, TI_5052_BOOT_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_5152_BOOT_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_5052_EEPROM_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_5052_FIRMWARE_PRODUCT_ID) },
	{ }
};

static const struct usb_device_id ti_id_table_combined[] = {
	{ USB_DEVICE(TI_VENDOR_ID, TI_3410_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_3410_EZ430_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_GSM_NO_FW_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_CDMA_NO_FW_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_CDMA_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_GSM_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_EDGE_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_MT9234MU_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_MT9234ZBA_PRODUCT_ID) },
	{ USB_DEVICE(MTS_VENDOR_ID, MTS_MT9234ZBAOLD_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_5052_BOOT_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_5152_BOOT_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_5052_EEPROM_PRODUCT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, TI_5052_FIRMWARE_PRODUCT_ID) },
	{ USB_DEVICE(IBM_VENDOR_ID, IBM_4543_PRODUCT_ID) },
	{ USB_DEVICE(IBM_VENDOR_ID, IBM_454B_PRODUCT_ID) },
	{ USB_DEVICE(IBM_VENDOR_ID, IBM_454C_PRODUCT_ID) },
	{ USB_DEVICE(ABBOTT_VENDOR_ID, ABBOTT_PRODUCT_ID) },
	{ USB_DEVICE(ABBOTT_VENDOR_ID, ABBOTT_STRIP_PORT_ID) },
	{ USB_DEVICE(TI_VENDOR_ID, FRI2_PRODUCT_ID) },
	{ USB_DEVICE(HONEYWELL_VENDOR_ID, HONEYWELL_HGI80_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1110_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1130_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1150_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1151_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1131_PRODUCT_ID) },
	{ }
};

static struct usb_serial_driver ti_1port_device = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "ti_usb_3410_5052_1",
	},
	.description		= "TI USB 3410 1 port adapter",
	.id_table		= ti_id_table_3410,
	.num_ports		= 1,
	.attach			= ti_startup,
	.release		= ti_release,
	.port_probe		= ti_port_probe,
	.port_remove		= ti_port_remove,
	.open			= ti_open,
	.close			= ti_close,
	.tx_empty		= ti_tx_empty,
	.ioctl			= ti_ioctl,
	.set_termios		= ti_set_termios,
	.tiocmget		= ti_tiocmget,
	.tiocmset		= ti_tiocmset,
	.tiocmiwait		= usb_serial_generic_tiocmiwait,
	.get_icount		= usb_serial_generic_get_icount,
	.break_ctl		= ti_break,
	.read_int_callback	= ti_interrupt_callback,
};

static struct usb_serial_driver ti_2port_device = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "ti_usb_3410_5052_2",
	},
	.description		= "TI USB 5052 2 port adapter",
	.id_table		= ti_id_table_5052,
	.num_ports		= 2,
	.attach			= ti_startup,
	.release		= ti_release,
	.port_probe		= ti_port_probe,
	.port_remove		= ti_port_remove,
	.open			= ti_open,
	.close			= ti_close,
	.tx_empty		= ti_tx_empty,
	.ioctl			= ti_ioctl,
	.set_termios		= ti_set_termios,
	.tiocmget		= ti_tiocmget,
	.tiocmset		= ti_tiocmset,
	.tiocmiwait		= usb_serial_generic_tiocmiwait,
	.get_icount		= usb_serial_generic_get_icount,
	.break_ctl		= ti_break,
	.read_int_callback	= ti_interrupt_callback,
};

static struct usb_serial_driver * const serial_drivers[] = {
	&ti_1port_device, &ti_2port_device, NULL
};

MODULE_AUTHOR(TI_DRIVER_AUTHOR);
MODULE_DESCRIPTION(TI_DRIVER_DESC);
MODULE_LICENSE("GPL");

MODULE_FIRMWARE("ti_3410.fw");
MODULE_FIRMWARE("ti_5052.fw");
MODULE_FIRMWARE("mts_cdma.fw");
MODULE_FIRMWARE("mts_gsm.fw");
MODULE_FIRMWARE("mts_edge.fw");
MODULE_FIRMWARE("mts_mt9234mu.fw");
MODULE_FIRMWARE("mts_mt9234zba.fw");
MODULE_FIRMWARE("moxa/moxa-1110.fw");
MODULE_FIRMWARE("moxa/moxa-1130.fw");
MODULE_FIRMWARE("moxa/moxa-1131.fw");
MODULE_FIRMWARE("moxa/moxa-1150.fw");
MODULE_FIRMWARE("moxa/moxa-1151.fw");

MODULE_DEVICE_TABLE(usb, ti_id_table_combined);

module_usb_serial_driver(serial_drivers, ti_id_table_combined);

static int ti_send_ctrl_data_urb(struct usb_serial *serial, u8 request,
				 u16 value, u16 index, void *data, size_t size)
{
	int status;

	status = usb_control_msg(serial->dev,
				 usb_sndctrlpipe(serial->dev, 0),
				 request,
				 (USB_TYPE_VENDOR | USB_RECIP_DEVICE |
				  USB_DIR_OUT), value, index,
				 data, size,
				 USB_CTRL_SET_TIMEOUT);
	if (status < 0) {
		dev_err(&serial->interface->dev,
			"%s - usb_control_msg failed: %d\n",
			__func__, status);
		return status;
	}

	if (status != size) {
		dev_err(&serial->interface->dev,
			"%s - short write (%d / %zd)\n",
			__func__, status, size);
		return -EIO;
	}

	return 0;
}

static int ti_send_ctrl_urb(struct usb_serial *serial,
			    u8 request, u16 value, u16 index)
{
	return ti_send_ctrl_data_urb(serial, request, value, index,
				     NULL, 0);
}

static int ti_recv_ctrl_urb(struct usb_serial *serial, u8 request,
			    u16 value, u16 index, void *data, size_t size)
{
	int status;

	status = usb_control_msg(serial->dev,
				 usb_rcvctrlpipe(serial->dev, 0),
				 request,
				 (USB_TYPE_VENDOR | USB_RECIP_DEVICE |
				  USB_DIR_IN), value, index,
				 data, size,
				 USB_CTRL_SET_TIMEOUT);
	if (status < 0) {
		dev_err(&serial->interface->dev,
			"%s - usb_control_msg failed: %d\n",
			__func__, status);
		return status;
	}

	if (status != size) {
		dev_err(&serial->interface->dev,
			"%s - short read (%d / %zd)\n",
			__func__, status, size);
		return -EIO;
	}

	return 0;
}

static int ti_write_byte(struct usb_serial_port *port, u32 addr,
			u8 mask, u8 byte)
{
	int status;
	size_t size;
	struct ti_write_data_bytes *data;

	dev_dbg(&port->dev, "%s - addr 0x%08X, mask 0x%02X, byte 0x%02X\n",
		__func__, addr, mask, byte);

	size = sizeof(struct ti_write_data_bytes) + 2;
	data = kmalloc(size, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->bAddrType = TI_RW_DATA_ADDR_XDATA;
	data->bDataType = TI_RW_DATA_BYTE;
	data->bDataCounter = 1;
	data->wBaseAddrHi = cpu_to_be16(addr >> 16);
	data->wBaseAddrLo = cpu_to_be16(addr);
	data->bData[0] = mask;
	data->bData[1] = byte;

	status = ti_send_ctrl_data_urb(port->serial, TI_WRITE_DATA, 0,
				       TI_RAM_PORT, data, size);
	if (status < 0)
		dev_err(&port->dev, "%s - failed, %d\n", __func__, status);

	kfree(data);

	return status;
}

static int ti_startup(struct usb_serial *serial)
{
	struct ti_device *tdev;
	struct usb_device *dev = serial->dev;
	struct usb_host_interface *cur_altsetting;
	int status;

	dev_dbg(&dev->dev,
		"%s - product 0x%4X, num configurations %d, configuration value %d\n",
		__func__, le16_to_cpu(dev->descriptor.idProduct),
		dev->descriptor.bNumConfigurations,
		dev->actconfig->desc.bConfigurationValue);

	tdev = kzalloc(sizeof(struct ti_device), GFP_KERNEL);
	if (!tdev)
		return -ENOMEM;

	mutex_init(&tdev->td_open_close_lock);
 	usb_set_serial_data(serial, tdev);

	/* determine device type */
	if (serial->type == &ti_1port_device)
		tdev->td_is_3410 = 1;
	dev_dbg(&dev->dev, "%s - device type is %s\n", __func__,
		tdev->td_is_3410 ? "3410" : "5052");

	tdev->td_model = le16_to_cpu(dev->descriptor.idProduct);

	cur_altsetting = serial->interface->cur_altsetting;

	/* if we have only 1 configuration and 1 endpoint, download firmware */
	if (dev->descriptor.bNumConfigurations == 1 &&
	    cur_altsetting->desc.bNumEndpoints == 1) {
		status = ti_download_firmware(serial);

		if (status != 0)
			goto free_tdev;

		/* 3410 must be reset, 5052 resets itself */
		if (tdev->td_is_3410) {
			msleep_interruptible(100);
			usb_reset_device(dev);
		}

		status = -ENODEV;
		goto free_tdev;
	}

	/* the second configuration must be set */
	if (dev->actconfig->desc.bConfigurationValue == TI_BOOT_CONFIG) {
		status = usb_driver_set_configuration(dev, TI_ACTIVE_CONFIG);
		status = status ? status : -ENODEV;
		goto free_tdev;
	}

	return 0;

free_tdev:
	kfree(tdev);
	usb_set_serial_data(serial, NULL);
	return status;
}


static void ti_release(struct usb_serial *serial)
{
	struct ti_device *tdev = usb_get_serial_data(serial);

	kfree(tdev);
}

static int ti_port_probe(struct usb_serial_port *port)
{
	struct ti_port *tport;

	tport = kzalloc(sizeof(*tport), GFP_KERNEL);
	if (!tport)
		return -ENOMEM;

	spin_lock_init(&tport->tp_lock);
	if (port == port->serial->port[0])
		tport->tp_uart_base_addr = TI_UART1_BASE_ADDR;
	else
		tport->tp_uart_base_addr = TI_UART2_BASE_ADDR;

	tport->tp_port = port;
	tport->tp_tdev = usb_get_serial_data(port->serial);

	switch (tport->tp_tdev->td_model) {
	case MXU1_1130_PRODUCT_ID:
	case MXU1_1131_PRODUCT_ID:
		tport->tp_uart_mode = TI_UART_485_RECEIVER_DISABLED;
		break;
	default:
		tport->tp_uart_mode = TI_UART_232;	/* default is RS232 */
	}

	usb_set_serial_port_data(port, tport);

	port->port.closing_wait =
			msecs_to_jiffies(TI_DEFAULT_CLOSING_WAIT * 10);
	port->port.drain_delay = 3;

	return 0;
}

static int ti_port_remove(struct usb_serial_port *port)
{
	struct ti_port *tport;

	tport = usb_get_serial_port_data(port);
	kfree(tport);

	return 0;
}

static int ti_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct ti_port *tport = usb_get_serial_port_data(port);
	struct usb_serial *serial = port->serial;
	struct ti_device *tdev;
	struct urb *urb;
	int port_number;
	int status;
	u16 open_settings;

	open_settings = (TI_PIPE_MODE_CONTINOUS |
			 TI_PIPE_TIMEOUT_ENABLE |
			 (TI_TRANSFER_TIMEOUT << 2));

	tdev = tport->tp_tdev;

	/* only one open on any port on a device at a time */
	if (mutex_lock_interruptible(&tdev->td_open_close_lock))
		return -ERESTARTSYS;

	port_number = port->port_number;

	tport->tp_msr = 0;
	tport->tp_shadow_mcr |= (TI_MCR_RTS | TI_MCR_DTR);

	/* start interrupt urb the first time a port is opened on this device */
	if (tdev->td_open_port_count == 0) {
		dev_dbg(&port->dev, "%s - start interrupt in urb\n", __func__);
		urb = serial->port[0]->interrupt_in_urb;
		if (!urb) {
			dev_err(&port->dev, "%s - no interrupt urb\n", __func__);
			status = -EINVAL;
			goto release_lock;
		}
		status = usb_submit_urb(urb, GFP_KERNEL);
		if (status) {
			dev_err(&port->dev, "%s - submit interrupt urb failed, %d\n", __func__, status);
			goto release_lock;
		}
	}

	if (tty)
		ti_set_termios(tty, port, &tty->termios);

	status = ti_send_ctrl_urb(serial, TI_OPEN_PORT, open_settings,
				  TI_UART1_PORT + port_number);
	if (status) {
		dev_err(&port->dev, "%s - cannot send open command, %d\n",
			__func__, status);
		goto unlink_int_urb;
	}

	status = ti_send_ctrl_urb(serial, TI_START_PORT, 0,
				  TI_UART1_PORT + port_number);
	if (status) {
		dev_err(&port->dev, "%s - cannot send start command, %d\n",
							__func__, status);
		goto unlink_int_urb;
	}

	status = ti_send_ctrl_urb(serial, TI_PURGE_INPUT, TI_PURGE_INPUT,
				  TI_UART1_PORT + port_number);
	if (status) {
		dev_err(&port->dev, "%s - cannot clear input buffers, %d\n",
							__func__, status);
		goto unlink_int_urb;
	}

	status = ti_send_ctrl_urb(serial, TI_PURGE_INPUT, TI_PURGE_OUTPUT,
				  TI_UART1_PORT + port_number);
	if (status) {
		dev_err(&port->dev, "%s - cannot clear output buffers, %d\n",
							__func__, status);
		goto unlink_int_urb;
	}

	/* reset the data toggle on the bulk endpoints to work around bug in
	 * host controllers where things get out of sync some times */
	usb_clear_halt(serial->dev, port->write_urb->pipe);
	usb_clear_halt(serial->dev, port->read_urb->pipe);

	if (tty)
		ti_set_termios(tty, port, &tty->termios);

	status = ti_send_ctrl_urb(serial, TI_OPEN_PORT, open_settings,
				  TI_UART1_PORT + port_number);
	if (status) {
		dev_err(&port->dev, "%s - cannot send open command (2), %d\n",
							__func__, status);
		goto unlink_int_urb;
	}

	status = ti_send_ctrl_urb(serial, TI_START_PORT, 0,
				  TI_UART1_PORT + port_number);
	if (status) {
		dev_err(&port->dev, "%s - cannot send start command (2), %d\n",
							__func__, status);
		goto unlink_int_urb;
	}

	/* start read urb */
	urb = port->read_urb;
	if (!urb) {
		dev_err(&port->dev, "%s - no read urb\n", __func__);
		status = -EINVAL;
		goto unlink_int_urb;
	}
	urb->context = tport;
	status = usb_submit_urb(urb, GFP_KERNEL);
	if (status) {
		dev_err(&port->dev, "%s - submit read urb failed, %d\n",
							__func__, status);
		goto unlink_int_urb;
	}

	++tdev->td_open_port_count;

	goto release_lock;

unlink_int_urb:
	if (tdev->td_open_port_count == 0)
		usb_kill_urb(port->serial->port[0]->interrupt_in_urb);
release_lock:
	mutex_unlock(&tdev->td_open_close_lock);
	return status;
}


static void ti_close(struct usb_serial_port *port)
{
	struct ti_device *tdev;
	struct ti_port *tport;
	int port_number;
	int status;
	int do_unlock;
	unsigned long flags;

	tdev = usb_get_serial_data(port->serial);
	tport = usb_get_serial_port_data(port);

	usb_kill_urb(port->read_urb);
	usb_kill_urb(port->write_urb);
	spin_lock_irqsave(&tport->tp_lock, flags);
	kfifo_reset_out(&port->write_fifo);
	spin_unlock_irqrestore(&tport->tp_lock, flags);

	port_number = port->port_number;

	status = ti_send_ctrl_urb(port->serial, TI_CLOSE_PORT, 0,
				  TI_UART1_PORT + port_number);
	if (status)
		dev_err(&port->dev,
			"%s - cannot send close port command, %d\n"
							, __func__, status);

	/* if mutex_lock is interrupted, continue anyway */
	do_unlock = !mutex_lock_interruptible(&tdev->td_open_close_lock);
	--tport->tp_tdev->td_open_port_count;
	if (tport->tp_tdev->td_open_port_count <= 0) {
		/* last port is closed, shut down interrupt urb */
		usb_kill_urb(port->serial->port[0]->interrupt_in_urb);
		tport->tp_tdev->td_open_port_count = 0;
	}
	if (do_unlock)
		mutex_unlock(&tdev->td_open_close_lock);
}

static bool ti_tx_empty(struct usb_serial_port *port)
{
	struct ti_port *tport = usb_get_serial_port_data(port);
	int ret;
	u8 lsr;

	ret = ti_get_lsr(tport, &lsr);
	if (!ret && !(lsr & TI_LSR_TX_EMPTY))
		return false;

	return true;
}

static int ti_ioctl(struct tty_struct *tty,
	unsigned int cmd, unsigned long arg)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ti_port *tport = usb_get_serial_port_data(port);

	switch (cmd) {
	case TIOCGSERIAL:
		return ti_get_serial_info(tport,
				(struct serial_struct __user *)arg);
	case TIOCSSERIAL:
		return ti_set_serial_info(tty, tport,
				(struct serial_struct __user *)arg);
	}
	return -ENOIOCTLCMD;
}


static void ti_set_termios(struct tty_struct *tty,
		struct usb_serial_port *port, struct ktermios *old_termios)
{
	struct ti_port *tport = usb_get_serial_port_data(port);
	struct ti_uart_config *config;
	tcflag_t cflag, iflag;
	int baud;
	int status;
	int port_number = port->port_number;
	unsigned int mcr;

	cflag = tty->termios.c_cflag;
	iflag = tty->termios.c_iflag;

	dev_dbg(&port->dev,
		"%s - cflag 0x%08x, iflag 0x%08x\n", __func__, cflag, iflag);

	if (old_termios) {
		dev_dbg(&port->dev, "%s - old clfag 0x%08x, old iflag 0x%08x\n",
			__func__,
			old_termios->c_cflag,
			old_termios->c_iflag);
	}

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config)
		return;

	/* these flags must be set */
	config->wFlags |= TI_UART_ENABLE_MS_INTS;
	config->wFlags |= TI_UART_ENABLE_AUTO_START_DMA;
	config->bUartMode = tport->tp_uart_mode;

	switch (C_CSIZE(tty)) {
	case CS5:
		    config->bDataBits = TI_UART_5_DATA_BITS;
		    break;
	case CS6:
		    config->bDataBits = TI_UART_6_DATA_BITS;
		    break;
	case CS7:
		    config->bDataBits = TI_UART_7_DATA_BITS;
		    break;
	default:
	case CS8:
		    config->bDataBits = TI_UART_8_DATA_BITS;
		    break;
	}

	/* CMSPAR isn't supported by this driver */
	tty->termios.c_cflag &= ~CMSPAR;

	if (C_PARENB(tty)) {
		if (C_PARODD(tty)) {
			config->wFlags |= TI_UART_ENABLE_PARITY_CHECKING;
			config->bParity = TI_UART_ODD_PARITY;
		} else {
			config->wFlags |= TI_UART_ENABLE_PARITY_CHECKING;
			config->bParity = TI_UART_EVEN_PARITY;
		}
	} else {
		config->wFlags &= ~TI_UART_ENABLE_PARITY_CHECKING;
		config->bParity = TI_UART_NO_PARITY;
	}

	if (C_CSTOPB(tty))
		config->bStopBits = TI_UART_2_STOP_BITS;
	else
		config->bStopBits = TI_UART_1_STOP_BITS;

	if (C_CRTSCTS(tty)) {
		/* RTS flow control must be off to drop RTS for baud rate B0 */
		if ((C_BAUD(tty)) != B0)
			config->wFlags |= TI_UART_ENABLE_RTS_IN;
		config->wFlags |= TI_UART_ENABLE_CTS_OUT;
	}

	if (I_IXOFF(tty) || I_IXON(tty)) {
		config->cXon  = START_CHAR(tty);
		config->cXoff = STOP_CHAR(tty);

		if (I_IXOFF(tty))
			config->wFlags |= TI_UART_ENABLE_X_IN;

		if (I_IXON(tty))
			config->wFlags |= TI_UART_ENABLE_X_OUT;
	}

	baud = tty_get_baud_rate(tty);
	if (!baud)
		baud = 9600;
	if (tport->tp_tdev->td_is_3410)
		config->wBaudRate = (TI_3410_BAUD_BASE + baud / 2) / baud;
	else
		config->wBaudRate = (TI_5052_BAUD_BASE + baud / 2) / baud;

	/* FIXME: Should calculate resulting baud here and report it back */
	if ((C_BAUD(tty)) != B0)
		tty_encode_baud_rate(tty, baud, baud);

	dev_dbg(&port->dev,
		"%s - BaudRate=%d, wBaudRate=%d, wFlags=0x%04X, bDataBits=%d, bParity=%d, bStopBits=%d, cXon=%d, cXoff=%d, bUartMode=%d\n",
		__func__, baud, config->wBaudRate, config->wFlags,
		config->bDataBits, config->bParity, config->bStopBits,
		config->cXon, config->cXoff, config->bUartMode);

	cpu_to_be16s(&config->wBaudRate);
	cpu_to_be16s(&config->wFlags);

	status = ti_send_ctrl_data_urb(port->serial, TI_SET_CONFIG, 0,
				       TI_UART1_PORT + port_number, config,
				       sizeof(*config));
	if (status)
		dev_err(&port->dev, "%s - cannot set config on port %d, %d\n",
					__func__, port_number, status);

	/* SET_CONFIG asserts RTS and DTR, reset them correctly */
	mcr = tport->tp_shadow_mcr;
	/* if baud rate is B0, clear RTS and DTR */
	if ((C_BAUD(tty)) == B0)
		mcr &= ~(TI_MCR_DTR | TI_MCR_RTS);
	status = ti_set_mcr(tport, mcr);
	if (status)
		dev_err(&port->dev,
			"%s - cannot set modem control on port %d, %d\n",
						__func__, port_number, status);

	kfree(config);
}


static int ti_tiocmget(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ti_port *tport = usb_get_serial_port_data(port);
	unsigned int result;
	unsigned int msr;
	unsigned int mcr;
	unsigned long flags;

	spin_lock_irqsave(&tport->tp_lock, flags);
	msr = tport->tp_msr;
	mcr = tport->tp_shadow_mcr;
	spin_unlock_irqrestore(&tport->tp_lock, flags);

	result = ((mcr & TI_MCR_DTR) ? TIOCM_DTR : 0)
		| ((mcr & TI_MCR_RTS) ? TIOCM_RTS : 0)
		| ((mcr & TI_MCR_LOOP) ? TIOCM_LOOP : 0)
		| ((msr & TI_MSR_CTS) ? TIOCM_CTS : 0)
		| ((msr & TI_MSR_CD) ? TIOCM_CAR : 0)
		| ((msr & TI_MSR_RI) ? TIOCM_RI : 0)
		| ((msr & TI_MSR_DSR) ? TIOCM_DSR : 0);

	dev_dbg(&port->dev, "%s - 0x%04X\n", __func__, result);

	return result;
}


static int ti_tiocmset(struct tty_struct *tty,
				unsigned int set, unsigned int clear)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ti_port *tport = usb_get_serial_port_data(port);
	unsigned int mcr;
	unsigned long flags;

	spin_lock_irqsave(&tport->tp_lock, flags);
	mcr = tport->tp_shadow_mcr;

	if (set & TIOCM_RTS)
		mcr |= TI_MCR_RTS;
	if (set & TIOCM_DTR)
		mcr |= TI_MCR_DTR;
	if (set & TIOCM_LOOP)
		mcr |= TI_MCR_LOOP;

	if (clear & TIOCM_RTS)
		mcr &= ~TI_MCR_RTS;
	if (clear & TIOCM_DTR)
		mcr &= ~TI_MCR_DTR;
	if (clear & TIOCM_LOOP)
		mcr &= ~TI_MCR_LOOP;
	spin_unlock_irqrestore(&tport->tp_lock, flags);

	return ti_set_mcr(tport, mcr);
}


static void ti_break(struct tty_struct *tty, int break_state)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ti_port *tport = usb_get_serial_port_data(port);
	int status;

	status = ti_write_byte(port,
		tport->tp_uart_base_addr + TI_UART_OFFSET_LCR,
		TI_LCR_BREAK, break_state == -1 ? TI_LCR_BREAK : 0);
	if (status)
		dev_dbg(&port->dev, "%s - error setting break, %d\n", __func__, status);
}


static void ti_interrupt_callback(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	unsigned char *data = urb->transfer_buffer;
	int length = urb->actual_length;
	int port_number;
	int function;
	int status = urb->status;
	u8 msr;

	switch (status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		dev_dbg(&port->dev, "%s - urb shutting down, %d\n",
			__func__, status);
		return;
	default:
		dev_err(&port->dev, "%s - nonzero urb status, %d\n",
			__func__, status);
		goto exit;
	}

	if (length != 2) {
		dev_dbg(&port->dev, "%s - bad packet size, %d\n",
			__func__, length);
		goto exit;
	}

	if (data[0] == TI_CODE_HARDWARE_ERROR) {
		dev_err(&port->dev, "%s - hardware error, %d\n",
			__func__, data[1]);
		goto exit;
	}

	port_number = ti_get_port_from_code(data[0]);
	function = ti_get_func_from_code(data[0]);

	dev_dbg(&port->dev, "%s - port_number %d, function %d, data 0x%02X\n",
		__func__, port_number, function, data[1]);

	if (port_number >= port->serial->num_ports) {
		dev_err(&port->dev, "%s - bad port number, %d\n",
						__func__, port_number);
		goto exit;
	}

	switch (function) {
	case TI_CODE_DATA_ERROR:
		dev_err(&port->dev, "%s - DATA ERROR, port %d, data 0x%02X\n",
			__func__, port_number, data[1]);
		break;

	case TI_CODE_MODEM_STATUS:
		msr = data[1];
		ti_handle_new_msr(port, msr);
		break;

	default:
		dev_err(&port->dev, "%s - unknown interrupt code, 0x%02X\n",
							__func__, data[1]);
		break;
	}

exit:
	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status)
		dev_err(&port->dev, "%s - resubmit interrupt urb failed, %d\n",
			__func__, status);
}

static int ti_set_mcr(struct ti_port *tport, unsigned int mcr)
{
	unsigned long flags;
	int status;

	status = ti_write_byte(tport->tp_port,
		tport->tp_uart_base_addr + TI_UART_OFFSET_MCR,
		TI_MCR_RTS | TI_MCR_DTR | TI_MCR_LOOP, mcr);

	spin_lock_irqsave(&tport->tp_lock, flags);
	if (!status)
		tport->tp_shadow_mcr = mcr;
	spin_unlock_irqrestore(&tport->tp_lock, flags);

	return status;
}


static int ti_get_lsr(struct ti_port *tport, u8 *lsr)
{
	int size, status;
	struct usb_serial_port *port = tport->tp_port;
	int port_number = port->port_number;
	struct ti_port_status *data;

	size = sizeof(struct ti_port_status);
	data = kmalloc(size, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	status = ti_recv_ctrl_urb(port->serial, TI_GET_PORT_STATUS, 0,
				  TI_UART1_PORT + port_number, data, size);
	if (status) {
		dev_err(&port->dev,
			"%s - get port status command failed, %d\n",
							__func__, status);
		goto free_data;
	}

	dev_dbg(&port->dev, "%s - lsr 0x%02X\n", __func__, data->bLSR);

	*lsr = data->bLSR;

free_data:
	kfree(data);
	return status;
}


static int ti_get_serial_info(struct ti_port *tport,
	struct serial_struct __user *ret_arg)
{
	struct usb_serial_port *port = tport->tp_port;
	struct serial_struct ret_serial;
	unsigned cwait;
	int baud_base;

	if (!ret_arg)
		return -EFAULT;

	cwait = port->port.closing_wait;
	if (cwait != ASYNC_CLOSING_WAIT_NONE)
		cwait = jiffies_to_msecs(cwait) / 10;

	memset(&ret_serial, 0, sizeof(ret_serial));

	if (tport->tp_tdev->td_is_3410)
		baud_base = TI_3410_BAUD_BASE;
	else
		baud_base = TI_5052_BAUD_BASE;

	ret_serial.type = PORT_16550A;
	ret_serial.line = port->minor;
	ret_serial.port = port->port_number;
	ret_serial.xmit_fifo_size = port->bulk_out_size;
	ret_serial.baud_base = baud_base;
	ret_serial.closing_wait = cwait;

	if (copy_to_user(ret_arg, &ret_serial, sizeof(*ret_arg)))
		return -EFAULT;

	return 0;
}


static int ti_set_serial_info(struct tty_struct *tty, struct ti_port *tport,
	struct serial_struct __user *new_arg)
{
	struct serial_struct new_serial;
	unsigned cwait;

	if (copy_from_user(&new_serial, new_arg, sizeof(new_serial)))
		return -EFAULT;

	cwait = new_serial.closing_wait;
	if (cwait != ASYNC_CLOSING_WAIT_NONE)
		cwait = msecs_to_jiffies(10 * new_serial.closing_wait);

	tport->tp_port->port.closing_wait = cwait;

	return 0;
}


static void ti_handle_new_msr(struct usb_serial_port *port, u8 msr)
{
	struct ti_port *tport = usb_get_serial_port_data(port);
	struct async_icount *icount;
	unsigned long flags;

	dev_dbg(&port->dev, "%s - msr 0x%02X\n", __func__, msr);

	if (msr & TI_MSR_DELTA_MASK) {
		spin_lock_irqsave(&tport->tp_lock, flags);
		icount = &tport->tp_port->icount;
		if (msr & TI_MSR_DELTA_CTS)
			icount->cts++;
		if (msr & TI_MSR_DELTA_DSR)
			icount->dsr++;
		if (msr & TI_MSR_DELTA_CD)
			icount->dcd++;
		if (msr & TI_MSR_DELTA_RI)
			icount->rng++;
		wake_up_interruptible(&port->port.delta_msr_wait);
		spin_unlock_irqrestore(&tport->tp_lock, flags);
	}

	tport->tp_msr = msr & TI_MSR_MASK;
}

static int ti_do_download(struct usb_device *dev, int pipe,
						u8 *buffer, int size)
{
	int pos;
	u8 cs = 0;
	int done;
	struct ti_firmware_header *header;
	int status = 0;
	int len;

	for (pos = sizeof(*header); pos < size; pos++)
		cs = (u8)(cs + buffer[pos]);

	header = (struct ti_firmware_header *)buffer;
	header->wLength = cpu_to_le16(size - sizeof(*header));
	header->bCheckSum = cs;

	dev_dbg(&dev->dev, "%s - downloading firmware\n", __func__);
	for (pos = 0; pos < size; pos += done) {
		len = min(size - pos, TI_DOWNLOAD_MAX_PACKET_SIZE);
		status = usb_bulk_msg(dev, pipe, buffer + pos, len, &done,
				      TI_DOWNLOAD_TIMEOUT);
		if (status)
			break;
	}
	return status;
}

static int ti_download_firmware(struct usb_serial *serial)
{
	int status;
	int buffer_size;
	u8 *buffer;
	struct usb_device *dev = serial->dev;
	struct ti_device *tdev = usb_get_serial_data(serial);
	unsigned int pipe;
	const struct firmware *fw_p;
	char buf[32];

	pipe = usb_sndbulkpipe(dev, serial->port[0]->bulk_out_endpointAddress);

	/* try ID specific firmware first, then try generic firmware */
	sprintf(buf, "ti_usb-v%04x-p%04x.fw",
			le16_to_cpu(dev->descriptor.idVendor),
			le16_to_cpu(dev->descriptor.idProduct));
	status = request_firmware(&fw_p, buf, &dev->dev);

	if (status != 0) {
		buf[0] = '\0';
		if (le16_to_cpu(dev->descriptor.idVendor) == MTS_VENDOR_ID) {
			switch (le16_to_cpu(dev->descriptor.idProduct)) {
			case MTS_CDMA_PRODUCT_ID:
				strcpy(buf, "mts_cdma.fw");
				break;
			case MTS_GSM_PRODUCT_ID:
				strcpy(buf, "mts_gsm.fw");
				break;
			case MTS_EDGE_PRODUCT_ID:
				strcpy(buf, "mts_edge.fw");
				break;
			case MTS_MT9234MU_PRODUCT_ID:
				strcpy(buf, "mts_mt9234mu.fw");
				break;
			case MTS_MT9234ZBA_PRODUCT_ID:
				strcpy(buf, "mts_mt9234zba.fw");
				break;
			case MTS_MT9234ZBAOLD_PRODUCT_ID:
				strcpy(buf, "mts_mt9234zba.fw");
				break;			}
		}

		if (le16_to_cpu(dev->descriptor.idVendor) == MXU1_VENDOR_ID) {
			snprintf(buf,
				sizeof(buf),
				"moxa/moxa-%04x.fw",
				le16_to_cpu(dev->descriptor.idProduct));
		}

		if (buf[0] == '\0') {
			if (tdev->td_is_3410)
				strcpy(buf, "ti_3410.fw");
			else
				strcpy(buf, "ti_5052.fw");
		}
		status = request_firmware(&fw_p, buf, &dev->dev);
	}
	if (status) {
		dev_err(&dev->dev, "%s - firmware not found\n", __func__);
		return -ENOENT;
	}
	if (fw_p->size > TI_FIRMWARE_BUF_SIZE) {
		dev_err(&dev->dev, "%s - firmware too large %zu\n", __func__, fw_p->size);
		release_firmware(fw_p);
		return -ENOENT;
	}

	buffer_size = TI_FIRMWARE_BUF_SIZE + sizeof(struct ti_firmware_header);
	buffer = kmalloc(buffer_size, GFP_KERNEL);
	if (buffer) {
		memcpy(buffer, fw_p->data, fw_p->size);
		memset(buffer + fw_p->size, 0xff, buffer_size - fw_p->size);
		status = ti_do_download(dev, pipe, buffer, fw_p->size);
		kfree(buffer);
	} else {
		status = -ENOMEM;
	}
	release_firmware(fw_p);
	if (status) {
		dev_err(&dev->dev, "%s - error downloading firmware, %d\n",
							__func__, status);
		return status;
	}

	dev_dbg(&dev->dev, "%s - download successful\n", __func__);

	return 0;
}
