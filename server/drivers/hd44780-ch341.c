/** \file server/drivers/hd44780-ch341.c
 * \c ch341i2c connection type of \c hd44780 driver for Hitachi HD44780 based LCD displays.
 *
 * This backend uses a CH341 USB adapter in I2C mode to drive a PCF8574 backpack.
 */

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <usb.h>

#include "hd44780-ch341.h"
#include "hd44780-low.h"
#include "shared/report.h"

#define CH341_VENDOR_ID        0x1A86
#define CH341_PRODUCT_ID       0x5512

#define CH341_DEFAULT_EP_OUT   0x02
#define CH341_DEFAULT_EP_IN    0x82
#define CH341_DEFAULT_IFACE    0
#define CH341_DEFAULT_I2C_ADDR 0x27

#define CH341_USB_TIMEOUT_MS   1000
#define CH341_I2C_RETRIES      3

/* CH341 stream mode commands. */
#define CH341_CMD_I2C_STREAM   0xAA
#define CH341_CMD_I2C_STM_STA  0x74
#define CH341_CMD_I2C_STM_STO  0x75
#define CH341_CMD_I2C_STM_OUT  0x80
#define CH341_CMD_I2C_STM_END  0x00

/* PCF8574 bit mapping used by this backend (do not change). */
#define CH341_LINE_RS          0x01
#define CH341_LINE_RW          0x02
#define CH341_LINE_EN          0x04
#define CH341_LINE_BL          0x08
#define CH341_LINE_D4          0x10
#define CH341_LINE_D5          0x20
#define CH341_LINE_D6          0x40
#define CH341_LINE_D7          0x80

static int ch341_i2c_write(PrivateData *p, unsigned char i2c_addr, const unsigned char *payload, size_t payload_len);
static int ch341_write_port(PrivateData *p, unsigned char port_value);
static int ch341_send_raw(PrivateData *p, unsigned char rs, unsigned char ch);

static void ch341_HD44780_senddata(PrivateData *p, unsigned char displayID, unsigned char flags, unsigned char ch);
static void ch341_HD44780_backlight(PrivateData *p, unsigned char state);
static void ch341_HD44780_close(PrivateData *p);


static int
ch341_i2c_write(PrivateData *p, unsigned char i2c_addr, const unsigned char *payload, size_t payload_len)
{
	unsigned char tx[64];
	size_t tx_len;
	int sent;
	int attempt;

	if ((payload == NULL) || (payload_len == 0)) {
		return -1;
	}
	if (payload_len > (sizeof(tx) - 5U)) {
		return -1;
	}

	tx[0] = CH341_CMD_I2C_STREAM;
	tx[1] = CH341_CMD_I2C_STM_STA;
	tx[2] = CH341_CMD_I2C_STM_OUT | (unsigned char)(payload_len + 1U);
	tx[3] = (unsigned char)(i2c_addr << 1);
	memcpy(&tx[4], payload, payload_len);
	tx[4 + payload_len] = CH341_CMD_I2C_STM_STO;
	tx[5 + payload_len] = CH341_CMD_I2C_STM_END;
	tx_len = 6U + payload_len;

	for (attempt = 0; attempt < CH341_I2C_RETRIES; attempt++) {
		sent = usb_bulk_write(p->usbHandle, p->usbEpOut, (char *) tx, tx_len, CH341_USB_TIMEOUT_MS);
		if (sent == (int) tx_len) {
			return 0;
		}
	}

	p->hd44780_functions->drv_report(RPT_ERR,
		"hd44780-ch341: i2c write failed (addr=0x%02X, len=%d, usb_res=%d)",
		i2c_addr, (int) payload_len, sent);
	return -1;
}


static int
ch341_write_port(PrivateData *p, unsigned char port_value)
{
	return ch341_i2c_write(p, (unsigned char) p->port, &port_value, 1U);
}


static int
ch341_send_raw(PrivateData *p, unsigned char rs, unsigned char ch)
{
	unsigned char ctrl = (unsigned char) ((rs ? CH341_LINE_RS : 0) | p->backlight_bit);
	unsigned char hi = 0;
	unsigned char lo = 0;

	if (ch & 0x80) hi |= CH341_LINE_D7;
	if (ch & 0x40) hi |= CH341_LINE_D6;
	if (ch & 0x20) hi |= CH341_LINE_D5;
	if (ch & 0x10) hi |= CH341_LINE_D4;
	if (ch & 0x08) lo |= CH341_LINE_D7;
	if (ch & 0x04) lo |= CH341_LINE_D6;
	if (ch & 0x02) lo |= CH341_LINE_D5;
	if (ch & 0x01) lo |= CH341_LINE_D4;

	if (ch341_write_port(p, (unsigned char) (ctrl | hi)) < 0)
		return -1;
	if (p->delayBus)
		p->hd44780_functions->uPause(p, 1);
	if (ch341_write_port(p, (unsigned char) (ctrl | hi | CH341_LINE_EN)) < 0)
		return -1;
	if (p->delayBus)
		p->hd44780_functions->uPause(p, 1);
	if (ch341_write_port(p, (unsigned char) (ctrl | hi)) < 0)
		return -1;

	if (ch341_write_port(p, (unsigned char) (ctrl | lo)) < 0)
		return -1;
	if (p->delayBus)
		p->hd44780_functions->uPause(p, 1);
	if (ch341_write_port(p, (unsigned char) (ctrl | lo | CH341_LINE_EN)) < 0)
		return -1;
	if (p->delayBus)
		p->hd44780_functions->uPause(p, 1);
	if (ch341_write_port(p, (unsigned char) (ctrl | lo)) < 0)
		return -1;

	return 0;
}


int
hd_init_ch341i2c(Driver *drvthis)
{
	PrivateData *p = (PrivateData *) drvthis->private_data;
	struct usb_bus *bus;
	struct usb_device *found_dev;
	int iface;

	p->hd44780_functions->senddata = ch341_HD44780_senddata;
	p->hd44780_functions->backlight = ch341_HD44780_backlight;
	p->hd44780_functions->close = ch341_HD44780_close;

	p->port = drvthis->config_get_int(drvthis->name, "Port", 0, CH341_DEFAULT_I2C_ADDR) & 0x7F;
	iface = drvthis->config_get_int(drvthis->name, "USBInterface", 0, CH341_DEFAULT_IFACE);
	p->usbEpOut = drvthis->config_get_int(drvthis->name, "USBEndpointOut", 0, CH341_DEFAULT_EP_OUT);
	p->usbEpIn = drvthis->config_get_int(drvthis->name, "USBEndpointIn", 0, CH341_DEFAULT_EP_IN);
	p->backlight_bit = CH341_LINE_BL;
	p->func_set_mode = FUNCSET | IF_4BIT | TWOLINE | SMALLCHAR;

	usb_init();
	usb_find_busses();
	usb_find_devices();

	p->usbHandle = NULL;
	found_dev = NULL;
	for (bus = usb_get_busses(); bus != NULL; bus = bus->next) {
		struct usb_device *dev;

		for (dev = bus->devices; dev != NULL; dev = dev->next) {
			if ((dev->descriptor.idVendor == CH341_VENDOR_ID) &&
			    (dev->descriptor.idProduct == CH341_PRODUCT_ID)) {
				p->usbHandle = usb_open(dev);
				if (p->usbHandle != NULL) {
					found_dev = dev;
					goto opened;
				}
			}
		}
	}

	report(RPT_ERR, "hd44780-ch341: no CH341 device found (VID=0x%04X PID=0x%04X)",
		CH341_VENDOR_ID, CH341_PRODUCT_ID);
	return -1;

opened:
	if ((found_dev != NULL) && (found_dev->descriptor.bNumConfigurations > 0) &&
	    (usb_set_configuration(p->usbHandle, found_dev->config[0].bConfigurationValue) < 0)) {
		report(RPT_DEBUG, "hd44780-ch341: usb_set_configuration failed (non-fatal): %s",
			strerror(errno));
	}
	if (usb_claim_interface(p->usbHandle, iface) < 0) {
#if defined(LIBUSB_HAS_DETACH_KERNEL_DRIVER_NP)
		if ((usb_detach_kernel_driver_np(p->usbHandle, iface) < 0) ||
		    (usb_claim_interface(p->usbHandle, iface) < 0)) {
			report(RPT_ERR, "hd44780-ch341: unable to claim USB interface %d: %s", iface, strerror(errno));
			usb_close(p->usbHandle);
			p->usbHandle = NULL;
			return -1;
		}
#else
		report(RPT_ERR, "hd44780-ch341: unable to claim USB interface %d: %s", iface, strerror(errno));
		usb_close(p->usbHandle);
		p->usbHandle = NULL;
		return -1;
#endif
	}
	p->usbIndex = iface;

	/* Ensure known idle state before init sequence. */
	(void) ch341_write_port(p, p->backlight_bit);
	p->hd44780_functions->uPause(p, 20000);

	/* Keep manual init sequence exactly as validated on target hardware. */
	ch341_send_raw(p, 0, 0x30);
	p->hd44780_functions->uPause(p, 5000);
	ch341_send_raw(p, 0, 0x30);
	p->hd44780_functions->uPause(p, 5000);
	ch341_send_raw(p, 0, 0x30);
	p->hd44780_functions->uPause(p, 200);
	ch341_send_raw(p, 0, 0x20);
	p->hd44780_functions->uPause(p, 200);

	ch341_send_raw(p, 0, 0x28);
	p->hd44780_functions->uPause(p, 40);
	ch341_send_raw(p, 0, 0x0C);
	p->hd44780_functions->uPause(p, 40);
	ch341_send_raw(p, 0, 0x06);
	p->hd44780_functions->uPause(p, 40);
	ch341_send_raw(p, 0, 0x01);
	p->hd44780_functions->uPause(p, 1600);

	report(RPT_INFO, "hd44780-ch341: initialized CH341 I2C LCD at address 0x%02X", p->port);
	return 0;
}


static void
ch341_HD44780_senddata(PrivateData *p, unsigned char displayID, unsigned char flags, unsigned char ch)
{
	unsigned char rs;

	(void) displayID;
	rs = (flags == RS_INSTR) ? 0 : 1;
	(void) ch341_send_raw(p, rs, ch);
}


static void
ch341_HD44780_backlight(PrivateData *p, unsigned char state)
{
	p->backlight_bit = ((have_backlight_pin(p) && state) ? CH341_LINE_BL : 0);
	(void) ch341_write_port(p, p->backlight_bit);
}


static void
ch341_HD44780_close(PrivateData *p)
{
	if (p->usbHandle != NULL) {
		(void) usb_release_interface(p->usbHandle, p->usbIndex);
		usb_close(p->usbHandle);
		p->usbHandle = NULL;
	}
}

/* EOF */
