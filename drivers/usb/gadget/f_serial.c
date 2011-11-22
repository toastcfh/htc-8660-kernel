/*
 * f_serial.c - generic USB serial function driver
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 by David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/usb/android_composite.h>

#include "u_serial.h"
#include "gadget_chips.h"
#include <linux/wakelock.h>
#include <mach/perflock.h>

static struct wake_lock vbus_idle_wake_lock;
static struct perf_lock usb_perf_lock;

#if defined(CONFIG_ARCH_MSM8X60_LTE)
extern int diag_init_enabled_state;
#endif

#define FSERIAL_DEAFAULT_TTY_NO 3


#define CONFIG_MODEM_SUPPORT
#if defined(CONFIG_MACH_VERDI_LTE) && defined(CONFIG_USB_ANDROID_MTP36)
#define DISABLE_SERIAL_NOTIFY
#endif
/*
 * This function packages a simple "generic serial" port with no real
 * control mechanisms, just raw data transfer over two bulk endpoints.
 *
 * Because it's not standardized, this isn't as interoperable as the
 * CDC ACM driver.  However, for many purposes it's just as functional
 * if you can arrange appropriate host side drivers.
 */

struct gser_descs {
	struct usb_endpoint_descriptor	*in;
	struct usb_endpoint_descriptor	*out;
#ifdef CONFIG_MODEM_SUPPORT
	struct usb_endpoint_descriptor	*notify;
#endif
};

struct f_gser {
	struct gserial			port;
	u8				data_id;
	u8				port_num;
	u8				disabled;
	u8				configured;

	struct gser_descs		fs;
	struct gser_descs		hs;
	u8				online;
	enum transport_type		transport;

#ifdef CONFIG_MODEM_SUPPORT
	u8				pending;
	spinlock_t			lock;
	struct usb_ep			*notify;
	struct usb_endpoint_descriptor	*notify_desc;
	struct usb_request		*notify_req;

	struct usb_cdc_line_coding	port_line_coding;

	/* SetControlLineState request */
	u16				port_handshake_bits;
#define ACM_CTRL_RTS	(1 << 1)	/* unused with full duplex */
#define ACM_CTRL_DTR	(1 << 0)	/* host is ready for data r/w */

	/* SerialState notification */
	u16				serial_state;
#define ACM_CTRL_OVERRUN	(1 << 6)
#define ACM_CTRL_PARITY		(1 << 5)
#define ACM_CTRL_FRAMING	(1 << 4)
#define ACM_CTRL_RI		(1 << 3)
#define ACM_CTRL_BRK		(1 << 2)
#define ACM_CTRL_DSR		(1 << 1)
#define ACM_CTRL_DCD		(1 << 0)
#endif
	/* add callback for function enable/disable */
	void (*state_chg_notify)(int, int);
};
static struct usb_function *modem_function;
static struct usb_function *modem_mdm_function;
static struct usb_function *serial_function;

static unsigned int no_tty_ports;
static unsigned int no_sdio_ports;
static unsigned int no_smd_ports;
static unsigned int nr_ports;

static struct port_info {
	enum transport_type	transport;
	enum fserial_func_type func_type;
	unsigned		port_num;
	unsigned		client_port_num;
} gserial_ports[GSERIAL_NO_PORTS];

static inline struct f_gser *func_to_gser(struct usb_function *f)
{
	return container_of(f, struct f_gser, port.func);
}

#ifdef CONFIG_MODEM_SUPPORT
static inline struct f_gser *port_to_gser(struct gserial *p)
{
	return container_of(p, struct f_gser, port);
}
#define GS_LOG2_NOTIFY_INTERVAL		5	/* 1 << 5 == 32 msec */
#define GS_NOTIFY_MAXPACKET		10	/* notification + 2 bytes */
#endif
/*-------------------------------------------------------------------------*/

/* interface descriptor: */

static struct usb_interface_descriptor gser_interface_desc __initdata = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
#ifdef CONFIG_MODEM_SUPPORT
	.bNumEndpoints =	3,
#else
	.bNumEndpoints =	2,
#endif
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceProtocol =	USB_CLASS_VENDOR_SPEC,
	.iInterface = 0
};
#ifdef CONFIG_MODEM_SUPPORT
static struct usb_cdc_header_desc gser_header_desc  = {
	.bLength =		sizeof(gser_header_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_HEADER_TYPE,
	.bcdCDC =		__constant_cpu_to_le16(0x0110),
};

static struct usb_cdc_call_mgmt_descriptor
gser_call_mgmt_descriptor  = {
	.bLength =		sizeof(gser_call_mgmt_descriptor),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_CALL_MANAGEMENT_TYPE,
	.bmCapabilities =	0,
	/* .bDataInterface = DYNAMIC */
};

static struct usb_cdc_acm_descriptor gser_descriptor  = {
	.bLength =		sizeof(gser_descriptor),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_ACM_TYPE,
	.bmCapabilities =	USB_CDC_CAP_LINE,
};

static struct usb_cdc_union_desc gser_union_desc  = {
	.bLength =		sizeof(gser_union_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
	/* .bMasterInterface0 =	DYNAMIC */
	/* .bSlaveInterface0 =	DYNAMIC */
};
#endif
/* full speed support: */
#ifdef CONFIG_MODEM_SUPPORT
static struct usb_endpoint_descriptor gser_fs_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(GS_NOTIFY_MAXPACKET),
	.bInterval =		1 << GS_LOG2_NOTIFY_INTERVAL,
};
#endif

static struct usb_endpoint_descriptor gser_fs_in_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor gser_fs_out_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *gser_fs_function[] __initdata = {
	(struct usb_descriptor_header *) &gser_interface_desc,
#ifdef CONFIG_MODEM_SUPPORT
	(struct usb_descriptor_header *) &gser_header_desc,
/*
	(struct usb_descriptor_header *) &gser_call_mgmt_descriptor,
	(struct usb_descriptor_header *) &gser_descriptor,
	(struct usb_descriptor_header *) &gser_union_desc,
*/
	(struct usb_descriptor_header *) &gser_fs_notify_desc,
#endif
	(struct usb_descriptor_header *) &gser_fs_in_desc,
	(struct usb_descriptor_header *) &gser_fs_out_desc,
	NULL,
};

/* high speed support: */
#ifdef CONFIG_MODEM_SUPPORT
static struct usb_endpoint_descriptor gser_hs_notify_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(GS_NOTIFY_MAXPACKET),
	.bInterval =		GS_LOG2_NOTIFY_INTERVAL+4,
};
#endif

static struct usb_endpoint_descriptor gser_hs_in_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor gser_hs_out_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *gser_hs_function[] __initdata = {
	(struct usb_descriptor_header *) &gser_interface_desc,
#ifdef CONFIG_MODEM_SUPPORT
/*
	(struct usb_descriptor_header *) &gser_header_desc,
	(struct usb_descriptor_header *) &gser_call_mgmt_descriptor,
	(struct usb_descriptor_header *) &gser_descriptor,
	(struct usb_descriptor_header *) &gser_union_desc,
*/
	(struct usb_descriptor_header *) &gser_hs_notify_desc,
#endif
	(struct usb_descriptor_header *) &gser_hs_in_desc,
	(struct usb_descriptor_header *) &gser_hs_out_desc,
	NULL,
};

/* string descriptors: */

static struct usb_string modem_string_defs[] = {
	[0].s = "HTC Modem",
	{  } /* end of list */
};

static struct usb_gadget_strings modem_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		modem_string_defs,
};

static struct usb_gadget_strings *modem_strings[] = {
	&modem_string_table,
	NULL,
};

static struct usb_string modem_mdm_string_defs[] = {
	[0].s = "HTC 9k Modem",
	{  } /* end of list */
};

static struct usb_gadget_strings modem_mdm_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		modem_mdm_string_defs,
};

static struct usb_gadget_strings *modem_mdm_strings[] = {
	&modem_mdm_string_table,
	NULL,
};

static struct usb_string serial_string_defs[] = {
	[0].s = "HTC Serial",
	{  } /* end of list */
};

static struct usb_gadget_strings serial_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		serial_string_defs,
};

static struct usb_gadget_strings *serial_strings[] = {
	&serial_string_table,
	NULL,
};

static char *transport_to_str(enum transport_type t)
{
	switch (t) {
	case USB_GADGET_FSERIAL_TRANSPORT_TTY:
		return "TTY";
	case USB_GADGET_FSERIAL_TRANSPORT_SDIO:
		return "SDIO";
	case USB_GADGET_FSERIAL_TRANSPORT_SMD:
		return "SMD";
	}

	return "NONE";
}

static int gport_setup(struct usb_configuration *c)
{
	int ret = 0;

	pr_info("%s: no_tty_ports:%u no_sdio_ports: %u nr_ports:%u\n",
			__func__, no_tty_ports, no_sdio_ports, nr_ports);

	if (no_tty_ports)
		ret = gserial_setup(c->cdev->gadget, no_tty_ports);
	if (no_sdio_ports)
		ret = gsdio_setup(c->cdev->gadget, no_sdio_ports);
	if (no_smd_ports)
		ret = gsmd_setup(c->cdev->gadget, no_smd_ports);

	return ret;
}

static int gport_connect(struct f_gser *gser)
{
	unsigned port_num;

	pr_info("%s: transport:%s f_gser:%p gserial:%p port_num:%d\n",
			__func__, transport_to_str(gser->transport),
			gser, &gser->port, gser->port_num);

	port_num = gserial_ports[gser->port_num].client_port_num;

	switch (gser->transport) {
	case USB_GADGET_FSERIAL_TRANSPORT_TTY:
		gserial_connect(&gser->port, port_num);
		break;
	case USB_GADGET_FSERIAL_TRANSPORT_SDIO:
		gsdio_connect(&gser->port, port_num);
		break;
	case USB_GADGET_FSERIAL_TRANSPORT_SMD:
		gsmd_connect(&gser->port, port_num);
		break;
	default:
		pr_err("%s: Un-supported transport: %s\n", __func__,
				transport_to_str(gser->transport));
		return -ENODEV;
	}

	return 0;
}

static int gport_disconnect(struct f_gser *gser)
{
	unsigned port_num;

	pr_info("%s: transport:%s f_gser:%p gserial:%p port_num:%d\n",
			__func__, transport_to_str(gser->transport),
			gser, &gser->port, gser->port_num);

	port_num = gserial_ports[gser->port_num].client_port_num;

	switch (gser->transport) {
	case USB_GADGET_FSERIAL_TRANSPORT_TTY:
		gserial_disconnect(&gser->port);
		break;
	case USB_GADGET_FSERIAL_TRANSPORT_SDIO:
		gsdio_disconnect(&gser->port, port_num);
		break;
	case USB_GADGET_FSERIAL_TRANSPORT_SMD:
		gsmd_disconnect(&gser->port, port_num);
		break;
	default:
		pr_err("%s: Un-supported transport:%s\n", __func__,
				transport_to_str(gser->transport));
		return -ENODEV;
	}

	return 0;
}

#ifdef CONFIG_MODEM_SUPPORT
static void gser_complete_set_line_coding(struct usb_ep *ep,
		struct usb_request *req)
{
	struct f_gser            *gser = ep->driver_data;
	struct usb_composite_dev *cdev = gser->port.func.config->cdev;

	if (req->status != 0) {
		DBG(cdev, "gser ttyGS%d completion, err %d\n",
				gser->port_num, req->status);
		return;
	}

	/* normal completion */
	if (req->actual != sizeof(gser->port_line_coding)) {
		DBG(cdev, "gser ttyGS%d short resp, len %d\n",
				gser->port_num, req->actual);
		usb_ep_set_halt(ep);
	} else {
		struct usb_cdc_line_coding	*value = req->buf;
		gser->port_line_coding = *value;
	}
}
/*-------------------------------------------------------------------------*/
static int
gser_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct f_gser            *gser = func_to_gser(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	 *req = cdev->req;
	int			 value = -EOPNOTSUPP;
	u16			 w_index = le16_to_cpu(ctrl->wIndex);
	u16			 w_value = le16_to_cpu(ctrl->wValue);
	u16			 w_length = le16_to_cpu(ctrl->wLength);

	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {

	/* SET_LINE_CODING ... just read and save what the host sends */
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_SET_LINE_CODING:
		if (w_length != sizeof(struct usb_cdc_line_coding)
				|| w_index != gser->data_id)
			goto invalid;
		value = w_length;
		cdev->gadget->ep0->driver_data = gser;
		req->complete = gser_complete_set_line_coding;
		break;

	/* GET_LINE_CODING ... return what host sent, or initial value */
	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_GET_LINE_CODING:
		if (w_index != gser->data_id)
			goto invalid;

		value = min_t(unsigned, w_length,
				sizeof(struct usb_cdc_line_coding));
		memcpy(req->buf, &gser->port_line_coding, value);
		break;

	/* SET_CONTROL_LINE_STATE ... save what the host sent */
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		if (w_index != gser->data_id)
			goto invalid;

		value = 0;
		gser->port_handshake_bits = w_value;
		if (gser->port.notify_modem) {
			unsigned port_num =
				gserial_ports[gser->port_num].client_port_num;

			gser->port.notify_modem(&gser->port,
					port_num, w_value);
		}
		break;

	default:
invalid:
		ERROR(cdev, "invalid control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		DBG(cdev, "gser ttyGS%d req%02x.%02x v%04x i%04x l%d\n",
			gser->port_num, ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->zero = 0;
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			ERROR(cdev, "gser response on ttyGS%d, err %d\n",
					gser->port_num, value);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}
#endif
static int gser_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_gser		*gser = func_to_gser(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_interface_descriptor *desc;

	/* we know alt == 0, so this is an activation or a reset */
	if (cdev->gadget->speed == USB_SPEED_HIGH)
		desc = (struct usb_interface_descriptor *) *(f->hs_descriptors);
	else
		desc = (struct usb_interface_descriptor *) *(f->descriptors);
	gser->data_id = desc->bInterfaceNumber;

#ifdef CONFIG_MODEM_SUPPORT
#if 0
	if (gser->notify->driver_data) {
		DBG(cdev, "reset generic ttyGS%d\n", gser->port_num);
		usb_ep_disable(gser->notify);
	}
#endif
#ifdef DISABLE_SERIAL_NOTIFY
	if (strncmp(gser->port.func.name, "serial", 6)) {
#endif
		gser->notify_desc = ep_choose(cdev->gadget,
			gser->hs.notify,
			gser->fs.notify);
		usb_ep_enable(gser->notify, gser->notify_desc);
		gser->notify->driver_data = gser;
#ifdef DISABLE_SERIAL_NOTIFY
	}
#endif
#endif
#if 0
	if (gser->port.in->driver_data) {
		DBG(cdev, "reset generic ttyGS%d\n", gser->port_num);
		gport_disconnect(gser);
	} else {
		DBG(cdev, "activate generic ttyGS%d\n", gser->port_num);
	}
#endif
	gser->port.in_desc = ep_choose(cdev->gadget,
			gser->hs.in, gser->fs.in);
	gser->port.out_desc = ep_choose(cdev->gadget,
			gser->hs.out, gser->fs.out);
	gport_connect(gser);
	gser->online = 1;
	return 0;
}

static void gser_disable(struct usb_function *f)
{
	struct f_gser	*gser = func_to_gser(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	DBG(cdev, "generic ttyGS%d deactivated\n", gser->port_num);

	gport_disconnect(gser);

#if 0
	/* disable endpoints, aborting down any active I/O */
	usb_ep_fifo_flush(gser->port.out);
	usb_ep_disable(gser->port.out);
	gser->port.out->driver_data = NULL;

	usb_ep_fifo_flush(gser->port.in);
	usb_ep_disable(gser->port.in);
	gser->port.in->driver_data = NULL;
#endif
#ifdef CONFIG_MODEM_SUPPORT
#ifdef DISABLE_SERIAL_NOTIFY
	if (strncmp(gser->port.func.name, "serial", 6)) {
#endif
		usb_ep_fifo_flush(gser->notify);
		usb_ep_disable(gser->notify);
#ifdef DISABLE_SERIAL_NOTIFY
	}
#endif
#endif
	gser->online = 0;
}
#ifdef CONFIG_MODEM_SUPPORT
static int gser_notify(struct f_gser *gser, u8 type, u16 value,
		void *data, unsigned length)
{
	struct usb_ep			*ep = gser->notify;
	struct usb_request		*req;
	struct usb_cdc_notification	*notify;
	const unsigned			len = sizeof(*notify) + length;
	void				*buf;
	int				status;
	struct usb_composite_dev *cdev = gser->port.func.config->cdev;

	req = gser->notify_req;
	gser->notify_req = NULL;
	gser->pending = false;

	req->length = len;
	notify = req->buf;
	buf = notify + 1;

	notify->bmRequestType = USB_DIR_IN | USB_TYPE_CLASS
			| USB_RECIP_INTERFACE;
	notify->bNotificationType = type;
	notify->wValue = cpu_to_le16(value);
	notify->wIndex = cpu_to_le16(gser->data_id);
	notify->wLength = cpu_to_le16(length);
	memcpy(buf, data, length);

	status = usb_ep_queue(ep, req, GFP_ATOMIC);
	if (status < 0) {
		ERROR(cdev, "gser ttyGS%d can't notify serial state, %d\n",
				gser->port_num, status);
		gser->notify_req = req;
	}

	return status;
}

static int gser_notify_serial_state(struct f_gser *gser)
{
	int			 status;
	unsigned long flags;
	struct usb_composite_dev *cdev = gser->port.func.config->cdev;
	if (gser->disabled)
		return 0;

	spin_lock_irqsave(&gser->lock, flags);
	if (gser->notify_req) {
		DBG(cdev, "gser ttyGS%d serial state %04x\n",
				gser->port_num, gser->serial_state);
		status = gser_notify(gser, USB_CDC_NOTIFY_SERIAL_STATE,
				0, &gser->serial_state,
					sizeof(gser->serial_state));
	} else {
		gser->pending = true;
		status = 0;
	}
	spin_unlock_irqrestore(&gser->lock, flags);
	return status;
}

static void gser_notify_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_gser *gser = req->context;
	u8	      doit = false;
	unsigned long flags;

	/* on this call path we do NOT hold the port spinlock,
	 * which is why ACM needs its own spinlock
	 */
	spin_lock_irqsave(&gser->lock, flags);
	if (req->status != -ESHUTDOWN)
		doit = gser->pending;
	gser->notify_req = req;
	spin_unlock_irqrestore(&gser->lock, flags);

	if (doit && gser->online)
		gser_notify_serial_state(gser);
}
static void gser_connect(struct gserial *port)
{
	struct f_gser *gser = port_to_gser(port);

	gser->serial_state |= ACM_CTRL_DSR | ACM_CTRL_DCD;
	gser_notify_serial_state(gser);
}

unsigned int gser_get_dtr(struct gserial *port)
{
	struct f_gser *gser = port_to_gser(port);

	if (gser->port_handshake_bits & ACM_CTRL_DTR)
		return 1;
	else
		return 0;
}

unsigned int gser_get_rts(struct gserial *port)
{
	struct f_gser *gser = port_to_gser(port);

	if (gser->port_handshake_bits & ACM_CTRL_RTS)
		return 1;
	else
		return 0;
}

unsigned int gser_send_carrier_detect(struct gserial *port, unsigned int yes)
{
	struct f_gser *gser = port_to_gser(port);
	u16			state;

	state = gser->serial_state;
	state &= ~ACM_CTRL_DCD;
	if (yes)
		state |= ACM_CTRL_DCD;

	gser->serial_state = state;
	return gser_notify_serial_state(gser);

}

unsigned int gser_send_ring_indicator(struct gserial *port, unsigned int yes)
{
	struct f_gser *gser = port_to_gser(port);
	u16			state;

	state = gser->serial_state;
	state &= ~ACM_CTRL_RI;
	if (yes)
		state |= ACM_CTRL_RI;

	gser->serial_state = state;
	return gser_notify_serial_state(gser);

}
static void gser_disconnect(struct gserial *port)
{
	struct f_gser *gser = port_to_gser(port);

	gser->serial_state &= ~(ACM_CTRL_DSR | ACM_CTRL_DCD);
	gser_notify_serial_state(gser);
}

static int gser_send_break(struct gserial *port, int duration)
{
	struct f_gser *gser = port_to_gser(port);
	u16			state;

	state = gser->serial_state;
	state &= ~ACM_CTRL_BRK;
	if (duration)
		state |= ACM_CTRL_BRK;

	gser->serial_state = state;
	return gser_notify_serial_state(gser);
}

static int gser_send_modem_ctrl_bits(struct gserial *port, int ctrl_bits)
{
	struct f_gser *gser = port_to_gser(port);

	gser->serial_state = ctrl_bits;

	return gser_notify_serial_state(gser);
}
#endif
/*-------------------------------------------------------------------------*/

/* serial function driver setup/binding */

static int __init
gser_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_gser		*gser = func_to_gser(f);
	int			status;
	struct usb_ep		 *ep;
	struct usb_gadget_strings	*s;

	/* allocate instance-specific interface IDs */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	gser->data_id = status;
	gser_interface_desc.bInterfaceNumber = status;
	if (f->strings) {
		s = *(f->strings);
		gser_interface_desc.iInterface = s->strings[0].id;
	}

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &gser_fs_in_desc);
	if (!ep)
		goto fail;
	gser->port.in = ep;
	ep->driver_data = gser;	/* claim */

	ep = usb_ep_autoconfig(cdev->gadget, &gser_fs_out_desc);
	if (!ep)
		goto fail;
	gser->port.out = ep;
	ep->driver_data = gser;	/* claim */

#ifdef CONFIG_MODEM_SUPPORT
#ifdef DISABLE_SERIAL_NOTIFY
	if (strncmp(gser->port.func.name, "serial", 6)) {
#endif
		printk("%s:%s allocate ep for modem notify function\n", __func__, gser->port.func.name);
		ep = usb_ep_autoconfig(cdev->gadget, &gser_fs_notify_desc);
		if (!ep)
			goto fail;
		gser->notify = ep;
		ep->driver_data = gser;	/* claim */
		/* allocate notification */
		gser->notify_req = gs_alloc_req(ep,
				sizeof(struct usb_cdc_notification) + 2,
				GFP_KERNEL);
		if (!gser->notify_req)
			goto fail;

		gser->notify_req->complete = gser_notify_complete;
		gser->notify_req->context = gser;
#ifdef DISABLE_SERIAL_NOTIFY
	}
#endif
#endif
	/* copy descriptors, and track endpoint copies */
	f->descriptors = usb_copy_descriptors(gser_fs_function);

	gser->fs.in = usb_find_endpoint(gser_fs_function,
			f->descriptors, &gser_fs_in_desc);
	gser->fs.out = usb_find_endpoint(gser_fs_function,
			f->descriptors, &gser_fs_out_desc);
#ifdef CONFIG_MODEM_SUPPORT
#ifdef DISABLE_SERIAL_NOTIFY
	if (strncmp(gser->port.func.name, "serial", 6)) {
#endif
		gser->fs.notify = usb_find_endpoint(gser_fs_function,
			f->descriptors, &gser_fs_notify_desc);
#ifdef DISABLE_SERIAL_NOTIFY
	}
#endif
#endif


	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		gser_hs_in_desc.bEndpointAddress =
				gser_fs_in_desc.bEndpointAddress;
		gser_hs_out_desc.bEndpointAddress =
				gser_fs_out_desc.bEndpointAddress;
#ifdef CONFIG_MODEM_SUPPORT
#ifdef DISABLE_SERIAL_NOTIFY
	if (strncmp(gser->port.func.name, "serial", 6)) {
#endif
		gser_hs_notify_desc.bEndpointAddress =
				gser_fs_notify_desc.bEndpointAddress;
#ifdef DISABLE_SERIAL_NOTIFY
	}
#endif
#endif

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(gser_hs_function);

		gser->hs.in = usb_find_endpoint(gser_hs_function,
				f->hs_descriptors, &gser_hs_in_desc);
		gser->hs.out = usb_find_endpoint(gser_hs_function,
				f->hs_descriptors, &gser_hs_out_desc);
#ifdef CONFIG_MODEM_SUPPORT
#ifdef DISABLE_SERIAL_NOTIFY
	if (strncmp(gser->port.func.name, "serial", 6)) {
#endif
		gser->hs.notify = usb_find_endpoint(gser_hs_function,
				f->hs_descriptors, &gser_hs_notify_desc);
#ifdef DISABLE_SERIAL_NOTIFY
	}
#endif
#endif
	}

	DBG(cdev, "generic ttyGS%d: %s speed IN/%s OUT/%s\n",
			gser->port_num,
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			gser->port.in->name, gser->port.out->name);
	return 0;

fail:
#ifdef CONFIG_MODEM_SUPPORT
#ifdef DISABLE_SERIAL_NOTIFY
	if (strncmp(gser->port.func.name, "serial", 6)) {
#endif
		if (gser->notify_req)
			gs_free_req(gser->notify, gser->notify_req);

	/* we might as well release our claims on endpoints */
		if (gser->notify)
			gser->notify->driver_data = NULL;
#ifdef DISABLE_SERIAL_NOTIFY
	}
#endif
#endif
	/* we might as well release our claims on endpoints */
	if (gser->port.out)
		gser->port.out->driver_data = NULL;
	if (gser->port.in)
		gser->port.in->driver_data = NULL;

	ERROR(cdev, "%s: can't bind, err %d\n", f->name, status);

	return status;
}

static void
gser_unbind(struct usb_configuration *c, struct usb_function *f)
{
#ifdef CONFIG_MODEM_SUPPORT
	struct f_gser *gser = func_to_gser(f);
#endif
	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);
#ifdef CONFIG_MODEM_SUPPORT
#ifdef DISABLE_SERIAL_NOTIFY
	if (strncmp(gser->port.func.name, "serial", 6)) {
#endif
		gs_free_req(gser->notify, gser->notify_req);
#ifdef DISABLE_SERIAL_NOTIFY
	}
#endif
#endif
	kfree(func_to_gser(f));
}

static void
gser_release(struct usb_configuration *c, struct usb_function *f)
{
	struct f_gser *gser = func_to_gser(f);
	if (gser->port.in)
		gser->port.in->driver_data = NULL;
	if (gser->port.in)
		gser->port.out->driver_data = NULL;

	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);
#ifdef CONFIG_MODEM_SUPPORT
#ifdef DISABLE_SERIAL_NOTIFY
	if (strncmp(gser->port.func.name, "serial", 6)) {
#endif
		if (gser->notify) {
			gser->notify->driver_data = NULL;
			gs_free_req(gser->notify, gser->notify_req);
		}
#ifdef DISABLE_SERIAL_NOTIFY
	}
#endif
#endif
	usb_interface_id_remove(c, 1);
}

/**
 * gser_bind_config - add a generic serial function to a configuration
 * @c: the configuration to support the serial instance
 * @port_num: /dev/ttyGS* port this interface will use
 * Context: single threaded during gadget setup
 *
 * Returns zero on success, else negative errno.
 *
 * Caller must have called @gserial_setup() with enough ports to
 * handle all the ones it binds.  Caller is also responsible
 * for calling @gserial_cleanup() before module unload.
 */
int __init gser_bind_config(struct usb_configuration *c, u8 port_num)
{
	struct f_gser	*gser;
	int		status;
	struct port_info *p = &gserial_ports[port_num];

	if (p->func_type == USB_FSER_FUNC_NONE) {
		pr_info("%s: non function port : %d\n", __func__, port_num);
		return 0;
	}

	/* REVISIT might want instance-specific strings to help
	 * distinguish instances ...
	 */

	/* maybe allocate device-global string ID */
	if (modem_string_defs[0].id == 0 &&
			p->func_type == USB_FSER_FUNC_MODEM) {
		status = usb_string_id(c->cdev);
		if (status < 0) {
			printk(KERN_ERR "%s: return %d\n", __func__, status);
			return status;
		}
		modem_string_defs[0].id = status;
	}

	if (modem_mdm_string_defs[0].id == 0 &&
			p->func_type == USB_FSER_FUNC_MODEM_MDM) {
		status = usb_string_id(c->cdev);
		if (status < 0) {
			printk(KERN_ERR "%s: return %d\n", __func__, status);
			return status;
		}
		modem_mdm_string_defs[0].id = status;
	}

	if (serial_string_defs[0].id == 0 &&
			p->func_type == USB_FSER_FUNC_SERIAL) {
		status = usb_string_id(c->cdev);
		if (status < 0) {
			printk(KERN_ERR "%s: return %d\n", __func__, status);
			return status;
		}
		serial_string_defs[0].id = status;
	}
	/* allocate and initialize one new instance */
	gser = kzalloc(sizeof *gser, GFP_KERNEL);
	if (!gser)
		return -ENOMEM;

#ifdef CONFIG_MODEM_SUPPORT
	spin_lock_init(&gser->lock);
#endif

	gser->port_num = port_num;
	gser->transport = p->transport;

	switch (p->func_type) {
	case USB_FSER_FUNC_MODEM:
		gser->port.func.name = "modem";
		gser->port.func.strings = modem_strings;
		modem_function = &gser->port.func;
		break;
	case USB_FSER_FUNC_MODEM_MDM:
		gser->port.func.name = "modem_mdm";
		gser->port.func.strings = modem_mdm_strings;
		modem_mdm_function = &gser->port.func;
		break;
	case USB_FSER_FUNC_SERIAL:
		gser->port.func.name = "serial";
		gser->port.func.strings = serial_strings;
		serial_function = &gser->port.func;
		break;
	case USB_FSER_FUNC_NONE:
	default	:
		break;
	}

	gser->port.func.bind = gser_bind;
	gser->port.func.unbind = gser_unbind;
	gser->port.func.set_alt = gser_set_alt;
	gser->port.func.disable = gser_disable;
	gser->port.func.release = gser_release;
#ifdef CONFIG_MODEM_SUPPORT
#ifdef DISABLE_SERIAL_NOTIFY
	if (strncmp(gser->port.func.name, "serial", 6)) {
#endif
		gser->port.func.setup = gser_setup;
		gser->port.connect = gser_connect;
		gser->port.get_dtr = gser_get_dtr;
		gser->port.get_rts = gser_get_rts;
		gser->port.send_carrier_detect = gser_send_carrier_detect;
		gser->port.send_ring_indicator = gser_send_ring_indicator;
		gser->port.send_modem_ctrl_bits = gser_send_modem_ctrl_bits;
		gser->port.disconnect = gser_disconnect;
		gser->port.send_break = gser_send_break;
#ifdef DISABLE_SERIAL_NOTIFY
	}
#endif
#endif
	gser->port.func.dynamic = 1;

#if defined(CONFIG_ARCH_MSM8X60_LTE)
	if (strncmp(gser->port.func.name, "modem", 5) == 0) {
		gser->port.func.hidden = !diag_init_enabled_state;
		gser->disabled = !diag_init_enabled_state;

		if (p->transport == USB_GADGET_FSERIAL_TRANSPORT_SDIO) {
			/* Add callback function for modem_over_sdio
			 */
			gser->state_chg_notify = gsdio_state_chg_notify;
			gser->state_chg_notify(p->client_port_num,
				diag_init_enabled_state);
		}

	} else {
		gser->port.func.hidden = 1;
		gser->disabled = 1;
	}
#else
	gser->port.func.hidden = 1;
	gser->disabled = 1;
#endif

	status = usb_add_function(c, &gser->port.func);
	if (status)
		kfree(gser);
	return status;
}

static int modem_set_enabled(const char *val, struct kernel_param *kp)
{
	struct f_gser *gser;
	int enabled = simple_strtol(val, NULL, 0);
	unsigned port_num;

	printk(KERN_INFO "%s: %d\n", __func__, enabled);
	gser = func_to_gser(modem_function);
	if (!gser)
		return 0;
	if (enabled) {
		wake_lock(&vbus_idle_wake_lock);
		if (!is_perf_lock_active(&usb_perf_lock))
			perf_lock(&usb_perf_lock);
	} else {
		wake_unlock(&vbus_idle_wake_lock);
		if (is_perf_lock_active(&usb_perf_lock))
			perf_unlock(&usb_perf_lock);
	}
	gser->disabled = !enabled;

	port_num = gserial_ports[gser->port_num].client_port_num;


	if (gser->state_chg_notify) {
		printk(KERN_INFO "%s: state_chg_notify\n", __func__);
		gser->state_chg_notify(port_num, enabled);
	}

	android_enable_function(modem_function, enabled);
	return 0;
}

static int modem_get_enabled(char *buffer, struct kernel_param *kp)
{
	buffer[0] = '0' + !modem_function->hidden;
	printk(KERN_INFO "%s: %d\n", __func__, buffer[0] - '0');
	return 1;
}
module_param_call(modem_enabled, modem_set_enabled, modem_get_enabled, NULL, 0664);

static int serial_set_enabled(const char *val, struct kernel_param *kp)
{
	struct f_gser *gser;
	int enabled = simple_strtol(val, NULL, 0);
	printk(KERN_INFO "%s: %d\n", __func__, enabled);
	gser = func_to_gser(serial_function);
	if (!gser)
		return 0;
	gser->disabled = !enabled;
	android_enable_function(serial_function, enabled);
	return 0;
}

static int serial_get_enabled(char *buffer, struct kernel_param *kp)
{
	buffer[0] = '0' + !serial_function->hidden;
	/*printk(KERN_INFO "%s: %d\n", __func__, buffer[0] - '0');*/
	return 1;
}
module_param_call(serial_enabled, serial_set_enabled, serial_get_enabled, NULL, 0664);


static int serial_get_name(char *buffer, struct kernel_param *kp)
{
	int i;
	for (i=0;i<nr_ports;i++) {
		if (gserial_ports[i].func_type == USB_FSER_FUNC_SERIAL)
			return sprintf(buffer, "%s%d", PREFIX,
					gserial_ports[i].client_port_num);
	}

	pr_info("%s: use default serial name", __func__);
	return sprintf(buffer, "%s%d", PREFIX, 2);
}
module_param_call(serial_name, NULL, serial_get_name, NULL, 0444);

static int serial_bind_config(struct usb_configuration *c)
{
	int ret = 0;
	int i;

	printk(KERN_INFO "serial_bind_config\n");

	for (i = 0; i < nr_ports; i++) {
		ret = gser_bind_config(c, i);
		if (ret)
			return ret;
	}

	/* See if composite driver can allocate
	 * serial ports. But for now allocate
	 * two ports for modem and nmea.
	 */
	if (ret == 0) {
		ret = gport_setup(c);
	}

	return ret;
}

static struct android_usb_function android_serial_function = {
	.name = "serial",
	.bind_config = serial_bind_config,
};

#if defined(CONFIG_USB_F_SERIAL_SDIO) || defined(CONFIG_USB_F_SERIAL_SMD)
static int fserial_remove(struct platform_device *dev)
{
	gserial_cleanup();

	return 0;
}

static int __init fserial_probe(struct platform_device *pdev)
{
	struct usb_gadget_fserial_platform_data	*pdata =
					pdev->dev.platform_data;
	int i;

	printk(KERN_INFO "%s: probe\n", __func__);
	if (!pdata)
		goto probe_android_register;


	/* clean data first */
	no_tty_ports = 0;
	no_sdio_ports = 0;
	no_smd_ports = 0;
	memset(gserial_ports, 0, sizeof(gserial_ports));
	nr_ports = pdata->no_ports;
	for (i = 0; i < nr_ports; i++) {
		gserial_ports[i].transport = pdata->transport[i];
		gserial_ports[i].port_num = i;
		gserial_ports[i].func_type = pdata->func_type[i];

		switch (gserial_ports[i].transport) {
		case USB_GADGET_FSERIAL_TRANSPORT_TTY:
			gserial_ports[i].client_port_num = no_tty_ports;
			no_tty_ports++;
			break;
		case USB_GADGET_FSERIAL_TRANSPORT_SDIO:
			gserial_ports[i].client_port_num = no_sdio_ports;
			no_sdio_ports++;
			break;
		case USB_GADGET_FSERIAL_TRANSPORT_SMD:
			gserial_ports[i].client_port_num = no_smd_ports;
			no_smd_ports++;
			break;
		default:
			pr_err("%s: Un-supported transport transport: %u\n",
					__func__, gserial_ports[i].transport);
			return -ENODEV;
		}
	}


	pr_info("%s:gport:tty_ports:%u sdio_ports:%u "
			"smd_ports:%u nr_ports:%u\n",
			__func__, no_tty_ports, no_sdio_ports,
			no_smd_ports, nr_ports);

probe_android_register:
	wake_lock_init(&vbus_idle_wake_lock, WAKE_LOCK_IDLE, "modem_idle_lock");
	perf_lock_init(&usb_perf_lock, PERF_LOCK_HIGHEST, "usb");

	android_register_function(&android_serial_function);
	return 0;
}

static struct platform_driver usb_fserial = {
	.remove		= fserial_remove,
	.driver = {
		.name = "usb_fserial",
		.owner = THIS_MODULE,
	},
};
#endif

static void __init fserial_set_default_portinfo(void)
{
	int i;

	nr_ports = FSERIAL_DEAFAULT_TTY_NO;
	for (i = 0; i < nr_ports; i++) {
		gserial_ports[i].transport = USB_GADGET_FSERIAL_TRANSPORT_TTY;
		gserial_ports[i].port_num = i;
		gserial_ports[i].client_port_num = no_tty_ports;
		no_tty_ports++;
	}

	/* original design */
	gserial_ports[0].func_type = USB_FSER_FUNC_MODEM;
	gserial_ports[1].func_type = USB_FSER_FUNC_NONE;
	gserial_ports[2].func_type = USB_FSER_FUNC_SERIAL;
}

static int __init init(void)
{
	printk(KERN_INFO "serial init\n");

	fserial_set_default_portinfo();

#if defined(CONFIG_USB_F_SERIAL_SDIO) || defined(CONFIG_USB_F_SERIAL_SMD)
	return platform_driver_probe(&usb_fserial, fserial_probe);
#else
	wake_lock_init(&vbus_idle_wake_lock, WAKE_LOCK_IDLE, "modem_idle_lock");
	perf_lock_init(&usb_perf_lock, PERF_LOCK_HIGHEST, "usb");

	android_register_function(&android_serial_function);
	return 0;
#endif

}
module_init(init);
