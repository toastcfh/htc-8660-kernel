/*
 *   TI BT Char Driver for Texas Instrument's Connectivity Chip.
 *   Copyright (C) 2009 Texas Instruments
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef HCIIFDRV_H
#define HCIIFDRV_H

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/tty.h>
#include <linux/sched.h>

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/skbuff.h>
#include <linux/interrupt.h>
#include <linux/list.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <net/bluetooth/hci.h>
#include <linux/ti_wilink_st.h>


/*#define VERBOSE*/

#undef VERBOSE
#undef DEBUG

/* Debug macros*/
#define HCIIFDRV_ERR(fmt, arg...)  printk(KERN_ERR "(HCI_IF: ERR):"fmt"\n" , ## arg)
#if defined(DEBUG)		/* limited debug messages */
#define HCIIFDRV_DBG(fmt, arg...)  printk(KERN_INFO "(HCI_IF):"fmt"\n" , ## arg)
#define HCIIFDRV_VER(fmt, arg...)
#elif defined(VERBOSE)		/* very verbose */
#define HCIIFDRV_DBG(fmt, arg...)  printk(KERN_INFO "(HCI_IF):"fmt"\n" , ## arg)
#define HCIIFDRV_VER(fmt, arg...)  printk(KERN_INFO "(HCI_IF):"fmt"\n" , ## arg)
#else /* Error msgs only */
#define HCIIFDRV_DBG(fmt, arg...)
#define HCIIFDRV_VER(fmt, arg...)
#endif

/* List of error codes returned by the TI BT driver */
enum {
	HCIIF_ERR_FAILURE = -1,	/* check struct             */
	HCIIF_SUCCESS,
	HCIIF_ERR_PENDING = -5,	/* to call reg_complete_cb  */
	HCIIF_ERR_ALREADY,	    /* already registered       */
	HCIIF_ERR_INPROGRESS,
	HCIIF_ERR_NOPROTO,	    /* protocol not supported   */
	HCIIF_ERR_CLASS = -15,
	HCIIF_ERR_CPY_TO_USR,
	HCIIF_ERR_CPY_FRM_USR,
	HCIIF_ERR_TIMEOUT,
	HCIIF_ERR_UNKNOWN,
};


#define DEVICE_NAME  "tihci"

/* Packet Type */
#define HCIIF_CHAN_CMD							0x1
#define HCIIF_CHAN_ACL							0x2
#define HCIIF_CHAN_SCO							0x3
#define HCIIF_CHAN_EVT							0x4
#define HCIIF_CHAN_FM							0x8

/* Header Size */
#define HCIIF_HDR_SIZE_ACL         				4 /* opcode 2, Len 2 */
#define HCIIF_HDR_SIZE_SCO         				3 /* opcode 2, Len 1 */
#define HCIIF_HDR_SIZE_EVT       				2 /* opcode 1, Len 1 */
#define HCIIF_HDR_SIZE_FM						1 /* Len 1           */
#define HCIIF_HDR_SIZE_DEF						3

/* Max Frame Size */
#define HCIIF_FRAME_SIZE_ACL         			HCI_MAX_FRAME_SIZE
#define HCIIF_FRAME_SIZE_SCO         			HCI_MAX_FRAME_SIZE
#define HCIIF_FRAME_SIZE_EVT       				HCI_MAX_FRAME_SIZE
#define HCIIF_FRAME_SIZE_FM					    255
#define HCIIF_FRAME_SIZE_DEF					255


/* Length Size */
#define HCIIF_LEN_SIZE_ACL         				2
#define HCIIF_LEN_SIZE_SCO         				1
#define HCIIF_LEN_SIZE_EVT	       				1
#define HCIIF_LEN_SIZE_FM						1
#define HCIIF_LEN_SIZE_DEF						2

/* Length Offset */
#define HCIIF_LEN_OFFSET_ACL         			2
#define HCIIF_LEN_OFFSET_SCO         			2
#define HCIIF_LEN_OFFSET_EVT       				1
#define HCIIF_LEN_OFFSET_FM					    0
#define HCIIF_LEN_OFFSET_DEF					1

/* Commands opcode */
#define HCIIF_CMD_HOST_NUM_OF_CMPLT_PKTS     	0x0c35

/* Events opcode */
#define HCIIF_OP_EVT_COMPLETED   				0x0e
#define HCIIF_OP_EVT_STATUS					    0x0f

/* Macros for Syncronising Open & Close operations */
#define HCIIF_ST_INPROGRESS 					0

/* Read time out defined to 10 seconds */
/* This value should be raised after debugging session */
#define HCIIFDRV_READ_TIMEOUT 					10000
/* Reg time out defined to 6 seconds */
#define HCIIFDRV_REG_TIMEOUT 					6000

/* HCI IF Drv max number of clients */
#define HCIIF_CMD_CLIENTS_NUM					8

/* Events Registration */
#define HCIIF_EVT_DEFUALT						0xff

/* HCI IF IOCTLs */
#define HCIIF_IOCTL_DEVUP						1

/* HCI IF maximum of registered events per client */
#define HCIIF_MAX_REG_EVENTS					4

struct  hciif_filter_t {
	unsigned long    chan_mask;
	unsigned char    evt_type[HCIIF_MAX_REG_EVENTS];
	/*up to 4 events can be registered per client (0 will mark EOF)
	 evtType[0] == 0xff makes a wildcard */
} ;


/* HCI IF driver Client Structure */
struct hciif_client {
	struct list_head			list;					/*  List is needed for sending back cmd complete */
	struct file				    *file;				    /*  Client's file pointer */
	wait_queue_head_t			hciif_client_q;		    /*  Used to signal to read upon recv packet */
	spinlock_t				    rx_lock;				/*  spin lock data for safe-gaurding operations on SKB */
	struct sk_buff_head		    rx_list;				/*  Rx data SKB queue */
	uint16_t					cmd_opcode;		        /*  Saving the opcode of the command that the client sent */
	struct  hciif_filter_t		filter;			        /*  Client's channels & events */
};


/* HCI IF driver operation structure */
struct hciif_handle {
	int						hciif_major;						        /*  BT major number */
	struct class			*hciif_class;						        /*  BT class during class_create */
	struct device			*hciif_dev;						            /*  BT dev during device_create */
	struct completion		hciif_reg_completed;				        /*  comepletion handler to synchronize hciif_chrdev_open */
	struct completion		hciif_oper_completed;				        /*  comepletion handler to synchronize Open/Close operation between clients */
	char					streg_cbdata;						        /*  ST registration callback  status */
	char					stoper_cbdata;						        /*  ST Open/Close operation callback status */
	unsigned long			state;								        /*  used locally,to maintain various BT driver status */
	int						hciif_cmdClientsCounter;			        /*  Number of Clients registered */
	struct hciif_client		*hciif_RawClients[ST_MAX_CHANNELS];	        /*  Registered Raw Clients */
	struct hciif_client		*hciif_CmdClients[HCIIF_CMD_CLIENTS_NUM];	/*  Registered Command Clients */
	struct hciif_client		hciif_WriteClientsList;				        /*  List of client that performed CMD write */
	spinlock_t				hciif_lock;							        /*  spin lock data for safe-gaurding operations on WriteClientsList */
	long                    (*st_write) (struct sk_buff *skb);
};



static int  hciif_init(void);
static void hciif_exit(void);
int         hciif_dev_up(struct file *file, struct hciif_filter_t *rcv_filter, struct hciif_client	**client);
int         hciif_dev_down(struct hciif_client *client);
long        hciif_send_frame(struct sk_buff *skb, struct hciif_client *client);

#endif /*HCIIFDRV_H */
