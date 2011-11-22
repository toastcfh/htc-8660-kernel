/*
 *   GPS Char Driver for Texas Instrument's Connectivity Chip.
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

#ifndef GPSDRV_H
#define GPSDRV_H

#undef VERBOSE
#undef DEBUG


/* Debug macros*/
#if defined(DEBUG)		/* limited debug messages */
#define GPSDRV_DBG(fmt, arg...)  printk(KERN_INFO "(gpsdrv):"fmt"\n" , ## arg)
#define GPSDRV_VER(fmt, arg...)
#elif defined(VERBOSE)		/* very verbose */
#define GPSDRV_DBG(fmt, arg...)  printk(KERN_INFO "(gpsdrv):"fmt"\n" , ## arg)
#define GPSDRV_VER(fmt, arg...)  printk(KERN_INFO "(gpsdrv):"fmt"\n" , ## arg)
#define GPSDRV_ERR(fmt, arg...)  printk(KERN_ERR "(gpsdrv):"fmt"\n" , ## arg)
#else /* Error msgs only */
#define GPSDRV_ERR(fmt, arg...)  printk(KERN_ERR "(gpsdrv):"fmt"\n" , ## arg)
#define GPSDRV_VER(fmt, arg...)
#define GPSDRV_DBG(fmt, arg...)
#endif

void gpsdrv_tsklet_write(unsigned long data);

/* List of error codes returned by the gps driver*/
enum {
	GPS_ERR_FAILURE = -1,	/* check struct */
	GPS_SUCCESS,
	GPS_ERR_PENDING = -5,	/* to call reg_complete_cb */
	GPS_ERR_ALREADY,	/* already registered */
	GPS_ERR_INPROGRESS,
	GPS_ERR_NOPROTO,	/* protocol not supported */
	GPS_ERR_CLASS = -15,
	GPS_ERR_CPY_TO_USR,
	GPS_ERR_CPY_FRM_USR,
	GPS_ERR_TIMEOUT,
	GPS_ERR_UNKNOWN,
};

/* Channel-9 details for GPS */
#define GPS_CH9_PKT_HDR_SIZE		4
#define GPS_CH9_OP_WRITE		0x1
#define GPS_CH9_OP_READ			0x2
#define GPS_CH9_OP_COMPLETED_EVT      	0x3

/* Macros for Syncronising GPS registration and other R/W/ICTL operations */
#define GPS_ST_REGISTERED		0
#define GPS_ST_RUNNING  		1

/* Read time out defined to 10 seconds */
#define GPSDRV_READ_TIMEOUT 		10000
/* Reg time out defined to 6 seconds */
#define GPSDRV_REG_TIMEOUT 		6000


struct gpsdrv_event_hdr {
	uint8_t opcode;
	uint16_t plen;
} __attribute__ ((packed));

/* BT driver operation structure */
struct gpsdrv_data {

	int gpsdrv_major;		/* GPS major number */
	struct class *gpsdrv_class;	/* GPS class during class_create */
	struct device *gpsdrv_dev;	/* GPS dev during device_create */

	struct completion gpsdrv_reg_completed;
		/*comepletion handler to synchronize gpsdrv_chrdev_open */
	unsigned char streg_cbdata;	/* ST registration callback  status */

	unsigned long state;
		/* used locally,to maintain various GPS driver status */
	unsigned char tx_count;	/* Count value which gives number of Tx GPS
				 * commands that can be sent to GPS chip */

	long (*st_write) (struct sk_buff *skb);
				/* write function pointer of ST driver */
	struct sk_buff_head rx_list;	/* Rx data SKB queue */
	struct sk_buff_head tx_list;	/* Tx data SKB queue */

	wait_queue_head_t gpsdrv_data_q;
			/* Used to syncronize read call and poll */

	spinlock_t lock;
		/* spin lock data for safe-gaurding operations on SKB */

};

long st_write(struct sk_buff *skb);

#endif /*GPS_H */
