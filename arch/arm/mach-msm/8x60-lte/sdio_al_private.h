/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * SDIO-Abstraction-Layer internal interface.
 */

#ifndef __SDIO_AL_PRIVATE__
#define __SDIO_AL_PRIVATE__

#include <linux/mmc/card.h>
#include <linux/platform_device.h>

#if defined(CONFIG_ARCH_MSM7X30_LTE)
#include <mach/7x30-lte/sdio_al.h>
#elif defined(CONFIG_ARCH_MSM8X60_LTE)
#include <mach/8x60-lte/sdio_al.h>
#endif

/** HTC: Modification History
	01: 2011-05-02 bert_lin - add sdio_memcpy debugging mechanism
	02: 2011-05-04 bert_lin - <potential fix>
		1. wakeup sdio device before sdio_memcpy of sdio_read
		2. don't sleep until sdio reader/writer finished.
	03: 2011-05-04 bert_lin - add tp statistic but not enable it.
	04: 2011-05-12 bert_lin -
		control all sdio mux/channel debugging logs on sdio_al.
	05: 2011-05-16 bert_lin - sync code with commit
		d59ec5ac79150791de724b9ca555fd4c708afeba
	06: 2011-05-18 bert_lin - Vigor 3040C
		201a397092af19c4971b08db7d32ec4a2be635d3
	07: 2011-05-25 bert_lin - Vigor patch
		0936fb164d340ca57f34e98fb59156ede24e9826
	08: 2011-05-30 bert_lin - enable vigor lpm logs by default
		49e34d2abf7355f6c542c2a2baf73d407d641318
		1. print out sdio_read buf for vigor by default
		2. rollback rpc commit for the issue of rpc pending data
		3. weekly sync.
	09: 2011-06-07 bert_lin - Vigor patch
		58497bdf0d72c8c6986a94e5c5704f83880d336a
		c0e40f1ffda9321408a3daf4487856387c1b84ff
	10: 2011-06-09 bert_lin - Vigor patch
		2a037451a95ae85dc08cc3cb3da0ff4c09353058
	11: 2011-06-14 bert_lin - Vigor patch
		7cc3c135da76552728ec1be75ce5924064c98946 - sdio_al
		add fatal error checking on sdio_dmux
		1. modify sdio_tty to reduce the msg of sdio_dun open fail
	12: 2011-06-14 bert_lin - Vigor patch
		e5b6c78bf7df02a780a09cb5d5fe0270303f26d2 - sdio_al
		1. Add SDIO verifications for modem reset feature
		htc: add sdio_memcpy wrapper function.
	14: 2011-07-04 bert_lin - Vigor sdio_al 3160
		1. sdio_al -> 3160
		2. Print out SMD_PKT open success message instead of
			FAIL messages.
		3. Print out [RMNTSMD] or [RMNTSDIO] prefix instead of [RIL]
	15: 2011-07-04 bert_lin - Vigor sdio_al_dloader 3160
		=> update sdio_al_dloader.c as 3160
		=> version update to 3.0001, 9k need has the same update
	16: 2011-07-15 bert_lin - remove rpc raw data logs on sdio_al layer
	17: 2011-07-20 bert_lin - add sync function between
		sdio_al_print_info & sdio_al_sdio_remove
		1. sdio_al_sdio_remove should wait for sdio_al_print_info finish
			(if sdio_al_print_info is running) otherwise the
			sdio_memcpy_fromio would cause kernel panic
		2. modify the sdio_al dump_buf function to support txt mode
	18: 2011-07-21 bert_lin - reduce LPM logs of sdio_al
		1. reduce lpm logs
		2. sync cmux changes ac6817ac9db91ed7ac4fa388d7e6d59a420d9fdf
		3. fix sdio_tty klocwork potential issue
	19: 2011-07-25 bert_lin Vigor
		d808ed2c82a57746d4e6b01877064e30adf7bb19 CRs-fixed: 294068
		msm: sdio: Change the rx threshold after sleep according to
			read avail
	20: 2011-07-26 bert_lin Vigor SDIO_AL
		1. lower down the log level of reader_count/writer_count
		2. lower down the log level of cardN wakeup from int.
		3. dad365688b01c0bca0d356f3689ed7b389208541
			msm: sdio: prevent modem sleep until completion of
			sleep sequence
		4. dmux 6470a859f21df026b78b4f8302156d5f997c9001
			msm: sdio_dmux: Clear pending write queue after reset
				notification
		5. dmux 17f0288803e5c26d80310f7ac9ae0f8ff3151565
			partial sync -> sync the locking changes of
				msm_sdio_dmux_write
	21: 2011-08-01 bert_lin Vigor SDIO_DMUX
		revert commit
	22: Viger only, has not phase in to RUBY CRC line

	23: Viger only, has not phase in to RUBY CRC line

	24: 2011-08-10 Mars Lin
		1. Trigger ketnel penic when tx pending data
		2. Declare completion for sdio_write, wait for sdio_write complete before call sdio_remove.
 */
#define HTC_VER "_24]"
#define MODULE_NAME "[SDAL" HTC_VER

#define DRV_VERSION "1.30"
#define SDIOC_CHAN_TO_FUNC_NUM(x)	((x)+2)
#define REAL_FUNC_TO_FUNC_IN_ARRAY(x)	((x)-1)
#define SDIO_PREFIX "SDIO_"
#define PEER_CHANNEL_NAME_SIZE		4
#define CHANNEL_NAME_SIZE (sizeof(SDIO_PREFIX) + PEER_CHANNEL_NAME_SIZE)
#define SDIO_TEST_POSTFIX_SIZE 5

struct sdio_al_device; /* Forward Declaration */

/**
 * Peer SDIO-Client channel configuration.
 *
 *  @is_ready - channel is ready and the data is valid.
 *
 *  @max_rx_threshold - maximum rx threshold, according to the
 *		total buffers size on the peer pipe.
 *  @max_tx_threshold - maximum tx threshold, according to the
 *		total buffers size on the peer pipe.
 *  @tx_buf_size - size of a single buffer on the peer pipe; a
 *		transfer smaller than the buffer size still
 *		make the buffer unusable for the next transfer.
 * @max_packet_size
 * @is_host_ok_to_sleep - Host marks this bit when it's okay to
 *		sleep (no pending transactions)
 */
struct peer_sdioc_channel_config {
	u32 is_ready;
	u32 max_rx_threshold; /* Downlink */
	u32 max_tx_threshold; /* Uplink */
	u32 tx_buf_size;
	u32 max_packet_size;
	u32 is_host_ok_to_sleep;
	u32 is_packet_mode;
	u32 reserved[25];
};


/**
 * Peer SDIO-Client channel statsitics.
 *
 * @last_any_read_avail - the last read avail in all the
 *		 channels including this channel.
 * @last_read_avail - the last read_avail that was read from HW
 *	    mailbox.
 * @last_old_read_avail - the last read_avail channel shadow.
 * @total_notifs - the total number of read notifications sent
 *	 to this channel client
 * @total_read_times - the total number of successful sdio_read
 *	     calls for this channel
 */
struct sdio_channel_statistics {
	int last_any_read_avail;
	int last_read_avail;
	int last_old_read_avail;
	int total_notifs;
	int total_read_times;
};

/**
 *  SDIO Channel context.
 *
 *  @name - channel name. Used by the caller to open the
 *	  channel.
 *
 *  @read_threshold - Threshold on SDIO-Client mailbox for Rx
 *				Data available bytes. When the limit exceed
 *				the SDIO-Client generates an interrupt to the
 *				host.
 *
 *  @write_threshold - Threshold on SDIO-Client mailbox for Tx
 *				Data available bytes. When the limit exceed
 *				the SDIO-Client generates an interrupt to the
 *				host.
 *
 *  @def_read_threshold - Default theshold on SDIO-Client for Rx
 *
 *  @min_write_avail - Threshold of minimal available bytes
 *					 to write. Below that threshold the host
 *					 will initiate reading the mailbox.
 *
 *  @poll_delay_msec - Delay between polling the mailbox. When
 *				 the SDIO-Client doesn't generates EOT
 *				 interrupt for Rx Available bytes, the host
 *				 should poll the SDIO-Client mailbox.
 *
 *  @is_packet_mode - The host get interrupt when a packet is
 *				available at the SDIO-client (pipe EOT
 *				indication).
 *
 *  @num - channel number.
 *
 *  @notify - Client's callback. Should not call sdio read/write.
 *
 *  @priv - Client's private context, provided to callback.
 *
 *  @is_valid - Channel is used (we have a list of
 *		SDIO_AL_MAX_CHANNELS and not all of them are in
 *		use).
 *
 *  @is_open - Channel is open.
 *
 *  @func - SDIO Function handle.
 *
 *  @rx_pipe_index - SDIO-Client Pipe Index for Rx Data.
 *
 *  @tx_pipe_index - SDIO-Client Pipe Index for Tx Data.
 *
 *  @ch_lock - Channel lock to protect channel specific Data
 *
 *  @rx_pending_bytes - Total number of Rx pending bytes, at Rx
 *				  packet list. Maximum of 16KB-1 limited by
 *				  SDIO-Client specification.
 *
 *  @read_avail - Available bytes to read.
 *
 *  @write_avail - Available bytes to write.
 *
 *  @rx_size_list_head - The head of Rx Pending Packets List.
 *
 *  @pdev - platform device - clients to probe for the sdio-al.
 *
 *  @signature - Context Validity check.
 *
 *  @sdio_al_dev - a pointer to the sdio_al_device instance of
 *   this channel
 *
 *   @statistics - channel statistics
 *
 */
struct sdio_channel {
	/* Channel Configuration Parameters*/
	char name[CHANNEL_NAME_SIZE];
	char ch_test_name[CHANNEL_NAME_SIZE+SDIO_TEST_POSTFIX_SIZE];
	int read_threshold;
	int write_threshold;
	int def_read_threshold;
	int threshold_change_cnt;
	int min_write_avail;
	int poll_delay_msec;
	int is_packet_mode;

	struct peer_sdioc_channel_config ch_config;

	/* Channel Info */
	int num;

	void (*notify)(void *priv, unsigned channel_event);
	void *priv;

	int is_valid;
	int is_open;

	struct sdio_func *func;

	int rx_pipe_index;
	int tx_pipe_index;

	struct mutex ch_lock;

	u32 read_avail;
	u32 write_avail;

	u32 peer_tx_buf_size;

	u16 rx_pending_bytes;

	struct list_head rx_size_list_head;

	struct platform_device *pdev;

	u32 total_rx_bytes;
	u32 total_tx_bytes;

	u32 signature;

	struct sdio_al_device *sdio_al_dev;

	struct sdio_channel_statistics statistics;
};

/**
 * sdio_downloader_setup
 * initializes the TTY driver
 *
 * @card: a pointer to mmc_card.
 * @num_of_devices: number of devices.
 * @channel_number: channel number.
 * @return 0 on success or negative value on error.
 *
 * The TTY stack needs to know in advance how many devices it should
 * plan to manage. Use this call to set up the ports that will
 * be exported through SDIO.
 */
int sdio_downloader_setup(struct mmc_card *card,
			  unsigned int num_of_devices,
			  int func_number,
			  int(*func)(void));

/**
 * test_channel_init
 * initializes a test channel
 *
 * @name: the channel name.
 * @return 0 on success or negative value on error.
 *
 */
int test_channel_init(char *name);

/**
 * sdio_al_register_lpm_cb
 * Allow the sdio_al test to register for lpm voting
 * notifications
 *
 * @device_handle: the device handle.
 * @wakeup_callback: callback function to be called when voting.
 *
 */
void sdio_al_register_lpm_cb(void *device_handle,
				       int(*lpm_callback)(void *, int));

/**
 * sdio_al_unregister_lpm_cb
 * Allow the sdio_al test to unregister for lpm voting
 * notifications
 *
 * @device_handle: the device handle.
 *
 */
void sdio_al_unregister_lpm_cb(void *device_handle);

#endif /* __SDIO_AL_PRIVATE__ */
