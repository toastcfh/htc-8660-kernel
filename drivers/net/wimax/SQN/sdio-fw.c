/* * This is part of the Sequans SQN1130 driver.
 * Copyright 2008 SEQUANS Communications
 * Written by Dmitriy Chumak <chumakd@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */


#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/firmware.h>
#include <linux/byteorder/generic.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/irqflags.h>
#include <linux/jiffies.h>
#include <linux/vmalloc.h>
#include <linux/fcntl.h>

#include "sdio.h"
#include "msg.h"
#include "sdio-sqn.h"
#include "sdio-driver.h"
#include "sdio-netdev.h"
#include "version.h"
#include "sdio-fw.h"

static u8 __sqn_per_file_dbg = 0;
u8* __sqn_fw_per_file_dbg_addr(void)
{
	return &__sqn_per_file_dbg;
}

#ifdef SQN_UNIQUE_FIRMWARE
char	*bootrom1130_name = "sqn1130.bin";
char	*bootrom1210_name = "sqn1210.bin";

#if   defined(CONFIG_ARM)
char	*fw1130_name = "/etc/firmware/sqn1130.fw";
char	*fw1210_name = "/etc/firmware/sqn1210.fw";
#elif defined(CONFIG_X86)
char	*fw1130_name = "/lib/firmware/sqn1130.fw";
char	*fw1210_name = "/lib/firmware/sqn1210.fw";
#endif


struct sqn_fw_header {
	u32	version;
	u32	cookie;
	u32	offset;
};

#else

char	*fw1130_name = "sqn1130.bin";
char	*fw1210_name = "sqn1210.bin";

#endif

/* Tag, Lenght, Value struct */
struct sqn_tlv {
	u32	tag;
#define SWM_INFO_TAG_SQN_ROOT		0x80000000	/* SEQUANS root tag */
#define SWM_INFO_TAG_SQN_MEMCPY		0x80000005	/* SEQUANS memcpy tag */
#define SWM_INFO_TAG_SQN_MEMSET		0x80000006	/* SEQUANS memset tag */
#define SWM_INFO_TAG_SQN_BOOTROM_GROUP	0x80040000
#define SWM_INFO_TAG_SQN_ID_GROUP	0x80010000	/* SEQUANS identification group tag */
#define SWM_INFO_TAG_SQN_FW_GROUP	0x80020000	/* SEQUANS firmware group tag */
#define SWM_INFO_TAG_SQN_FW_DATA	0x80020001	/* SEQUANS firmware data tag */
#define SWM_INFO_TAG_SQN_MAC_ADDRESS	0x80010010	/* SEQUANS mac address tag */
	u32	length;
	u8	value[0];
};


/* body of SWM_INFO_TAG_SQN_MEMCPY tag */
struct sqn_tag_memcpy {
	u32	address;
	u32	access_size;
	u32	data_size;
	u8	data[0];
};


/* body of SWM_INFO_TAG_SQN_MEMSET tag */
struct sqn_tag_memset {
	u32	address;
	u32	access_size;
	u32	size;
	u8	pattern;
};


#define SQN_1130_SDRAM_BASE      0x00000000
#define SQN_1130_SDRAM_END       0x03FFFFFF
#define SQN_1130_SDRAMCTL_BASE   0x4B400000
#define SQN_1130_SDRAMCTL_END    0x4B4003FF

#define SQN_1210_SDRAM_BASE      0x00000000
#define SQN_1210_SDRAM_END       0x07FFFFFF
#define SQN_1210_SDRAMCTL_BASE   0x20002000
#define SQN_1210_SDRAMCTL_END    0x2000207F

#ifdef SQN_UNIQUE_FIRMWARE

static int sqn_is_good_ahb_address(u32 address, enum sqn_card_version card_version)
{
	u32 sdram_base = 0;
	u32 sdram_end = 0;
	u32 sdram_ctl_base = 0;
	u32 sdram_ctl_end = 0;
	int status = 0;

	sqn_pr_enter();

	if (SQN_1130 == card_version) {
		sqn_pr_dbg("using 1130 AHB address space\n");
		sdram_base	= SQN_1130_SDRAM_BASE;
		sdram_end	= SQN_1130_SDRAM_END;
		sdram_ctl_base	= SQN_1130_SDRAMCTL_BASE;
		sdram_ctl_end	= SQN_1130_SDRAMCTL_END;
	} else if (SQN_1210 == card_version) {
		sqn_pr_dbg("using 1210 AHB address space\n");
		sdram_base	= SQN_1210_SDRAM_BASE;
		sdram_end	= SQN_1210_SDRAM_END;
		sdram_ctl_base	= SQN_1210_SDRAMCTL_BASE;
		sdram_ctl_end	= SQN_1210_SDRAMCTL_END;
	} else {
		sqn_pr_warn("Unable to check AHB address because of unknown"
			" card version\n");
		status = 0;
		goto out;
	}

	status = ((sdram_base <= address && address < sdram_end)
			|| (sdram_ctl_base <= address && address < sdram_ctl_end));
out:
	sqn_pr_leave();
	return status;
}


// Fix big buffer allocation problem during Firmware loading
/**
 *	sqn_alloc_big_buffer - tries to alloc a big buffer with kmalloc
 *	@buf: pointer to buffer
 *	@size: required buffer size
 *	@gfp_flags: GFP_* flags
 *
 *	Tries to allocate a buffer of requested size with kmalloc. If it fails,
 *	then decrease buffer size in two times (adjusting the new size to be a
 *	multiple of 4) and try again. Use 6 retries in case of failures, after
 *	this give up and try to alloc 4KB buffer if requested size bigger than
 *	4KB, otherwise allocate nothing and return 0.
 *
 *  @return a real size of allocated buffer or 0 if allocation failed
 *
 *   Normal: 3912*4kB 4833*8kB 0*16kB 0*32kB 0*64kB 0*128kB 0*256kB 0*512kB 0*1024kB 0*2048kB 0*4096kB = 54312kB
 */

static size_t sqn_alloc_big_buffer(u8 **buf, size_t size, gfp_t gfp_flags)
{
	size_t	real_size = size;
	// int	retries   = 6;
       int	retries   = 3;

	sqn_pr_enter();

	/* Try to allocate buff		sqn_pr_dbg("AHB: buf_size=%u [aln=%u size=%u pad=%u]\n"
			, buf_size, aln_size, size, pad_size);er of requested size, if it failes try to
	 * allocate a twice smaller buffer. Repeat this <retries> number of
	 * times. */
/*
	do
	{
		*buf = kmalloc(real_size, gfp_flags);
		//printk("%s: kmalloc %d in %u trial:%d\n", __func__, real_size, **buf, retries);
		//sqn_pr_info("%s: kmalloc %d in %u trial:%d\n", __func__, real_size,**buf,retries);

		if (!(*buf)) {
            printk("%s: kmalloc %d failed, trial:%d\n", __func__, real_size, retries); 
			// real_size /= 2;
            real_size /= 4;
			// adjust the size to be a multiple of 4
			real_size += real_size % 4 ? 4 - real_size % 4 : 0;
		}
	} while (retries-- > 0 && !(*buf));
   */

	// If all retries failed, then allocate 4KB buffer
	if (!(*buf)) {
		real_size = 8 * 1024;
		if (size >= real_size) {
			*buf = kmalloc(real_size, gfp_flags);
			//printk("%s: kmalloc %d in %u\n", __func__, real_size, **buf);
			//sqn_pr_info("%s: kmalloc %d in %u\n", __func__, real_size,**buf);

			// If it also failed, then just return 0, indicating
			// that we failed to alloc buffer
			if (!(*buf))
				real_size = 0;
		} else {
			// We should _not_ return buffer bigger than requested
			// real_size = 0;

			//printk("%s: We should _not_ return buffer bigger than requested size:%d real_size:%d\n", __func__, size, real_size);
			*buf = kmalloc(size, gfp_flags);
			real_size = size;
		}
	}

	if (!(*buf)) {
		printk("%s: kmalloc failed!!!\n", __func__);
	}

	sqn_pr_leave();

	return real_size;
}


enum sqn_ahb_direction {
	SQN_AHB_READ
	, SQN_AHB_WRITE
};

static int sqn_ahb_read_write(struct sdio_func *func, u32 addr, u8 *data
	, u32 size, enum sqn_ahb_direction dir)
{
	struct sqn_sdio_card *sqn_card = sdio_get_drvdata(func);
	int rv = 0;

	sqn_pr_enter();

	if (addr % 4 || size % 4) {
		sqn_pr_err("AHB address [0x%x] and size [%u] should be a"
			" multiple of 4\n", addr, size);
		rv = -1;
		goto out;
	}

	if (!sqn_is_good_ahb_address(addr, sqn_card->version)) {
		sqn_pr_err("incorrect AHB address: 0x%x\n", addr);
		goto out;
	}

	sqn_pr_dbg("set AHB ADA_ADDR to 0x%x\n", addr);
	sdio_writel(func, addr, SQN_SDIO_ADA_ADDR, &rv);
	if (rv) {
		sqn_pr_err("failed to set AHB ADA_ADDR register to 0x%x\n", addr);
		goto out;
	}

	switch (dir) {
	case SQN_AHB_READ:
		sqn_pr_dbg("read data from AHB ADA_RDWR, size=%u\n", size);
		rv = sdio_readsb(func, data, SQN_SDIO_ADA_RDWR, size);
		if (rv) {
			sqn_pr_err("failed to read data from AHB ADA_RDWR register\n");
			goto out;
		}
		break;
	case SQN_AHB_WRITE:
		sqn_pr_dbg("write data to AHB ADA_RDWR, size=%u\n", size);
		rv = sdio_writesb(func, SQN_SDIO_ADA_RDWR, data, size);
		if (rv) {
			sqn_pr_err("failed to write data to AHB ADA_RDWR register\n");
			goto out;
		}
		break;
	default:
		sqn_pr_err("incorrect AHB direction\n");
	}
out:
	sqn_pr_leave();
	return rv;
}


static int sqn_write_data(struct sdio_func *func, u32 addr, const void *data
	, u32 size, u32 access_size)
{
	int rv = 0;
	struct sqn_sdio_card *sqn_card = sdio_get_drvdata(func);

	sqn_pr_enter();
	sdio_claim_host(func);
	//printk("%s: size %d access_size %d\n", __func__, size,access_size);
	if (sqn_is_good_ahb_address(addr, sqn_card->version)
		&& 0 == (size % 4) )
	{
		/* write data using AHB */
		u8 *buf = 0;
		size_t buf_size = 0;
		u32 written_size = 0;

#ifdef DEBUG
		u8 *read_data  = 0;
#endif

		//printk("write data using AHB with small kmalloc\n");
		sdio_writel(func, addr, SQN_SDIO_ADA_ADDR, &rv);
		if (rv) {
			sqn_pr_dbg("can't set SQN_SDIO_ADA_ADDR register\n");
			goto out;
		}
		sqn_pr_dbg("after SQN_SDIO_ADA_ADDR\n");

		written_size = 0;
		buf_size = sqn_alloc_big_buffer(&buf, size, GFP_KERNEL | GFP_DMA);
		//printk("%s: buf_size %d size%d\n", __func__, buf_size,size); 
		if (!buf) {
			sqn_pr_err("failed to allocate buffer of %u bytes\n", size);
			goto out;
		}

		do {
			memcpy(buf, data + written_size, buf_size);
			rv = sdio_writesb(func, SQN_SDIO_ADA_RDWR, buf, buf_size);
			if (rv) {
				sqn_pr_dbg("can't write to SQN_SDIO_ADA_RDWR register\n");
				goto out;
			}
			written_size += buf_size;
			if (written_size + buf_size > size)
				buf_size = size - written_size;
		} while (written_size < size);
		kfree(buf);

		/*
		 * Workaround when sdio_writesb doesn't work because DMA
		 * alignment
		 */
		/*
		int i = 0;
		for (; i < size/4; ++i) {
			sdio_writel(func, *((u32*)data + i), SQN_SDIO_ADA_RDWR, &rv);
			if (rv) {
				sqn_pr_dbg("can't write to SQN_SDIO_ADA_RDWR register\n");
				goto out;
			}
		}
		*/

		sqn_pr_dbg("after SQN_SDIO_ADA_RDWR\n");

		/* ******** only for debugging ******** */
		/* validate written data */
/* #ifdef DEBUG */
#if 0
		sqn_pr_dbg("reading data using AHB\n");
		sdio_writel(func, addr, SQN_SDIO_ADA_ADDR, &rv);
		if (rv) {
			sqn_pr_dbg("can't set SQN_SDIO_ADA_ADDR register\n");
			goto out;
		}
		sqn_pr_dbg("after SQN_SDIO_ADA_ADDR\n");

		read_data = kmalloc(size, GFP_KERNEL);
		rv = sdio_readsb(func, read_data, SQN_SDIO_ADA_RDWR, size);
		if (rv) {
			sqn_pr_dbg("can't read from SQN_SDIO_ADA_RDWR register\n");
			kfree(read_data);
			goto out;
		}

		if (memcmp(data, read_data, size))
			sqn_pr_dbg("WARNING: written data are __not__ equal\n");
		else
			sqn_pr_dbg("OK: written data are equal\n");

		kfree(read_data);
#endif /* DEBUG */
		/* ******** only for debugging ******** */
	}
	else if (sqn_is_good_ahb_address(addr, sqn_card->version)
		&& 0 != (size % 4) )
	{
		/* write data using AHB */
		/*
		 * In firmware, target AHB address may not be 4 byte aligned so
		 * we need to read data from card before the "addr" to be sure
		 * we not overwrite any data. Also the same is applied to the
		 * "size", if it's not a multiple of 4 we should read data after
		 * "addr + size".
		 *
		 * +-----------------+----------+--------------+
		 * | addr - aln_size |   addr   | addr + size  |
		 * +-----------------+----------+--------------+
		 * |<--- aln_size -->|<- size ->|<- pad_size ->|
		 *
		 */
		u8 aln_size = addr % 4;
		/* Also we need to pad AHB data size to be a multiple of 4 */
		u8 pad_size = (size + aln_size) % 4 ? 4 - (size + aln_size)% 4 : 0;
		u32 buf_size = aln_size + size + pad_size;
		u8 *buf = 0;

		//printk("write data using AHB normal\n");
		sqn_pr_dbg("AHB: buf_size=%u [aln=%u size=%u pad=%u]\n"
			, buf_size, aln_size, size, pad_size);

		buf = kmalloc(buf_size, GFP_KERNEL | GFP_DMA);
		if (!buf) {
			sqn_pr_err("failed to alloc buffer of size %d\n", buf_size);
			rv = -1;
			goto out;
		}

		if (aln_size) {
			/* fill first 4 bytes of buf with data from card */
			rv = sqn_ahb_read_write(func, addr - aln_size, buf
				, 4, SQN_AHB_READ);
			if (rv)
				goto out;
		}

		if (pad_size) {
			/* fill last 4 bytes of buf with data from card */
			rv = sqn_ahb_read_write(func, addr + size + pad_size - 4
				, buf + buf_size - 4, 4, SQN_AHB_READ);
			if (rv)
				goto out;
		}

		memcpy(buf + aln_size, data, size);

		rv = sqn_ahb_read_write(func, addr - aln_size,
			buf, buf_size, SQN_AHB_WRITE);
		if (rv)
			goto out;

#ifdef DEBUG
/* Maintain in compilable state but use only if needed */
if (0) {
		/* Validate written data */
		memset(buf, 0, buf_size);

		rv = sqn_ahb_read_write(func, addr - aln_size,
			buf, buf_size, SQN_AHB_READ);
		if (rv)
			goto out;

		if (memcmp(data, buf + aln_size, size))
			sqn_pr_dbg("FAILED: written data are __NOT__ equal\n");
		else
			sqn_pr_dbg("OK: written data are equal\n");
}
#endif /* DEBUG */

		kfree(buf);
	} else if (4 == access_size && size >= 4) {
		/* write data using CMD53 */
		printk("%s: write data using CMD53\n", __func__);
		sqn_pr_dbg("write data using CMD53: addr 0x%x, size %u\n"
			, addr, size);
		rv = sdio_memcpy_toio(func, addr, (void*)data , size);
	} else {
		/* write data using CMD52 */
		int i = 0;
		printk("%s: write data using CMD52\n", __func__);
		sqn_pr_dbg("write data using CMD52: addr 0x%x, size %u\n"
			, addr, size);
		for (i = 0; i < size; ++i) {
			sdio_writeb(func, *((u8*)data + i), addr + i, &rv);
			if (rv) {
				sqn_pr_dbg("failed to write 1 byte to 0x%x addr"
					" using CMD52\n", addr + i);
				goto out;
			}
		}
	}
out:
	sdio_release_host(func);
	sqn_pr_leave();
	return rv;
}


static int sqn_handle_memcpy_tag(struct sdio_func *func
	, const struct sqn_tag_memcpy * mcpy_tag_const)
{
	int rv = 0;
	struct sqn_tag_memcpy mcpy_tag = { 0 };

	sqn_pr_enter();

	/*
	 * Convert values accordingly to platform "endianes"
	 * (big or little endian) because bootstrapper file
	 * data is big endian
	 */
	mcpy_tag.address = be32_to_cpu(mcpy_tag_const->address);
	mcpy_tag.access_size = be32_to_cpu(mcpy_tag_const->access_size);
	mcpy_tag.data_size = be32_to_cpu(mcpy_tag_const->data_size);

	/* sqn_pr_dbg("----------------------------------------\n"); */
	sqn_pr_dbg("address: 0x%02X access_size: %u data_size: %u\n"
			, mcpy_tag.address, mcpy_tag.access_size
			, mcpy_tag.data_size);
	/* sqn_pr_dbg_dump("|", mcpy_tag.data, mcpy_tag.data_size); */

	rv = sqn_write_data(func, mcpy_tag.address, mcpy_tag_const->data
		, mcpy_tag.data_size, mcpy_tag.access_size);

	sqn_pr_leave();
	return rv;
}


static int sqn_handle_memset_tag(struct sdio_func *func
	, const struct sqn_tag_memset * mset_tag_const)
{
	int rv = 0;
	struct sqn_tag_memset mset_tag = { 0 };
	u8 *buf = 0;
	const u32 buf_size = 4096;
	u32 left_bytes = 0;

	sqn_pr_enter();

	/*
	 * Convert values accordingly to platform "endianes"
	 * (big or little endian) because bootstrapper file
	 * data is big endian
	 */
	mset_tag.address = be32_to_cpu(mset_tag_const->address);
	mset_tag.access_size = be32_to_cpu(mset_tag_const->access_size);
	mset_tag.size = be32_to_cpu(mset_tag_const->size);
	mset_tag.pattern = mset_tag_const->pattern;

	/* sqn_pr_dbg("----------------------------------------\n"); */
	sqn_pr_dbg("address: 0x%02X access_size: %u size: %u pattern 0x%02X\n"
			, mset_tag.address, mset_tag.access_size
			, mset_tag.size, mset_tag.pattern);

	buf = kmalloc(buf_size, GFP_KERNEL);
	if (0 == buf)
		return -ENOMEM;

	memset(buf, mset_tag.pattern, buf_size);

	left_bytes = mset_tag.size;

	while (left_bytes) {
		u32 bytes_to_write = min(buf_size, left_bytes);
		rv = sqn_write_data(func, mset_tag.address, buf, bytes_to_write,
			mset_tag.access_size);
		if (rv)
			goto out;
		left_bytes -= bytes_to_write;
	}

out:
	kfree(buf);
	sqn_pr_leave();
	return rv;
}


static int sqn_char_to_int(u8 c)
{
	int rv = 0;

	if ('0' <= c && c <= '9') {
		rv = c - '0';
	} else if ('a' <= c && c <= 'f') {
		rv = c - 'a' + 0xA;
	} else if ('A' <= c && c <= 'F') {
		rv = c - 'A' + 0xA;
	} else {
		rv = -1;
	}

	return rv;
}


static int sqn_get_mac_addr_from_str(const u8 *data, u32 length, u8 *result)
{
	int rv = 0;
	int i = 0;

	sqn_pr_enter();

	if (0 == length) {
		rv = -1;
		goto out;
	}

	/*
	 * Check if we have delimiters on appropriate places:
	 *
	 * X X : X X : X X : X X  :  X  X  :  X  X
	 * 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
	 */

	if ( !( ( ':' == data[2] || '-' == data[2])
		&& ( ':' == data[5] || '-' == data[5])
		&& ( ':' == data[8] || '-' == data[8])
		&& ( ':' == data[11] || '-' == data[11])
		&& ( ':' == data[14] || '-' == data[14]) ))
	{
		sqn_pr_err("can't get mac address from firmware"
			" - incorrect mac address\n");
		rv = -1;
		goto out;
	}

	i = 0;
	while (i < length) {
		int high = 0;
		int low = 0;

		if ((high = sqn_char_to_int(data[i])) >= 0
			&& (low = sqn_char_to_int(data[i + 1])) >= 0)
		{
			result[i/3] = low;
			result[i/3] |= high << 4;
		} else {
			sqn_pr_err("can't get mac address from firmware"
					" - incorrect mac address\n");
			rv = -1;
			goto out;
		}

		i += 3;
	}

out:
	sqn_pr_dbg_dump("mac_addr:", result, ETH_ALEN);
	sqn_pr_leave();
	return rv;
}


static int sqn_handle_mac_addr_tag(struct sdio_func *func, const u8 *data, u32 length)
{
	int rv = 0;
	struct sqn_private *priv =
		((struct sqn_sdio_card *)sdio_get_drvdata(func))->priv;

	sqn_pr_enter();

	/*
	 * This tag could contain one or two mac addresses in string
	 * form, delimited by some symbol (space or something else).
	 * Each mac address written as a string has constant length.
	 * Thus we can determine the number of mac addresses by the
	 * length of the tag:
	 *
	 * mac addr length in string form: XX:XX:XX:XX:XX:XX = 17 bytes
	 * tag length: 17 bytes [ + 1 byte + 17 bytes ]
	 */

#define MAC_ADDR_STRING_LEN	17

	/*
	 * If we have only one mac addr we should increment it by one
	 * and use it.
	 * If we have two mac addresses we should use a second one.
	 */

	if (MAC_ADDR_STRING_LEN <= length
		&& length < 2 * MAC_ADDR_STRING_LEN + 1)
	{
		sqn_pr_dbg("single mac address\n");
		/* we have only one mac addr */
		sqn_get_mac_addr_from_str(data, length, priv->mac_addr);

		// Andrew 0720
		// ++(priv->mac_addr[ETH_ALEN - 1])
		// real MAC: 38:E6:D8:86:00:00 
		// hboot will store: 38:E6:D8:85:FF:FF (minus 1)
		// sdio need to recovery it by plusing 1: 38:E6:D8:86:00:00 (plus 1)

		if ((++(priv->mac_addr[ETH_ALEN - 1])) == 0x00)
			if ((++(priv->mac_addr[ETH_ALEN - 2])) == 0x00)
				if ((++(priv->mac_addr[ETH_ALEN - 3])) == 0x00)
					if ((++(priv->mac_addr[ETH_ALEN - 4])) == 0x00)
						if ((++(priv->mac_addr[ETH_ALEN - 5])) == 0x00)
							++(priv->mac_addr[ETH_ALEN - 6]);

	}
	else if (2 * MAC_ADDR_STRING_LEN + 1 == length) { /* we have two macs */
		sqn_pr_dbg("two mac addresses, using second\n");
		sqn_get_mac_addr_from_str(data + MAC_ADDR_STRING_LEN + 1
			, length - (MAC_ADDR_STRING_LEN + 1), priv->mac_addr);
	}
	else { /* incorrect data length */
		sqn_pr_err("can't get mac address from bootloader"
			" - incorrect mac address length\n");
		rv = -1;
		goto out;
	}

	sqn_pr_info("setting MAC address from bootloader: "
		"%02x:%02x:%02x:%02x:%02x:%02x\n", priv->mac_addr[0]
		, priv->mac_addr[1], priv->mac_addr[2], priv->mac_addr[3]
		, priv->mac_addr[4], priv->mac_addr[5]);

out:
	sqn_pr_leave();
	return rv;
}


/** sqn_parse_and_load_tlv_data - reads a bootrom/firmware file, analize it
 *  and loads data to the card.
 *
 *  Bootrom/firmware consists of Tag, Length, Value (TLV) sections.
 *  Each section starts with 4 bytes tag. Then goes length of data (4 bytes)
 *  and then a data itself.
 *
 *  All TLV values stored in BIG ENDIAN format.
 */
static int sqn_parse_and_load_tlv_data(struct sdio_func *func, const u8 *data, int size)
{
	const struct sqn_tlv *tlv_const = (const struct sqn_tlv*) data;
	struct sqn_tlv tlv = { 0 };
	int rv = 0;

	sqn_pr_enter();

	while (size > sizeof(struct sqn_tlv)) {
		/*
		 * Convert values accordingly to platform "endianes"
		 * (big or little endian) because bootstrapper file
		 * data is big endian
		 */
		tlv.tag = be32_to_cpu(tlv_const->tag);
		tlv.length = be32_to_cpu(tlv_const->length);
		sqn_pr_dbg("current tag: tlv %p size %d\n", tlv_const, size);

		switch (tlv.tag) {
		case SWM_INFO_TAG_SQN_ROOT:
			sqn_pr_dbg("ROOT tag, length: %u\n", tlv.length);
			sqn_pr_dbg("set data size to the length of ROOT tag:"
				" current size %d, new size %u\n"
				, size, sizeof(*tlv_const) + tlv.length);
			size = sizeof(*tlv_const) + tlv.length;
		case SWM_INFO_TAG_SQN_BOOTROM_GROUP:
		case SWM_INFO_TAG_SQN_ID_GROUP:
		case SWM_INFO_TAG_SQN_FW_GROUP:
		case SWM_INFO_TAG_SQN_FW_DATA:
			/*
			 * This tag is a "container" tag - it's value field
			 * contains other tags
			 */

			/* sqn_pr_dbg("========================================\n"); */
			sqn_pr_dbg("tag: CONTAINER %x length: %u\n", tlv.tag
				, tlv.length);
			/* sqn_pr_dbg_dump("|", tlv_const->value, tlv.length); */

			/*
			 * If this is a buggy tag, adjust length to
			 * the rest of data
			 */
			if (0 == tlv.length)
				tlv.length = size - sizeof(*tlv_const);

			rv = sqn_parse_and_load_tlv_data(func, (u8*) tlv_const->value
				, tlv.length);
			if (rv)
				goto out;
			break;

		case SWM_INFO_TAG_SQN_MEMCPY:
			/* sqn_pr_dbg("========================================\n"); */
			sqn_pr_dbg("tag: SWM_INFO_TAG_SQN_MEMCPY length: %u\n"
					, tlv.length);
			/* sqn_pr_dbg_dump("|", tlv_const->value, tlv.length); */
			rv = sqn_handle_memcpy_tag(func
				, (struct sqn_tag_memcpy*) tlv_const->value);
			if (rv)
				goto out;
			break;

		case SWM_INFO_TAG_SQN_MEMSET:
			/* sqn_pr_dbg("========================================\n"); */
			sqn_pr_dbg("tag: SWM_INFO_TAG_SQN_MEMSET length: %u\n"
					, tlv.length);
			/* sqn_pr_dbg_dump("|", tlv_const->value, tlv.length); */
			rv = sqn_handle_memset_tag(func
				, (struct sqn_tag_memset*) tlv_const->value);
			if (rv)
				goto out;
			break;

		case SWM_INFO_TAG_SQN_MAC_ADDRESS:
			/* sqn_pr_dbg("========================================\n"); */
			sqn_pr_dbg("tag: SWM_INFO_TAG_SQN_MAC_ADDRESS length: %u\n"
					, tlv.length);
			/* sqn_pr_dbg_dump("|", tlv_const->value, tlv.length); */

			rv = sqn_handle_mac_addr_tag(func, tlv_const->value
				, tlv.length);
			if (rv)
				goto out;
			break;

		default:
			/* skip all other tags */
			/* sqn_pr_dbg("========================================\n"); */
			sqn_pr_dbg("tag: UNKNOWN %x length: %u\n"
					, tlv.tag, tlv.length);
			/* sqn_pr_dbg_dump("|", tlv_const->value, tlv.length); */
			break;
		}

		/* increment tlv to point it to the beginning of the next
		 * sqn_tlv struct and decrement size accordingly
		 */
		size = (int)(size - (sizeof(*tlv_const) + tlv.length));
		tlv_const = (const struct sqn_tlv*) ((const u8*)tlv_const + sizeof(*tlv_const) + tlv.length);
		sqn_pr_dbg("next tag: tlv %p size %d\n", tlv_const, size);
	}

	if (0 != size) {
		/*
		 * Something wrong with parsing of tlv values, but we don't
		 * return error, let's try to proceed with card initialization.
		 */
		sqn_pr_warn("something wrong with parsing of tlv values"
			", size = %d\n", size);
	}
out:
	sqn_pr_leave();
	return rv;
}


static int sqn_load_bootrom(struct sdio_func *func)
{
	int rv = 0;
	const struct firmware *fw = 0;
	struct sqn_sdio_card *sqn_card = sdio_get_drvdata(func);

	sqn_pr_enter();

	sqn_pr_info("trying to find bootrom image: '%s'\n", bootrom_name);

	if ((rv = request_firmware(&fw, bootrom_name, &func->dev)))
		goto out;

	if (SQN_1130 == sqn_card->version) {
		sdio_claim_host(func);

		/* properly setup registers for firmware loading */
		sqn_pr_dbg("setting up SQN_H_SDRAM_NO_EMR register\n");
		sdio_writeb(func, 0, SQN_H_SDRAM_NO_EMR, &rv);
		if (rv) {
			sdio_release_host(func);
			goto out;
		}

		sqn_pr_dbg("setting up SQN_H_SDRAMCTL_RSTN register\n");
		sdio_writeb(func, 1, SQN_H_SDRAMCTL_RSTN, &rv);
		sdio_release_host(func);
		if (rv)
			goto out;
	}

	sqn_pr_info("loading bootrom to the card...\n");
	rv = sqn_parse_and_load_tlv_data(func, fw->data, fw->size);
	if (rv)
		goto out;
out:
	release_firmware(fw);
	sqn_pr_leave();
	return rv;
}


#define SQN_STARTUP_SCRIPT_ADDR 0xA00000

static int sqn_load_startup_script(struct sdio_func *func)
{
	int rv = 0;
	const struct firmware *fw = 0;
	struct sqn_sdio_card *sqn_card = sdio_get_drvdata(func);

	sqn_pr_enter();

	sqn_pr_info("trying to find startup script: '%s'\n", bootrom_name);

	if ((rv = request_firmware(&fw, bootrom_name, &func->dev)))
		goto out;

	sqn_pr_info("loading startup script (size %u) to the card...\n", fw->size);
	rv = sqn_write_data(func, SQN_STARTUP_SCRIPT_ADDR, fw->data
		, fw->size, 4);
	if (rv)
		goto out;
out:
	release_firmware(fw);
	sqn_pr_leave();
	return rv;
}


static int sqn_load_fw(struct sdio_func *func)
{
	int rv = 0;
	u8 *fw_data = 0;
	int fw_size = 0;
	struct file *fp = 0;
	ssize_t read_size = 0;
	mm_segment_t old_fs;
	/* struct task_struct *cur_task = 0; */
	struct sqn_sdio_card *sqn_card = sdio_get_drvdata(func);

	sqn_pr_enter();

	sqn_pr_info("sqn_load_fw ...\n");
	sqn_pr_info("trying to open firmware image: '%s'\n", firmware_name);

	fp = filp_open(firmware_name, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		sqn_pr_err("failed to open firmware file '%s'\n", firmware_name);
// APP FW open recovery mechanism +
// If load link file /data/wimax/default.fw failed, try to load /data/wimax/app_default.fw directly.
		sqn_pr_info("trying to open firmware image: '%s'\n", firmware_name_recovery);
		fp = filp_open(firmware_name_recovery, O_RDONLY, 0);
		if (IS_ERR(fp)) {
			sqn_pr_err("failed to open firmware_recovery file '%s'\n", firmware_name_recovery);
			rv = -1;
			goto out;
		}

		//rv = -1;
		//goto out;
// APP FW open recovery mechanism -
	}

	/* cur_task = current; */
	fw_size = (int) fp->f_dentry->d_inode->i_size;

	/* dummy check: fw should contain at least fw_header and one tlv and
	 * should be less than 20 MB in size */
#define SQN_MAX_FW_SIZE		(20 * 1024 * 1024)
	if (fw_size < sizeof(struct sqn_fw_header) + sizeof(struct sqn_tlv)
		|| fw_size > SQN_MAX_FW_SIZE)
	{
		sqn_pr_err("incorrect firmware size\n");
		goto close_fp;
	}
#undef SQN_MAX_FW_SIZE

	sqn_pr_dbg("try to vmalloc %d size buffer\n", fw_size);
	fw_data = vmalloc(fw_size);
	if (!fw_data) {
		sqn_pr_err("failed to alloc memory to hold firmware\n");
		rv = -1;
		goto close_fp;
	} else {
		sqn_pr_dbg("vmalloc %p\n", fw_data);
	}

	sqn_pr_dbg("try to read firmware file\n");
#ifdef CONFIG_ARM
	sqn_pr_dbg("current thread address limit %x\n"
		, (u32) current_thread_info()->addr_limit);
#elif defined(CONFIG_X86)
	sqn_pr_dbg("current thread address limit %x\n"
		, (u32) current_thread_info()->addr_limit.seg);
#endif

	/* Set address limit to KERNEL for current proccess to
	 * workaround address limit check in read()
	 */
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	read_size = fp->f_op->read(fp, (char*) fw_data, fw_size, &fp->f_pos);
	set_fs(old_fs);

	if (read_size != fw_size) {
		sqn_pr_err("failed to read firmware into a buffer"
			", read size = %d\n", read_size);
		rv = -1;
		goto free_buf;
	}

	sqn_pr_dbg("fw_data 0x%p fw_size %d\n", fw_data, fw_size);
	sqn_pr_dbg_dump("FW:", fw_data, 100);

	sqn_pr_info("loading firmware to the card...\n");
	rv = sqn_parse_and_load_tlv_data(func
		/* skip FW header */
		, fw_data + sizeof(struct sqn_fw_header)
		, fw_size - sizeof(struct sqn_fw_header));
	if (rv)
		goto free_buf;
free_buf:
	sqn_pr_dbg("free firmware buffer\n");
	vfree(fw_data);
close_fp:
	sqn_pr_dbg("close firmware file\n");
	fput(fp);
out:
	sqn_pr_leave();
	return rv;
}


/** sqn_load_firmware - loads firmware to card
 *  @func: SDIO function, used to transfer data via SDIO interface,
 *         also used to obtain pointer to device structure.
 *
 *  Load bootrom and try to find a firmware, if it present - load it also.
 */
int sqn_load_firmware(struct sdio_func *func)
{
	int rv = 0;
	u32 jstart = jiffies;
	u32 jend = 0;

	sqn_pr_enter();

	rv = sqn_load_bootrom(func);
	if (rv) {
		sqn_pr_err("failed to load bootrom\n");
		goto out;
	}

	rv = sqn_load_fw(func);
	if (rv)
		sqn_pr_err("failed to load firmware, but it still can be loaded"
			" by 'sequansd' - a userspace application\n");

	bootrom_name = SQN_DEFAULT_STARTUP_SCRIPT_NAME;
	rv = sqn_load_startup_script(func);
	if (rv) {
		sqn_pr_err("failed to load startup script\n");
		goto out;
	}

	jend = jiffies;
	sqn_pr_dbg("load time: jiffies: start=%u end=%u diff=%d msec: %d\n"
		, jstart, jend, jend - jstart, jiffies_to_msecs(jend - jstart));
	sqn_pr_info("bootrom/firmware is loaded, time %d msec\n"
		, jiffies_to_msecs(jend - jstart));

	sqn_pr_info("starting the card...\n");
	sdio_claim_host(func);
	sdio_writeb(func, 1, SQN_H_CRSTN, &rv);
	sdio_release_host(func);
	if (rv) {
		sqn_pr_info("  [FAILED]\n");
		goto out;
	}
out:
	sqn_pr_leave();
	return rv;
}



#else

static int is_good_ahb_address(u32 address, enum sqn_card_version card_version)
{
	u32 sdram_base = 0;
	u32 sdram_end = 0;
	u32 sdram_ctl_base = 0;
	u32 sdram_ctl_end = 0;
	int status = 0;

	sqn_pr_enter();

	if (address % 4)
		return 0;

	if (SQN_1130 == card_version) {
		sqn_pr_dbg("using 1130 AHB address boundaries\n");
		sdram_base	= SQN_1130_SDRAM_BASE;
		sdram_end	= SQN_1130_SDRAM_END;
		sdram_ctl_base	= SQN_1130_SDRAMCTL_BASE;
		sdram_ctl_end	= SQN_1130_SDRAMCTL_END;
	} else if (SQN_1210 == card_version) {
		sqn_pr_dbg("using 1210 AHB address boundaries\n");
		sdram_base	= SQN_1210_SDRAM_BASE;
		sdram_end	= SQN_1210_SDRAM_END;
		sdram_ctl_base	= SQN_1210_SDRAMCTL_BASE;
		sdram_ctl_end	= SQN_1210_SDRAMCTL_END;
	} else {
		sqn_pr_warn("Can't check AHB address because of unknown"
			" card version\n");
		status = 0;
		goto out;
	}

	status = ((sdram_base <= address && address < sdram_end)
			|| (sdram_ctl_base <= address && address < sdram_ctl_end));
out:
	sqn_pr_leave();
	return status;
}
 
// Fix big buffer allocation problem during Firmware loading
/**
 *	sqn_alloc_big_buffer - tries to alloc a big buffer with kmalloc
 *	@buf: pointer to buffer
 *	@size: required buffer size
 *	@gfp_flags: GFP_* flags
 *
 *	Tries to allocate a buffer of requested size with kmalloc. If it fails,
 *	then decrease buffer size in two times (adjusting the new size to be a
 *	multiple of 4) and try again. Use 6 retries in case of failures, after
 *	this give up and try to alloc 4KB buffer if requested size bigger than
 *	4KB, otherwise allocate nothing and return 0.
 *
 *  @return a real size of allocated buffer or 0 if allocation failed
 * 
 *   Normal: 3912*4kB 4833*8kB 0*16kB 0*32kB 0*64kB 0*128kB 0*256kB 0*512kB 0*1024kB 0*2048kB 0*4096kB = 54312kB
 */

static size_t sqn_alloc_big_buffer(u8 **buf, size_t size, gfp_t gfp_flags)
{
	size_t	real_size = size;
	// int	retries   = 6;
    // int	retries   = 3;

	sqn_pr_enter();

	/* Try to allocate buffer of requested size, if it failes try to
	 * allocate a twice smaller buffer. Repeat this <retries> number of
	 * times. */
	/*
	do
	{
		*buf = kmalloc(real_size, gfp_flags);
		printk("%s: kmalloc %d in %x trial:%d\n", __func__, real_size, *buf, retries); 

		if (!(*buf)) {
            printk("%s: kmalloc %d failed, trial:%d\n", __func__, real_size, retries); 
			// real_size /= 2;
            real_size /= 4;
			// adjust the size to be a multiple of 4
			real_size += real_size % 4 ? 4 - real_size % 4 : 0;
		}
	} while (retries-- > 0 && !(*buf));
    */

	// If all retries failed, then allocate 4KB buffer
	if (!(*buf)) {
		real_size = 8 * 1024;
		if (size >= real_size) {
			*buf = kmalloc(real_size, gfp_flags);
			// printk("%s: kmalloc %d in %x\n", __func__, real_size, *buf); 

			// If it also failed, then just return 0, indicating
			// that we failed to alloc buffer
			if (!(*buf))
				real_size = 0;
		} else {
			// We should _not_ return buffer bigger than requested
			// real_size = 0;
						
			// printk("%s: We should _not_ return buffer bigger than requested size:%d real_size:%d\n", __func__, size, real_size); 
			*buf = kmalloc(size, gfp_flags);
			real_size = size;			
		}
	} 

	sqn_pr_leave();

	return real_size;
}

#define SQN_SDIO_ADA_ADDR	0x00002060
#define SQN_SDIO_ADA_RDWR	0x00002064


static int write_data(struct sdio_func *func, u32 addr, void *data
	, u32 size, u32 access_size)
{
	int rv = 0, retry = 0;
	struct sqn_sdio_card *sqn_card = sdio_get_drvdata(func);

	sqn_pr_enter();
	sdio_claim_host(func);

	if (is_good_ahb_address(addr, sqn_card->version)
		&& 0 == (size % 4) && 4 == access_size)
	{
		/* write data using AHB */
		u8 *buf = 0;
		size_t buf_size = 0;
		u32 written_size = 0;

#ifdef DEBUG
		u8 *read_data  = 0;
#endif

		sqn_pr_dbg("write data using AHB\n");
		sdio_writel(func, addr, SQN_SDIO_ADA_ADDR, &rv);
		if (rv) {
			sqn_pr_dbg("can't set SQN_SDIO_ADA_ADDR register\n");
			goto out;
		}
		sqn_pr_dbg("after SQN_SDIO_ADA_ADDR\n");

		written_size = 0;
		buf_size = sqn_alloc_big_buffer(&buf, size, GFP_KERNEL | GFP_DMA);
		if (!buf) {
			sqn_pr_err("failed to allocate buffer of %u bytes\n", size);
			goto out;
		}

		do {
			memcpy(buf, data + written_size, buf_size);
			rv = sdio_writesb(func, SQN_SDIO_ADA_RDWR, buf, buf_size);
			if (rv) {
				sqn_pr_dbg("can't write to SQN_SDIO_ADA_RDWR register\n");
				goto out;
			}
			written_size += buf_size;
			if (written_size + buf_size > size)
				buf_size = size - written_size;
		} while (written_size < size);
		kfree(buf);

		/*
		 * Workaround when sdio_writesb doesn't work because DMA
		 * alignment
		 */
		/*
		int i = 0;
		for (; i < size/4; ++i) {
			sdio_writel(func, *((u32*)data + i), SQN_SDIO_ADA_RDWR, &rv);
			if (rv) {
				sqn_pr_dbg("can't write to SQN_SDIO_ADA_RDWR register\n");
				goto out;
			}
		}
		*/

		sqn_pr_dbg("after SQN_SDIO_ADA_RDWR\n");

		/* ******** only for debugging ******** */
		/* validate written data */
/* #ifdef DEBUG */
#if 0
		sqn_pr_dbg("reading data using AHB\n");
		sdio_writel(func, addr, SQN_SDIO_ADA_ADDR, &rv);
		if (rv) {
			sqn_pr_dbg("can't set SQN_SDIO_ADA_ADDR register\n");
			goto out;
		}
		sqn_pr_dbg("after SQN_SDIO_ADA_ADDR\n");

		read_data = kmalloc(size, GFP_KERNEL);
		rv = sdio_readsb(func, read_data, SQN_SDIO_ADA_RDWR, size);
		if (rv) {
			sqn_pr_dbg("can't read from SQN_SDIO_ADA_RDWR register\n");
			kfree(read_data);
			goto out;
		}

		if (memcmp(data, read_data, size))
			sqn_pr_dbg("WARNING: written data are __not__ equal\n");
		else
			sqn_pr_dbg("OK: written data are equal\n");

		kfree(read_data);
#endif /* DEBUG */
		/* ******** only for debugging ******** */

	} else if (4 == access_size && size >= 4) {
		/* write data using CMD53 */
		sqn_pr_dbg("write data using CMD53\n");
		rv = sdio_memcpy_toio(func, addr, data , size);
	} else {
		/* write data using CMD52 */
		/* not implemented yet, so we use CMD53 */
		/* rv = sdio_memcpy_toio(func, addr, data , size); */
		int i = 0;
		sqn_pr_dbg("write data using CMD52\n");
		for (i = 0; i < size; ++i) {
			sdio_writeb(func, *((u8*)data + i), addr + i, &rv);
			if (rv) {
				sqn_pr_dbg("can't write 1 byte to %xh addr using CMD52\n"
					, addr + i);
				goto out;
			}
		}
	}

out:
	sdio_release_host(func);
	sqn_pr_leave();
	return rv;
}


static int sqn_handle_memcpy_tag(struct sdio_func *func
	, struct sqn_tag_memcpy * mcpy_tag)
{
	int rv = 0;

	sqn_pr_enter();

	/*
	 * Convert values accordingly to platform "endianes"
	 * (big or little endian) because bootstrapper file
	 * data is big endian
	 */
	mcpy_tag->address = be32_to_cpu(mcpy_tag->address);
	mcpy_tag->access_size = be32_to_cpu(mcpy_tag->access_size);
	mcpy_tag->data_size = be32_to_cpu(mcpy_tag->data_size);

	/* sqn_pr_dbg("----------------------------------------\n"); */
	sqn_pr_dbg("address: 0x%02X access_size: %u data_size: %u\n"
			, mcpy_tag->address, mcpy_tag->access_size
			, mcpy_tag->data_size);
	/* sqn_pr_dbg_dump("|", mcpy_tag->data, mcpy_tag->data_size); */

	rv = write_data(func, mcpy_tag->address, mcpy_tag->data
		, mcpy_tag->data_size, mcpy_tag->access_size);

	sqn_pr_leave();
	return rv;
}


static int sqn_handle_memset_tag(struct sdio_func *func
	, struct sqn_tag_memset * mset_tag)
{
	int rv = 0;
	u8 *buf = 0;
	const u32 buf_size = 1024;
	u32 left_bytes = 0;

	sqn_pr_enter();

	/*
	 * Convert values accordingly to platform "endianes"
	 * (big or little endian) because bootstrapper file
	 * data is big endian
	 */
	mset_tag->address = be32_to_cpu(mset_tag->address);
	mset_tag->access_size = be32_to_cpu(mset_tag->access_size);
	mset_tag->size = be32_to_cpu(mset_tag->size);

	/* sqn_pr_dbg("----------------------------------------\n"); */
	sqn_pr_dbg("address: 0x%02X access_size: %u size: %u pattern 0x%02X\n"
			, mset_tag->address, mset_tag->access_size
			, mset_tag->size, mset_tag->pattern);

	buf = kmalloc(buf_size, GFP_KERNEL);
	if (0 == buf)
		return -ENOMEM;

	memset(buf, mset_tag->pattern, buf_size);

	left_bytes = mset_tag->size;

	while (left_bytes) {
		u32 bytes_to_write = min(buf_size, left_bytes);
		rv = write_data(func, mset_tag->address, buf, bytes_to_write,
			mset_tag->access_size);
		if (rv)
			goto out;
		left_bytes -= bytes_to_write;
	}

out:
	kfree(buf);
	sqn_pr_leave();
	return rv;
}


static int char_to_int(u8 c)
{
	int rv = 0;

	if ('0' <= c && c <= '9') {
		rv = c - '0';
	} else if ('a' <= c && c <= 'f') {
		rv = c - 'a' + 0xA;
	} else if ('A' <= c && c <= 'F') {
		rv = c - 'A' + 0xA;
	} else {
		rv = -1;
	}

	return rv;
}


static int get_mac_addr_from_str(u8 *data, u32 length, u8 *result)
{
	int rv = 0;
	int i = 0;

	sqn_pr_enter();

	if (0 == length) {
		rv = -1;
		goto out;
	}

	/*
	 * Check if we have delimiters on appropriate places:
	 *
	 * X X : X X : X X : X X  :  X  X  :  X  X
	 * 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
	 */

	if ( !( ( ':' == data[2] || '-' == data[2])
		&& ( ':' == data[5] || '-' == data[5])
		&& ( ':' == data[8] || '-' == data[8])
		&& ( ':' == data[11] || '-' == data[11])
		&& ( ':' == data[14] || '-' == data[14]) ))
	{
		sqn_pr_err("can't get mac address from firmware"
			" - incorrect mac address\n");
		rv = -1;
		goto out;
	}

	i = 0;
	while (i < length) {
		int high = 0;
		int low = 0;

		if ((high = char_to_int(data[i])) >= 0
			&& (low = char_to_int(data[i + 1])) >= 0)
		{
			result[i/3] = low;
			result[i/3] |= high << 4;
		} else {
			sqn_pr_err("can't get mac address from firmware"
					" - incorrect mac address\n");
			rv = -1;
			goto out;
		}

		i += 3;
	}

out:
	if (length > 0) {
		data[length - 1] = 0;
		sqn_pr_dbg("mac addr string: %s\n", data);
	}
	sqn_pr_leave();
	return rv;
}


static int sqn_handle_mac_addr_tag(struct sdio_func *func, u8 *data, u32 length)
{
	int rv = 0;
	struct sqn_private *priv =
		((struct sqn_sdio_card *)sdio_get_drvdata(func))->priv;

	sqn_pr_enter();

	/*
	 * This tag could contain one or two mac addresses in string
	 * form, delimited by some symbol (space or something else).
	 * Each mac address written as a string has constant length.
	 * Thus we can determine the number of mac addresses by the
	 * length of the tag:
	 *
	 * mac addr length in string form: XX:XX:XX:XX:XX:XX = 17 bytes
	 * tag length: 17 bytes [ + 1 byte + 17 bytes ]
	 */

#define MAC_ADDR_STRING_LEN	17

	/*
	 * If we have only one mac addr we should increment it by one
	 * and use it.
	 * If we have two mac addresses we should use a second one.
	 */

	if (MAC_ADDR_STRING_LEN <= length
		&& length < 2 * MAC_ADDR_STRING_LEN + 1)
	{
		sqn_pr_dbg("single mac address\n");
		/* we have only one mac addr */
		get_mac_addr_from_str(data, length, priv->mac_addr);

		// Andrew 0720
		// ++(priv->mac_addr[ETH_ALEN - 1])
		// real MAC: 38:E6:D8:86:00:00 
		// hboot will store: 38:E6:D8:85:FF:FF (minus 1)
		// sdio need to recovery it by plusing 1: 38:E6:D8:86:00:00 (plus 1)

		if ((++(priv->mac_addr[ETH_ALEN - 1])) == 0x00)
			if ((++(priv->mac_addr[ETH_ALEN - 2])) == 0x00)
				if ((++(priv->mac_addr[ETH_ALEN - 3])) == 0x00)
					if ((++(priv->mac_addr[ETH_ALEN - 4])) == 0x00)
						if ((++(priv->mac_addr[ETH_ALEN - 5])) == 0x00)
							++(priv->mac_addr[ETH_ALEN - 6]);

	}
	else if (2 * MAC_ADDR_STRING_LEN + 1 == length) { /* we have two macs */
		sqn_pr_dbg("two mac addresses, using second\n");
		get_mac_addr_from_str(data + MAC_ADDR_STRING_LEN + 1
			, length - (MAC_ADDR_STRING_LEN + 1), priv->mac_addr);
	}
	else { /* incorrect data length */
		sqn_pr_err("can't get mac address from bootloader"
			" - incorrect mac address length\n");
		rv = -1;
		goto out;
	}

	sqn_pr_info("setting MAC address from bootloader: "
		"%02x:%02x:%02x:%02x:%02x:%02x\n", priv->mac_addr[0]
		, priv->mac_addr[1], priv->mac_addr[2], priv->mac_addr[3]
		, priv->mac_addr[4], priv->mac_addr[5]);

out:
	sqn_pr_leave();
	return rv;
}


/** sqn_load_bootstrapper - reads a binary boostrapper file, analize it
 *  and loads data to the card.
 *
 *  Bootstrapper is consists of Tag, Length, Value (TLV) sections.
 *  Each section starts with 4 bytes tag. Then goes length of data (4 bytes)
 *  and then the data itself.
 *
 *  All fields of bootstrapper file is in BIG ENDIAN format.
 */
static int sqn_load_bootstrapper(struct sdio_func *func, u8 *data, int size)
{
	struct sqn_tlv *tlv = (struct sqn_tlv*) data;
	int rv = 0;

	sqn_pr_enter();

	while (size > 0) {
		/*
		 * Convert values accordingly to platform "endianes"
		 * (big or little endian) because bootstrapper file
		 * data is big endian
		 */
		tlv->tag = be32_to_cpu(tlv->tag);
		tlv->length = be32_to_cpu(tlv->length);

		switch (tlv->tag) {
		case SWM_INFO_TAG_SQN_ROOT:
		case SWM_INFO_TAG_SQN_BOOTROM_GROUP:
		case SWM_INFO_TAG_SQN_ID_GROUP:
			/*
			 * This tag is a "container" tag - it's value field
			 * contains other tags
			 */

			/* sqn_pr_dbg("========================================\n"); */
			sqn_pr_dbg("tag: CONTAINER %x length: %u\n", tlv->tag
				, tlv->length);
			/* sqn_pr_dbg_dump("|", tlv->value, tlv->length); */

			/*
			 * If this is a buggy tag, adjust length to
			 * the rest of data
			 */
			if (0 == tlv->length)
				tlv->length = size - sizeof(*tlv);

			rv = sqn_load_bootstrapper(func, (u8*) tlv->value
				, tlv->length);
			if (rv)
				goto out;
			break;

		case SWM_INFO_TAG_SQN_MEMCPY:
			/* sqn_pr_dbg("========================================\n"); */
			sqn_pr_dbg("tag: SWM_INFO_TAG_SQN_MEMCPY length: %u\n"
					, tlv->length);
			/* sqn_pr_dbg_dump("|", tlv->value, tlv->length); */
			rv = sqn_handle_memcpy_tag(func
				, (struct sqn_tag_memcpy*) tlv->value);
			if (rv)
				goto out;
			break;

		case SWM_INFO_TAG_SQN_MEMSET:
			/* sqn_pr_dbg("========================================\n"); */
			sqn_pr_dbg("tag: SWM_INFO_TAG_SQN_MEMSET length: %u\n"
					, tlv->length);
			/* sqn_pr_dbg_dump("|", tlv->value, tlv->length); */
			rv = sqn_handle_memset_tag(func
				, (struct sqn_tag_memset*) tlv->value);
			if (rv)
				goto out;
			break;

		case SWM_INFO_TAG_SQN_MAC_ADDRESS:
			/* sqn_pr_dbg("========================================\n"); */
			sqn_pr_dbg("tag: SWM_INFO_TAG_SQN_MAC_ADDRESS length: %u\n"
					, tlv->length);
			/* sqn_pr_dbg_dump("|", tlv->value, tlv->length); */

			rv = sqn_handle_mac_addr_tag(func, tlv->value
				, tlv->length);
			if (rv)
				goto out;
			break;

		default:
			/* skip all other tags */
			/* sqn_pr_dbg("========================================\n"); */
			sqn_pr_dbg("tag: UNKNOWN %x length: %u\n"
					, tlv->tag, tlv->length);
			/* sqn_pr_dbg_dump("|", tlv->value, tlv->length); */
			break;
		}

		/* increment tlv to point it to the beginning of the next
		 * sqn_tlv struct and decrement size accordingly
		 */
		size = (int)(size - (sizeof(*tlv) + tlv->length));
		tlv = (struct sqn_tlv*) ((u8*)tlv + sizeof(*tlv) + tlv->length);
	}

	if (0 != size) {
		/* something wrong with parsing of tlv values */
		rv = -1;
		goto out;
	}

out:
	sqn_pr_leave();
	return rv;
}


extern char *firmware_name;

/** sqn_load_firmware - loads firmware to card
 *  @func: SDIO function, used to transfer data via SDIO interface,
 *         also used to obtain pointer to device structure.
 *
 *  But now the only work it does - is loading of bootstrapper to card,
 *  because firmware is supposed to be loaded by a userspace program.
 */
int sqn_load_firmware(struct sdio_func *func)
{
	int rv = 0;
	const struct firmware *fw = 0;
//Create a local firmware_name with path to replace original global firmware_name -- Tony Wu.
	const char *firmware_name = "../../../data/wimax/Boot.bin";

	struct sqn_sdio_card *sqn_card = sdio_get_drvdata(func);

	sqn_pr_enter();

	sqn_pr_info("trying to find bootloader image: \"%s\"\n", firmware_name);
	if ((rv = request_firmware(&fw, firmware_name, &func->dev)))
		goto out;

	if (SQN_1130 == sqn_card->version) {
		sdio_claim_host(func);

		/* properly setup registers for firmware loading */
		sqn_pr_dbg("setting up SQN_H_SDRAM_NO_EMR register\n");
		sdio_writeb(func, 0, SQN_H_SDRAM_NO_EMR, &rv);
		if (rv) {
			sdio_release_host(func);
			goto out;
		}

		sqn_pr_dbg("setting up SQN_H_SDRAMCTL_RSTN register\n");
		sdio_writeb(func, 1, SQN_H_SDRAMCTL_RSTN, &rv);
		sdio_release_host(func);
		if (rv)
			goto out;
	}

	sqn_pr_info("loading bootloader to the card...\n");
	if ((rv = sqn_load_bootstrapper(func, (u8*) fw->data, fw->size)))
		goto out;

	/* boot the card */
	sqn_pr_info("bootting the card...\n");
	sdio_claim_host(func); // by daniel
	sdio_writeb(func, 1, SQN_H_CRSTN, &rv);
	sdio_release_host(func); // by daniel
	if (rv)
		goto out;
	sqn_pr_info("  done\n");

out:
	// To avoid kzalloc leakage in /drivers/base/firmware_class.c	
	if (fw) {
		release_firmware(fw);
		fw = NULL;
	}

	sqn_pr_leave();
	return rv;
}
#endif
