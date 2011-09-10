/*
 * This is part of the Sequans SQN1130 driver.
 * Copyright 2010 SEQUANS Communications
 * Written by Dmitriy Chumak <chumakd@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */


#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include "version.h"
#include "debugfs.h"
#include "msg.h"
#include "sdio.h"
#include "sdio-sqn.h"

static u8 __sqn_per_file_dbg = 0;


struct sqn_dfs		sqn_dfs = { { 0 } };
struct sqn_dfs_perf_stat sqn_dfs_pstat = { { { 0 } } };
static struct dentry	*sqn_dfs_rootdir;

extern int debug;

static void sqn_dfs_init_struct(void)
{
	sqn_pr_enter();

#ifdef DEBUG
	sqn_dfs.mf.all = 1;
	sqn_dfs.ff.all = 1;
#else
	if (debug) {
		sqn_dfs.mf.all = 1;
		sqn_dfs.ff.all = 1;
	}
#endif

#ifdef SQN_DEBUG_TRACE_FUNCS
	sqn_dfs.ff.trace_funcs = 1;
#endif

	sqn_pr_leave();
}


static void *sqn_dfs_perf_rx_seq_start(struct seq_file *seq, loff_t *pos)
{
	u32 *idx = 0;

	sqn_pr_enter();

	if (0 == *pos) {
		sqn_pr_dbg("zero pos\n");
		idx = SEQ_START_TOKEN;
		goto out;
	} else if (SQN_DFS_PERF_STAT_SIZE <= *pos) {
		/* indicate beyond end of file position */
		sqn_pr_dbg("beyond end of file position %llu\n", *pos);
		idx = 0;
		goto out;
	}

	idx = kmalloc(sizeof(u32), GFP_KERNEL);

	if (!idx) {
		sqn_pr_dbg("failed to alloc seq_file iterator\n");
		goto out;
	}

	*idx = *pos;
	sqn_pr_dbg("start pos %u\n", *idx);
out:
	sqn_pr_leave();
	return idx;
}


static void *sqn_dfs_perf_rx_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	u32 *idx = 0;

	sqn_pr_enter();

	++(*pos);

	if (SEQ_START_TOKEN == v) {
		idx = kmalloc(sizeof(u32), GFP_KERNEL);
		if (!idx) {
			sqn_pr_dbg("failed to alloc seq_file iterator\n");
			goto out;
		}
	} else {
		idx = v;
	}

	*idx = *pos - 1;
	sqn_pr_dbg("idx %u, pos %llu\n", *idx, *pos);

	if (*idx >= SQN_DFS_PERF_STAT_SIZE) {
		/* indicate end of sequence */
		sqn_pr_dbg("end of sequence, idx %u\n", *idx);
		idx = 0;
	}
out:
	sqn_pr_leave();
	return idx;
}


static void sqn_dfs_perf_rx_seq_stop(struct seq_file *seq, void *v)
{
	sqn_pr_enter();
	kfree(v);
	sqn_pr_leave();
}


static int sqn_dfs_perf_rx_seq_show(struct seq_file *seq, void *v)
{
	u32 *idx = 0;
	u32 speed = 0;
	u32 time_in_sec = 0;
	int rv = 0;

	sqn_pr_enter();

	if (SEQ_START_TOKEN == v) {
		/* print header */
		seq_puts(seq, " Pkt size  | Pkt count | Total time | Speed (Mbit/s)\n");
		seq_puts(seq, "-----------+-----------+------------+---------------\n");
		goto out;
	}

	idx = v;

	/* skip empty counters */
	if (0 == sqn_dfs_pstat.rx[*idx].count) {
		rv = SEQ_SKIP;
		goto out;
	}

	/* if total_time greater then 1 second we can calculate speed,
	 * *idx contains paket size */
	// #define USEC_PER_SEC	1000000L
	time_in_sec = sqn_dfs_pstat.rx[*idx].total_time / USEC_PER_SEC;
	if (time_in_sec) {	
		/* calculate speed in several steps to avoid overflow */
		u32 mbytes = (*idx * (sqn_dfs_pstat.rx[*idx].count / 1024)) / 1024;
		speed =  (mbytes * BITS_PER_BYTE) / time_in_sec;
	}
		
	seq_printf(seq, " %6u      %9u   %10u       %6u\n"
			, *idx, sqn_dfs_pstat.rx[*idx].count
			, sqn_dfs_pstat.rx[*idx].total_time, speed);
out:
	sqn_pr_leave();
	return rv;
}


static const struct seq_operations sqn_dfs_perf_rx_seq_ops = {
	.start = sqn_dfs_perf_rx_seq_start,
	.next  = sqn_dfs_perf_rx_seq_next,
	.stop  = sqn_dfs_perf_rx_seq_stop,
	.show  = sqn_dfs_perf_rx_seq_show,
};


static int sqn_dfs_perf_rx_open(struct inode *i, struct file *f)
{
	int rv = 0;

	sqn_pr_enter();

	rv = seq_open(f, &sqn_dfs_perf_rx_seq_ops);

	sqn_pr_leave();

	return rv;
}

static int sqn_dfs_fetch_rx_open(struct inode *i, struct file *f)
{
	f->private_data = i->i_private;
	return 0;
}

struct sqn_sdio_card *_g_sqn_sdio_card = 0;

int sqn_sdio_it_lsb(struct sdio_func *func);

static ssize_t sqn_dfs_fetch_rx_read(struct file *file, char __user *ubuf,
		       size_t count, loff_t *ppos)
{
	char buf[1024];
	int max = sizeof(buf) - 1;
	int i = 0;

	sqn_pr_info("fetching RX packets\n");

	sdio_claim_host(_g_sqn_sdio_card->func);
	sqn_sdio_it_lsb(_g_sqn_sdio_card->func);
	sdio_release_host(_g_sqn_sdio_card->func); 

	i += scnprintf(buf + i, max - i, "RX packets fetched\n");

	return simple_read_from_buffer(ubuf, count, ppos, buf, i);
}

static const struct file_operations sqn_perf_rx_fops = {
	.owner		= THIS_MODULE,
	.open		= sqn_dfs_perf_rx_open,
	.read		= seq_read,
	/* .write		= kmemleak_write, */
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static const struct file_operations sqn_fetch_rx_fops = {
	.owner		= THIS_MODULE,
	.open		= sqn_dfs_fetch_rx_open,
	.read		= sqn_dfs_fetch_rx_read,
};

u8* __sqn_sdio_per_file_dbg_addr(void);
u8* __sqn_main_per_file_dbg_addr(void);
u8* __sqn_fw_per_file_dbg_addr(void);
u8* __sqn_pm_per_file_dbg_addr(void);
u8* __sqn_thp_per_file_dbg_addr(void);

void sqn_dfs_init(void)
{
	struct dentry	*debug_dir	= 0;
	char		*debug_dir_name	= "dbg";

	struct dentry	*config_dir	= 0;
	char		*config_dir_name= "cfg";

	struct dentry	*module_dir	= 0;
	char		*module_dir_name= "modules";

	struct dentry	*feature_dir	= 0;
	char		*feature_dir_name= "features";
	struct dentry	*perf_rx_file	= 0;
	char		*perf_rx_file_name = "raw_speed_rx";

	char *fetch_rx_file_name = "fetch_rx_packets";

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)

	sqn_dfs_init_struct();

	sqn_dfs_rootdir = debugfs_create_dir(SQN_MODULE_NAME, NULL);
	if (!sqn_dfs_rootdir) {
		sqn_pr_err("Can't create debugfs entry '%s'\n", SQN_MODULE_NAME);
		goto out;
	}

	debug_dir = debugfs_create_dir(debug_dir_name, sqn_dfs_rootdir);
	if (!debug_dir) {
		sqn_pr_err("Can't create debugfs entry '%s'\n", debug_dir_name);
		goto out;
	}

	config_dir = debugfs_create_dir(config_dir_name, sqn_dfs_rootdir);
	if (!config_dir) {
		sqn_pr_err("Can't create debugfs entry '%s'\n", config_dir_name);
		goto out;
	}

	module_dir = debugfs_create_dir(module_dir_name, debug_dir);
	if (!module_dir) {
		sqn_pr_err("Can't create debugfs entry '%s'\n", module_dir_name);
		goto out;
	}

	feature_dir = debugfs_create_dir(feature_dir_name, debug_dir);
	if (!feature_dir) {
		sqn_pr_err("Can't create debugfs entry '%s'\n", feature_dir_name);
		goto out;
	}

#define sqn_dfs_add_u8(parent, name, value)			\
({								\
	struct dentry *p = (parent);				\
	const char *n = (name);					\
	u8 *v = (value);					\
	struct dentry *d = 0;					\
	d = debugfs_create_u8(n, 0600, p, v);			\
	if (!d) {						\
		sqn_pr_err("Can't create debugfs entry '%s'\n"	\
			, name);				\
		goto out;					\
	}							\
})

	sqn_dfs_add_u8(module_dir, "all", &sqn_dfs.mf.all);
	sqn_dfs_add_u8(module_dir, "thp", __sqn_thp_per_file_dbg_addr());
	sqn_dfs_add_u8(module_dir, "main", __sqn_main_per_file_dbg_addr());
	sqn_dfs_add_u8(module_dir, "fw", __sqn_fw_per_file_dbg_addr());
	sqn_dfs_add_u8(module_dir, "pm", __sqn_pm_per_file_dbg_addr());
	sqn_dfs_add_u8(module_dir, "sdio", __sqn_sdio_per_file_dbg_addr());

	sqn_dfs_add_u8(feature_dir, "all", &sqn_dfs.ff.all);
	sqn_dfs_add_u8(feature_dir, "dump_all_pkts", &sqn_dfs.ff.dump_all_pkts);
	sqn_dfs_add_u8(feature_dir, "dump_tx_pkt", &sqn_dfs.ff.dump_tx_pkt);
	sqn_dfs_add_u8(feature_dir, "dump_rx_pkt", &sqn_dfs.ff.dump_rx_pkt);
	sqn_dfs_add_u8(feature_dir, "dump_lsp_pkt", &sqn_dfs.ff.dump_lsp_pkt);
	sqn_dfs_add_u8(feature_dir, "dump_thp_pkt", &sqn_dfs.ff.dump_thp_pkt);
	sqn_dfs_add_u8(feature_dir, "verbose_thp_hdr", &sqn_dfs.ff.verbose_thp_hdr);
	sqn_dfs_add_u8(feature_dir, "verbose_lsp", &sqn_dfs.ff.verbose_lsp);
	sqn_dfs_add_u8(feature_dir, "wake_lock", &sqn_dfs.ff.wake_lock);
	sqn_dfs_add_u8(feature_dir, "trace_funcs", &sqn_dfs.ff.trace_funcs);

	{
		struct dentry *dentry;
		dentry = debugfs_create_file(perf_rx_file_name, S_IRUGO, sqn_dfs_rootdir, 0, &sqn_perf_rx_fops);
		if (!dentry)
			sqn_pr_err("failed to create the debugfs %s file\n", perf_rx_file_name);

		dentry = debugfs_create_file(fetch_rx_file_name, S_IRUGO, sqn_dfs_rootdir, 0, &sqn_fetch_rx_fops);
		if (!dentry)
			sqn_pr_err("failed to create the debugfs %s file\n", fetch_rx_file_name);
	}

#undef sqn_dfs_add_u8

#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27) */

out:
	sqn_pr_leave();
	return;
}


void sqn_dfs_cleanup(void)
{
	sqn_pr_enter();

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	debugfs_remove_recursive(sqn_dfs_rootdir);
#endif
	sqn_dfs_rootdir = 0;

	sqn_pr_leave();
}
