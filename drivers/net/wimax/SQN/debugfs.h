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

#ifndef _SQN_DEBUGFS_H
#define _SQN_DEBUGFS_H

#include <linux/types.h>


struct sqn_dfs_module_flags {
	u8	all;
	u8	main;
	u8	sdio;
	u8	thp;
	u8	fw;
	u8	pm;
};


struct sqn_dfs_feature_flags {
	u8	all;
	u8	dump_all_pkts;
	u8	dump_tx_pkt;
	u8	dump_rx_pkt;
	u8	dump_lsp_pkt;
	u8	dump_thp_pkt;
	u8	verbose_thp_hdr;
	u8	verbose_lsp;
	u8	wake_lock;
	u8	trace_funcs;
};


struct sqn_dfs {
	struct sqn_dfs_module_flags	mf;
	struct sqn_dfs_feature_flags	ff;
};

extern struct sqn_dfs	sqn_dfs;

struct sqn_dfs_perf_stat_data {
	u32	count;
	u32	total_time;
};

struct sqn_dfs_perf_stat {
#define SQN_DFS_PERF_STAT_SIZE     2000
       struct sqn_dfs_perf_stat_data   rx[SQN_DFS_PERF_STAT_SIZE];
       struct sqn_dfs_perf_stat_data   tx[SQN_DFS_PERF_STAT_SIZE];
};

extern struct sqn_dfs_perf_stat sqn_dfs_pstat;


void sqn_dfs_init(void);
void sqn_dfs_cleanup(void);


#endif /* _SQN_DEBUGFS_H */
