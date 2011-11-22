/*
 *	Abstract:
 *		Debug Flags for SDIO_AL
 *
 *	Working environment:
 *		Android/LTE 8x60 projects
 *
 *	Referenced documents:
 *		N/A
 *
 *	Revision history:
 *		Trial12MAY2011 --Bert Lin--
 */
#ifndef __SDIODBG_8X60_H__
#define __SDIODBG_8X60_H__

#include <mach/board.h>

/* external module flags */
#define DBG_DMUX BIT31
#define DBG_RMNET BIT30
#define DBG_CMUX BIT29
#define DBG_CTL BIT28
#define DBG_RPC BIT27
#define DBG_MDM BIT26

/* inter-module flags */
#define DBG_LAWDATA BIT5
#define DBG_MEMCPY BIT4
#define DBG_ALDEBUG BIT3
#define DBG_DATA BIT2
#define DBG_LPM BIT1

#endif /* __SDIODBG_8X60_H__ */
