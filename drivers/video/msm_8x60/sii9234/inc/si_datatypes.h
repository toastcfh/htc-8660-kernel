//***************************************************************************
//!file     si_datatypes.h
//!brief    Silicon Image data type header (conforms to C99).
//
// No part of this work may be reproduced, modified, distributed,
// transmitted, transcribed, or translated into any language or computer
// format, in any form or by any means without written permission of
// Silicon Image, Inc., 1060 East Arques Avenue, Sunnyvale, California 94085
//
// Copyright 2008-2009, Silicon Image, Inc.  All rights reserved.
//***************************************************************************/

#ifndef __SI_DATATYPES_H__
#define __SI_DATATYPES_H__

#define ROM     static        // 8051 type of ROM memory
#define XDATA   //xdata       // 8051 type of external memory

//------------------------------------------------------------------------------
// Configuration defines used by hal_config.h
//------------------------------------------------------------------------------

/* HTC: these definitions are not referenced at all and conflicts with <mach/board.h>
#define BIT0                    0x01
#define BIT1                    0x02
#define BIT2                    0x04
#define BIT3                    0x08
#define BIT4                    0x10
#define BIT5                    0x20
#define BIT6                    0x40
#define BIT7                    0x80
#endif
*/

#define MSG_ALWAYS              0x00
#define MSG_STAT                0x01
#define MSG_DBG                 0x02
#define DEBUG_PRINT(l,x)      if (l<=0) printf x
#if 0
#define CI2CA_HIGH	0
#if machine_is_shooter()
#define cbus_slave_addr 0xCC//0xC8
#else
#define CBUS_SLAVE_ADDR 0xC8
#endif
#else
//extern int CBUS_SLAVE_ADDR;
#define	CBUS_SLAVE_ADDR	0xC8
#endif

#define SET_BITS    0xFF
#define CLEAR_BITS  0x00

#endif  // __SI_DATATYPES_H__

