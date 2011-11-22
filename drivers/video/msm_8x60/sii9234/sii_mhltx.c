/***************************************************************************
 *
 *   SiI9234 - MHL Transmitter Driver
 *
 * Copyright (C) 2011 SiliconImage, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *****************************************************************************/

#include <linux/input.h>
#include <mach/board.h>
#include	"defs.h"
#include	"sii_mhltx_api.h"
#include	"sii_mhltx.h"
#include	"mhl_defs.h"

#include "TPI_Access.h"
#include "TPI.h"
#include "i2c_master_sw.h"
#include "inc/si_datatypes.h"

static	mhlTx_config_t	mhlTxConfig;

static	bool 		SiiMhlTxRapkSend( void );
static	void		MhlTxDriveStates( void );
static	void		MhlTxResetStates( void );
static	bool		MhlTxSendMscMsg ( uint8_t command, uint8_t cmdData );
extern	uint8_t		rcpSupportTable [];

static uint8_t Chk_Dongle_Step;

void SiiMhlTxInitialize( bool interruptDriven, uint8_t pollIntervalMs )
{
	TPI_DEBUG_PRINT( ("MhlTx: SiiMhlTxInitialize\n") );
	mhlTxConfig.interruptDriven = interruptDriven;
	mhlTxConfig.pollIntervalMs  = pollIntervalMs;

	MhlTxResetStates( );

	Chk_Dongle_Step=0;

//	SiiMhlTxChipInitialize ();
}

void SiiMhlTxGetEvents( uint8_t *event, uint8_t *eventParameter )
{
	TPI_Poll();
	MhlTxDriveStates( );

	*event = MHL_TX_EVENT_NONE;
	*eventParameter = 0;

	if( mhlTxConfig.mhlConnectionEvent )
	{
		TPI_DEBUG_PRINT( ("MhlTx: SiiMhlTxGetEvents mhlConnectionEvent\n") );

		mhlTxConfig.mhlConnectionEvent = false;
		*event          = mhlTxConfig.mhlConnected;
		*eventParameter	= mhlTxConfig.mscFeatureFlag;

		if(MHL_TX_EVENT_DISCONNECTION == mhlTxConfig.mhlConnected)
		{
			MhlTxResetStates( );
		}
	}
	else if( mhlTxConfig.mscMsgArrived )
	{
		TPI_DEBUG_PRINT( ("MhlTx: SiiMhlTxGetEvents MSC MSG <%02X, %02X>\n",
							(int) ( mhlTxConfig.mscMsgSubCommand ),
							(int) ( mhlTxConfig.mscMsgData )) );

		mhlTxConfig.mscMsgArrived = false;

		switch( mhlTxConfig.mscMsgSubCommand )
		{
			case	MHL_MSC_MSG_RAP:
				if( 0x10 == mhlTxConfig.mscMsgData)
				{
					SiiMhlTxDrvTmdsControl( true );
				}
				else if( 0x11 == mhlTxConfig.mscMsgData)
				{
					SiiMhlTxDrvTmdsControl( false );

				}
				SiiMhlTxRapkSend( );
				break;

			case	MHL_MSC_MSG_RCP:
				if((0x01 << 7) & rcpSupportTable [mhlTxConfig.mscMsgData & 0x7F] )
				{
					*event          = MHL_TX_EVENT_RCP_RECEIVED;
					*eventParameter = mhlTxConfig.mscMsgData;
				}
				else
				{
					mhlTxConfig.mscSaveRcpKeyCode = mhlTxConfig.mscMsgData;
					SiiMhlTxRcpeSend( RCPE_INEEFECTIVE_KEY_CODE );
				}
				break;

			case	MHL_MSC_MSG_RCPK:
				*event = MHL_TX_EVENT_RCPK_RECEIVED;
				*eventParameter = mhlTxConfig.mscMsgData;
				break;

			case	MHL_MSC_MSG_RCPE:
				*event = MHL_TX_EVENT_RCPE_RECEIVED;
				*eventParameter = mhlTxConfig.mscMsgData;
				break;

			case	MHL_MSC_MSG_RAPK:
				break;

			default:
				break;
		}
	}
}

static	void	MhlTxDriveStates( void )
{

	switch( mhlTxConfig.mscState )
	{
		case MSC_STATE_BEGIN:
			SiiMhlTxReadDevcap( 0x02 );
			break;
		case MSC_STATE_POW_DONE:
			SiiMhlTxReadDevcap( 0x0A );
			break;
		case MSC_STATE_IDLE:
		case MSC_STATE_RCP_READY:
			break;
		default:
			break;

	}
}

#ifdef CONFIG_CABLE_DETECT_ACCESSORY
extern void ProcessMhlStatus(bool, bool);
extern int mscCmdInProgress;
extern enum usb_connect_type gStatusMHL;

bool Tri_state_dongle_GPIO0(void)
{
	bool result = true;

#if 1

#define	INTR_CBUS1_DESIRED_MASK			(BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6)
#define	UNMASK_CBUS1_INTERRUPTS			I2C_WriteByte(CBUS_SLAVE_ADDR, 0x09, INTR_CBUS1_DESIRED_MASK)
#define	MASK_CBUS1_INTERRUPTS			I2C_WriteByte(CBUS_SLAVE_ADDR, 0x09, 0x00)
#define	INTR_CBUS2_DESIRED_MASK			(BIT_2 | BIT_3 | BIT_4)
#define	UNMASK_CBUS2_INTERRUPTS			I2C_WriteByte(CBUS_SLAVE_ADDR, 0x1F, INTR_CBUS2_DESIRED_MASK)
#define	MASK_CBUS2_INTERRUPTS			I2C_WriteByte(CBUS_SLAVE_ADDR, 0x1F, 0x00)

	int timeout = 100;

	MASK_CBUS1_INTERRUPTS;
	MASK_CBUS2_INTERRUPTS;


	//don't activate this function before problem solved
	return result;


	while(mscCmdInProgress && --timeout) {
		hr_msleep(1);
	}

	printk("%s: timeout = %d\n", __func__, timeout);

	if(!timeout) {
		result = false;
		goto l_end;
	}
#endif
	I2C_WriteByte(CBUS_SLAVE_ADDR,0x13, 0x33);       // enable backdoor access
	I2C_WriteByte(CBUS_SLAVE_ADDR,0x14, 0x80);
	I2C_WriteByte(CBUS_SLAVE_ADDR,0x12, 0x08);

	// Set GPIO0=Input
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0xc0,  0xff);          // main page  ; FE for Cbus page
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0xc1,  0x7F);        // offset
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0xc2,  0xFF);       // data set GPIO=input
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x20,  0x02);         // burst length-1
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x13,  0x48);         // offset in scratch pad
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x12,  0x10);        // trig this command

	I2C_WriteByte(CBUS_SLAVE_ADDR,0x13, 0x33);           // disable backdoor access
	I2C_WriteByte(CBUS_SLAVE_ADDR,0x14, 0x00);
	I2C_WriteByte(CBUS_SLAVE_ADDR,0x12, 0x08);

#if 1
l_end:
	UNMASK_CBUS2_INTERRUPTS;
	UNMASK_CBUS1_INTERRUPTS;
#endif

	return result;
}

void Low_dongle_GPIO0(void)
{

	I2C_WriteByte(CBUS_SLAVE_ADDR,0x13, 0x33);                               // enable backdoor access
	I2C_WriteByte(CBUS_SLAVE_ADDR,0x14, 0x80);
	I2C_WriteByte(CBUS_SLAVE_ADDR,0x12, 0x08);

	// Set GPIO0=low
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0xc0,  0xff);                              // main page  ; FE for Cbus page
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0xc1,  0x7F);                             // offset
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0xc2,  0xFC);                             // data set GPIO0=Lo ; 0xF3 set GPIO1=Lo
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x20,  0x02);                             // burst length-1
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x13,  0x48);                             // offset in scratch pad
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x12,  0x10);                             // trig this command

	I2C_WriteByte(CBUS_SLAVE_ADDR,0x13, 0x33);                               // disable backdoor access
	I2C_WriteByte(CBUS_SLAVE_ADDR,0x14, 0x00);
	I2C_WriteByte(CBUS_SLAVE_ADDR,0x12, 0x08);
}

void SiiMhlTxMscDetectCharger( uint8_t data1)
{
	if ((data1 & 0x13) ==0x11) { 							 // connected to TV; and TV has power output (default 0.5A min )
		/* Turn off phone Vbus output ; */
		/* Set battery charge current=500mA; */
		/* Enable battery charger; */
		mscCmdInProgress = false;
		mhlTxConfig.mscState	  = MSC_STATE_POW_DONE;
	}

	if ((data1 & 0x03) ==0x03 ) {			 //03=dongle

		if ( Chk_Dongle_Step==0 ) {

			if( ! Tri_state_dongle_GPIO0() ) {
				printk("%s: faild to set as GPI\n", __func__);
				mscCmdInProgress = false;
				return;
			}

			/* GPIO0_state=3; */

			mscCmdInProgress = false;
			SiiMhlTxReadDevcap( 02 );			  // send ReadDevCapReg0x02 packet out; POW will be returned on next MhlCbusIsr( ) 
			mscCmdInProgress = false;
			Chk_Dongle_Step=1;
		}

		if ( Chk_Dongle_Step==1) {

			mscCmdInProgress = false;

			if ((data1 & 0x10)) {			 // POW bit=1=Rx has power
				/* Turn off phone Vbus output ; */
				Low_dongle_GPIO0( );

				/* GPIO0_state = 0;  */
				SiiMhlTxReadDevcap( 02 );			  // send ReadDevCapReg0x02 packet out; POW will be returned on next MhlCbusIsr( ) 
				mscCmdInProgress = false;
				Chk_Dongle_Step=2;
			} else {

//				Chk_Dongle_Step=3;
				Chk_Dongle_Step=0;

				mhlTxConfig.mscState= MSC_STATE_POW_DONE;

				/* turn on phone VBUS output.; */
				TPI_DEBUG_PRINT(("No charger!!\n"));

				if(gStatusMHL != CONNECT_TYPE_NONE) {
					gStatusMHL = CONNECT_TYPE_NONE;
					ProcessMhlStatus(true, false);
				}
				//system should periodically call siiMhlTxReadDevcap(02), next siiMhlTxGetEvents( )  MhlCbusIsr( ) will on/off Vbus, set charge current here
			}
		}

		if (Chk_Dongle_Step==2) {			   //GPIO0_low=true

			mhlTxConfig.mscState= MSC_STATE_POW_DONE;				//02 ;
			mscCmdInProgress = false;

//			Chk_Dongle_Step=3;
			Chk_Dongle_Step=0;

			if (data1 & 0x10) { 			 //[bit4] POW ==1=AC charger attached
				/* Set charge battery current=AC charger rating-100mA ; */

				/* Enable battery charger; &*/
				TPI_DEBUG_PRINT(("1000mA charger!!\n"));
				if(gStatusMHL != CONNECT_TYPE_AC) {
					gStatusMHL = CONNECT_TYPE_AC;
					ProcessMhlStatus(true, false);
				}

			} else {	// charger port only has USB source provide 5V/100mA , that just enough dongle to work, no more current to charge phone battery
				/* turn off phone VBUS output; */		   // at least no need to send out 5V/100mA power 
				TPI_DEBUG_PRINT(("500mA charger!!\n"));
				if(gStatusMHL != CONNECT_TYPE_USB) {
					gStatusMHL = CONNECT_TYPE_USB;
					ProcessMhlStatus(true, false);
				}
			}
		}
	}
}
#endif
void	SiiMhlTxMscCommandDone( uint8_t data1 )
{
	TPI_DEBUG_PRINT( ("MhlTx: SiiMhlTxMscCommandDone. data1 = %02X\n", (int) data1) );

	if(( MHL_READ_DEVCAP == mhlTxConfig.mscLastCommand ) &&
			(0x02 == mhlTxConfig.mscLastOffset))
	{

#ifdef CONFIG_CABLE_DETECT_ACCESSORY
		SiiMhlTxMscDetectCharger(data1);
#endif

		mhlTxConfig.mscState	= MSC_STATE_POW_DONE;
	}
	else if((MHL_READ_DEVCAP == mhlTxConfig.mscLastCommand) &&
				(0x0A == mhlTxConfig.mscLastOffset))
	{
		mhlTxConfig.mscState	= MSC_STATE_RCP_READY;

		mhlTxConfig.mscFeatureFlag	= data1;

		mhlTxConfig.mhlConnectionEvent = true;
		mhlTxConfig.mhlConnected = MHL_TX_EVENT_RCP_READY;

		mhlTxConfig.mscLastCommand = 0;
		mhlTxConfig.mscLastOffset  = 0;

		TPI_DEBUG_PRINT( ("MhlTx: Peer's Feature Flag = %02X\n\n", (int) data1) );
	}
	else if(MHL_MSC_MSG_RCPE == mhlTxConfig.mscMsgLastCommand)
	{
		if( SiiMhlTxRcpkSend( mhlTxConfig.mscSaveRcpKeyCode ) )
		{
			mhlTxConfig.mscMsgLastCommand = 0;
			mhlTxConfig.mscMsgLastData    = 0;
		}
	}
}

void	SiiMhlTxGotMhlMscMsg( uint8_t subCommand, uint8_t cmdData )
{
	mhlTxConfig.mscMsgArrived		= true;
	mhlTxConfig.mscMsgSubCommand	= subCommand;
	mhlTxConfig.mscMsgData			= cmdData;
}

void	SiiMhlTxGotMhlIntr( uint8_t intr_0, uint8_t intr_1 )
{
	TPI_DEBUG_PRINT( ("MhlTx: INTERRUPT Arrived. %02X, %02X\n", (int) intr_0, (int) intr_1) );


	if(BIT_0 & intr_0)
	{
		SiiMhlTxReadDevcap( 0x02 );
	}
	else if(BIT_1 & intr_1)
	{
		SiiMhlTxDrvNotifyEdidChange ( );
	}
}

void	SiiMhlTxGotMhlStatus( uint8_t status_0, uint8_t status_1 )
{
	TPI_DEBUG_PRINT( ("MhlTx: STATUS Arrived. %02X, %02X\n", (int) status_0, (int) status_1) );

	if(BIT_0 & status_0)
	{
		mhlTxConfig.mscState	 = MSC_STATE_BEGIN;
	}
	mhlTxConfig.status_0 = status_0;
	mhlTxConfig.status_1 = status_1;
}

bool SiiMhlTxRcpSend( uint8_t rcpKeyCode )
{
	if((0 == (BIT_0 & mhlTxConfig.mscFeatureFlag)) ||
		(MSC_STATE_RCP_READY != mhlTxConfig.mscState))
	{
		return	false;
	}
	return	( MhlTxSendMscMsg ( MHL_MSC_MSG_RCP, rcpKeyCode ) );
}

bool SiiMhlTxRcpkSend( uint8_t rcpKeyCode )
{
	return	( MhlTxSendMscMsg ( MHL_MSC_MSG_RCPK, rcpKeyCode ) );
}

static	bool SiiMhlTxRapkSend( void )
{
	return	( MhlTxSendMscMsg ( MHL_MSC_MSG_RAPK, 0 ) );
}

bool SiiMhlTxRcpeSend( uint8_t rcpeErrorCode )
{
	return( MhlTxSendMscMsg ( MHL_MSC_MSG_RCPE, rcpeErrorCode ) );
}

bool SiiMhlTxReadDevcap( uint8_t offset )
{
	cbus_req_t	req;

	req.command     = mhlTxConfig.mscLastCommand = MHL_READ_DEVCAP;
	req.offsetData  = mhlTxConfig.mscLastOffset  = offset;
	return(SiiMhlTxDrvSendCbusCommand( &req  ));
}

static bool MhlTxSendMscMsg ( uint8_t command, uint8_t cmdData )
{
	cbus_req_t	req;
	uint8_t		ccode;

	req.command     = MHL_MSC_MSG;
	req.msgData[0]  = mhlTxConfig.mscMsgLastCommand = command;
	req.msgData[1]  = mhlTxConfig.mscMsgLastData    = cmdData;

	ccode = SiiMhlTxDrvSendCbusCommand( &req  );
	return( (bool) ccode );

}

void	SiiMhlTxNotifyConnection( bool mhlConnected )
{

	mhlTxConfig.mhlConnectionEvent = true;

	mhlTxConfig.mscState	 = MSC_STATE_IDLE;
	if(mhlConnected)
	{
		mhlTxConfig.mhlConnected = MHL_TX_EVENT_CONNECTION;
	}
	else
	{
		mhlTxConfig.mhlConnected = MHL_TX_EVENT_DISCONNECTION;
	}
}

void	SiiMhlTxNotifyDsHpdChange( uint8_t dsHpdStatus )
{
	if( 0 == dsHpdStatus )
	{
	    TPI_DEBUG_PRINT(("MhlTx: Disable TMDS\n"));
		SiiMhlTxDrvTmdsControl( false );
	}
	else
	{
	    TPI_DEBUG_PRINT(("MhlTx: Enable TMDS\n"));
		SiiMhlTxDrvTmdsControl( true );
	}
}

static void	MhlTxResetStates( void )
{
	mhlTxConfig.mhlConnectionEvent	= false;
	mhlTxConfig.mhlConnected		= MHL_TX_EVENT_DISCONNECTION;
	mhlTxConfig.mscMsgArrived		= false;
	mhlTxConfig.mscState			= MSC_STATE_IDLE;
}
extern void sii9234_send_keyevent(uint32_t key, uint32_t type);
static uint8_t ProcessRcpKeyCode(uint8_t rcpKeyCode)
{
    uint8_t rcpkStatus = rcpKeyCode;

    TPI_DEBUG_PRINT(("RCP Key Code: 0x%02X\n", (int)rcpKeyCode));

    switch ( rcpKeyCode )
    {
        case MHD_RCP_CMD_SELECT:
			TPI_DEBUG_PRINT(( "\n MHD_RCP_CMD_SELECT received %d\n\n", (int)rcpKeyCode ));
			sii9234_send_keyevent(KEY_ENTER, 0);
			break;
        case MHD_RCP_CMD_UP:
			TPI_DEBUG_PRINT(( "\n MHD_RCP_CMD_UP received %d\n\n", (int)rcpKeyCode ));
			sii9234_send_keyevent(KEY_UP, 0);
			break;
         case MHD_RCP_CMD_DOWN:
			TPI_DEBUG_PRINT(( "\n MHD_RCP_CMD_DOWN received %d\n\n", (int)rcpKeyCode ));
			sii9234_send_keyevent(KEY_DOWN, 0);
			break;
        case MHD_RCP_CMD_LEFT:
			TPI_DEBUG_PRINT(( "\n MHD_RCP_CMD_LEFT received %d\n\n", (int)rcpKeyCode ));
			sii9234_send_keyevent(KEY_LEFT, 0);
			break;
         case MHD_RCP_CMD_RIGHT:
			TPI_DEBUG_PRINT(( "\n MHD_RCP_CMD_RIGHT received %d\n\n", (int)rcpKeyCode ));
			sii9234_send_keyevent(KEY_RIGHT, 0);
			break;
         case MHD_RCP_CMD_ROOT_MENU:
			TPI_DEBUG_PRINT(( "\n MHD_RCP_CMD_ROOT_MENU received %d\n\n", (int)rcpKeyCode ));
			sii9234_send_keyevent(KEY_HOME, 0);
			break;
         case MHD_RCP_CMD_EXIT:
			TPI_DEBUG_PRINT(( "\n MHD_RCP_CMD_EXIT received %d\n\n", (int)rcpKeyCode ));
			sii9234_send_keyevent(KEY_BACK, 0);
			break;
        default:
            break;
    }

    return( rcpkStatus );
}

void    ProcessRcp( uint8_t event, uint8_t eventParameter)
{
        uint8_t         rcpKeyCode;

        switch( event )
        {
                case    MHL_TX_EVENT_DISCONNECTION:
                        TPI_DEBUG_PRINT(("App: Got event = MHL_TX_EVENT_DISCONNECTION\n"));
                        break;

                case    MHL_TX_EVENT_CONNECTION:
                        TPI_DEBUG_PRINT(("App: Got event = MHL_TX_EVENT_CONNECTION\n"));
                        break;

                case    MHL_TX_EVENT_RCP_READY:

                        rcpKeyCode = APP_DEMO_RCP_SEND_KEY_CODE;

                        TPI_DEBUG_PRINT(("App: Got event = MHL_TX_EVENT_RCP_READY...Sending RCP (%02X)\n", (int) rcpKeyCode));


                        if( (0 == (BIT_0 & eventParameter)) )
                        {
                                TPI_DEBUG_PRINT(( "App: Peer does NOT support RCP\n" ));
                        }
                        if( (0 == (BIT_1 & eventParameter)) )
                        {
                                TPI_DEBUG_PRINT(( "App: Peer does NOT support RAP\n" ));
                        }
                        if( (0 == (BIT_2 & eventParameter)) )
                        {
                                TPI_DEBUG_PRINT(( "App: Peer does NOT support WRITE_BURST\n" ));
                        }


                        if( SiiMhlTxRcpSend( rcpKeyCode ))
                        {
                                TPI_DEBUG_PRINT(("App: SiiMhlTxRcpSend (%02X)\n", (int) rcpKeyCode));
                        }
                        else
                        {
                                TPI_DEBUG_PRINT(("App: SiiMhlTxRcpSend (%02X) Returned Failure.\n", (int) rcpKeyCode));
                        }
                        break;

                case    MHL_TX_EVENT_RCP_RECEIVED:
			TPI_DEBUG_PRINT(("App: Received an RCP key code = %02X\n", eventParameter ));
			rcpKeyCode = ProcessRcpKeyCode(eventParameter);
                        SiiMhlTxRcpkSend((int) rcpKeyCode);
                        break;

                case    MHL_TX_EVENT_RCPK_RECEIVED:
                        TPI_DEBUG_PRINT(("App: Received an RCPK = %02X\n", (int)eventParameter));
                        break;

                case    MHL_TX_EVENT_RCPE_RECEIVED:
                        TPI_DEBUG_PRINT(("App: Received an RCPE = %02X\n", (int)eventParameter));
                        break;

                default:
			TPI_DEBUG_PRINT(("App: Got event = %02X, eventParameter = %02X\n", (int)event, (int)eventParameter));
                        break;
        }
}

