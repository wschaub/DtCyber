/*--------------------------------------------------------------------------
**
**  Copyright (c) 2003-2011, Tom Hunter
**
**  Name: npu_tip.c
**
**  Description:
**      Perform emulation of the Terminal Interface Protocol in an NPU
**      consisting of a CDC 2550 HCP running CCP.
**
**  This program is free software: you can redistribute it and/or modify
**  it under the terms of the GNU General Public License version 3 as
**  published by the Free Software Foundation.
**  
**  This program is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU General Public License version 3 for more details.
**  
**  You should have received a copy of the GNU General Public License
**  version 3 along with this program in file "license-gpl-3.0.txt".
**  If not, see <http://www.gnu.org/licenses/gpl-3.0.txt>.
**
**--------------------------------------------------------------------------
*/

/*
**  -------------
**  Include Files
**  -------------
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "const.h"
#include "types.h"
#include "proto.h"
#include "npu.h"

/*
**  -----------------
**  Private Constants
**  -----------------
*/

/*
**  Terminal primary/secondary function codes.
*/
#define PfcCTRL   0xC1          // terminal characteristics 
#define   SfcDEF     0x04       // define characteristics   
#define   SfcCHAR    0x08       // define multiple char.    
#define   SfcRTC     0x09       // request terminal istics  
#define   SfcTCD     0x0A       // term istics definition   
                                                  
#define PfcBD     0xC2          // terminal istics alternate
#define   SfcCHG     0x00       // change                   
                                                  
#define PfcBF     0xC3          // file characteristics     
//#define   SfcCHG     0x00     // change                   
                                                  
#define PfcTO     0xC4          // terminate output marker  
#define   SfcMARK    0x00       // marker                   
                                                  
#define PfcSI     0xC5          // start input              
#define   SfcNONTR   0x01       // non-transparent          
#define   SfcTRAN    0x02       // transparent              
#define   SfcRSM     0x03       // resume                   
                                                  
#define PfcAI     0xC6          // abort input              
#define   SfcTERM    0x00       // terminal                 
                                                  
#define PfcIS     0xC7          // input stopped            
#define   SfcNR      0x04       // not ready                
#define   SfcSC      0x02       // slipped card             
#define   SfcES      0x03       // end-of-stream            
#define   SfcBI      0x01       // batch interrupt          
                                                  
#define PfcOS     0xC8          // output stopped           
//#define   SfcBI      0x01     // batch interrupt          
#define   SfcPM      0x02       // printer message          
#define   SfcFLF     0x03       // file limit               
//#define   SfcNR      0x04     // not ready                
                                                  
#define PfcAD     0xC9          // accounting data          
#define   SfcEOI     0x01       // EOI processed            
#define   SfcIOT     0x02       // I/O terminated           
#define   SfcTF      0x03       // terminal failure         
                                                  
#define PfcBI     0xCA          // break indication marker  
//#define   SfcMARK    0x00     // marker                   
                                                  
#define PfcRO     0xCB          // resume output marker     
//#define   SfcMARK    0x00     // marker                   
                                                  
#define PfcFT     0xCC          // A - A using PRU type data
#define   SfcON      0x00       // turn on PRU mode         
#define   SfcOFF     0x01       // turn off PRU mode        

/*
**  Field name codes, used in various packets such as CNF/TE. These are
**  defined in "NAM 1 Host Application Progr. RM (60499500W 1987)" on
**  pages 3-59 to 3-62.
*/
#define FnTdAbortBlock      0x29    // Abort block character
#define FnTdBlockFactor     0x19    // Blocking factor; multiple of 100 chars (upline block)
#define FnTdBreakAsUser     0x33    // Break as user break 1; yes (1), no (0)
#define FnTdBS              0x27    // Backspace character
#define FnTdUserBreak1      0x2A    // User break 1 character
#define FnTdUserBreak2      0x2B    // User break 2 character
#define FnTdEnaXUserBreak   0x95    // Enable transparent user break commands; yes (1), no (0)
#define FnTdCI              0x2C    // Carriage return idle count
#define FnTdCIAuto          0x2E    // Carriage return idle count - TIP calculates suitable number
#define FnTdCN              0x26    // Cancel character
#define FnTdCursorPos       0x47    // Cursor positioning; yes (1), no (0)
#define FnTdCT              0x28    // Network control character
#define FnTdXCharFlag       0x38    // Transparent input delimiter character specified flag; yes (1), no (0)
#define FnTdXCntMSB         0x39    // Transparent input delimiter count MSB
#define FnTdXCntLSB         0x3A    // Transparent input delimiter count LSB
#define FnTdXChar           0x3B    // Transparent input delimiter character
#define FnTdXTimeout        0x3C    // Transparent input mode delimiter timeout; yes (1), no (0)
#define FnTdXModeMultiple   0x46    // Transparent input mode; multiple (1), single (0)
#define FnTdEOB             0x40    // End of block character
#define FnTdEOBterm         0x41    // End of block terminator; EOL (1), EOB (2)
#define FnTdEOBCursorPos    0x42    // EOB cursor pos; no (0), CR (1), LF (2), CR & LF (3)
#define FnTdEOL             0x3D    // End of line character
#define FnTdEOLTerm         0x3E    // End of line terminator; EOL (1), EOB (2)
#define FnTdEOLCursorPos    0x3F    // EOL cursor pos; no (0), CR (1), LF (2), CR & LF (3)
#define FnTdEchoplex        0x31    // Echoplex mode
#define FnTdFullASCII       0x37    // Full ASCII input; yes (1), no (0)
#define FnTdInFlowControl   0x43    // Input flow control; yes (1), no (0)
#define FnTdXInput          0x34    // Transparent input; yes (1), no (0)
#define FnTdInputDevice     0x35    // Keyboard (0), paper tape (1), block mode (2)
#define FnTdLI              0x2D    // Line feed idle count
#define FnTdLIAuto          0x2F    // Line feed idle count - TIP calculates suitable number; yes (1), no (0)
#define FnTdLockKeyboard    0x20    // Lockout unsolicited input from keyboard; yes (1), no (0)
#define FnTdOutFlowControl  0x44    // Output flow control; yes (1), no (0)
#define FnTdOutputDevice    0x36    // Printer (0), display (1), paper tape (2)
#define FnTdParity          0x32    // Zero (0), odd (1), even (2), none (3), ignore (4)
#define FnTdPG              0x25    // Page waiting; yes (1), no (0)
#define FnTdPL              0x24    // Page length in lines; 0, 8 - FF
#define FnTdPW              0x23    // Page width in columns; 0, 20 - FF
#define FnTdSpecialEdit     0x30    // Special editing mode; yes (1), no (0)
#define FnTdTC              0x22    // Terminal class
#define FnTdXStickyTimeout  0x92    // Sticky transparent input forward on timeout; yes (1), no (0)
#define FnTdXModeDelimiter  0x45    // Transparent input mode delimiter character
#define FnTdDuplex          0x57    // full (1), half (0)
#define FnTdTermTransBsMSB  0x1E    // Terminal transmission block size MSB
#define FnTdTermTransBsLSB  0x1F    // Terminal transmission block size LSB
#define FnTdSolicitInput    0x70    // yes (1), no (0)
#define FnTdCIDelay         0x93    // Carriage return idle delay in 4 ms increments
#define FnTdLIDelay         0x94    // Line feed idle delay in 4 ms increments

/*
**  The Field Name values below are not documented in the NAM manual.
*/
#define FnTdHostNode        0x14    // Selected host node
#define FnTdAutoConnect     0x16    // yes (1), no (0)
#define FnTdPriority        0x17    // Terminal priority
#define FnTdUBL             0x18    // Upline block count limit
#define FnTdABL             0x1A    // Application block count limit
#define FnTdDBL             0x1B    // Downline block count limit
#define FnTdDBSizeMSB       0x1C    // Downline block size MSB
#define FnTdDBSizeLSB       0x1D    // Downline block size LSB
#define FnTdRestrictedRBF   0x4D    // Restriced capacity RBF

/*
**  -----------------------
**  Private Macro Functions
**  -----------------------
*/

/*
**  -----------------------------------------
**  Private Typedef and Structure Definitions
**  -----------------------------------------
*/

/*
**  ---------------------------
**  Private Function Prototypes
**  ---------------------------
*/
static void npuTipSetupDefaultTc2(void);
static void npuTipSetupDefaultTc3(void);
static void npuTipSetupDefaultTc7(void);

/*
**  ----------------
**  Public Variables
**  ----------------
*/
Tcb *npuTcbs;
int npuTcbCount;

/*
**  -----------------
**  Private Variables
**  -----------------
*/
static u8 ackInitBt[] =
    {
    AddrHost,           // DN
    AddrNpu,            // SN
    1,                  // CN
    (0 << BlkShiftBSN) | BtHTBACK, // BT/BSN/PRIO
    };

static u8 respondToInitBt[] =
    {
    AddrHost,           // DN
    AddrNpu,            // SN
    1,                  // CN
    (0 << BlkShiftBSN) | BtHTNINIT, // BT/BSN/PRIO
    };

static u8 requestInitBt[] =
    {
    AddrHost,           // DN
    AddrNpu,            // SN
    1,                  // CN
    (0 << BlkShiftBSN) | BtHTRINIT, // BT/BSN/PRIO
    };

static u8 blockAck[] =
    {
    AddrHost,           // DN
    AddrNpu,            // SN
    1,                  // CN
    BtHTBACK,           // BT/BSN/PRIO
    };

static u8 blockTerm[] =
    {
    AddrHost,           // DN
    AddrNpu,            // SN
    1,                  // CN
    BtHTTERM,           // BT/BSN/PRIO
    };

static u8 intrRsp[4] =
    {
    AddrHost,           // DN
    AddrNpu,            // SN
    1,                  // CN
    BtHTICMR,           // BT/BSN/PRIO
    };

TipParams work;
TipParams defaultTc2;
TipParams defaultTc3;
TipParams defaultTc7;

/*
**--------------------------------------------------------------------------
**
**  Public Functions
**
**--------------------------------------------------------------------------
*/

/*--------------------------------------------------------------------------
**  Purpose:        Initialize TIP.
**
**  Parameters:     Name        Description.
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuTipInit(void)
    {
    int i;
    Tcb *tp;

    /*
    **  Allocate TCBs.
    */
    npuTcbCount = npuNetTcpConns;
    npuTcbs = calloc(npuTcbCount, sizeof(Tcb));
    if (npuTcbs == NULL)
        {
        fprintf(stderr, "Failed to allocate NPU TCBs\n");
        exit(1);
        }

    /*
    **  Initialize default terminal class parameters.
    */
    npuTipSetupDefaultTc2();
    npuTipSetupDefaultTc3();
    npuTipSetupDefaultTc7();

    /*
    **  Initialise TCBs.
    */
    tp = npuTcbs;
    for (i = 0; i < npuNetTcpConns; i++, tp++)
        {
        tp->portNumber = i + 1;
        tp->params = defaultTc3;
        tp->tipType = TtASYNC;
        npuTipInputReset(tp);
        }

    /*
    **  Initialise network.
    */
    npuNetInit(TRUE);
    }

/*--------------------------------------------------------------------------
**  Purpose:        Reset TIP.
**
**  Parameters:     Name        Description.
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuTipReset(void)
    {
    int i;
    Tcb *tp = npuTcbs;

    /*
    **  Iterate through all TCBs.
    */
    for (i = 0; i < npuNetTcpConns; i++, tp++)
        {
        memset(tp, 0, sizeof(Tcb));
        tp->portNumber = i + 1;
        tp->params = defaultTc3;
        tp->tipType = TtASYNC;
        npuTipInputReset(tp);
        }

    /*
    **  Re-initialise network.
    */
    npuNetInit(FALSE);
    }

/*--------------------------------------------------------------------------
**  Purpose:        Process service message from host.
**
**  Parameters:     Name        Description.
**                  bp          buffer with service message.
**                  priority    priority
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuTipProcessBuffer(NpuBuffer *bp, int priority)
    {
    static int count = 0;
    u8 *block = bp->data;
    Tcb *tp;
    u8 cn;
    bool last;

    /*
    **  Determine associated terminal control block.
    */
    cn = block[BlkOffCN];
    if (cn == 0 || cn > npuTcbCount)
        {
        npuLogMessage("Unexpected TIP connection number %u in message %02X/%02X in state %d", cn, block[BlkOffPfc], block[BlkOffSfc]);
        return;
        }
    else
        {
        tp = npuTcbs + cn - 1;
        }

    switch (block[BlkOffBTBSN] & BlkMaskBT)
        {
    case BtHTRINIT:
        ackInitBt[BlkOffCN] = block[BlkOffCN];
        npuBipRequestUplineCanned(ackInitBt, sizeof(ackInitBt));

        respondToInitBt[BlkOffCN] = block[BlkOffCN];
        npuBipRequestUplineCanned(respondToInitBt, sizeof(respondToInitBt));

        requestInitBt[BlkOffCN] = block[BlkOffCN];
        npuBipRequestUplineCanned(requestInitBt, sizeof(requestInitBt));
        break;

    case BtHTCMD:
        if (block[BlkOffPfc] == PfcCTRL && block[BlkOffSfc] == SfcCHAR)
            {
            /*
            **  Terminal characteristics/define multiple characteristics -
            **  setup TCB with supported FN/FV values.
            */
            npuTipParseFnFv(block + BlkOffP3, bp->numBytes - 6, tp);
            }
        else if (block[BlkOffPfc] == PfcRO && block[BlkOffSfc] == SfcMARK)
            {
            /*
            **  Resume output marker after user break 1 or 2.
            */
            tp->breakPending = FALSE;
            }

        /*
        **  Acknowledge any command (although most are ignored).
        */
        blockAck[BlkOffCN] = block[BlkOffCN];
        blockAck[BlkOffBTBSN] &= BlkMaskBT;
        blockAck[BlkOffBTBSN] |= block[BlkOffBTBSN] & (BlkMaskBSN << BlkShiftBSN);
        npuBipRequestUplineCanned(blockAck, sizeof(blockAck));
        break;

    case BtHTBLK:
    case BtHTMSG:
        if (tp->state == StTermHostConnected)
            {
            last = (block[BlkOffBTBSN] & BlkMaskBT) == BtHTMSG;
            npuAsyncProcessDownlineData(block[BlkOffCN], bp, last);
            }
        else
            {
            /*
            **  Handle possible race condition while disconnecting. Acknowledge any
            **  packets arriving during this time, but discard the contents.
            */
            blockAck[BlkOffCN] = block[BlkOffCN];
            blockAck[BlkOffBTBSN] &= BlkMaskBT;
            blockAck[BlkOffBTBSN] |= block[BlkOffBTBSN] & (BlkMaskBSN << BlkShiftBSN);
            npuBipRequestUplineCanned(blockAck, sizeof(blockAck));
            }
        break;

    case BtHTBACK:
        /*
        **  Ignore acknowledgment for now.
        */
        break;

    case BtHTTERM:
        if (tp->state == StTermHostDisconnect)
            {
            /*
            **  Host has echoed our TERM block now send the TCN/TA/N to host.
            */
            npuSvmDiscReplyTerminal(tp);

            /*
            **  Finally disconnect the network.
            */
            npuNetDisconnected(tp);
            }
        else if (tp->state == StTermNpuDisconnect)
            {
            /*
            **  Echo TERM block.
            */
            blockTerm[BlkOffCN] = block[BlkOffCN];
            npuBipRequestUplineCanned(blockTerm, sizeof(blockTerm));
            }
        else
            {
            npuLogMessage("Unexpected TERM block on connection %u", cn);
            }
        break;

    case BtHTICMD:
        /*
        **  Interrupt command.  Discard any pending output.
        */
        if(! (tp->params.fvXInput && tp->params.fvInFlowControl)) {
            tp->xoff = FALSE;
        }
        npuTipDiscardOutputQ(tp);
        intrRsp[BlkOffCN] = block[BlkOffCN];
        intrRsp[BlkOffBTBSN] &= BlkMaskBT;
        intrRsp[BlkOffBTBSN] |= block[BlkOffBTBSN] & (BlkMaskBSN << BlkShiftBSN);
        npuBipRequestUplineCanned(intrRsp, sizeof(intrRsp));
        break;
        
    case BtHTICMR:
        /*
        **  Ignore interrupt response.
        */
        break;
        }

    /*
    **  Release downline buffer.
    */
    npuBipBufRelease(bp);
    }

/*--------------------------------------------------------------------------
**  Purpose:        Process terminate connection message from host.
**
**  Parameters:     Name        Description.
**                  tp          pointer to TCB
**                  bsn         block sequence number
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuTipTerminateConnection(Tcb *tp)
    {
    /*
    **  Clean up flow control state and discard any pending output.
    */
    tp->xoff = FALSE;
    npuTipDiscardOutputQ(tp);
    tp->state = StTermHostDisconnect;

    /*
    **  Send an initial TERM block which will be echoed by the host.
    */
    blockTerm[BlkOffCN] = tp->portNumber;
    npuBipRequestUplineCanned(blockTerm, sizeof(blockTerm));
    }

/*--------------------------------------------------------------------------
**  Purpose:        Setup default parameters for the specified terminal class.
**
**  Parameters:     Name        Description.
**                  tp          pointer to TCB
**                  tc          terminal class
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuTipSetupTerminalClass(Tcb *tp, u8 tc)
    {
    switch (tc)
        {
    case 2:
        tp->params = defaultTc2;
        break;

    default:
    case 3:
        tp->params = defaultTc3;
        break;

    case 7:
        tp->params = defaultTc7;
        break;
        }
    }

/*--------------------------------------------------------------------------
**  Purpose:        Parse FN/FV string.
**
**  Parameters:     Name        Description.
**                  mp          message pointer
**                  len         message length
**                  tp          pointer to TCB
**
**  Returns:        TRUE if no error, FALSE otherwise.
**
**------------------------------------------------------------------------*/
bool npuTipParseFnFv(u8 *mp, int len, Tcb *tp)
    {
    TipParams *pp = &tp->params;

    while (len > 0)
        {
        switch (mp[0])
            {
        case FnTdAbortBlock:     // Abort block character
            pp->fvAbortBlock = mp[1];
            break;

        case FnTdBlockFactor:    // Blocking factor; multiple of 100 chars (upline block)
            /*
            **  Only accept a sane blocking factor. The resulting block must not
            **  be larger then NPU buffer. This will also protect us from buffer
            **  overruns in the upline functions of the ASYNC TIP.
            */
            if (mp[1] > 0 && mp[1] <= 20)
                {
                pp->fvBlockFactor = mp[1];
                }
            break;

        case FnTdBreakAsUser:    // Break as user break 1; yes (1), no (0)
            pp->fvBreakAsUser = mp[1] != 0;
            break;

        case FnTdBS:             // Backspace character
            pp->fvBS = mp[1];
            break;

        case FnTdUserBreak1:     // User break 1 character
            pp->fvUserBreak1 = mp[1];
            break;

        case FnTdUserBreak2:     // User break 2 character
            pp->fvUserBreak2 = mp[1];
            break;

        case FnTdEnaXUserBreak:  // Enable transparent user break commands; yes (1), no (0)
            pp->fvEnaXUserBreak = mp[1] != 0;
            break;

        case FnTdCI:             // Carriage return idle count
            pp->fvCI = mp[1];
            break;

        case FnTdCIAuto:         // Carriage return idle count - TIP calculates suitable number
            pp->fvCIAuto = mp[1] != 0;
            break;

        case FnTdCN:             // Cancel character
            pp->fvCN = mp[1];
            break;

        case FnTdCursorPos:      // Cursor positioning; yes (1), no (0)
            pp->fvCursorPos = mp[1] != 0;
            break;

        case FnTdCT:             // Network control character
            pp->fvCT = mp[1];
            break;

        case FnTdXCharFlag:      // Transparent input delimiter character specified flag; yes (1), no (0)
            pp->fvXCharFlag = mp[1] != 0;
            break;

        case FnTdXCntMSB:        // Transparent input delimiter count MSB
            pp->fvXCnt &= 0x00FF;     
            pp->fvXCnt |= mp[1] << 8; 
            break;

        case FnTdXCntLSB:        // Transparent input delimiter count LSB
            pp->fvXCnt &= 0xFF00;     
            pp->fvXCnt |= mp[1] << 0; 
            break;

        case FnTdXChar:          // Transparent input delimiter character
            pp->fvXChar = mp[1];
            break;

        case FnTdXTimeout:       // Transparent input delimiter timeout; yes (1), no (0)
            pp->fvXTimeout = mp[1] != 0;
            break;

        case FnTdXModeMultiple:  // Transparent input mode; multiple (1), single (0)
            pp->fvXModeMultiple = mp[1] != 0;
            break;

        case FnTdEOB:            // End of block character
            pp->fvEOB = mp[1];
            break;

        case FnTdEOBterm:        // End of block terminator; EOL (1), EOB (2)
            pp->fvEOBterm = mp[1];
            break;

        case FnTdEOBCursorPos:   // EOB cursor pos; no (0), CR (1), LF (2), CR & LF (3)
            pp->fvEOBCursorPos = mp[1];
            break;

        case FnTdEOL:            // End of line character
            pp->fvEOL = mp[1];
            break;

        case FnTdEOLTerm:        // End of line terminator; EOL (1), EOB (2)
            pp->fvEOLTerm = mp[1];
            break;

        case FnTdEOLCursorPos:   // EOL cursor pos; no (0), CR (1), LF (2), CR & LF (3)
            pp->fvEOLCursorPos = mp[1];
            break;

        case FnTdEchoplex:       // Echoplex mode
            pp->fvEchoplex = mp[1] != 0;
            break;

        case FnTdFullASCII:      // Full ASCII input; yes (1), no (0)
            pp->fvFullASCII = mp[1] != 0;
            break;

        case FnTdInFlowControl:  // Input flow control; yes (1), no (0)
            pp->fvInFlowControl = mp[1] != 0;
            break;

        case FnTdXInput:         // Transparent input; yes (1), no (0)
            pp->fvXInput = mp[1] != 0;
            break;

        case FnTdInputDevice:    // Keyboard (0), paper tape (1), block mode (2)
            pp->fvInputDevice = mp[1];
            break;

        case FnTdLI:             // Line feed idle count
            pp->fvLI = mp[1];
            break;

        case FnTdLIAuto:         // Line feed idle count - TIP calculates suitable number; yes (1), no (0)
            pp->fvLIAuto = mp[1] != 0;
            break;

        case FnTdLockKeyboard:   // Lockout unsolicited input from keyboard; yes (1), no (0)
            pp->fvLockKeyboard = mp[1] != 0;
            break;

        case FnTdOutFlowControl: // Output flow control; yes (1), no (0)
            pp->fvOutFlowControl = mp[1] != 0;
            if (!pp->fvOutFlowControl)
                {
                /*
                **  If flow control is now disabled, turn off the xoff flag
                **  if it was set.
                */
                tp->xoff = FALSE;
                } 
            break;

        case FnTdOutputDevice:   // Printer (0), display (1), paper tape (2)
            pp->fvOutputDevice = mp[1];
            break;

        case FnTdParity:         // Zero (0), odd (1), even (2), none (3), ignore (4)
            pp->fvParity = mp[1];
            // printf("Term = %u, Parity = %u\n", tp->portNumber, mp[1]); fflush(stdout);
            break;

        case FnTdPG:             // Page waiting; yes (1), no (0)
            pp->fvPG = mp[1] != 0;
            break;

        case FnTdPL:             // Page length in lines; 0, 8 - FF
            pp->fvPL = mp[1];
            break;

        case FnTdPW:             // Page width in columns; 0, 20 - FF
            pp->fvPW = mp[1];
            break;

        case FnTdSpecialEdit:    // Special editing mode; yes (1), no (0)
            pp->fvSpecialEdit = mp[1] != 0;
            break;

        case FnTdTC:             // Terminal class
            if (pp->fvTC != mp[1])
                {
                pp->fvTC = mp[1];
                switch (pp->fvTC)
                    {
                case 2:
                    tp->params = defaultTc2;
                    break;

                default:
                case 3:
                    tp->params = defaultTc3;
                    break;

                case 7:
                    tp->params = defaultTc7;
                    break;
                    }
                }
            break;

        case FnTdXStickyTimeout:// Sticky transparent input forward on timeout; yes (1), no (0)
            pp->fvXStickyTimeout = mp[1] != 0;
            break;

        case FnTdXModeDelimiter: // Transparent input mode delimiter character
            pp->fvXModeDelimiter = mp[1];
            break;

        case FnTdDuplex:         // full (1), half (0)
            pp->fvDuplex = mp[1] != 0;
            break;

        case FnTdTermTransBsMSB: // Terminal transmission block size MSB
            pp->fvTermTransBs &= 0x00FF;     
            pp->fvTermTransBs |= mp[1] << 8; 
            break;

        case FnTdTermTransBsLSB: // Terminal transmission block size LSB
            pp->fvTermTransBs &= 0xFF00;     
            pp->fvTermTransBs |= mp[1] << 0; 
            break;

        case FnTdSolicitInput:   // yes (1), no (0)
            pp->fvSolicitInput = mp[1] != 0;
            break;

        case FnTdCIDelay:        // Carriage return idle delay in 4 ms increments
            pp->fvCIDelay = mp[1];
            break;

        case FnTdLIDelay:        // Line feed idle delay in 4 ms increments
            pp->fvLIDelay = mp[1];
            break;

        case FnTdHostNode:       // Selected host node
            pp->fvHostNode = mp[1];
            break;

        case FnTdAutoConnect:    // yes (1), no (0)
            pp->fvAutoConnect = mp[1] != 0;
            break;

        case FnTdPriority:       // Terminal priority
            pp->fvPriority = mp[1];
            break;

        case FnTdUBL:            // Upline block count limit
            pp->fvUBL = mp[1];
            break;

        case FnTdABL:            // Application block count limit
            pp->fvABL = mp[1];
            break;

        case FnTdDBL:            // Downline block count limit
            pp->fvDBL = mp[1];
            break;

        case FnTdDBSizeMSB:      // Downline block size MSB
            pp->fvDBSize &= 0x00FF;    
            pp->fvDBSize |= mp[1] << 8;
            break;

        case FnTdDBSizeLSB:      // Downline block size LSB
            pp->fvDBSize &= 0xFF00;    
            pp->fvDBSize |= mp[1] << 0;
            break;

        case FnTdRestrictedRBF:  // Restriced capacity RBF
            pp->fvRestrictedRBF = mp[1];
            break;

        default:
            npuLogMessage("TIP: unknown FN/FV (%d/%d)[%02X/%02X]", mp[0], mp[1], mp[0], mp[1]);
            break;
            }

        /*
        **  Advance to next FN/FV pair.
        */
        mp += 2;
        len -= 2;
        }

    return(TRUE);
    }

/*--------------------------------------------------------------------------
**  Purpose:        Reset the input buffer state.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuTipInputReset(Tcb *tp)
    {
    u8 *mp = tp->inBuf;

    /*
    **  Increment BSN.
    */
    tp->uplineBsn += 1;
    if (tp->uplineBsn == 8)
        {
        tp->uplineBsn = 1;
        }

    /*
    **  Build upline data header.
    */
    *mp++ = AddrHost;           // DN
    *mp++ = AddrNpu;            // SN
    *mp++ = tp->portNumber;     // CN
    *mp++ = BtHTMSG | (tp->uplineBsn << BlkShiftBSN);    // BT=MSG
    *mp++ = 0;

    /*
    **  Initialise buffer pointers.
    */
    tp->inBufStart = mp;
    tp->inBufPtr = mp;
    }

/*--------------------------------------------------------------------------
**  Purpose:        Send user break 1 or 2 to host.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**                  bt          break type (1 or 2)
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuTipSendUserBreak(Tcb *tp, u8 bt)
    {
    u8 *mp;

    /*
    **  Ignore user break if previous break has not yet been processed.
    */
    if (tp->breakPending)
        {
        return;
        }

    tp->breakPending = TRUE;

    /*
    **  Build upline ICMD.
    */
    mp = tp->inBuf;
    *mp++ = AddrHost;           // DN
    *mp++ = AddrNpu;            // SN
    *mp++ = tp->portNumber;     // CN
    *mp++ = BtHTICMD | (tp->uplineBsn << BlkShiftBSN);    // BT=BRK
    *mp++ = (1 << (bt - 1)) + 2;

    /*
    **  Send the ICMD.
    */
    npuBipRequestUplineCanned(tp->inBuf, mp - tp->inBuf);

    /*
    **  Increment BSN.
    */
    tp->uplineBsn += 1;
    if (tp->uplineBsn == 8)
        {
        tp->uplineBsn = 1;
        }

    /*
    **  Build upline BI/MARK.
    */
    mp = tp->inBuf;
    *mp++ = AddrHost;           // DN
    *mp++ = AddrNpu;            // SN
    *mp++ = tp->portNumber;     // CN
    *mp++ = BtHTCMD | (tp->uplineBsn << BlkShiftBSN);    // BT=BRK
    *mp++ = PfcBI;
    *mp++ = SfcMARK;

    /*
    **  Send the BI/MARK.
    */
    npuBipRequestUplineCanned(tp->inBuf, mp - tp->inBuf);

    /*
    **  Purge output and send back all acknowledgments.
    */
    npuTipDiscardOutputQ(tp);

    /*
    **  Reset input buffer.
    */
    npuTipInputReset(tp);
    }

/*--------------------------------------------------------------------------
**  Purpose:        Discard the pending output queue, but generate required
**                  acknowledgments.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuTipDiscardOutputQ(Tcb *tp)
    {
    NpuBuffer *bp;

    while ((bp = npuBipQueueExtract(&tp->outputQ)) != NULL)
        {
        if (bp->blockSeqNo != 0)
            {
            blockAck[BlkOffCN] = tp->portNumber;
            blockAck[BlkOffBTBSN] &= BlkMaskBT;
            blockAck[BlkOffBTBSN] |= bp->blockSeqNo;
            npuBipRequestUplineCanned(blockAck, sizeof(blockAck));
            }

        npuBipBufRelease(bp);
        }
    }

/*--------------------------------------------------------------------------
**  Purpose:        Network has sent the data - generate acknowledgement.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**                  blockSeqNo  block sequence number
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuTipNotifySent(Tcb *tp, u8 blockSeqNo)
    {
    blockAck[BlkOffCN] = tp->portNumber;
    blockAck[BlkOffBTBSN] &= BlkMaskBT;
    blockAck[BlkOffBTBSN] |= blockSeqNo;
    npuBipRequestUplineCanned(blockAck, sizeof(blockAck));
    }

/*
**--------------------------------------------------------------------------
**
**  Private Functions
**
**--------------------------------------------------------------------------
*/

/*--------------------------------------------------------------------------
**  Purpose:        Setup CDC 713 defaults (terminal class 2)
**
**  Parameters:     Name        Description.
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
static void npuTipSetupDefaultTc2(void)
    {
    defaultTc2.fvAbortBlock      = 'X' - 0x40;
    defaultTc2.fvBlockFactor     = 1;
    defaultTc2.fvBreakAsUser     = FALSE;
    defaultTc2.fvBS              = ChrBS;
    defaultTc2.fvUserBreak1      = 'P' - 0x40;
    defaultTc2.fvUserBreak2      = 'T' - 0x40;
    defaultTc2.fvEnaXUserBreak   = FALSE;
    defaultTc2.fvCI              = 0;
    defaultTc2.fvCIAuto          = FALSE;
    defaultTc2.fvCN              = 'X' - 0x40;
    defaultTc2.fvCursorPos       = TRUE;
    defaultTc2.fvCT              = ChrESC;
    defaultTc2.fvXCharFlag       = FALSE;
    defaultTc2.fvXCnt            = 2043;
    defaultTc2.fvXChar           = ChrCR;
    defaultTc2.fvXTimeout        = FALSE;
    defaultTc2.fvXModeMultiple   = FALSE;
    defaultTc2.fvEOB             = ChrEOT;
    defaultTc2.fvEOBterm         = 2;
    defaultTc2.fvEOBCursorPos    = 3;
    defaultTc2.fvEOL             = ChrCR;
    defaultTc2.fvEOLTerm         = 1;
    defaultTc2.fvEOLCursorPos    = 2;
    defaultTc2.fvEchoplex        = TRUE;
    defaultTc2.fvFullASCII       = FALSE;
    defaultTc2.fvInFlowControl   = FALSE;
    defaultTc2.fvXInput          = FALSE;
    defaultTc2.fvInputDevice     = 0;
    defaultTc2.fvLI              = 0;
    defaultTc2.fvLIAuto          = FALSE;
    defaultTc2.fvLockKeyboard    = FALSE;
    defaultTc2.fvOutFlowControl  = FALSE;
    defaultTc2.fvOutputDevice    = 1;
    defaultTc2.fvParity          = 2;
    defaultTc2.fvPG              = FALSE;
    defaultTc2.fvPL              = 24;
    defaultTc2.fvPW              = 80;
    defaultTc2.fvSpecialEdit     = FALSE;
    defaultTc2.fvTC              = Tc721;
    defaultTc2.fvXStickyTimeout  = FALSE;
    defaultTc2.fvXModeDelimiter  = 0;
    defaultTc2.fvDuplex          = FALSE;
    defaultTc2.fvTermTransBs     = 1;
    defaultTc2.fvSolicitInput    = FALSE;
    defaultTc2.fvCIDelay         = 0;
    defaultTc2.fvLIDelay         = 0;
    defaultTc2.fvHostNode        = 1;
    defaultTc2.fvAutoConnect     = FALSE;
    defaultTc2.fvPriority        = 1;
    defaultTc2.fvUBL             = 7;
    defaultTc2.fvABL             = 2;
    defaultTc2.fvDBL             = 2;
    defaultTc2.fvDBSize          = 940;
    defaultTc2.fvRestrictedRBF   = 0;
    }

/*--------------------------------------------------------------------------
**  Purpose:        Setup CDC 721 defaults (terminal class 3)
**
**  Parameters:     Name        Description.
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
static void npuTipSetupDefaultTc3(void)
    {
    defaultTc3.fvAbortBlock      = 'X' - 0x40;
    defaultTc3.fvBlockFactor     = 1;
    defaultTc3.fvBreakAsUser     = FALSE;
    defaultTc3.fvBS              = ChrBS;
    defaultTc3.fvUserBreak1      = 'P' - 0x40;
    defaultTc3.fvUserBreak2      = 'T' - 0x40;
    defaultTc3.fvEnaXUserBreak   = FALSE;
    defaultTc3.fvCI              = 0;
    defaultTc3.fvCIAuto          = FALSE;
    defaultTc3.fvCN              = 'X' - 0x40;
    defaultTc3.fvCursorPos       = TRUE;
    defaultTc3.fvCT              = ChrESC;
    defaultTc3.fvXCharFlag       = FALSE;
    defaultTc3.fvXCnt            = 2043;
    defaultTc3.fvXChar           = ChrCR;
    defaultTc3.fvXTimeout        = FALSE;
    defaultTc3.fvXModeMultiple   = FALSE;
    defaultTc3.fvEOB             = ChrEOT;
    defaultTc3.fvEOBterm         = 2;
    defaultTc3.fvEOBCursorPos    = 3;
    defaultTc3.fvEOL             = ChrCR;
    defaultTc3.fvEOLTerm         = 1;
    defaultTc3.fvEOLCursorPos    = 2;
    defaultTc3.fvEchoplex        = TRUE;
    defaultTc3.fvFullASCII       = FALSE;
    defaultTc3.fvInFlowControl   = FALSE;
    defaultTc3.fvXInput          = FALSE;
    defaultTc3.fvInputDevice     = 0;
    defaultTc3.fvLI              = 0;
    defaultTc3.fvLIAuto          = FALSE;
    defaultTc3.fvLockKeyboard    = FALSE;
    defaultTc3.fvOutFlowControl  = FALSE;
    defaultTc3.fvOutputDevice    = 1;
    defaultTc3.fvParity          = 2;
    defaultTc3.fvPG              = FALSE;
    defaultTc3.fvPL              = 24;
    defaultTc3.fvPW              = 80;
    defaultTc3.fvSpecialEdit     = FALSE;
    defaultTc3.fvTC              = Tc721;
    defaultTc3.fvXStickyTimeout  = FALSE;
    defaultTc3.fvXModeDelimiter  = 0;
    defaultTc3.fvDuplex          = FALSE;
    defaultTc3.fvTermTransBs     = 1;
    defaultTc3.fvSolicitInput    = FALSE;
    defaultTc3.fvCIDelay         = 0;
    defaultTc3.fvLIDelay         = 0;
    defaultTc3.fvHostNode        = 1;
    defaultTc3.fvAutoConnect     = FALSE;
    defaultTc3.fvPriority        = 1;
    defaultTc3.fvUBL             = 7;
    defaultTc3.fvABL             = 2;
    defaultTc3.fvDBL             = 2;
    defaultTc3.fvDBSize          = 940;
    defaultTc3.fvRestrictedRBF   = 0;
    }

/*--------------------------------------------------------------------------
**  Purpose:        Setup ANSI X3.64 defaults (VT100 terminal class 7)
**
**  Parameters:     Name        Description.
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
static void npuTipSetupDefaultTc7(void)
    {
    defaultTc7.fvAbortBlock      = 'X' - 0x40;
    defaultTc7.fvBlockFactor     = 1;
    defaultTc7.fvBreakAsUser     = FALSE;
    defaultTc7.fvBS              = ChrBS;
    defaultTc7.fvUserBreak1      = 'P' - 0x40;
    defaultTc7.fvUserBreak2      = 'T' - 0x40;
    defaultTc7.fvEnaXUserBreak   = FALSE;
    defaultTc7.fvCI              = 0;
    defaultTc7.fvCIAuto          = FALSE;
    defaultTc7.fvCN              = 'X' - 0x40;
    defaultTc7.fvCursorPos       = TRUE;
    defaultTc7.fvCT              = '%';
    defaultTc7.fvXCharFlag       = FALSE;
    defaultTc7.fvXCnt            = 2043;
    defaultTc7.fvXChar           = ChrCR;
    defaultTc7.fvXTimeout        = FALSE;
    defaultTc7.fvXModeMultiple   = FALSE;
    defaultTc7.fvEOB             = ChrEOT;
    defaultTc7.fvEOBterm         = 2;
    defaultTc7.fvEOBCursorPos    = 3;
    defaultTc7.fvEOL             = ChrCR;
    defaultTc7.fvEOLTerm         = 1;
    defaultTc7.fvEOLCursorPos    = 2;
    defaultTc7.fvEchoplex        = TRUE;
    defaultTc7.fvFullASCII       = FALSE;
    defaultTc7.fvInFlowControl   = TRUE;
    defaultTc7.fvXInput          = FALSE;
    defaultTc7.fvInputDevice     = 0;
    defaultTc7.fvLI              = 0;
    defaultTc7.fvLIAuto          = FALSE;
    defaultTc7.fvLockKeyboard    = FALSE;
    defaultTc7.fvOutFlowControl  = TRUE;
    defaultTc7.fvOutputDevice    = 1;
    defaultTc7.fvParity          = 2;
    defaultTc7.fvPG              = FALSE;
    defaultTc7.fvPL              = 24;
    defaultTc7.fvPW              = 80;
    defaultTc7.fvSpecialEdit     = FALSE;
    defaultTc7.fvTC              = TcX364;
    defaultTc7.fvXStickyTimeout  = FALSE;
    defaultTc7.fvXModeDelimiter  = 0;
    defaultTc7.fvDuplex          = FALSE;
    defaultTc7.fvTermTransBs     = 1;
    defaultTc7.fvSolicitInput    = FALSE;
    defaultTc7.fvCIDelay         = 0;
    defaultTc7.fvLIDelay         = 0;
    defaultTc7.fvHostNode        = 1;
    defaultTc7.fvAutoConnect     = FALSE;
    defaultTc7.fvPriority        = 1;
    defaultTc7.fvUBL             = 7;
    defaultTc7.fvABL             = 2;
    defaultTc7.fvDBL             = 2;
    defaultTc7.fvDBSize          = 940;
    defaultTc7.fvRestrictedRBF   = 0;
    }

/*---------------------------  End Of File  ------------------------------*/

