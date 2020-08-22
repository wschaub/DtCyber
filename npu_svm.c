/*--------------------------------------------------------------------------
**
**  Copyright (c) 2003-2011, Tom Hunter
**
**  Name: npu_svm.c
**
**  Description:
**      Perform emulation of the Service Message (SVM) subsystem in an NPU
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
**  Primary Service Message function codes.
*/
#define PfcREG          0x1     // logical link regulation 
#define PfcICN          0x2     // initiate connection     
#define PfcTCN          0x3     // terminate connection    
#define PfcCHC          0x4     // change terminal characteristics
#define PfcNPU          0xA     // initialize npu          
#define PfcSUP          0xE     // initiate supervision    
#define PfcCNF          0xF     // configure terminal      
#define PfcENB          0x10    // enable command(s)       
#define PfcDIB          0x11    // disable command(s)      
#define PfcNPS          0x12    // npu status request      
#define PfcLLS          0x13    // ll status request       
#define PfcLIS          0x14    // line status request     
#define PfcTES          0x15    // term status request     
#define PfcTRS          0x16    // trunk status request    
#define PfcCPS          0x17    // coupler status request  
#define PfcVCS          0x18    // svc status request       
#define PfcSTU          0x19    // unsolicated statuses    
#define PfcSTI          0x1A    // statistics              
#define PfcMSG          0x1B    // message(s)             
#define PfcLOG          0x1C    // error log entry         
#define PfcALM          0x1D    // operator alarm          
#define PfcNPI          0x1E    // reload npu               
#define PfcCDI          0x1F    // count(s)                
#define PfcOLD          0x20    // on-line diagnostics     

/*
**  Secondary Service Message function codes.
*/
#define SfcNP           0x0     // npu                     
#define SfcLL           0x1     // logical link            
#define SfcLI           0x2     // line                    
#define SfcTE           0x3     // terminal                
#define SfcTR           0x4     // trunk                   
#define SfcCP           0x5     // coupler                 
#define SfcVC           0x6     // switched virtual circuit
#define SfcOP           0x7     // operator                
#define SfcTA           0x8     // terminate connection    
#define SfcAP           0x9     // outbound a-a connection 
#define SfcIN           0xA     // initiate supervision    
#define SfcDO           0xB     // dump option             
#define SfcPB           0xC     // program block           
#define SfcDT           0xD     // data                    
#define SfcTM           0xE     // terminate diagnostics   
#define SfcLD           0xE     // load                    
#define SfcGO           0xF     // go                      
#define SfcER           0x10    // error(s)                
#define SfcEX           0x11    // a to a connection       
#define SfcNQ           0x12    // sfc for *pbperform* sti 
#define SfcNE           0x13    // nip block protocol error
#define SfcPE           0x14    // pip block protocol error
#define SfcRC           0x11    // reconfigure terminal    

/*
**  Regulation level change bit masks.
*/
#define RegLvlBuffers       0x03
#define RegLvlCsAvailable   0x04
#define RegLvlNsAvailable   0x08

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
static bool npuSvmRequestTerminalConfig(Tcb *tp);
static bool npuSvmProcessTerminalConfig(Tcb *tp, NpuBuffer *bp);
static bool npuSvmRequestTerminalConnection(Tcb *tp);

/*
**  ----------------
**  Public Variables
**  ----------------
*/

/*
**  -----------------
**  Private Variables
**  -----------------
*/
static u8 linkRegulation[] =
    {
    AddrHost,           // DN
    AddrNpu,            // SN
    0,                  // CN
    4,                  // BT=CMD
    PfcREG,             // PFC
    SfcLL,              // SFC
    0x0F,               // NS=1, CS=1, Regulation level=3
    0,0,0,0,            // not used
    0,0,0,              // not used
    };

static u8 requestSupervision[] =
    {
    AddrHost,           // DN
    AddrNpu,            // SN
    0,                  // CN
    4,                  // BT=CMD
    PfcSUP,             // PFC
    SfcIN,              // SFC
    0,                  // PS
    0,                  // PL
    0,                  // RI
    0,0,0,              // not used
    3,                  // CCP version
    1,                  // ...
    0,                  // CCP level
    0,                  // ...
    0,                  // CCP cycle or variant
    0,                  // ...
    0,                  // not used
    0,0                 // NCF version in NDL file (ignored)
    };

static u8 responseNpuStatus[] =
    {
    AddrHost,           // DN
    AddrNpu,            // SN
    0,                  // CN
    4,                  // BT=CMD
    PfcNPS,             // PFC
    SfcNP | SfcResp,    // SFC
    };

static u8 responseTerminateConnection[] =
    {
    AddrHost,           // DN
    AddrNpu,            // SN
    0,                  // CN
    4,                  // BT=CMD
    PfcTCN,             // PFC
    SfcTA | SfcResp,    // SFC
    0,                  // CN
    };

static u8 requestTerminateConnection[] =
    {
    AddrHost,           // DN
    AddrNpu,            // SN
    0,                  // CN
    4,                  // BT=CMD
    PfcTCN,             // PFC
    SfcTA,              // SFC
    0,                  // CN
    };

static enum
    {
    StIdle,
    StWaitSupervision,
    StReady,
    } svmState = StIdle;

static u8 oldRegLevel = 0;

/*
**--------------------------------------------------------------------------
**
**  Public Functions
**
**--------------------------------------------------------------------------
*/

/*--------------------------------------------------------------------------
**  Purpose:        Initialize SVM.
**
**  Parameters:     Name        Description.
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuSvmInit(void)
    {
    /*
    **  Set initial state.
    */
    svmState = StIdle;
    }

/*--------------------------------------------------------------------------
**  Purpose:        Reset SVM.
**
**  Parameters:     Name        Description.
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuSvmReset(void)
    {
    /*
    **  Set initial state.
    */
    svmState = StIdle;
    oldRegLevel = 0;
    }

/*--------------------------------------------------------------------------
**  Purpose:        Process regulation order word.
**
**  Parameters:     Name        Description.
**                  regLevel    regulation level
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuSvmNotifyHostRegulation(u8 regLevel)
    {
    if (svmState == StIdle || regLevel != oldRegLevel)
        {
        oldRegLevel = regLevel;
        linkRegulation[BlkOffP3] = regLevel;
        npuBipRequestUplineCanned(linkRegulation, sizeof(linkRegulation));
        }

    if (svmState == StIdle && (regLevel & RegLvlCsAvailable) != 0)
        {
        npuBipRequestUplineCanned(requestSupervision, sizeof(requestSupervision));
        svmState = StWaitSupervision;
        }
    }

/*--------------------------------------------------------------------------
**  Purpose:        Start host connection sequence.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**
**  Returns:        TRUE if sequence started, FALSE otherwise.
**
**------------------------------------------------------------------------*/
bool npuSvmConnectTerminal(Tcb *tp)
    {
    if (npuSvmRequestTerminalConfig(tp))
        {
        tp->state = StTermRequestConfig;
        return(TRUE);
        }

    return(FALSE);
    }

/*--------------------------------------------------------------------------
**  Purpose:        Process service message from host.
**
**  Parameters:     Name        Description.
**                  bp          buffer with service message.
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuSvmProcessBuffer(NpuBuffer *bp)
    {
    u8 *block = bp->data;
    Tcb *tp;
    u8 cn;

    /*
    **  Ensure there is at least a minimal service message.
    */
    if (bp->numBytes < BlkOffSfc + 1)
        {
        if (bp->numBytes == BlkOffBTBSN + 1 && block[BlkOffCN] != 0)
            {
            /*
            **  Exception to minimal service message:
            **  For some strange reason NAM sends an input acknowledgment
            **  as a SVM - forward it to the TIP which is better equipped
            **  to deal with this.
            */
            npuTipProcessBuffer(bp, 0);
            return;
            }

        /*
        **  Service message must be at least DN/SN/0/BSN/PFC/SFC.
        */
        npuLogMessage("Short SVM message in state %d", svmState);

        /*
        **  Release downline buffer and return.
        */
        npuBipBufRelease(bp);
        return;
        }

    /*
    **  Connection number for all service messages must be zero.
    */
    cn = block[BlkOffCN];
    if (cn != 0)
        {
        /*
        **  Connection number out of range.
        */
        npuLogMessage("Connection number is %u but must be zero in SVM messages %02X/%02X", cn, block[BlkOffPfc], block[BlkOffSfc]);

        /*
        **  Release downline buffer and return.
        */
        npuBipBufRelease(bp);
        return;
        }

    /*
    **  Extract the true connection number for those messages which carry it in P3.
    */
    switch (block[BlkOffPfc])
        {
    case PfcCNF:
    case PfcICN:
    case PfcTCN:
        if (bp->numBytes < BlkOffP3 + 1)
            {
            /*
            **  Message too short.
            */
            npuLogMessage("SVM message %02X/%02X is too short and has no required P3", block[BlkOffPfc], block[BlkOffSfc]);
            npuBipBufRelease(bp);
            return;
            }

        cn = block[BlkOffP3];
        if (cn == 0 || cn > npuTcbCount)
            {
            /*
            **  Port number out of range.
            */
            npuLogMessage("Unexpected port number %u in SVM message %02X/%02X", cn, block[BlkOffPfc], block[BlkOffSfc]);
            npuBipBufRelease(bp);
            return;
            }

        tp = npuTcbs + cn - 1;
        break;
        }

    /*
    **  Process message.
    */
    switch (block[BlkOffPfc])
        {
    case PfcSUP:
        if (block[BlkOffSfc] == (SfcIN | SfcResp))
            {
            if (svmState != StWaitSupervision)
                {
                npuLogMessage("Unexpected Supervision Reply in state %d", svmState);
                break;
                }

            /*
            **  Host (CS) has agreed to supervise us, we are now ready to handle network
            **  connection attempts.
            */
            svmState = StReady;
            }
        else
            {
            npuLogMessage("Unexpected SVM message %02X/%02X in state %d", block[BlkOffPfc], block[BlkOffSfc], svmState);
            }

        break;

    case PfcNPS:
        if (block[BlkOffSfc] == SfcNP)
            {
            npuBipRequestUplineCanned(responseNpuStatus, sizeof(responseNpuStatus));
            }
        else
            {
            npuLogMessage("Unexpected SVM message %02X/%02X in state %d", block[BlkOffPfc], block[BlkOffSfc], svmState);
            }

        break;

    case PfcCNF:
        if (tp->state != StTermRequestConfig)
            {
            npuLogMessage("Unexpected Terminal Configuration Reply in state %d", tp->state);
            break;
            }

        if (block[BlkOffSfc] == (SfcTE | SfcResp))
            {
            /*
            **  Process configuration reply and if all is well, issue
            **  terminal connection request.
            */
            if (   npuSvmProcessTerminalConfig(tp, bp)
                && npuSvmRequestTerminalConnection(tp))
                {
                tp->state = StTermRequestConnection;
                }
            else
                {
                npuNetDisconnected(tp);
                }
            }
        else if (block[BlkOffSfc] == (SfcTE | SfcErr))
            {
            /*
            **  This port appears to be unknown to the host.
            */
            npuLogMessage("Terminal on port %d not configured", cn);
            npuNetDisconnected(tp);
            }
        else
            {
            npuLogMessage("Unexpected SVM message %02X/%02X with CN %d", block[BlkOffPfc], block[BlkOffSfc], cn);
            npuNetDisconnected(tp);
            }

        break;

    case PfcICN:
        if (tp->state != StTermRequestConnection)
            {
            npuLogMessage("Unexpected Terminal Connection Reply in state %d", tp->state);
            break;
            }

        if (block[BlkOffSfc] == (SfcTE | SfcResp))
            {
            npuNetConnected(tp);
            }
        else if (block[BlkOffSfc] == (SfcTE | SfcErr))
            {
            npuLogMessage("Terminal Connection Rejected - reason 0x%02X", block[BlkOffP4]);
            npuNetDisconnected(tp);
            }
        else
            {
            npuLogMessage("Unexpected SVM message %02X/%02X with CN %d", block[BlkOffPfc], block[BlkOffSfc], cn);
            npuNetDisconnected(tp);
            }

        break;

    case PfcTCN:
        if (block[BlkOffSfc] == SfcTA)
            {
            /*
            **  Terminate connection from host.
            */
            npuTipTerminateConnection(tp);
            }
        else if (block[BlkOffSfc] == (SfcTA | SfcResp))
            {
            if (tp->state == StTermNpuDisconnect)
                {
                /*
                **  Reset connection state.
                */
                tp->state = StTermIdle;
                }
            }
        else
            {
            npuLogMessage("Unexpected SVM message %02X/%02X with CN %d", block[BlkOffPfc], block[BlkOffSfc], cn);
            }

        break;
        }

    /*
    **  Release downline buffer.
    */
    npuBipBufRelease(bp);
    }

/*--------------------------------------------------------------------------
**  Purpose:        Cleanup and send a TCN/TA/R to host.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**
**  Returns:        Nothing
**
**------------------------------------------------------------------------*/
void npuSvmDiscRequestTerminal(Tcb *tp)
    {
    if (tp->state == StTermHostConnected)
        {
        /*
        **  Clean up flow control state and discard any pending output.
        */
        tp->xoff = FALSE;
        npuTipDiscardOutputQ(tp);
        tp->state = StTermNpuDisconnect;

        /*
        **  Send the TCN/TA/R message.
        */
        requestTerminateConnection[BlkOffP3] = tp->portNumber;
        npuBipRequestUplineCanned(requestTerminateConnection, sizeof(requestTerminateConnection));
        }
    else
        {
        /*
        **  Just reset the state to idle.
        */
        tp->state = StTermIdle;
        }
    }

/*--------------------------------------------------------------------------
**  Purpose:        Send a TCN/TA/N to host.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**
**  Returns:        Nothing
**
**------------------------------------------------------------------------*/
void npuSvmDiscReplyTerminal(Tcb *tp)
    {
    responseTerminateConnection[BlkOffP3] = tp->portNumber;
    npuBipRequestUplineCanned(responseTerminateConnection, sizeof(responseTerminateConnection));
    }

/*--------------------------------------------------------------------------
**  Purpose:        Determine if host is ready for connection request
**
**  Parameters:     Name        Description.
**
**  Returns:        TRUE if host is ready to accept connections, FALSE
**                  otherwise.
**
**------------------------------------------------------------------------*/
bool npuSvmIsReady(void)
    {
    return(svmState == StReady);
    }

/*
**--------------------------------------------------------------------------
**
**  Private Functions
**
**--------------------------------------------------------------------------
*/

/*--------------------------------------------------------------------------
**  Purpose:        Send terminal configuration request to host.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
static bool npuSvmRequestTerminalConfig(Tcb *tp)
    {
    NpuBuffer *bp;
    u8 *mp;

    bp = npuBipBufGet();
    if (bp == NULL)
        {
        return(FALSE);
        }

    /*
    **  Assemble configure request.
    */
    mp = bp->data;

    *mp++ = AddrHost;           // DN
    *mp++ = AddrNpu;            // SN
    *mp++ = 0;                  // CN
    *mp++ = 4;                  // BT=CMD
    *mp++ = PfcCNF;             // PFC
    *mp++ = SfcTE;              // SFC
    *mp++ = tp->portNumber;     // non-zero port number from "PORT=" parameter in NDL source
    *mp++ = 0;                  // sub-port number (always 0 for async ports)
    *mp++ = (0 << 7) | (tp->tipType << 3); // no auto recognition; TIP type; subtype 0

    bp->numBytes = mp - bp->data;

    /*
    **  Send the request.
    */
    npuBipRequestUplineTransfer(bp);

    return(TRUE);
    }

/*--------------------------------------------------------------------------
**  Purpose:        Process terminal configuration reply from host.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**                  bp          buffer with service message.
**
**  Returns:        TRUE if successful, FALSE otherwise.
**
**------------------------------------------------------------------------*/
static bool npuSvmProcessTerminalConfig(Tcb *tp, NpuBuffer *bp)
    {
    u8 *mp = bp->data;
    int len = bp->numBytes;
    u8 port;
    u8 subPort;
    u8 address1;
    u8 address2;
    u8 deviceType;
    u8 subTip;
    u8 termName[7];
    u8 termClass;
    u8 status;
    u8 lastResp;
    u8 codeSet;

    /*
    **  Extract configuration.
    */
    mp += BlkOffP3;
    port        = *mp++;
    subPort     = *mp++;
    address1    = *mp++;
    address2    = *mp++;
    deviceType  = *mp++;
    subTip      = *mp++;

    memcpy(termName, mp, sizeof(termName));
    mp += sizeof(termName);

    termClass   = *mp++;
    status      = *mp++;
    lastResp    = *mp++;
    codeSet     = *mp++;

    /*
    **  Verify minimum length;
    */
    len -= mp - bp->data;
    if (len < 0)
        {
        npuLogMessage("Short Terminal Configuration response with length %d", bp->numBytes);
        return(FALSE);
        }

    /*
    **  Setup default operating parameters for the specified terminal class.
    */
    npuTipSetupTerminalClass(tp, termClass);

    /*
    **  Setup TCB with supported FN/FV values.
    */
    npuTipParseFnFv(mp, len, tp);

    /*
    **  Transfer configuration to TCB.
    */
    tp->enabled = status == 0;
    memcpy(tp->termName, termName, sizeof(termName));
    tp->deviceType = deviceType;
    tp->subTip = subTip;
    tp->codeSet = codeSet;
    tp->params.fvTC = termClass;

    /*
    **  Reset user break 2 status.
    */
    tp->breakPending = FALSE;

    return(TRUE);
    }

/*--------------------------------------------------------------------------
**  Purpose:        Send connect request to host.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**
**  Returns:        TRUE if request sent, FALSE otherwise.
**
**------------------------------------------------------------------------*/
static bool npuSvmRequestTerminalConnection(Tcb *tp)
    {
    NpuBuffer *bp;
    u8 *mp;

    bp = npuBipBufGet();
    if (bp == NULL)
        {
        return(FALSE);
        }

    /*
    **  Assemble connect request.
    */
    mp = bp->data;

    *mp++ = AddrHost;               // DN
    *mp++ = AddrNpu;                // SN
    *mp++ = 0;                      // CN
    *mp++ = 4;                      // BT=CMD
    *mp++ = PfcICN;                 // PFC
    *mp++ = SfcTE;                  // SFC
    *mp++ = tp->portNumber;         // CN
    *mp++ = tp->params.fvTC;        // TC
    *mp++ = tp->params.fvPL;        // page length
    *mp++ = tp->params.fvPW;        // page width
    *mp++ = tp->deviceType;         // device type
    *mp++ = 3;                      // downline block limit

    memcpy(mp, tp->termName, 7);    // terminal name
    mp += 7;                        

    *mp++ = 3;                      // application block limit
    *mp++ = 0x07;                   // application block size
    *mp++ = 0x00;                   // ...
    *mp++ = 0;                      // auto login flag
    *mp++ = 0;                      // device ordinal
    *mp++ = 0x07;                   // transmission block size
    *mp++ = 0x00;                   // ...
    *mp++ = 0;                      // sub device type
                                    
    memcpy(mp, tp->termName, 7);    // owning console
    mp += 7;                        
                                    
//    *mp++ = 0;                      // security level
    *mp++ = 7;                      // security level
    *mp++ = tp->params.fvPriority;  // priority
    *mp++ = 1;                      // interactive capability
//    *mp++ = tp->params.fvEchoplex;  // echoplex
    *mp++ = 1;                      // echoplex
    *mp++ = 100;                    // upline block size
    *mp++ = 1;                      // hardwired flag
    *mp++ = 0;                      // VTP ???
    *mp++ = 0;                      // DTE address length

    bp->numBytes = mp - bp->data;

    /*
    **  Send the request.
    */
    npuBipRequestUplineTransfer(bp);

    return(TRUE);
    }

/*---------------------------  End Of File  ------------------------------*/

