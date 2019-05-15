/*--------------------------------------------------------------------------
**
**  Copyright (c) 2003-2011, Tom Hunter, Paul Koning
**
**  Name: npu_async.c
**
**  Description:
**      Perform emulation of the ASYNC TIP in an NPU consisting of a
**      CDC 2550 HCP running CCP.
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
#define MaxIvtData          100

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
static void npuAsyncDoFeBefore(u8 fe);
static void npuAsyncDoFeAfter(u8 fe);
static void npuAsyncProcessUplineTransparent(Tcb *tp);
static void npuAsyncProcessUplineAscii(Tcb *tp);
static void npuAsyncProcessUplineSpecial(Tcb *tp);
static void npuAsyncProcessUplineNormal(Tcb *tp);

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
static Tcb *npuTp;

static u8 fcSingleSpace[] = "\r\n";
static u8 fcDoubleSpace[] = "\r\n\n";
static u8 fcTripleSpace[] = "\r\n\n\n";
static u8 fcBol[] = "\r";
static u8 fcTofAnsi[] = "\r\n\033[H";
static u8 fcTof[] = "\f";
static u8 fcClearHomeAnsi[] = "\r\n\033[H\033[J";

static u8 netBEL[] = {ChrBEL};
static u8 netLF[] = {ChrLF};
static u8 netCR[] = {ChrCR};
static u8 netCRLF[] = {ChrCR, ChrLF};
static u8 echoBuffer[1000];
static u8 *echoPtr;
static int echoLen;

/*
**--------------------------------------------------------------------------
**
**  Public Functions
**
**--------------------------------------------------------------------------
*/

/*--------------------------------------------------------------------------
**  Purpose:        Process downline data from host.
**
**  Parameters:     Name        Description.
**                  cn          connection number
**                  bp          buffer with downline data message.
**                  last        last buffer.
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuAsyncProcessDownlineData(u8 cn, NpuBuffer *bp, bool last)
    {
    u8 *blk = bp->data + BlkOffData;
    int len = bp->numBytes - BlkOffData;
    u8 dbc;
    u8 fe;
    int textlen;
    u8 *ptrUS;

    /*
    **  Locate TCB dealing with this connection.
    */
    if (cn == 0 || cn > npuTcbCount)
        {
        npuLogMessage("ASYNC: unexpected CN %d - message ignored", cn);
        return;
        }

    npuTp = npuTcbs + cn - 1;

    /*
    **  Extract Data Block Clarifier settings.
    */
    dbc = *blk++;
    len -= 1;
    npuTp->dbcNoEchoplex  = (dbc & DbcEchoplex)    != 0;
    npuTp->dbcNoCursorPos = (dbc & DbcNoCursorPos) != 0;

    if ((dbc & DbcTransparent) != 0)
        {
        npuNetSend(npuTp, blk, len);
        npuNetQueueAck(npuTp, (u8)(bp->data[BlkOffBTBSN] & (BlkMaskBSN << BlkShiftBSN)));
        return;
        }

    /*
    **  Process data.
    */
    while (len > 0)
        {
        if ((dbc & DbcNoFe) != 0)
            {
            /*
            **  Format effector is supressed - output is single-spaced.
            */
            fe = ' ';
            }
        else
            {
            fe = *blk++;
            len -= 1;
            }

        /*
        **  Process leading format effector.
        */
        npuAsyncDoFeBefore(fe);

        if (len == 0)
            {
            /*
            **  End-of-data.
            */
            break;
            }

        /*
        **  Locate the US byte which defines the end-of-line.
        */
        ptrUS = memchr(blk, ChrUS, len);
        if (ptrUS == NULL)
            {
            /*
            **  No US byte in the rest of the buffer, so send the entire
            **  rest to the terminal.
            */
            npuNetSend(npuTp, blk, len);
            break;
            }

        /*
        **  Send the line.
        */
        textlen = ptrUS - blk;
        npuNetSend(npuTp, blk, textlen);

        /*
        **  Process trailing format effector.
        */
        if ((dbc & DbcNoCursorPos) == 0)
            {
            npuAsyncDoFeAfter(fe);
            }

        /*
        **  Advance pointer past the US byte and adjust the length.
        */
        blk += textlen + 1;
        len -= textlen + 1;
        }

    npuNetQueueAck(npuTp, (u8)(bp->data[BlkOffBTBSN] & (BlkMaskBSN << BlkShiftBSN)));
    }

/*--------------------------------------------------------------------------
**  Purpose:        Process upline data from terminal.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuAsyncProcessUplineData(Tcb *tp)
    {
    echoPtr = echoBuffer;

    if (tp->params.fvXInput)
        {
        npuAsyncProcessUplineTransparent(tp);
        }
    else if (tp->params.fvFullASCII)
        {
        npuAsyncProcessUplineAscii(tp);
        }
    else if (tp->params.fvSpecialEdit)
        {
        npuAsyncProcessUplineSpecial(tp);
        }
    else
        {
        npuAsyncProcessUplineNormal(tp);
        }

    /*
    **  Optionally echo characters.
    */
    if (!tp->dbcNoEchoplex)
        {
        echoLen = echoPtr - echoBuffer;
        if (echoLen)
            {
            npuNetSend(tp, echoBuffer, echoLen);
            }
        }
    }

/*--------------------------------------------------------------------------
**  Purpose:        Flush transparent upline data from terminal.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
void npuAsyncFlushUplineTransparent(Tcb *tp)
    {
    if (!tp->params.fvXStickyTimeout)
        {
        /*
        **  Terminate transparent mode unless sticky timeout has been selected.
        */
        tp->params.fvXInput = FALSE;
        }

    /*
    **  Send the upline data.
    */
    tp->inBuf[BlkOffDbc] = DbcTransparent;
    npuBipRequestUplineCanned(tp->inBuf, tp->inBufPtr - tp->inBuf);
    npuTipInputReset(tp);
    tp->xInputTimerRunning = FALSE;
    }

/*
**--------------------------------------------------------------------------
**
**  Private Functions
**
**--------------------------------------------------------------------------
*/

/*--------------------------------------------------------------------------
**  Purpose:        Process format effector at start of line
**
**  Parameters:     Name        Description.
**                            format effector
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
static void npuAsyncDoFeBefore(u8 fe)
    {
    switch (fe)
        {
    case ' ':
        if (npuTp->lastOpWasInput)
            {
            npuNetSend(npuTp, fcBol, sizeof(fcBol) - 1);
            }
        else
            {
            npuNetSend(npuTp, fcSingleSpace, sizeof(fcSingleSpace) - 1);
            }
        break;

    case '0':
        if (npuTp->lastOpWasInput)
            {
            npuNetSend(npuTp, fcSingleSpace, sizeof(fcSingleSpace) - 1);
            }
        else
            {
            npuNetSend(npuTp, fcDoubleSpace, sizeof(fcDoubleSpace) - 1);
            }
        break;

    case '-':
        if (npuTp->lastOpWasInput)
            {
            npuNetSend(npuTp, fcDoubleSpace, sizeof(fcDoubleSpace) - 1);
            }
        else
            {
            npuNetSend(npuTp, fcTripleSpace, sizeof(fcTripleSpace) - 1);
            }
        break;

    case '+':
        npuNetSend(npuTp, fcBol, sizeof(fcBol) - 1);
        break;

    case '*':
        if (npuTp->params.fvTC == TcX364)
            {
            /*
            **  Cursor Home (using ANSI/VT100 control sequences) for VT100.
            */
            npuNetSend(npuTp, fcTofAnsi, sizeof(fcTofAnsi) - 1);
            }
        else
            {
            /*
            **  Formfeed for any other terminal.
            */
            npuNetSend(npuTp, fcTof, sizeof(fcTof) - 1);
            }

        break;

    case '1':
        if (npuTp->params.fvTC == TcX364)
            {
            /*
            **  Cursor Home and Clear (using ANSI/VT100 control sequences) for VT100.
            */
            npuNetSend(npuTp, fcClearHomeAnsi, sizeof(fcClearHomeAnsi) - 1);
            }
        else
            {
            /*
            **  Formfeed for any other terminal.
            */
            npuNetSend(npuTp, fcTof, sizeof(fcTof) - 1);
            }

        break;

    case ',':
        /*
        **  Do not change position.
        */
        break;
        }

    npuTp->lastOpWasInput = FALSE;
    }

/*--------------------------------------------------------------------------
**  Purpose:        Process format effector at end of line
**
**  Parameters:     Name        Description.
**                  fe          format effector
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
static void npuAsyncDoFeAfter(u8 fe)
    {
    switch (fe)
        {
    case '.':
        npuNetSend(npuTp, fcSingleSpace, sizeof(fcSingleSpace) - 1);
        break;

    case '/':
        npuNetSend(npuTp, fcBol, sizeof(fcBol) - 1);
        break;
        }
    }

/*--------------------------------------------------------------------------
**  Purpose:        Process upline data from terminal.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
static void npuAsyncProcessUplineTransparent(Tcb *tp)
    {
    u8 *dp;
    int len;
    u8 ch;

    dp = tp->inputData;
    len = tp->inputCount;

    /*
    **  Cancel transparent input forwarding timeout.
    */
    tp->xInputTimerRunning = FALSE;

    /*
    **  Process transparent input.
    */
    while (len--)
        {
        ch = *dp++;

        if (tp->params.fvEchoplex)
            {
            *echoPtr++ = ch;
            }

        /* CYBIS wants transparent mode plus the ability to turn xon/xoff on and off at will
         * after digging through all the manuals I see nothing that gives a blessing to do this
         * but I see nothing that forbids it either, I choose to interpret what PNI does as legal */
        if(tp->params.fvOutFlowControl) {
            if( (ch == ChrDC1 || ch == ChrDC3) ){
                if(ch == ChrDC1)
                    tp->xoff = FALSE; /* XON turn output on */
                else
                    tp->xoff = TRUE; /* XOFF turn output off */
                fprintf(stderr,"flow control handled in transparent mode! %x, portnumber = %d connfd = %d\n ",ch,tp->portNumber,tp->connFd);
                continue; /* swallow xon/xoff char */
            }
        }

        if (tp->params.fvXCharFlag && ch == tp->params.fvXChar)
            {
            if (!tp->params.fvXModeMultiple)
                {
                /*
                **  Terminate single message transparent mode.
                */
                tp->params.fvXInput = FALSE;
                }

            /*
            **  Send the upline data.
            */
            tp->inBuf[BlkOffDbc] = DbcTransparent;
            npuBipRequestUplineCanned(tp->inBuf, tp->inBufPtr - tp->inBuf);
            npuTipInputReset(tp);
            }
        else if (ch == tp->params.fvUserBreak2 && tp->params.fvEnaXUserBreak)
            {
            *tp->inBufPtr++ = ch;
            tp->inBuf[BlkOffDbc] = DbcTransparent;
            npuBipRequestUplineCanned(tp->inBuf, tp->inBufPtr - tp->inBuf);
            npuTipInputReset(tp);
            }
        else
            {
            *tp->inBufPtr++ = ch;
            if (   tp->inBufPtr - tp->inBufStart >= tp->params.fvXCnt
                || tp->inBufPtr - tp->inBufStart >= MaxBuffer - BlkOffDbc - 2)
                {
                if (!tp->params.fvXModeMultiple)
                    {
                    /*
                    **  Terminate single message transparent mode.
                    */
                    tp->params.fvXInput = FALSE;
                    }

                /*
                **  Send the upline data.
                */
                tp->inBuf[BlkOffDbc] = DbcTransparent;
                npuBipRequestUplineCanned(tp->inBuf, tp->inBufPtr - tp->inBuf);
                npuTipInputReset(tp);
                }
            }
        }

    /*
    **  If data is pending, schedule transparent input forwarding timeout.
    */
    if (tp->params.fvXTimeout && tp->inBufStart != tp->inBufPtr)
        {
        tp->xStartCycle = cycles;
        tp->xInputTimerRunning = TRUE;
        }
    }

/*--------------------------------------------------------------------------
**  Purpose:        Process upline data from terminal.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
static void npuAsyncProcessUplineAscii(Tcb *tp)
    {
    u8 *dp;
    int len;
    u8 ch;

    dp = tp->inputData;
    len = tp->inputCount;

    /*
    **  Process normalised input.
    */
    tp->inBuf[BlkOffDbc] = 0;   // non-transparent data

    while (len--)
        {
        ch = *dp++ & Mask7;

        /*
        **  Ignore the following characters when at the begin of a line.
        */
        if (tp->inBufPtr - tp->inBufStart == 0)
            {
            switch (ch)
                {
            case ChrNUL:
            case ChrLF:
            case ChrDEL:
                continue;
                }
            }

        if (   (ch == ChrDC1 || ch == ChrDC3)
            && tp->params.fvOutFlowControl)
            {
            /*
            **  Flow control characters if enabled.
            */
            if (ch == ChrDC1)
                {
                /*
                **  XON (turn output on)
                */
                tp->xoff = FALSE;
                }
            else
                {
                /* 
                **  XOFF (turn output off)
                */
                tp->xoff = TRUE;
                }

            continue;
            }

        if (   ch == tp->params.fvCN
            || ch == tp->params.fvEOL)
            {
            /*
            **  EOL or Cancel entered - send the input upline.
            */
            *tp->inBufPtr++ = ch;
            npuBipRequestUplineCanned(tp->inBuf, tp->inBufPtr - tp->inBuf);
            npuTipInputReset(tp);

            /*
            **  Optionally echo characters.
            */
            if (tp->dbcNoEchoplex)
                {
                /*
                **  DBC prevented echoplex for this line.
                */
                tp->dbcNoEchoplex = FALSE;
                echoPtr = echoBuffer;
                }
            else
                {
                echoLen = echoPtr - echoBuffer;
                if (echoLen)
                    {
                    npuNetSend(tp, echoBuffer, echoLen);
                    echoPtr = echoBuffer;
                    }
                }

            /*
            **  Perform cursor positioning.
            */
            if (tp->dbcNoCursorPos)
                {
                tp->dbcNoCursorPos = FALSE;
                }
            else
                {
                if (tp->params.fvCursorPos)
                    {
                    switch (tp->params.fvEOLCursorPos)
                        {
                    case 0:
                        break;

                    case 1:
                        *echoPtr++ = ChrCR;
                        break;

                    case 2:
                        *echoPtr++ = ChrLF;
                        break;

                    case 3:
                        *echoPtr++ = ChrCR;
                        *echoPtr++ = ChrLF;
                        break;
                        }
                    }
                }

            continue;
            }

        if (tp->params.fvEchoplex)
            {
            *echoPtr++ = ch;
            }

        /*
        **  Store the character for later transmission.
        */
        *tp->inBufPtr++ = ch;

        if (tp->inBufPtr - tp->inBufStart >= (tp->params.fvBlockFactor * MaxIvtData))
            {
            /*
            **  Send long lines.
            */
            tp->inBuf[BlkOffBTBSN] = BtHTBLK | (tp->uplineBsn << BlkShiftBSN);
            npuBipRequestUplineCanned(tp->inBuf, tp->inBufPtr - tp->inBuf);
            npuTipInputReset(tp);
            }
        }
    }

/*--------------------------------------------------------------------------
**  Purpose:        Process upline data from terminal.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
static void npuAsyncProcessUplineSpecial(Tcb *tp)
    {
    u8 *dp;
    int len;
    u8 ch;
    int i;
    int cnt;

    dp = tp->inputData;
    len = tp->inputCount;

    /*
    **  Process normalised input.
    */
    tp->inBuf[BlkOffDbc] = 0;   // non-transparent data

    while (len--)
        {
        ch = *dp++ & Mask7;

        /*
        **  Ignore the following characters.
        */
        switch (ch)
            {
        case ChrNUL:
        case ChrDEL:
            continue;
            }

        /*
        **  Ignore the following characters when at the begin of a line.
        */
        if (tp->inBufPtr - tp->inBufStart == 0)
            {
            switch (ch)
                {
            case ChrSTX:
                continue;
                }
            }

        if (   (ch == ChrDC1 || ch == ChrDC3)
            && tp->params.fvOutFlowControl)
            {
            /*
            **  Flow control characters if enabled.
            */
            if (ch == ChrDC1)
                {
                /*
                **  XON (turn output on)
                */
                tp->xoff = FALSE;
                }
            else
                {
                /* 
                **  XOFF (turn output off)
                */
                tp->xoff = TRUE;
                }

            continue;
            }

        if (ch == tp->params.fvCN)
            {
            /*
            **  Cancel character entered - erase all characters entered
            **  and indicate it to user via "*DEL*". Use the echobuffer
            **  to build and send the sequence.
            */
            echoPtr = echoBuffer;
            cnt = tp->inBufPtr - tp->inBufStart;
            for (i = cnt; i > 0; i--)
                {
                *echoPtr++ = ChrBS;
                }

            for (i = cnt; i > 0; i--)
                {
                *echoPtr++ = ' ';
                }

            for (i = cnt; i > 0; i--)
                {
                *echoPtr++ = ChrBS;
                }

            *echoPtr++ = '*';
            *echoPtr++ = 'D';
            *echoPtr++ = 'E';
            *echoPtr++ = 'L';
            *echoPtr++ = '*';
            *echoPtr++ = '\r';
            *echoPtr++ = '\n';
            npuNetSend(tp, echoBuffer, echoPtr - echoBuffer);

            /*
            **  Send the line, but signal the cancel character.
            */
            tp->inBuf[BlkOffDbc] = DbcCancel;
            npuBipRequestUplineCanned(tp->inBuf, tp->inBufPtr - tp->inBuf);

            /*
            **  Reset input and echoplex buffers.
            */
            npuTipInputReset(tp);
            echoPtr = echoBuffer;
            continue;
            }

        if (ch == tp->params.fvUserBreak1)
            {
            /*
            **  User break 1 (typically Ctrl-I).
            */
            npuTipSendUserBreak(tp, 1);
            continue;
            }

        if (ch == tp->params.fvUserBreak2)
            {
            /*
            **  User break 2 (typically Ctrl-T).
            */
            npuTipSendUserBreak(tp, 2);
            continue;
            }

        if (tp->params.fvEchoplex)
            {
            *echoPtr++ = ch;
            }

        if (ch == tp->params.fvEOL)
            {
            /*
            **  EOL entered - send the input upline.
            */
            npuBipRequestUplineCanned(tp->inBuf, tp->inBufPtr - tp->inBuf);
            npuTipInputReset(tp);

            /*
            **  Optionally echo characters.
            */
            if (tp->dbcNoEchoplex)
                {
                /*
                **  DBC prevented echoplex for this line.
                */
                tp->dbcNoEchoplex = FALSE;
                echoPtr = echoBuffer;
                }
            else
                {
                echoLen = echoPtr - echoBuffer;
                if (echoLen)
                    {
                    npuNetSend(tp, echoBuffer, echoLen);
                    echoPtr = echoBuffer;
                    }
                }

            /*
            **  Perform cursor positioning.
            */
            if (tp->dbcNoCursorPos)
                {
                tp->dbcNoCursorPos = FALSE;
                }
            else
                {
                if (tp->params.fvCursorPos)
                    {
                    switch (tp->params.fvEOLCursorPos)
                        {
                    case 0:
                        break;

                    case 1:
                        npuNetSend(tp, netCR, sizeof(netCR));
                        break;

                    case 2:
                        npuNetSend(tp, netLF, sizeof(netLF));
                        break;

                    case 3:
                        npuNetSend(tp, netCRLF, sizeof(netCRLF));
                        break;
                        }
                    }
                }

            continue;
            }

        /*
        **  Store the character for later transmission.
        */
        *tp->inBufPtr++ = ch;

        if (tp->inBufPtr - tp->inBufStart >= (tp->params.fvBlockFactor * MaxIvtData))
            {
            /*
            **  Send long lines.
            */
            tp->inBuf[BlkOffBTBSN] = BtHTBLK | (tp->uplineBsn << BlkShiftBSN);
            npuBipRequestUplineCanned(tp->inBuf, tp->inBufPtr - tp->inBuf);
            npuTipInputReset(tp);
            }
        }
    }

/*--------------------------------------------------------------------------
**  Purpose:        Process upline data from terminal.
**
**  Parameters:     Name        Description.
**                  tp          TCB pointer
**
**  Returns:        Nothing.
**
**------------------------------------------------------------------------*/
static void npuAsyncProcessUplineNormal(Tcb *tp)
    {
    u8 *dp;
    int len;
    u8 ch;
    int i;
    int cnt;

    dp = tp->inputData;
    len = tp->inputCount;

    /*
    **  Process normalised input.
    */
    tp->inBuf[BlkOffDbc] = 0;   // non-transparent data

    while (len--)
        {
        ch = *dp++ & Mask7;

        /*
        **  Ignore the following characters.
        */
        switch (ch)
            {
        case ChrNUL:
        case ChrLF:
        case ChrDEL:
            continue;
            }

        if (   (ch == ChrDC1 || ch == ChrDC3)
            && tp->params.fvOutFlowControl)
            {
            /*
            **  Flow control characters if enabled.
            */
            if (ch == ChrDC1)
                {
                /*
                **  XON (turn output on)
                */
                tp->xoff = FALSE;
                }
            else
                {
                /* 
                **  XOFF (turn output off)
                */
                tp->xoff = TRUE;
                }

            continue;
            }

        if (ch == tp->params.fvCN)
            {
            /*
            **  Cancel character entered - erase all characters entered
            **  and indicate it to user via "*DEL*". Use the echobuffer
            **  to build and send the sequence.
            */
            echoPtr = echoBuffer;
            cnt = tp->inBufPtr - tp->inBufStart;
            for (i = cnt; i > 0; i--)
                {
                *echoPtr++ = ChrBS;
                }

            for (i = cnt; i > 0; i--)
                {
                *echoPtr++ = ' ';
                }

            for (i = cnt; i > 0; i--)
                {
                *echoPtr++ = ChrBS;
                }

            *echoPtr++ = '*';
            *echoPtr++ = 'D';
            *echoPtr++ = 'E';
            *echoPtr++ = 'L';
            *echoPtr++ = '*';
            *echoPtr++ = '\r';
            *echoPtr++ = '\n';
            npuNetSend(tp, echoBuffer, echoPtr - echoBuffer);

            /*
            **  Send the line, but signal the cancel character.
            */
            tp->inBuf[BlkOffDbc] = DbcCancel;
            npuBipRequestUplineCanned(tp->inBuf, tp->inBufPtr - tp->inBuf);

            /*
            **  Reset input and echoplex buffers.
            */
            npuTipInputReset(tp);
            echoPtr = echoBuffer;
            continue;
            }

        if (ch == tp->params.fvUserBreak1)
            {
            /*
            **  User break 1 (typically Ctrl-I).
            */
            npuTipSendUserBreak(tp, 1);
            continue;
            }

        if (ch == tp->params.fvUserBreak2)
            {
            /*
            **  User break 2 (typically Ctrl-T).
            */
            npuTipSendUserBreak(tp, 2);
            continue;
            }

        if (tp->params.fvEchoplex)
            {
            *echoPtr++ = ch;
            }

        if (ch == tp->params.fvEOL)
            {
            /*
            **  EOL entered - send the input upline.
            */
            npuBipRequestUplineCanned(tp->inBuf, tp->inBufPtr - tp->inBuf);
            npuTipInputReset(tp);
            tp->lastOpWasInput = TRUE;

            /*
            **  Optionally echo characters.
            */
            if (tp->dbcNoEchoplex)
                {
                /*
                **  DBC prevented echoplex for this line.
                */
                tp->dbcNoEchoplex = FALSE;
                echoPtr = echoBuffer;
                }
            else
                {
                echoLen = echoPtr - echoBuffer;
                if (echoLen)
                    {
                    npuNetSend(tp, echoBuffer, echoLen);
                    echoPtr = echoBuffer;
                    }
                }

            /*
            **  Perform cursor positioning.
            */
            if (tp->dbcNoCursorPos)
                {
                tp->dbcNoCursorPos = FALSE;
                }
            else
                {
                if (tp->params.fvCursorPos)
                    {
                    switch (tp->params.fvEOLCursorPos)
                        {
                    case 0:
                        break;

                    case 1:
                        npuNetSend(tp, netCR, sizeof(netCR));
                        break;

                    case 2:
                        npuNetSend(tp, netLF, sizeof(netLF));
                        break;

                    case 3:
                        npuNetSend(tp, netCRLF, sizeof(netCRLF));
                        break;
                        }
                    }
                }

            continue;
            }

        if (ch == tp->params.fvBS)
            {
            /*
            **  Process backspace.
            */
            if (tp->inBufPtr > tp->inBufStart)
                {
                tp->inBufPtr -= 1;
                *echoPtr++ = ' ';
                *echoPtr++ = tp->params.fvBS;
                }
            else
                {
                /*
                **  Beep when trying to go past the start of line.
                */
                npuNetSend(tp, netBEL, 1);
                }

            continue;
            }

        /*
        **  Store the character for later transmission.
        */
        *tp->inBufPtr++ = ch;

        if (tp->inBufPtr - tp->inBufStart >= (tp->params.fvBlockFactor * MaxIvtData))
            {
            /*
            **  Send long lines.
            */
            tp->inBuf[BlkOffBTBSN] = BtHTBLK | (tp->uplineBsn << BlkShiftBSN);
            npuBipRequestUplineCanned(tp->inBuf, tp->inBufPtr - tp->inBuf);
            npuTipInputReset(tp);
            }
        }
    }

/*---------------------------  End Of File  ------------------------------*/
