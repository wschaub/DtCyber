#ifndef CONST_H
#define CONST_H
/*--------------------------------------------------------------------------
**
**  Copyright (c) 2003-2009, Tom Hunter (see license.txt)
**
**  Name: const.h
**
**  Description:
**      This file defines public constants and macros
**
**--------------------------------------------------------------------------
*/

/*
**  ----------------
**  Public Constants
**  ----------------
*/
#define DtCyberVersion          "Desktop CYBER 5.3.1.pci"
#define DtCyberCopyright        "Copyright (C) Tom Hunter"
#define DtCyberLicense          "Licensed to controlfreaks members for hobbyist use only"

#ifndef NULL                    
#define NULL                    ((void *) 0)
#endif                          
                                
#ifndef FALSE
#define FALSE                   0
#endif

#ifndef TRUE
#define TRUE                    (!FALSE)
#endif

/*
**  Conditional compiles:
**  ====================
*/

/*
**  Large screen support.
*/
#define CcLargeWin32Screen      0

/*
**  Debug support
*/
#define CcDebug                 0

/*
**  Measure cycle time
*/
#define CcCycleTime             0

/*
**  Use Telnet protocol for NPU connections
*/
#define CcTelnet                0

/*
**  Device types.
*/
#define DtNone                  0
#define DtDeadStartPanel        1
#define DtMt607                 2
#define DtMt669                 3
#define DtDd6603                4
#define DtDd8xx                 5
#define DtCr405                 6
#define DtLp1612                7
#define DtLp5xx                 8
#define DtRtc                   9
#define DtConsole               10
#define DtMux6676               11
#define DtCp3446                12
#define DtCr3447                13
#define DtDcc6681               14
#define DtTpm                   15
#define DtDdp                   16
#define DtNiu                   17
#define DtMt679                 18
#define DtNpu                   19
#define DtMch                   20
#define DtStatusControlRegister 21
#define DtInterlockRegister     22
#define DtPciChannel            23

/*
**  Special channels.
*/
#define ChClock                 014
#define ChInterlock             015
#define ChTwoPortMux            015
#define ChStatusAndControl      016
#define ChMaintenance           017

/*
**  Misc constants.
*/
#define PpMemSize               010000
                                
#define MaxUnits                010
#define MaxUnits2               020
#define MaxEquipment            010
#define MaxDeadStart            020
#define MaxChannels             040

#define MaxIwStack              12

#define NEWLINE                 "\n"

#define FontLarge               32
#define FontMedium              16
#define FontSmall               8
#define FontDot                 0

#if CcLargeWin32Screen == 1
#define OffLeftScreen           010
#define OffRightScreen          01100
#else
#define OffLeftScreen           010
#define OffRightScreen          01040
#endif

/*
**  Bit masks.
*/
#define Mask1                   01
#define Mask2                   03
#define Mask3                   07
#define Mask4                   017
#define Mask5                   037
#define Mask6                   077
#define Mask7                   0177
#define Mask8                   0377
#define Mask9                   0777
#define Mask10                  01777
#define Mask11                  03777
#define Mask12                  07777
#define Mask15                  077777
#define Mask17                  0377777
#define Mask18                  0777777
#define Mask21                  07777777
#define Mask24                  077777777
#define Mask24Ecs               077777700
#define Mask30                  07777777777
#define Mask30Ecs               07777777700
#define Mask48                  000007777777777777777
#define Mask50                  000037777777777777777
#define Mask60                  077777777777777777777
#define MaskCoeff               000007777777777777777
#define MaskExp                 037770000000000000000
#define MaskNormalize           000004000000000000000

/*
**  Trace masks.
*/
#define TraceCpu                (1 << 31)
#define TraceExchange           (1 << 30)

/*
**  Sign extension and overflow.
*/
#define Overflow12              010000

#define Sign18                  0400000
#define Overflow18              01000000

#define Sign21                  04000000
#define Overflow21              010000000

#define Sign24                  040000000
#define Overflow24              0100000000

#define Sign48                  000004000000000000000

#define Sign60                  040000000000000000000
#define Overflow60              0100000000000000000000

#define SignExtend18To60        077777777777777000000

#define NegativeZero            077777777777777777777

/*
**  CPU exit mode and flag bits.
*/
#define EmNone                  00000000
#define EmAddressOutOfRange     00010000
#define EmOperandOutOfRange     00020000
#define EmIndefiniteOperand     00040000

#define EmFlagStackPurge        00200000
#define EmFlagEnhancedBlockCopy 01000000
#define EmFlagExpandedAddress   02000000
#define EmFlagUemEnable         04000000

/*
**  Channel status masks.
*/
#define MaskActive              0x4000 
#define MaskFull                0x2000

/*
**  ----------------------
**  Public Macro Functions
**  ----------------------
*/
#define LogErrorLocation        __FILE__, __LINE__
#if defined (__GNUC__) || defined(__SunOS)
#define stricmp strcasecmp
#endif

#endif /* CONST_H */
/*---------------------------  End Of File  ------------------------------*/
