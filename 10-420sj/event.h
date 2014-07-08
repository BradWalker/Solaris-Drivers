/*
 * Events 
 *
 * This file is a .h file even though it has initialized data because
 * we want to this the only file that has to be editted to add new events.
 *
 * The driver is the only file which includes this file.
 *
 * The events are built into the driver so that the utilities xxtrace
 * and printtrace are independent of the event name list.
 *
 * The first entry is not used.  The list must be terminated by a NULL.
 */
struct event event_tab[] = {
	{ "NULL",			NULL				},
/*
 * * * Events for SERIAL PORTS
 */
/*
 * open
 */

#define OPEN_L1_ENTRY                   1
        { "open.l1.entry",		"\tq=0x%x\tflag=%d\tsflag=%d"	},

#define OPEN_L1_EXIT                    2
        { "open.l1.exit",		"\tline %d"			},

#define OPEN_L2_SLEEP_CFHANG            3
	{ "open.l2.sleep_cfhang",	"\tsleep on CF_HANG"		},

#define OPEN_L2_SLEEP_INT               4
        { "open.l2.sleep_int",		"\tsleep interrupted! line %d"	},

#define OPEN_L2_WAKEUP_CFHANG           5
	{ "open.l2.wakeup_cfhang",	"\twakeup from CF_HANG sleep"	},

#define OPEN_L2_1STOPEN                 6
        { "open.l2.1stopen",		"\tfirst open"			},

#define OPEN_L2_EXCLUDE                 7
        { "open.l2.exclude",		"\talready exclusive open"	},

#define OPEN_L2_DTR                     8
        { "open.l2.dtr",		"\traise DTR & RTS"		},

#define OPEN_L2_MSR                     9
	{ "open.l2.msr",		"\tmsr=0x%x"			},

#define OPEN_L2_SLEEP_CD                10
	{ "open.l2.sleep_cd",		"\tsleep on CD"			},

#define OPEN_L2_WAKEUP_CD               11
	{ "open.l2.wakeup_cd",		"\twakeup from CD sleep"	},

#define OPEN_L2_FIFO                    12
	{ "open.l2.fifo",		"\tFIFO set, FCR = 0x%x"	},
/*
 * close
 */

#define CLOSE_L1_ENTRY                  13
        { "close.l1.entry",		"\tq=0x%x\tflag=0x%x"		},

#define CLOSE_L2_SLEEP                  14
        { "close.l2.sleep",	"\tlasttxmp 0x%x lastrxmp 0x%x flags 0x%x"},

#define CLOSE_L2_WAKEUP                 15
        { "close.l2.wakeup",	"\tlasttxmp 0x%x lastrxmp 0x%x flags 0x%x"},

#define CLOSE_L1_EXIT                   16
        { "close.l1.exit",		"\tline %d"			},

#define CLOSE_L2_DTR                    17
        { "close.l2.dtr",		"\tlower DTR & RTS"		},
/*
 * unhang
 */

#define UNHANG_L1_ENTRY                 18
        { "unhang.l1.entry",		"\tcfline=0x%x"			},

#define UNHANG_L1_EXIT                  19
        { "unhang.l1.exit",		"\tline %d"			},
/*
 * wput
 */

#define WPUT_L1_ENTRY                   20
        { "wput.l1.entry",		"\tq=0x%x\tmp=0x%x"		},

#define WPUT_L1_EXIT                    21
        { "wput.l1.exit",		"\tline %d"			},

#define WPUT_L2_M_STOP                  22
        { "wput.l2.m_stop",		"\tM_STOP"			},

#define WPUT_L2_M_START                 23
        { "wput.l2.m_start",		"\tM_START"			},

#define WPUT_L2_M_IOCTL                 24
        { "wput.l2.m_ioctl",		"\tM_IOCTL"			},

#define WPUT_L2_SETBRK                  25
        { "wput.l2.setbrk",		"\tSETBRK ioc_cmd %d"		},

#define WPUT_L2_IOCTL                   26
	{ "wput.l2.ioctl",		"\tcall cfioctl 0x%x"		},

#define WPUT_L2_M_FLUSH                 27
        { "wput.l2.m_flush",		"\tM_FLUSH"			},

#define WPUT_L2_M_STOPI                 28
        { "wput.l2.m_stopi",		"\tM_STOPI"			},

#define WPUT_L2_M_STARTI                29
        { "wput.l2.m_starti",		"\tM_STARTI"			},

#define WPUT_L2_M_DATA                  30
        { "wput.l2.m_data",		"\tM_BREAK, M_DELAY, or M_DATA"	},

#define WPUT_L2_UNKNOWN                 31
        { "wput.l2.unknown",		"\tunknown stream msg=0x%x"	},
/*
 * wsrv
 */

#define WSRV_L1_ENTRY                   32
        { "wsrv.l1.entry",		"\tq=0x%x"			},

#define WSRV_L1_EXIT                    33
        { "wsrv.l1.exit",		"\tline %d"			},
/*
 * rsrv
 */

#define RSRV_L1_ENTRY                   34
        { "rsrv.l1.entry",		"\tq=0x%x"			},

#define RSRV_L1_EXIT                    35
        { "rsrv.l1.exit",		"\tline %d"			},

#define RSRV_L2_CANPUT                  36
	{ "rsrv.l2.canput",		"\tput message upstream!"	},

#define RSRV_L2_CANTPUT                 37
	{ "rsrv.l2.cantput",	"\tcannot put message upstream!"	},
/*
 * ioctl
 */

#define IOCTL_L1_ENTRY                  38
        { "ioctl.l1.entry",		"\tcfline=0x%x\tq=0x%x\tmp=0x%x"},

#define IOCTL_L1_EXIT                   39
        { "ioctl.l1.exit",		"\tline %d"			},

#define IOCTL_L2_TCSET                  40
        { "ioctl.l2.tcset",		"\tcmd = 0x%x"			},

#define IOCTL_L2_MR_SETFIFO             41
	{ "ioctl.l2.mr_setfifo",	"\trcv_fifo 0x%x xmit_fifo 0x%x"},

#define IOCTL_L2_MRREAD_TUNE            42
        { "ioctl.l2.mrread_tune",       "\tget tuning parameters"	},

#define IOCTL_L2_TCSBRK                 43
	{ "ioctl.l2.tcsbrk",		"\tTCSBRK %d"			},

#define IOCTL_L2_TIOCSBRK               44
	{ "ioctl.l2.tiocsbrk",		"\tTIOCSBRK"			},

#define IOCTL_L2_TIOCCBRK               45
	{ "ioctl.l2.tioccbrk",		"\tTIOCCBRK"			},

#define IOCTL_L2_TIOCSDTR               46
	{ "ioctl.l2.tiocsdtr",		"\tTIOCSDTR"			},

#define IOCTL_L2_TIOCCDTR               47
	{ "ioctl.l2.tioccdtr",		"\tTIOCCDTR"			},

#define IOCTL_L2_TIOCMSET               48
	{ "ioctl.l2.tiocmset",		"\tTIOCMSET 0x%x"		},

#define IOCTL_L2_TIOCMBIS               49
	{ "ioctl.l2.tiocmbis",		"\tTIOCMBIS 0x%x"		},

#define IOCTL_L2_TIOCMBIC               50
	{ "ioctl.l2.tiocmbic",		"\tTIOCMBIC 0x%x"		},

#define IOCTL_L2_TIOCMGET               51
	{ "ioctl.l2.tiocmget",		"\tTIOCMGET 0x%x"		},

#define IOCTL_L2_UNKNOWN                52
	{ "ioctl.l2.unknown",		"\tunknown ioctl = 0x%x"	},

#define IOCTL_L2_IOCNAK                 53
        { "ioctl.l2.iocnak",		"\tioctl error, M_IOCNAK"	},

#define IOCTL_L2_IOCACK                 54
        { "ioctl.l2.iocack",		"\tioctl acknowledge, M_IOCACK"	},
/*
 * reioctl
 */

#define REIOCTL_L1_ENTRY                55
        { "reioctl.l1.entry",		"\tcfl = 0x%x"			},

#define REIOCTL_L1_EXIT                 56
        { "reioctl.l1.exit",		"\tline %d"			},

#define REIOCTL_L2_IOCTL                57
	{ "reioctl.l2.ioctl", "\tcall ioctl (cfl=0x%x, q=0x%x, mp=0x%x)"},
/*
 * param
 */

#define PARAM_L1_ENTRY                  58
        { "param.l1.entry",		"\tcfline=0x%x"			},

#define PARAM_L1_EXIT                   59
        { "param.l1.exit",		"\tline %d"			},

#define PARAM_L2_CSIZE                  60
        { "param.l2.csize",		"\tchar size=%d"		},

#define PARAM_L2_PAREVEN                61
	{ "param.l2.pareven",		"\tparity enabled even"		},

#define PARAM_L2_PARODD                 62
	{ "param.l2.parodd",		"\tparity enabled odd"		},

#define PARAM_L2_PARDIS                 63
	{ "param.l2.pardis",		"\tparity disabled"		},

#define PARAM_L2_STOP                   64
	{ "param.l2.stop",		"\t%d stop bits"		},

#define PARAM_L2_SETBAUD0               65
	{ "param.l2.setbaud0",		"\tset baud=0, lower DTR"	},

#define PARAM_L2_SETBAUD                66
	{ "param.l2.setbaud",		"\tset baud=%d"			},

#define PARAM_L2_CHAR_PERIOD            67
	{ "param.l2.char_period",	"\tchar period = %d"		},
/*
 * start
 */

#define START_L3_ENTRY                  68
        { "start.l3.entry",		"\tcfline=0x%x"			},

#define START_L3_EXIT                   69
        { "start.l3.exit",		"\tline %d"			},

#define START_L3_SENDQD                 70
	{ "start.l3.sendqd",		"\tsend queued char 0x%x '%c'"	},

#define START_L3_STOPPED                71
	{ "start.l3.stopped",		"\toutput stopped (CF_OSTOPPED)"},

#define START_L3_TXSTOP                 72
	{ "start.l3.txstop",		"\toutput stopped (h/w)"	},

#define START_L3_M_BREAK                73
        { "start.l3.m_break",		"\tM_BREAK"			},

#define START_L3_M_DELAY                74
	{ "start.l3.m_delay",		"\tM_DELAY"			},

#define START_L3_M_IOCTL                75
	{ "start.l3.m_ioctl",		"\tM_IOCTL"			},

#define START_L3_COUNT                  76
        { "start.l3.count",		"\txmit %d chars, msg cnt %d"	},

#define START_L3_XMITCHAR               77
        { "start.l3.xmitchar",		"\txmit 0x%x '%c', line %d"	},

#define START_L3_LASTTXMP               78
        { "start.l2.lasttxmp",		"\tlasttxmp == NULL"		},

#define START_L3_STARTTIME              79
        { "start.l3.starttime",	"\tstarttime tv_sec %ld tv_usec %ld"	},

#define START_L3_UNKNOWN                80
        { "start.l3.unknown",		"\tunknown ioctl 0x%x"		},

#define START_L3_FIFOSAVE               81
        { "start.l3.fifosave",		"\tfifocnt = %d"		},
/*
 * rstrt
 */

#define RSTRT_L1_ENTRY                  82
	{ "rstrt.l1.entry",		NULL				},

#define RSTRT_L1_EXIT                   83
	{ "rstrt.l1.exit",		"\tline %d"			},
/*
 * mset
 */

#define SETDTRRTS_L1_ENTRY              84
	{ "mset.l1.entry",		"\tcfline=0x%x, value=0x%x, time=%d" },

#define SETDTRRTS_L1_EXIT               85
	{ "mset.l1.exit",		"\tline %d"			},

#define SETDTRRTS_L2_SETBITS            86
	{ "mset.l2.setbits",		"\tset DTR & RTS, time=%d"	},

#define SETDTRRTS_L2_DTRLOW             87
	{ "mset.l2.dtrlow",		"\tsetting cfp->dtrlow"		},

#define SETDTRRTS_L2_SLEEP              88
	{ "mset.l2.sleep",		"\tsleeping"			},
/*
 * intr
 */

#define INTR_L3_ENTRY                   89
        { "intr.l3.entry",		"\tcfl = 0x%x\tisr=0x%x"		},

#define INTR_L3_EXIT                    90
        { "intr.l3.exit",		"\tline %d"			},

#define INTR_L3_UNKNOWN                 91
	{ "intr.l3.unknown",		"\tunknown interrupt=0x%x"	},
/*
 * rxintr
 */

#define RXINTR_L3_ENTRY                 92
        { "rxintr.l3.entry",		"\tcfl = 0x%x"			},

#define RXINTR_L3_EXIT                  93
        { "rxintr.l3.exit",		"\tline %d"			},

#define RXINTR_L3_ALLOCB                94
	{ "rxintr.l3.allocb",		"\tallocb, size=%d"		},

#define RXINTR_L3_RCVCHAR               95
	{ "rxintr.l3.rcvchar",		"\treceived char 0x%x '%c'"	},

#define RXINTR_L3_TXSTOP                96
        { "rxintr.l3.txstop",		"\treceived xoff, stop xmit"	},

#define RXINTR_L3_TXSTART               97
        { "rxintr.l3.txstart",		"\treceived xon, start xmit"	},
/*
 * rsintr
 */

#define RSINTR_L3_ENTRY                 98
        { "rsintr.l3.entry",		"\tcfl = 0x%x\tstatus = %d"	},

#define RSINTR_L3_EXIT                  99
        { "rsintr.l3.exit",		"\tline %d"			},

#define RSINTR_L3_OVRRUN                100
	{ "rsintr.l3.ovrrun",		"\tover run!"			},

#define RSINTR_L3_ALLOCB                101
	{ "rsintr.l3.allocb",		"\tallocb, size=%d"		},

#define RSINTR_L3_FRMERR                102
	{ "rsintr.l3.frmerr",		"\tframe error!"		},

#define RSINTR_L3_BRKDET                103
	{ "rsintr.l3.brkdet",		"\treceived BREAK"		},

#define RSINTR_L3_PARERR                104
	{ "rsintr.l3.parerr",		"\tparity error! 0x%x '%c'"	},

#define RSINTR_L3_TXSTOP                105
        { "rsintr.l3.txstop",		"\treceived xoff, stop xmit"	},

#define RSINTR_L3_TXSTART               106
        { "rsintr.l3.txstart",		"\treceived xon, start xmit"	},
/*
 * rstimeout
 */

#define RXTIMEOUT_L3_ENTRY              107
        { "rxtimeout.l3.entry",		"\tcfl = 0x%x"			},

#define RXTIMEOUT_L3_EXIT               108
        { "rxtimeout.l3.exit",		"\tline %d"			},

#define RXTIMEOUT_L3_CANPUT             109
        { "rxtimeout.l3.canput",	"\tsend message upstream"	},

#define RXTIMEOUT_L3_RCVCHAR            110
	{ "rxtimeout.l3.rcvchar",	"\treceived char 0x%x '%c'"	},

#define RXTIMEOUT_L3_TXSTOP             111
        { "rxtimeout.l3.txstop",	"\treceived xoff, stop xmit"	},

#define RXTIMEOUT_L3_TXSTART            112
        { "rxtimeout.l3.txstart",	"\treceived xon, start xmit"	},

#define RXTIMEOUT_L3_LASTRXMP           113
        { "rxtimeout.l3.lastrxmp",	"\tlastrxmp == 0x%x"		},
/*
 * txintr
 */

#define TXINTR_L3_ENTRY                 114
        { "txintr.l3.entry",		"\tcfl = 0x%x"			},

#define TXINTR_L3_EXIT                  115
        { "txintr.l3.exit",		"\tline %d"			},
/*
 * mintr
 */

#define MINTR_L3_ENTRY                  116
        { "mintr.l3.entry",		"\tcfl = 0x%x, msr = 0x%x"	},

#define MINTR_L3_EXIT                   117
        { "mintr.l3.exit",		NULL				},

#define MINTR_L3_CD_WAKEUP              118
	{ "mintr.l3.cd_wakeup",		"\tCD high (or carrier) wakeup"	},

#define MINTR_L3_CD_UNHANGUP            119
	{ "mintr.l3.cd_unhangup",	"\tCD high, send M_UNHANGUP"	},

#define MINTR_L3_CDLOW                  120
        { "mintr.l3.nocd",		"\tCD low"			},

#define MINTR_L3_M_HANGUP               121
	{ "mintr.l3.m_hangup",		"\tsend M_HANGUP"		},

#define MINTR_L3_CTSHIGH                122
        { "mintr.l3.ctshigh",		"\tCTS high"			},

#define MINTR_L3_TXSTART                123
        { "mintr.l3.txstart",		"\tstart transmitting"		},

#define MINTR_L3_CTSLOW                 124
        { "mintr.l3.ctslow",		"\tCTS low"			},

#define MINTR_L3_TXSTOP                 125
        { "mintr.l3.txstop",		"\tstop transmitting"		},

#define MINTR_L3_NOCD                   126
        { "mintr.l3.nocd",		"\tno CD"			},

#define MINTR_L3_CURTIME                127
        { "mintr.l3.curtime",	"\tcurrent time tv_sec %ld tv_usec %ld"	},

#define MINTR_L3_CHARSENT               128
        { "mintr.l3.charsent", "\tnchar sent = %d tv_sec %ld tv_usec %ld"},

#define MINTR_L3_CLEAR_FIFO             129
        { "mintr.l3.clear_fifo",	"\trefill fifo from silo"	},
/*
 * stash
 */

#define STASH_L3_ENTRY                  130
        { "stash.l3.entry",		"\tq=0x%x\tc=0x%x '%c'"		},

#define STASH_L3_ALLOCB                 131
        { "stash.l3.allocb",		"\tline %d"			},

#define STASH_L3_PUTNEXT                132
        { "stash.l3.putnext",		"\tputnext"			},

#define STASH_L3_DROPDATA               133
        { "stash.l3.dropdata",		"\tOops, we dropped data!"	},

#define STASH_L3_EXIT                   134
        { "stash.l3.exit",		"\tline %d"			},
/*
 * pushdb
 */

#define PUSHDB_L1_ENTRY                 135
        { "pushdb.l1.entry",		"\tcfline 0x%x"			},

#define PUSHDB_L2_TIMEOUT               136
        { "pushdb.l1.timeout",		"\ttimeout called"		},

#define PUSHDB_L1_EXIT                  137
        { "pushdb.l1.exit",		"\tline %d"			},
/*
 * * * * * * * * * * * * Events for PARALLEL PORTS * * * * * * * * * * * *
 */
/*
 * popen
 */

#define POPEN_L1_ENTRY                  138
        { "Popen.l1.entry",		"\tq=0x%x\tflag=%d\tsflag =%d"	},

#define POPEN_L1_EXIT                   139
        { "Popen.l1.exit",		"\tline %d"			},

#define POPEN_L2_FNDELAY                140
	{ "Popen.l2.fndelay",		"\topened with FNDELAY"		},

#define POPEN_L2_NOTREADY               141
	{ "Popen.l2.notready",	"\tprinter not ready, status=0x%x"	},

#define POPEN_L2_1STOPEN                142
	{ "Popen.l2.1stopen",		"\tfirst open"			},
/*
 * pclose
 */

#define PCLOSE_L1_ENTRY                 143
        { "Pclose.l1.entry",		"\tq=%x\tflag=%x"		},

#define PCLOSE_L1_EXIT                  144
        { "Pclose.l1.exit",		"\tline %d"			},

#define PCLOSE_L1_LASTTXMP              145
        { "Pclose.l2.freemp",		"\tlasttxmp != NULL"		},
/*
 * ptimeout
 */

#define PTIMEOUT_L1_ENTRY               146
        { "Ptimeout.l1.entry",		"\tcfpline=0x%x\tsent_OK=%d"	},

#define PTIMEOUT_L1_EXIT                147
        { "Ptimeout.l1.exit",		"\tn %d"			},

#define PTIMEOUT_L2_M_ERROR             148
        { "Ptimeout.l2.m_error",	"\tsend M_ERROR"		},
/*
 * pwput
 */

#define PWPUT_L1_ENTRY                  149
        { "Pwput.l1.entry",		"\tq=0x%x\tmp=0x%x"		},

#define PWPUT_L1_EXIT                   150
        { "Pwput.l1.exit",		"\tline %d" 			},

#define PWPUT_L2_M_FLUSH                151
        { "Pwput.l2.m_flush",		"\tM_FLUSH"			},

#define PWPUT_L2_M_IOCTL                152
        { "Pwput.l2.m_ioctl",		"\tM_IOCTL"			},

#define PWPUT_L2_TCSBRK                 153
        { "Pwput.l2.tcsbrk",		"\tTCSBRK"			},

#define PWPUT_L2_M_DATA                 154
        { "Pwput.l2.m_data",		"\tM_DATA"			},

#define PWPUT_L2_UNKNOWN                155
	{ "Pwput.l2.unknown",	"\tunknown streams message=0x%x"	},
/* 
 * PWSRV
 */

#define PWSRV_L1_ENTRY                  156
        { "Pwsrv.l1.entry",		"\tq=0x%x"			},

#define PWSRV_L1_EXIT                   157
        { "Pwsrv.l1.exit",		"\tline %d"			},

#define PWSRV_L2_M_IOCTL                158
        { "Pwsrv.l2.m_ioctl",		"\tM_IOCTL"			},

#define PWSRV_L2_M_DATA                 159
        { "Pwsrv.l2.m_data",		"\tM_DATA"			},

#define PWSRV_L2_UNKNOWN                160
        { "Pwsrv.l2.unknown",	"\tunknown streams message=0x%x"	},
/*
 * pstart
 */

#define PSTART_L3_ENTRY                 161
        { "Pstart.l3.entry",		"\tcfpline=0x%x"		},

#define PSTART_L3_EXIT                  162
        { "Pstart.l3.exit",		"\tline %d"			},

#define PSTART_L3_OFFLINE               163
        { "Pstart.l3.offline",		"\toff line, try again"		},

#define PSTART_L3_STATUS_ERROR          164
        { "Pstart.l3.status.error",	"\tstatus (0x%x) & mask (0x%x) != normal (0x%x)"},

#define PSTART_L3_XMIT                  165
	{ "Pstart.l3.xmit",		"\tstart xmit %d, fifocnt %d"	},

#define PSTART_L2_LASTTXMP              166
        { "Pstart.l2.lasttxmp",		"\tlasttxmp == NULL"		},

#define PSTART_L2_NEXTTXMP              167
        { "Pstart.l2.nexttxmp",		"\tnexttxmp == NULL"		},

#define PSTART_L3_XMITCHAR              168
        { "Pstart.l3.xmitchar",		"\txmit char 0x%x '%c'"		},

#define PSTART_L3_NO_PAPER              169
        { "Pstart.l3.no_paper",		"\tno paper, try again"		},

#define PSTART_L3_M_IOCTL               170
	{ "start.l3.m_ioctl",		"\tM_IOCTL"			},

#define PSTART_L3_M_DELAY               171
	{ "start.l3.m_delay",		"\tM_DELAY"			},

#define PSTART_L3_UNKNOWN               172
        { "Pstart.l3.unknown",		"\tunknown status=0x%x"		},

#define PSTART_L3_NODATA                173
	{ "Pstart.l3.nodata",		"\tno data to send"		},

#define PSTART_L3_SLEEP                 174
        { "Pstart.l3.sleep",		"\tsleeping, staus = 0x%x"	},

#define PSTART_L3_RETIMEOUT             175
	{ "Pstart.l3.retimeout",	"\treenable timeouts"		},
/*
 * rstrt
 */

#define PRESTART_L1_ENTRY               176
        { "Prestart.l1.entry",		"\tcfpline=0x%x"		},

#define PRESTART_L1_EXIT                177
        { "Prestart.l1.exit",		"\tline %d"			},
/*
 * pioctl
 */

#define PIOCTL_L1_ENTRY                 178
        { "Pioctl.l1.entry",	"\tcfpline=0x%x\tq=0x%x\tmp=0x%x"	},

#define PIOCTL_L1_EXIT                  179
        { "Pioctl.l1.exit",		"\tline %d"			},

#define PIOCTL_L2_TTYIOCTL              180
	{ "Pioctl.l2.ttyioctl",		"\tcall ttycommon_ioctl"	},

#define PIOCTL_L2_TCSBRK                181
	{ "Pioctl.l2.tcsbrk",		"\tTCSBRK"			},

#define PIOCTL_L2_MR_PPSETFIFO          182
	{ "Pioctl.l2.mr_ppsetfifo", "\tset to transmit 0x%x chars each time"},

#define PIOCTL_L2_MRSEND_TUNE           183
	{ "Pioctl.l2.mrsend_tune", "\tburst=%d, retry=%d, cpu_idle=%d"	},

#define PIOCTL_L2_MRREAD_TUNE           184
	{ "Pioctl.l2.mrread_tune", "\tburst=%d, retry=%d, cpu_idle=%d"	},

#define PIOCTL_L2_MRIGNORE_PERRS        185
	{ "Pioctl.l2.mrignore_perrs",	"\tignore perrs, mask=0x%x, cond=0x%x"},

#define PIOCTL_L2_MRIGNORE_RESET        186
	{ "Pioctl.l2.mrignore_reset",	"\tperrs reset, mask=0x%x, cond=0x%x"},

#define PIOCTL_L2_MDRVP_GETPARAM        187
	{ "Pioctl.l2.mdrvp_getparam",	"\tget signals, mask=0x%x, value1=0x%x, value2=0x%x"},

#define PIOCTL_L2_MDRVP_SETPARAM        188
	{ "Pioctl.l2.mdrvp_setparam",	"\tset signals, mask=0x%x, value1=0x%x, value2=0x%x"},

#define PIOCTL_L2_UNKNOWN               189
	{ "Pioctl.l2.unknown",		"\tunknown ioctl 0x%x"		},
/*
 * reioctl
 */

#define PREIOCTL_L1_ENTRY               190
        { "Preioctl.l1.entry",		"\tcfp = 0x%x"			},

#define PREIOCTL_L1_EXIT                191
        { "Preioctl.l1.exit",		"\tline %d"			},
/*
 * pintr
 */

#define PINTR_L3_ENTRY                  192
        { "Pintr.l3.entry",		"\tcfp = 0x%x"			},

#define PINTR_L3_EXIT                   193
        { "Pintr.l3.exit",		"\tline %d" 			},

#define PINTR_L3_XMITCHAR               194
        { "Pintr.l3.xmitchar",		"\txmit 0x%x '%c'"		},

#define PINTR_L3_BUSY                   195
	{ "Pintr.l3.busy",		"\tbusy, break from reading"	},

#define PINTR_L3_ACK                    196
	{ "Pintr.l3.ack",		"\tacknowledge received"	},
/*
 * pintena
 */

#define PINTENA_L1_ENTRY                197
        { "Pintena.l1.entry",		"\tcfPline=0x%x"		},

#define PINTENA_L1_EXIT                 198
        { "Pintena.l1.exit",		"\tline %d" 			},
/*
 * pwatchdog
 */

#define PWATCHDOG_L1_ENTRY              199
        { "Pwatchdog.l1.entry",		"\tcfpline=0x%x"		},

#define PWATCHDOG_L1_EXIT               200
        { "Pwatchdog.l1.exit",		"\tline %d"			},
	 /* add more events here... */

#define CHECK_POINT                     201
	{ "check.point",		"\tcheck point %d, %d"		},

#define TIME_CHECK                      202
	{ "time.check",			"\ttv_sec %d, tv_usec %d pt %d"	},

#define STATUS_CHECK                    203
	{ "status.check",		"\tmask 0x%x, status 0x%x"	},
	{ NULL,				NULL				}
};
/* size of event table */
int xxtrevents = sizeof (event_tab) / sizeof (event_tab[0]);
