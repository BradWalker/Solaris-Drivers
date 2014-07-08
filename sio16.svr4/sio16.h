/*
 * Aurora Cirrus CL-CD180/1865 Async Driver (sio16)
 *
 * This include file defines the registers and register
 *  contents of the Cirrus Logic CL-CD180 and CL-CD1865
 *  chips.
 *
 * COPYRIGHT (c) 1992-1996 BY AURORA TECHNOLOGIES, INC., WALTHAM, MA.
 *
 * THIS SOFTWARE IS FURNISHED UNDER A LICENSE AND MAY BE USED AND
 * COPIED ONLY IN ACCORDANCE WITH THE TERMS OF SUCH LICENSE AND WITH
 * THE INCLUSION OF THE ABOVE COPYRIGHT NOTICE.  THIS SOFTWARE OR
 * ANY OTHER COPIES THEREOF MAY NOT BE PROVIDED OR OTHERWISE MADE
 * AVAILABLE TO ANY OTHER PERSON.  NO TITLE TO AND OWNERSHIP OF THE
 * PROGRAM IS HEREBY TRANSFERRED.
 *
 * THE INFORMATION IN THIS SOFTWARE IS SUBJECT TO CHANGE WITHOUT
 * NOTICE AND SHOULD NOT BE CONSIDERED AS A COMMITMENT BY AURORA
 * TECHNOLOGIES, INC.
 *
 *	file: sio16.h
 *	author: bwalker
 *	created: 2/1/93
 *	revision info: $Id: sio16.h,v 1.9.2.3 1996/10/22 15:17:04 bkd Exp $
 */

/*
 * $Log: sio16.h,v $
 * Revision 1.9.2.3  1996/10/22 15:17:04  bkd
 * Changed modinfo tag line of driver from:
 * aurora 16 port serial
 * to:
 * Aurora CL-CD180/1865 driver
 *
 * Revision 1.9.2.2  1996/10/17 03:41:29  bkd
 * + removed old modem/flow control mechanisms (L_DTRSWAP macro,
 *   hw_flags structure member)
 * + added new modem/flow control configuration mechanism
 *
 * Revision 1.9.2.1  1996/10/08 22:47:59  bkd
 * + removed CD180_REVx definitions (they now
 *   live in cd18x.h as GFRCR definitions)
 * + added SIO_RING_EMPTY and SIO_RING_LOWATER
 *   macros
 * + added chip type and chip revision definitions
 *   and members to line structure
 * + added bufcall member to line structure (for new
 *   rsrv design)
 *
 * Revision 1.9  1996/09/27 14:24:15  bkd
 * + modified to meet Aurora coding standards (4 space indent, fully
 *   ANSIfied, complete and proper prototypes, objects sorted by type in
 *   source file, standard Aurora copyright boilerplate, RCS ids and
 *   copyrights, no long lines, only boolean expressions (no arithmetic
 *   expressions) in for, if, while, and do, mandatory opening and
 *   closing braces after for, if, and while)
 * + moved CL-CD180/1864/1865 definitions and declarations to cd18x.h
 * + changed baud rate divisor calculation algorithm to round properly
 * + removed already ifdef'ed-out definition of SETCHAN
 * + changed baud rate table elements to proper types (unsigned short and
 *   unsigned char -- was short and int)
 * + removed support for non-Solaris 2 (e.g. SunOS 4) operating systems
 * + removed support for old hardware (1600S, 1600SM)
 * + eliminated many unnecessary macros
 * + added new CHIP_MUTEX_{ENTER,EXIT} macros
 * + removed unused mutex instrumentation definitions
 * + converted to use the new, standard XXTRACE (xxtrace.svr4) by
 *   modifying the header in the sioline structure
 * + removed fields from sioline structure: old_chan, tiocmget_val
 * + added cflag field to sioline structure
 * + changed all 'struct cd180's to be volatile
 * + removed prototypes for static routines in sio16.c and sio16_osdep.c
 * + added full ANSI prototypes for routines in sio16.c and sio16_osdep.c
 * + added fields to sio16_board structure: clk, instance
 * + removed fields to sio16_board structure: intr_pri, old_spl
 * + changed siolines in sio16_board structure from (a pointer to an
 *   array of pointers to structures) to a pointer to an array of
 *   structures.  This decreases fragmentation and speeds up accesses
 *
 * Revision 1.8  1995/08/15 23:56:26  bkd
 * Bug fix attempt: panic: mutex reentered (poll->txintr->start->
 * ioctl->qreply calling ldterm calling wput); fixed by adding
 * 'intr' parameter to ioctl()
 *
 * Revision 1.7  1995/08/15  23:14:17  bkd
 * Updated copyrights; added CVS control information
 *
 */

#ifdef sun				/* XXXAIX */
#   pragma ident "@(#)$Header: /vol/sources.cvs/dev/sio16.svr4/sio16.h,v 1.9.2.3 1996/10/22 15:17:04 bkd Exp $"
#endif

/* ------------------------------------------------------------------------- */
/* General macros */

#define DBGPRINTF	if (sio16_debug)	\
				printf

#define S_DRIVER_NAME	"Aurora Model 1600 "
#define SM_DRIVER_NAME	"Aurora Model 1600sm"
#define SE_DRIVER_NAME	"sio16"

#define MODULE_NAME	"Aurora CL-CD180/1865 driver"

/* Board memory map
 *
 *      1600S                   1600SM
 *
 *   0k	+-------+ 0x000      0k +-------+ 0x000
 *      | PROM	|               | PROM  |
 *      +-------+ 0x100         +-------+ 0x100
 *      | PROM  |               | PROM  |
 *      +-------+ 0x200         +-------+ 0x200
 *      | PROM  |               | C/S   |
 *      +-------+ 0x300         +-------+ 0x300
 *      | PROM  |               |       |
 *   1k	+-------+ 0x400         +-------+ 0x400
 *      | PROM	|               |       |
 *      +-------+ 0x500         +-------+ 0x500
 *      | PROM  |               |       |
 *      +-------+ 0x600         +-------+ 0x600
 *      | PROM  |               |       |
 *      +-------+ 0x700         +-------+ 0x700
 *      | PROM  |               |       |
 *   2k	+-------+ 0x800         +-------+ 0x800
 *      | CE0	|               | CS2   |
 *      +-------+ 0x900         +-------+ 0x900
 *      | CE1   |               |       |
 *      +-------+ 0xa00         +-------+ 0xa00
 *      | IACK  |               | CS1   |
 *      +-------+ 0xb00         +-------+ 0xb00
 *      | CSR   |               |       |
 *   3k	+-------+ 0xc00         +-------+ 0xc00
 *      |       |               |       |
 *      +-------+ 0xc00         +-------+ 0xd00
 *      |       |               |       |
 *     	+-------+ 0xc00         +-------+ 0xe00
 *      |       |               | IACK  |
 *     	+-------+ 0xc00         +-------+ 0xf00
 */

/*
 * Register sets.  These must match the order of the registers specified
 * in the prom on the  board!
 */
#define REG_CSR		0
#define REG_CD1		1
#define REG_CD2		2
#define REG_IACK        3

#define	BCSR_OFFSET	0x0
#define	CHIP1_OFFSET	0x80
#define CHIP2_OFFSET	0x100
#define	IACK_OFFSET	0x180

/*
 * minor device numbering
 *
 * +----~ ---+---+---+---+---+---+---+---+
 * | 17 | 16 | 15 thru 4 | 3 | 2 | 1 | 0 |
 * +----~ ---+---+---+---+---+---+---+---+
 *    |        |   |   |   |   |   |   |
 *    |        |   |   |   +---+---+---+--> port number (the low order 3 bits
 *    |	       |   |   |		    are the channel #)
 *    |        |   |   |
 *    |        +-- +---+------------------> board number (up to 2048)
 *    |
 *    ------------------------------------> dialout line
 */

#define PORT_MASK	0xf	/* board only support 16 ports */
#define BOARD_MASK	0x7ff	/* we support 2048 boards at this time */
#define CHANNEL_MASK	0x7	/* which physical chip channel */
#define NSIO_PORTS	0x10	/* we have 16 ports on a board */
#define UNIT(dev)	((dev) & PORT_MASK)	/* logical board port */
#define CHANNEL(dev)	((dev) & CHANNEL_MASK)	/* physical chip channel */
#define CHIP(dev)	(((dev) >> 3) & 0x1)	/* chip we are dealing with */
#define BOARD(dev)	(((dev) >> 4) & BOARD_MASK) /* logical board number */
#define BOARD2MINOR(board) (board << 4)		/* convert board to minor */
#define OUTLINE		(1 << (NBITSMINOR - 1))
#define ISOUT(dev)	((dev) & OUTLINE)

/*
 *  the following is passed as a flag to sio16_stash() to tell what
 *  we want to do with this character
 */
#define PUSHCHAR	0x1		/* send the char upstream */
#define NOPUSHCHAR	0x2		/* don't sent the char upstream */
#define PUSHDB		0x4		/* push data block up - no chars go */

/*
 * the following is used to turn on/off RTS & DTR
 */
#define SIO_ON		0x01
#define SIO_OFF		0x02

#define TX_FIFO		0x8		/* how deep is the chip fifo */

#define PILR1		0x81		/* modem iack addr */
#define PILR2		0x82		/* tx iack addr */
#define PILR3		0x83		/* rx iack addr */

#define RX_NORMAL	0x0
#define RS_INTR		0x1

/*
 * state flags
 */
#define SIO_WOPEN	0x0001		/* waiting for open to complete */
#define SIO_ISOPEN	0x0002		/* channel is open */
#define SIO_OUT		0x0004		/* out line with CD */
#define SIO_CARR_ON	0x0008		/* current state of CD */
#define SIO_OSTOPPED	0x0010		/* stop output */
#define SIO_ISTOPPED	0x0020		/* stop input please */
#define SIO_DELAY	0x0040		/* transmitting delayed char */
#define SIO_BREAK	0x0080		/* transmit break flag set */
#define SIO_ENDBREAK	0x0100		/* waiting for the end of break */
#define SIO_BUSY	0x0200
#define SIO_CDTRACK	0x0400		/* tracking CD (1600SE ports 12-15) */
#define	SIO_THROTTLE	0x0800		/* rxintr stopped input */

/*
 * the following defines the model types
 */
#define SIO16_BADMOD		-1		/* invalid model */
#define SIO16_1600SE		0x0		/* Aurora 1600se */
#define SIO16_1600SM		0x1		/* Aurora 1600sm */
#define SIO16_1600S		0x2		/* Aurora 1600s  */

/*
 * these are the bit settings for the CSR on the 1600s & 1600se
 * the 1600se is specifics in that you have a clear reset bit (this must
 * be set to 1 in order to clear the RESET* line tied to the chip
 */
#define SBUS_INT_5_1600S	0x20	/* SBus level 5 */
#define SBUS_INT_6_1600S	0x40	/* SBus level 6 */
#define SBUS_INT_7_1600S	0x80	/* SBus level 7 */

#define SBUS_INT_2_1600SE	0x00	/* SBus level 2 */
#define SBUS_INT_3_1600SE	0x01	/* SBus level 3 */
#define SBUS_INT_5_1600SE	0x02	/* SBus level 5 */
#define SBUS_INT_7_1600SE	0x03	/* SBus level 7 */
#define BCSR_INT_EN		0x04	/* enable interrupts from board */
#define BCSR_CLR_RESET		0x08	/* clear the reset on the board */

/*
 * 1600s/1600sm CSR bits
 */
#define BCSR_IREQ1		0x01	/* modem interrupt pending */
#define BCSR_IREQ2		0x02	/* transmit interrupt pending */
#define BCSR_IREQ3		0x04	/* receive interrupt pending */

/* a single clock tick */
#define TICK	(hz / 100)

/* check to see if a bit in the subset is set */
#define BIT_SET(bit, subset)	(((subset) & (bit)) ? 1 : 0)

/* baudrate table */
struct sio16_baud {
    unsigned short	 b_bpr;		/* baud period register */
    unsigned char	 b_rtpr;	/* rx timeout (x1 ms) */
    unsigned char	 b_rxth;	/* rx fifo threshold level */
};
 
#define SE2_CLK	((unsigned long) 11059200)	/* 11.0592 MHz */
#define SE_CLK	((unsigned long) 14745600)	/* 14.7456 MHz */

/* divide x by y, rounded */
#define ROUND_DIV(x, y)		(((x) + ((y) >> 1)) / (y))

#define SE2_BSPEED(baud)	(unsigned short)		\
			ROUND_DIV(SE2_CLK, 16 * (unsigned long) (baud))
#define SE_BSPEED(baud)		(unsigned short)		\
			ROUND_DIV(SE_CLK, 16 * (unsigned long) (baud))

#define SETCHAN(line)				\
{						\
    ASSERT(LOCKED(line));			\
    (line)->cd180->car = CHANNEL((line)->dev);	\
    DELAY(1);					\
}

#define UNSETCHAN(line)				\
{						\
    ASSERT(LOCKED(line));			\
}


#define CRITICAL(line) \
	mutex_enter((line)->chiplock)

#define UNCRITICAL(line) \
	mutex_exit((line)->chiplock)

#define CHIP_MUTEX_ENTER(sbp, n) \
	mutex_enter(&((sbp)->chiplock[(n)]))

#define CHIP_MUTEX_EXIT(sbp, n) \
	mutex_exit(&((sbp)->chiplock[(n)]))

#define PRIV(cred) drv_priv(cred)

#define ERR printf

#define GETTIME(arg) drv_getparm(LBOLT, arg)

#define HZTOSEC(nhz) (drv_hztousec(nhz) / 1000000)

#define LOCKED(siop) mutex_owned(siop->chiplock)

/*
 * the DELAYCK macro is used to spin wait on an event
 * for dcnt u-sec. if dcnt becomes 0 then print out str
 * for a certain unit
 */
#define DELAYCK(event, sio_port, str)				\
{								\
    int dcnt = 1000;						\
    while (event) {						\
	if (--dcnt == 0) {					\
	    printf("line %d: %s timeout, channel = %d\n",	\
			__LINE__, str, (sio_port));		\
	    break;						\
	}							\
	DELAY(100);						\
    }								\
}

/*
 * the following defines allows us to use a ring buffer to store the data
 */
#define	SIO_RINGBITS	8			/* # of bits in ring ptrs */
#define	SIO_RINGSIZE	(1 << SIO_RINGBITS)	/* size of ring */
#define	SIO_RINGMASK	(SIO_RINGSIZE-1)
#define	SIO_RINGFRAC	2			/* fraction of ring to force flush */

/* init ring */
#define	SIO_RING_INIT(siop)	((siop)->sio_rput = (siop)->sio_rget = 0)

/* is the ring empty? */
#define	SIO_RING_EMPTY(siop)			\
	(((siop)->sio_rput == (siop)->sio_rget))

/* number of chars in the ring */
#define	SIO_RING_CNT(siop)			\
	(((siop)->sio_rput - (siop)->sio_rget) & SIO_RINGMASK)

/* have we reached a threshhold level */
#define	SIO_RING_FRAC(siop)			\
	(SIO_RING_CNT(siop) >= (SIO_RINGSIZE/SIO_RINGFRAC))

/* hiwater is 3/4 of RINGSIZE, stop input from interrupt routine */
#define	SIO_RING_HIWATER(siop)			\
	(SIO_RING_CNT(siop) + (1 << (SIO_RINGBITS - 2)) >= SIO_RINGSIZE)

/* lowater is 1/4 of RINGSIZE, stop input from interrupt routine */
#define	SIO_RING_LOWATER(siop)			\
	(SIO_RING_CNT(siop) < (SIO_RINGSIZE / 4))

/* can we poke n chars into the ring */
#define	SIO_RING_POKE(siop, n) 	(SIO_RING_CNT(siop) < (SIO_RINGSIZE-(n)))

/* put the chars into the ring */
#define	SIO_RING_PUT(siop, c) \
	((siop)->sio_ring[(siop)->sio_rput++ & SIO_RINGMASK] =  (u_char)(c))

/* unput the chars from the ring */
#define	SIO_RING_UNPUT(siop)	((siop)->sio_rput--)

/* number of chars in ring > n */
#define	SIO_RING_GOK(siop, n) 	(SIO_RING_CNT(siop) >= (n))

/* get chars from ring */
#define	SIO_RING_GET(siop)			\
	((siop)->sio_ring[(siop)->sio_rget++ & SIO_RINGMASK])

/* drop chars from ring */
#define	SIO_RING_EAT(siop, n)	 ((siop)->sio_rget += (n))

/* chip types: */
#define CT_UNKNOWN	0x0	/* unknown */
#define CT_CL_CD180	0x1	/* Cirrus Logic CD-180 */
#define CT_CL_CD1864	0x2	/* Cirrus Logic CD-1864 */
#define CT_CL_CD1865	0x3	/* Cirrus Logic CD-1864 */

/* chip revisions: */
#define CR_UNKNOWN	0x0	/* unknown */
#define CR_REVA		0x1	/* revision A */
#define CR_REVB		0x2	/* revision B */
#define CR_REVC		0x3	/* revision C */
/* ...and so on ... */

/* ------------------------------------------------------------------------- */
/* New types and type-specific macros */

typedef int boolean;

#ifdef TRUE
#   undef TRUE
#endif

#ifdef FALSE
#   undef FALSE
#endif

#define TRUE (0 == 0)
#define FALSE (!TRUE)

/* line structure - per serial port */
struct sioline {
    /*
     * XXTRACE this exact format to be at the beginning of the
     * line structure
     */
    unsigned short	 last_event;
    unsigned char	 board;
    unsigned char	 port;

    /*
     * line state information
     */
    dev_t		 dev;		/* the major/minor # for this line */
    int			 flags;		/* state flags */
    struct sio16_baud	*baud_tbl;	/* ptr to board baudrate table */
    unsigned int	 cust_baud;	/* custom baud rate */
    unsigned short	 cust_bpr;	/* custom period reg */
    unsigned char	 modem_flags;	/* for last 4 ports */
    int            	 dtrlow;	/* when dtr went low */
    unsigned int	 close_timeout;	/* timeout for sleep */
    mblk_t		*lasttxmp;	/* current blk being transmitted */
    tty_common_t	 ttycommon;	/* queue's and flags */
    unsigned long	 cflag;		/* last valid cflag value */
    int			 bufcall;
    unsigned char	 srer;		/* last value sent to ier */
    /*
     * Modem/flow control bits.  This is how the different
     *  wirings of the CL-CD180/1865 boards are described.
     * If any of the mask values are zero, then the signal
     *  does not exist.
     * All of these signals are defined in their "logical"
     *  sense.  That is, when we refer to "DTR" below, we
     *  are referring to the signal on pin 20 of the final
     *  DB25, and *NOT* the CL-CD180/1865 pin "DTR".  Logical
     *  in this sense refers to what the end user will see.
     *
     * Now, with respect to modem_flags.  The modem_flags
     *  field is only for reference in the ioctl code; it
     *  does not contain critical state.  The only exception
     *  to this is the SIO_DTRFLOW bit.  What I've done is
     *  taken this to indicate that CRTSCTS flow control
     *  should be done on the *logical* DTR pin.  This way,
     *  the user is unlikely to ever need to use it.
     *
     * In summary, *all* of the modem/flow control configuration
     *  information is stored in the following fields.  These
     *  data are in the form of MSVR bit masks.
     */
    /* input masks -- these corresponding indicate bits in MSVR */
    unsigned char	 cts_mask;
    unsigned char	 dsr_mask;
    unsigned char	 cd_mask;
    /* output masks -- these are just for reading things back */
    unsigned char	 dtr_mask;
    unsigned char	 rts_mask;
    /* pseudo-mask -- this is a copy of either dtr_mask or rts_mask */
    unsigned char	 iflow_mask;	/* input flow control mask */
    /*
     * async error statistics
     */
    caddr_t		 l_sio_stats;    /* async i/o statistics         */
    caddr_t		 l_sio_stats_valid; /* 0 = valid, 1 = overflow   */
    /*
     * chip/board information
     */
    volatile struct cd180 *cd180;	/* cd180 addr */
    struct sio16_board	*board_ptr;
    kcondvar_t		 cv;		/* condition variable */
    kmutex_t		*chiplock;	/* mutex to lock chip */
    /*
     * receive ring buffer
     */
    short		 sio_rput;	/* producing ptr for input */
    short		 sio_rget;	/* consuming ptr for input */
    u_char		 sio_ring[SIO_RINGSIZE]; /* circular input buffer */
    int			 timeoutpending; /* flag for timeout()*/
    int			 timeoutid;	/* id of timeout() */
    int			 chip_type;	/* chip type */
    int			 chip_rev;	/* chip revision */
};

/* board structure - per board */
struct sio16_board {
    volatile u_char	*bcsr;		/* board csr */
    volatile u_char	*chip1_reg;	/* board chip 1 reg */
    volatile u_char	*chip2_reg;	/* board chip 1 reg */
    volatile u_char	*iackbase;	/* board iack reg */
    dev_info_t		*dip;		/* devinfo pointer */
    struct sioline	*siolines;	/* where all the siolines reside */
    int			 nport;		/* num. of ports on the board */
    ddi_iblock_cookie_t	 iblock;	/* returned by ddi_add_intr()	*/
    ddi_idevice_cookie_t idevice;	/* returned by ddi_add_intr()	*/
    kmutex_t		 chiplock[2];	/* chip mutexes */
    int			 model;
    int			 clk;		/* clock speed - from "clk" prop */
    int			 instance;	/* this board's instance number */
};

/* ------------------------------------------------------------------------- */
/* Global variables and macros */

/* sio16.c */
extern int sio16_debug;
#ifdef XXTRACE
extern int sio16_xtinit;		/* xxtrace initialized flag */
#endif		/* XXTRACE */

/* sio16_osdep.c */
extern void *sio16_head;
extern ddi_iblock_cookie_t *sio16_iblock;
extern int sio16_close_timeout;
extern int sio16_maxinstance;	/* maximum instance */

/* ------------------------------------------------------------------------- */
/* Global function prototypes and macros */

#ifdef XXTRACE
/* xxtracelib.c */
/*
 * Unfortunately, this file is included *before* xxtrace.h.
 *  This means that 'struct trdev' hasn't been defined yet.
 *  Hence, the ugliness below.
 */
/* extern void xxtrace(int event_num, struct trdev *ptr,
		unsigned int a, unsigned int b, unsigned int c); */
extern void xxtrace(int event_num, void *ptr,
		unsigned int a, unsigned int b, unsigned int c);
extern void xxtraceinit0(void);
extern void xxtraceinit_bitmap(int board, int nunits);

#endif		/* XXTRACE */

/* sio16.c */
extern int sio16_open(queue_t *q, dev_t *devp, int flag,
			int sflag, cred_t *credp);
extern int sio16_close(queue_t *rq, int flag);
extern int sio16_wput(queue_t *q, mblk_t *mp);
extern int sio16_rsrv(queue_t *q);
extern u_int sio16_poll(caddr_t arg);


