/*

	COPYRIGHT (c) 1992 BY AURORA TECHNOLOGIES, INC., WALTHAM, MA.
	
	THIS SOFTWARE IS FURNISHED UNDER A LICENSE AND MAY BE USED AND
	COPIED ONLY IN ACCORDANCE WITH THE TERMS OF SUCH LICENSE AND WITH
	THE INCLUSION OF THE ABOVE COPYRIGHT NOTICE.  THIS SOFTWARE OR
	ANY OTHER COPIES THEREOF MAY NOT BE PROVIDED OR OTHERWISE MADE
	AVAILABLE TO ANY OTHER PERSON.  NO TITLE TO AND OWNERSHIP OF THE
	PROGRAM IS HEREBY TRANSFERRED.

	THE INFORMATION IN THIS SOFTWARE IS SUBJECT TO CHANGE WITHOUT
	NOTICE AND SHOULD NOT BE CONSIDERED AS A COMMITMENT BY AURORA
	TECHNOLOGIES, INC.


	file: sio.h
	author: bwalker
	created: 7/23/92
	updated: jmartin 5/18/1993
	sccs info: @(#)sio.h	1.6 5/20/93 1

*/

#ifndef _SIO_H_
#define _SIO_H_

#ifdef BOARDSJ
#define CONFIG_NAME    		"aurora sio driver" 
#endif
#ifdef BOARDS
#define CONFIG_NAME		"aurora xsio driver"
#endif

/*
 *
 * configurable parameters
 *
 */

/* Maximum boards cfidentify will say "yes" to. */
#define MAXBOARDS       8			/* total boards supported */
#define NPORTS		8			/* number of port per board */

/*
 * The registers are mapped in to the driver by map_regs(), which takes
 * as one of its args the register offset from the start of the boards
 * geographic address space. This number is set by the autoconf routines
 * which take the reg attribute from the fcode PROM on the SBus board
 * and add the slot base address where the card is located. Since we set
 * the fcode offset to 0, we include the PROM space in the mapping that
 * map_regs() gives us. We must know where the registers are in relation
 * to the PROM in order to access them.
 */

#ifdef BOARDSJ
/*
 * reg			actual address
 * ---			--------------
 */
#define PROM_OFFSET	0x0000		/* PROM    0x0000-0x1FFF   */
#define PARALLEL_OFFSET 0x2000		/* /CS2    0x2000-0x4003   */
#define UART0_OFFSET    0x6000		/* /CS0    0x6000-0x8007   */
#define UART1_OFFSET    0x4000		/* /CS1    0x4000-0x6007   */
#define BCSR_OFFSET	0x8000		/* Board level CSR - only used on the 210SJ */

#define SCAN_OFFSET	0xA000		/* /SCAN   0xA000          */
#define INT2CLK_OFFSET  0xC000		/* INT2CLK 0xC000          */

/*
 * BCTRL values - external CSR on the board
 */

/* board specific CSR bits */
#define UART0_INT_ENA		0x08
#define CTL_PAR_INT_ENA		0x02
#endif

#ifdef BOARDS
/*
 * reg			actual address
 * ---			--------------
 */
#define PARALLEL_OFFSET	0x000
#define UART0_OFFSET	0x200
#define UART1_OFFSET	0x100
#define BCSR_OFFSET	0x300

/* board specific CSR bits */
#define UART0_INT_ENA		0x02
#define CTL_PARA_INT_ENA	0x08
#endif

/* board generic CSR bits */
#define LPTOE			0x01
#define UART1_INT_ENA		0x04
#define SBUS_INT_5		0x20
#define SBUS_INT_6		0x40
#define SBUS_INT_7		0x80

#define BCSR_REG	0x0		/* first register on the board */
#define PORT_REG_OFFSET	0x1		/* this is where the port registers start */

/*
 * mask to convert status reg to something valid to write to ctrl reg
 * preserves LPTOE and INT ENAs, sets CLR_INT2 to 0, and masks in
 * the sbus interrupt level bits from the hard-wired level.
 */
#define STA_REG_MASK(cfp)            (u_char)((*((cfp)->bcreg) & 0x0f) | SBUS_INT_5)

/*
 * STREAMS declarations
 */

/*
 * following are used in mblk_t allocation, depending upon the baud rate
 * we ask for different size mbkl_t
 */

#define CFSIZE1         16	/* baud <= 9600, get 16 byte buffer */
#define CFSIZE2         256	/* baud >= 19200, get 256 byte buffer */
#define ISPEED          B9600
#define IFLAGS          (IXON|IXOFF)
#define CFLAGS          (CS7|CREAD|PARENB|HUPCL)

/*
 * Minor device number definition
 *	MSB					LSB
 *	+---+---+---+---+---+---+---+---+---+---+
 *	| N |.......| 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 *	+---+---+---+---+---+---+---+---+---+---+
 *	  |		  |   |   |   |   |   |
 *	  |		  |   |   |   +---+---+----> port number
 *	  |		  |   |   |
 *	  |		  +---+---+----------------> board number
 *	  |
 *	  |
 *	  |
 *	  +----------------------------------------> dialout line
 */

#define PORT_MASK	0x7			/* support max 8 ports/board */
#define OUTLINE		(1 << (NBITSMINOR -1))		/* mask to check for dial out */
#define PORT(dev)	(getminor(dev) & PORT_MASK)
/*
 * instance(ie. board #) is encoded in the
 * bits following the port. Ends when
 * we hit the MSB
 */
#define	BOARD(dev)	((getminor(dev) & ~OUTLINE) >> 3)
#define ISOUT(dev)	(getminor(dev) & OUTLINE)	/* is this a dial out line ? */
#define FIFO_DPTH	0x10			/* depth of the FIFO on the VLSI 552 */

/* Per uart info */
struct sioline {
	u_char		last_event;
	u_char		board;
	u_char		port;
	u_char		pad;
	dev_t		dev;			/* port number - must be 1st field*/
        u_int		flags;			/* state flags */        
        kmutex_t	sio_mutex;		/* need a mutex so we can be MT */
	kcondvar_t	login_cv;
        u_char		*base;			/* uart base address */
	int		reg_size;		/* size of registers */
	int		flow;
	int		bufcid;
	int		tid;
        u_char		softmcr;
	u_int		baudrate;		/* save baudrate */
	u_int		fifo;			/* fifo status */
	u_char		rcv_fifo;		/* current recv fifo trigger level */
	u_char		xmit_fifo;		/* how many chars. do we put into the FIFO at a time */
	mblk_t		*lastrxmp;
	mblk_t		*lasttxmp;		/* point to the mblk_t */
        tty_common_t	ttycommon; 		/* queue's and flags */
	int		trycanput;		/* retry to send char. upstream before we drop */
	int		dtrlow;
};

struct serial_board {
	int		bintr;
        int		nport;
	u_char		*bcsr;		/* board CSR - only on 10 & 210 cards */
	ddi_iblock_cookie_t iblock_cookie;		/* for mutexes */
	ddi_idevice_cookie_t idevice_cookie;
	struct	sioline	**siolines;
        dev_info_t *devinfo;
};

/*
 *  the following is used as a flag to siostash() to tell it that we want this
 *  character to go upstream immediately
 */
#define PUSHCHAR	0x1			/* send the char upstream */
#define NOPUSHCHAR	0x2			/* don't sent the char upstream */
#define PUSHDB		0x4			/* push data block up - no chars go */

/*
 *  is a bit set
 */
#define BITSET(flag, bit)	(((flag) && bit) == (bit))

/* a single clock tick */
#define TICK	(hz / 100)

/*
 * SERIAL PORT STUFF
 *
 */

/* register offsets */
#define IER       0x1		/* interrupt enable */
#define ISR       0x2		/* a la IIR - read only */
#define FCR       0x2		/* write only */
#define LCR       0x3		/* line control */
#define MCR       0x4		/* modem control */
#define LSR       0x5		/* line status */
#define MSR       0x6		/* modem status */
#define SCR       0x7		/* scratch */

/* IER values */
#define Rx_en     0x1		/* receive intr enable */
#define Tx_en     0x2		/* transmit intr enable */
#define Rs_en     0x4		/* receiver status intr enable */
#define M_en      0x8		/* modem status intr enable */

/*
 * FCR value
 * FIFO Trigger depends on bit 6 and 7 
 * So in driver, these need to be OR'ed with line->fifo_trigger
 */
#define FIFO_INIT	0x0F
#define FIFO_ENABLE	0x09
#define FIFO_DISABLE	0x01
#define FIFO_XMIT_RESET	0x04
#define FIFO_RECV_RESET	0x02 

/*
 * FIFO_TRIGGER
 * set the trigger level for Receiver FIFO
 */
#define FIFO_TRIG_1	0x00
#define FIFO_TRIG_4	0x40
#define FIFO_TRIG_8	0x80
#define FIFO_TRIG_14	0xc0


/* ISR values */
#define INT_MASK	0x0f	/* mask for checking interrupts */
#define RSTATUS         0x06 	/* 1st prior LS=OE,FE,PE,BI */
#define RxRDY           0x04	/* 2nd prior Recv Data Avail */
#define RxEND		0x0c	/* 2nd prior Char Timeout */
#define TxRDY           0x02	/* 3rd prior X Holding R Empty */
#define MSTATUS         0x00	/* 4th prior Modem Status */
#define FIFO_PRESENT    0xc0	/* Is FIFO enabled? */
#define NO_INT_PENDING  0x1     /* !(isr & NO_INT_PENDING) = interrupt */

/* LCR values */
#define RXLEN     	0x03
#define STOP1     	0x00
#define STOP2     	0x04
#define PAREN     	0x08
#define PAREVN    	0x10
#define PARMARK   	0x20
#define SNDBRK    	0x40
#define DLAB      	0x80		/* divisor access latch bit */

#define BITS5     	0x00
#define BITS6     	0x01
#define BITS7     	0x02
#define BITS8     	0x03

/* MCR values */
#define DTR       	((u_char)0x01)
#define RTS       	((u_char)0x02)
#define OUT1      	((u_char)0x04)
#define OUT2      	((u_char)0x08)
#define LOOP      	((u_char)0x10)

/* LSR values */
#define RCA             0x01
#define OVRRUN          0x02
#define PARERR          0x04
#define FRMERR          0x08
#define BRKDET          0x10
#define XHRE            0x20
#define XSRE            0x40
#define FIFOERR		0x80

/* MSR values */
#define DCTS            0x01
#define DDSR            0x02
#define DRI             0x04
#define DDCD            0x08
#define CTS             0x10
#define DSR             0x20
#define RI              0x40
#define DCD             0x80


/* State flags */
#define CF_WOPEN        0x01
#define ISOPEN		0x02
#define CF_OUT          0x04
#define CF_CARR_ON      0x08
#define CF_XCLUDE       0x10
#define OSTOPPED	0x20			/* stop output for both h/w & s/w handshake */
#define CF_DELAY        0x40
#define INPUT_STOP	0x80
#define CF_BUSY         0x100
#define CF_BREAK        0x200
#define CF_ERROR        0x800
#define RXINTR		0x1000			/* we are servicing sometype of RX interrupt */
#define FIFO_SAVE	0x2000			/* we need to reseed the fifo with the stashed */

/* flags for siosetdtrrts */
#define MSETDTR		0x1
#define MSETRTS		0x2

/* cfline.transmit (Transmitter flag) */
#define CF_TRANSMIT_ON	0x1

/* cfline.fifo (fifo status - use to check if 452 or 552) */
#define CF_FIFO_ON	0x1

#define MRREAD_TUNE 70
#define MR_SETFIFO  71 /* set RECV FIFO trigger level */

/* fifo structure used by the SETFIFO ioctl*/
struct fifo_data {
	int	xmit_fifo;
	int	rcv_fifo;
};


#endif _SIO_H_
