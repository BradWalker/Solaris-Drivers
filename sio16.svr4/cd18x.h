/*
 * Aurora Cirrus CL-CD180/1865 Async Driver (sio16)
 *
 * This include file defines the registers and register
 *  contents of the Cirrus Logic CL-CD180 and CL-CD1865
 *  chips.
 *
 * COPYRIGHT (c) 1996 BY AURORA TECHNOLOGIES, INC., WALTHAM, MA.
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
 *	file: cd18x.h
 *	author: bkd
 *	created: 9/13/1996
 *	revision info: $Id: cd18x.h,v 1.1.2.1 1996/10/08 22:46:08 bkd Exp $
 */

#ifdef sun				/* XXXAIX */
#   pragma ident "@(#)$Header: /vol/sources.cvs/dev/sio16.svr4/cd18x.h,v 1.1.2.1 1996/10/08 22:46:08 bkd Exp $"
#endif

/*
 * $Log: cd18x.h,v $
 * Revision 1.1.2.1  1996/10/08 22:46:08  bkd
 * Added definition for SRCR_PKGTYPE bit and
 * complete GFRCR definitions (they used
 * to live in sio16.h)
 *
 * Revision 1.1  1996/09/20 21:27:43  bkd
 * Added Files:
 * 	cd18x.h copyright.AURAdas8 depend depend.AURAdas8 m.sparc-sos5
 * 	makefile pkginfo.sparc pkginfo.sparc.AURAdas8
 * 	prototype.AURAdas8 version.h
 * Removed Files:
 * 	Makefile
 *
 */

/* ------------------------------------------------------------------------- */
/* General macros */

/* srcr bits */
#define SRCR_PRISEL		0x01
#define SRCR_AUTOPRI		0x02
#define SRCR_UNFAIR		0x08
#define SRCR_GLOBPRI		0x10
#define SRCR_REGACK_EN		0x40
#define SRCR_PKGTYPE		0x80

/* c_ccr bits */
#define CCR_RST_CHAN		0x80		/* reset a specific channel */
#define CCR_RST_CHIP		0x81		/* reset the chip */
#define CCR_CHG_COR3		0x48		/* cor 3 changed */
#define CCR_CHG_COR2		0x44		/* cor 2 changed */
#define CCR_CHG_COR1		0x42		/* cor 1 changed */
#define CCR_SSPC1		0x21
#define CCR_SSPC2		0x22
#define CCR_SSPC3		0x23
#define CCR_SSPC4		0x24
#define CCR_TXEN		0x18
#define CCR_TXDIS		0x14
#define CCR_RXEN		0x12
#define CCR_RXDIS		0x11

/* c_ier bits */
#define IER_DSR			0x80
#define IER_CD			0x40
#define IER_CTS			0x20
#define IER_RXDATA		0x10
#define IER_RXSC		0x08
#define IER_TXRDY		0x04
#define IER_TXMPTY		0x02
#define IER_RET			0x01

/*
 * Channel Option Register 1 (COR1) ($03) - R/W
 */
#define COR1_PARODD		0x80		/* odd parity */
#define COR1_PAREN		0x40		/* normal parity for tx & rx */
#define COR1_FORCE_PARITY	0x20		/* normal parity for tx & rx */
#define COR1_IGN_PARITY		0x10		/* eval parity on rcv char */
#define COR1_STOP1		0x00		/* 1 stop bit */
#define COR1_STOP15		0x04		/* 1.5 stop bits */
#define COR1_STOP2		0x08		/* 2 stop bits */
#define COR1_CS5		0x00		/* 5 bit char. length	*/
#define COR1_CS6		0x01		/* 6 bit	""	*/
#define COR1_CS7		0x02		/* 7 bit	""	*/
#define COR1_CS8		0x03		/* 8 bit	""	*/

/* c_cor2 bits */
#define COR2_IXM		0x80
#define COR2_TXIBE		0x40
#define COR2_ETC		0x20		/* special tx functions */
#define COR2_LLM		0x10
#define COR2_RLM		0x08
#define COR2_RTSAO		0x04
#define COR2_CTSAE		0x02
#define COR2_DSRAE		0x01

/* c_cor3 bits */
#define COR3_XON		0x80
#define COR3_XOFF		0x40
#define COR3_FCT		0x20
#define COR3_SCDE		0x10
#define COR3_RX_TH_MASK		0xf		/* rx threshold mask */
#define COR3_RX_THRESH_1	0x1		/* 1 char threshold */
#define COR3_RX_THRESH_2	0x2		/* 2 char	  */
#define COR3_RX_THRESH_3	0x3		/* 3 char	  */
#define COR3_RX_THRESH_4	0x4		/* 4 char	  */
#define COR3_RX_THRESH_5	0x5		/* 5 char	  */
#define COR3_RX_THRESH_6	0x6		/* 6 char	  */
#define COR3_RX_THRESH_7	0x7		/* 7 char	  */
#define COR3_RX_THRESH_8	0x8		/* 8 char	  */

/* c_ccsr bits */
#define CCSR_RXEN		0x80
#define CCSR_RXFLOFF		0x40
#define CCSR_RXFLON		0x20
#define CCSR_TXEN		0x08
#define CCSR_TXFLOFF		0x04
#define CCSR_TXFLON		0x02

/* c_mcor1, c_mcor2, c_mcr bits */
#define MCOR_DSR		0x80		/* detect transition on DSR */
#define MCOR_CD			0x40		/* detect transition on CD */
#define MCOR_CTS		0x20		/* detect transition on CTS */
#define MCOR_THRESH_MASK	0xf		/* DTR threshold mask */
#define MCOR_THRESH_1		0x1		/* threshold level 1 char */
#define MCOR_THRESH_2		0x2		/*	""	   2 char */
#define MCOR_THRESH_3		0x3		/*	""	   3 char */
#define MCOR_THRESH_4		0x4		/*	""	   4 char */
#define MCOR_THRESH_5		0x5		/*	""	   5 char */
#define MCOR_THRESH_6		0x6		/*	""	   6 char */
#define MCOR_THRESH_7		0x7		/*	""	   7 char */
#define MCOR_THRESH_8		0x8		/*	""	   8 char */

/* c_msvr bits */
#define MSVR_RTS		0x01
#define MSVR_DTR		0x02
#define MSVR_CTS		0x20
#define MSVR_CD			0x40
#define MSVR_DSR		0x80

/*
 * MCR bits
 */
#define MCR_DCTS		0x20
#define MCR_DCD			0x40
#define MCR_DDSR		0x80

/*
 * GIVR bits
*/
#define GIVR_MD			0x01
#define GIVR_TX			0x02
#define GIVR_RX			0x03
#define GIVR_RS			0x07
#define GIVR_TYP_MASK		0x07

/*
 * RCSR bits
 */
#define RCSR_TOUT		0x80		/* rx FIFO empty, no new data */
#define RCSR_SC_DETMASK		0x70		/* special char detect mask */
#define RCSR_BRK		0x08		/* break detection */
#define RCSR_PE			0x04		/* parity error */
#define RCSR_FE			0x02		/* framing error */
#define RCSR_OE			0x01		/* overrun error */
#define RCSR_SCD1		0x10		/* special char - start */
#define RCSR_SCD2		0x20		/* special char - stop */
#define RCSR_SCD3		0x30		/* special char - start */
#define RCSR_SCD4		0x40		/* special char - stop */

/*
 * GFRCR definitions
 *
 * Must use the SRCR_PKGTYPE bit in the SRCR to
 *  distinguish between the 180 rev. B and the
 *  1864 rev. A.  Note that we never built any
 *  products based on the 1864.
 */
#define GFRCR_CD180_REVA	0x80	/* ???CL-CD180 Rev. A 84 pin PLCC */
#define GFRCR_CD180_REVB	0x81	/* CL-CD180 Rev. B 84 pin PLCC */
#define GFRCR_CD180_REVC	0x82	/* CL-CD180 Rev. C 84 pin PLCC */
#define GFRCR_CD1864_REVA	0x82	/* CL-CD1864 Rev. A 100 pin PQFP */
#define GFRCR_CD1865_REVA	0x83	/* CL-CD1865 Rev. A 100 pin PQFP */

/* ------------------------------------------------------------------------- */
/* New types and type-specific macros */

/*
 * layout of the Cirrus Logic chip registers
 */
struct cd180 {
	u_char		pad0[1];
	u_char		ccr;
	u_char		srer;
	u_char		cor1;
	u_char		cor2;
	u_char		cor3;
	u_char		ccsr;
	u_char		rdcr;
	u_char		pad1[1];
	u_char		schr1;
	u_char		schr2;
	u_char		schr3;
	u_char		schr4;
	u_char		pad2[3];
	u_char		mcor1;
	u_char		mcor2;
	u_char		mcr;
	u_char		pad3[5];
	u_char		rtpr;
	u_char		pad4[15];
	u_char		msvr;
	u_char		msvrts;
	u_char		msvdtr;
	u_char		pad5[6];
	u_char		rbprh;
	u_char		rbprl;
	u_char		pad6[6];
	u_char		tbprh;
	u_char		tbprl;
	u_char		pad7[5];
	u_char		gsvr;
	u_char		gscr;
	u_char		pad8[31];
	u_char		msmr;
	u_char		tsmr;
	u_char		rsmr;
	u_char		car;
	u_char		srsr;
	u_char		srcr;
	u_char		pad10[4];
	u_char		gfrcr;
	u_char		pad11[4];
	u_char		pprh;
	u_char		pprl;
	u_char		pad12[6];
	u_char		rdr;
	u_char		pad13[1];
	u_char		rcsr;
	u_char		tdr;
	u_char		pad14[3];
	u_char		eosrr;
};

