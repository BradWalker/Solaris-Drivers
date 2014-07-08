/*
 * Aurora Cirrus CL-CD180/1865 Async Driver (sio16)
 *
 * This module is the bulk of the STREAMS driver for
 *  the Cirrus Logic CL-CD180 and CL-CD1865.
 *
 * COPYRIGHT (c) 1993-1996 BY AURORA TECHNOLOGIES, INC., WALTHAM, MA.
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
 *	file: sio16_osdep.c
 *	author: bwalker
 *	created: 2/1/93
 *	revision info: $Id: sio16.c,v 1.16.2.4 1996/10/17 20:25:14 bkd Exp $
 */

/*
 * $Log: sio16.c,v $
 * Revision 1.16.2.4  1996/10/17 20:25:14  bkd
 * Moved input flow control setup to the sio16_iflow_setup
 * routine.  This fixes a little bug with SIO_SETRTS and
 * SIO_SETDTR.
 *
 * Revision 1.16.2.3  1996/10/17 03:40:28  bkd
 * + switched to new xxx_mask modem/flow control line handling
 *   mechanism.  This gets rid of the old "DTRSWAP" macros, etc.
 *   It also *greatly* clarifies the modem control ioctls,
 *   and gets rid of the constant port number checks (>= 0xc,
 *   and <= 0xf).
 * + cached srer.  This improves performance a little bit (by
 *   minimizing access to the hardware), and fixes a major bug.
 *   The driver used to constantly reset RSER to IER_MDRX.
 *   This would re-activate receive interrupts.  This broke the
 *   hardware input flow control mechanism that used the built
 *   in DTR function.  Input hardware flow control is now much
 *   more robust.
 * + as part of the new xxx_mask mechanism, I cleaned up mdintr
 *   quite a bit.  The two CD cases have been folded into one.
 *
 * Revision 1.16.2.2  1996/10/16 20:17:55  bkd
 * Incorporated rsrv re-design (as initially done in
 * the 400-800.svr4 async driver):
 * + added bufcall mechanism to rsrv (and added rersrv)
 * + redesigned rsrv data loop to minimize the number of
 *   mblks that are sent up by maximizing the size of
 *   the mblks.
 * + removed local queueing of mblks (this also removes
 *   the second data loss point in rsrv)
 * + fixed reception re-activation in rsrv (used to always
 *   call istart; now waits until low water mark is
 *   crossed).
 * + re-arranged qenable code in rx/rsintr.  Do the
 *   qenable after possibling calling istop.
 *
 * Revision 1.16.2.1  1996/10/02 17:50:50  bkd
 * Fixed crash error in SIO_STATS_RST_ALL -- the line pointer
 * argument to XT was 0.  This caused 'mset -statsreset' to crash
 * the system.
 *
 * Revision 1.16  1996/09/27 14:23:51  bkd
 * + modified to meet Aurora coding standards (4 space indent, fully
 *   ANSIfied, complete and proper prototypes, objects sorted by type in
 *   source file, standard Aurora copyright boilerplate, RCS ids and
 *   copyrights, no long lines, only boolean expressions (no arithmetic
 *   expressions) in for, if, while, and do, mandatory opening and
 *   closing braces after for, if, and while)
 * + pared-down the headers by removing unnecessary includes
 * + removed support for old hardware (1600S, 1600SM)
 * + removed support for non-Solaris 2 (e.g. SunOS 4) operating systems
 * + switched to ddi_soft_state (removed sio16_boards array of pointers);
 *   this pretty much removes the board count restriction
 * + changed line data structures to be an array of structures pointed to
 *   by the board structure (it was an array of pointers to
 *   individually-allocated structures).  This decreases fragmentation
 *   and speeds up accesses slightly.
 * + gave global variables more unique names (sio_stats_chk ->
 *   sio16_stats_chk, xxtrace_setup -> sio16_xtinit, iblock ->
 *   sio16_iblock)
 * + added new 'cflag' line variable -- contains previous value t_cflag;
 *   it's used when the values in a new cflag are not valid (e.g. baud
 *   rate)
 * + changed sio16_poll to take a pointer to a board structure.  Used to
 *   take a board number.  This change is necessary for the
 *   ddi_soft_state conversion.
 * + changed close_timeout to be initialized in sio16_osdep.c
 * + moved baud rate tables, sio16_close_timeout to sio16_osdep.c
 * + converted to use the new, standard XXTRACE (xxtrace.svr4)
 * + switched to ACSA-style transparent ioctls.  Now all transparent
 *   ioctls are properly handled.
 * + added support for Solaris 2.5 extended baud rates
 * + added casts to eliminate warnings in XT statements (specifically
 *   pointers)
 * + moved decommission to sio16_osdep.c/sio16_decommission
 * + removed old, previously ifdef'ed-out mutex trace code
 * + removed unncessary 'xxx' variable declarations
 * + changed all 'struct cd180's to be volatile
 * + use "ttymodes" property to initialize port parameters on first open
 * + de-macrofied many Solaris 2-hiding macros (e.g. cv_wait_sig,
 *   qprocson, etc.)
 * + added inclusion of cd18x.h
 * + made static all routines that are local to sio16.c
 *
 * Revision 1.15  1996/09/13 16:07:05  chung
 * Make sure to signal the end of service context properly in RXINTR.
 *
 * Revision 1.14  1996/05/28 18:01:51  chung
 * Change to make sure a cu device cannot be opened if its corresponding
 * term device has already been opened.
 *
 * Revision 1.13  1995/08/15 23:56:21  bkd
 * Bug fix attempt: panic: mutex reentered (poll->txintr->start->
 * ioctl->qreply calling ldterm calling wput); fixed by adding
 * 'intr' parameter to ioctl()
 *
 * Revision 1.12  1995/08/15  23:14:11  bkd
 * Updated copyrights; added CVS control information
 *
 */

/*
 * bkd 5/10/1996 - under SunOS 5.3, sys/stropts.h includes sys/conf.h,
 *  which includes sys/systm.h, which declares a routine called 'min'.
 *  sys/ddi.h defines 'min' as a macro, and this screws things up.  So,
 *  we must include sys/stropts.h *before* sys/ddi.h
 */
#include <sys/types.h>
#include <sys/stream.h>
#include <sys/stropts.h>	/* for FLUSHW &c */
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/kmem.h>	/* for KM_SLEEP */
#include <sys/fcntl.h>	/* for O_NDELAY and O_NONBLOCK */
#include <sys/tty.h>	/* for tty_common_t */
#include <sys/errno.h>
#include <sys/mkdev.h>	/* needed for NBITSMINOR macro */
#include <sys/debug.h>	/* for the ASSERT() macro */

#include "sio16.h"
#include "sio16_ioctl.h"
#include "cd18x.h"

#define XXTRACE_HOST
#include "xxtrace.h"
#include "event.h"


/* ------------------------------------------------------------------------- */
/* Version variables */

static char RcsId[] = "@(#)$Header: /vol/sources.cvs/dev/sio16.svr4/sio16.c,v 1.16.2.4 1996/10/17 20:25:14 bkd Exp $";
static char copyright[] =
		"@(#)Copyright (c) 1993-1996 Aurora Technologies, Inc.";

/* ------------------------------------------------------------------------- */
/* General macros */

/* Driver config defines */
#define ISPEED		B9600
#define IFLAGS		(IXON | IXOFF)
#define CFLAGS		(CS7 | CREAD | PARENB | HUPCL)

#define IER_MDRX	(IER_DSR | IER_CTS | IER_RXDATA | IER_RET)

#if !defined(CBAUDEXT) && defined(SUNOS5)
	/* from Solaris 2.5 /usr/include/sys/termios.h */
#   define CBAUDEXT	010000000
#   define CIBAUDEXT	020000000
#endif		/* !CBAUDEXT */

#ifdef CBAUDEXT
#   define MAXBAUD	22	/* B460800 */
	/* extract baud from cflags */
#   define GETBAUD(cf)			\
	( ((cf) & CBAUDEXT) != 0 ? (((cf) & CBAUD) + CBAUD + 1)	\
				 : ((cf) & CBAUD) )
#else		/* CBAUDEXT */
#   define MAXBAUD	B38400
	/* extract baud from cflags */
#   define GETBAUD(cf)			\
				((cf) & CBAUD)
#endif		/* !CBAUDEXT */

/* ------------------------------------------------------------------------- */
/* External prototypes */

/* ------------------------------------------------------------------------- */
/* Static prototypes */

static int sio16_ioctl(struct sioline *siop, queue_t *q, mblk_t *mp,
			boolean intr);
static void sio16_reioctl(long arg);
static void sio16_param(struct sioline *siop);
static void sio16_setdtrrts(struct sioline *siop, u_int value);
static int sio16_start(struct sioline *siop);
static void sio16_restart(caddr_t arg);
static void sio16_rxintr(struct sioline *siop, int norm);
static void sio16_rsintr(struct sioline *siop);
static void sio16_txintr(struct sioline *siop);
static void sio16_mdintr(struct sioline *siop);
static int sio16_sendbreak(struct sioline *siop);
static int sio16_endbreak(struct sioline *siop);
static void sio16_istart(struct sioline *siop);
static void sio16_istop(struct sioline *siop, boolean intr);
static void sio16_iocput(struct sioline *siop,
			queue_t *q, mblk_t *mp, int cmd);
static unsigned int sio16_iocin(int ioc_cmd);
static unsigned int sio16_iocout(int cmd);
static void chip_dump(struct sioline *siop);
static void sio16_rersrv(long arg);
static void sio16_iflow_setup(struct sioline *siop);

/* ------------------------------------------------------------------------- */
/* External variables */

extern kcondvar_t lbolt_cv;

/* ------------------------------------------------------------------------- */
/* Un-initialized static variables */

/* ------------------------------------------------------------------------- */
/* Un-initialized global variables */

/*
 * statistics structure
 */
sio_stats_check_t sio16_stats_chk;

/* ------------------------------------------------------------------------- */
/* Initialized static variables */

/* Variable used to track devices */
static int hold_dtr_low = 3;

/* ------------------------------------------------------------------------- */
/* Initialized global variables */


#ifdef XXTRACE
int sio16_xtinit = 0;		/* xxtrace initialized flag */
#endif		/* XXTRACE */

/*
 *  debug flags used during attach, modload, & and other routines
 *  until xxtraceinit() gets called
 */
int sio16_debug = 0;

/* ------------------------------------------------------------------------- */
/* Global routines */


/*
 * !!!bkd 9/13/1996 - rewrite this confusing thing.
 */
int
sio16_open(queue_t *q, dev_t *devp, int flag, int sflag, cred_t *credp)
{
    register struct sio16_board		*sbp;
    register struct sioline		*siop;
    register volatile struct cd180	*cdp;
    register tty_common_t		*ttc;
    struct stroptions			*strop;
    mblk_t				*bp;
    u_char				 msvr;
    int					 minor_dev, tiop_len;
    struct termios			*tiop;

#ifdef XXTRACE
    /*
     * By the time the first open is called, all of the attaches
     * have been done and we know the max number of boards and
     * devices out there.
     */
    if (sio16_xtinit == 0) {
	xxtraceinit(sio16_iblock);
	sio16_xtinit = 1;
    }
#endif

    minor_dev = getminor(*devp);

    /*
     * Is this a valid board?
     */
    if ((sbp = (struct sio16_board *) ddi_get_soft_state(sio16_head,
						BOARD(minor_dev))) == NULL) {
	return ENODEV;
    }

    /*
     * Is this a valid unit on the board?
     */

    if (UNIT(minor_dev) >= sbp->nport) {
	return ENODEV;
    }

    siop = &(sbp->siolines[UNIT(minor_dev)]);
    cdp = siop->cd180;
    ttc = &siop->ttycommon;
    XT(OPEN_L1_ENTRY, siop, (unsigned int) q, flag, minor_dev);

    CRITICAL(siop);
again:
    if ((siop->flags & SIO_ISOPEN) == 0) {
	XT(OPEN_L2_1STOPEN, siop, 0, 0, 0);

	/*
	 * allocate statistics structures, ONE TIME ONLY
	 */
	if (siop->l_sio_stats == NULL) {
	    if ((siop->l_sio_stats = (caddr_t)
			kmem_zalloc(sizeof(sio_stats_t), KM_SLEEP)) == NULL) {

		UNCRITICAL(siop);
		XT(OPEN_L1_EXIT, ND, __LINE__, 0, 0);
		return ENOMEM;
	    }
	}
	if (siop->l_sio_stats_valid == NULL) {
	    if ((siop->l_sio_stats_valid = (caddr_t)
			kmem_zalloc(sizeof(sio_stats_t), KM_SLEEP)) == NULL) {

		UNCRITICAL(siop);
		XT(OPEN_L1_EXIT, ND, __LINE__, 0, 0);
		return ENOMEM;
	    }
	}
	SIO_RING_INIT(siop);
	siop->dev = minor_dev;
	siop->bufcall = 0;		/* zero out bufcall */
	siop->iflow_mask = 0;

	/*
	 * Fetch the default value for cflag from the "ttymodes" property
	 *  stored in the root node (see file /kernel/drv/options.conf)
	 */
	/* Set the "previous" cflag values to the hard-coded defaults */
	siop->cflag = (ISPEED << IBSHIFT) | ISPEED | CFLAGS;

	/*
	 * !!!bkd 9/18/1996 - why aren't we looking at the
	 *  other properties in 'struct termios'?
	 */

	if (ddi_getlongprop(DDI_DEV_T_ANY, ddi_root_node(), 0,
			    "ttymodes", (caddr_t) &tiop, &tiop_len)
							== DDI_PROP_SUCCESS) {
	    if (tiop_len != sizeof(struct termios)) {
		cmn_err(CE_WARN, "sio16_open: invalid termios length (%d)",
					    tiop_len);
		ttc->t_cflag = (ISPEED << IBSHIFT) | ISPEED | CFLAGS;
	    }
	    else {
		ttc->t_cflag = tiop->c_cflag;
	    }

	    kmem_free((void *) tiop, tiop_len);
	}
	else {
	    cmn_err(CE_WARN, "sio16_open: couldn't get ttymodes property");
	    ttc->t_cflag = (ISPEED << IBSHIFT) | ISPEED | CFLAGS;
	}

	ttc->t_iflag = IFLAGS;
	ttc->t_stopc = CSTOP;
	ttc->t_startc = CSTART;
	ttc->t_iocpending = NULL;
	ttc->t_size.ws_row = 0;
	ttc->t_size.ws_col = 0;
	ttc->t_size.ws_xpixel = 0;
	ttc->t_size.ws_ypixel = 0;

	/*
	 * Set the high-water and low-water marks on the stream head
	 */
#if 0
	if ((bp = allocb((int) sizeof(struct stroptions), BPRI_MED)) != NULL) {
	    strop = (struct stroptions *)bp->b_wptr;
	    strop->so_flags = SO_HIWAT | SO_LOWAT | SO_MAXPSZ | SO_MINPSZ;
	    strop->so_hiwat = sio16_info.mi_hiwat;
	    strop->so_lowat = sio16_info.mi_lowat;
	    strop->so_maxpsz = INFPSZ;
	    strop->so_minpsz = sio16_info.mi_minpsz;
	    bp->b_wptr += sizeof (struct stroptions);
	    bp->b_datap->db_type = M_SETOPTS;
	    putnext(q, bp);
	}
#endif

	SETCHAN(siop);

	/*
	 * if ttysoftcar is set, DO NOT generate interrupts
	 * on CD transistions.
	 */
	if ((ttc->t_flags & TS_SOFTCAR) == 0) {
	    if (siop->cd_mask == MSVR_DSR) {
		/* CD pin is seen as DSR pin on CD180 */
		cdp->mcor1 = MCOR_DSR;
		cdp->mcor2 = MCOR_DSR;
	    }
	    else if (siop->cd_mask == MSVR_CTS) {
		/* last 4 ports share CD/CTS with CTS pin */
		cdp->mcor1 = MCOR_CTS;
		cdp->mcor2 = MCOR_CTS;
	    }
	}
	/* enable interrupt only when it is asked to open */
	sio16_param(siop);
	siop->srer = IER_MDRX;		/* turn on interrupts */
	cdp->srer = siop->srer;

	/* enable transmitter and receiver */
	cdp->ccr = CCR_TXEN | CCR_RXEN;
	DELAYCK(cdp->ccr, UNIT(siop->dev), "enable tx and rx");

    }
    else if ((ttc->t_flags & TS_XCLUDE) != 0 && !PRIV(credp)) {
	/*
	 * check for exclusive flag, allow root to override
	 */
	UNCRITICAL(siop);
	XT(OPEN_L1_EXIT, siop, __LINE__, 0, 0);
	return EBUSY;
    }
    else if (ISOUT(minor_dev) && (siop->flags & SIO_OUT) == 0) {
	/*
	 * modem device cannot open port if terminal device
	 * already has it open.
	 */
	UNCRITICAL(siop);
	XT(OPEN_L1_EXIT, siop, __LINE__, 0, 0);
	return EBUSY;
    }
    SETCHAN(siop);
    /* raise dtr/rts */
    sio16_setdtrrts(siop, SIO_ON);

    /*
     * delay for 50 usec because of possible
     * crosstalk that can influence the state
     * of the CD line.  this was seen when the
     * lines were not driven by the peripheral.
     */
    DELAY(100);

    /*
     * read the modem control lines & check for DCD
     */

    msvr = cdp->msvr;

    if (ISOUT(minor_dev)) {
	siop->flags |= SIO_OUT | SIO_CARR_ON;
    }
    else if ((ttc->t_flags & TS_SOFTCAR) != 0
	     || (siop->cd_mask == 0 || (msvr & siop->cd_mask) != 0)) {

	siop->flags |= SIO_CARR_ON;
    }

    if ((flag & (O_NDELAY | O_NONBLOCK)) == 0
	&& (ttc->t_cflag & CLOCAL) == 0) {

	if ((siop->flags & SIO_CARR_ON) == 0
	    || ((siop->flags & SIO_OUT) != 0 && !ISOUT(minor_dev))) {

	    siop->flags |= SIO_WOPEN;
	    XT(OPEN_L2_SLEEP_CD, siop, 0, 0, 0);

	    if (cv_wait_sig(&(siop->cv), siop->chiplock) == 0) {
		XT(OPEN_L2_SLEEP_INT, siop, __LINE__, 0, 0);
		if ((siop->flags & SIO_ISOPEN) == 0) {
		    /*
		     * drop DTR
		     */
		    sio16_setdtrrts(siop, SIO_OFF);
		    SETCHAN(siop);
		    siop->srer = 0;
		    cdp->srer = siop->srer;
		}
		siop->flags &= ~SIO_WOPEN;
		UNCRITICAL(siop);
		XT(OPEN_L1_EXIT, siop, __LINE__, 0, 0);

		return EINTR;
	    }
	    goto again;
	}
    }
    else if ((siop->flags & SIO_OUT) != 0 && !(ISOUT(minor_dev))) {
	/*
	 * terminal device cannot open device if
	 * modem device already has port opened
	 */
	UNCRITICAL(siop);
	XT(OPEN_L1_EXIT, siop, __LINE__, 0, 0);
	return EBUSY;
    }

    ttc->t_readq = q;
    ttc->t_writeq = WR(q);
    q->q_ptr = WR(q)->q_ptr = (caddr_t) siop;
    /*
     * the the WOPEN flag is cleared because we are no longer
     * waiting for the getty to wakeup & OSTOPPED is cleared
     * because we are going to be sending data out
     */
    siop->flags &= ~(SIO_WOPEN | SIO_OSTOPPED);
    qprocson(q);
    siop->flags |= SIO_ISOPEN;

    UNCRITICAL(siop);
    XT(OPEN_L1_EXIT, siop, 0, 0, 0);
    return 0;
}

int
sio16_close(queue_t *rq, int flag)
{
    register struct sioline		*siop;
    register volatile struct cd180	*cdp;
    unsigned long			 start, now;

    if ((siop = (struct sioline *) rq->q_ptr) == NULL) {
	return 0;
    }

    XT(CLOSE_L1_ENTRY, siop, (unsigned int) rq, flag, 0);
    cdp = siop->cd180;

    GETTIME(&start);

    XT(CLOSE_L2_WAIT, siop, (unsigned int) siop->lasttxmp,
				SIO_RING_CNT(siop), siop->flags);
    CRITICAL(siop);

    while ((siop->flags & (SIO_BREAK | SIO_DELAY)) != 0
	   || siop->lasttxmp != NULL
	   || SIO_RING_CNT(siop) != 0) {

	if (cv_wait_sig(&lbolt_cv, siop->chiplock) != 0) {
	    /* XXX need XT event here... */
	    break;
	}
	GETTIME(&now);

	if (siop->close_timeout != 0
	    && (int) HZTOSEC(now - start) > siop->close_timeout) {

	    XT(CLOSE_L2_TIMEOUT, siop, now, 0, 0);
	    break;
	}
    }
    /*
     * remove any restart timeouts that might exist
     */
    if (siop->timeoutpending != 0) {
	untimeout(siop->timeoutid);
    }

    /*
     * kill the pending bufcall
     */
    if (siop->bufcall != 0) {
	unbufcall(siop->bufcall);
	siop->bufcall = 0;
    }

    if (SIO_RING_CNT(siop) != 0) {  /* clear the ring of any received chars. */
	SIO_RING_INIT(siop);
    }

    if (siop->lasttxmp != NULL) {	/* drop the tx mblk */
	freemsg(siop->lasttxmp);
	siop->lasttxmp = NULL;
    }

    /*
     * reset channel and turn off interrupts
     */
    SETCHAN(siop);

    siop->srer = 0;
    cdp->srer = siop->srer;

    cdp->ccr = CCR_RST_CHAN;	/* reset channel */
    DELAYCK(cdp->ccr, UNIT(siop->dev), "reset channel");

    if ((siop->flags & (SIO_WOPEN | SIO_ISOPEN)) != SIO_ISOPEN
	|| (siop->ttycommon.t_cflag & HUPCL) != 0) {

	/* Drop dtr/rts */
	XT(CLOSE_L2_DTR, siop, 0, 0, 0);
	sio16_setdtrrts(siop, SIO_OFF);
    }

    siop->flags = 0x0;
    ttycommon_close(&siop->ttycommon);
    rq->q_ptr = WR(rq)->q_ptr = NULL;
    cv_signal(&(siop->cv));
    UNCRITICAL(siop);
    qprocsoff(rq);
    XT(CLOSE_L1_EXIT, siop, 0, 0, 0);
    return 0;
}

int
sio16_wput(queue_t *q, mblk_t *mp)
{
    struct sioline		*siop;
    struct iocblk		*iocp;
    unsigned int		 size_needed;

    if ((siop = (struct sioline *) q->q_ptr) == NULL) {
	freemsg(mp);
	XT(WPUT_L1_EXIT, ND, __LINE__, 0, 0);
	return 0;
    }

    XT(WPUT_L1_ENTRY, siop, (unsigned int) q, (unsigned int) mp, 0);

    switch (mp->b_datap->db_type) {

    case M_STOP:		/* We received ^S */
	XT(WPUT_L2_M_STOP, siop, 0, 0, 0);
	CRITICAL(siop);
	if ((siop->flags & SIO_OSTOPPED) == 0) {
	    siop->flags |= SIO_OSTOPPED;
	}
	UNCRITICAL(siop);
	freemsg(mp);
	break;

    case M_START:		/* We received ^Q */
	XT(WPUT_L2_M_START, siop, 0, 0, 0);
	CRITICAL(siop);
	if ((siop->flags & SIO_OSTOPPED) != 0) {
	    siop->flags &= ~SIO_OSTOPPED;
	    /*
	     * output was stopped so turn on interrupts this will
	     * allow the system to setup the proper TX context
	     */
	    SETCHAN(siop);
	    /* siop->cd180->srer = (IER_MDRX | IER_TXRDY); */
	    siop->srer |= IER_TXRDY;
	    siop->cd180->srer = siop->srer;
	}
	UNCRITICAL(siop);
	freemsg(mp);
	break;

    /*
     * process ioctl command
     */
    case M_IOCTL:
        iocp = (struct iocblk *) mp->b_rptr;

	XT(WPUT_L2_M_IOCTL, siop,
			iocp->ioc_cmd, iocp->ioc_id, iocp->ioc_count);

        if (iocp->ioc_count == TRANSPARENT &&
	    (size_needed = sio16_iocin(iocp->ioc_cmd)) != 0) {

	    struct copyreq *cp;

	    cp = (struct copyreq *) mp->b_rptr;

	    cp->cq_flag = 0;
	    cp->cq_size = size_needed;
	    cp->cq_addr = *((caddr_t *) mp->b_cont->b_rptr);

	    ASSERT((int) cp->cq_addr != 0);	/* it's important */

	    XT(WPUT_L2_M_IOCTL_TRANS, siop, size_needed, (int) cp->cq_addr, 0);

	    /*
	     * Stow away a copy of the user address.
	     *  This is for ioctls that are both
	     *  input and output (e.g. SETBAUD).
	     *  A non-NULL cq_private indicates that
	     *  the M_IOCDATA we will receive is the
	     *  response to an M_COPYIN message.  A NULL
	     *  cq_private indicates that it is a response
	     *  to an M_COPYOUT.  See M_IOCDATA, below.
	     */
	    cp->cq_private = (mblk_t *) cp->cq_addr;
	    freemsg(mp->b_cont);
	    mp->b_cont = NULL;

	    mp->b_wptr = mp->b_rptr + sizeof(struct copyreq);
	    mp->b_datap->db_type = M_COPYIN;

	    qreply(q, mp);
	    break;	/* outside of switch */
	}
	else {
	    sio16_iocput(siop, q, mp, iocp->ioc_cmd);
	}
	break;

    /*
     * Reply from M_COPYOUT and M_COPYIN messages.
     */
    case M_IOCDATA:
    {
	struct copyresp		*cp;

	cp = (struct copyresp *) mp->b_rptr;

	XT(WPUT_L2_M_IOCDATA, siop, cp->cp_cmd, cp->cp_id,
			(unsigned int) cp->cp_private);

	if (cp->cp_rval != 0) {	/* failure */
	    XT(WPUT_L2_M_IOCDATA_FAIL, siop, (unsigned int) cp->cp_rval, 0, 0);
	    freemsg(mp);
	}
	else {
	    iocp = (struct iocblk *) mp->b_rptr;

	    /*
	     * Is this is a copyout response?  If so, then
	     *  break...we're done.
	     */

	    if (cp->cp_private != (mblk_t *) 0) {
		/*
		 * Message appears to be a response to an
		 *  M_COPYIN request.
		 */

		if (mp->b_cont == NULL) {
		    /*
		     * Is this a copyin reply with no data attached?
		     *  (Shouldn't happen)
		     */

		    XT(WPUT_L2_M_IOCDATA_INERR, siop,
						cp->cp_cmd, cp->cp_id, 0);
		    mp->b_datap->db_type = M_IOCNAK;
		    iocp->ioc_error = EINVAL;
		    qreply(q, mp);
		}
		else {
		    sio16_iocput(siop, q, mp, cp->cp_cmd);
		}
	    }
	    else if ((size_needed = sio16_iocout(cp->cp_cmd)) != 0) {
		/*
		 * This appears to be a response to an M_COPYOUT.
		 */
		XT(WPUT_L2_M_IOCDATA_OUT, siop, cp->cp_cmd, cp->cp_id, 0);

		iocp->ioc_error = 0;
		iocp->ioc_rval = 0;
		iocp->ioc_count = size_needed;
		mp->b_datap->db_type = M_IOCACK;
		qreply(q, mp);
	    }
	    else {
		/*
		 * This appears to be a response to an M_COPYOUT,
		 *  but we don't seem to believe that we needed
		 *  data copied out.  (This shouldn't happen.)
		 */
		XT(WPUT_L2_M_IOCDATA_OUTERR, siop, cp->cp_cmd, cp->cp_id, 0);
		mp->b_datap->db_type = M_IOCNAK;
		iocp->ioc_error = EINVAL;
		qreply(q, mp);
	    }
	}
    }
	break;

    case M_FLUSH:
	XT(WPUT_L2_M_FLUSH, siop, 0, 0, 0);

	if ((*mp->b_rptr & FLUSHW) != 0) {
	    /*
	     * abort any output in progress.  we protect around the flushq
	     * because we could get an txintr that would cause us to
	     * hang
	     */
	    CRITICAL(siop);
	    flushq(q, FLUSHDATA);	/* flush write queue */

	    if ((siop->flags & SIO_BUSY) != 0) {
		siop->flags &= ~SIO_BUSY;
	    }

	    if (siop->lasttxmp != NULL) {
		freemsg(siop->lasttxmp);
		siop->lasttxmp = NULL;
	    }

	    UNCRITICAL(siop);
	    *mp->b_rptr &= ~FLUSHW;	/* flush is complete */
	}

	if ((*mp->b_rptr & FLUSHR) != 0) {
	    CRITICAL(siop);
	    flushq(RD(q), FLUSHDATA);
	    UNCRITICAL(siop);
	    qreply(q, mp);
	}
	else {
	    freemsg(mp);
	}

	/*
	 * We must make sure we process messages that survive the
	 * write-side flush.  Without this call, the close protocol
	 * with ldterm can hang forever.  (ldterm will have sent us a
	 * TCSBRK ioctl that it expects a response to.)
	 *
	 * in order to do this we must turn on interrupts so the
	 * system can setup the proper TX context and thus generate a txintr
	 */
	CRITICAL(siop);
	SETCHAN(siop);
	siop->srer = (IER_MDRX | IER_TXRDY);
	siop->cd180->srer = siop->srer;
	UNCRITICAL(siop);
	break;

    case M_STOPI:	/* Send ^S */
	XT(WPUT_L2_M_STOPI, siop, 0, 0, 0);
	CRITICAL(siop);
	sio16_istop(siop, FALSE);
	UNCRITICAL(siop);
	freemsg(mp);
	break;

    case M_STARTI:	/* Send ^q */
	XT(WPUT_L2_M_STARTI, siop, 0, 0, 0);
	CRITICAL(siop);
	sio16_istart(siop);
	UNCRITICAL(siop);
	freemsg(mp);
	break;

    case M_BREAK:
    case M_DELAY:
	XT(WPUT_L2_M_BREAK, siop, 0, 0, 0);
	putq(q, mp);
	/*
	 * we have data to send out so we turn on the TX interrupts
	 * so that the chip will have a chance to set the proper context
	 * and then interrupt us with a TX intr. signaling that it's ready
	 */
	CRITICAL(siop);
	SETCHAN(siop);
	/* siop->cd180->srer = (IER_MDRX | IER_TXRDY); */
	siop->srer |= IER_TXRDY;
	siop->cd180->srer = siop->srer;
	UNCRITICAL(siop);
	break;

    case M_DATA:
	XT(WPUT_L2_M_DATA, siop, 0, 0, 0);
	putq(q, mp);
	/*
	 * we have data to send out so we turn on the TX interrupts
	 * so that the chip will have a chance to set the proper context
	 * and then interrupt us with a TX intr. signaling that it's ready
	 */
	CRITICAL(siop);
	SETCHAN(siop);
	siop->cd180->srer = (IER_MDRX | IER_TXRDY);
	siop->srer |= IER_TXRDY;
	siop->cd180->srer = siop->srer;
	UNCRITICAL(siop);
	break;

    default:
	XT(WPUT_L2_UNKNOWN, siop, mp->b_datap->db_type, 0, 0);
	freemsg(mp);
	break;

    }

#if defined(DEBUG) && defined(XXTRACE)
    chip_dump(siop);
#endif		/* DEBUG */

    XT(WPUT_L1_EXIT, siop, 0, 0, 0);
    return 0;
}

int
sio16_rsrv(queue_t *q)
{
    register struct sioline		*siop;
    register int			 cc, total_cnt, mblk_size;
    register mblk_t			*bp;
    register boolean			 done;

    if ((siop = (struct sioline *) q->q_ptr) == NULL) {
	XT(RSRV_L1_EXIT, ND, __LINE__, 0, 0);
	return 0;
    }

    XT(RSRV_L1_ENTRY, siop, (unsigned int) q, 0, 0);

    /*
     * bkd 10/8/1996 - I've redesigned this code a
     *  little bit, based on the work I did on the
     *  CD240X FIFO-mode async driver.
     *
     * Now, the local receive queue is *only* used
     *  by the ISRs for storage of control/high-priority
     *  messages.
     */
    done = FALSE;
    while (!done) {
	/*
	 * Loop through and pull messages off of the
	 *  local queue.
	 */
	while ((bp = getq(q)) != NULL) {
	    if (bp->b_datap->db_type < QPCTL && !canputnext(q)) {
		/*
		 * We can't put the message up because
		 *  it is low priority and the Stream above
		 *  is full.  We'll put it back on the local
		 *  queue.
		 * The putbq call is bracketed by noenable/enableok
		 *  calls so that putbq does not automatically
		 *  re-schedule this queue; this would be an
		 *  infinite loop.  We will wait to be back-enabled
		 *  when the upper Stream clears up a little.
		 */
		noenable(q);
		putbq(q, bp);
		enableok(q);
		done = TRUE;
		break;
	    }

	    XT(RSRV_L2_PUTNEXT, siop, (int) bp, 0, 0);
	    putnext(q, bp);
	}

	/*
	 * Drain the receive ring.
	 */

	if (!done && (cc = SIO_RING_CNT(siop)) > 0 && canputnext(q)) {
	    XT(RSRV_L2_RINGCC, siop, cc, 0, 0);

	    /*
	     * Add a small amount to the size of the mblk.
	     *  Why?  Because while we drain the ring, an
	     *  interrupt could come in and add more characters
	     *  to the ring.  Realistically, the most this
	     *  should be is FIFOSIZE worth of characters.
	     *  FIFOSIZE is 16.
	     */

	    mblk_size = cc + 16;

	    if ((bp = allocb(mblk_size, BPRI_MED)) == NULL) {
		/*
		 * Schedule a re-enable of the rsrv
		 *  routine so that we can pull the
		 *  data out later.
		 */
		XT(RSRV_L2_ALLOCFAIL, siop, mblk_size, 0, 0);
		if (siop->bufcall != 0) {
		    unbufcall(siop->bufcall);
		}
		siop->bufcall = bufcall(mblk_size, BPRI_MED,
						sio16_rersrv, (long) q);
		break;
	    }

	    /*
	     * Now, loop until either there are no
	     *  characters left in the ring, or we fill
	     *  the mblk.
	     */
	    total_cnt = 0;
	    while (mblk_size > 0 && !SIO_RING_EMPTY(siop)) {
		*bp->b_wptr++ = SIO_RING_GET(siop);
		total_cnt++;
		mblk_size--;
	    }

	    /*
	     * Put the message to the next queue.
	     */
	    XT(RSRV_L2_MSGDONE, siop, total_cnt, mblk_size, 0);
	    SIO_STAT_UPDATE(siop, stats_receive_chars, total_cnt);
	    XT(RSRV_L2_PUTNEXT, siop, (int) bp, 0, 0);
	    putnext(q, bp);
	}
	else {
	    done = TRUE;
	}
    }

    /*
     * Have we drained the ring sufficiently so that
     *  we can turn reception back on?
     * Why, you may ask, do we duplicate the conditional
     *  below?  (Once outside of the critical, once
     *  inside the critical).  The reason is that
     *  this is an optimization that prevents us
     *  from grabbing the chip lock unnecessarily.
     */

    if ((siop->flags & SIO_THROTTLE) != 0 && SIO_RING_LOWATER(siop)) {
	CRITICAL(siop);

	if ((siop->flags & SIO_THROTTLE) != 0 && SIO_RING_LOWATER(siop)) {
	    siop->flags &= ~SIO_THROTTLE;
	    sio16_istart(siop);
	}

	UNCRITICAL(siop);
    }

    XT(RSRV_L1_EXIT, siop, 0, 0, 0);
    return 0;
}

/*
 * sio16_poll
 *
 *
 * The vector received during an iack cycle contains a binary encoding
 * of the chip number.  The vector was previously set via the
 * gsvr register.
 */
u_int
sio16_poll(caddr_t arg)
{	
    register struct sio16_board *sbp;
    register struct sioline	*siop;
    int	chip;
    u_char	port;
    u_char	vector;
    u_int	serviced;
    int	iack_offset;
    int loop;
    u_int	board;
#ifdef TIMING
    struct timeval start, end;
#endif TIMING

    sbp = (struct sio16_board *) arg;
    board = (u_int) sbp->instance;
    serviced = DDI_INTR_UNCLAIMED;

    XT(INTR_L3_ENTRY, ND, board, 0, 0);

	siop = &(sbp->siolines[0]);
	if ((siop->cd180->srsr & 0x3f) == 0) {
		return(serviced);
	}

	do {
		iack_offset = PILR1 & 0x7f;

		/*
		 * grab locks for all chips on the board.  this
		 * ensures interrupt will wait until upper layer
		 * is finished with chip before it allows chip
		 * to automatically change channel to interrupting
		 * channel.  when actual channel is determined
		 * by reading livr then release the unused chip
		 * lock.
		 */
		CHIP_MUTEX_ENTER(sbp, 0);
		CHIP_MUTEX_ENTER(sbp, 1);

		/* find out what interrupt vector we have */
		vector = *(sbp->iackbase + iack_offset);	/* do IACK */
		XT(INTR_L3_VECTOR, ND, vector, 0, 0);

		/* find out what chip this came from */
		chip = (vector >> 3) & 0x1;
		siop = &(sbp->siolines[chip << 3]);

		/*
		 * find out which port this is
		 * we shift right and mask the lower 3 bits because we're
		 * only interested in bits 4-2
		 */
		port = (((siop->cd180->gscr) >> 2) & 0x7);
		XT(INTR_L3_PORTID, ND, chip, port, 0);

		/*
		 * now we know which channel is interrupting us so we
		 * use the chip & port to index into the appropriate 
		 * data structures
		 */
		siop = &(sbp->siolines[(chip * 8) + port]);

		/*
		 * Give up the mutex for the chip we're not using.
		 */
		CHIP_MUTEX_EXIT(sbp, CHIP(siop->port) ^ 1);

#ifdef TIMING
		uniqtime(&start);
		XT(TIME_SLICE, siop, start.tv_sec, start.tv_usec, 0);
#endif

		switch (vector & GIVR_TYP_MASK) {

		case GIVR_RX:
		    sio16_rxintr(siop, RX_NORMAL);
		    serviced = DDI_INTR_CLAIMED;
		    break;

		case GIVR_TX:
		    sio16_txintr(siop);
		    serviced = DDI_INTR_CLAIMED;
		    break;

		case GIVR_RS:
		    sio16_rsintr(siop);
		    serviced = DDI_INTR_CLAIMED;
		    break;

		case GIVR_MD:
		    sio16_mdintr(siop);
		    serviced = DDI_INTR_CLAIMED;
		    break;

		}

		CHIP_MUTEX_EXIT(sbp, CHIP(siop->port));

		/*
		 * spin wait just to make we aren't needed again
		 */
		drv_usecwait(1);
		siop = &(sbp->siolines[0]);

	} while ((siop->cd180->srsr & 0x3f) != 0);


#ifdef TIMING
    uniqtime(&end);
    XT(TIME_SLICE, siop, end.tv_sec, end.tv_usec, 0);
#endif TIMING

    XT(INTR_L3_EXIT, ND, serviced, 0, 0);
    return serviced;
}

/* ------------------------------------------------------------------------- */
/* Static routines */

static int
sio16_ioctl(struct sioline *siop, queue_t *q, mblk_t *mp, boolean intr)
{
    register struct iocblk		*iocp;
    register volatile struct cd180	*cdp;
    unsigned int			 insize, outsize;
    int					 error;
    int					 nchars;    /* number of chars sent */
    register mblk_t			*outblk;
    register struct copyresp		*cp;
    register struct copyreq		*cr;
    caddr_t				 uaddr;		/* for M_COPYOUT */
    boolean				 copyout, iocdata, outalloc;
    int					 cmd, mflags;
    register unsigned char		 msvr;

    nchars = 0;
    ASSERT(LOCKED(siop));

    XT(IOCTL_L1_ENTRY, siop, (unsigned int) siop,
			(unsigned int) q, (unsigned int) mp);

    /*
     * Check to see if the given message type is M_IOCDATA.
     *  We do this here (and set the value of the boolean
     *  'iocdata') because it appears that ttycommon_ioctl
     *  rewrites this.
     */

    iocp = (struct iocblk *) mp->b_rptr;
    cp = (struct copyresp *) mp->b_rptr;

    if (mp->b_datap->db_type == M_IOCDATA) {
	iocdata = TRUE;
	uaddr = (caddr_t) cp->cp_private;
	cmd = cp->cp_cmd;
    }
    else {
	iocdata = FALSE;
	cmd = iocp->ioc_cmd;
    }

    if ((outsize = ttycommon_ioctl(&(siop->ttycommon), q, mp, &error)) != 0) {
	(void) bufcall(outsize, BPRI_HI, sio16_reioctl, (long) siop);
	XT(IOCTL_L1_EXIT, siop, __LINE__, 0, 0);
	return nchars;
    }

    if (error == 0) {
	switch (cmd) {

	case TCSETS:
	case TCSETSW:
	case TCSETSF:
	case TCSETA:
	case TCSETAW:
	case TCSETAF:
	    XT(IOCTL_L2_TCSET, siop, cmd, 0, 0);

	    if (!intr) {
		SETCHAN(siop);
	    }

	    sio16_param(siop);
	    break;

	default:
	    XT(IOCTL_L2_UNKNOWN, siop, cmd, __LINE__, iocp->ioc_count);
	    break;

	}

	/*
	 * Ioctl was handled completely by ttycommon_ioctl.
	 * It should have formatted the reply.
	 */
	iocp->ioc_error = 0;
	mp->b_datap->db_type = M_IOCACK;
    }
    else if (error < 0) {
	/*
	 * We deal with the data size ENTIRELY here.
	 *
	 * If a given ioctl requires input data, sio16_iocin
	 *  will return the size required.  The M_IOCTL or
	 *  M_IOCDATA message that this routine is processing
	 *  should already have the input data attached.
	 *  If not, then we abort the ioctl right here.
	 *
	 * If the ioctl requires output data, sio16_iocout
	 *  will return the size required.  We try to fit
	 *  the output data into b_cont.  If it won't fit
	 *  there, we allocate a new block.  In any case,
	 *  the block is pointed to by outblk.
	 *
	 * In order for this mechanism to work, it is imperative
	 *  that sio16_iocin(), and sio16_iocout() return
	 *  the correct values.  Otherwise, this code could
	 *  very well write past the end of boundaries, or
	 *  derefence NULL pointers, etc.
	 */

	error = 0;
	outalloc = FALSE;
	copyout = FALSE;

	/*
	 * Determine if the given ioctl requires
	 *  input data.
	 */

	if ((insize = sio16_iocin(cmd)) != 0
	    && (mp->b_cont == NULL ||
		(mp->b_cont->b_wptr - mp->b_cont->b_rptr) < insize)) {

	    error = EINVAL;
	}
	else if ((outsize = sio16_iocout(cmd)) != 0) {
	    /*
	     * Look at outgoing data.  First, try to figure
	     *  out how an M_COPYOUT would work, if it is
	     *  needed.  If the incoming message is an
	     *  M_IOCDATA (from an M_COPYIN request), then
	     *  we've tucked the user address in cq_private.
	     *  If it's an M_IOCTL, then the user address is
	     *  right where we'd expect it.
	     */

	    if (iocdata) {
		/* outputs MUST be by M_COPYOUT */
		/* uaddr is already initialized (above) */
		copyout = TRUE;
	    }
	    else if (iocp->ioc_count == TRANSPARENT) {
		/*
		 * mp->b_cont is guaranteed to be non-NULL (this is
		 *  an M_IOCTL with ioc_count == TRANSPARENT -- it's
		 *  a standard transparent IOCTL)
		 */
		uaddr = *((caddr_t *) mp->b_cont->b_rptr);
		copyout = TRUE;
	    }

	    /*
	     * Now, find some space to place the output results
	     *  into.  We may be able to place the results
	     *  into b_cont, or we may have to allocate a
	     *  new block.
	     */

	    if (mp->b_cont != NULL
		&& (mp->b_cont->b_datap->db_lim -
		    mp->b_cont->b_rptr) >= outsize) {

		outblk = mp->b_cont;
	    }
	    else {
		if ((outblk = allocb(outsize, BPRI_MED)) == NULL) {
		    /*
		     * If we cannot allocate space for output
		     *  data right now, then stow away the
		     *  ioctl and see what comes up later.
		     */
		    if (siop->ttycommon.t_iocpending != NULL) {
			freemsg(siop->ttycommon.t_iocpending);
		    }
		    siop->ttycommon.t_iocpending = mp;

		    (void) bufcall(outsize, BPRI_MED,
					sio16_reioctl, (long) siop);
		    XT(IOCTL_L1_EXIT, siop, __LINE__, 0, 0);

		    return nchars;
		}
		outalloc = TRUE;
	    }
	    /* error == 0 -> outblk is now non-NULL */
	}

	if (error == 0) {
	    cdp = siop->cd180;
	    if (!intr) {
		SETCHAN(siop);
	    }

	    switch (cmd) {

	    case SIO_SETTIMEOUT:
		XT(IOCTL_L2_SETTIMEOUT, siop,
				*((int *) mp->b_cont->b_rptr), 0, 0);
		siop->close_timeout = *((int *) mp->b_cont->b_rptr);
		break;

	    case SIO_GETTIMEOUT:
		XT(IOCTL_L2_GETTIMEOUT, siop, siop->close_timeout, 0, 0);

		*((int *) outblk->b_rptr) = siop->close_timeout;
		break;

	    case SIO_GETMFLG:
		XT(IOCTL_L2_GETMFLG, siop, siop->modem_flags, 0, 0);
		*((int *) outblk->b_rptr) = siop->modem_flags;
		break;

	    case SIO_SETRTS:
		XT(IOCTL_L2_RTS, siop, 0, 0, 0);
		/*
		 * Indicate the the singular output (either
		 *  MSVR_RTS or MSVR_DTR) should be used as
		 *  the logical RTS signal.
		 */
		if ((siop->modem_flags & (SIO_MAPALL | SIO_MAPDTR))
							== SIO_MAPDTR) {

		    siop->rts_mask = siop->dtr_mask;
		    siop->dtr_mask = 0;
		    siop->modem_flags &= ~SIO_MAPDTR;

		    sio16_iflow_setup(siop);
		}
		break;

	    case SIO_SETDTR:
		XT(IOCTL_L2_DTR, siop, 0, 0, 0);
		/*
		 * Indicate the the singular output (either
		 *  MSVR_RTS or MSVR_DTR) should be used as
		 *  the logical DTR signal.
		 */
		if ((siop->modem_flags & (SIO_MAPALL | SIO_MAPDTR)) == 0) {
		    siop->dtr_mask = siop->rts_mask;
		    siop->rts_mask = 0;
		    siop->modem_flags |= SIO_MAPDTR;

		    sio16_iflow_setup(siop);
		}
		break;


	    case SIO_SETCTS:
		XT(IOCTL_L2_CTS, siop, 0, 0, 0);
		/*
		 * Indicate that the singular input MSVR_CTS
		 *  should be used as the logical CTS signal.
		 */
		if ((siop->modem_flags & (SIO_MAPALL | SIO_MAPCD))
							== SIO_MAPCD) {

		    cdp->mcor1 &= ~MCOR_CTS;
		    cdp->mcor2 &= ~MCOR_CTS;

		    siop->cts_mask = siop->cd_mask;
		    siop->cd_mask = 0;
		    siop->modem_flags &= ~SIO_MAPCD;
		}

		break;

	    case SIO_SETCD:
		XT(IOCTL_L2_CD, siop, 0, 0, 0);
		/*
		 * Indicate that the singular input MSVR_CTS
		 *  will be used as the logical CD signal.
		 */
		if ((siop->modem_flags & (SIO_MAPALL | SIO_MAPCD)) == 0) {
		    if ((siop->ttycommon.t_flags & TS_SOFTCAR) == 0) {
			cdp->mcor1 |= MCOR_CTS;
			cdp->mcor2 |= MCOR_CTS;
		    }

		    siop->cd_mask = siop->cts_mask;
		    siop->cts_mask = 0;
		    siop->modem_flags |= SIO_MAPCD;
		}

		break;

	    case SIO_SETDTRFLOW:
		XT(IOCTL_L2_DTRFLOW, siop, 0, 0, 0);

		/*
		 * Use the logical DTR signal for flow
		 *  control.  This means use pin 20 of
		 *  the final DB25 for flow control.
		 */
		siop->modem_flags |= SIO_DTRFLOW;

		/*
		 * If we're properly wired, then activate
		 *  the automatic use of DTR.
		 */
		sio16_iflow_setup(siop);
		break;

	    case SIO_SETRTSFLOW:
		XT(IOCTL_L2_RTSFLOW, siop, 0, 0, 0);

		/*
		 * Use the logical RTS signal for flow
		 *  control.  This means use pin 20 of
		 *  the final DB25 for flow control.
		 */
		siop->modem_flags &= ~SIO_DTRFLOW;

		/*
		 * Is the RTS line not wired to DTR?  If
		 *  this is true, then de-activate the
		 *  automatic input flow control.
		 */
		sio16_iflow_setup(siop);
		break;

	    case SIO_SETBAUD:
	    {
		int		 g;

		g = *((int *) mp->b_cont->b_rptr);
		XT(IOCTL_L2_SETBAUD, siop, g, 0, 0);

		/*
		 * valid value is under 115.2Kb
		 */
		if (g > 115200) {
		    error = EINVAL;
		    break;
		}

		siop->cust_baud = g;
		/*
		 * determine the correct clock speed macro from the
		 * baud rate table pointer.  it was set in the attach
		 * routine based on the clock value.  (yuch).
		 */
		if (siop->board_ptr->clk == 110500) {
		    siop->cust_bpr = SE2_BSPEED(g);
		}
		else {
		    siop->cust_bpr = SE_BSPEED(g);
		}

		*((int *) outblk->b_rptr) = g;
		break;
	    }

	    case SIO_GETBAUD:
		XT(IOCTL_L2_GETBAUD, siop, siop->cust_baud, 0, 0);
		/*
		 * write value
		 */
		*((int *) outblk->b_rptr) = siop->cust_baud;
		break;

	    case TCSBRK:
		XT(IOCTL_L2_TCSBRK, siop, *((int *) mp->b_cont->b_rptr), 
		    0, 0);
		if (*((int *) mp->b_cont->b_rptr) == 0) {
		    siop->flags |= SIO_BREAK;
		    nchars = sio16_sendbreak(siop);
		    if (siop->timeoutpending == 0) {
			untimeout(siop->timeoutid);
			siop->timeoutid = timeout(sio16_restart,
			    (caddr_t) siop, hz / 4);
			siop->timeoutpending = 1;
		    }
		}
		break;

	    case TIOCSBRK:
		XT(IOCTL_L2_TIOCSBRK, siop, 0, 0, 0);
		siop->flags |= SIO_BREAK;
		nchars = sio16_sendbreak(siop);
		break;

	    case TIOCCBRK:
		XT(IOCTL_L2_TIOCCBRK, siop, 0, 0, 0);
		siop->flags &= ~SIO_BREAK;
		siop->flags |= SIO_ENDBREAK;
		/*
		 * turn on interrupts so the system can
		 *  the proper TX context
		 *  and start running again
		 */
		/* cdp->srer = (IER_MDRX | IER_TXRDY); */
		siop->srer |= IER_TXRDY;
		cdp->srer = siop->srer;
		break;

	    case TIOCSDTR:
		XT(IOCTL_L2_TIOCSDTR, siop, 0, 0, 0);
		/*
		 * Do we have a mapping for DTR?
		 */
		if (siop->dtr_mask == MSVR_DTR) {
		    cdp->msvdtr = MSVR_DTR;
		}
		else if (siop->dtr_mask == MSVR_RTS) {
		    cdp->msvrts = MSVR_RTS;
		}
		break;

	    case TIOCCDTR:
		XT(IOCTL_L2_TIOCCDTR, siop, 0, 0, 0);
		/*
		 * Do we have a mapping for DTR?
		 */
		if (siop->dtr_mask == MSVR_DTR) {
		    cdp->msvdtr = 0;
		}
		else if (siop->dtr_mask == MSVR_RTS) {
		    cdp->msvrts = 0;
		}

		break;

	    case TIOCMGET:
		msvr = cdp->msvr;
		mflags = 0;

		if ((msvr & siop->cts_mask) != 0) {
		    mflags |= TIOCM_CTS;
		}

		if ((msvr & siop->dsr_mask) != 0) {
		    mflags |= TIOCM_DSR;
		}

		if ((msvr & siop->cd_mask) != 0) {
		    mflags |= TIOCM_CD;
		}

		if ((msvr & siop->dtr_mask) != 0) {
		    mflags |= TIOCM_DTR;
		}

		if ((msvr & siop->rts_mask) != 0) {
		    mflags |= TIOCM_RTS;
		}
		*((u_int *) outblk->b_rptr) = mflags;

		XT(IOCTL_L2_TIOCMGET, siop, mflags, (int) msvr, 0);
		break;

	    case TIOCMSET:
	    case TIOCMBIS:
		mflags = *((int *) mp->b_cont->b_rptr);
		if (cmd == TIOCMSET) {
		    msvr = 0;
		}
		else { 		/* cmd == TIOCMBIS */
		    msvr = cdp->msvr;
		}

		if ((mflags & TIOCM_DTR) != 0) {
		    msvr |= siop->dtr_mask;
		}
		if ((mflags & TIOCM_RTS) != 0) {
		    msvr |= siop->rts_mask;
		}
		cdp->msvr = msvr;

		if (cmd == TIOCMSET) {
		    XT(IOCTL_L2_TIOCMSET, siop, mflags, (int) msvr, 0);
		}
		else { 		/* cmd == TIOCMBIS */
		    XT(IOCTL_L2_TIOCMBIS, siop, mflags, (int) msvr, 0);
		}
		break;

	    case TIOCMBIC:
		mflags = *((int *) mp->b_cont->b_rptr);
		msvr = cdp->msvr;

		if ((mflags & TIOCM_DTR) != 0) {
		    msvr &= ~(siop->dtr_mask);
		}
		if ((mflags & TIOCM_RTS) != 0) {
		    msvr &= ~(siop->rts_mask);
		}
		cdp->msvr = msvr;
		XT(IOCTL_L2_TIOCMBIC, siop, mflags, (int) msvr, 0);
		break;

	    /*
	     * Read Statistics for a line, and optionally reset.
	     */
	    case SIO_STATS:
	    case SIO_STATS_RST:
		XT(IOCTL_L2_SIO_STATS, siop, 0, 0, 0);

		((sio_stats_report_t *) (outblk->b_rptr))->sio_statistics =
			*((sio_stats_t *) siop->l_sio_stats);
		((sio_stats_report_t *) (outblk->b_rptr))->sio_stats_overflow =
			*((sio_stats_t *) siop->l_sio_stats_valid);

		if (cmd == SIO_STATS_RST) {
			bzero(siop->l_sio_stats, sizeof(sio_stats_t));
			bzero(siop->l_sio_stats_valid, sizeof(sio_stats_t));
			sio16_stats_chk[siop->board] &=
						~((int) (1 << siop->port));
		}
		break;

	    /*
	     * Quick check if Errors occured on a line
	     */
	    case SIO_STATS_CHK:
		XT(IOCTL_L2_SIO_STATS_CHK, siop, 0, 0, 0);

		bcopy((caddr_t) sio16_stats_chk, (caddr_t) outblk->b_rptr,
				sizeof(sio_stats_check_t));
		break;

	    /*
	     * Reset/Clear statistics for all channels
	     */
	    case SIO_STATS_RST_ALL:
	    {
		int			 board, port;
		struct sio16_board	*sbp;
		struct sioline     	*line;

		XT(IOCTL_L2_SIO_STATS_RST_ALL, siop, 0, 0, 0);

		/* clear quick check array */
		bzero((caddr_t) sio16_stats_chk, sizeof(sio_stats_check_t));

		/* clear all the statistics for each line */
		for (board = 0; board <= sio16_maxinstance; board++) {
		    if ((sbp = (struct sio16_board *)
			ddi_get_soft_state(sio16_head, board)) == NULL) {

			continue;
		    }

		    if (sbp->siolines == NULL) {
			continue;
		    }

		    for (port = 0; port < sbp->nport; port++) {
			line = &(sbp->siolines[port]);
			if (line->l_sio_stats != NULL) {
			    bzero(line->l_sio_stats, sizeof(sio_stats_t));
			}
			if (line->l_sio_stats_valid != NULL) {
			    bzero(line->l_sio_stats_valid,
						sizeof(sio_stats_t));
			}
		    }
		}
		break;
	    }

	    default:
		XT(IOCTL_L2_UNKNOWN, siop, cmd, 0, 0);
		error = ENOTTY;
		break;

	    }		/* end of switch (cmd) */
	}		/* end of if (error == 0) */

	if (error != 0) {
	    XT(IOCTL_L2_IOCNAK, siop, error, 0, 0);

	    if (outalloc) {
		freemsg(outblk);
	    }

	    iocp->ioc_error = error;
	    mp->b_wptr = mp->b_rptr + sizeof(struct iocblk);
	    mp->b_datap->db_type = M_IOCNAK;
	}
	else {
	    /*
	     * Success!
	     */

	    /*
	     * Do we want to replace b_cont with outblk?
	     */
	    if (outalloc) {
		if (mp->b_cont != NULL) {
		    freemsg(mp->b_cont);
		}
		mp->b_cont = outblk;
	    }
	    else {
		outblk = mp->b_cont;
	    }

	    /*
	     * Possibly update the b_wptr.
	     */

	    if (outblk != NULL) {
		outblk->b_wptr = outblk->b_rptr + outsize;
	    }

	    /*
	     * Do we need to format this as an M_COPYOUT?
	     */
	    if (copyout) {		/* copyout -> outsize > 0 */
		cr = (struct copyreq *) mp->b_rptr;

		cr->cq_flag = 0;
		cr->cq_private = (mblk_t *) 0;
		cr->cq_size = outsize;
		cr->cq_addr = uaddr;

		mp->b_wptr = mp->b_rptr + sizeof(struct copyreq);
		mp->b_datap->db_type = M_COPYOUT;
	    }
	    else {
		iocp->ioc_error = 0;
		iocp->ioc_count = outsize;

		mp->b_wptr = mp->b_rptr + sizeof(struct iocblk);
		mp->b_datap->db_type = M_IOCACK;
	    }
	}
    }

    /*
     * bkd 8/15/95 - the following code is to prevent
     *  a lock reentrancy problem (qreply from an interrupt
     *  service routine causing wput to be called)
     */

    if (!intr) {
	/* non-interrupt context may qreply directly */
	qreply(q, mp);
    }
    else {
	/* interrupt context must wait */
	putq(RD(q), mp);

	qenable(RD(q));
    }

    XT(IOCTL_L1_EXIT, siop, 0, 0, 0);
    return nchars;
}

static void
sio16_reioctl(long arg)
{
    queue_t        *q;
    mblk_t         *mp;
    struct sioline	*siop;

    siop = (struct sioline *) arg;

    XT(REIOCTL_L1_ENTRY, siop, (unsigned int) siop, 0, 0);
    if ((q = siop->ttycommon.t_writeq) == NULL) {
	return;
    }

    if ((mp = siop->ttycommon.t_iocpending) != NULL) {
	siop->ttycommon.t_iocpending = NULL;
	CRITICAL(siop);
	sio16_ioctl(siop, q, mp, FALSE);
	UNCRITICAL(siop);
    }

    XT(REIOCTL_L1_EXIT, siop, 0, 0, 0);
}

static void
sio16_param(struct sioline *siop)
{
    register volatile struct cd180	*cdp;
    register u_char			 cor, cor2, cor3;
    unsigned long			 cflag, iflag;
    int					 baudrate;
    struct sio16_baud *baud_tbl;
    u_short bpr;

    ASSERT(LOCKED(siop));

    XT(PARAM_L1_ENTRY, siop, (unsigned int) siop, 0, 0);
    cdp = siop->cd180;

    cflag = siop->ttycommon.t_cflag;
    iflag = siop->ttycommon.t_iflag;
    baud_tbl = siop->baud_tbl;

    baudrate = GETBAUD(cflag);

    if (baudrate == B0) {
	/* Drop dtr/rts */
	sio16_setdtrrts(siop, SIO_OFF);
	XT(PARAM_L1_EXIT, siop, __LINE__, 0, 0);
	return;
    }
    else {
	if (baudrate == B38400) {
	    cdp->tbprh = siop->cust_bpr >> 8;
	    cdp->tbprl = siop->cust_bpr;
	    cdp->rbprh = siop->cust_bpr >> 8;
	    cdp->rbprl = siop->cust_bpr;
	}
	else {
	    /*
	     * Check to see if the baudrate is alright
	     */
	    if (baudrate > MAXBAUD
		|| baud_tbl[baudrate].b_bpr == 0) {

		/*
		 * bkd 9/18/1996 - default to the
		 *  *previous* usable baud rate.
		 */

		baudrate = GETBAUD(siop->cflag);
	    }

	    cdp->tbprh = baud_tbl[baudrate].b_bpr >> 8;
	    cdp->tbprl = baud_tbl[baudrate].b_bpr;
	    cdp->rbprh = baud_tbl[baudrate].b_bpr >> 8;
	    cdp->rbprl = baud_tbl[baudrate].b_bpr;
	}

	cor3 = (COR3_RX_TH_MASK & baud_tbl[baudrate].b_rxth);
	cdp->rtpr = baud_tbl[baudrate].b_rtpr;

    }
    XT(PARAM_L2_BAUDRATE, siop, baudrate, 0, 0);

    XT(PARAM_L1_FLAGS, siop, cflag, iflag, 0);
    switch (cflag & CSIZE) {

    case CS7:
	cor = COR1_CS7;
	break;

    case CS8:
	cor = COR1_CS8;
	break;

    case CS5:
	cor = COR1_CS5;
	break;

    case CS6:
	cor = COR1_CS6;
	break;

    default:
	cor = COR1_CS8;
	break;
    }

    if ((cflag & PARENB) != 0) {
	cor |= COR1_PAREN;
	if ((cflag & PARODD) != 0) {
	    cor |= COR1_PARODD;
	}
    }
    else {
	cor |= COR1_IGN_PARITY;
    }

    if ((cflag & CSTOPB) != 0) {
	cor |= COR1_STOP2;	/* 2 stop bits */
    }
    else {
	cor |= COR1_STOP1;	/* 1 stop bit */
    }

    /*
     * define start stop characters
     */
    if ((iflag & (IXON | IXOFF)) != 0) {
	cdp->schr1 = siop->ttycommon.t_startc;
	cdp->schr2 = siop->ttycommon.t_stopc;
	cdp->schr3 = siop->ttycommon.t_startc;
	cdp->schr4 = siop->ttycommon.t_stopc;
    }

    /* On-chip inband flow control */
    cor2 = 0x0;
    if ((iflag & IXON) != 0) {
	if ((iflag & IXANY) != 0) {
	    cor2 |= (COR2_IXM | COR2_TXIBE);
	}
	else {
	    cor2 |= COR2_TXIBE;
	}
	cor3 |= (COR3_FCT | COR3_SCDE);
    }

    /* On-chip hardware ouptput flow control */
    if ((cflag & CRTSCTS) != 0) {
	cor2 |= COR2_CTSAE;
    }

    XT(PARAM_L2_COR1, siop, cor, 0, 0);
    cdp->cor1 = cor;
    /*
     * issue a change core command for cor1, we will check before
     * leaving if it is cleared
     */
    cdp->ccr = CCR_CHG_COR1;
    DELAYCK(cdp->ccr, UNIT(siop->dev), "ccr change cor1");

    XT(PARAM_L2_COR2, siop, cor2, 0, 0);
    cdp->cor2 = cor2;

    XT(PARAM_L2_COR3, siop, cor3, 0, 0);
    cdp->cor3 = cor3;

    /*
     * Put the baud rate back into the true cflags.  This
     *  is just in case the user specifies an illegal
     *  baud rate (e.g. 153.6kbps on a 11.0592 MHz board), stty
     *  will report the correct value.
     */
#ifdef CBAUDEXT
    if (baudrate > CBAUD) {
	cflag |= (CBAUDEXT | CIBAUDEXT);
	cflag = (cflag & ~(CBAUD | CIBAUD))
			| ((baudrate - (CBAUD + 1)) & CBAUD)
			| (((baudrate - (CBAUD + 1)) << IBSHIFT) & CIBAUD);
    }
    else {
	cflag &= ~(CBAUDEXT | CIBAUDEXT);
	cflag = (cflag & ~(CBAUD | CIBAUD))
			| (baudrate & CBAUD)
			| ((baudrate << IBSHIFT) & CIBAUD);
    }
#else		/* CBAUDEXT */
    cflag = (cflag & ~(CBAUD | CIBAUD))
		| (baudrate & CBAUD)
		| ((baudrate << IBSHIFT) & CIBAUD);
#endif		/* !CBAUDEXT */

    siop->cflag = siop->ttycommon.t_cflag = cflag;
    siop->ttycommon.t_iflag = iflag;

    /* set up the on-chip hardware input flow control */
    sio16_iflow_setup(siop);

    XT(PARAM_L1_EXIT, siop, 0, 0, 0);
}

static void
sio16_setdtrrts(struct sioline *siop, u_int value)
{
    unsigned long now, held;
    unsigned long start;

    ASSERT(LOCKED(siop));

    GETTIME(&now);
    XT(SETDTRRTS_L1_ENTRY, siop, (unsigned int) siop, value, now);
again:
    /*
     * check time
     */
    GETTIME(&now);		/* get time */

    /*
     * if DTR going high then wait
     */
    if (value == SIO_ON) {
	held = (int) HZTOSEC(now - siop->dtrlow);
	if (held < hold_dtr_low) {
	    /*
	     * if DTR going high and we haven't waited 
	     * long enough then wait
	     */
	    (void) cv_wait_sig(&lbolt_cv, siop->chiplock);
	    goto again;
	}
    }
    else {
	siop->dtrlow = now;	/* save time */
    }

    /*
     * set bits
     */
    SETCHAN(siop);
    XT(SETDTRRTS_L2_SETBITS, siop, now, siop->dtrlow, 0);
    if (value == SIO_ON) {
	/* raise the HW handshanking lines */
	siop->cd180->msvr = (MSVR_RTS | MSVR_DTR);
    }
    else {
	/* drop the HW handshanking lines */
	siop->cd180->msvr = 0;
    }

out:
    XT(SETDTRRTS_L1_EXIT, siop, 0, 0, 0);
}

/*
 * sio16_start
 */
static int
sio16_start(struct sioline *siop)
{
    register volatile struct cd180	*cdp;
    register int	fifocnt, bpcnt;
    register int            ncharsent = 0;
    register mblk_t	*bp;
    mblk_t	*nbp;
    queue_t	*q;

#ifdef TIMING
    struct timeval start, end;
#endif TIMING

    XT(START_L3_ENTRY, siop, (unsigned int) siop, siop->flags, 0);

    ASSERT(LOCKED(siop));

#ifdef TIMING
    uniqtime(&start);
    XT(TIME_SLICE, siop, start.tv_sec, start.tv_usec, 0);
#endif

    cdp = siop->cd180;

    if ((q = siop->ttycommon.t_writeq) == NULL) {
	siop->srer &= ~IER_TXRDY;
	cdp->srer = siop->srer;
	XT(START_L3_EXIT, siop, __LINE__, 0, 0);
	return ncharsent;
    }

    if ((siop->flags & SIO_ENDBREAK) != 0) {
	ncharsent = sio16_endbreak(siop);
	siop->flags &= ~SIO_ENDBREAK;
	siop->srer |= IER_TXRDY;
	cdp->srer = siop->srer;
	XT(START_L3_EXIT, siop, __LINE__, 0, 0);
	return ncharsent;
    }

    /*
     * if the following flags are set then we want to keep the TX interrupts
     * going else we could possibly get into an exiting state
     */
    if ((siop->flags & (SIO_BUSY | SIO_DELAY | SIO_BREAK)) != 0) {
	XT(START_L3_EXIT, siop, __LINE__, 0, 0);
	return ncharsent;
    }

    /*
     * if the following flag is set then let's turn off TX interrupts
     * because the output has been stopped
     */
    if ((siop->flags & SIO_OSTOPPED) != 0) {
	siop->srer &= ~IER_TXRDY;
	cdp->srer = siop->srer;
	XT(START_L3_EXIT, siop, __LINE__, 0, 0);
	return ncharsent;
    }

    if ((bp = siop->lasttxmp) == NULL) {
	XT(START_L3_LASTTXMP, siop, 0, 0,0);
	bp = getq(q);
	if (bp == NULL) {
	    siop->srer &= ~IER_TXRDY;
	    cdp->srer = siop->srer;
	    XT(START_L3_EXIT, siop, __LINE__, 0, 0);
	    return ncharsent;
	}
	siop->lasttxmp = bp;
    }

    switch (bp->b_datap->db_type) {

    case M_DATA:
	XT(START_L3_M_DATA, siop, 0, 0, 0);
	if ((bpcnt = (bp->b_wptr - bp->b_rptr)) <= 0) {
	    freemsg(bp);
	    siop->lasttxmp = NULL;
	    break;
	}

	fifocnt = (bpcnt < TX_FIFO ? bpcnt : TX_FIFO);
	siop->flags |= SIO_BUSY;
	XT(START_L3_COUNT, siop, fifocnt, bpcnt, 0);

	while (fifocnt > 0) {
	    XT(START_L3_XMITCHAR, siop, *(bp->b_rptr), *(bp->b_rptr), 0);
	    cdp->tdr = *(bp->b_rptr);
	    bp->b_rptr++;
	    fifocnt--;
	    bpcnt--;
	    ncharsent++;
	}

	if (bpcnt == 0) {
	    nbp = bp;
	    bp = bp->b_cont;
	    freeb(nbp);
	    /*
	     * lasttxmp will either equal NULL or have
	     *  data, we don't care becase the next time
	     *  through we will handle the pointer
	     */
	    siop->lasttxmp = bp;
	}

	if (siop->lasttxmp == NULL) {
	    bp = getq(q);
	    if (bp == NULL) {
		XT(START_L3_EXIT, siop, __LINE__, 0, 0);
		return ncharsent;
	    }
	    siop->lasttxmp = bp;
	}

	break;

    case M_BREAK:
	    XT(START_L3_M_BREAK, siop, 0, 0, 0);
	    siop->flags |= SIO_BREAK;
	    ncharsent = sio16_sendbreak(siop);
	    if (siop->timeoutpending == 0) {
		/*
		 * !!!bkd 9/27/1996 - I don't think it's legal
		 *  to untimeout an invalid timeout id.  Check
		 *  this everywhere.
		 */
		untimeout(siop->timeoutid);
		siop->timeoutid = timeout(sio16_restart, (caddr_t) siop, hz / 4);
		siop->timeoutpending = 1;
	    }
	    freemsg(bp);
	    siop->lasttxmp = NULL;
	    break;

    case M_DELAY:
	    XT(START_L3_M_DELAY, siop, 0, 0, 0);
	    siop->flags |= SIO_DELAY;
	    if (siop->timeoutpending == 0) {
		untimeout(siop->timeoutid);
		siop->timeoutid = timeout(sio16_restart, (caddr_t) siop,
		    (int)(*(u_char *)bp->b_rptr + 6));
		siop->timeoutpending = 1;
	    }
	    freemsg(bp);
	    siop->lasttxmp = NULL;
	    break;

    case M_IOCTL:
    case M_IOCDATA:
	if (bp->b_datap->db_type == M_IOCTL) {
	    XT(START_L3_M_IOCTL, siop, 0, 0, 0);
	}
	else {
	    XT(START_L3_M_IOCDATA, siop, 0, 0, 0);
	}

	ncharsent = sio16_ioctl(siop, q, bp, TRUE);
	siop->lasttxmp = NULL;
	break;

    default:
	XT(START_L3_M_UNKNOWN, siop, bp->b_datap->db_type, 0, 0);
	break;

    }

#ifdef TIMING
    uniqtime(&end);
    XT(TIME_SLICE, siop, end.tv_sec, end.tv_usec, 0);
#endif

    XT(START_L3_EXIT, siop, 0, 0, 0);
    return ncharsent;
}

/*
 * sio16_restart
 */
static void
sio16_restart(caddr_t arg)
{
    struct sioline *siop;

    siop = (struct sioline *) arg;

    XT(RSTRT_L1_ENTRY, siop, (unsigned int) siop, 0, 0);

    CRITICAL(siop);
    SETCHAN(siop);
    siop->timeoutpending = 0;
    if ((siop->flags & SIO_BREAK) != 0) {
	siop->flags |= SIO_ENDBREAK;
    }
    siop->flags &= ~(SIO_DELAY | SIO_BREAK);

    /* siop->cd180->srer = (IER_MDRX | IER_TXRDY); */
    siop->srer |= IER_TXRDY;
    siop->cd180->srer = siop->srer;
    UNCRITICAL(siop);
    XT(RSTRT_L1_EXIT, siop, 0, 0, 0);
}

static void
sio16_rxintr(struct sioline *siop, int norm)
{
    register volatile struct cd180 *cdp;
    register tty_common_t *ttc;
    register u_char	c, rcnt;
    queue_t	*rq;

    XT(RXINTR_L3_ENTRY, siop, (unsigned int) siop, 0, 0);

    cdp = siop->cd180;
    ttc = &siop->ttycommon;

    if ((rq = ttc->t_readq) == NULL) {
	/*
	 * drain FIFO until there is no more 
	 */
	rcnt = cdp->rdcr & 0xf;
	while (rcnt > (u_char) 0) {
	    c = cdp->rdr;
	    rcnt--;
	}
	SIO_RING_INIT(siop);
	XT(RXINTR_L3_EXIT, siop, __LINE__, 0, 0);
	cdp->eosrr = 0x1;
	return;
    }

    if (norm == RX_NORMAL) {
	rcnt = cdp->rdcr & 0xf;
    }
    else {
	rcnt = 1;
    }

    while (rcnt-- > (u_char) 0) {
	c = cdp->rdr;
	XT(RXINTR_L3_RCVCHAR, siop, c, c, 0);

	/* Output control was enabled */
	if ((ttc->t_iflag & IXON) != 0) {
	    /* if it wants to stop transmittion */
	    if ((c & 0x7f) == ttc->t_stopc) {
		XT(RXINTR_L3_TXSTOP, siop, 0, 0, 0);
		/*
		 * turn off FCT so that flow-char. will get
		 * reported to the driver
		 */
		cdp->cor3 &= ~(COR3_FCT);
		siop->flags |= SIO_OSTOPPED;
		continue;
	    }
	    else if ((c & 0x7f) == ttc->t_startc) {
		XT(RXINTR_L3_TXSTART, siop, 0, 0, 0);
		/*
		 * turn on interrupts so the system can setup
		 * the proper TX context and call sio16_start
		 */
		cdp->cor3 &= ~(COR3_FCT);
		cdp->ccr = CCR_TXEN;
		siop->flags &= ~(SIO_OSTOPPED);
		siop->srer |= IER_TXRDY;
		cdp->srer = siop->srer;
		continue;
	    }
	}

	if (c == 0377
	    && (ttc->t_iflag & PARMRK) != 0
	    && (ttc->t_iflag & (IGNPAR | ISTRIP)) == 0) {

	    if (SIO_RING_POKE(siop, 2)) {
		SIO_RING_PUT(siop, 0377);
		SIO_RING_PUT(siop, c);
	    }
	    else {
		/*
		 * ring buffer overflow
		 */
		SIO_STAT_CHK_UPDATE(siop, error_receive_dropchars, 1);
		XT(RXINTR_L3_RING_OVR, siop, 0, 0, 0);
#if 0
		ERR("sio16 - ring buffer overflow: board %d port %d\n",
					BOARD(siop->dev), UNIT(siop->dev));
#endif
	    }
	}
	else {
	    if (SIO_RING_POKE(siop, 1)) {
		SIO_RING_PUT(siop, c);
	    }
	    else {
		/*
		 * ring buffer overflow
		 */
		SIO_STAT_CHK_UPDATE(siop, error_receive_dropchars, 1);
		XT(RXINTR_L3_RING_OVR, siop, 0, 0, 0);
#if 0
		ERR("sio16 - ring buffer overflow: board %d port %d\n",
					BOARD(siop->dev), UNIT(siop->dev));
#endif
	    }
	}
    }

    if (SIO_RING_FRAC(siop)) {
	/*
	 * bkd 10/8/1996 - do the qenable LAST, because
	 *  it could take a non-trivial amount of realtime.
	 */
	if (SIO_RING_HIWATER(siop)) {
	    XT(RXINTR_L3_THROTTLE, siop, 0, 0, 0);
	    if ((siop->flags & SIO_ISTOPPED) == 0) {
		sio16_istop(siop, TRUE);
	    }
	    siop->flags |= SIO_THROTTLE;
	}

	XT(RXINTR_L3_QENABLE, siop, 0, 0, 0);
	qenable(rq);
    }

    /*
     * signal the end of service context only if this is a rxintr
     * else we will pop the context inside of rsintr
     */
    if (norm == RX_NORMAL) {
	cdp->eosrr = 0x1;
    }

    XT(RXINTR_L3_EXIT, siop, 0, 0, 0);
}

/*
 * sio16_rsintr
 *
 * This is where we get receive status char. interrupts here.  We don't
 * check for Special Char. Detect (SCD)
 */
static void
sio16_rsintr(struct sioline *siop)
{
    register volatile struct cd180	*cdp;
    register tty_common_t		*ttc;
    register queue_t			*q;
    unsigned char			 c, rcsr;

    cdp = siop->cd180;
    rcsr = cdp->rcsr;
    ttc = &siop->ttycommon;

    XT(RSINTR_L3_ENTRY, siop, (unsigned int) siop, (unsigned int) rcsr, 0);

    if ((q = ttc->t_readq) == NULL) {
	XT(RSINTR_L3_EXIT, siop, __LINE__, 0, 0);
	cdp->eosrr = 0x1;
	return;
    }

    /* Overrun Error */
    if ((rcsr & RCSR_OE) != 0) {
	SIO_STAT_CHK_UPDATE(siop, error_receive_overrun, 1);
	XT(RSINTR_L3_OVRRUN, siop, __LINE__, 0, 0);
	sio16_rxintr(siop, RS_INTR);
#if 0
	ERR("sio16 - receive overrun!: board %d, port %d\n", BOARD(siop->dev),
							UNIT(siop->dev));
#endif
    }

    /*
     * char. timeout
     */
    if ((rcsr & RCSR_TOUT) != 0) {
	XT(RSINTR_L3_TIMEOUT, siop, (unsigned int) siop, 0, 0);
	qenable(q);
    }

    /*
     * framing error
     */
    if ((rcsr & RCSR_FE) != 0) {
	SIO_STAT_CHK_UPDATE(siop, error_receive_framing, 1);
	XT(RSINTR_L3_FRMERR, siop, 0, 0, 0);
    }

    /*
     * break detected
     */
    if ((rcsr & RCSR_BRK) != 0) {
	XT(RSINTR_L3_BRKDET, siop, 0, 0, 0);
	putctl(q, M_BREAK);
	qenable(q);
    }

    /*
     * parity error
     */
    if ((rcsr & RCSR_PE) != 0) {
	/* read the char */
	c = cdp->rdr;

	/* we have a parity error - check for mis-matched parity */
	SIO_STAT_CHK_UPDATE(siop, error_receive_parity, 1);
	XT(RSINTR_L3_PARERR, siop, c, c, 0);
	if ((ttc->t_iflag & IXON) != 0) {
	    /* check if remote wants to stop transmission */
	    if ((c & 0x7f) == ttc->t_stopc) {
		/*
		 * turn off FCT so that flow-char. will get
		 * reported to the driver
		 */
		cdp->cor3 &= ~(COR3_FCT);
		siop->flags |= SIO_OSTOPPED;
		cdp->eosrr = 0x1;
		XT(RSINTR_L3_EXIT, siop, __LINE__, 0, 0);
		return;
	    }
	    else if ((c & 0x7f) == ttc->t_startc) {
		/*
		 * turn off FCT so that flow-char. will get
		 * reported to the driver
		 */
		cdp->cor3 &= ~(COR3_FCT);
		cdp->ccr = CCR_TXEN;
		siop->flags &= ~(SIO_OSTOPPED);
		/*
		 * turn on interrupts so the system can setup
		 * the proper TX context and call sio16_start
		 */
		siop->srer |= IER_TXRDY;
		cdp->srer = siop->srer;
		cdp->eosrr = 0x1;
		XT(RSINTR_L3_EXIT, siop, __LINE__, 0, 0);
		return;
	    }
	}

	if ((ttc->t_iflag & INPCK) != 0) {
	    if ((ttc->t_iflag & IGNPAR) == 0) {
		if ((ttc->t_iflag & PARMRK) != 0) {
			if (SIO_RING_POKE(siop, 3)) {
			    SIO_RING_PUT(siop, 0377);
			    SIO_RING_PUT(siop, 0);
			    SIO_RING_PUT(siop, c);
			}
			else {
			    /*
			     * ring buffer overflow
			     */
			    SIO_STAT_CHK_UPDATE(siop,
					error_receive_dropchars, 1);
			    XT(RSINTR_L3_RING_OVR, siop, 0, 0, 0);
#if 0
			    ERR(
			"sio16 - ring buffer overflow: board %d port %d\n",
				    BOARD(siop->dev), UNIT(siop->dev));
#endif
			}
		    }
		    else {
			if (SIO_RING_POKE(siop, 1)) {
			    SIO_RING_PUT(siop, '\0');
			}
			else {
			    /*
			     * ring buffer overflow
			     */
			    SIO_STAT_CHK_UPDATE(siop,
					error_receive_dropchars, 1);
			    XT(RSINTR_L3_RING_OVR, siop, 0, 0, 0);
#if 0
			    ERR(
			"sio16 - ring buffer overflow: board %d port %d\n",
				    BOARD(siop->dev), UNIT(siop->dev));
#endif
		    }
		}
	    }
	}
	else {
	    if (c == 0377
		&& (ttc->t_iflag & PARMRK) != 0
		&& (ttc->t_iflag & (IGNPAR | ISTRIP)) == 0) {

		if (SIO_RING_POKE(siop, 2)) {
		    SIO_RING_PUT(siop, 0377);
		    SIO_RING_PUT(siop, c);
		}
		else {
		    /*
		     * ring buffer overflow
		     */
		    SIO_STAT_CHK_UPDATE(siop, error_receive_dropchars, 1);
		    XT(RSINTR_L3_RING_OVR, siop, 0, 0, 0);
#if 0
		    ERR("sio16 - ring buffer overflow: board %d port %d\n",
				    BOARD(siop->dev), UNIT(siop->dev));
#endif
		}
	    }
	    else {
		if (SIO_RING_POKE(siop, 1)) {
		    SIO_RING_PUT(siop, c);
		}
		else {
		    /*
		     * ring buffer overflow
		     */
		    SIO_STAT_CHK_UPDATE(siop, error_receive_dropchars, 1);
		    XT(RSINTR_L3_RING_OVR, siop, 0, 0, 0);
#if 0
		    ERR("sio16 - ring buffer overflow: board %d port %d\n",
				    BOARD(siop->dev), UNIT(siop->dev));
#endif
		}
	    }
	}

	/*
	 * bkd 10/8/1996 - do the qenable LAST, because
	 *  it could take a non-trivial amount of realtime.
	 */
	if (SIO_RING_HIWATER(siop)) {
	    XT(RSINTR_L3_THROTTLE, siop, 0, 0, 0);
	    if ((siop->flags & SIO_ISTOPPED) == 0) {
		sio16_istop(siop, TRUE);
	    }
	    siop->flags |= SIO_THROTTLE;
	}

	XT(RSINTR_L3_QENABLE, siop, 0, 0, 0);
	qenable(q);
    }

    /* If Special characters detected */
    if ((rcsr & RCSR_SC_DETMASK) != 0) {
	/* read the char */
	c = cdp->rdr;

	if ((ttc->t_iflag & IXON) != 0) {
	    XT(RSINTR_L3_SPECIAL, siop, c, rcsr, 0);

	    switch (rcsr & RCSR_SC_DETMASK) {

	    case RCSR_SCD1:
	    case RCSR_SCD3:
		if ((c & 0x7f) == ttc->t_startc) {
		    /*
		     * check if remote wants to start transmission
		     * enable flow control transparently
		     */
		    if ((siop->flags && SIO_OSTOPPED) != 0) {
			/*
			 * turn off FCT so that flow-char. will get
			 * reported to the driver	
			 */
			cdp->cor3 |= COR3_FCT;
			siop->flags &= ~SIO_OSTOPPED;

			/*
			 * turn on interrupts so the system can setup
			 * the proper TX context and call sio16_start
			 */
			siop->srer |= IER_TXRDY;
			cdp->srer = siop->srer;
		    }
		}
		break;

	    case RCSR_SCD2:
	    case RCSR_SCD4:
		/* check if remote wants to stop transmission */
		if ((c & 0x7f) == ttc->t_stopc) {
		    /*
		     * turn off FCT so that flow-char. will get
		     * reported to the driver
		     */
		    cdp->cor3 &= ~(COR3_FCT);

		    siop->flags |= SIO_OSTOPPED;
		}
		break;

	    default:
		break;

	    }
	}
    }

    cdp->eosrr = 0x1;

    XT(RSINTR_L3_EXIT, siop, 0, 0, 0);
}

static void
sio16_txintr(struct sioline *siop)
{
    register int    ncharsent = 0;

    XT(TXINTR_L3_ENTRY, siop, (unsigned int) siop, 0, 0);

    siop->flags &= ~SIO_BUSY;
    ncharsent = sio16_start(siop);
    if (ncharsent > 0) {
	SIO_STAT_UPDATE(siop, stats_transmit_chars, ncharsent);
    }
    siop->cd180->eosrr = 0x1;

    XT(TXINTR_L3_EXIT, siop, 0, 0, 0);
}

static void
sio16_mdintr(struct sioline *siop)
{
    register volatile struct cd180	*cdp;
    register queue_t			*q;
    u_char				 mcr, msvr;

    /*
     * read the msvr register & mcr then reset the mcr register
     */
    cdp = siop->cd180;
    msvr = cdp->msvr;
    mcr = cdp->mcr;

    XT(MINTR_L3_ENTRY, siop, (unsigned int) siop,
			(unsigned int) msvr, (unsigned int) mcr);

    /*
     * if ttysoftcar is setup AFTER open of port
     * then disable CD interrupts.
     */
    if ((siop->ttycommon.t_flags & TS_SOFTCAR) != 0) {
	if (siop->cd_mask == MSVR_DSR) {
	    /* CD pin is seen as DSR pin on CD180 */
	    cdp->mcor1 &= ~MCOR_DSR;
	    cdp->mcor2 &= ~MCOR_DSR;
	}
	else if (siop->cd_mask == MSVR_CTS) {
	    /* last 4 ports share CD/CTS with CTS pin */
	    cdp->mcor1 &= ~MCOR_CTS;
	    cdp->mcor2 &= ~MCOR_CTS;
	}
    }

    /*
     * Did we see a transition in the CD line?
     */
    if (((mcr & MCR_DDSR) != 0 && siop->cd_mask == MSVR_DSR)
	|| ((mcr & MCR_DCTS) != 0 && siop->cd_mask == MSVR_CTS)) {

	if ((msvr & siop->cd_mask) != 0
	    || (siop->ttycommon.t_flags & TS_SOFTCAR) != 0) {

	    XT(MINTR_L3_CD, siop, 0, 0, 0);
	    if ((siop->flags & SIO_CARR_ON) == 0) {
		siop->flags |= SIO_CARR_ON;
		if ((q = siop->ttycommon.t_readq) != NULL) {
		    putctl(q, M_UNHANGUP);
		    qenable(q);
		}

		XT(MINTR_L3_CD_WAKEUP, siop, 0, 0, 0);
		cv_signal(&(siop->cv));
	    }
	}
	else {
	    /* no dcd and not an outcall line */
	    XT(MINTR_L3_NOCD, siop, 0, 0, 0);
	    if ((siop->flags & SIO_CARR_ON) != 0
		&& (siop->ttycommon.t_cflag & CLOCAL) == 0) {

		if (siop->lasttxmp != NULL) {
		    freemsg(siop->lasttxmp);
		    siop->lasttxmp = NULL;
		}

		siop->flags &= ~(SIO_OSTOPPED | SIO_BUSY);

		if ((q = siop->ttycommon.t_readq) != NULL) {
		    /*
		     * enable the TX & RX - just
		     * in case we had a ^S before CD went low
		     * enable receive interrupts in case
		     * they were disabled from hw flow
		     */
		    cdp->ccr = CCR_TXEN | CCR_RXEN;
		    siop->srer |= IER_RXDATA;
		    cdp->srer = siop->srer;

		    XT(MINTR_L3_M_HANGUP, siop, 0, 0, 0);
		    putctl(q, M_HANGUP);
		    qenable(q);
		}
	    }
	    siop->flags &= ~SIO_CARR_ON;
	}
    }

    cdp->mcr = 0x0;
    cdp->eosrr = 0x1;

    XT(MINTR_L3_EXIT, siop, 0, 0, 0);
}

static int
sio16_sendbreak(struct sioline *siop)
{
    register volatile struct cd180   *cdp;

    XT(SENDBREAK_L1_EXIT, siop, __LINE__, 0, 0);

    ASSERT(LOCKED(siop));

    cdp = siop->cd180;

    cdp->cor2 |= COR2_ETC;

    /* send break */
    cdp->tdr = 0x0;
    cdp->tdr = 0x81;

    XT(SENDBREAK_L1_EXIT, siop, 0, 0, 0);
    return 2;
}

static int
sio16_endbreak(struct sioline *siop)
{
    register volatile struct cd180   *cdp;

    XT(ENDBREAK_L1_ENTRY, siop, __LINE__, 0, 0);

    ASSERT(LOCKED(siop));

    cdp = siop->cd180;

    /* end break */
    cdp->tdr = 0x0;
    cdp->tdr = 0x83;

    cdp->cor2 &= ~COR2_ETC;

    XT(ENDBREAK_L1_EXIT, siop, 0, 0, 0);
    return 2;
}

static void
sio16_istart(struct sioline *siop)
{
    register volatile struct cd180   *cdp;

    XT(ISTART_L1_ENTRY, siop, 0, 0, 0);

    ASSERT(LOCKED(siop));

    cdp = siop->cd180;

    if ((siop->flags & SIO_ISTOPPED) != 0) {
	SETCHAN(siop);

	if ((siop->ttycommon.t_iflag & IXOFF) != 0) {
	    /* send flow ctl character */
	    /*
	     * This method of sending high priority flow ctl chars is
	     * undesireable since it may tie up the ccr for up to 2
	     * character periods.
	     */
	    DELAYCK(siop->cd180->ccr, UNIT(siop->dev), "send special char.");
	    siop->cd180->ccr = CCR_SSPC1;
	}

	if ((siop->ttycommon.t_cflag & CRTSCTS) != 0) {	/* raise "RTS" */
	    /*
	     * Turn on reception (far side transmission) using
	     *  the out-of-band flow control technique.  This
	     *  could mean either trying to raise the physical DTR
	     *  or RTS signals.
	     */
	    if (siop->iflow_mask == MSVR_DTR) {
		/* we want to raise the physical DTR */
		siop->srer |= IER_RXDATA;
		cdp->srer = siop->srer;
	    }
	    else if (siop->iflow_mask == MSVR_RTS) {
		/* we want to raise the physical RTS */
		cdp->msvrts = MSVR_RTS;
	    }
	}

	siop->flags &= ~SIO_ISTOPPED;

    }
    XT(ISTART_L1_EXIT, siop, 0, 0, 0);
}

static void
sio16_istop(struct sioline *siop, boolean intr)
{
    register volatile struct cd180   *cdp;

    XT(ISTOP_L1_ENTRY, siop, (unsigned int) siop, 0, 0);

    ASSERT(LOCKED(siop));

    cdp = siop->cd180;

    if ((siop->flags & SIO_ISTOPPED) == 0) {
	if (!intr) {
	    SETCHAN(siop);
	}

	if ((siop->ttycommon.t_iflag & IXOFF) != 0) {
	    /* send flow ctl character */
	    /*
	     * This method of sending high priority flow ctl chars is
	     * undesireable since it may tie up the ccr for up to 2
	     * character periods.
	     */
	    DELAYCK(siop->cd180->ccr, UNIT(siop->dev), "send special char.");
	    siop->cd180->ccr = CCR_SSPC2;
	}

	if ((siop->ttycommon.t_cflag & CRTSCTS) != 0) {	/* raise "RTS" */
	    /*
	     * Turn off reception (far side transmission) using
	     *  the out-of-band flow control technique.  This
	     *  could mean either trying to lower the physical DTR
	     *  or RTS signals.
	     */
	    if (siop->iflow_mask == MSVR_DTR) {
		/* we want to lower the physical DTR */
		siop->srer &= ~IER_RXDATA;
		cdp->srer = siop->srer;
	    }
	    else if (siop->iflow_mask == MSVR_RTS) {
		/* we want to lower the physical RTS */
		cdp->msvrts = 0;
	    }
	}

	siop->flags |= SIO_ISTOPPED;

    }
    XT(ISTOP_L1_EXIT, siop, 0, 0, 0);
}

/*
 * put an IOCTL command...maybe to queue, maybe service
 *  right now.
 */
static void
sio16_iocput(struct sioline *siop, queue_t *q, mblk_t *mp, int cmd)
{
    /*
     * This is a regular ioctl...maybe queue it.
     */

    XT(IOCPUT_L1_ENTRY, siop, 0, 0, 0);

    switch (cmd) {

    case TCSETS:
    case TCSETA:
    case TCSETSW:
    case TCSETSF:
    case TCSETAW:
    case TCSETAF:
    case TCSBRK:
    case TIOCSBRK:
    case TIOCCBRK:
	/*
	 * The changes do not take effect until all output has
	 *  queued up has drained. Put this message on the queue
	 *  so that sio_start will it when done with the output
	 *  before it
	 */
	XT(IOCPUT_L2_ENQUEUE, siop, (int) cmd, 0, 0);
	putq(q, mp);

	CRITICAL(siop);
	SETCHAN(siop);
	/* siop->cd180->srer = (IER_MDRX | IER_TXRDY); */
	siop->srer |= IER_TXRDY;
	siop->cd180->srer = siop->srer;
	UNCRITICAL(siop);
	break;

    default:
	/*
	 * Do it now!
	 */
	XT(IOCPUT_L2_RUN, siop, (int) cmd, 0, 0);

	CRITICAL(siop);
	sio16_ioctl(siop, q, mp, FALSE);
	UNCRITICAL(siop);
	break;

    }

    XT(IOCPUT_L1_EXIT, siop, 0, 0, 0);
}

/*
 * Return the size of the 'in' datum required
 *  by the given ioctl.
 *
 * Returns 0 if no data are needed.
 */
static unsigned int
sio16_iocin(int ioc_cmd)
{
    switch (ioc_cmd) {

    /* Aurora specific: */
    case SIO_SETTIMEOUT:
    case SIO_SETBAUD:
	return sizeof(int);

    /* termio generic: */

#if (defined(SUNOS5) && defined(_SYS_TERMIO_H))		\
	|| (defined(SUNOS4) && defined(TCSETA))
    /*
     * struct termio is defined in sys/termio.h under both
     *  SunOS 4 and Solaris 2.  The indicator that this file
     *  has been included is different under the two, though.
     */
    case TCSETA:
    case TCSETAW:
    case TCSETAF:
	return sizeof(struct termio);
#endif

    case TIOCSPGRP:
	return sizeof(pid_t);

    case TIOCSWINSZ:
	return sizeof(struct winsize);

    case TCSETS:
    case TCSETSW:
    case TCSETSF:
	return sizeof(struct termios);

    case TIOCSSOFTCAR:
    case TIOCMBIS:
    case TIOCMBIC:
    case TIOCMSET:
	return sizeof(int);

    default:
	return 0;

    }

    /* NOTREACHED */
}

/*
 * Determine the size of the output
 *  datum needed for the given ioctl.
 *
 * Returns 0 if none is defined.
 */
static unsigned int
sio16_iocout(int cmd)
{
    switch (cmd) {

    case SIO_GETMFLG:
    case SIO_GETTIMEOUT:
    case SIO_SETBAUD:
    case SIO_GETBAUD:
    case TIOCMGET:
	return sizeof(int);

    case SIO_STATS:
    case SIO_STATS_RST:
	return sizeof(sio_stats_report_t);

    case SIO_STATS_CHK:
	return sizeof(sio_stats_check_t);

    default:
	return 0;

    }

    /* NOTREACHED */
}


static void
chip_dump(struct sioline *siop)
{
    register volatile struct cd180 *cdp;

    cdp = siop->cd180;

    XT(CHIP_L4_DUMP1, siop, cdp->ccr, cdp->ccsr, cdp->srer);
    XT(CHIP_L4_DUMP2, siop, cdp->cor1, cdp->cor2, cdp->cor3);
    XT(CHIP_L4_DUMP3, siop, cdp->gsvr, cdp->srsr, cdp->gscr);
    XT(CHIP_L4_DUMP4, siop, cdp->srcr, cdp->mcr, cdp->msvr);
    XT(CHIP_L4_DUMP6, siop, cdp->schr1, cdp->schr2, cdp->srsr);
    XT(CHIP_L4_DUMP7, siop, cdp->mcor1, cdp->mcor2, cdp->car);
}



/*
 * Re-run the rsrv command, which was stopped
 *  because we ran out of space.  Right now this
 *  is only for interrupt mode.
 */

static void
sio16_rersrv(long arg)
{
    register queue_t			*q;
    register struct sioline		*siop;

    if ((q = (queue_t *) arg) == NULL) {
	return;
    }

    if ((siop = (struct sioline *) q->q_ptr) == NULL) {
	return;
    }

    XT(RERSRV_L1_ENTRY, siop, 0, 0, 0);
    qenable(q);
    XT(RERSRV_L1_EXIT, siop, 0, 0, 0);
}

/*
 * sio16_iflow_setup - Setup the input flow control.
 *  This appears in a subroutine because it happens
 *  so often.
 *
 * This routine requires that the caller has acquired
 *  the chip lock and set the chip channel.
 *
 * Also requires that ttycommon.t_cflag and baud_tbl
 *  be set up.
 */
 
static void
sio16_iflow_setup(struct sioline *siop)
{
    int					 baud_idx, cflag;
    register volatile struct cd180	*cdp;
    register unsigned char		 mcor1;

    siop->iflow_mask = 0;
    cflag = siop->ttycommon.t_cflag;

    if ((cflag & CRTSCTS) != 0) {
	cdp = siop->cd180;
	if ((siop->modem_flags & SIO_DTRFLOW) != 0) {
	    siop->iflow_mask = siop->dtr_mask;
	}
	else {
	    siop->iflow_mask = siop->rts_mask;
	}

	mcor1 = cdp->mcor1;
	if (siop->iflow_mask == MSVR_DTR) {
	    /* use chip's automatic DTR control */
	    baud_idx = GETBAUD(cflag);
	    mcor1 &= ~MCOR_THRESH_MASK;
	    mcor1 |= siop->baud_tbl[baud_idx].b_rxth + 2;
	}
	else {
	    mcor1 &= ~MCOR_THRESH_MASK;
	}
	cdp->mcor1 = mcor1;
    }
}
