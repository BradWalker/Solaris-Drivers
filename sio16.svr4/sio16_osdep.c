/*
 * Aurora Cirrus CL-CD180/1865 Async Driver (sio16)
 *
 * This module is the Solaris 2 dependent section of the
 *  driver.  It contains all of the loadable wrapper
 *  and attach/detach code.
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
 *	revision info: $Id: sio16_osdep.c,v 1.16.2.4 1996/10/22 15:17:25 bkd Exp $
 */

/*
 * $Log: sio16_osdep.c,v $
 * Revision 1.16.2.4  1996/10/22 15:17:25  bkd
 * Fixed bug in _fini that prevented driver from
 * being unloaded (oops)
 *
 * Revision 1.16.2.3  1996/10/22 13:28:25  bkd
 * Added new sio16_detach (to fix panic problem when
 * restoring under Solaris 2.5).
 *
 * Revision 1.16.2.2  1996/10/17 03:42:41  bkd
 * + cleaned up the messages when the "model" property
 *   is not acceptable.
 * + switched to new modem/flow control configuration
 *   mechanism.
 * + fixed bug where only the first board in the system
 *   was available to xxtrace
 *
 * Revision 1.16.2.1  1996/10/08 22:48:57  bkd
 * + added code to print out board clock speed
 * + added code to determine chip type and revision,
 *   store this information into line structures,
 *   and print it out in load banner
 *
 * Revision 1.16  1996/09/27 14:24:30  bkd
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
 * + converted to use the new, standard XXTRACE (xxtrace.svr4)
 * + added baud rate tables (were in sio16.c)
 * + re-wrote attach routine to handle failure more gracefully (to
 *   properly recover resources).
 * + removed the old, bizarre version numbering mechanism and switched to
 *   the new one (version.h)
 *
 * Revision 1.15  1996/06/27 14:31:44  bkd
 * Changed versions from 5.12 to 5.13 Internal.
 *
 * Revision 1.14  1996/05/28 18:01:52  chung
 * Change to make sure a cu device cannot be opened if its corresponding
 * term device has already been opened.
 *
 * Revision 1.13  1995/08/15 23:14:22  bkd
 * Updated copyrights; added CVS control information
 *
 */

#include <sys/mkdev.h>	/* needed for NBITSMINOR macro */
#include <sys/modctl.h>
#include <sys/conf.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>
#include <sys/stream.h>
#include <sys/tty.h>	/* for tty_common_t */
#include <sys/kmem.h>	/* for KM_SLEEP and kmem_ prototypes */
#include <sys/errno.h>
#include <sys/stat.h>	/* S_IFCHR, below */
#include <sys/debug.h>	/* for the ASSERT() macro */

#include "sio16.h"
#include "sio16_ioctl.h"
#include "cd18x.h"
#include "version.h"

#define XXTRACE_HOST
#include "xxtrace.h"
#include "event.h"

/* ------------------------------------------------------------------------- */
/* Version variables */

static char copyright[] =
		"@(#)Copyright (c) 1993-1996 Aurora Technologies, Inc.";
static char RcsId[] = "@(#)$Header: /vol/sources.cvs/dev/sio16.svr4/sio16_osdep.c,v 1.16.2.4 1996/10/22 15:17:25 bkd Exp $";

/* ------------------------------------------------------------------------- */
/* External prototypes */

extern int nodev();	/* returns ENODEV when called */

/* ------------------------------------------------------------------------- */
/* Static prototypes */

static int sio16_identify(dev_info_t *dip);
static int sio16_attach(dev_info_t *dip, ddi_attach_cmd_t cmd);
static int sio16_detach(dev_info_t *dip, ddi_detach_cmd_t cmd);
static int sio16_getinfo(dev_info_t *dip, ddi_info_cmd_t infocmd,
		void *arg, void **result);
static int sio16_mapin(struct sio16_board *sbp);
static void sio16_mapout(struct sio16_board *sbp);
static void sio16_detachboard(register struct sio16_board *sbp);

/* ------------------------------------------------------------------------- */
/* External variables */

extern struct mod_ops mod_driverops;

/* ------------------------------------------------------------------------- */
/* Un-initialized static variables */

/* ------------------------------------------------------------------------- */
/* Un-initialized global variables */

void *sio16_head;

/* ------------------------------------------------------------------------- */
/* Initialized static variables */

#ifdef XXTRACE
static int maxprio = 0;
#endif		/* XXTRACE */

static int nboards = 0;		/* total number of attached boards */

static struct module_info sio16_info = {
    2,
    "sio16",
    0,
    INFPSZ,
    8192,
    1024
};

/* Read queue */
static struct qinit sio16_rinit = {
    putq,
    sio16_rsrv,
    sio16_open,
    sio16_close,
    NULL,
    &sio16_info,
    NULL
};

/* Write queue */
static struct qinit sio16_winit = {
    sio16_wput,
    NULL,
    NULL,
    NULL,
    NULL,
    &sio16_info,
    NULL
};

struct streamtab sio16_stab = {
    &sio16_rinit, 
    &sio16_winit, 
    NULL,
    NULL
};

DDI_DEFINE_STREAM_OPS(sio16_ops,\
    sio16_identify,		\
    nulldev,		 	\
    sio16_attach,		\
    sio16_detach,	 	\
    nodev,			\
    sio16_getinfo,		\
    D_NEW | D_MP,		\
    &sio16_stab);

static struct modldrv sio16_modldrv = {
    &mod_driverops,
    MODULE_NAME,		/* used for modstat only */
    &sio16_ops,
};

static struct modlinkage modlinkage = {
    MODREV_1, { (void *) &sio16_modldrv, NULL }
};


static struct sio16_baud sio16e_baud[] = {
    0,			0,		0,		/* B0 */
    SE_BSPEED(50),	255,	COR3_RX_THRESH_6,	/* B50 */
    SE_BSPEED(75),	255,	COR3_RX_THRESH_6,	/* B75 */
    SE_BSPEED(110),	255,	COR3_RX_THRESH_6,	/* B110 */
    SE_BSPEED(269/2),	238,	COR3_RX_THRESH_6,	/* B134 */
    SE_BSPEED(150),	213,	COR3_RX_THRESH_6,	/* B150 */
    SE_BSPEED(200),	160,	COR3_RX_THRESH_6,	/* B200 */
    SE_BSPEED(300),	107,	COR3_RX_THRESH_6,	/* B300 */
    SE_BSPEED(600),	53,	COR3_RX_THRESH_6,	/* B600 */
    SE_BSPEED(1200),	50,	COR3_RX_THRESH_6,	/* B1200 */
    SE_BSPEED(1800),	50,	COR3_RX_THRESH_6,	/* B1800 */
    SE_BSPEED(2400),	50,	COR3_RX_THRESH_6,	/* B2400 */
    SE_BSPEED(4800),	50,	COR3_RX_THRESH_6,	/* B4800 */
    SE_BSPEED(9600),	50,	COR3_RX_THRESH_6,	/* B9600 */ 
    SE_BSPEED(19200),	20,	COR3_RX_THRESH_5,	/* B19200 */
    SE_BSPEED(38400),	20,	COR3_RX_THRESH_4,	/* B38400 */

    /* Solaris 2.5 extended baud rates: */
    SE_BSPEED(57600),	20,	COR3_RX_THRESH_4,	/* B57600 */
    SE_BSPEED(76800),	20,	COR3_RX_THRESH_4,	/* B76800 */
    SE_BSPEED(115200),	20,	COR3_RX_THRESH_4,	/* B115200 */
    SE_BSPEED(153600),	20,	COR3_RX_THRESH_4,	/* B153600 */
    SE_BSPEED(230400),	20,	COR3_RX_THRESH_4,	/* B230400 */
    SE_BSPEED(307200),	20,	COR3_RX_THRESH_4,	/* B307200 */
    SE_BSPEED(460800),	20,	COR3_RX_THRESH_4	/* B460800 */
};

static struct sio16_baud sio16e2_baud[] = {
    0,			0,		0,		/* B0 */
    SE2_BSPEED(50),	255,	COR3_RX_THRESH_6,	/* B50 */
    SE2_BSPEED(75),	255,	COR3_RX_THRESH_6,	/* B75 */
    SE2_BSPEED(110),	255,	COR3_RX_THRESH_6,	/* B110 */
    SE2_BSPEED(269/2),	238,	COR3_RX_THRESH_6,	/* B134 */
    SE2_BSPEED(150),	213,	COR3_RX_THRESH_6,	/* B150 */
    SE2_BSPEED(200),	160,	COR3_RX_THRESH_6,	/* B200 */
    SE2_BSPEED(300),	107,	COR3_RX_THRESH_6,	/* B300 */
    SE2_BSPEED(600),	53,	COR3_RX_THRESH_6,	/* B600 */
    SE2_BSPEED(1200),	50,	COR3_RX_THRESH_6,	/* B1200 */
    SE2_BSPEED(1800),	50,	COR3_RX_THRESH_6,	/* B1800 */
    SE2_BSPEED(2400),	50,	COR3_RX_THRESH_6,	/* B2400 */
    SE2_BSPEED(4800),	50,	COR3_RX_THRESH_6,	/* B4800 */
    SE2_BSPEED(9600),	50,	COR3_RX_THRESH_6,	/* B9600 */ 
    SE2_BSPEED(19200),	20,	COR3_RX_THRESH_5,	/* B19200 */
    SE2_BSPEED(38400),	20,	COR3_RX_THRESH_4,	/* B38400 */

    /* Solaris 2.5 extended baud rates: */
    SE2_BSPEED(57600),	20,	COR3_RX_THRESH_4,	/* B57600 */
    SE2_BSPEED(76800),	20,	COR3_RX_THRESH_4,	/* B76800 */
    SE2_BSPEED(115200),	20,	COR3_RX_THRESH_4,	/* B115200 */
    0,			0,	0,			/* B153600 */
    SE2_BSPEED(230400),	20,	COR3_RX_THRESH_4,	/* B230400 */
    0,			0,	0,			/* B307200 */
    0,			0,	0			/* B460800 */
};

/* ------------------------------------------------------------------------- */
/* Initialized global variables */

#ifdef XXTRACE
ddi_iblock_cookie_t *sio16_iblock = NULL;
#endif		/* XXTRACE */
int sio16_close_timeout = 15;
int sio16_maxinstance = -1;	/* maximum instance */

/* ------------------------------------------------------------------------- */
/* Global routines */

/************************** LOADABLE DRIVER SUPPORT *********************/

int
_init(void)
{
    int		 e;

    if ((e = ddi_soft_state_init(&sio16_head,
	    sizeof(struct sio16_board), 1)) != 0) {
	return e;
    }

    if ((e = mod_install(&modlinkage)) != 0) {
	ddi_soft_state_fini(&sio16_head);
    }

#ifdef XXTRACE
    xxtraceinit0();	/* init xxtrace event table */
#endif		/* XXTRACE */

    return e;
}

int
_fini()
{
    int		 status;

    /* mod_remove causes the detach routines to be called */

    if ((status = mod_remove(&modlinkage)) != 0) {
	return status;
    }

    /*
     * Check to see if any boards are still attached.
     */

    /*
     * !!!bkd 9/13/1996 - I believe that under Solaris 2,
     *  this test is unnecessary.  I don't think the system
     *  will allow the module to be unloaded if there are
     *  open devices with it.
     */

    if (nboards != 0) {
	return EBUSY;
    }

#ifdef XXTRACE
    /* call xxtraceunload() if xxtrace has been setup */
    if (sio16_xtinit != 0) {
	xxtraceunload();
    }
#endif /* XXTRACE */

    ddi_soft_state_fini(&sio16_head);

    return status;
}

int
_info(struct modinfo *modinfop)
{
    return mod_info(&modlinkage, modinfop);
}


/* ------------------------------------------------------------------------- */
/* Static routines */


/************************* DRIVER IDENTIFY and ATTACH ************************/

static int
sio16_identify(dev_info_t *dip)
{
    register char *name;

    name = ddi_get_name(dip);
    /*
     * You can't count number of calls any more because
     * there is no guarantee that all identifies are called before
     * the first attach.  See page 37 of SunOS 5.0 writing device drivers.
     */
    DBGPRINTF("sio16_identify called with %s\n", name);

    if (strcmp(name, "sio16") == 0) {
	return DDI_IDENTIFIED;
    }

    return DDI_NOT_IDENTIFIED;
}

/* 
 * This is called once per board 
 */
static int
sio16_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
    register struct sio16_board		*sbp;
    char				*model;
    int					 proplen, modnum, bintr;
    int					 instance;
    int					 port, nports;
    int					 clk;		/* from prom */
    unsigned long			 clkspeed;	/* in Hz */
    int					 chip_type, chip_rev;
    register struct sioline		*linearray, *siop;
    unsigned char			 sbusintr, gfrcr, srcr;
    volatile struct cd180		*cdp;
    char				 minor_name[80];
    char				*dtr_rts;
    boolean				 dtr_rts_swapped;

    switch (cmd) {

    case DDI_ATTACH:
        /* the devinfo pointer should NEVER be NULL!! */
	ASSERT(dip != NULL);

	instance = ddi_get_instance(dip);

	/*
	 * Attempt to decode the board model
	 */

	modnum = SIO16_BADMOD;

	if (ddi_getlongprop(DDI_DEV_T_ANY, dip,
	    DDI_PROP_DONTPASS | DDI_PROP_CANSLEEP,
	    "model", (caddr_t) &model, &proplen) == DDI_PROP_SUCCESS) {

	    DBGPRINTF("model = %s\n", model);

	    if (strcmp(model, "1600se") == 0) {
		modnum = SIO16_1600SE;
	    }
	    else {
		cmn_err(CE_WARN,
			"sio16_attach: instance %d: invalid model (%s)",
			instance, model);
	    }

	    kmem_free((void *) model, (size_t) proplen);
	}
	else {
	    cmn_err(CE_WARN,
		    "sio16_attach: missing model property",
		    instance, model);
	}
	    
	if (modnum == SIO16_BADMOD) {
	    goto fail1;
	}

	bintr = ddi_getprop(DDI_DEV_T_ANY, dip, DDI_PROP_DONTPASS, 
	    "bintr", -1);

	DBGPRINTF("getprop of \"bintr\" returned %d\n", bintr);

	if (bintr < 0 || bintr > 7) {
	    bintr = 5;
	    cmn_err(CE_WARN,
		"sio16_attach: instance %d: setting board intr to %d\n",
				instance, bintr);
	}

	/*
	 * Translate the "bintr" property into a value
	 *  that can be used to program the board.
	 */

	switch (bintr) {

	case 2:
	    sbusintr = SBUS_INT_2_1600SE;
	    break;

	case 3:
	    sbusintr = SBUS_INT_3_1600SE;
	    break;

	case 5:
	    sbusintr = SBUS_INT_5_1600SE;
	    break;

	case 7:
	    sbusintr = SBUS_INT_7_1600SE;
	    break;

	default:
	    goto fail2;
	    /* NOTREACHED */

	}

	if (ddi_soft_state_zalloc(sio16_head, instance) != DDI_SUCCESS) {
	    goto fail3;
	}


	sbp = (struct sio16_board *) ddi_get_soft_state(sio16_head, instance);

	sbp->model = modnum;
	sbp->dip = dip;			/* store devinfo pointer */
	sbp->instance = instance;

	/*
	 * Figure out how many ports there are on this board.
	 */
	if ((nports = (int) ddi_getprop(DDI_DEV_T_ANY, dip,
				DDI_PROP_DONTPASS | DDI_PROP_CANSLEEP,
				"nports", NSIO_PORTS)) <= 0) {
	    nports = NSIO_PORTS;
	}

	sbp->nport = nports;

	/*
	 * Allocate an array of line structures
	 */

	/* allocate space for each line on the board */
	if ((linearray = (struct sioline *) kmem_zalloc(
			nports * sizeof(struct sioline), KM_SLEEP)) == NULL) {
	    cmn_err(CE_WARN,
    "sio16_attach: instance %d: could not allocate space for line structures",
			instance);
	    goto fail4;
	}

	sbp->siolines = linearray;

	/*
	 * Attempt to map the board in.
	 */

	if (sio16_mapin(sbp) != 0) {
	    goto fail5;
	}

	/*
	 * read value of Clock.  Integer value, 11.05Mhz = 110500
	 */
	clk = ddi_getprop(DDI_DEV_T_ANY, dip, DDI_PROP_DONTPASS, "clk", -1);
	if (clk == 110500) {
	    clkspeed = SE2_CLK;
	}
	else {
	    clkspeed = SE_CLK;
	}
	DBGPRINTF("getprop of \"clk\" = %d\n", clk);
	sbp->clk = clk;

	/*
	 * bkd 10/15/1996 - ugh.  The 1600SE Rev. A
	 *  (labeled on the PCB as 1600Sx) and the 1600SE
	 *  Rev. B both use the same PROM (1.1).  The 1.1
	 *  PROM does not define a dtr_rts property.  Unfortunately,
	 *  the Rev. A has straight-through DTR/RTS wiring
	 *  and the Rev. B has swapped DTR/RTS wiring.
	 *  I cannot distinguish between the two here in
	 *  the driver.  So, I'm going to assume that
	 *  only the Rev. A boards were sent to customers.
	 *  What this means is that if I don't see a dtr_rts
	 *  property, I'm going to assume that the wiring
	 *  is straight through.
	 * Right off the bat, this means that the 1600SE
	 *  Rev. B boards *WILL NOT WORK PROPERLY*.  I don't
	 *  think this is a huge problem, because 1) I know
	 *  that the current driver (5.11) doesn't work
	 *  properly with them and 2) I suspect that customers
	 *  never received the things.
	 */
	/*
	 * read value of dtr_rts flag.  If "swapped" then set DTRSWAP
	 */
	if (ddi_getlongprop(DDI_DEV_T_ANY, dip,
	    DDI_PROP_DONTPASS | DDI_PROP_CANSLEEP,
	    "dtr_rts", (caddr_t) &dtr_rts, &proplen) == DDI_PROP_SUCCESS) {

	    DBGPRINTF("dtr_rts = %s\n", dtr_rts);
	    if (strcmp(dtr_rts, "swapped") == 0) {
		dtr_rts_swapped = TRUE;
	    }
	    else if (strcmp(dtr_rts, "straight-thru") == 0) {
		dtr_rts_swapped = FALSE;
	    }
	    else {
		dtr_rts_swapped = FALSE;
		DBGPRINTF("unknown value for \"dtr_rts\"\n");
	    }

	    kmem_free((void *) dtr_rts, (size_t) proplen);
	}
	else {
	    dtr_rts_swapped = FALSE;
	    DBGPRINTF("no \"dtr_rts\" flag found\n");
	}

	/*
	 * Add driver interrupt
	 */
	if (ddi_add_intr(dip, (u_int) 0, &(sbp->iblock), &(sbp->idevice),
			sio16_poll, (caddr_t) sbp) != DDI_SUCCESS) {
	    goto fail6;
	}

	/*
	 * Initialize the chip mutexes...must be done *after*
	 *  interrupt is added.
	 */
	mutex_init(&(sbp->chiplock[0]), "sio16_chiplock_0",
	    MUTEX_DRIVER, (void *) sbp->iblock);
	mutex_init(&(sbp->chiplock[1]), "sio16_chiplock_1",
	    MUTEX_DRIVER, (void *) sbp->iblock);

	/*
	 * reset the board & wait for the reset to propagate to the chip
	 */
	DBGPRINTF("reset the board\n");

	DBGPRINTF("set the interrupt for 1600se\n");
	/*
	 * disable board interrupts
	 * and toggle the RESET* line
	 */
	*(sbp->bcsr) = 0x0;
	DBGPRINTF("setting bcsr = 0x%x\n",
			(BCSR_INT_EN | sbusintr | BCSR_CLR_RESET));
	*(sbp->bcsr) = (BCSR_INT_EN | sbusintr | BCSR_CLR_RESET);

	/*
	 * This is the delay for the cd180 microcode initialization
	 * i.e., setting bcsr above.
	 */
	DELAY(1000);

	chip_type = CT_UNKNOWN;
	chip_rev = CR_UNKNOWN;

	for (port = 0; port < nports; port++) {

	    /*
	     * Create the tty device nodes.
	     */
	    sprintf(minor_name, "%c", 'a' + port);
	    if (ddi_create_minor_node(dip, minor_name, S_IFCHR,
		port + BOARD2MINOR(instance), DDI_NT_SERIAL,0) != 
		DDI_SUCCESS) {

		goto fail7;
	    }

	    /*
	     * Create the cu device nodes.
	     */
	    sprintf(minor_name, "%c,cu", 'a' + port);
	    if (ddi_create_minor_node(dip, minor_name, S_IFCHR,
		port + BOARD2MINOR(instance) | OUTLINE,
		DDI_NT_SERIAL_DO, 0) != DDI_SUCCESS) {

		goto fail7;
	    }

	    siop = &(linearray[port]);
	    DBGPRINTF("siop = 0x%x\n", siop);
	    siop->board_ptr = sbp;

	    /*
	     * Initialize the mutexes and condition variables all
	     * at once so it's easier to clean up if attach fails later.
	     */
	    cv_init(&siop->cv, "sio16_cv", CV_DRIVER, NULL);
	    siop->chiplock = &(sbp->chiplock[CHIP(port)]);

	    /* store the board & port values for XXTRACE use */
	    siop->board = instance;
	    siop->port = port;

	    /* which chip are we dealing with */
	    cdp = siop->cd180 = 
		CHIP(port) == 1 ? (volatile struct cd180 *) sbp->chip2_reg :
				  (volatile struct cd180 *) sbp->chip1_reg;

	    /* load baudrate table */
	    /*
	     * initialize custom baud rate, can only be changed
	     * on 1600SE
	     */
	    siop->cust_baud = 38400;
	    if (clk == 110500) {
		/* 11.05Mhz */
		siop->baud_tbl = sio16e2_baud;
		siop->cust_bpr = SE2_BSPEED(38400);
	    }
	    else {
		/* set to 14.7456Mhz by default */
		siop->baud_tbl = sio16e_baud;
		siop->cust_bpr = SE_BSPEED(38400);
	    }

	    /* initialize modem flags */
	    siop->dsr_mask = 0;
	    if (port >= 0xc && port <= 0xf) {
		siop->cts_mask = 0;
		siop->cd_mask = MSVR_CTS;
		siop->rts_mask = 0;

		if (dtr_rts_swapped) {		/* 1600SE Rev. C */
		    siop->dtr_mask = MSVR_DTR;
		}
		else {				/* 1600SE Rev. A */
		    siop->dtr_mask = MSVR_RTS;
		}

		siop->modem_flags = SIO_MAPCD | SIO_MAPDTR;
	    }
	    else {
		siop->cts_mask = MSVR_CTS;
		siop->cd_mask = MSVR_DSR;

		if (dtr_rts_swapped) {		/* 1600SE Rev. C */
		    siop->dtr_mask = MSVR_RTS;
		    siop->rts_mask = MSVR_DTR;
		}
		else {				/* 1600SE Rev. A */
		    siop->dtr_mask = MSVR_DTR;
		    siop->rts_mask = MSVR_RTS;
		}

		siop->modem_flags = SIO_MAPALL;
	    }

	    /* close timeout */
	    siop->close_timeout = sio16_close_timeout;

	    /* init chip */
	    if ((port & 0x7) == 0) {
		cdp->car = 0x0;
		DBGPRINTF("init chip\n");
		cdp->gsvr = 0;
		cdp->ccr = CCR_RST_CHIP;	/* reset chip */

		DELAY(1);
		/*
		 * wait for gsvr to become 0xff before continuing
		 */
		DELAYCK((cdp->gsvr != 0xff), port, "gsvr");

		/*
		 * load up the gsvr with a chip identifier
		 */
		cdp->gsvr = (port & 0x8);
		DBGPRINTF("gsvr = %d\n", (port & 0x8));

		/*
		 * set up the service match registers
		 */
		cdp->msmr = PILR1;	/* modem/timer iack */
		cdp->tsmr = PILR2;	/* tx iack */
		cdp->rsmr = PILR3;	/* rx iack */

		/*
		 * set the prescalar values
		 *
		 * 1ms prescalar, 0.5ms minimum
		 */
		if (clk == 110500) {
		    cdp->pprh = (11050 >> 8);
		    cdp->pprl = 11050;
		}
		else {
		    cdp->pprh = (14745 >> 8);
		    cdp->pprl = 14745;
		}

		/*
		 * Figure out what kind of chip we are talking to.
		 *  This information is then used to initialize
		 *  all of the other lines on this chip.
		 */

		gfrcr = cdp->gfrcr;
		srcr = cdp->srcr;

		if ((srcr & SRCR_PKGTYPE) == 0) {	/* PLCC - CD-180 */
		    chip_type = CT_CL_CD180;
		    chip_rev = CR_REVA + (gfrcr - GFRCR_CD180_REVA);
		}
		else {					/* PQFP - 1864/1865 */
		    if (gfrcr == GFRCR_CD1864_REVA) {	/* 1864 */
			chip_type = CT_CL_CD1864;
			chip_rev = CR_REVA;
		    }
		    else {				/* 1865 */
			chip_type = CT_CL_CD1865;
			chip_rev = CR_REVA + (gfrcr - GFRCR_CD1865_REVA);
		    }
		}

		/* set up the priority */
		cdp->srcr = (SRCR_GLOBPRI | SRCR_AUTOPRI);
		DBGPRINTF("setting srcr\n");
	    }

	    siop->chip_type = chip_type;
	    siop->chip_rev = chip_rev;
	}

#ifdef XXTRACE
	/*
	 * Get the iblock pointer for the highest priority board.
	 *  Only process this board if it is within the range of
	 *  what is acceptable to XXTRACE.
	 */

	if (instance < MAXDEVS) {
	    if (sio16_iblock == NULL
	        || (int) sbp->idevice.idev_priority > maxprio) {

		maxprio = (int) sbp->idevice.idev_priority;
		sio16_iblock = &(sbp->iblock);
	    }
	    xxtraceinit_bitmap(instance, nports);
	}

	DBGPRINTF("maxprio %x\n", maxprio);
#endif

	DBGPRINTF("iblock %x, idevice %x,",
	    *((long *) &(sbp->iblock)), *((long *) &(sbp->idevice)));

	/*
	 * Keep track of the maximum instance of board.
	 *  This is to allow us to check all of the boards
	 *  at unload time (which is probably not necessary
	 *  anyway).
	 */
	if (instance > sio16_maxinstance) {
	    sio16_maxinstance = instance;
	}

	/*
	 * print load string.  Chip information is based
	 *  on the last chip on the board.
	 */
	printf("Aurora %s board:%d, %s-%c %lu.%04lu MHz, intr %d, %s\n",
		sbp->model == SIO16_1600SE ? "1600SE" : "1600??",
		instance,
		chip_type == CT_CL_CD180 ? "CL-CD180" :
		 (chip_type == CT_CL_CD1864 ? "CL-CD1864" :
		  (chip_type == CT_CL_CD1865 ? "CL-CD1865" : "???")),
		chip_rev == CR_UNKNOWN ? '?' : (chip_rev - CR_REVA) + 'A',
		(clkspeed / (unsigned long) 1000000),
		((clkspeed / (unsigned long) 100) % (unsigned long) 10000),
		bintr, 
		MOD_VERSION);

	ddi_report_dev(dip);
	nboards++;

	return DDI_SUCCESS;

fail7:	/* either ddi_create_minor_nodes failed in the port loop */
	while (port-- > 0) {
	    cv_destroy(&(sbp->siolines[port].cv));
	}
	    
	ddi_remove_minor_node(dip, NULL);
	mutex_destroy(&(sbp->chiplock[1]));
	mutex_destroy(&(sbp->chiplock[0]));
	ddi_remove_intr(dip, (u_int) 0, sbp->iblock);

fail6:	/* ddi_add_intr() failed */
	sio16_mapout(sbp);

fail5:	/* sio16_mapin() failed */
	kmem_free((void *) linearray, nports * sizeof(struct sioline));

fail4:	/* kmem_zalloc() fails for linearray */
	ddi_soft_state_free(sio16_head, instance);

fail3:	/* ddi_soft_state_zalloc() failed */

fail2:	/* bintr != {2, 3, 5, 7} */

fail1:	/* modnum == SIO16_BADMOD */
	return DDI_FAILURE;
	/* NOTREACHED */

#ifdef DDI_RESUME
    case DDI_RESUME:
	return DDI_FAILURE;
#endif		/* DDI_RESUME */

    default:
	break;

    }

    return DDI_FAILURE;
}

static int
sio16_detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
    register struct sio16_board		*sbp;
    int					 instance, port;

    switch (cmd) {

    case DDI_DETACH:
	instance = ddi_get_instance(dip);

	if ((sbp = (struct sio16_board *) ddi_get_soft_state(sio16_head,
					instance)) == NULL) {
	    return DDI_FAILURE;
	}

	/*
	 * Loop through all of the ports and make sure
	 *  that no port is open.
	 */

	for (port = 0; port < sbp->nport; port++) {
	    if ((sbp->siolines[port].flags & SIO_ISOPEN) != 0) {
		return DDI_FAILURE;
	    }
	}

	sio16_detachboard(sbp);
	nboards--;
	return DDI_SUCCESS;

#ifdef DDI_SUSPEND
    case DDI_SUSPEND:
	return DDI_FAILURE;
#endif		/* DDI_SUSPEND */

    }

    return DDI_FAILURE;
}

/*
 * Dev_info Conversion routines.
 */
static int
sio16_getinfo(dev_info_t *dip, ddi_info_cmd_t infocmd,
		void *arg, void **result)
{
    register int		 instance;
    register int		 retcode;
    struct sio16_board		*sbp;

    instance = BOARD(getminor((dev_t) arg));

    switch (infocmd) {

    case DDI_INFO_DEVT2DEVINFO:
	sbp = (struct sio16_board *) ddi_get_soft_state(sio16_head, instance);

	if (sbp != NULL) {
	    *result = (void *) sbp->dip;
	    retcode = DDI_SUCCESS;
	}
	else {
	    retcode = DDI_FAILURE;
	}
	break;

    case DDI_INFO_DEVT2INSTANCE:
	*result = (void *) instance;
	retcode = DDI_SUCCESS;
	break;

    default:
	retcode = DDI_FAILURE;
	break;

    }

    return retcode;
}

/*
 * Map the hardware in.  This is in a separate
 *  subroutine in order to try to simplify error
 *  recovery.
 *
 * Returns 0 on success, -1 otherwise.
 */
static int
sio16_mapin(struct sio16_board *sbp)
{
    dev_info_t		*dip;

    dip = sbp->dip;

    /*
     * Be sensitive to board type here.
     */

    if (ddi_map_regs(dip, REG_CSR, (caddr_t *) &(sbp->bcsr), 0, 0)
						    != DDI_SUCCESS) {
	goto fail1;
    }

    if (ddi_map_regs(dip, REG_CD1, (caddr_t *) &(sbp->chip1_reg), 0, 0)
						    != DDI_SUCCESS) {
	goto fail2;
    }

    if (ddi_map_regs(dip, REG_CD2, (caddr_t *) &(sbp->chip2_reg), 0, 0)
						    != DDI_SUCCESS) {
	goto fail3;
    }

    if (ddi_map_regs(dip, REG_IACK, (caddr_t *) &(sbp->iackbase), 0, 0)
						    != DDI_SUCCESS) {
	goto fail4;
    }

    return 0;

fail5:
    ddi_unmap_regs(dip, REG_IACK, (caddr_t *) &(sbp->iackbase), 0, 0);

fail4:
    ddi_unmap_regs(dip, REG_CD2, (caddr_t *) &(sbp->chip2_reg), 0, 0);

fail3:
    ddi_unmap_regs(dip, REG_CD1, (caddr_t *) &(sbp->chip1_reg), 0, 0);

fail2:
    ddi_unmap_regs(dip, REG_CSR, (caddr_t *) &(sbp->bcsr), 0, 0);

fail1:

    return -1;
}


/*
 * Map the hardware out; remove all mappings to the hardware.
 */

static void
sio16_mapout(struct sio16_board *sbp)
{
    dev_info_t		*dip;

    dip = sbp->dip;

    /*
     * Be sensitive to board type here.
     */

    ddi_unmap_regs(dip, REG_IACK, (caddr_t *) &(sbp->iackbase), 0, 0);
    ddi_unmap_regs(dip, REG_CD2, (caddr_t *) &(sbp->chip2_reg), 0, 0);
    ddi_unmap_regs(dip, REG_CD1, (caddr_t *) &(sbp->chip1_reg), 0, 0);
    ddi_unmap_regs(dip, REG_CSR, (caddr_t *) &(sbp->bcsr), 0, 0);
}

/*
 * Detach the given board.  This frees up all
 *  resources used by the board and unmaps the
 *  device.
 */

static void
sio16_detachboard(register struct sio16_board *sbp)
{
    register int		 instance, port;
    struct sioline		*siop;
    dev_info_t			*dip;
    volatile struct cd180	*cdp;

    instance = sbp->instance;
    dip = sbp->dip;

    /*
     * Turn off interrupts from the
     *  board bcsr & clear the reset
     */
    *(sbp->bcsr) = (BCSR_CLR_RESET);

    /*
     * Go through each port and destroy all of the
     *  port-associated resources.  This includes
     *  the condition variable.
     */

    for (port = 0; port < sbp->nport; port++) {
	siop = &(sbp->siolines[port]);
	cv_destroy(&(siop->cv));

	/* reset the chip */
	if ((port & 0x7) == 0) {
	    cdp = siop->cd180;

	    cdp->car = 0x0;
	    DBGPRINTF("reset chip\n");
	    cdp->gsvr = 0;
	    cdp->ccr = CCR_RST_CHIP; /* reset chip */

	    /*
	     * wait for gsvr to become 0xff 
	     * before continuing
	     */
	    DELAYCK((cdp->gsvr != 0xff), port, "gsvr");
	}

	/*
	 * Free up the statistics blocks
	 */

	if (siop->l_sio_stats != NULL) {
	    kmem_free((void *) siop->l_sio_stats, sizeof(sio_stats_t));
	}

	if (siop->l_sio_stats_valid != NULL) {
	    kmem_free((void *) siop->l_sio_stats_valid, sizeof(sio_stats_t));
	}

    }

    ddi_remove_minor_node(dip, NULL);
    mutex_destroy(&(sbp->chiplock[1]));
    mutex_destroy(&(sbp->chiplock[0]));
    ddi_remove_intr(dip, (u_int) 0, sbp->iblock);

    sio16_mapout(sbp);

    kmem_free((void *) sbp->siolines, sbp->nport * sizeof(struct sioline));

    ddi_soft_state_free(sio16_head, instance);
}
