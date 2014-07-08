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


	file: pio.c
	author: bwalker
	created: 11/7/92
	sccs info: @(#)pio.c	1.14 09 Apr 1993 1

	1990 - 1992 (c) Aurora Technologies, Inc. All Rights Reserved.
*/
static char CopyId[] = "@(#)Copyright (c) 1992 INSERT_HERE_COPYR_NAME";
static char SccsId[] = "@(#)INSERT_HERE_CO_NAME, @(#)pio.c	1.14 09 Apr 1993 1 [pio Version INSERT_HERE_VERSION]";

#include <sys/types.h>
#include <sys/param.h>
#include <sys/errno.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/map.h>
#include <sys/kmem.h>
#include <sys/debug.h>
#include <sys/modctl.h>

#include <sys/systm.h>
#include <sys/stream.h>
#include <sys/stropts.h>
#include <sys/termios.h>
#include <sys/termio.h>
#include <sys/ttold.h>
#include <sys/tty.h>
#include <sys/user.h>
#include <sys/proc.h>
#include <sys/buf.h>
#include <sys/obpdefs.h>

#include <sys/ddi.h>
#include <sys/sunddi.h>

#include "pio.h"

extern kcondvar_t lbolt_cv;

/*
 *  debug flags used
 */
static int debug = 0;
#define DBG_CONT(x)		if (debug) \
					cmn_err(CE_CONT, x);
#define DBG_WARN(x)		if (debug) \
					cmn_err(CE_WARN, x);

#define NORMAL_MASK	0xb8
#define NORMAL_COND	0x98

#define IGNORE_MASK	0x90
#define IGNORE_COND	0x90

#define BUSY_MASK       0x90
#define BUSY_STATUS     0x10

/* declaration of general board routines */
static int	pio_identify(dev_info_t *);			/* identify each board */
static int	pio_attach(dev_info_t *, ddi_attach_cmd_t);	/* attach board identified */
static int	pio_detach(dev_info_t *dip, ddi_detach_cmd_t cmd);
static int	pio_getinfo(dev_info_t *dip, ddi_info_cmd_t infocmd, void *arg, void **result);
        

/* parallel port routines */
static int	pio_open(queue_t *, dev_t *, int, int, cred_t *);
static int	pio_close(queue_t *, int, cred_t *);

static void	pio_start(struct pioline *);
static u_int	pio_intr(caddr_t);

static int	pioioctl(struct pioline *, queue_t *, mblk_t  *);
static void	pio_reioctl(struct pioline *);
static int	pio_wput(queue_t *, mblk_t *);
static void	piorestart(struct pioline *);

/*
 * an array of pointers that is allocated in pioattach()
 * each structure describes a line
 */
static void *parallel_boards;

static struct module_info pio_info = {
	0,
	"pio",
	0,
	INFPSZ,
	PSIZE,
	1024
};

/* Read queue */
static struct qinit piorinit = {
	NULL,
	NULL,
	pio_open,
	pio_close,
	NULL,
	&pio_info,
	NULL
};

/* Write queue */
static struct qinit piowinit = {
	pio_wput,
	NULL,
	NULL,
	NULL,
	NULL,
	&pio_info,
	NULL
};

struct streamtab pio_stab = {
	&piorinit,
	&piowinit,
	NULL,
	NULL
};

/*
 * When our driver is loaded or unloaded, the system calls our _init or
 * _fini routine with a modlinkage structure.  The modlinkage structure
 * contains:
 *
 *      modlinkage->
 *              modldrv->
 *                      dev_ops->
 *                              cb_ops
 *
 * cb_ops contains the normal driver entry points
 *
 *
 * dev_ops contains, in addition to the pointer to cb_ops, the routines
 * that support loading and unloading our driver.
 *
 * DDI_DEFINE_STREAM_OPS will set up the cb_ops & dev_ops structures for
 * us.  this is helpful in regards to STREAMS module/driver building
 */
DDI_DEFINE_STREAM_OPS(	pio_ops, \
			pio_identify, \
			nulldev, \
			pio_attach, \
			pio_detach, \
			nodev, \
			pio_getinfo, \
			D_NEW | D_MP, \
			&pio_stab);
 
extern  struct  mod_ops mod_driverops;
static  struct modldrv modldrv = {
        &mod_driverops,
        "r&d version of parallel driver",
        &pio_ops,
};

static  struct modlinkage modlinkage = {
        MODREV_1,               /* MODREV_1 indicated by manual */
        (void *)&modldrv,
        NULL,                   /* termination of list of linkage structures */
};

/*
 * _init, _info, and _fini support loading and unloading the driver.
 */
static int
_init(void)
{
	int error;

	DBG_CONT("inside of _init\n");
	error = ddi_soft_state_init(&parallel_boards, sizeof(struct parallel_board), 0);
	if (error != 0)
		return(error);

	if ((error = mod_install(&modlinkage)) != 0)
		ddi_soft_state_fini(&parallel_boards);

	return(error);
}
 
static int
_info(struct modinfo *modinfop)
{
        return (mod_info(&modlinkage, modinfop));
}
 
static int
_fini(void)
{
        int error;
 
	DBG_CONT("inside of _fini\n");
        if ((error = mod_remove(&modlinkage)) != 0)
                return (error);

	ddi_soft_state_fini(&parallel_boards);

	return(0);
}

/*
 * This is a pretty generic getinfo routine as describe in the manual.
 */
/*ARGSUSED*/
static int
pio_getinfo(dev_info_t *dip, ddi_info_cmd_t infocmd, void *arg, void **result)
{
        register struct parallel_board *pbp;
        int error, instance;

        switch (infocmd) {
        case DDI_INFO_DEVT2DEVINFO:
		instance = BOARD((dev_t)arg);
		pbp = (struct parallel_board *)ddi_get_soft_state(parallel_boards, instance);
		if (pbp == NULL) {
                        *result = NULL;
                        error = DDI_FAILURE;
                } else {
                        *result = (void *)pbp->devinfo;
                        error = DDI_SUCCESS;
                }
                break;
        case DDI_INFO_DEVT2INSTANCE:
                *result = (void *)BOARD((dev_t)arg);
                error = DDI_SUCCESS;
                break;
        default:
                *result = NULL;
                error = DDI_FAILURE;
        }
        return (error);
}

/*
 * When our driver is loaded, pio_identify() is called with a dev_info_t
 * full of information from the FCode on our board.
 *
 * in 5.0, identify() will only be called for devices whose
 * FCode "name" property matches the name of the driver (e.g. name: "pio",
 * driver source: pio.c, driver module: /kernel/drv/pio).  You can get
 * around this by editing /etc/driver_aliases or by using the "-i" option
 * for add_drv(1).  (e.g. name: "Aurora-210SJ-", driver module: /kernel/drv/pio.)
 */
static int	
pio_identify(dev_info_t *dip)
{
	char *dev_name;

	dev_name = ddi_get_name(dip);

	if ((strcmp(dev_name, "pio1") == 0) ||
	    (strcmp(dev_name, "pio2") == 0)) {
		return (DDI_IDENTIFIED);
	} else {
		return (DDI_NOT_IDENTIFIED);
	}
}
 
/*
 * pio_attach gets called if pio_identify returns DDI_IDENTIFIED
 */
static int
pio_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	struct	parallel_board *pio_board_ptr;
        struct	pioline	**parallel_lines, *piop;
	int	reg_size, nports;
	int	instance, cur_port;
	int	nreg, bintr;
	char	*name;
	char	port_name[32];
	u_char	status;

	if (cmd != DDI_ATTACH)
		return (DDI_FAILURE);

	DBG_CONT("inside of attach\n");

	instance = ddi_get_instance(dip);
	name = ddi_get_name(dip);

	if (ddi_soft_state_zalloc(parallel_boards, instance) != DDI_SUCCESS)
		return (DDI_FAILURE);
	if (strcmp(name, "pio2") == 0) {
		nports = 2;
	} else if (strcmp(name, "pio1") == 0) {
		nports = 1;
	}

	pio_board_ptr = (struct parallel_board *)ddi_get_soft_state(parallel_boards, instance);

	parallel_lines = (struct pioline **)kmem_zalloc(
				(nports * sizeof(struct pioline *)), KM_NOSLEEP);
	if (parallel_lines == (struct pioline **)NULL) {
		DBG_WARN("kmem_zalloc: ENOMEM for parallel_lines\n");
		goto attach_failed;
	}
	pio_board_ptr->piolines = parallel_lines;
	pio_board_ptr->nport = nports;
	pio_board_ptr->devinfo = dip;

	DBG_CONT("setup add_intr\n");
	/* put this board onto the interrupt vector list */
	if (ddi_add_intr(dip, (u_int)0, &(pio_board_ptr->iblock_cookie),
		&(pio_board_ptr->idevice_cookie), (u_int (*)())pio_intr,
			(caddr_t)instance) != DDI_SUCCESS)
			return (DDI_FAILURE);

	/*
	 * THE REGISTERS LOOK OK, SO LET'S MAP THEM IN SO WE CAN USE THEM
	 */
	if (strcmp(name, "pio1") == 0) {
		/* we need to map in BCSR & IACKBAS registers */
		reg_size = ddi_dev_regsize(dip, BCSR_REG, (off_t *)&reg_size);
		if (ddi_map_regs(dip, BCSR_REG, (caddr_t*)&(pio_board_ptr->bcsr),
					0, reg_size) != DDI_SUCCESS) {
			goto attach_failed;
		}
		reg_size = ddi_dev_regsize(dip, IACK_REG, (off_t *)&reg_size);
		if (ddi_map_regs(dip, IACK_REG, (caddr_t*)&(pio_board_ptr->int2clk),
					0, reg_size) != DDI_SUCCESS) {
			goto attach_failed;
		}

		DBG_CONT("get bintr property\n");
		bintr = (int)ddi_getprop(DDI_DEV_T_ANY, dip, 0, "bintr", -1);
		if (bintr == -1) {
			bintr = 7;
			cmn_err(CE_CONT, "setting board intr to %d\n", bintr);
		}

		switch (bintr) {
		case 5: bintr = SBUS_INT_5;
			break;
		case 6: bintr = SBUS_INT_6;
			break;
		case 7: bintr = SBUS_INT_7;
			break;
		default: goto attach_failed;
			/*NOTREACHED*/
			break;
		}
		pio_board_ptr->bintr = bintr;
       		status = *pio_board_ptr->bcsr;
	       	*pio_board_ptr->bcsr = ((status & 0x0f) | bintr) & ~LPTOE;
	} else {
		pio_board_ptr->bintr = 0;
		pio_board_ptr->bcsr = NULL;
		pio_board_ptr->int2clk = NULL;
	}

	for (cur_port = 0; cur_port < nports; cur_port++) {
		parallel_lines[cur_port] =
                        (struct pioline *)kmem_zalloc(
						sizeof(struct pioline), KM_NOSLEEP);
                if (parallel_lines[cur_port] == (struct pioline *)NULL) {
                        cmn_err(CE_WARN, "kmem_zalloc: failed parallel_lines[%d]\n", cur_port);
                        goto attach_failed;
                }
                piop = parallel_lines[cur_port];

		/* back pointer to the board for this port */
		piop->board_ptr = pio_board_ptr;

		DBG_CONT("setup mutex\n");
		/*
		 * initialize MUTEX; we will use this MUTEX later on to lock
		 * our instance structure
		 */
		mutex_init(&piop->mutex, "pio mutex",
			MUTEX_DRIVER, (void *)&(pio_board_ptr->iblock_cookie));

		DBG_CONT("setup cond. variable\n");
		/*
		 * initialize cond. var.; we will use this later on to wait for
		 * the ACK to come through
		 */
		cv_init(&piop->cv_busy, "pio_cv_busy", CV_DRIVER,
				(void*)&(pio_board_ptr->iblock_cookie));

		if (ddi_dev_nregs(dip, &nreg) != DDI_SUCCESS) {
			cmn_err(CE_WARN, "no registers for %s\n", name);
			goto attach_failed;
		}

		reg_size = ddi_dev_regsize(dip, cur_port, (off_t *)&reg_size);
		/*
		 * the registers look OK, so let's map them in so we can use them
		 */
		if (strcmp(name, "pio1") == 0) {
			if (ddi_map_regs(dip, (PORT_REG_OFFSET + cur_port),
				(caddr_t *)&(piop->base), 0, reg_size) != DDI_SUCCESS) {
					goto attach_failed;
			}
			status = *piop->board_ptr->bcsr;
			DBG_CONT("turn off LPTOE\n");
			*piop->board_ptr->bcsr = ((status & 0x0f) | bintr) & ~LPTOE;
 
			status = *piop->board_ptr->bcsr;
			DBG_CONT("setup INT\n");
			*piop->board_ptr->bcsr = ((status & 0x0f) | bintr) | PARA_INT_ENA;
		} else if (ddi_map_regs(dip, cur_port, (caddr_t *)&(piop->base),
						0, reg_size) != DDI_SUCCESS) {
				goto attach_failed;
		}

		DBG_CONT("calling ddi_create_minor_node\n");
		/*
		 * ddi_create_minor_node creates an entry in an internal kernel
		 * table; the actual entry in the file system is created by
		 * drvconfig(1) when you run add_drv(1);
		 */
		sprintf(port_name, "%d", cur_port);
		if (ddi_create_minor_node(dip, port_name, S_IFCHR, ((instance << 2) + cur_port),
					DDI_PSEUDO, NULL) == DDI_FAILURE) {
			DBG_WARN("ddi_create_minor_node failed\n");
			goto attach_failed;
		}

		/* setup the default chars that we try to send out */
		piop->xmit_fifo = PSIZE;

		/* set up status mask and normal condition for board */
		piop->status_mask = NORMAL_MASK;
		piop->normal_status = NORMAL_COND;

		/*
		 *  disable sbus interrupts from the parallel port
		 *  they get turned on in the open routine and turned off in close
		 */
		P_DISABLE(piop);

		DBG_CONT("clear AFD\n");
		/* clear the AFD bit to tell ScanJet we don't wanna listen */

		*(piop->base + READ_WRITE_CTRL) &= ~AFD;

		DBG_CONT("set direction bit\n");
		/* clear the READ_PARALLEL direction bit so we do output only */
		*(piop->base + READ_WRITE_CTRL) &= ~READ_PARALLEL;

		/*
		 *  init Printer
		 */
		DBG_CONT("clear init\n");
		*(piop->base + READ_WRITE_CTRL) &= ~INIT;

		/*
		 *  set the parallel port interrupt enable
		 */
		DBG_CONT("set init\n");
		*(piop->base + READ_WRITE_CTRL) |= (INIT | PIRQ_EN);
 
		CLR_INTR(piop);
		DBG_CONT("finish setting parallel base addresses\n");
	}
 
        ddi_report_dev(dip);
        return (0);

attach_failed:
        /*
         * need to add the code here to kmem free any allocated data structures
         */
        DBG_WARN("pioattach: failed!\n");
	DBG_WARN("fatal error\n");
        return (-1);
}
 
/*
 * When our driver is unloaded, pio_detach cleans up and frees the resources
 * we allocated in pio_attach.
 */
static int
pio_detach(dev_info_t *dip, ddi_detach_cmd_t cmd)
{
	register struct pioline	*piop;
        struct parallel_board	*pbp;
	int	instance, port;

	DBG_CONT("inside of detach\n");
	switch(cmd) {
	case DDI_DETACH:
		instance = ddi_get_instance(dip);
		pbp = (struct parallel_board *)ddi_get_soft_state(parallel_boards, instance);

		port = pbp->nport;
		for (--port; port >= 0; port--) {
			piop = pbp->piolines[port];

			DBG_CONT("turn off interrupt\n");
			P_DISABLE(piop);

			DBG_CONT("mutex_destroy\n");
			mutex_destroy(&piop->mutex);

			DBG_CONT("cv_destory\n");
			cv_destroy(&piop->cv_busy);

			kmem_free(pbp->piolines[port], sizeof(struct pioline));
		}
		kmem_free(pbp->piolines, (pbp->nport * sizeof(struct pioline *)));

		DBG_CONT("remove_intr\n");
		ddi_remove_intr(pbp->devinfo, 0, pbp->iblock_cookie);

		DBG_CONT("ddi_soft_state_free\n");
		ddi_soft_state_free(parallel_boards, instance);

		DBG_CONT("leaving detach\n");
		return (DDI_SUCCESS);
	default:
                return (DDI_FAILURE);
	}
}

/*
 *  pio_open
 */
static int
pio_open(queue_t *rq, dev_t *dev, int flag, int sflag, cred_t *credp)
{
        register struct parallel_board *pbp;
        register struct pioline *piop;
        int	retval = 0, instance, port;
	u_char  status;

	if (sflag != 0) /* driver does not module or clone open */
		return(ENXIO);

	instance = BOARD(*dev);
	port = PORT(*dev);
	if (port > NPIO_PORTS) {
		return(ENODEV);
	}

	pbp = (struct parallel_board *)ddi_get_soft_state(parallel_boards, instance);
        ASSERT(pbp != (struct parallel_board *)NULL);
	piop = pbp->piolines[port];
	/* Verify instance structure */
	if (piop == NULL)
                return (ENXIO);

        mutex_enter(&piop->mutex);
 
        /* opened with O_NDELAY, then skip checking the printer status */
        if (flag & (FNDELAY | FNONBLOCK)) {
                goto Popen;
        }
 
        /*
         * Fail open until the device is ready and selected unless it's a scanner,
         * in which case the application must clear scanner errors on its own.
         */
        status = *(piop->base + READ_STATUS);
        if ((status & piop->status_mask) != piop->normal_status) {
                printf("PARALLEL DEVICE IS NOT READY -- STATUS <0x%x>\n", status);
		mutex_exit(&piop->mutex);
                return(EIO);
        }
 
Popen:
        piop->unit = *dev;
        piop->ttycommon.t_readq = rq;
        piop->ttycommon.t_writeq = WR(rq);
        rq->q_ptr = WR(rq)->q_ptr = (caddr_t)piop;
 
        if (!(piop->flags & ISOPEN)) {
                piop->flags |= ISOPEN;
                piop->flags &= ~CF_BUSY;
                piop->flags &= ~CF_ERROR;
                piop->lasttxmp = NULL;
                piop->lasttxmp_cnt = 0;
                /* turn slin low */
                *(piop->base + READ_WRITE_CTRL) |= SLIN;
        }
        mutex_exit(&piop->mutex);
        qprocson(rq);
	return(retval);
 
}


static int
pio_close(queue_t *rq, int flag, cred_t *credp)
{
        register struct pioline *piop;
 
	if ((piop = (struct pioline *)rq->q_ptr) == NULL) {
		return(ENODEV);		/* already closed */
	}

	DBG_CONT("inside of pio_close\n");
 
	mutex_enter(&piop->mutex);

	while (piop->lasttxmp != NULL) {
		DBG_CONT("calling cv_wait\n");
		if (cv_wait_sig(&lbolt_cv, &piop->mutex) == 0)
			break;
	}
	DBG_CONT("piop->lasttxmp == NULL\n");

        /* disable & clear the interrupts */
        P_DISABLE(piop);
	CLR_INTR(piop);
 
        ttycommon_close(&piop->ttycommon);
        piop->flags = 0;
        piop->lasttxmp_cnt = 0;
        if (piop->lasttxmp != NULL) {
		piop->lasttxmp = NULL;
	        mutex_exit(&piop->mutex);
		freemsg(piop->lasttxmp);
	} else {
	        mutex_exit(&piop->mutex);
	}
	qprocsoff(rq);
        rq->q_ptr = WR(rq)->q_ptr = NULL;
	piop->ttycommon.t_readq = NULL;
	piop->ttycommon.t_writeq = NULL;
	return(0);
}

/*
 *  pio_wput
 */
static int
pio_wput(queue_t *q, mblk_t *mp)
{
        register struct pioline *piop;
 
        if ((piop = (struct pioline *)q->q_ptr) == NULL) {
                return;
        }
 
	switch (mp->b_datap->db_type) {
	case M_FLUSH:
		if (*mp->b_rptr & FLUSHW) {
			/*
			 * abort any output in progress.  we protect around the flushq
			 * because we could get an txintr that would cause us to
			 * hang
			 */
			mutex_enter(&piop->mutex);
			piop->flags &= ~(CF_BUSY | CF_ERROR);
			flushq(q, FLUSHDATA);
			if (piop->lasttxmp != NULL) {
				piop->lasttxmp_cnt = 0;
				mutex_exit(&piop->mutex);
				freemsg(piop->lasttxmp);
				mutex_enter(&piop->mutex);
				piop->lasttxmp = NULL;
			}
			mutex_exit(&piop->mutex);
			*mp->b_rptr &= ~FLUSHW;
		}

		if (*mp->b_rptr & FLUSHR) {
			flushq(RD(q), FLUSHDATA);
			qreply(q, mp);
		} else
			freemsg(mp);
		/*
		 * We must make sure we process messages that survive the
		 * write-side flush.  Without this call, the close protocol
		 * with ldterm can hang forever.  (ldterm will have sent us a
		 * TCSBRK ioctl that it expects a response to.)
		 */
		pio_start(piop);
		break;
	case M_IOCTL:
		pioioctl(piop, q, mp);
		break;

	case M_DATA:
		DBG_CONT("received M_DATA from wput\n");
		putq(q, mp);
		pio_start(piop);
		break;
 
        default:
		freemsg(mp);
		break;
        }
 
        return(0);
}
 
/*
 *  pio_start
 */
static void
pio_start(register struct pioline *piop)
{
        register mblk_t  *bp, *nbp;
        int     fifocnt;
        u_char  status;
        queue_t *q;

        /* interrupts may be generated from power cycling */
        /* or resetting the scanner. If there's no writeq */
        /* then there's no data to send, so we assume the */
        /* interrupt was a "benign" one...                */
        if ((q = piop->ttycommon.t_writeq) == NULL) {
                P_DISABLE(piop);
                CLR_INTR(piop);
                return;
	}

        if ((piop->flags & (CF_BUSY | CF_ERROR)) != 0) {
                return;
        }
 
	/* clear any pending interrupts */
	status = *(piop->base + READ_STATUS);
        mutex_enter(&piop->mutex);
 
        /* if offline, disable interrupts & flag an error */
        if ((status & PR_OFF_LINE) == 0) {
                /* disable parallel interrupts */
                P_DISABLE(piop);
                piop->flags |= CF_ERROR;
                timeout((void (*)())piorestart, (caddr_t)piop, TICK * 10);
		mutex_exit(&piop->mutex);
                return;
        }
 
        /* no paper, wait to make sure, then re-queue */
        if (status & PE) {
                printf("pio2%d: Printer is out of paper\n", 0);
                P_DISABLE(piop);
                piop->flags |= CF_ERROR;
                timeout((void (*)())piorestart, (caddr_t)piop, TICK * 10);
		mutex_exit(&piop->mutex);
                return;
        }
 
        if ((bp = piop->lasttxmp) == NULL) {
                DBG_CONT("calling getq from pio_start\n");
		bp = getq(q);
                if (bp == NULL) {
			mutex_exit(&piop->mutex);
                        return;
                }
                piop->lasttxmp = bp;
                piop->lasttxmp_cnt = bp->b_wptr - bp->b_rptr;
        }
 
startover:
        switch (bp->b_datap->db_type) {
        case M_DATA:
                piop->flags &= ~CF_ERROR;
                /* enable parallel interrupts */
                P_ENABLE(piop);
         
                fifocnt = min(piop->lasttxmp_cnt, piop->xmit_fifo);
                while (fifocnt != 0) {
                        /*
                         *  this status check will clear any pending interrupts
                         */
                        status = *(piop->base + READ_STATUS);
                        if ((status  & BUSY_MASK) != BUSY_STATUS) {
                                /* send the characters if you can */
                                if ((status & piop->status_mask) == piop->normal_status) {
                                        piop->flags |= CF_BUSY;
 
                                        CLR_INTR(piop);
 
                                        /* load the character into the register */
                                        *(piop->base) = *(bp->b_rptr);
         
                                        /* raise and lower strobe to clock it out */
                                        *(piop->base + READ_WRITE_CTRL) |= ASSERT_STROBE;
                                        *(piop->base + READ_WRITE_CTRL) &= ~ASSERT_STROBE;
         
                                        /* bump the character counters */
                                        bp->b_rptr++;
                                        piop->lasttxmp_cnt--;
                                        fifocnt--;

					/* check for continuations */
                                        if (piop->lasttxmp_cnt == 0) {
                                                nbp = bp;
                                                bp = bp->b_cont;
						freeb(nbp);
                                                if (bp != NULL) {
							piop->lasttxmp_cnt =
								bp->b_wptr - bp->b_rptr;
							fifocnt =
                                                                min(piop->lasttxmp_cnt,
									piop->xmit_fifo);
                                                }
                                                piop->lasttxmp = bp;

						if (piop->lasttxmp == NULL) {
							bp = getq(q);
					                if (bp == NULL) {
								break;
					                }
					                piop->lasttxmp = bp;
							piop->lasttxmp_cnt =
								bp->b_wptr - bp->b_rptr;
							break;
						}
 
					}
                                        continue;
                                }
                                /* if offline, disable interrupts & flag an error */
                                if ((status & PR_OFF_LINE) == 0) {
                                        /* disable parallel interrupts */
                                        P_DISABLE(piop);
                                        piop->flags |= CF_ERROR;
					timeout((void (*)())piorestart, (caddr_t)piop, TICK * 10);
                                        break;
                                }
                                /* no paper, wait to make sure, then re-queue */
                                if (status & PE) {
                                        printf("pio2%d: Printer is out of paper\n", 0);
                                        piop->flags |= CF_ERROR;
					timeout((void (*)())piorestart, (caddr_t)piop, TICK * 10);
                                        break;
                                }
                        } else {
                                if (piop->flags & CF_BUSY) {
                                        /* printer is busy so wait for the ACK to wake us up */
                                        break;
						/*
						cv_wait(&piop->cv_busy, &piop->mutex);
						continue;
						*/
                                } else {
                                        /*
                                         * printer has received ACK signal while it
                                         * was busy so we timeout restart to come back around
                                         */
                                        piop->flags |= CF_ERROR;
					timeout((void (*)())piorestart, (caddr_t)piop, TICK * 1);
					break;
                                }
                        }
                }
                break;
	case M_IOCTL:
                pioioctl(piop, q, bp);
                piop->lasttxmp = NULL;
                piop->lasttxmp_cnt = 0;
                break;
        default:
                piop->lasttxmp = NULL;
                piop->lasttxmp_cnt = 0;
                break;
        }
	mutex_exit(&piop->mutex);
        return;
}

/*
 *  piorestart
 */
static void
piorestart(struct pioline *piop)
{
        mutex_enter(&piop->mutex);
        piop->flags &= ~(CF_ERROR | CF_BREAK);
        mutex_exit(&piop->mutex);
        pio_start(piop);
}

/*
 * pioioctl
 */
static int
pioioctl(struct pioline *piop, queue_t *q, mblk_t  *mp)
{
        struct iocblk *iocp;
        struct gen_para *gcp;
        int     error;
        u_int   datasize;
        u_char  set, set2;
 
	if (piop->ttycommon.t_iocpending != NULL) {
		/*
		 * We were holding an "ioctl" response pending the
		 * availability of an "mblk" to hold data to be passed up;
		 * another "ioctl" came through, which means that "ioctl"
		 * must have timed out or been aborted.
		 */
		freemsg(piop->ttycommon.t_iocpending);
		piop->ttycommon.t_iocpending = NULL;
	}

	/* 1st block contains the iocblk structure */
	iocp = (struct iocblk *)mp->b_rptr;

	mutex_enter(&piop->mutex);

        /*
         *  pass ioctls to ttycommon_ioctl to see if it can handle it.
         *  if it recognizes the cmd it will handle the ACK/NAK itself.
         */
        if ((datasize = ttycommon_ioctl(&piop->ttycommon, q, mp, &error)) != 0) {
                /* if out of buffers, get one and reschedule */
		if (piop->bufcid)
			unbufcall(piop->bufcid);
                piop->bufcid = bufcall(datasize, BPRI_HI, pio_reioctl, (long)piop);
                return;
        }
 
	/*
	 * ttycommon_ioctl didn't recognize it assume it's local
	 * (I_STR) and ttycommon_ioctl set everything up
	 */
	if (error < 0) {
 
                /* The following I_STR ioctls are defined for the
                 * parallel port for 010/210:
                 *      MDRVP_GETPARAM get parallel ioctl lines
                 *      MDRVP_SETPARAM set parallel ioctl lines
                 */
                error = 0;
                switch (iocp->ioc_cmd) {
                case TCSBRK: /* TCSBRK parallel support */
			piop->flags |= CF_BREAK;
			timeout((void (*)())piorestart, (caddr_t)piop, hz / 4);
			break;
 
                case MR_PPSETFIFO: {
			struct pio_fifo	*tunep;
                        /*
                         *  MR_SETFIFO will change the number of chars that we attempt to
                         *  send out in the start routine
                         */
 
                        /* get fifo_data structure from the argument */
                        tunep = (struct pio_fifo *)mp->b_cont->b_rptr;
 
                        /* set xmit FIFO trigger level */
                        if ((tunep->xmit_fifo > 0) && (tunep->xmit_fifo <= PSIZE)) {
                                piop->xmit_fifo = tunep->xmit_fifo;
                        }
                        break;
                        }
			/* read fifo parameters */
		case MRREAD_TUNE: {
			struct pio_fifo *tunep;
			iocp->ioc_count = sizeof(struct gen_para);
			tunep = (struct pio_fifo *)mp->b_cont->b_rptr;

			tunep->xmit_fifo = piop->xmit_fifo;
			tunep->rcv_fifo = -1;
			break;
                        }
			/* ignore Printer Error and Paper Empty */
                case MRIGNORE_PERRS:
                        piop->status_mask = IGNORE_MASK;
                        piop->normal_status = IGNORE_COND;
                        break;
 
                        /* don't ignore Printer Error and Paper Empty */
                case MRIGNORE_RESET:
                        piop->status_mask = NORMAL_MASK;
                        piop->normal_status = NORMAL_COND;
                        break;
 
                        /* get parallel port condition */
                case MDRVP_GETPARAM:
                        iocp->ioc_count = sizeof(struct gen_para);
                        gcp = (struct gen_para *)mp->b_cont->b_rptr;
 
                        gcp->value1 =
                            ((*(piop->base + READ_WRITE_CTRL) << 8) |
                            *(piop->base + READ_STATUS));
 
                        gcp->value2 =
                            ((*(piop->base + READ_WRITE_CTRL) << 8) |
                            *(piop->base + READ_STATUS));
                        gcp->value1 &= gcp->mask;
                        /* raise bits to be raised */
                        if (gcp->mask & PSIG_BUSY)
                                if (gcp->value1 & PSIG_BUSY)
                                        gcp->value1 &= ~(PSIG_BUSY);
                                else
                                        gcp->value1 |= (PSIG_BUSY);
 
                        if (gcp->mask & PSIG_STROBE)
                                if (gcp->value1 & PSIG_STROBE)
                                        gcp->value1 &= ~(PSIG_STROBE);
                                else
                                        gcp->value1 |= (PSIG_STROBE);
 
                        if (gcp->mask & PSIG_AUTO)
                                if (gcp->value1 & PSIG_AUTO)
                                        gcp->value1 &= ~(PSIG_AUTO);
                                else
                                        gcp->value1 |= (PSIG_AUTO);
 
                        if (gcp->mask & PSIG_SELC)
                                if (gcp->value1 & PSIG_SELC)
                                        gcp->value1 &= ~(PSIG_SELC);
                                else
                                        gcp->value1 |= (PSIG_SELC);
 
                        break;
 
                        /************** set parallel port condition */
                case MDRVP_SETPARAM:
                        iocp->ioc_count = 0;
                        gcp = (struct gen_para *)mp->b_cont->b_rptr;
                        set = *(piop->base + READ_WRITE_CTRL);
 
                        /* clear all specified signals */
                        set &= ~(gcp->mask >> 8);
                        set |= PSIG_WMASK;
                        set2 = set;
 
                        /* raise bits to be raised */
                        set |= (gcp->value1 >> 8);
                        if (gcp->mask & PSIG_STROBE) {
                                if (gcp->value1 & PSIG_STROBE)
                                        set &= ~(PSIG_STROBE >> 8);
                                else
                                        set |= (PSIG_STROBE >> 8);
 			}

                        if (gcp->mask & PSIG_AUTO)
                                if (gcp->value1 & PSIG_AUTO)
                                        set &= ~(PSIG_AUTO >> 8);
                                else
                                        set |= (PSIG_AUTO >> 8);
 
                        if (gcp->mask & PSIG_SELC)
                                if (gcp->value1 & PSIG_SELC)
                                        set &= ~(PSIG_SELC >> 8);
                                else
                                        set |= (PSIG_SELC >> 8);
 
                        if (gcp->mask & USE_VALUE2) {
                                /* raise bits to be raised */
                                set2 |= (gcp->value2 >> 8);
                                if (gcp->mask & PSIG_STROBE)
                                        if (gcp->value2 & PSIG_STROBE) {
                                                set2 &= ~(PSIG_STROBE >> 8);
                                        } else {
                                                set2 |= (PSIG_STROBE >> 8);
 					}
                                if (gcp->mask & PSIG_AUTO)
                                        if (gcp->value2 & PSIG_AUTO) {
                                                set2 &= ~(PSIG_AUTO >> 8);
                                        } else {
                                                set2 |= (PSIG_AUTO >> 8);
 					}
                                if (gcp->mask & PSIG_SELC)
                                        if (gcp->value2 & PSIG_SELC) {
                                                set2 &= ~(PSIG_SELC >> 8);
                                        } else {
                                                set2 |= (PSIG_SELC >> 8);
                                        }
                        }
 
                        *(piop->base + READ_WRITE_CTRL) = set;
                        if (gcp->mask & USE_VALUE2)
                                *(piop->base + READ_WRITE_CTRL) = set2;
                        break;
 
                        /* unknown ioctl - NAK the sender */
                default:
                        error = ENOTTY;
                        break;
                }
        }
 
        /* set the message type to ACK or NAK */
        if (error == 0) {
                mp->b_datap->db_type = M_IOCACK;
        } else {
                /* Set the error for the ACK/NAK to the error value. */
                /* This is returned to the caller as errno...        */  
                iocp->ioc_error = error;
                mp->b_datap->db_type = M_IOCNAK;
        }
        qreply(q, mp);
	mutex_exit(&piop->mutex);
        return;
}

/*
 * pio_reioctl
 */
static void 
pio_reioctl(struct pioline *piop)
{
        queue_t  *q;
        mblk_t   *mp;
 
        if ((q = piop->ttycommon.t_writeq) == NULL) {
                return;
        }
        if ((mp = piop->ttycommon.t_iocpending) != NULL) {
                piop->ttycommon.t_iocpending = NULL;
                pioioctl(piop, q, mp);
        }
}

/*
 * pio_intr
 *
 * we know there is an interrupt, now come into this routine to determine
 * what kind of interrupt and dispatch it accordingly
 */
static u_int       
pio_intr(caddr_t instance)
{
        register struct pioline	*piop;
	struct parallel_board	*pbp;
	register int	port;
        int 	serviced = DDI_INTR_UNCLAIMED;
        u_char	status;
 
	pbp = (struct parallel_board *)ddi_get_soft_state(parallel_boards, (u_int)instance);

	port = pbp->nport;
	for (--port; port >= 0; port--) {
		piop = pbp->piolines[port];

		if (piop->board_ptr->bcsr != NULL) {
			status = *piop->board_ptr->bcsr;

			if (status & BCSR_PINT) {
				serviced = DDI_INTR_CLAIMED;
				P_DISABLE(piop);
				status = *(piop->base + READ_STATUS);
				CLR_INTR(piop);
				piop->flags &= ~CF_BUSY;
				pio_start(piop);
			}
		} else if ((*(piop->base + READ_STATUS) & PIRQ) == 0) {
			P_DISABLE(piop);
			CLR_INTR(piop);
			piop->flags &= ~CF_BUSY;
			pio_start(piop);
			serviced = DDI_INTR_CLAIMED;
/*
			mutex_enter(&piop->mutex);
			cv_signal(&piop->cv_busy);
			mutex_exit(&piop->mutex);
*/
		}
	}
	return(serviced);
}	
