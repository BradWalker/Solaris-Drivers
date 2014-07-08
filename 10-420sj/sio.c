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


	file: sio.c
	author: bwalker
	created: 7/16/92
	updated: jmartin 5/19/1993
	sccs info: %W% %G% %R%

	1990 - 1992 (c) Aurora Technologies, Inc. All Rights Reserved.
*/
static char CopyId[] = "@(#)Copyright (c) 1992 INSERT_HERE_COPYR_NAME";
static char SccsId[] = "@(#)INSERT_HERE_CO_NAME, %W% %G% %R% [sio Version INSERT_HERE_VERSION]";

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
#include <sys/mkdev.h>
#include <sys/obpdefs.h>

#include <sys/ddi.h>
#include <sys/sunddi.h>

#include "sio.h"
#include "xxtrace.h"
#include "event.h"
 
extern kcondvar_t lbolt_cv;

/*
 *  debug flags used during attach, modload, & and other routines
 *  until xxtraceinit() gets called
 */
static int sio_debug = 0;
#define DBG_CONT(x)             if (sio_debug) \
					cmn_err(CE_CONT, x);
#define DBG_WARN(x)             if (sio_debug) \
					cmn_err(CE_WARN, x);

/* send a break char */
#define SENDBRK(base)		(*((u_char *)((base) + LCR)) |= SNDBRK)

/* turn off sending of break char */
#define ENDBRK(base)		(*((u_char *)((base) + LCR)) &= ~SNDBRK)

/* send a character out the port */
#define TRANSMIT(base, char)	(*base = char) 

/* declaration of serial port routines */
static int	sio_identify(dev_info_t *);
static int	sio_attach(dev_info_t *, ddi_attach_cmd_t);

static int	sio_detach(dev_info_t *, ddi_detach_cmd_t );
static int	sio_info(dev_info_t *, ddi_info_cmd_t, void *, void **);

/* open & setup the port */
static int	sio_open(queue_t *, dev_t *, int, int, cred_t *);

/* cleanup & shut down port */
static int	sio_close(queue_t *, int, cred_t *);

static int	sio_wput(queue_t *, mblk_t *);		/* write put routine */
static int	sio_rsrv(queue_t *);			/* read service */
static void	sio_ioctl(struct sioline *, queue_t *, mblk_t *);
static void	sio_reioctl(struct sioline *);

static void	sio_start(struct sioline *);
static void	sio_restart(struct sioline *);
static void	sio_param(struct sioline *);
static void	sio_stash(struct sioline *, u_char, int);
static void	sio_pushdb(struct sioline *);
static void	sio_setdtrrts(struct sioline *, u_char);

/* driver interrupt routines */
static u_int	sio_intr(caddr_t);			/* what intr was received */

static void	sio_rxintr(struct sioline *);		/* receive */
static void	sio_rsintr(struct sioline *);		/* line status */
static void	sio_txintr(struct sioline *);		/* transmit & FIFO */
static void	sio_mintr(struct sioline *);		/* modem */
static void	sio_rxtimeout(struct sioline *);	/* receive timeout */

#ifdef XXTRACE
static int	xxtraceinit_flag = 0;			/* xxtraceinit called */
#endif

static int	hold_dtr_low = 3;		/* secs to hold DTR low */
static int	trycanput = 70;			/* number of canput failures before dropping data */

/*
 * We need to keep track of the highest priority interrupting device
 * so that we can pass the iblock cookie to xxtrace.
 */
static ddi_iblock_cookie_t * iblock = NULL;
static int maxprio = 0;

/*
 * an array of pointers that is allocated in sioattach()
 * each structure describes a line
 */
static void *serial_boards;

/*
 * baud rate register settings.  used to setup baud rates on the
 * chip.  assumes a 1.8432 Mhz clock on the board
 */
static u_int sio_baud[] = {
	0,		/* B0 */
	2304,		/* B50 */
	1536,		/* B75 */
	1047,		/* B110 */
	857,		/* B134.5 */
	768,		/* B150 */
	576,		/* B200 */
	384,		/* B300 */
	192,		/* B600 */
	96,		/* B1200 */
	64,		/* B1800 */
	48,		/* B2400 */
	24,		/* B4800 */
	12,		/* B9600 */
	6,		/* B19200 */
	3,		/* B38400 */
	2		/* B56000 */
};

static struct module_info sio_mod_info = {
	0,
	"sio",
	0,
	INFPSZ,
	4096,
	512
};

/* Read queue */
static struct qinit sio_rinit = {
	putq,
	sio_rsrv,
	sio_open,
	sio_close,
	NULL,
	&sio_mod_info,
	NULL
};

/* Write queue */
static struct qinit sio_winit = {
	sio_wput,
	NULL,
	NULL,
	NULL,
	NULL,
	&sio_mod_info,
	NULL
};

struct streamtab sio_stab = {
	&sio_rinit,
	&sio_winit,
	NULL,
	NULL,
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
DDI_DEFINE_STREAM_OPS(	sio_ops, \
			sio_identify, \
			nulldev, \
			sio_attach, \
			nulldev, \
			nodev, \
			sio_info, \
			D_NEW | D_MP, \
			&sio_stab);
 
extern  struct  mod_ops mod_driverops;
static  struct modldrv modldrv = {
        &mod_driverops,
#ifdef BOARDSJ
	"Aurora sio serial device driver",
#endif
#ifdef BOARDS
	"Aurora xsio serial device driver",
#endif
        &sio_ops,
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
	error = ddi_soft_state_init(&serial_boards, sizeof(struct serial_board), 0);
	if (error != 0)
		return(error);

	if ((error = mod_install(&modlinkage)) != 0)
		ddi_soft_state_fini(&serial_boards);

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

	ddi_soft_state_fini(&serial_boards);

	return(0);
}

/*
 * This is a pretty generic getinfo routine as describe in the manual.
 */
/*ARGSUSED*/
static int
sio_info(dev_info_t *dip, ddi_info_cmd_t infocmd, void *arg, void **result)
{
        register struct serial_board *sbp;
        int error, instance;

        switch (infocmd) {
        case DDI_INFO_DEVT2DEVINFO:
		instance = BOARD((dev_t)arg);
		sbp = (struct serial_board *)ddi_get_soft_state(serial_boards, instance);
		if (sbp == NULL) {
                        *result = NULL;
                        error = DDI_FAILURE;
                } else {
                        *result = (void *)sbp->devinfo;
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
 * When our driver is loaded, sio_identify() is called with a dev_info_t
 * full of information from the FCode on our board.
 *
 * in 5.0, identify() will only be called for devices whose
 * FCode "name" property matches the name of the driver (e.g. name: "pio",
 * driver source: pio.c, driver module: /kernel/drv/pio).  You can get
 * around this by editing /etc/driver_aliases or by using the "-i" option
 * for add_drv(1).  (e.g. name: "Aurora-210SJ-", driver module: /kernel/drv/pio.)
 */
static int	
sio_identify(dev_info_t *dip)
{
	char *dev_name;

	dev_name = ddi_get_name(dip);

#ifdef BOARDSJ
	if ((strcmp(dev_name, "sio4") ==0) ||
	    (strcmp(dev_name, "sio2") == 0)) {
#endif
#ifdef BOARDS
	if (strcmp(dev_name, "xsio") == 0) {
#endif
		return (DDI_IDENTIFIED);
	} else {
		return (DDI_NOT_IDENTIFIED);
	}
}
 
/*
 * sio_attach gets called if sio_identify returns DDI_IDENTIFIED
 */
static int
sio_attach(dev_info_t *dip, ddi_attach_cmd_t cmd)
{
	struct sioline *siop;
	struct serial_board *sio_board_ptr;
	struct sioline **serial_lines;
	int	bintr, nports, reg_size;
        int	instance, cur_port;
	u_char	status;
	char	*name;
	char	port_name[32];

	if (cmd != DDI_ATTACH)
		return (DDI_FAILURE);

	instance = ddi_get_instance(dip);
	name = ddi_get_name(dip);
	sprintf(port_name, "%s", name);

	if (ddi_soft_state_zalloc(serial_boards, instance) != DDI_SUCCESS)
		return (DDI_FAILURE);

#ifdef BOARDSJ
	if (strcmp(name, "sio4") == 0) {
		nports = 4;
	} else if (strcmp(name, "sio2") == 0) {
#endif
#ifdef BOARDS
	if (strcmp(name, "xsio") == 0) {
#endif
		nports = 2;
	}

	sio_board_ptr = (struct serial_board *)ddi_get_soft_state(serial_boards, instance);

	serial_lines = (struct sioline **)kmem_zalloc((nports *sizeof(struct sioline *)), KM_NOSLEEP);
 	if (serial_lines == (struct sioline **)NULL) {
		cmn_err(CE_WARN, "kmem_zalloc: failed for serial_lines\n");
		goto attach_failed;
	}
	sio_board_ptr->siolines = serial_lines;
	sio_board_ptr->nport = nports;
	sio_board_ptr->devinfo = dip;

	DBG_CONT("setup add_intr\n");
	/* put this board onto the interrupt vector list */
	if (ddi_add_intr(dip, 0, &(sio_board_ptr->iblock_cookie),
		&(sio_board_ptr->idevice_cookie), sio_intr,
		(caddr_t)instance) != DDI_SUCCESS) {
			return (DDI_FAILURE);
	}

	/*
	 * THE REGISTERS LOOK OK, SO LET'S MAP THEM IN SO WE CAN USE THEM
	 */
	if (strcmp(name, "sio2") == 0) {
		/* we need to map in BCSR registers */
		reg_size = ddi_dev_regsize(dip, BCSR_REG, (off_t *)&reg_size);
		if (ddi_map_regs(dip, BCSR_REG, (caddr_t*)&(sio_board_ptr->bcsr),
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
		sio_board_ptr->bintr = bintr;
/*
		status = *sio_board_ptr->bcsr;
		*sio_board_ptr->bcsr = ((status & 0x0f) | bintr) & ~LPTOE;
*/
		status = *sio_board_ptr->bcsr;
		*sio_board_ptr->bcsr = ((status & 0x0f) | bintr) | UART0_INT_ENA | UART1_INT_ENA;
	} else {
		sio_board_ptr->bintr = 0;
		sio_board_ptr->bcsr = NULL;
	}

	for (cur_port = 0; cur_port < nports; cur_port++) {
		serial_lines[cur_port] =
			(struct sioline *)kmem_zalloc(sizeof(struct sioline), KM_NOSLEEP);
		if (serial_lines[cur_port] == (struct sioline *)NULL) {
			cmn_err(CE_WARN, "kmem_zalloc: failed serial_lines[%d]\n", cur_port);
			goto attach_failed;
		}
		siop = serial_lines[cur_port];

		DBG_CONT("setup mutex_init\n");
		/*
		 * initialize MUTEX; we will use this MUTEX later on to lock
		 * our instance structure
		 */
		mutex_init(&siop->sio_mutex, "sio mutex", MUTEX_DRIVER,
				(void*)&(sio_board_ptr->iblock_cookie));

		DBG_CONT("setup condition var.\n");
		/*
		 * initialize cond. var.; we will use this later on to wait for
		 * the login to be awoken
		 */
	        cv_init(&siop->login_cv, "sio_login_cv", CV_DRIVER,
				(void*)&(sio_board_ptr->iblock_cookie));

		reg_size = ddi_dev_regsize(dip, cur_port, (off_t*)&reg_size);
		/*
		 * the registers look OK, so let's map them in so we can use them
		 */
		if INSERT_HERE_NODE_NAME_SIO {
			if (ddi_map_regs(dip, PORT_REG_OFFSET + cur_port,
				(caddr_t *)&(siop->base), 0, reg_size) != DDI_SUCCESS) {
					goto attach_failed;
			}
		} else if (ddi_map_regs(dip, cur_port, (caddr_t *)&(siop->base),
						0, reg_size) != DDI_SUCCESS) {
				goto attach_failed;
		}
 
		/*
		 * ddi_create_minor_node creates an entry in an internal kernel
		 * table; the actual entry in the file system is created by
		 * drvconfig(1) when you run add_drv(1);
		 */
		sprintf(port_name, "%c", ('a' + cur_port));
		if (ddi_create_minor_node(dip, port_name,
			S_IFCHR,((instance << 3) + cur_port),
			DDI_NT_SERIAL, NULL) == DDI_FAILURE) {
			cmn_err(CE_WARN, "ddi_create_minor_node failed\n");
			goto attach_failed;
		}

		sprintf(port_name, "%c,cu", ('a' + cur_port));
		if (ddi_create_minor_node(dip, port_name,
			S_IFCHR, ((instance << 3) + cur_port) | OUTLINE,
			DDI_NT_SERIAL_DO, NULL) == DDI_FAILURE) {
			cmn_err(CE_WARN, "ddi_create_minor_node failed\n");
			goto attach_failed;
		}

		/* setup the default chars that we try to send out */
		siop->xmit_fifo = 16;

		/*
		 * setup the chip
		 *
		 * init the MCR
		 * init and enable FIFO
		 * set-up what interrupt priority the port is at
		 *
		 */

		*(siop->base + MCR) = ~(DTR | RTS);
		*(siop->base + FCR) = (FIFO_INIT);

		/* Check type of chip present (452 or 552) */
		status = *(siop->base + ISR);
		if (status & FIFO_PRESENT) {
			siop->fifo |= CF_FIFO_ON;
			siop->rcv_fifo = (FIFO_ENABLE | FIFO_TRIG_8);
			siop->xmit_fifo = FIFO_DPTH;
		} else {
			siop->fifo &= ~CF_FIFO_ON;
			siop->rcv_fifo = (u_char)-1;
			siop->xmit_fifo = 1;
		}
	}

/*
	ddi_set_driver_private(dip, (caddr_t)serial_lines);
*/

#ifdef XXTRACE
	xxtraceinit_bitmap(instance, nports);
#endif

	DBG_CONT("finish setting serial addresses\n");
	ddi_report_dev(dip);
	return (0);

attach_failed:
        /*
         * need to add the code here to kmem free any allocated data
structures
         */
        cmn_err(CE_WARN, "pioattach: failed!\n");
        return (DDI_FAILURE); 
}
 
/*
 * When our driver is unloaded, sio_detach cleans up and frees the resources
 * we allocated in sio_attach.
 */
static int sio_detach(dev_info_t *dip, ddi_detach_cmd_t cmd) {
        struct serial_board *sbp;
	struct sioline *siop;
	int	instance, port;
	u_char	status, temp;

	DBG_CONT("inside of detach\n");
	switch(cmd) {
	case DDI_DETACH:
		instance = ddi_get_instance(dip);
		sbp = (struct serial_board *)ddi_get_soft_state(serial_boards, instance);

		if (sbp->bcsr != NULL) {
			status = *sbp->bcsr;
                	*sbp->bcsr = ((status & 0x0f) | sbp->bintr) &
					~(UART0_INT_ENA | UART1_INT_ENA);
		}

		for (port = sbp->nport; port >= 0; port--) {
			siop = sbp->siolines[port];

			/* turn off interrupts */
			*(siop->base + IER) = 0;

			/* read the registers to reset them */
			temp = *(siop->base);
			temp = *(siop->base + ISR);
			temp = *(siop->base + LSR);
			temp = *(siop->base + MSR);

			/* free the mutex */
			mutex_destroy(&siop->sio_mutex);

			/* free the condition variable */
			cv_destroy(&siop->login_cv);

			kmem_free(sbp->siolines[port], sizeof(struct sioline));
		}
		kmem_free(sbp->siolines, (sbp->nport * sizeof(struct sioline *)));

#ifdef XXTRACE
		if (xxtraceinit_flag)
			xxtraceunload();
#endif

		ddi_remove_intr(sbp->devinfo, 0, sbp->iblock_cookie);

		ddi_soft_state_free(serial_boards, instance);

		DBG_CONT("leaving detach\n");
		return (DDI_SUCCESS);
	default:
                return (DDI_FAILURE);
	}
}

/*
 * sio_open
 */
static int
sio_open(queue_t *rq, dev_t *dev, int flag, int sflag, cred_t *credp)
{
	register struct serial_board *sbp;
	register struct sioline	*siop;
	u_char	dcd, msr;
	u_char	temp;
	int	instance;

	/*
	 * initialize xxtrace utility
	 */
#ifdef XXTRACE
	if (! xxtraceinit_flag ) {
		xxtraceinit(iblock);
		xxtraceinit_flag = 1;
	}
#endif

	/* driver does not module or clone open */
	if (sflag != 0) {
		XT(OPEN_L1_EXIT, ND, __LINE__, 0x0, 0x0);
		return(ENXIO);
	}

	XT(OPEN_L1_ENTRY, ND, rq, flag, sflag);
	instance = BOARD(*dev);
	sbp = (struct serial_board *)ddi_get_soft_state(serial_boards, instance);
	ASSERT(sbp != (struct serial_board *)NULL);
	siop = sbp->siolines[PORT(*dev)];

	/*
         * Verify instance structure
         */
        if (siop == NULL) {
		XT(OPEN_L1_EXIT, ND, __LINE__, 0x0, 0x0);
                return (ENXIO);
	}

	mutex_enter(&siop->sio_mutex);
again:
	if (!(siop->flags & ISOPEN)) {
		XT(OPEN_L2_1STOPEN, siop, 0x0, 0x0, 0x0);

		/* turn off FIFO_SAVE - this is still under development */
		siop->flags &= ~FIFO_SAVE;

		siop->trycanput = 0;
		siop->dev = *dev;
		siop->ttycommon.t_iflag = IFLAGS;
		siop->ttycommon.t_cflag = (ISPEED << IBSHIFT) | ISPEED | CFLAGS;
		siop->ttycommon.t_stopc = CSTOP;
		siop->ttycommon.t_startc = CSTART;
		siop->ttycommon.t_iocpending = NULL;
		siop->ttycommon.t_size.ws_row = 0;
		siop->ttycommon.t_size.ws_col = 0;
		siop->ttycommon.t_size.ws_xpixel = 0;
		siop->ttycommon.t_size.ws_ypixel = 0;

		/*
		 * make sure that the registers are cleared
		 */
		temp = *siop->base;
		temp = *(siop->base + ISR);
		temp = *(siop->base + LSR);
		temp = *(siop->base + MSR);

		sio_param(siop);

		/* turn on the fifo */
		if (siop->fifo & CF_FIFO_ON) {
			*(siop->base + FCR) = (siop->rcv_fifo);
			XT(OPEN_L2_FIFO, siop, *(siop->base + FCR), 0x0, 0x0);
		}
		*(siop->base + IER) = M_en;
	} else if (siop->ttycommon.t_flags & TS_XCLUDE && !drv_priv(credp)) {
		XT(OPEN_L2_EXCLUDE, siop, 0x0, 0x0, 0x0);
		mutex_exit(&siop->sio_mutex);
		XT(OPEN_L1_EXIT, siop, __LINE__, 0x0, 0x0);
		return (EBUSY);
	} else if (ISOUT(*dev) && !(siop->flags & CF_OUT)) {
		mutex_exit(&siop->sio_mutex);
		XT(OPEN_L1_EXIT, siop, __LINE__, 0x0, 0x0);
		return (EBUSY);
	}

	siop->flags |= CF_WOPEN;
	/* raise dtr */
	XT(OPEN_L2_DTR, siop, 0x0, 0x0, 0x0);

	siop->softmcr = OUT2;
	sio_setdtrrts(siop, (DTR | RTS));
	
	/* read the modem control lines */
	msr = *(siop->base + MSR);
	XT(OPEN_L2_MSR, siop, msr, 0x0, 0x0);
	dcd = msr & DCD;

	if (ISOUT(*dev))
		siop->flags |= (CF_OUT | CF_CARR_ON);
	else if (dcd || (siop->ttycommon.t_flags & TS_SOFTCAR))
		siop->flags |= CF_CARR_ON;

	if (!(flag & (FNDELAY|FNONBLOCK)) &&
	    !(siop->ttycommon.t_cflag & CLOCAL)) {
		if (!(siop->flags & CF_CARR_ON) ||
		    ((siop->flags & CF_OUT) && !ISOUT(*dev))) {
			siop->flags |= CF_WOPEN;
			XT(OPEN_L2_SLEEP_CD, siop, 0x0, 0x0, 0x0);
			if (cv_wait_sig(&siop->login_cv, &siop->sio_mutex) == 0) {
				XT(OPEN_L2_SLEEP_INT, siop, __LINE__, 0x0, 0x0);
				if ((siop->flags & ISOPEN) == 0) {
					sio_setdtrrts(siop, ~(DTR | RTS));
					*(siop->base + IER) = 0;
				}
				siop->flags &= ~CF_WOPEN;
				mutex_exit(&siop->sio_mutex);
				XT(OPEN_L1_EXIT, siop, __LINE__, 0x0, 0x0);
				return (EINTR);
			}
			XT(OPEN_L2_WAKEUP_CD, siop, 0x0, 0x0, 0x0);
			goto again;
		}
	} else if ((siop->flags & CF_OUT) && !ISOUT(*dev)) {
		mutex_exit(&siop->sio_mutex);
		XT(OPEN_L1_EXIT, siop, __LINE__, 0x0, 0x0);
		return (EBUSY);
	}

	siop->ttycommon.t_readq = rq;
	siop->ttycommon.t_writeq = WR(rq);
	rq->q_ptr = WR(rq)->q_ptr = (caddr_t)siop;
	siop->flags &= ~CF_WOPEN;
	siop->flags |= ISOPEN;
	*(siop->base + IER) = (Tx_en | Rx_en | M_en | Rs_en);
	mutex_exit(&siop->sio_mutex);
	qprocson(rq);
	XT(OPEN_L1_EXIT, siop, __LINE__, 0x0, 0x0);
	return(0);
}


/*
 * sio_close
 */
static int
sio_close(queue_t *rq, int flag, cred_t *credp)
{
	register struct sioline *siop;
	u_char	temp;

	if ((siop = (struct sioline *)rq->q_ptr) == NULL) {
		XT(CLOSE_L1_EXIT, ND, __LINE__, 0x0, 0x0);
		return(ENODEV);		/* already closed */
	}
	XT(CLOSE_L1_ENTRY, siop, rq, flag, 0x0);

	mutex_enter(&siop->sio_mutex);

	XT(CLOSE_L2_SLEEP, siop, siop->lasttxmp, siop->lastrxmp, siop->flags);
	while ((siop->flags & (CF_BREAK | CF_DELAY)) ||
	       (siop->lasttxmp != NULL) || (siop->lastrxmp != NULL))
		if (cv_wait_sig(&lbolt_cv, &siop->sio_mutex) == 0)
			break;

	XT(CLOSE_L2_WAKEUP, siop, siop->lasttxmp, siop->lastrxmp, siop->flags);

	/* disable interrupts from this uart */
	*(siop->base + IER) = 0;

	/* read the registers to reset them */
	temp = *(siop->base);
	temp = *(siop->base + ISR);
	temp = *(siop->base + LSR);
	temp = *(siop->base + MSR);

	/*
	 * remove sio_pushdb to prevent sio_stash
	 * from being called
	 */
	(void)untimeout(siop->tid);

	if ((siop->flags & CF_WOPEN) ||
	    (siop->ttycommon.t_cflag & HUPCL) ||
	    !(siop->flags & ISOPEN)) {
		/* Drop dtr */
		XT(CLOSE_L2_DTR, siop, 0x0, 0x0, 0x0);
		sio_setdtrrts(siop, ~(DTR | RTS));
	}

	siop->flags = 0;
	ttycommon_close(&siop->ttycommon);
	cv_broadcast(&siop->login_cv);
	mutex_exit(&siop->sio_mutex);
	qprocsoff(rq);

	if(siop->lasttxmp != NULL) {
		freemsg(siop->lasttxmp);
		siop->lasttxmp = NULL;
	}

	if (siop->lastrxmp != NULL) {
		freemsg(siop->lastrxmp);
		siop->lastrxmp = NULL;
	}

	rq->q_ptr = WR(rq)->q_ptr = NULL;
	siop->ttycommon.t_readq = NULL;
	siop->ttycommon.t_writeq = NULL;

	XT(CLOSE_L1_EXIT, siop, __LINE__, 0x0, 0x0)

	return(0); 
}

/*
 * sio_wput
 */
static int
sio_wput(queue_t *q, mblk_t *mp)
{
	register struct sioline	*siop;

	siop = (struct sioline *)q->q_ptr;

	XT(WPUT_L1_ENTRY, siop, q, mp, 0x0);

	switch (mp->b_datap->db_type) {
	case M_STOP:	/* We received ^S */
		XT(WPUT_L2_M_STOP, siop, 0x0, 0x0, 0x0);
		mutex_enter(&siop->sio_mutex);
		siop->flags |= OSTOPPED;
		mutex_exit(&siop->sio_mutex);
		freemsg(mp);
		break;
	case M_START:	/* We received ^Q */
		XT(WPUT_L2_M_START, siop, 0x0, 0x0, 0x0);
		if (siop->flags & OSTOPPED) {
			mutex_enter(&siop->sio_mutex);
			siop->flags &= ~OSTOPPED;
			mutex_exit(&siop->sio_mutex);
			sio_start(siop);
		}
		freemsg(mp);
		break;
	case M_IOCTL:
		XT(WPUT_L2_M_IOCTL, siop, 0x0, 0x0, 0x0);
		switch (((struct iocblk *)mp->b_rptr)->ioc_cmd) {
		case TCSETSW:
		case TCSETSF:
		case TCSETAW:
		case TCSETAF:
		case TCSBRK:
			/*
			 * The changes do not take effect until all output has
			 * queued up has drained. Put this message on the queue
			 * so that sio_start will it when done with the output
			 * before it
			 */
			putq(q, mp);
			sio_start(siop);
			break;
		default:
			/*
			 * Do it now!
			 */
			XT(WPUT_L2_IOCTL, siop, ((struct iocblk *)mp->b_rptr)->ioc_cmd, 0x0, 0x0);
			sio_ioctl(siop, q, mp);
			break;
		}
		break;
	case M_FLUSH:
		XT(WPUT_L2_M_FLUSH, siop, 0x0, 0x0, 0x0);
		if (*mp->b_rptr & FLUSHW) {
			mutex_enter(&siop->sio_mutex);
			flushq(q, FLUSHDATA);
			if (siop->flags & CF_BUSY)
				siop->flags &= ~CF_BUSY;
			if (siop->lasttxmp != NULL) {
				freemsg(siop->lasttxmp);
				siop->lasttxmp = NULL;
			}
			mutex_exit(&siop->sio_mutex);
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
		sio_start(siop);
		break;	
	case M_STOPI:	/* Send ^S */
		XT(WPUT_L2_M_STOPI, siop, 0x0, 0x0, 0x0);
		if (siop->ttycommon.t_cflag & CRTSCTS) {
			mutex_enter(&siop->sio_mutex);
			siop->flags |= INPUT_STOP;
			/* lower RTS */
			siop->softmcr &= ~RTS;
			*(siop->base + MCR) = siop->softmcr;
			mutex_exit(&siop->sio_mutex);
		}

                if (siop->ttycommon.t_iflag & IXOFF) {
			mutex_enter(&siop->sio_mutex);
			siop->flags |= INPUT_STOP;
			/* send flow control char */
			siop->flow = siop->ttycommon.t_stopc;
			mutex_exit(&siop->sio_mutex);
			sio_start(siop);
		}
		freemsg(mp);
		break;
	case M_STARTI:	/* Send ^q */
		XT(WPUT_L2_M_STARTI, siop, 0x0, 0x0, 0x0);
		mutex_enter(&siop->sio_mutex);
		if (siop->flags & INPUT_STOP) {
			siop->flags &= ~INPUT_STOP;
			if (siop->ttycommon.t_cflag & CRTSCTS) {
				/* raise RTS */
				siop->softmcr |= RTS;
				*(siop->base + MCR) = siop->softmcr;
			}

			if (siop->ttycommon.t_iflag & IXOFF) {
				/* send flow control char */
				siop->flow = siop->ttycommon.t_startc;
				mutex_exit(&siop->sio_mutex);
				sio_start(siop);
				mutex_enter(&siop->sio_mutex);
			}
		}
		mutex_exit(&siop->sio_mutex);
		freemsg(mp);
		break;
	case M_BREAK:
	case M_DELAY:
	case M_DATA:
		XT(WPUT_L2_M_DATA, siop, 0x0, 0x0, 0x0);
		putq(q, mp);
		sio_start(siop);
		break;
	default:
		XT(WPUT_L2_UNKNOWN, siop, mp->b_datap->db_type, 0x0, 0x0);
		freemsg(mp);
		break;
	}
	XT(WPUT_L1_EXIT, siop, __LINE__, 0x0, 0x0);
	return(0);
}

/*
 * sio_rsrv
 */
static int
sio_rsrv(queue_t *q)
{
	register mblk_t	 *mp;

	XT(RSRV_L1_ENTRY, ND, q, 0x0, 0x0);

	while ((mp = getq(q)) != NULL) {
		if (canput(q->q_next)) {
			XT(RSRV_L2_CANPUT, ND, 0x0, 0x0, 0x0);
			putnext(q, mp);
		} else {
			XT(RSRV_L2_CANTPUT, ND, 0x0, 0x0, 0x0);
			noenable(q);
			putbq (q, mp);
			enableok(q);
			break;
		}
	}
	XT(RSRV_L1_EXIT, ND, __LINE__, 0x0, 0x0);
	return(0);
}

/*
 * sio_ioctl routine
 */
static void
sio_ioctl(struct sioline *siop, queue_t *wq, mblk_t *mp)
{
	u_int 		datasize;
	struct iocblk *iocp;
	struct fifo_data *tunep;
	int	error;

	XT(IOCTL_L1_ENTRY, siop, siop, wq, mp);

	if (siop->ttycommon.t_iocpending != NULL) {
		/*
		 * We were holding an "ioctl" response pending the
		 * availability of an "mblk" to hold data to be passed up;
		 * another "ioctl" came through, which means that "ioctl"
		 * must have timed out or been aborted.
		 */
		freemsg(siop->ttycommon.t_iocpending);
		siop->ttycommon.t_iocpending = NULL;
	}

	iocp = (struct iocblk *)mp->b_rptr;

	/*
	 * The only way in which "ttycommon_ioctl" can fail is if the "ioctl"
	 * requires a response containing data to be returned to the user,
	 * and no mblk could be allocated for the data.
	 * No such "ioctl" alters our state.  Thus, we always go ahead and
	 * do any state-changes the "ioctl" calls for.  If we couldn't allocate
	 * the data, "ttycommon_ioctl" has stashed the "ioctl" away safely, so
	 * we just call "bufcall" to request that we be called back when we
	 * stand a better chance of allocating the data.
	 */
	if ((datasize = ttycommon_ioctl(&siop->ttycommon, wq, mp, &error)) != 0) {
		if (siop->bufcid)
			unbufcall(siop->bufcid);
		siop->bufcid = bufcall(datasize, BPRI_HI, sio_reioctl, (long)siop);
		XT(IOCTL_L1_EXIT, siop, __LINE__, 0x0, 0x0);
		return;
	}

	if (error == 0) {
		/*
		 * "ttycommon_ioctl" did most of the work; we just use the
		 * data it set up.
		 */
		switch (iocp->ioc_cmd) {
		case TCSETS:
		case TCSETSW:
		case TCSETSF:
		case TCSETA:
		case TCSETAW:
		case TCSETAF:
			XT(IOCTL_L2_TCSET, siop, iocp->ioc_cmd, 0x0, 0x0);
			sio_param(siop);
			break;
		}
	} else if (error < 0) {
		/*
		 * "ttycommon_ioctl" didn't do anything; we process it here.
		 */
		error = 0;
		switch (iocp->ioc_cmd) {
#if 0
		case TIOCCONS:
                        /*
                         * This ioctl is still accepted for the sake of
                         * compatibility with old releases, but it no longer
                         * does anything.
                         */
			break;
#endif

		case TCSBRK:
			XT(IOCTL_L2_TCSBRK, siop, *((int *)mp->b_cont->b_rptr), 0x0, 0x0);
			if (*((int *)mp->b_cont->b_rptr) == 0) {
				SENDBRK(siop->base);
				(void)timeout((void (*)())sio_restart, (caddr_t)siop, hz/4);
				mutex_enter(&siop->sio_mutex);
				siop->flags |= CF_BREAK;
				mutex_exit(&siop->sio_mutex);
			}
			break;
		case TIOCSBRK:
			XT(IOCTL_L2_TIOCSBRK, siop, 0x0, 0x0, 0x0);
			mutex_enter(&siop->sio_mutex);
			SENDBRK(siop->base);
			siop->flags |= CF_BREAK;
			mutex_exit(&siop->sio_mutex);
			break;
		case MR_SETFIFO: {
			/*
			 *  MR_SETFIFO will change the RCVR fifo interrupt trigger
			 *  level for the channel.  This can be done through
			 *  the "fifo_trigger=n" ioctl call from mr_tune's mset.
			 *
			 *  Following is a reference for a 255 byte buffer:
			 *	fifo_trigger == 1,  translates to 900 interrupt/sec
			 *	fifo_trigger == 4,  translates to 250 interrupt/sec
			 *	fifo_trigger == 8,  translates to 160 interrupt/sec
			 *	fifo_trigger == 14, translates to 90  interrupt/sec
			 */
				/* get fifo_data structure from the argument */
				tunep = (struct fifo_data *)mp->b_cont->b_rptr;

 				mutex_enter(&siop->sio_mutex);;
				/* set xmit FIFO trigger level */
				if ((tunep->xmit_fifo > 0) && (tunep->xmit_fifo <= FIFO_DPTH)) {
					siop->xmit_fifo = (u_char)tunep->xmit_fifo;
				}

				/* set xmit FIFO trigger level */
				if ((tunep->rcv_fifo > 0) && (tunep->rcv_fifo <= FIFO_DPTH)) {
					/* set recv FIFO trigger level */
					switch (tunep->rcv_fifo) {
					case 1: siop->rcv_fifo = FIFO_TRIG_1;
						break;
					case 4: siop->rcv_fifo = FIFO_TRIG_4;
						break;
					case 8: siop->rcv_fifo = FIFO_TRIG_8;
						break;
					case 14: siop->rcv_fifo = FIFO_TRIG_14;
						 break;
					default: error = EPERM;
						 break;
					}
					siop->rcv_fifo |= FIFO_ENABLE;
					*(siop->base + FCR) = siop->rcv_fifo;
				}
				mutex_exit(&siop->sio_mutex);
				XT(IOCTL_L2_MR_SETFIFO, siop, siop->rcv_fifo, siop->xmit_fifo, 0x0);
				break;
				}
		case MRREAD_TUNE:
				XT(IOCTL_L2_MRREAD_TUNE, siop, 0x0, 0x0, 0x0);
				iocp->ioc_count = sizeof(struct fifo_data);
				tunep = (struct fifo_data *) mp->b_cont->b_rptr;

				tunep->xmit_fifo = siop->xmit_fifo;

				switch (siop->rcv_fifo & ~FIFO_ENABLE) {
				case FIFO_TRIG_1:
					tunep->rcv_fifo = 1;
					break;
				case FIFO_TRIG_4:
					tunep->rcv_fifo = 4;
					break;
				case FIFO_TRIG_8:
					tunep->rcv_fifo = 8;
					break;
				case FIFO_TRIG_14:
					tunep->rcv_fifo = 14;
					break;
				}
				break;
		case TIOCCBRK:
			XT(IOCTL_L2_TIOCCBRK, siop, 0x0, 0x0, 0x0);
			mutex_enter(&siop->sio_mutex);
			ENDBRK(siop->base);
			siop->flags &= ~CF_BREAK;
			mutex_exit(&siop->sio_mutex);
			sio_start(siop);
			break;
		case TIOCSDTR:
			XT(IOCTL_L2_TIOCSDTR, siop, 0x0, 0x0, 0x0);
			mutex_enter(&siop->sio_mutex);
			siop->softmcr |= DTR;
			*(siop->base + MCR) = siop->softmcr;
			mutex_exit(&siop->sio_mutex);
			break;
		case TIOCCDTR:
			XT(IOCTL_L2_TIOCCDTR, siop, 0x0, 0x0, 0x0);
			mutex_enter(&siop->sio_mutex);
			siop->softmcr &= ~DTR;
			*(siop->base + MCR) = siop->softmcr;
			mutex_exit(&siop->sio_mutex);
			break;
		case TIOCMSET:
			XT(IOCTL_L2_TIOCMSET, siop, (*(int *) mp->b_cont->b_rptr), 0x0, 0x0);
			mutex_enter(&siop->sio_mutex);
			if ((*(int *) mp->b_cont->b_rptr) & TIOCM_DTR)
				siop->softmcr |= DTR;
			else
				siop->softmcr &= ~DTR;
			if ((*(int *) mp->b_cont->b_rptr) & TIOCM_RTS)
				siop->softmcr |= RTS;
			else
				siop->softmcr &= ~RTS;
			*(siop->base + MCR) = siop->softmcr;
			mutex_exit(&siop->sio_mutex);
			break;
		case TIOCMBIS:
			XT(IOCTL_L2_TIOCMBIS, siop, (*(int *) mp->b_cont->b_rptr), 0x0, 0x0);
			mutex_enter(&siop->sio_mutex);
			if ((*(int *) mp->b_cont->b_rptr) & TIOCM_DTR)
				siop->softmcr |= DTR;
			if ((*(int *) mp->b_cont->b_rptr) & TIOCM_RTS)
				siop->softmcr |= RTS;
			*(siop->base + MCR) = siop->softmcr;
			mutex_exit(&siop->sio_mutex);
			break;
		case TIOCMBIC:
			XT(IOCTL_L2_TIOCMBIC, siop, (*(int *) mp->b_cont->b_rptr), 0x0, 0x0);
			mutex_enter(&siop->sio_mutex);
			if ((*(int *) mp->b_cont->b_rptr) & TIOCM_DTR)
				siop->softmcr &= ~DTR;
			if ((*(int *) mp->b_cont->b_rptr) & TIOCM_RTS)
				siop->softmcr &= ~RTS;
			*(siop->base + MCR) = siop->softmcr;
			mutex_exit(&siop->sio_mutex);
			break;
		case TIOCMGET:
		{
			u_char msr, mcr;
			u_int g = 0;

			mutex_enter(&siop->sio_mutex);
			msr = *(siop->base + MSR);
			mcr = *(siop->base + MCR);

			if (mcr & DTR)
				g |= TIOCM_DTR;
			if (mcr & RTS)
				g |= TIOCM_RTS;
			if (msr & CTS)
				g |= TIOCM_CTS;
			if (msr & DSR)
				g |= TIOCM_DSR;
			if (msr & RI)
				g |= TIOCM_RNG;
			if (msr & DCD)
				g |= TIOCM_CAR;
			*(int *)mp->b_cont->b_rptr = g;
			mutex_exit(&siop->sio_mutex);
			XT(IOCTL_L2_TIOCMGET, siop, g, 0x0, 0x0);
			break;
		}
		default:
			XT(IOCTL_L2_UNKNOWN, siop, iocp->ioc_cmd, 0x0, 0x0);
			error = EINVAL;
			break;
		}
	}

	iocp->ioc_error = error;
	if (error != 0) {
		XT(IOCTL_L2_IOCNAK, siop, 0x0, 0x0, 0x0);
		mp->b_datap->db_type = M_IOCNAK;
	} else {
		XT(IOCTL_L2_IOCACK, siop, 0x0, 0x0, 0x0);
		mp->b_datap->db_type = M_IOCACK;
	}
	qreply(wq, mp);
	XT(IOCTL_L1_EXIT, siop, __LINE__, 0x0, 0x0);
	return;
}

/*
 * sio_reioctl
 */
static void
sio_reioctl(struct sioline *siop)
{
	queue_t	 *q;
	mblk_t	 *mp;

	XT(REIOCTL_L1_ENTRY, siop, siop, 0x0, 0x0);
	if ((q = siop->ttycommon.t_writeq) == NULL) {
		XT(REIOCTL_L1_EXIT, siop, __LINE__, 0x0, 0x0);
		return;
	}
	if ((mp = siop->ttycommon.t_iocpending) != NULL) {
		siop->ttycommon.t_iocpending = NULL;
		XT(REIOCTL_L2_IOCTL, siop, siop, q, mp);
		sio_ioctl(siop, q, mp);
	}
	XT(REIOCTL_L1_EXIT, siop, 0x0, 0x0, 0x0);
	return;
}


/*
 * sio_param
 */
static void 
sio_param(struct sioline *siop)
{
	u_int	cflag;
	u_char	lctrl;
	u_char	 *base;

	XT(PARAM_L1_ENTRY, siop, siop, 0x0, 0x0);
	base = siop->base;
	cflag = siop->ttycommon.t_cflag;

	switch (cflag & CSIZE) {
	case CS5:
		XT(PARAM_L2_CSIZE, siop, 5, 0x0, 0x0);
		lctrl = BITS5;
		break;
	case CS6:
		XT(PARAM_L2_CSIZE, siop, 6, 0x0, 0x0);
		lctrl = BITS6;
		break;
	case CS7:
		XT(PARAM_L2_CSIZE, siop, 7, 0x0, 0x0);
		lctrl = BITS7;
		break;
	case CS8:
		XT(PARAM_L2_CSIZE, siop, 8, 0x0, 0x0);
		lctrl = BITS8;
		break;
	default:
		XT(PARAM_L2_CSIZE, siop, 8, 0x0, 0x0);
		lctrl = BITS8;
		break;
	}
	if (cflag & PARENB) {
		lctrl |= PAREN;
		if (!(cflag & PARODD)) {
			XT(PARAM_L2_PAREVEN, siop, 0x0, 0x0, 0x0);
			lctrl |= PAREVN;  /* even parity */
		}
		else {
			XT(PARAM_L2_PARODD, siop, 0x0, 0x0, 0x0);
			lctrl &= ~PAREVN; /* odd parity */
		}
		/* add the parity bit */
	} else {
		XT(PARAM_L2_PARDIS, siop, 0x0, 0x0, 0x0);
		lctrl &= ~PAREN;
	}

	if (cflag & CSTOPB) {
		XT(PARAM_L2_STOP, siop, 2, 0x0, 0x0);
		lctrl |= STOP2;       /* 2 stop bit */
	} else {
		XT(PARAM_L2_STOP, siop, 1, 0x0, 0x0);
		lctrl &= ~STOP2;     /* 1 stop bit */
	}

	*(base + LCR) = lctrl;

	if ((cflag & CBAUD) == 0) {
		/* Drop dtr */
		XT(PARAM_L2_SETBAUD0, siop, 0x0, 0x0, 0x0);
		sio_setdtrrts(siop, ~(DTR | RTS));
		return;
	}

	cflag &= CBAUD;
	XT(PARAM_L2_SETBAUD, siop, cflag, 0x0, 0x0);

	/* set DLAB so that we can access the baud rate generator */
	*(base + LCR) = lctrl | DLAB;

	/* set the divisor rate */
	*base = sio_baud[cflag];
	*(base + 1) = sio_baud[cflag] >> 8;

	/* turn off DLAB - this allows access to RCVR/XMIT buffer & IER registers */
	*(base + LCR) = lctrl;

	XT(PARAM_L1_EXIT, siop, __LINE__, 0x0, 0x0);
	return;
}

/*
 *  set output modem signals (DTR & RTS).  Insure that there is a
 *  delay from DTR low to DTR high transition
 */
static void
sio_setdtrrts(struct sioline *siop, u_char value)
{
	int	now, held;

	XT(SETDTRRTS_L1_ENTRY, siop, siop, value, 0x0);
again:
	/*
	 * check time
	 */
	now = hrestime.tv_sec;
	held = now - siop->dtrlow;

	/*
	 * if DTR going low then save time
	 */
	if (!(value & DTR)) {
		XT(SETDTRRTS_L2_DTRLOW, siop, 0x0, 0x0, 0x0);
		siop->dtrlow = now;
	}/*
	  * if DTR going high then wait
	  */
	else if (held < hold_dtr_low) {
		XT(SETDTRRTS_L2_SLEEP, siop, 0x0, 0x0, 0x0);
		(void)cv_wait(&lbolt_cv, &siop->sio_mutex);
		goto again;
	}

	/*
	 * set bits
	 */
	XT(SETDTRRTS_L2_SETBITS, siop, 0, 0, 0);
	(value & DTR) ? (siop->softmcr |= DTR) : (siop->softmcr &= ~DTR);
	(value & RTS) ? (siop->softmcr |= RTS) : (siop->softmcr &= ~RTS);
	*(siop->base + MCR) = siop->softmcr;

	XT(SETDTRRTS_L1_EXIT, siop, 0, 0, 0);
	return;
}

/*
 * sio_start - Start output on a line, unless it's busy, frozen, or otherwise.
 */
static void 
sio_start(struct sioline *siop)
{

	register mblk_t	*bp;
	register int	bpcnt, fifocnt;
	mblk_t	*nbp;
	queue_t	 *q;

	XT(START_L3_ENTRY, siop, siop, 0x0, 0x0);
	mutex_enter(&siop->sio_mutex);

	if ((q = siop->ttycommon.t_writeq) == NULL) {
		mutex_exit(&siop->sio_mutex);
		XT(START_L3_EXIT, siop, __LINE__, 0x0, 0x0);
		return;
	}

	if (siop->flags & (CF_BREAK | CF_BUSY | CF_DELAY)) {
		mutex_exit(&siop->sio_mutex);
		XT(START_L3_EXIT, siop, __LINE__, 0x0, 0x0);
		return;
	}

	/* do we have flow-control char to send, if so then send it */
	if (siop->flow) {
		XT(START_L3_SENDQD, siop, siop->flow, siop->flow, 0x0);
		siop->flags |= CF_BUSY;
		/* send out the flow control char */
		TRANSMIT(siop->base, siop->flow);
		siop->flow = 0;
		mutex_exit(&siop->sio_mutex);
		XT(START_L3_EXIT, siop, __LINE__, 0x0, 0x0);
		return;
	}

	if (siop->flags & OSTOPPED) {
		XT(START_L3_STOPPED, siop, 0x0, 0x0, 0x0);
		mutex_exit(&siop->sio_mutex);
		XT(START_L3_EXIT, siop, __LINE__, 0x0, 0x0);
		return;
	}

	if ((bp = siop->lasttxmp) == NULL) {
		XT(START_L3_LASTTXMP, siop, 0x0, 0x0, 0x0);
		bp = getq(q);
		if(bp == NULL) {
			mutex_exit(&siop->sio_mutex);
			XT(START_L3_EXIT, siop, __LINE__, 0x0, 0x0);
			return;
		}	
		siop->lasttxmp= bp;
	}

	switch (bp->b_datap->db_type) {
		/* if we have a break char then send it and then exit */
		case M_BREAK:
			XT(START_L3_M_BREAK, siop, 0x0, 0x0, 0x0);
			SENDBRK(siop->base);
			siop->flags |= CF_BREAK;
			(void)timeout((void (*)())sio_restart, (caddr_t)siop, hz/4);
			mutex_exit(&siop->sio_mutex);
			freemsg(bp);
			siop->lasttxmp = NULL;
			break;
		case M_DELAY:
			XT(START_L3_M_DELAY, siop, 0x0, 0x0, 0x0);
			(void)timeout((void (*)())sio_restart, (caddr_t)siop, 
			    (int)(*(u_char * )bp->b_rptr + 6));
			siop->flags |= CF_DELAY;
			mutex_exit(&siop->sio_mutex);
			freemsg(bp);
			siop->lasttxmp = NULL;
			break;
 		case M_IOCTL:
			XT(START_L3_M_IOCTL, siop, 0x0, 0x0, 0x0);
			mutex_exit(&siop->sio_mutex);
			sio_ioctl(siop, q, bp);
			siop->lasttxmp = NULL;
			break;
		case M_DATA:
			XT(START_L3_STARTTIME, siop, 0x0, 0x0, 0x0);
			if((bpcnt = bp->b_wptr - bp->b_rptr) <= 0) {
				mutex_exit(&siop->sio_mutex);
				freemsg(bp);
				mutex_enter(&siop->sio_mutex);
				siop->lasttxmp = NULL;
			}

			fifocnt = min(bpcnt, (int)siop->xmit_fifo);

			XT(START_L3_COUNT, siop, fifocnt, bpcnt, 0x0);
			siop->flags |= CF_BUSY;

			while (fifocnt > 0) {
				XT(START_L3_XMITCHAR, siop, *(bp->b_rptr),*(bp->b_rptr), 0x0);
				TRANSMIT(siop->base, *(bp->b_rptr));

				/* update the pointers & counters */
				bp->b_rptr++;
				fifocnt--;
				bpcnt--;
			}

			if (bpcnt == 0) {
				nbp = bp;
				bp = bp->b_cont;
				freeb(nbp);
				/*
				 * lasttxmp will either be NULL or not, we don't care
				 */
				siop->lasttxmp = bp;
			}
			mutex_exit(&siop->sio_mutex);
			break;
		default:
			XT(START_L3_UNKNOWN, siop, bp->b_datap->db_type, 0, 0);
			break;
	}
	XT(START_L3_EXIT, siop, __LINE__, 0x0, 0x0);
	return;
}


/*
 *  sio_restart - restart output on a line after a delay or a break timer expired
 */
static void 
sio_restart(struct sioline *siop)
{
	queue_t	*wq;

	XT(RSTRT_L1_ENTRY, siop, siop, 0x0, 0x0);
	mutex_enter(&siop->sio_mutex);
	if (siop->flags & CF_BREAK)
		ENDBRK(siop->base);
	siop->flags &= ~(CF_DELAY | CF_BREAK);

/*
        if ((wq = siop->ttycommon.t_writeq) != NULL) {
                enterq(wq);
	}
*/

	mutex_exit(&siop->sio_mutex);
	sio_start(siop);
	XT(RSTRT_L1_EXIT, siop, __LINE__, 0x0, 0x0);

/*
	if (wq != NULL) {
		leaveq(wq);
	}
*/
}

/*
 *
 *  sio_intr - process an interrupt
 *
 *  we know that there is an interupt, now come into this routine to determine
 *  what kind of interrupt and dispatch it accordingly
 *
 */
static u_int
sio_intr(caddr_t instance)
{
	register struct sioline *siop;
	register int port;
	struct serial_board *sbp;
	int serviced = DDI_INTR_UNCLAIMED;
	u_char	isr;

	XT(INTR_L3_ENTRY, ND, 0x0, 0x0, 0x0);
	sbp = (struct serial_board *)ddi_get_soft_state(serial_boards,
							(u_int)instance);

	port = sbp->nport;
	for(--port; port >= 0; port--) {
		siop = sbp->siolines[port];

		isr = (*(siop->base + ISR) & INT_MASK);
		if (!(isr & NO_INT_PENDING)) {
			serviced = DDI_INTR_CLAIMED;
					
			switch (isr) {
			/* Data Available or Trigger Level Reached */
			case RxRDY:
				/*
				 * remove sio_pushdb to prevent sio_stash
				 * from being re-entrant
				 */
				(void)untimeout(siop->tid);
				siop->flags |= RXINTR;
				(void)sio_rxintr(siop);
				/*
				 * we want to call sio_pushdb to send the current
				 * mblk_t upstream, this is because we may have
				 * data that just sits there and never goes back
				 * upstream
				 */
				siop->tid = timeout((void (*)())sio_pushdb, (caddr_t)siop, (TICK * 1));
				break;

			/* fifo not empty and No more character */
			case RxEND:
				(void)untimeout(siop->tid);
				siop->flags |= RXINTR;
				(void)sio_rxtimeout(siop);
				break;

			/* Line Status OVRRUN,PARERR,FRMERR or BRKDET */
			case RSTATUS:
				(void)untimeout(siop->tid);
				siop->flags |= RXINTR;
				(void)sio_rsintr(siop);
				break;

			/* Transmitter Holding Register Empty */
			case TxRDY:
				(void)sio_txintr(siop);
				break;

			/* Modem Status */
			case MSTATUS:
				(void)sio_mintr(siop);
				break;

			default:
				XT(INTR_L3_UNKNOWN, siop, isr, 0x0, 0x0);
				break;
			}
			siop->flags &= ~RXINTR;
		}
	}
	XT(INTR_L3_EXIT, ND, serviced, 0x0, 0x0);
	return (serviced);
}

/*
 * sio_rxintr
 */
static void 
sio_rxintr(struct sioline *siop)
{
	register u_char *base; 
	int c;

	XT(RXINTR_L3_ENTRY, siop, siop, 0x0, 0x0);

	base = siop->base;
       	if (siop->ttycommon.t_readq == NULL) {
		/*
		 * drain FIFO until LSR(0) is reset & IIR=04 is reset
		 */
		while (*(base + LSR) & RCA) {
			c = *base;
		}
		XT(RXINTR_L3_EXIT, siop, __LINE__, 0x0, 0x0);
		return;
	}

	/* while FIFO is not empty */
	while (*(base + LSR) & RCA) {
		c = *base;
		XT(RXINTR_L3_RCVCHAR, siop, c, c, 0x0);

		/* Output control was enabled */
		if (siop->ttycommon.t_iflag & IXON) {
			/* if it wants to stop transmitter */
			if ((siop->ttycommon.t_iflag & ISTRIP) ?
				((c & 0x7f) == siop->ttycommon.t_stopc) :
				(c == siop->ttycommon.t_stopc)) {
				/*
				 * stop transmiter in middle of FIFO up to
				 * 16 characters may be lost but don't seem so
				 */
				XT(RXINTR_L3_TXSTOP, siop, 0x0, 0x0, 0x0);
				siop->flags |= OSTOPPED;
				continue;
			} /* if it wants to start transmitter */
			else if ((siop->ttycommon.t_iflag & ISTRIP) ?
				((c & 0x7f) == siop->ttycommon.t_startc) :
				(c == siop->ttycommon.t_startc)) { 
				XT(RXINTR_L3_TXSTART, siop, 0x0, 0x0, 0x0);
				if (siop->flags & OSTOPPED) {
					(void)timeout((void (*)())sio_start, (caddr_t)siop, (TICK * 1));
				}
				siop->flags &= ~(OSTOPPED | CF_BUSY);
				continue;
			}
		}

		if ( (c == 0377) && (siop->ttycommon.t_iflag & PARMRK)
			&& !(siop->ttycommon.t_iflag & (IGNPAR | ISTRIP)) ) {
			sio_stash(siop, 0377, NOPUSHCHAR);
			sio_stash(siop, c, NOPUSHCHAR);
		} else {
			/* put the char upstream */
			sio_stash(siop, c, NOPUSHCHAR);
		}
	}
	XT(RXINTR_L3_EXIT, siop, 0x0, 0x0, 0x0);
	return;
}

/*
 * sio_rxtimeout
 */
static void 
sio_rxtimeout(struct sioline *siop)
{
	register u_char		*base;
	u_char	c;

	XT(RXTIMEOUT_L3_ENTRY, siop, siop, 0x0, 0x0);

	base = siop->base;

	/* check to see if someone is attached upstream */
	if (siop->ttycommon.t_readq == NULL) {
		/*
		 * no one has us open for reading - must be a stray interrupt
		 *
		 * drain FIFO until LSR(0) is reset & IIR=04 is reset
		 */
		while ( (*(base + LSR) & RCA) ) {
			c = *base;
		}
		XT(RXINTR_L3_EXIT, siop, __LINE__, 0x0, 0x0);
		return;
	}

	/* while FIFO is not empty */
	while ( (*(base + LSR) & RCA) ) {
		c = *base;

		XT(RXTIMEOUT_L3_RCVCHAR, siop, c, c, 0x0);
		/* Output control was enabled */
		if (siop->ttycommon.t_iflag & IXON) {
			/* if it wants to stop transmitter */
			if ( c == siop->ttycommon.t_stopc) {
			/*
			 * stop transmiter in middle of FIFO up to
			 * 16 characters may be lost but don't seem so
			 */
				XT(RXTIMEOUT_L3_TXSTOP, siop, 0x0, 0x0, 0x0);
				siop->flags |= OSTOPPED;
				continue;
			} 
			/* if it wants to start transmitter */
			else if ( c == siop->ttycommon.t_startc) {
				XT(RXTIMEOUT_L3_TXSTART, siop, 0x0, 0x0, 0x0);
				if (siop->flags & OSTOPPED) {
					(void)timeout((void (*)())sio_start, (caddr_t)siop, (TICK * 1));
				}
				siop->flags &= ~(OSTOPPED | CF_BUSY);
				continue;
			}
		}
 		if ( (c == 0377) && (siop->ttycommon.t_iflag & PARMRK)
			&& !(siop->ttycommon.t_iflag & (IGNPAR | ISTRIP)) ) {
			sio_stash(siop, 0377, NOPUSHCHAR);
			sio_stash(siop, c, PUSHCHAR);
		} else {
			/* put the char upstream */
			sio_stash(siop, c, PUSHCHAR);
		}
		XT(RXTIMEOUT_L3_EXIT, siop, __LINE__, 0x0, 0x0);

	}
	return;
}


/*
 *  sio_rsintr
 */
static void 
sio_rsintr(struct sioline *siop)
{
	register u_char		*base;
	register u_char		c;
	register queue_t	 *q;
	int	status;

	base = siop->base;

	c = *base;

	status = *(base + LSR);

	XT(RSINTR_L3_ENTRY, siop, siop, status, 0x0);

	if ((q = siop->ttycommon.t_readq) == NULL) {
		/*
		 * no one has us open for reading - must be a stray interrupt
		 *
		 * drain FIFO until LSR(0) is reset & IIR=04 is reset
		 */
		while ( (*(base + LSR) & RCA) ) {
			c = *base;
		}
		XT(RSINTR_L3_EXIT, siop, __LINE__, 0x0, 0x0);
		return;
	}

	if (status & OVRRUN) {
		XT(RSINTR_L3_OVRRUN, siop, 0x0, 0x0, 0x0);
		sio_rxintr(siop);
		XT(RSINTR_L3_EXIT, siop, __LINE__, 0x0, 0x0);
		return;
	}

	if (status & PARERR) {
		XT(RSINTR_L3_PARERR, siop, c, c, 0x0);
		/* mismatch parity, only 7-bit needed */

		/* Parity error -- possibly MISMATCH PARITY */
		/* If Software flow control is required */
		/* Output control was enabled */
		if (siop->ttycommon.t_iflag & IXON) {
			/* if it wants to stop transmitter */
			if ((c & 0x7f) == siop->ttycommon.t_stopc) {
				/*
				 * stop transmiter in middle of FIFO
		   		 * hopefully nothing get lost
				 */
				XT(RSINTR_L3_TXSTOP, siop, 0x0, 0x0, 0x0);
				siop->flags |= OSTOPPED;
				XT(RSINTR_L3_EXIT, siop, __LINE__, 0x0, 0x0);
				return;
			} /* if it wants to start transmitter */
			else if ((c & 0x7f) == siop->ttycommon.t_startc) {
				XT(RSINTR_L3_TXSTART, siop, 0x0, 0x0, 0x0);
				if (siop->flags & OSTOPPED) {
					(void)timeout((void (*)())sio_start, (caddr_t)siop, (TICK * 1));
				}
				siop->flags &= ~(OSTOPPED | CF_BUSY);
				XT(RSINTR_L3_EXIT, siop, __LINE__, 0x0, 0x0);
				return;
			}
		}

		if (siop->ttycommon.t_iflag & INPCK) {
			if (!(siop->ttycommon.t_iflag & IGNPAR)) {
				if (!(siop->ttycommon.t_iflag & PARMRK)) {
					sio_stash(siop, 0377, NOPUSHCHAR);
					sio_stash(siop, 0, NOPUSHCHAR);
					sio_stash(siop, c, PUSHCHAR);

				} else {
					sio_stash(siop, 0, PUSHCHAR);

				}
			}
		} else {
			if ( (c == 0377) && (siop->ttycommon.t_iflag & PARMRK)
			     && !(siop->ttycommon.t_iflag & (IGNPAR | ISTRIP)) ) {
				sio_stash(siop, 0377, NOPUSHCHAR);
				sio_stash(siop, c, PUSHCHAR);

			} else {
				sio_stash(siop, c, PUSHCHAR);

			}
		}
	}

	if (status & FRMERR) {
		XT(RSINTR_L3_FRMERR, siop, 0x0, 0x0, 0x0);
	}

	if (status & BRKDET) {
		XT(RSINTR_L3_BRKDET, siop, 0x0, 0x0, 0x0);
/*
		(void) putctl(q->q_next, M_BREAK);
*/
	}

	XT(RSINTR_L3_EXIT, siop, __LINE__, 0x0, 0x0);
	return;
}

/*
 * sio_txintr
 */
static void 
sio_txintr(struct sioline *siop)
{
	XT(TXINTR_L3_ENTRY, siop, siop, 0x0, 0x0);
	siop->flags &= ~(CF_BUSY);
	sio_start(siop);
	XT(TXINTR_L3_EXIT, siop, 0x0, 0x0, 0x0);
}

/*
 * sio_mintr
 */
static void 
sio_mintr(struct sioline *siop)
{
	u_char	*base, status;
	queue_t	*q;

	base = siop->base;
	status = *(base + MSR);
	XT(MINTR_L3_ENTRY, siop, siop, status, 0x0);

	/* CTS is high means modem is ready to receive */
	if (status & CTS) {
		XT(MINTR_L3_CTSHIGH, siop, 0x0, 0x0, 0x0);
		if ((siop->ttycommon.t_cflag & CRTSCTS) && 
		    (siop->flags & OSTOPPED)) {
			XT(MINTR_L3_TXSTART, siop, 0x0, 0x0, 0x0);
			siop->flags &= ~(OSTOPPED | CF_BUSY);
			/*
			 * start the transmit now
			 */
			sio_start(siop);
		}
	} else {
		XT(MINTR_L3_CTSLOW, siop, 0x0, 0x0, 0x0);
		if (siop->ttycommon.t_cflag & CRTSCTS) {
			XT(MINTR_L3_TXSTOP, siop, 0x0, 0x0, 0x0);
			siop->flags |= OSTOPPED;
		}
	}

	if (status & DDCD) {
		/* see if there's DCD */
		if ((status & DCD) || (siop->ttycommon.t_flags & TS_SOFTCAR)) {
			if (!(siop->flags & CF_CARR_ON)) {
				siop->flags |= CF_CARR_ON;
				if ((q = siop->ttycommon.t_readq) != NULL) {
					XT(MINTR_L3_CD_UNHANGUP, siop, 0x0, 0x0, 0x0);
					putctl(q->q_next, M_UNHANGUP);
				}
				XT(MINTR_L3_CD_WAKEUP, siop, 0x0, 0x0, 0x0);
				cv_broadcast(&siop->login_cv);
			}
		} else { /* Else no dcd */
			XT(MINTR_L3_NOCD, siop, 0x0, 0x0, 0x0);
			if ( (siop->flags & CF_CARR_ON) && 
			    !(siop->ttycommon.t_cflag & CLOCAL) ) {

				siop->flags &= ~(OSTOPPED | CF_BUSY);
				if ((q = siop->ttycommon.t_readq) != NULL) {
					XT(MINTR_L3_M_HANGUP, siop, 0x0, 0x0, 0x0);
					putctl(q->q_next, M_HANGUP);
				}

				if ((q = siop->ttycommon.t_writeq) != NULL) {
					flushq(q, FLUSHDATA);
				}

				if(siop->lasttxmp != NULL) {
					freemsg(siop->lasttxmp);
					siop->lasttxmp = NULL;
				}

				/* turn off all interrupts except modem */
				*(siop->base + IER) = M_en;
			}
			siop->flags &= ~CF_CARR_ON;
		}
	}
	XT(MINTR_L3_EXIT, siop, __LINE__, 0x0, 0x0);
}


/*
 * sio_stash
 */
static void 
sio_stash(struct sioline *siop, u_char c, int pushchar)
{
	register mblk_t *bp;
	queue_t	*q;
	int	bufsz;

	XT(STASH_L3_ENTRY, siop, siop, c, c)
	mutex_enter(&siop->sio_mutex);

	if ((q = siop->ttycommon.t_readq) == NULL) {
		XT(STASH_L3_EXIT,siop, __LINE__, 0x0, 0x0);
		mutex_exit(&siop->sio_mutex);
		return;
	} else {
/*
		enterq(q);
*/
	}

	if ((bp = siop->lastrxmp) == NULL) {
		XT(STASH_L3_ALLOCB, siop, __LINE__, 0x0, 0x0);
		bufsz = (siop->ttycommon.t_cflag & CBAUD) > B9600 ? CFSIZE2 : CFSIZE1;
		if ((bp = allocb(bufsz, BPRI_MED)) == NULL) {
			XT(STASH_L3_EXIT,siop, __LINE__, 0x0, 0x0);
			mutex_exit(&siop->sio_mutex);
			return;
		}
		siop->lastrxmp = bp;
	}

	if (pushchar != PUSHDB) {
		/* store the character */
		*bp->b_wptr++ = c;
	}

	switch (pushchar) {	
	case NOPUSHCHAR:
		/* Ready for a new buffer ? */
		if (bp->b_wptr >= bp->b_datap->db_lim) {
			if(canput(q->q_next)) {
				putq(q, bp);
				siop->trycanput = 0;
			} else {
				if (++(siop->trycanput) > trycanput) {
						XT(STASH_L3_DROPDATA, siop, __LINE__, 0x0, 0x0);
						printf("board %d port %d dropped data\n",
								siop->board, siop->port);
						ttycommon_qfull(&siop->ttycommon, q);
						mutex_exit(&siop->sio_mutex);
						freemsg(bp);
						mutex_enter(&siop->sio_mutex);
						siop->trycanput = 0;
				} else {
					putq(q, bp);
				}
			}
			siop->lastrxmp = NULL;
		}
		break;
	case PUSHDB:
	case PUSHCHAR:
		if(canput(q->q_next)) {
			putq(q, bp);
			siop->trycanput = 0;
		} else {
			if (++(siop->trycanput) > trycanput) {
				XT(STASH_L3_DROPDATA, siop, __LINE__, 0x0, 0x0);
				ttycommon_qfull(&siop->ttycommon, q);
				mutex_exit(&siop->sio_mutex);
				freemsg(bp);
				mutex_enter(&siop->sio_mutex);
				siop->trycanput = 0;
			} else {
				putq(q, bp);
			}
		}
		siop->lastrxmp = NULL;
		break;
	}
	mutex_exit(&siop->sio_mutex);
	XT(STASH_L3_EXIT, siop, __LINE__, 0x0, 0x0);
/*
	leaveq(q);
*/
}

/*
 * sio_pushdb - sends the mblk_t upstream
 *
 * this routine is called because we need to send the current mblk_t upstream
 * we could have a mblk_t sitting inside of lastrxmp and it never go back up
 */
static void
sio_pushdb(struct sioline *siop)
{
	XT(PUSHDB_L1_ENTRY, siop, siop, 0x0, 0x0);
	if (siop->flags & RXINTR) {
		XT(PUSHDB_L2_TIMEOUT, siop, siop, 0x0, 0x0);
		(void)timeout((void (*)())sio_pushdb, (caddr_t)siop, (TICK * 1));
	} else {
		sio_stash(siop, NULL, PUSHDB);
	}
	XT(PUSHDB_L1_EXIT, siop, 0x0, 0x0, 0x0);
}
