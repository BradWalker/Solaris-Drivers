/*

	COPYRIGHT (c) 1993-1996 BY AURORA TECHNOLOGIES, INC., WALTHAM, MA.
	
	THIS SOFTWARE IS FURNISHED UNDER A LICENSE AND MAY BE USED AND
	COPIED ONLY IN ACCORDANCE WITH THE TERMS OF SUCH LICENSE AND WITH
	THE INCLUSION OF THE ABOVE COPYRIGHT NOTICE.  THIS SOFTWARE OR
	ANY OTHER COPIES THEREOF MAY NOT BE PROVIDED OR OTHERWISE MADE
	AVAILABLE TO ANY OTHER PERSON.  NO TITLE TO AND OWNERSHIP OF THE
	PROGRAM IS HEREBY TRANSFERRED.

	THE INFORMATION IN THIS SOFTWARE IS SUBJECT TO CHANGE WITHOUT
	NOTICE AND SHOULD NOT BE CONSIDERED AS A COMMITMENT BY AURORA
	TECHNOLOGIES, INC.


	file: sio16_ioctl.h
	author: bwalker
	created: 5/4/93
	sccs info: @(#)$Header: /vol/sources.cvs/dev/sio16.svr4/sio16_ioctl.h,v 1.6 1996/09/20 21:23:09 bkd Exp $

	Copyright (c) 1993-1996 Aurora Technologies, Inc. All Rights Reserved.
*/

/*
 * $Log: sio16_ioctl.h,v $
 * Revision 1.6  1996/09/20 21:23:09  bkd
 * + changed copyright from 1992-1995 to 1993-1996
 * + added pragma ident
 * + removed unused ZZTCSBRK definition
 * + changed CS_MAXBOARDS to STATS_MAXBOARDS; added additional
 *   logic to SIO_STAT_CHK_UPDATE to limit board number
 *
 * Revision 1.5  1995/08/15 23:14:20  bkd
 * Updated copyrights; added CVS control information
 *
 */

#ifdef sun				/* XXXAIX */
#   pragma ident "@(#)$Header: /vol/sources.cvs/dev/sio16.svr4/sio16_ioctl.h,v 1.6 1996/09/20 21:23:09 bkd Exp $"
#endif

#include <sys/ioccom.h>

/* ------------------------------------------------------------------------- */
/* General macros */

#define	SIOC		('a' << 8)
/*
 * ioctls used for last 4 ports on 1600SE
 */
#define	SIO_GETMFLG	(SIOC|1)	/* get modem flg, returns (int) */
#define	SIO_MAPCD	0x01		/* CTS mapped to CD (12-15) */
#define	SIO_MAPDTR	0x02		/* RTS mapped to DTR (12-15) */
#define	SIO_MAPALL	0x04		/* all pins supported */
#define	SIO_DTRFLOW	0x08		/* use DTR for flow ctl (0-11) */

#define SIO_SETRTS	(SIOC|2)	/* map RTS to RTS, null return */
#define SIO_SETRTSFLOW	(SIOC|3)	/* use RTS for flow ctl, null rtn */
#define SIO_SETDTR	(SIOC|4)	/* map RTS to DTR, null rtn */
#define SIO_SETDTRFLOW	(SIOC|5)	/* use DTR for flow ctl, null rtn */

#define SIO_SETCTS	(SIOC|6)	/* map CTS to CTS, null rtn */
#define SIO_SETCD	(SIOC|7)	/* map CTS to CD, null rtn */

#define	SIO_SETTIMEOUT	(SIOC|8)	/* set close timeout, send (int) */
#define	SIO_GETTIMEOUT	(SIOC|9)	/* get close timeout, returns (int) */

/*
 * set baud rates for 1600SE
 */
#define SIO_SETBAUD	(SIOC|10)	/* set baud rate, send (int) */
#define SIO_GETBAUD	(SIOC|11)	/* get baud rate, returns (int) */

/* !!!bkd 9/13/1996 - hack alert: ugh: */
#define	STATS_MAXBOARDS	16		/* max boards for statistics */

/*
 * Async Error Statistics Support
 */

/* update statistics field and check for overflow */
#define SIO_STAT_UPDATE(linep,stat,x)                           \
{       unsigned int sio_stat_value;                            \
        sio_stats_t *sio_statsp = (sio_stats_t *)linep->l_sio_stats; \
        sio_stats_t *sio_validp = (sio_stats_t *)linep->l_sio_stats_valid; \
                                                                \
        sio_stat_value = sio_statsp->stat;                      \
        sio_stat_value += x;                                    \
        if (sio_stat_value < sio_statsp->stat) {                \
                sio_validp->stat = 1;                           \
        }                                                       \
        sio_statsp->stat = sio_stat_value;                      \
}

/* update statistics field, check for overflow, AND */
/* set the sio_stats_chk bit to indicate port error */
#define SIO_STAT_CHK_UPDATE(linep,stat,x)                               \
{       unsigned int sio_stat_value;                            \
        sio_stats_t *sio_statsp = (sio_stats_t *)linep->l_sio_stats; \
        sio_stats_t *sio_validp = (sio_stats_t *)linep->l_sio_stats_valid; \
                                                                \
        sio_stat_value = sio_statsp->stat;                      \
        sio_stat_value += x;                                    \
        if (sio_stat_value < sio_statsp->stat) {                \
                sio_validp->stat = 1;                           \
        }                                                       \
        sio_statsp->stat = sio_stat_value;                      \
	if ((linep)->board < STATS_MAXBOARDS) {			\
	    sio16_stats_chk[(linep)->board] |= (1 << (linep)->port);    \
	}							\
}

/*
 * Asynchronous Line Error & Statistics Support:
 *
 * There are 3 ioctl commands defined to facilitate reporting/reseting
 * error statistics.  These have been added because there is no equivalent
 * support through UNIX to record overrun and framing errors (parity errors
 * are supported through UNIX but in a very cumbersome way).  Also added
 * is support for number of characters xmitted adn dropped.
 *
 * SIO_STATS -	This command allows the user to request the driver to
 *		respond with current channel statistics.  The driver responds
 *		with an "sio_stats_report" structure (see below). The
 *		statistics maintained are self descriptive.  If any statistic
 *		overflows the space allocated, the corresponding cell in the
 *		"sio_stats_overflow" is set non-zero.
 *
 * SIO_STATS_RST - This command functions like the SIO_STATS command and has
 *		the additional effect of clearing the statistics
 *		(sio_stats_report & sio_stats_check) following reporting.
 *
 * SIO_STATS_CHK - This command provides the user with a quick method to
 *		check if any channel error has occured in the system.
 *		The driver responds with an "sio_stats_check" array where each
 *		cell corresponds to a board and each bit of a cell corresponds
 *		to a channel.  If the bit is set an error has been detected
 *		on the channel.  The user will use the SIO_STATS ioctl to
 *		retreive the detailed statistics.  The sio_stats_check array
 *		is an array of integers, each element corresponding to a 
 *		board, each bit of the integer corresponding to a port.
 *
 *		A bit in the sio_stats_check array will be set only when a
 *		field in the sio_statistics structure beginning with "error_"
 *		is incremented.
 *	
 *		The first array element (index 0) corresponds to board #0,
 *		element #1 corresponds to board #1, etc.
 *
 *		The first bit (bit #0) corresponds to line #0 of that board,
 *		bit #1 corresponds to line #1, etc.
 *
 * SIO_STATS_RST_ALL - This command resets all statistics fields for all
 *		ports.
 *
 */

typedef struct sio_stats {
	unsigned int	error_receive_overrun;
	unsigned int	error_receive_framing;
	unsigned int	error_receive_parity;
	unsigned int	error_receive_dropchars;
	unsigned int	stats_receive_chars;
	unsigned int	stats_transmit_chars;
} sio_stats_t;

typedef struct sio_stats_report {
	sio_stats_t	sio_statistics;
	sio_stats_t	sio_stats_overflow;
} sio_stats_report_t;

typedef unsigned int sio_stats_check_t[STATS_MAXBOARDS];

#define	SIO_STATS		_IOR('a', 90, sio_stats_report_t)
#define	SIO_STATS_CHK		_IOR('a', 92, sio_stats_check_t)
#define	SIO_STATS_RST		_IOR('a', 91, sio_stats_report_t)
#define	SIO_STATS_RST_ALL	_IO('a', 93)

 
