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

*/

/*

	file: xxtrace.h
	author: joe provino
	created: 6/25/92
	sccs info: @(#)xxtrace.h	1.1 12/18/92 1

*/

#ifndef _XXTRACE_H
#define _XXTRACE_H
#include <sys/types.h>

struct event {
	char *event_name;
	char *event_format;
};

struct trevent {
	u_short		tr_eventnum;
	u_char		tr_dev;
	u_char		tr_unit;
	unsigned int	tr_a;
	unsigned int	tr_b;
	unsigned int	tr_c;
};

#define XXTR_MAGIC 0xdeadbeef		/* magic number			*/
#define MAXEVENTNUM	256		/* maximum number of events	*/
#define	MAXDEVS		16		/* maximum number of devices	*/
#define	MAXUNITS	32		/* maximum units per device	*/

/*
 * structure that is the trace device structure, every driver has a "line"
 * structure that will start with these elements.  This is the structure
 * sent to the xxtrace() routine.  MAKE SURE THIS STRUCTURE IS WORD
 * ALIGNED!!
 */

struct trdev {
	unsigned char last_event;		/* save last_event */
	unsigned char dev;			/* i.e. board # */
	unsigned char unit;			/* i.e. port # */
	unsigned char pad;			/* unused */
};

/*
 * NULL device
 */
static struct trdev NULLDEV = {
        0,
        255,                     /* board #255 */
        255,                     /* port #255 */
        0
};

#define ND &NULLDEV

struct trinfo {
	/*
	 * The following elements are set once and never changed.
	 */
	int xxtr_magic;			/* for sanity */
	struct event *xxtr_event_tab;	/* pointer to event name table */
	int xxtr_ndev;			/* number of devices */
	int xxtr_nunit;			/* number of units per device */
	int xxtr_actual_device[MAXDEVS];/* indicates devices/units present */

	/*
	 * The following elements are set by the driver only.
	 */
	struct trevent *xxtr_buf;	/* circular buffer */
	int xxtr_next;			/* index of next entry in circ buf */

	/*
	 * The following elements are set by both the user level utility 
	 * xxtrace and the driver.
	 *
	 * xxtr_maxevents is initialized to the value MAXEVENTS by the driver 
	 * when the driver is loaded and when a reset command is issued.
	 *
	 * The user utility sets xxtr_maxevents and restarts tracing
	 * to make it take effect.
	 */
	int xxtr_maxevents;		/* size of circular buffer */
	int xxtr_flag;			/* flags defined below */

	/*
	 * The following fields are set by the user level utility xxtrace only.
	 */
	char xxtr_event[MAXEVENTNUM];	/* events we're tracing */
	int xxtr_device[MAXDEVS];	/* indicates devices/units traced */

	/*
	 * Not used.
	 */
	int pad;			/* to make the struct long word sized */
};

/*
 * Flag definitions
 */
#define RUNNING 1
#define RESET 2
#define CLEAR 4

/*
 * Defaults.
 */
#define MAXEVENTS 40000			/* maximum allowable events */
#define DEFEVENTS 5000			/* default number of events */
#define RAWDATAFILE "trace.out"
#define	SCCSIDLEN 128

#ifdef _KERNEL

/*
 * Here's the macro that will be used in the driver only.
 * I just put it here so you could see what it will be like.
 * event_number is meant to be one of the "E_" definitions such as E_OPEN.
 * You need to sprinkle XXTRACE lines throughout the driver.
 * "a" - "c" are args with "interesting" information to stash away.
 */

#ifdef XXTRACE
#define XT(event_number, dev, a, b, c) \
	((struct trdev *)(dev))->last_event = event_number; \
	if (xxtrinfo.xxtr_flag) \
		xxtrace(event_number, dev, a, b, c);

#else
#define XT(e,d,a,b,c)
#endif

/*
 * extern/routine definitions
 */
#ifdef XXTRACE
extern struct trinfo xxtrinfo;
extern int xxtraceinit(/* int ndev; int nunit */);
extern int xxtraceunload();
#else
#define xxtraceinit(ndev,nunit) ;
#define xxtraceunload() ;
#endif

#endif

#endif _XXTRACE_H
