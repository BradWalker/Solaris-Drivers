/*

	COPYRIGHT (c) 1992-1996 BY AURORA TECHNOLOGIES, INC., WALTHAM, MA.
	
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

	file: mset.c
	author: dgk
	created: 11/16/92
	sccs info: $Id: mset.c,v 1.7 1996/09/20 21:15:49 bkd Exp $

*/

/*
 * $Log: mset.c,v $
 * Revision 1.7  1996/09/20 21:15:49  bkd
 * + changed copyright to 1996
 * + changed CS_MAXBOARDS to STATS_MAXBOARDS
 *
 * Revision 1.6  1995/08/15 23:13:56  bkd
 * Updated copyrights; added CVS control information
 *
 */

static char CopyId[] = "@(#)Copyright (c) 1992-1996 Aurora Technologies, Inc.";
static char SccsId[] = "@(#)$Header: /vol/sources.cvs/dev/sio16.svr4/mset.c,v 1.7 1996/09/20 21:15:49 bkd Exp $";
#include <sys/types.h>
#include <stdio.h>
#include <fcntl.h>
#include <stropts.h>
#include <sys/ttydev.h>
#include "sio16_ioctl.h"

extern int errno;

static int stats();
static int statsr();
static int statschk();
static int statsreset();
static int setcd();
static int setcts();
static int setrts();
static int setdtr();
static int setrtsflow();
static int setdtrflow();
static int setbaud();
static int timeout();
extern stdbaud();
extern show();
extern int parse_baud();
extern int parse_timeout();

struct option {
	char *name;		/* option name */
	int flag;		/* flag indicating option was specified. */
	int (*parsefcn)();	/* function to parse option parameters */
	int (*action)();	/* function to perform action for this option */
};

struct option option[] = {
	{"std",         0,      NULL,          stdbaud         },
	{"show",	0,	NULL,		show		},
	{"cd",		0,	NULL,		setcd		},
	{"cts",		0,	NULL,		setcts		},
	{"rts",		0,	NULL,		setrts		},
	{"dtr",		0,	NULL,		setdtr		},
	{"dtrflow",	0,	NULL,		setdtrflow	},
	{"rtsflow",	0,	NULL,		setrtsflow	},
	{"timeout",	0,	parse_timeout,	timeout		},
	{"stats",       0,       NULL,          stats           },
        {"statsr",      0,       NULL,          statsr          },
        {"statschk",    0,       NULL,          statschk        },
        {"statsreset",  0,       NULL,          statsreset      },

	/*
	 * this must be the last valid function in order to parse
	 * the baud rate without using a key word
	 */
	{"baud",	0,	parse_baud,	setbaud		},
	/* add options here */

	{NULL,	    NULL,	NULL}
};

static int fd;			/* file descriptor of device */
static char devname[100];	/* pointer to device name */
static int custom_baud;		/* custom baud rate desired */
static int time_val;		/* timeout value in seconds */
static struct strioctl strioctl;/* strioctl structure */
static sio_stats_report_t asr;  /* statistics structure */
static sio_stats_check_t asc;   /* overall statistics structure */

main(argc, argv)
	int argc;
	char *argv[];
{
	register struct option	*op;
	register char *cp;
	int n;

	if (argc < 2)
		usage("missing device name");

	if (argc < 3)
		usage("missing options");

	/*
	 * open device
	 */
	strcpy(devname, "/dev/");
	strcat(devname, argv[1]);
	if ( (fd = open(devname, O_RDONLY | O_NDELAY)) == -1 ) {
		printf("cannot open %s\n", devname);
		perror(NULL);
		exit(1);
	}

	/*
	 * Skip our program name & device name.
	 */
	argc -= 2;
	argv += 2;

	/*
	 * Get the options.
	 */
	while (argc > 0) {
		cp = *argv;
		if (*cp++ != '-')
			usage("options must start with '-'");

		for (op = option; op->name != NULL; op++) {
			if (strcmp(op->name, cp) == 0) {
				argc--;
				argv++;
				op->flag = 1;
				if (op->parsefcn) {
					n = (*op->parsefcn)(argc, argv);
					argc -= n;
					argv += n;  /* skip args processed */
				}
				break;
			}

			/*
			 * check for baud rate
			 */
			if (strcmp(op->name, "baud") == 0) {
				op->flag = 1;
				if (op->parsefcn) {
					n = (*op->parsefcn)(argc, argv);
					argc -= n;
					argv += n;  /* skip args processed */
				}
				break;
			}
		}
		if (op->name == NULL)
			usage("invalid option");	/* invalid option */
	}

	/*
	 * Now execute the actions for each option specified.
	 */
	for (op = option; op->name != NULL; op++) {
		if (op->flag && op->action)
			(*op->action)();	/* do it! */
	}
}

/*               
 * show error statistics 
 */
static int
stats()  
{
        strioctl.ic_cmd = SIO_STATS;
        strioctl.ic_timout = 0;
        strioctl.ic_len = 0;
        strioctl.ic_dp = (char *)&asr;
        if (ioctl(fd, I_STR, &strioctl) == -1) {
                perror("stats(): SIO_STATS");
                exit(1); 
        }

        printf("%s statistics:\n",devname);
        printf("\treceiver overruns:            %d %s\n",
                asr.sio_statistics.error_receive_overrun,
                asr.sio_stats_overflow.error_receive_overrun != 0 ? "(overflow)"
: "");
        printf("\treceiver frame errors:        %d %s\n",
                asr.sio_statistics.error_receive_framing,
                asr.sio_stats_overflow.error_receive_framing != 0 ? "(overflow)"
: "");
        printf("\treceiver parity:              %d %s\n",
                asr.sio_statistics.error_receive_parity,
                asr.sio_stats_overflow.error_receive_parity != 0 ? "(overflow)" : "");
        printf("\treceiver chars dropped:       %d %s\n",
                asr.sio_statistics.error_receive_dropchars,
                asr.sio_stats_overflow.error_receive_dropchars != 0 ? "(overflow)" : "");
        printf("\n");
        printf("\treceived chars:               %d %s\n",
                asr.sio_statistics.stats_receive_chars,
                asr.sio_stats_overflow.stats_receive_chars != 0 ? "(overflow)" :
"");
        printf("\ttransmitted chars:            %d %s\n",
                asr.sio_statistics.stats_transmit_chars,
                asr.sio_stats_overflow.stats_transmit_chars != 0 ? "(overflow)" : "");
}        
 
/*               
 * show error statistics and reset
 */                      
static int
statsr()
{
        strioctl.ic_cmd = SIO_STATS_RST;
        strioctl.ic_timout = 0;
        strioctl.ic_len = 0;
        strioctl.ic_dp = (char *)&asr;
        if (ioctl(fd, I_STR, &strioctl) == -1) {
                perror("statsr(): SIO_STATS_RST");
                exit(1);
        }
 
        printf("%s statistics:\n",devname);
        printf("\treceiver overruns:      %d %s\n",
                asr.sio_statistics.error_receive_overrun,
                asr.sio_stats_overflow.error_receive_overrun != 0 ? "(overflow)"
: "");
        printf("\treceiver frame errors:  %d %s\n",
                asr.sio_statistics.error_receive_framing,
                asr.sio_stats_overflow.error_receive_framing != 0 ? "(overflow)"
: "");
        printf("\treceiver parity:        %d %s\n",
                asr.sio_statistics.error_receive_parity,
                asr.sio_stats_overflow.error_receive_parity != 0 ? "(overflow)" : "");
        printf("\treceiver chars dropped:  %d %s\n",
                asr.sio_statistics.error_receive_dropchars,
                asr.sio_stats_overflow.error_receive_dropchars != 0 ? "(overflow)" : "");
        printf("\n");
        printf("\treceived chars:         %d %s\n",
                asr.sio_statistics.stats_receive_chars,
                asr.sio_stats_overflow.stats_receive_chars != 0 ? "(overflow)" :
"");
        printf("\ttransmitted chars:      %d %s\n",
                asr.sio_statistics.stats_transmit_chars,
                asr.sio_stats_overflow.stats_transmit_chars != 0 ? "(overflow)" : "");
}        
 
/*               
 * check for overall errors
 */                      
static int
statschk()
{
        int i, j;
        int print_hdr = 0;
        int print_lbl = 0;
 
        strioctl.ic_cmd = SIO_STATS_CHK;
        strioctl.ic_timout = 0;
        strioctl.ic_len = 0;
        strioctl.ic_dp = (char *)&asc;
        if (ioctl(fd, I_STR, &strioctl) == -1) {
                perror("statschk(): SIO_STATS_CHK");
                exit(1);
        }
 
        for ( i = 0 ; i < STATS_MAXBOARDS ; i++ ) {
                if (asc[i] != NULL) {
                        if (print_hdr == 0) {
                                printf("Errors detected on:\n\n");
                                print_hdr++;
                        }
                        print_lbl = 0;
                        for ( j = 0 ; j < (sizeof(*asc) * 8) ; j++ ) {
                                if (asc[i] & 1 << j) {
                                        if (print_lbl == 0) {
                                                printf("board %d, port : %d",i,j);
                                                print_lbl++;
                                        } else {
                                                printf(", %d",j);
                                        }
                                }
                        }
                        printf("\n");
                }
        }
 
        if (print_hdr == 0) {
                printf("No Errors detected");
        }
        printf("\n");
}
 
/*               
 * check for overall errors and reset
 */                      
static int
statsreset()
{
        int i, j;
        int print_hdr = 0;
        int print_lbl = 0;
 
        strioctl.ic_cmd = SIO_STATS_RST_ALL;
        strioctl.ic_timout = 0;
        strioctl.ic_len = 0;
        strioctl.ic_dp = (char *)&asc;
        if (ioctl(fd, I_STR, &strioctl) == -1) {
                perror("statsreset(): SIO_STATS_RST_ALL");
                exit(1);
        }
}

/*
 * set CD tracking for this port
 */
static int
setcd()
{
	strioctl.ic_cmd = SIO_SETCD;
	strioctl.ic_timout = 0;
	strioctl.ic_len = 0;
	strioctl.ic_dp = (char *)NULL;
	if (ioctl(fd, I_STR, &strioctl) == -1) {
		perror("setting CD not support on this port");
		exit(1);
	}
}

/*
 * set CTS tracking for this port
 */
static int
setcts()
{
	strioctl.ic_cmd = SIO_SETCTS;
	strioctl.ic_timout = 0;
	strioctl.ic_len = 0;
	strioctl.ic_dp = (char *)NULL;
	if (ioctl(fd, I_STR, &strioctl) == -1) {
		perror("setting CTS not support on this port");
		exit(1);
	}
}

/*
 * set RTS tracking for this port
 */
static int
setrts()
{
	strioctl.ic_cmd = SIO_SETRTS;
	strioctl.ic_timout = 0;
	strioctl.ic_len = 0;
	strioctl.ic_dp = (char *)NULL;
	if (ioctl(fd, I_STR, &strioctl) == -1) {
		perror("setting RTS not support on this port");
		exit(1);
	}
}

/*
 * set DTR tracking for this port
 */
static int
setdtr()
{
	strioctl.ic_cmd = SIO_SETDTR;
	strioctl.ic_timout = 0;
	strioctl.ic_len = 0;
	strioctl.ic_dp = (char *)NULL;
	if (ioctl(fd, I_STR, &strioctl) == -1) {
		perror("setting DTR not support on this port");
		exit(1);
	}
}

/*
 * set RTS hardware flow for this port
 */
static int
setrtsflow()
{
	strioctl.ic_cmd = SIO_SETRTSFLOW;
	strioctl.ic_timout = 0;
	strioctl.ic_len = 0;
	strioctl.ic_dp = (char *)NULL;
	if (ioctl(fd, I_STR, &strioctl) == -1) {
		perror("setting RTSFLOW not support on this port");
		exit(1);
	}
}

/*
 * set DTR hardware flow for this port
 */
static int
setdtrflow()
{
	strioctl.ic_cmd = SIO_SETDTRFLOW;
	strioctl.ic_timout = 0;
	strioctl.ic_len = 0;
	strioctl.ic_dp = (char *)NULL;
	if (ioctl(fd, I_STR, &strioctl) == -1) {
		perror("setting DTRFLOW not support on this port");
		exit(1);
	}
}

/*
 * reset B38400 baud rate slot to 38400 bits/sec
 */
stdbaud()
{
	int i = 38400;

	strioctl.ic_cmd = SIO_SETBAUD;
	strioctl.ic_timout = 0;
	strioctl.ic_len = sizeof(int);
	strioctl.ic_dp = (char *)&i;

	if (ioctl(fd, I_STR, &strioctl) == -1) {
		perror("stdbaud(): SETBAUD (38400)");
		exit(1);
	}
}

/*
/*
 * set custom baud rate
 */

static int
setbaud()
{
	strioctl.ic_cmd = SIO_SETBAUD;
	strioctl.ic_timout = 0;
	strioctl.ic_len = sizeof(int);
	strioctl.ic_dp = (char *)&custom_baud;
	if (ioctl(fd, I_STR, &strioctl) == -1) {
		perror("setting baud rate not supported on this port");
		exit(1);
	}

	print_maxbaud(custom_baud);
}

/*
 * set timeout value
 */
static int
timeout()
{
	strioctl.ic_cmd = SIO_SETTIMEOUT;
	strioctl.ic_timout = 0;
	strioctl.ic_len = sizeof(int);
	strioctl.ic_dp = (char *)&time_val;
	if (ioctl(fd, I_STR, &strioctl) == -1) {
		perror("setting timeout not supported on this port");
		exit(1);
	}
}

/*
 * Show the current trace options.
 */
show()
{
	int i,j;

	printf("%s settings:\n",devname);

	/*
	 * print baud rate setting for B38400
	 */
	strioctl.ic_cmd = SIO_GETBAUD;
	strioctl.ic_timout = 0;
	strioctl.ic_len = 0;
	strioctl.ic_dp = (char *)&i;
	if (ioctl(fd, I_STR, &strioctl) == -1) {
		perror("show(): SIO_GETBAUD");
		exit(1);
	}
	printf("\tB38400 set to %d bits/sec\n", i);

	/*
	 * print timeout value
	 */
	strioctl.ic_cmd = SIO_GETTIMEOUT;
	strioctl.ic_timout = 0;
	strioctl.ic_len = 0;
	strioctl.ic_dp = (char *)&i;
	if (ioctl(fd, I_STR, &strioctl) == -1) {
		perror("show(): SIO_GETTIMEOUT");
		exit(1);
	}

	if (i == -1) {
		printf("\tclose timeout disabled\n");
	} else {
		printf("\ttimeout = %d seconds\n",i);
	}

	/*
	 * get modem mappings
	 */
	strioctl.ic_cmd = SIO_GETMFLG;
	strioctl.ic_timout = 0;
	strioctl.ic_len = 0;
	strioctl.ic_dp = (char *)&j;
	if (ioctl(fd, I_STR, &strioctl) == -1) {
		perror("show(): SIO_GETMFLG");
		exit(1);
	}

	if (j & SIO_MAPALL) {		/* all pins active */
		printf("\trts, cts, cd, dtr active\n");
	} else {
		if (j & SIO_MAPCD) {
			printf("\tcd, ");
		} else {
			printf("\tcts, ");
		}
		if (j & SIO_MAPDTR) {
			printf("dtr active\n");
		} else {
			printf("rts active\n");
		}
	}
	if (j & SIO_DTRFLOW) {
		printf("\tinput hw flow control pin is DTR (pin 20)\n");
	} else {
		printf("\tinput hw flow control pin is RTS (pin 4)\n");
	}
}

/*
 * parse custom baud rate
 */
int
parse_baud(argc, argv)
	int argc;
	char *argv[];
{
	char *	cp;

	if (argc <= 0)
		return (0);

	cp = *argv;
	if (*cp == '-') cp++;

	if (sscanf(cp, "%d", &custom_baud) != 1)
		usage("invalid baud specification (%s)",cp);

	return (1);
}

/*
 * parse the timeout value
 */
int
parse_timeout(argc, argv)
	int argc;
	char *argv[];
{
	if (argc <= 0)
		return (0);

	if (sscanf(*argv, "%d", &time_val) != 1)
		usage("invalid timeout specification (%s)",*argv);

	return (1);
}

usage(msg)
{
	if (msg) {
		printf("%s", msg);
		printf("\n");
	}

	printf("\nusage:	mset <device file> <options>\n");
	printf("	<options>:	-std\n");
	printf("			-<custom baud rate>\n");
	printf("			-baud <custom baud rate>\n");
	printf("			-rtsflow (ports 0-11)\n");
	printf("			-dtrflow (ports 0-11)\n");
	printf("			-rts (ports 12-15)\n");
	printf("			-cts (ports 12-15)\n");
	printf("			-cd (ports 12-15)\n");
	printf("			-dtr (ports 12-15)\n");
	printf("			-timeout <n secs>\n");
	printf("			-show\n");
        printf("                        -stats\n");
        printf("                        -statsr\n");
        printf("                        -statschk\n");
        printf("                        -statsreset\n");
	exit(1);
}

/*
 * print the maximum baud rate that was set by SETBAUD
 */
print_maxbaud(i)
{
	printf("%s: maximum bits/second = %d\n", devname, i);
}
