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

*/

/*

	file:	ttysoftcar
	author: dmostrom
	created:931027

*/
/*
 * $Log: ttysoftcar.c,v $
 * Revision 1.3  1996/09/20 21:25:55  bkd
 * Changed copyrights from 1994 and 1994-1995 to 1993-1996
 *
 * Revision 1.2  1995/08/15 23:14:29  bkd
 * Updated copyrights; added CVS control information
 *
 * Revision 1.1  1995/07/20  13:43:44  chung
 * Initial revision
 *
 * Revision 1.1  1994/12/09  14:35:22  kozin
 * add ttysoftcar to distribution.
 *
*/

static char CopyId[] = "@(#)Copyright (c) 1993-1996 Aurora Technologies, Inc.";
static char SccsId[] = "@(#)$Header: /vol/sources.cvs/dev/sio16.svr4/ttysoftcar.c,v 1.3 1996/09/20 21:25:55 bkd Exp $";

/* main.c  aurora technologies */
/* last modification: 931027 dmostrom */

#include	<stdio.h>
#include	<fcntl.h>
#include	<errno.h>
#include	<termios.h>
#include	<unistd.h>

#define	TRUE	1
#define	FALSE	!TRUE

static void usage(prog, s)
char *prog;
char *s;
{
	printf("%s\n",s);
	printf("usage:\t%s [-y|-n] [port]\n", prog);
	printf(" e.g.:\t%s -y term/0\n", prog);
	printf("\t-y = set software carrier mode to on\n");
	printf("\t-n = set software carrier mode to off (i.e.");
	printf(" hardware carrier detect)\n");
	exit(1);
}

main(argc, argv)
        int             argc;
        char           *argv[];

{
	int             fd_port;		/* serial port fd's */
	int		mystdin = 0;
	char		*thisarg;
	char		*myinname = "stdin";
	char            portname[20];		/* names of the port */
	char		c;
	int		carrier = 0;
	int		setcar = FALSE;
	int		foundsw = FALSE;
	int		namefound = FALSE;
	int		ii = 0;			/* return result code */
	char		*progname;

	/* make sure the parameters are entered correctly */

	if (argc <= 1) usage(argv[0],"");

	progname = argv[0];
	while (--argc)
	{
		thisarg = *++argv;
		c = *thisarg;
		switch (c)
		{
		case '-':
			if (foundsw)
			{
			    usage(argv[0],"Only one setting (-y or -n) allowed");
			}
			foundsw = TRUE;
			switch (*++thisarg)
			{
				case 'n':
				case 'N':
					carrier = 0;
					setcar = TRUE;
					break;
				case 'y':
				case 'Y':
					carrier = 1;
					setcar = TRUE;
					break;
				default:
					usage(argv[0],"invalid setting");
			}
			break;
		default:
			if (namefound)
			{
				usage(argv[0],"Only one port name allowed\n");
			}
			namefound = TRUE;
			if (strlen(thisarg) <= 2)
			{
				strcpy(portname, "/dev/term/");
				strcpy(portname + 10, thisarg);
			}
			else if (!strncmp(thisarg,"term",4))
			{
				strcpy(portname, "/dev/");
				strcpy(portname + 5, thisarg);
			}
			else if (!strncmp(thisarg,"cua",3))
			{
				strcpy(portname, "/dev/");
				strcpy(portname + 5, thisarg);
			}
			else
			{
				strcpy(portname, thisarg);
			}
			break;
		}
	}

	/* open the port to be tested */
	if (namefound)
	{
		fd_port = open(portname, O_RDWR | O_NDELAY);
		if (fd_port == -1)
		{
			fprintf(stderr, "cannot open %s\n", portname);
			perror("Open error");
			exit(errno);
		}
	}
	else
	{
		strcpy(portname, myinname);
		fd_port = dup(mystdin);
		if (fd_port == -1)
		{
			fprintf(stderr, "cannot open stdin\n");
			perror("Open error");
			exit(errno);
		}
	}

	if (setcar)
	{
		/* set up the port with ttysoftcar */
		ii = ioctl(fd_port, TIOCSSOFTCAR, &carrier);
		if (ii == -1)
		{
			fprintf(stderr, "error setting TIOCSSOFTCAR \n");
			perror("ioctl error");
			exit(errno);
		}

	}
	else
	{
		/* get the current value of ttysoftcar */
		ii = ioctl(fd_port, TIOCGSOFTCAR, &carrier);
		if (ii == -1)
		{
			fprintf(stderr, "error setting TIOCGSOFTCAR \n");
			perror("ioctl error");
			exit(errno);
		}

		printf("%s is %c\n", portname, carrier ? 'y' : 'n');
	}

	close(fd_port);
	exit(0);
}
