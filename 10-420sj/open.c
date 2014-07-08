#include <fcntl.h>
#include <stdio.h>

extern int errno;

main()
{
	int	fd;

	if ((fd = open("/dev/term/0", O_RDWR)) == -1) {
		perror("open: ");
		exit(errno);
	}

	sleep(30);
	close(fd);
}
