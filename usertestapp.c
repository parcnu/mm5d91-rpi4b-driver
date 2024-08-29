#include <stdio.h>
#include <fcntl.h> 
#include <stdlib.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <unistd.h>
#include "mm5d91_ioctl.h"

void signal_handler(int sig)
{
	printf("Signal Received\n");
}

int main(int argc, char *argv[])
{
	int fd;
	pid_t pid = getpid();
	int snd_signal = SIGUSR1;

	printf("PROCESS PID:%d\n", pid);

	signal(SIGUSR1, signal_handler);
	fd = open("/dev/mm5d91", O_RDWR);
	if (fd < 0) {
		perror("fd failed");
		exit(2);
	}

	ioctl(fd, IOCTL_SET_PID, &pid);
	ioctl(fd, IOCTL_SET_SIGNAL, &snd_signal);
	ioctl(fd, IOCTL_SEND_SIGNAL);
	perror("ioctl");
	getchar();

	close(fd);
}