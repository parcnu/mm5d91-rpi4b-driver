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
	struct pid_sig_t p_s;
	pid_t pid = getpid();
	p_s.pid = (int)pid;
	int snd_signal = SIGUSR1;
	p_s.sig = (int)snd_signal;

	printf("PROCESS PID:%d\n", pid);

	signal(p_s.sig, signal_handler);
	fd = open("/dev/mm5d91", O_RDWR);
	if (fd < 0) {
		perror("fd failed");
		exit(2);
	}

	ioctl(fd, IOCTL_SET_PID_SIG, &p_s);
	getchar();
	ioctl(fd, IOCTL_SEND_SIGNAL);
	perror("ioctl");
	getchar();

	close(fd);
}