#include <stdio.h>
#include <fcntl.h> 
#include <stdlib.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <unistd.h>
#include "mm5d91_ioctl.h"

unsigned int userbuffer[BUFFER_LENGTH];
int fd;
unsigned char user_buf[BUFFER_LENGTH];
int len;

void signal_handler(int sig)
{
	int32_t cnt;
	printf("Signal Received\n");
	cnt = ioctl(fd, IOCTL_GET_MSG_LEN, (int32_t *) &cnt);
	printf("CNT %i\n", cnt);
	// add read from /dev/mm5d91 here to copy received message to user app.
	read(fd,user_buf,cnt);
	user_buf[10] = '\0';
	for (int i = 0 ; i<cnt; i++)
		printf("i %i, MSG: 0x%02x\n",i, user_buf[i]);

}

int main(int argc, char *argv[])
{
	
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