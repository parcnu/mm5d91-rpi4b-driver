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
static int calc = 0;

void signal_handler(int sig)
{
	int32_t cnt;
	calc++;
	printf("Signal Received %i\n", calc);
	cnt = ioctl(fd, IOCTL_GET_MSG_LEN, (int32_t *) &cnt);
	printf("CNT %i\n", cnt);
	read(fd,user_buf,cnt);
	user_buf[10] = '\0';
	for (int i = 0 ; i<cnt; i++)
		printf("i %i, MSG: 0x%02x\n",i, user_buf[i]);

}

int main(int argc, char *argv[])
{
	signal(SIGUSR1, signal_handler);
	fd = open("/dev/mm5d91", O_RDWR);
	if (fd < 0) {
		perror("fd failed");
		exit(2);
	}

	//ioctl(fd, IOCTL_SET_PID_SIG, &p_s);
	getchar();
	ioctl(fd, IOCTL_SEND_SIGNAL);
	perror("ioctl");
	getchar();

	close(fd);
}