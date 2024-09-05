#ifndef __IOCTL_CMDS_H
#define __IOCTL_CMDS_H


// struct pid_sig_t{
//     int pid;
//     int sig;
// };
#define BUFFER_LENGTH       128u

#define IOCTL_NUMBER        0x21

//#define IOCTL_SET_PID_SIG   _IOW(IOCTL_NUMBER, 1, struct pid_sig_t *)
#define IOCTL_GET_MSG_LEN   _IOWR(IOCTL_NUMBER, 2, int *)
#define IOCTL_SEND_SIGNAL   _IO(IOCTL_NUMBER, 3)

#define IOCTL_MAX_CMDS      3

#endif //__IOCTL_CMDS_H
