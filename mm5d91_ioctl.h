#ifndef __IOCTL_CMDS_H
#define __IOCTL_CMDS_H


struct pid_sig_t{
    int pid;
    int sig;
};

#define IOCTL_NUMBER        0x21

#define IOCTL_SET_PID_SIG   _IOW(IOCTL_NUMBER, 1, struct pid_sig_t *)
#define IOCTL_SEND_SIGNAL   _IO(IOCTL_NUMBER, 2)

#define IOCTL_MAX_CMDS      2

#endif //__IOCTL_CMDS_H
