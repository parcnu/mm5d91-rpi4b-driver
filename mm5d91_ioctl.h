#ifndef __IOCTL_CMDS_H
#define __IOCTL_CMDS_H

#define IOCTL_NUMBER        0x21

#define IOCTL_SET_PID       _IOW(IOCTL_NUMBER, 1, unsigned int)
#define IOCTL_SET_SIGNAL    _IOW(IOCTL_NUMBER, 2, unsigned int)
#define IOCTL_SEND_SIGNAL   _IO(IOCTL_NUMBER, 3)

#define IOCTL_MAX_CMDS      3

#endif //__IOCTL_CMDS_H
