#ifndef __IOCTL_CMDS_H
#define __IOCTL_CMDS_H

#define BUFFER_LENGTH       128u

#define IOCTL_NUMBER        0x21

#define IOCTL_GET_MSG_LEN   _IOWR(IOCTL_NUMBER, 1, int *)
#define IOCTL_SEND_SIGNAL   _IO(IOCTL_NUMBER, 2)

#define IOCTL_MAX_CMDS      2

#endif //__IOCTL_CMDS_H
