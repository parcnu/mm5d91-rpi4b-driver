
#ifndef MM5D91DRIVER_H
#define MM5D91DRIVER_H

#include <linux/module.h>
#include <linux/init.h>
#include "mm5d91_ioctl.h"

#define DEG_BASE 0x10
#define START_BYTE 0xd9
#define MSG_TYPE_DETECTION_ON 0x06
#define MSG_TYPE_DETECTION_OFF 0x07
#define MSG_TYPE_ACK 0x02
#define MSG_LEN_INDEX 0x02
#define MSG_TYPE_INDEX 0x01

#define MSG_ACK_VALUE 5

#define CRC_LEN 2
#define MSG_HEADER_LEN 4

struct msg_data_t{
    unsigned char chr;
    int msg_found; 
    int byte_index;
    int length;
    unsigned char buffer[BUFFER_LENGTH];
    int msg_type;
    int msg_ready_to_send;
    struct serdev_device *uart_device;
    struct cdev user_device;
};

struct crc_data_t{
    unsigned char crc_lo;
    unsigned char crc_hi;
};

int buff[128];
static int mm5d91_uart_recv(struct serdev_device *mm5d91, const unsigned char *buffer, size_t size);
static int mm5d91_uart_probe(struct serdev_device *mm5d91);
static void mm5d91_uart_remove(struct serdev_device *mm5d91);
static int mm5d91_uart_wrt(struct msg_data_t *msg);
static int __init mm5d91_uart_init(void);
static void __exit mm5d91_uart_exit(void);
static int construct_message(struct msg_data_t *msg);
static int check_message_type(struct msg_data_t *msg);
static void initialize_msg(struct msg_data_t *msg);

static int construct_uart_tx_message(unsigned char *buf, struct msg_data_t *message, size_t len);

static int mm5d91_open(struct inode *inode, struct file *file);
static int mm5d91_release(struct inode *inode, struct file *file);
static ssize_t mm5d91_read(struct file *file, char __user *user_buffer,
                           size_t count, loff_t *offset);
static ssize_t mm5d91_write(struct file *file, const char __user *user_buffer,
                            size_t count, loff_t *offset);
static ssize_t mm5d91_ioctl(struct file *file,  unsigned int cmd, unsigned long arg);

#endif //MM5D91DRIVER_H
