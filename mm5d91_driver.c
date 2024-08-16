#include <linux/module.h>
#include <linux/init.h>
#include <linux/serdev.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/cdev.h>
#include "mm5d91_driver.h"


/******************** USER SIDE FUNCTIONS *******************/
int base_minor = 0;
char *device_name = "mm5d91";
int count = 1;
dev_t devicenumber;

module_param(base_minor, int, 0);
module_param(count, int, 0);
module_param(device_name, charp, 0);

static struct class *class = NULL;
static struct device *device = NULL;
static struct cdev mm5d91dev;
static char *userbuf = NULL;

static struct msg_data rx_message = { 
	.byte_index = 0,
	.chr = 0,
	.length = 0,
	.msg_found = 0,
	.msg_type = 0,
	.msg_ready_to_send = 0,
	.uart_device = NULL,
};

static struct msg_data tx_message = { 
	.byte_index = 0,
	.chr = 0,
	.length = 0,
	.msg_found = 0,
	.msg_type = 0,
	.msg_ready_to_send = 0,
	.uart_device = NULL,
};

static int mm5d91_open(struct inode *inode, struct file *file)
{
	pr_info("%s\n", __func__);
	return 0;
}

static int mm5d91_release(struct inode *inode, struct file *file)
{
	pr_info("%s\n", __func__);
    return 0;
}

static ssize_t mm5d91_read(struct file *file, char __user *user_buffer,
                      size_t count, loff_t *offset)
{
	//pr_info("%s\n", __func__);
	//copy_to_user to be added here
    return 0;
}

static struct crc_data crc16(struct msg_data *msg)
{
	
	struct crc_data crc;
	crc.crc_lo = 0xff;
	crc.crc_hi = 0xff;
	uint16_t _crc = 0xFFFF;
	for (int i = 0; i < msg->length; i++)
	{
		_crc = ((uint8_t)(_crc >> 8) | (_crc << 8)) ^ msg->buffer[i];
		_crc ^= (uint8_t)(_crc & 0xFF) >> 4;
		_crc ^= (_crc << 12);
		_crc ^= ((_crc & 0xFF) << 5);
	}
	crc.crc_hi = (unsigned char)(_crc  >> 8);
	crc.crc_lo = (unsigned char)(_crc & 0x00FF);
	printk("CRC: 0x%04x CRC HI: 0x%02x, CRC LO: 0x%02x", _crc, crc.crc_hi, crc.crc_lo);
	msg->buffer[msg->length] = crc.crc_lo;
	msg->buffer[msg->length+1] = crc.crc_hi;
	return (crc);
}

static int construct_tx_message(unsigned char *buf, struct msg_data *message, size_t len)
{
	int cnt = 0;
	unsigned char localbuf[3];
	long chr;
	for (int i = 0; i<len-1; i=i+2) {
		unsigned char one = (unsigned char)buf[i];
		unsigned char sec = (unsigned char)buf[i+1];
		localbuf[0] = one;
		localbuf[1] = sec;
		localbuf[2] = '\0';
		if(!kstrtol(localbuf, 0x10, &chr)){
			message->buffer[cnt] = chr;
		}
		cnt++;
	}
	message->length = cnt;
	crc16(message);
	return 1;
}

static ssize_t mm5d91_write(struct file *file, const char __user *user_buffer,
                       size_t count, loff_t *offset)
{
	pr_info("%s\n", __func__);
	unsigned char buf[BUFFER_LENGTH];
	
	int nbr = copy_from_user(buf, user_buffer, count);
	//message validation to be done here
	construct_tx_message(buf, &tx_message, count);
	if (nbr){
		printk("ISSUE in copying from user");
		return -EFAULT;
	}
	*offset += count;
	nbr = mm5d91_uart_wrt(&tx_message);
	if (nbr < 0)
		return nbr;
    return (ssize_t)count;
}

struct file_operations mm5d91_fops = {
	.read = mm5d91_read,
	.write = mm5d91_write,
	.open = mm5d91_open,
	.release = mm5d91_release
};
/******************** SENSOR SIDE FUNCTIONS *****************/

/* Struct definitions */

static struct of_device_id mm5d91_uart_ids[] = {
	{
		.compatible = "Infineon,mm5d91",
	}, {  }
};

static const struct serdev_device_ops mm5d91_uart_ops = {
	.receive_buf = mm5d91_uart_recv,
	.write_wakeup	= serdev_device_write_wakeup,
};

static struct serdev_device_driver mm5d91_uart_driver = {
	.probe = mm5d91_uart_probe,
	.remove = mm5d91_uart_remove,
	.driver = {
		.name = "mm5d91",
		.of_match_table = mm5d91_uart_ids,
	},
};

/* Device table reference */
MODULE_DEVICE_TABLE(of, mm5d91_uart_ids);

/**
 * @brief Check message type.
 *        
 */
 static int check_message_type(struct msg_data *msg)
 {
	int msg_type = (int)msg->chr;
	int ret = 0;
	switch (msg_type){
		case MSG_TYPE_DETECTION_ON:
			printk("Detection ON");
			ret = 1;
			break;
			
		case MSG_TYPE_DETECTION_OFF:
			printk("Detection OFF");
			ret = 1;
			break;
		case MSG_TYPE_ACK:
			printk("ACK message received");
			ret = 1;
			break;
		default:
			printk("Unknowm message 0x%02x", msg_type);

			ret = 1;
	}
	return ret;
 }


/**
 * @brief Initialize message struct
 *        
 */
 static void initialize_msg(struct msg_data *msg)
 {
	for (int i=0; i<BUFFER_LENGTH; i++) msg->buffer[i] = 0;
	msg->byte_index = 0;
	msg->length = 0;
	msg->msg_type = 0;
	msg->msg_ready_to_send = 0;
 }

/**
 * @brief Construct message.
 *        Get message length from the UART message and create buffer based the len.
 */
 static int construct_message(struct msg_data *msg)
 {
	if (msg->chr == START_BYTE)
	{
		msg->msg_found = 1;
		initialize_msg(msg);
	}
	
	if (msg->msg_found)
	{
		msg->buffer[msg->byte_index] = msg->chr;
		if (msg->byte_index == MSG_TYPE){
			if (!check_message_type(msg)) return -1;
		} else if (msg->byte_index == MSG_LEN_INDEX) { 
			msg->length = (int)(msg->chr)+CRC_LEN;
			if ((msg->length) >= BUFFER_LENGTH) {
				printk("UART Buffer size exceeded and message skipped");
				msg->length=0;
				return 0;
			}
		}
		
		if ((msg->length)+CRC_LEN == msg->byte_index)
		{
			if (userbuf){
				int cnt = copy_to_user(userbuf,msg->buffer, msg->length );
				if (!cnt) return -1;
			}
			msg->msg_found = 0;
			msg->length = 0;
			msg->msg_ready_to_send = 0;
		}

		(msg->byte_index)++;
	}	
	return 1;
 }

/**
 * @brief Callback is called whenever a character is received from the radar
 */
static int mm5d91_uart_recv(struct serdev_device *mm5d91, const unsigned char *buffer, size_t size) {
	rx_message.chr = (char)buffer[0];
	if (construct_message(&rx_message)){
		return 1;
	} else {
		// add correct error and error code here
		printk("Error happened.");
		return -1;
	}
}

static int mm5d91_uart_wrt(struct msg_data * msg) {
	if (!msg->uart_device){
		printk("issue in serdev_device = NULL");
		//Add correct error code here to be returned
		return -1;
	}
	
	int ret = serdev_device_write(msg->uart_device, msg->buffer, (size_t)msg->length+CRC_LEN, 0);
	if (ret < 0 || ret < count)
		return ret;
	serdev_device_wait_until_sent(msg->uart_device, 0);
	for (int i = 0; i<msg->length; i++)
		printk("RET: %i BYTE: %c", ret, msg->buffer[i]);
	return ret;
}

/**
 * @brief This function is called on loading the driver 
 * 		  Initializing parameters for mm5d91 radar UART communication
 */
static int mm5d91_uart_probe(struct serdev_device *mm5d91) {
	int status;
	serdev_device_set_client_ops(mm5d91, &mm5d91_uart_ops);
	status = serdev_device_open(mm5d91);

	if(status) {
		printk("mm5d91 - Error opening serial port!\n");
		return -status;
	}
	serdev_device_set_baudrate(mm5d91, 115200);
	serdev_device_set_flow_control(mm5d91, false);
	serdev_device_set_parity(mm5d91, SERDEV_PARITY_NONE);
	tx_message.uart_device = (struct serdev_device*)mm5d91;
	return 0;
}

/**
 * @brief This function is called on unloading the driver 
 */
static void mm5d91_uart_remove(struct serdev_device *mm5d91) {
	serdev_device_close(mm5d91);
}

/**
 * @brief This function is called, when the module is loaded into the kernel
 * 		  Creates class
 * 		  allocates device number
 * 		  defines file operations and callbacks
 * 		  creates character device
 * 		  registers serdev driver for mm5d91 communication
 */
static int __init mm5d91_uart_init(void) {
	int ret = alloc_chrdev_region(&devicenumber, base_minor, count, device_name);
	if (!ret) {
		printk("Device number registered\n");
		printk("Major number received:%d\n", MAJOR(devicenumber));
		class = class_create("mm5d91");
		device = device_create(class, NULL, devicenumber, NULL, device_name);
		cdev_init(&mm5d91dev, &mm5d91_fops);
		mm5d91dev.owner = THIS_MODULE;
		cdev_add(&mm5d91dev, devicenumber, count);
		
	} else {
		printk("Device number registration Failed\n");
		return -ret;
	}

	ret = serdev_device_driver_register(&mm5d91_uart_driver);
	if(ret) {
		printk("mm5d91 - Error! Could not load driver\n");
		return -ret;
	}
	printk("mm5d91 - Driver loaded %d\n", ret);
		
	return 0;
}

/*
* Function to be called when driver is removed from kernel.
*/
static void __exit mm5d91_uart_exit(void) {
	printk("mm5d91 - Unloading driver");
	device_destroy(class, devicenumber);
    class_destroy(class);
	cdev_del(&mm5d91dev);
	unregister_chrdev_region(devicenumber, count);
	serdev_device_driver_unregister(&mm5d91_uart_driver);
}

module_init(mm5d91_uart_init);
module_exit(mm5d91_uart_exit);

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("jaakko.saarela@parcnu.fi");
MODULE_DESCRIPTION("RPi4 Driver for MM5D91 FMCW radar");

