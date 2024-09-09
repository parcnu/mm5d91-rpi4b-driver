#include <linux/module.h>
#include <linux/init.h>
#include <linux/serdev.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/cdev.h>
#include <linux/sched/signal.h>
#include "mm5d91_driver.h"
#include "mm5d91_ioctl.h"

static unsigned int mm5d91_reserved = 0;
static int sig_pid = 0;
static struct task_struct *sig_tsk = NULL;
static int sig_tosend = SIGUSR1;

/******************** USER SIDE FUNCTIONS *******************/
int base_minor = 0;
char *device_name = "mm5d91";
int count = 1;
dev_t devicenumber;

module_param(base_minor, int, 0);
module_param(count, int, 0);
module_param(device_name, charp, 0);

static struct class *mm5d91class = NULL;
static struct device *device = NULL;
static struct cdev mm5d91dev;

static char kernel_buffer[BUFFER_LENGTH];
static int kernel_buffer_len = 0;

static struct msg_data_t rx_message = { 
	.byte_index = 0,
	.chr = 0,
	.length = 0,
	.msg_found = 0,
	.msg_type = 0,
	.msg_ready_to_send = 0,
	.uart_device = NULL,
};

static struct msg_data_t tx_message = { 
	.byte_index = 0,
	.chr = 0,
	.length = 0,
	.msg_found = 0,
	.msg_type = 0,
	.msg_ready_to_send = 0,
	.uart_device = NULL,
};

/******************* HELPER FUNCTIONS *******************/

/**
 * @brief Calculates CRC16 sum for message. Is used to add crc code to messages that will be send
 *        but also to be cheked crc from receved messages from sensor.
 *        
 */
static struct crc_data_t crc16(struct msg_data_t *msg)
{
	
	struct crc_data_t crc;
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
	msg->buffer[msg->length] = crc.crc_lo;
	msg->buffer[msg->length+1] = crc.crc_hi;
	return (crc);
}

/**
 * @brief Creates tx message. Received message is string and before sensding it to sensor, 
 *        convertion to long is required byte by byte.
 *        
 */
static int construct_uart_tx_message(unsigned char *buf, struct msg_data_t *message, size_t len)
{
	int cnt = 0;
	unsigned char localbuf[3];
	long chr;
	for (int i = 0; i<len-1; i=i+2) {
		localbuf[0] = (unsigned char)buf[i];
		localbuf[1] = (unsigned char)buf[i+1];
		localbuf[2] = '\0';
		if(!kstrtol(localbuf, DEG_BASE, &chr)){
			message->buffer[cnt] = chr;
		}
		cnt++;
	}
	message->length = cnt;
	crc16(message);
	return 1;
}

/************* USER SPACE FUNCTIONS **********************/

static int mm5d91_open(struct inode *inode, struct file *file)
{
	//pr_info("%s\n", __func__);
	if (!mm5d91_reserved){
		mm5d91_reserved++;
		sig_pid = task_pid_nr(current);
		sig_tsk = pid_task(find_vpid(sig_pid), PIDTYPE_PID);
	} else {
		return -EBUSY;
	}
	return 0;
}

static int mm5d91_release(struct inode *inode, struct file *file)
{
	//(pr_info("%s\n", __func__);
	mm5d91_reserved = 0;
	sig_pid = 0;
	sig_tsk = NULL;
    return 0;
}

/**
 * @brief Is called from user space when getting data from sensor to user app.
 *        
 */
static ssize_t mm5d91_read(struct file *file, char __user *user_buffer,
                      size_t count, loff_t *offset)
{
	//(pr_info("%s\n", __func__);	
	int ret = -EFAULT;
	int ok = access_ok(user_buffer, count);
	if (ok){
		ret = copy_to_user(user_buffer, kernel_buffer,count);
	} else {
		return -EACCES;
	}
    return ret;
}

/**
 * @brief Function to called from user space when sending commad to sensor
 *        
 */
static ssize_t mm5d91_write(struct file *file, const char __user *user_buffer,
                       size_t count, loff_t *offset)
{
	//pr_info("%s\n", __func__);
	unsigned char buf[BUFFER_LENGTH];
	int nbr = copy_from_user(buf, user_buffer, count);
	construct_uart_tx_message(buf, &tx_message, count);
	if (nbr){
		pr_info("MM5D91: ISSUE in copying from user");
		return -EFAULT;
	}
	*offset += count;
	nbr = mm5d91_uart_wrt(&tx_message);
	if (nbr < 0)
		return nbr;
    return (ssize_t)count;
}

/**
 * @brief ioctl function. 
 * Gets pid from user space process and saves it to global variable.
 */
static ssize_t mm5d91_ioctl(struct file *file,  unsigned int cmd, unsigned long arg) {
	if (_IOC_TYPE(cmd) != IOCTL_NUMBER) return -ENOTTY;
	if (_IOC_NR(cmd) > IOCTL_MAX_CMDS) return -ENOTTY;
	
	switch(cmd)
	{
		case IOCTL_SEND_SIGNAL:	// to be used for future purposes
			break;
		case IOCTL_GET_MSG_LEN:
			return (kernel_buffer_len);
			break;
		default:
			pr_info("MM5D91: Unknown Command:%u\n", cmd);
			return -ENOTTY;
	}
	return 0;
}

struct file_operations mm5d91_fops = {
	.read = mm5d91_read,
	.write = mm5d91_write,
	.open = mm5d91_open,
	.release = mm5d91_release,
	.unlocked_ioctl = mm5d91_ioctl
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
 * @brief Set RW permissions for created device
 */
static int mm5d91_uevent(struct device *dev, struct kobj_uevent_env *env)
{
    add_uevent_var(env, "DEVMODE=%#o", 0666);
    return 0;
}

/**
 * @brief Check message len
 *        
 */
static int check_message_len(struct msg_data_t *msg){
	return msg->length;
}

/**
 * @brief Check message type.
 *        
 */
 static int check_message_type(struct msg_data_t *msg)
 {
	int msg_type = (int)msg->buffer[MSG_TYPE_INDEX];
	int ret = 0;
	switch (msg_type){
		//case MSG_TYPE_VERSION:
		case MSG_TYPE_ACK:
		case MSG_TYPE_MAX_RNG:
		case MSG_TYPE_SENSITIVITY:
		case MSG_TYPE_DETECTION_OUTPUT_STATUS:
		case MSG_TYPE_DETECTION_ON:
		case MSG_TYPE_DETECTION_OFF:
		case MSG_TYPE_DETECTION_STATUS:
		case MSG_TYPE_DETECTION_ENABLE:
		case MSG_TYPE_CIP_TEMP:
		case MSG_TYPE_CALIB_MODE:
		case MSG_TYPE_CALIB_MESSAGE:
		case MSG_TYPE_CALIB_OUTPUT_RATE:
		case MSG_TYPE_MIN_RNG:
		case MSG_TYPE_MACRO_THREHOLD:
		case MSG_TYPE_MICRO_THRESHOLD:
		case MSG_TYPE_MACRO_VALID:
		case MSG_TYPE_MICRO_VALID:
		case MSG_TYPE_DETECTION_MODE:
		case MSG_TYPE_MACRO_DETECTION_TRIGGER_RANGE:
		case MSG_TYPE_MACRO_DETECTION_TRIGGER_DELAY:
		case MSG_TYPE_CHIRP_PER_FRAME:
			ret = MSG_OK;
			break;
		default:
			//pr_info("MM5D91: Unknown message 0x%02x", msg_type);
			ret = MSG_NOK;
			break;
	}
	return ret;
 }


/**
 * @brief Initialize message struct
 *        
 */
 static void initialize_msg(struct msg_data_t *msg)
 {
	//for (int i=0; i<BUFFER_LENGTH; i++) msg->buffer[i] = 0;
	msg->msg_found = false;
	msg->byte_index = 0;
	msg->length = 0;
	msg->msg_type = 0;
	msg->msg_ready_to_send = false;
 }

/**
 * @brief check_crc calulates crc value from the message and compares that with received values.
 * 
 */
static int check_crc(struct msg_data_t * msg){
	struct msg_data_t cp_msg = {
		.length = 0,
	};
	for (int i = 0; i < msg->length-CRC_LEN; i++){
		cp_msg.buffer[i] = msg->buffer[i];
		cp_msg.length = i;
	}
	cp_msg.length++;
	crc16(&cp_msg);
	cp_msg.length=cp_msg.length+CRC_LEN;
	if (msg->buffer[msg->length - 2] == cp_msg.buffer[cp_msg.length-2] &&
		msg->buffer[msg->length-1] == cp_msg.buffer[cp_msg.length-1]){
			return 1;
	}
	return 0;
}

/**
 * @brief Construct message.
 *        Get message length from the UART message and create buffer based on the len.
 * @return true if message construction success, false if issues happened. 
 */
 static int construct_message(struct msg_data_t *msg)
 {
	int nok = 0;
	if (msg->chr == START_BYTE && !msg->msg_found)
	{
		initialize_msg(msg);
		msg->msg_found = true;
	}
	
	if (msg->msg_found)
	{
		msg->buffer[msg->byte_index] = msg->chr;
		if (msg->byte_index == MSG_LEN_INDEX) { 
			msg->length = (int)(msg->buffer[msg->byte_index])+CRC_LEN + MSG_HEADER_LEN;
			if ((msg->length) >= BUFFER_LENGTH) {
				initialize_msg(msg);
				return 0;
			}
		}
		
		if ((msg->length) == msg->byte_index+1 && msg->byte_index > 0)
		{
			if (!check_message_type(msg) || !check_message_len(msg)){
				initialize_msg(msg);
				return 0;
			} else {
				msg->msg_ready_to_send = true;
			}
		}

		if (msg->msg_ready_to_send){
			if (!check_crc(msg)) {
				initialize_msg(msg);
				return 0;
			}
			kernel_buffer_len = msg->length;
			for (int i = 0; i <= kernel_buffer_len; i++){
				kernel_buffer[i] = msg->buffer[i];
			}
			initialize_msg(msg);
			if (sig_tsk){
				nok = send_sig(sig_tosend, sig_tsk, 0);
				if (nok) {
					pr_info("MM5D91: error sending signal to USER PID %d", sig_tosend);
					return nok;
				}
			}
			return 1;
		}
		(msg->byte_index)++;
	}	
	return 1;
 }

/**
 * @brief Callback is called whenever a character is received from the radar
 */
static int mm5d91_uart_recv(struct serdev_device *mm5d91, const unsigned char *buffer, size_t size) {
	rx_message.chr = (unsigned char)buffer[0];
	if (construct_message(&rx_message)){
		return size;
	} else {
		return -EFAULT;
	}
}

/**
 * @brief Function which wraps the serdev sending function
 *        
 */
static int mm5d91_uart_wrt(struct msg_data_t * msg) {
	if (!msg->uart_device){
		pr_info("MM5D91: serdev_device = NULL");
		return -EFAULT;
	}
	
	int ret = serdev_device_write(msg->uart_device, msg->buffer, (size_t)msg->length+CRC_LEN, 0);
	if (ret < 0 || ret < count)
		return ret;
	serdev_device_wait_until_sent(msg->uart_device, 0);
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
		pr_info("MM5D91: Error opening serial port!\n");
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
		pr_info("MM5D91: Driver loaded\n");
		//printk("Major number received:%d\n", MAJOR(devicenumber));
		mm5d91class = class_create(THIS_MODULE, "mm5d91");
		mm5d91class->dev_uevent = mm5d91_uevent;
		if (IS_ERR(mm5d91class))
                return PTR_ERR(mm5d91class);
		device = device_create(mm5d91class, NULL, devicenumber, NULL, device_name);
		if (IS_ERR(device))
                return PTR_ERR(device);
		cdev_init(&mm5d91dev, &mm5d91_fops);
		mm5d91dev.owner = THIS_MODULE;
		cdev_add(&mm5d91dev, devicenumber, count);
		
	} else {
		pr_info("MM5D91: Device number registration Failed\n");
		return -ret;
	}

	ret = serdev_device_driver_register(&mm5d91_uart_driver);
	if(ret) {
		pr_info("MM5D91: Error! Could not load driver\n");
		return -ret;
	}		
	return 0;
}

/*
* Function to be called when driver is removed from kernel.
*/
static void __exit mm5d91_uart_exit(void) {
	pr_info("MM5D91: Driver removed");
	device_destroy(mm5d91class, devicenumber);
    class_destroy(mm5d91class);
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

