#include <linux/init.h>		/* modules */
#include <linux/module.h>	/* module */
#include <linux/types.h>	/* dev_t type */
#include <linux/fs.h>		/* chrdev allocation */
#include <linux/kernel.h>
#include <linux/slab.h>		/* kmalloc() */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/uaccess.h>	/* copy_from/to_user */
#include <linux/io.h>
#include <linux/acp_ncr.h>
#include "ncr_cd.h"


/* Will contain the device major and minor numbers */
dev_t dev;
/* Is device open?  Used to prevent multiple access to the device */
static int DeviceOpen = 0;
static unsigned char *kbuf = NULL;
static ncr_ioc_transfer tr;

static int device_open(struct inode *inode, struct file *file);
static int device_release(struct inode *inode, struct file *file);
static long device_ioctl(struct file *file,
			 unsigned int cmd,
			 unsigned long arg);

/* Structure that declares the common */
/* file access fcuntions */
struct file_operations ncr_fops = {
	.open = device_open,
	.unlocked_ioctl = device_ioctl,
	.release = device_release
};

/* Modules */
/* Called when a process tries to open the device file, like
 * "cat /dev/mycharfile"
 */
static int device_open(struct inode *inode, struct file *file)
{
	if (0 != DeviceOpen)
		return -EBUSY;

	DeviceOpen++;

	kbuf = kmalloc(NCR_BUFSZ, GFP_KERNEL);
	if (NULL == kbuf) {
		printk(KERN_WARNING "<1>kmalloc failed");
		return -ENOMEM;
	}

	return 0;
}

/* Called when a process closes the device file */
static int device_release(struct inode *inode, struct file *file)
{
	DeviceOpen--; /* We're now ready for our next caller */

	if (NULL != kbuf)
		kfree(kbuf);

	return 0;
}

static long device_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int retval = 0;

	memset(&tr, 0, sizeof(tr));

	switch (cmd) {
	case NCRWRITE32:
		/* Get the struct ncr_ioc_transfer */
		if (copy_from_user((void *) &tr, (void *) arg, sizeof(tr))) {
			printk(NCR_NAME ": copy_from_user failed\n");
			return -EFAULT;
		}

		/* Copy the data to write */
		if (copy_from_user(kbuf, tr.buffer, sizeof(long))) {
			printk(NCR_NAME ": copy_from_user failed\n");
			return -EFAULT;
		}

		ncr_write(tr.region, tr.offset, sizeof(long),
			  (unsigned long *)kbuf);
		break;

	case NCRREAD32:
		/* Get the struct mem_ioc_transfer */
		if (copy_from_user((void *) &tr, (void *) arg, sizeof(tr))) {
			printk(NCR_NAME ": get struct copy_from_user failed\n");
			return -EFAULT;
		}

		ncr_read(tr.region, tr.offset, sizeof(long),
			 (unsigned long *)kbuf);

		copy_to_user(tr.buffer, (void *) kbuf, sizeof(long));
		break;

	default:
		retval = -EINVAL;
	}
	return retval;

}

static int ncr_module_init(void)
{
	int result = register_chrdev(NCR_MAJOR, NCR_NAME, &ncr_fops);
	if (result < 0)
		printk(NCR_NAME ": cannot obtain major number %d\n", NCR_MAJOR);

	/* alloc_chrdev_region return 0 on success */
/*
	int res = alloc_chrdev_region(&dev,NCR_MINOR_START,
				      NCR_COUNT, NCR_NAME);
	if (res)
		printk(KERN_WARNING "ncr: could not allocate device\n");
	else
		printk(KERN_WARNING "ncr: registered with major number:%i\n",
		       MAJOR(dev));
	return res;
*/
	return result;
}

module_init(ncr_module_init);

/*
  ----------------------------------------------------------------------
  ncr_module_exit
*/

void ncr_module_exit(void)
{
	/* Free the devices */
	/* unregister_chrdev_region(dev, NCR_COUNT); */
	unregister_chrdev(NCR_MAJOR, NCR_NAME);
}

module_exit(ncr_module_exit);

MODULE_AUTHOR("/// Corporation");
MODULE_DESCRIPTION("NCR PPC476 driver");
MODULE_LICENSE("GPL");


