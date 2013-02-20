/*
 * PPC476 board specific routines
 *
 * Copyright 2009 Torez Smith, IBM Corporation.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

/*
 * Simple block device for the ISS simulator
 */

#undef DEBUG

#include <linux/major.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/file.h>
#include <linux/ioctl.h>
#include <linux/blkdev.h>
#include <linux/of.h>

#include <linux/io.h>

#define MAJOR_NR		63	/* FIXME */
#define NUM_ISS_BLK_MINOR	4

/* Command codes */
enum {
	ISS_BD_CMD_NOP		= 0,
	ISS_BD_CMD_OPEN		= 1,
	ISS_BD_CMD_CLOSE	= 2,
	ISS_BD_CMD_READ		= 3,
	ISS_BD_CMD_WRITE	= 4,
	ISS_BD_CMD_STATUS	= 5,
	ISS_BD_CMD_CHKCHANGE	= 6,
	ISS_BD_CMD_SYNC		= 7,
	ISS_BD_CMD_GET_BLKSIZE	= 8,
	ISS_BD_CMD_GET_DEVSIZE	= 9,
};

/* Status codes */
enum {
	ISS_BD_STATUS_OK	= 0,
	ISS_BD_STATUS_OP_ER	= 1,	/* Open error			  */
	ISS_BD_ALREADY_OPEN	= 2,	/* Block file already open	  */
	ISS_BD_NOT_OPEN		= 3,	/* Block file not open		  */
	ISS_BD_BAD_DEV_NUM	= 4,	/* Bad device number		  */
	ISS_BD_BAD_SEC_CNT	= 5,	/* Bad sector number		  */
	ISS_BD_SEEK_ERROR	= 6,	/* Bad sector count		  */
	ISS_BD_RW_ERROR		= 7,	/* Read/Write error		  */
	ISS_BD_SIZE_ERROR	= 8,	/* Unable to determine file size  */
	ISS_BD_FILE_CHANGED	= 9,	/* Media has changed		  */
};

struct iss_blk_regs {
	u8	cmd;
	u8	pad0[3];
	u32	stat;
	u32	sector;
	u32	count;
	u32	devno;
	u32	size;
	u8	pad1[0x1e8];
	u8	data[0x800];
};

struct iss_blk {
	struct gendisk		*disk;
	unsigned int		devno;
	unsigned int		sectsize;
	unsigned int		capacity;
	unsigned int		present;
	unsigned int		changed;
} iss_blks[NUM_ISS_BLK_MINOR];

static spinlock_t			iss_blk_qlock;
static spinlock_t			iss_blk_reglock;
static struct iss_blk_regs __iomem	*iss_blk_regs;

struct request *iss_req;
static bool iss_end_request(int err)
{
	unsigned int bytes = blk_rq_cur_bytes(iss_req);

	if (__blk_end_request(iss_req, err, bytes))
		return true;
	iss_req = NULL;
	return false;
}

static void iss_blk_setup(struct iss_blk *ib)
{
	unsigned long flags;
	u32 stat;

	pr_debug("iss_blk_setup %d\n", ib->devno);

	spin_lock_irqsave(&iss_blk_reglock, flags);
	out_8(iss_blk_regs->data, 0);
	out_be32(&iss_blk_regs->devno, ib->devno);
	out_8(&iss_blk_regs->cmd, ISS_BD_CMD_OPEN);
	stat = in_be32(&iss_blk_regs->stat);
	if (stat != ISS_BD_STATUS_OK) {
		pr_debug(" -> no file\n");
		goto failed;
	}
	out_8(&iss_blk_regs->cmd, ISS_BD_CMD_GET_BLKSIZE);
	ib->sectsize = in_be32(&iss_blk_regs->size);
	if (ib->sectsize != 512) {
		pr_err("issblk: unsupported sector size %d\n", ib->sectsize);
		goto failed;
	}
	out_8(&iss_blk_regs->cmd, ISS_BD_CMD_GET_DEVSIZE);
	ib->capacity = in_be32(&iss_blk_regs->size);
	ib->present = 1;
	ib->changed = 0;
	spin_unlock_irqrestore(&iss_blk_reglock, flags);

	pr_debug(" -> 0x%x sectors 0f %d bytes\n",
		 ib->capacity, ib->sectsize);

	blk_queue_bounce_limit(ib->disk->queue, BLK_BOUNCE_HIGH);
	blk_queue_logical_block_size(ib->disk->queue, ib->sectsize);
	set_capacity(ib->disk, ib->capacity);
	return;

 failed:
	spin_unlock_irqrestore(&iss_blk_reglock, flags);
}

static int __iss_blk_read(struct iss_blk *ib, void *buffer,
			  unsigned long sector, unsigned long count)
{
	unsigned long lcount, flags;
	u32 stat;

	pr_debug("__iss_blk_read 0x%ld sectors @ 0x%lx\n", count, sector);

	while (count) {
		lcount = min(count, 4ul);
		spin_lock_irqsave(&iss_blk_reglock, flags);
		out_be32(&iss_blk_regs->devno, ib->devno);
		out_be32(&iss_blk_regs->sector, sector);
		out_be32(&iss_blk_regs->count, lcount);
		out_8(&iss_blk_regs->cmd, ISS_BD_CMD_READ);
		stat = in_be32(&iss_blk_regs->stat);
		if (stat != ISS_BD_STATUS_OK) {
			spin_unlock_irqrestore(&iss_blk_reglock, flags);
			return -EIO;
		}
		memcpy_fromio(buffer, &iss_blk_regs->data,
			lcount * ib->sectsize);
		spin_unlock_irqrestore(&iss_blk_reglock, flags);
		count -= lcount;
		sector += lcount;
		buffer += lcount * ib->sectsize;
	}
	return 0;
}

static int __iss_blk_write(struct iss_blk *ib, void *buffer,
			   unsigned long sector, unsigned long count)
{
	unsigned long lcount, flags;
	u32 stat;

	pr_debug("__iss_blk_write 0x%ld sectors @ 0x%lx\n", count, sector);

	while (count) {
		lcount = min(count, 4ul);
		spin_lock_irqsave(&iss_blk_reglock, flags);
		out_be32(&iss_blk_regs->devno, ib->devno);
		out_be32(&iss_blk_regs->sector, sector);
		out_be32(&iss_blk_regs->count, lcount);
		memcpy_toio(&iss_blk_regs->data, buffer, lcount * ib->sectsize);
		out_8(&iss_blk_regs->cmd, ISS_BD_CMD_WRITE);
		stat = in_be32(&iss_blk_regs->stat);
		spin_unlock_irqrestore(&iss_blk_reglock, flags);
		if (stat != ISS_BD_STATUS_OK)
			return -EIO;
		count -= lcount;
		sector += lcount;
		buffer += lcount * ib->sectsize;
	}
	return 0;
}

static void iss_blk_do_request(struct request_queue *q)
{
	struct iss_blk *ib = q->queuedata;
	int rc = 0;

	pr_debug("iss_do_request dev %d\n", ib->devno);

	while (iss_req || ((iss_req = blk_fetch_request(q)) != NULL)) {
		pr_debug(" -> req @ %p, changed: %d\n", iss_req, ib->changed);
		if (ib->changed) {
			iss_end_request(-EIO);	/* failure */
			continue;
		}
		switch (rq_data_dir(iss_req)) {
		case READ:
			rc = __iss_blk_read(ib, iss_req->buffer,
				blk_rq_pos(iss_req),
					blk_rq_cur_sectors(iss_req));
			break;
		case WRITE:
			rc = __iss_blk_write(ib, iss_req->buffer,
				blk_rq_pos(iss_req),
					blk_rq_cur_sectors(iss_req));
		};

		pr_debug(" -> ending request, rc = %d\n", rc);
		if (rc)
			iss_end_request(-EIO);
		else
			iss_end_request(0);
	}
}

static int iss_blk_release(struct gendisk *disk, fmode_t mode)
{
	struct iss_blk *ib = disk->private_data;
	unsigned long flags;

	pr_debug("issblk%d: release !\n", disk->first_minor);

	spin_lock_irqsave(&iss_blk_reglock, flags);
	out_be32(&iss_blk_regs->devno, ib->devno);
	out_8(&iss_blk_regs->cmd, ISS_BD_CMD_SYNC);
	spin_unlock_irqrestore(&iss_blk_reglock, flags);

	return 0;
}

static int iss_blk_revalidate(struct gendisk *disk)
{
	struct iss_blk *ib = disk->private_data;
	unsigned long flags;

	pr_debug("issblk%d: revalidate !\n", disk->first_minor);

	if (ib->present && ib->changed) {
		spin_lock_irqsave(&iss_blk_reglock, flags);
		out_be32(&iss_blk_regs->devno, ib->devno);
		out_8(&iss_blk_regs->cmd, ISS_BD_CMD_CLOSE);
		ib->present = ib->changed = 0;
		spin_unlock_irqrestore(&iss_blk_reglock, flags);
	}
	iss_blk_setup(ib);
	return 0;
}

static int iss_blk_media_changed(struct gendisk *disk)
{
	struct iss_blk *ib = disk->private_data;
	u32 stat;

	pr_debug("issblk%d: media_changed !\n", disk->first_minor);

	out_be32(&iss_blk_regs->devno, ib->devno);
	out_8(&iss_blk_regs->cmd, ISS_BD_CMD_STATUS);
	stat = in_be32(&iss_blk_regs->stat);
	if (stat == ISS_BD_FILE_CHANGED)
		ib->changed = 1;

	return ib->changed;
}

static int iss_blk_open(struct block_device *bdev, fmode_t mode)
{
	struct gendisk *disk = bdev->bd_disk;
	struct iss_blk *ib = disk->private_data;

	pr_debug("issblk%d: open !\n", disk->first_minor);

	check_disk_change(bdev);
	if (ib->changed)
		iss_blk_setup(ib);
	if (!ib->present)
		return -ENOMEDIUM;
	return 0;
}

static struct block_device_operations iss_blk_fops = {
	.owner		  = THIS_MODULE,
	.open		  = iss_blk_open,
	.release	  = iss_blk_release,
	.media_changed	  = iss_blk_media_changed,
	.revalidate_disk  = iss_blk_revalidate,
};

static int __init iss_blk_init(void)
{
	struct device_node *np;
	int i;

	pr_debug("iss_regs offsets:\n");
	pr_debug("  cmd    : 0x%x\n", offsetof(struct iss_blk_regs, cmd));
	pr_debug("  stat   : 0x%x\n", offsetof(struct iss_blk_regs, stat));
	pr_debug("  sector : 0x%x\n", offsetof(struct iss_blk_regs, sector));
	pr_debug("  count  : 0x%x\n", offsetof(struct iss_blk_regs, count));
	pr_debug("  devno  : 0x%x\n", offsetof(struct iss_blk_regs, devno));
	pr_debug("  size   : 0x%x\n", offsetof(struct iss_blk_regs, size));
	pr_debug("  data   : 0x%x\n", offsetof(struct iss_blk_regs, data));

	np = of_find_node_by_path("/iss-block");
	if (np == NULL)
		return -ENODEV;
	iss_blk_regs = of_iomap(np, 0);
	if (iss_blk_regs == NULL) {
		pr_err("issblk: Failed to map registers\n");
		return -ENOMEM;
	}

	if (register_blkdev(MAJOR_NR, "iss_blk"))
		return -EIO;

	spin_lock_init(&iss_blk_qlock);
	spin_lock_init(&iss_blk_reglock);

	printk(KERN_INFO "ISS Block driver initializing for %d minors\n",
	       NUM_ISS_BLK_MINOR);

	for (i = 0; i < NUM_ISS_BLK_MINOR; i++) {
		struct gendisk *disk = alloc_disk(1);
		struct request_queue *q;
		struct iss_blk *ib = &iss_blks[i];

		if (!disk) {
			pr_err("issblk%d: Failed to allocate disk\n", i);
			break;
		}

		q = blk_init_queue(iss_blk_do_request, &iss_blk_qlock);
		if (q == NULL) {
			pr_err("issblk%d: Failed to init queue\n", i);
			put_disk(disk);
			break;
		}
		q->queuedata = ib;

		ib->disk = disk;
		ib->devno = i;
		ib->present = 0;
		ib->changed = 0;
		ib->capacity = 0;
		ib->sectsize = 512;

		disk->major = MAJOR_NR;
		disk->first_minor = i;
		disk->fops = &iss_blk_fops;
		disk->private_data = &iss_blks[i];
		disk->flags = GENHD_FL_REMOVABLE;
		disk->queue = q;
		sprintf(disk->disk_name, "issblk%d", i);

		iss_blk_setup(ib);

		add_disk(disk);
	}

	return 0;
}

static void __exit iss_blk_exit(void)
{
	int i;

	unregister_blkdev(MAJOR_NR, "iss_blk");

	for (i = 0; i < NUM_ISS_BLK_MINOR; i++) {
		struct iss_blk *ib = &iss_blks[i];

		if (ib->present) {
			out_be32(&iss_blk_regs->devno, ib->devno);
			out_8(&iss_blk_regs->cmd, ISS_BD_CMD_CLOSE);
		}
	}
}

module_init(iss_blk_init);
module_exit(iss_blk_exit);

MODULE_DESCRIPTION("ISS Simulator Block Device");
MODULE_LICENSE("GPL");
