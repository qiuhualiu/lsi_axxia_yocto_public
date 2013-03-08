/*
 * This file contains a driver for the LSI ACP 3400 DMA engine
 * As of now, not all HW capabilities are supported. Primary
 * intended use is for scooping contigous data blocks between main memory and
 * SRIO/PCIE memory mapped devices. Other applications may work
 * as well but those have not been tested.
 *
 * The driver is based on:
 *
 * lsi-dma.c - Copyright 2011 Mentor Graphics
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/export.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kref.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/delay.h>

#include <asm/page.h>
#include <asm/atomic.h>

#include "acp_gpdma.h"

static DEFINE_SPINLOCK(acp_gpdma_glob_lock);
static struct gpdma_engine *acp_gpdma_engine = NULL;

static void gpdma_dump(struct gpdma_engine *e)
{
	int i;

	dev_dbg(e->dev,
		"DMA Engine register dump (mmapped src = %p)\n",
		e->base);
	for (i = 0; i < 0x40 * MAX_GPDMA_CHANNELS / sizeof(u32); i++) {
		dev_dbg(e->dev,
			"%p == %08X ",
			(void *)(e->base + i * sizeof(u32)),
			__raw_in_le32(e->base + i * sizeof(u32)));
	}
}
static void gpdma_dump_channel(struct gpdma_channel *dmac)
{
	int i;
	void __iomem *base = BASE(dmac);

	dev_dbg(dmac->engine->dev,
		"DMA Engine register dump channel %d\n",
		dmac->channel);
	for (i = 0; i < 0x40/sizeof(u32); i++) {
		dev_dbg(dmac->engine->dev,
			"%p == %08X ",
			(void *)(base + i * sizeof(u32)),
			__raw_in_le32(base + i * sizeof(u32)));
	}
}
static void reset_channel(struct gpdma_engine *gpdma, int ch)
{
	void __iomem *base = gpdma->base + (ch * 0x40);
	int wait = 0;

	dev_dbg(gpdma->dev, "%s\n", __func__);

	__raw_out_le32((base + DMA_CHANNEL_CONFIG), 0);
	while (__raw_in_le32(base + DMA_CHANNEL_CONFIG)) {
		wait++;
		if (wait >= 1000) {
			dev_warn(gpdma->dev,
				 "%s - clear channel %d FIFO when channel not idle\n",
				 __func__, ch);
			break;
		}
	}
	wait = 0;
	__raw_out_le32((base + DMA_CHANNEL_CONFIG), DMA_CONFIG_CLEAR_FIFO);
	while (__raw_in_le32(base + DMA_CHANNEL_CONFIG)) {
		wait++;
		if (wait >= 1000) {
			dev_warn(gpdma->dev,
				 "%s - clear channel %d FIFO fails\n",
				 __func__, ch);
			break;
		}
	}
}
static void soft_reset(struct gpdma_engine *gpdma)
{
	int i;

	/**
	 * Reset GPDMA by writing Magic Number to reset reg
	 */
	__raw_out_le32(GPDMA_SOFT_RESET(gpdma), GPDMA_MAGIC);

	/**
	 *  Set has to be done twice???
	 *  Yep! According to LSI code it has to be done twice,
	 *  have no idea why.
	 */
	__raw_out_le32(GPDMA_GEN_CONFIG(gpdma),
		       (gpdma->desc.phys & 0xfff00000) | GEN_CONFIG_EXT_MEM);
	__raw_out_le32(GPDMA_GEN_CONFIG(gpdma),
		       (gpdma->desc.phys & 0xfff00000) | GEN_CONFIG_EXT_MEM);

	dev_dbg(gpdma->dev,
		"gpdma->desc.phys & 0xfff00000 == %llx\n",
		(gpdma->desc.phys & 0xfff00000));
	/**
	 * Clear all channels
	 */
	for (i = 0; i < MAX_GPDMA_CHANNELS; i++)
		reset_channel(gpdma, i);
}

static inline struct descriptor *get_descriptor(struct gpdma_desc *desc)
{
	struct gpdma_engine *e = desc_to_engine(desc);
	unsigned long flags;
	struct descriptor *d = NULL;

	spin_lock_irqsave(&e->lock, flags);

	if (desc->next_di == GPDMA_MAX_DESCRIPTORS)
		goto done;
	d = desc->stack[desc->next_di];
	desc->next_di += 1;
done:
	spin_unlock_irqrestore(&e->lock, flags);
	return d;
}
static inline void free_descriptor(struct gpdma_desc *desc,
				   struct descriptor *d)
{
	unsigned long flags;
	struct gpdma_engine *e = desc_to_engine(desc);

	spin_lock_irqsave(&e->lock, flags);

	BUG_ON(desc->next_di == 0);
	desc->next_di -= 1;
	desc->stack[desc->next_di] = d;

	spin_unlock_irqrestore(&e->lock, flags);
}
static int alloc_desc_table(struct gpdma_desc *desc)
{
	struct gpdma_engine *e = desc_to_engine(desc);
	u32 count = 0x400;
	u32 order =  __ilog2((ALIGN(count, PAGE_SIZE)) >> PAGE_SHIFT);
	struct descriptor *d;
	int i;

	desc->va = (void *)__get_free_pages(GFP_KERNEL|GFP_DMA, order);
	if (!desc->va)
		return -ENOMEM;
	desc->phys = virt_to_phys(desc->va);
	desc->order = order;
	d = (struct descriptor *)desc->va;

	for (i = 0; i < GPDMA_MAX_DESCRIPTORS; i++) {
		desc->stack[i] = d++;
		dev_dbg(e->dev,
			"desc[%d] == %p phys %lx\n",
			i, (void *)desc->stack[i],
			virt_to_phys(desc->stack[i]));
	}
	desc->next_di = 0;
	return 0;
}
static void free_desc_table(struct gpdma_desc *desc)
{
	if (desc->va)
		free_pages((unsigned long)desc->va, desc->order);
}

static inline void __kick_channel(struct gpdma_engine *engine,
				  struct gpdma_chan_job *job)
{
	struct descriptor *d = job->d;
	struct gpdma_channel *dmac = job_to_gchan(job);
	void __iomem *base = BASE(dmac);
	unsigned long flags;
	u32 reg;

	if (likely(test_and_clear_bit(GPDMA_CH_TX_QUEUED, &dmac->job.state))) {
		spin_lock_irqsave(&dmac->lock, flags);

		__raw_out_le32(base + DMA_STATUS, DMA_STATUS_CLEAR);

		if (dmac->direction == DMA_TO_DEVICE) {
			__raw_out_le32(base + DMA_NXT_DESCR,
				       ((u32)d & 0xfffff));
		} else {
			/**
			 * Oh No! Can't use descriptors when
			 * reading from device?
			 */
			reg = __raw_in_le16(&d->src_x_ctr);
			__raw_out_le32(base + DMA_X_SRC_COUNT, reg);
			reg = __raw_in_le16(&d->src_y_ctr);
			__raw_out_le32(base + DMA_Y_SRC_COUNT, reg);

			*(volatile u32 *)(base + DMA_X_MODIF_SRC) =
				d->src_x_mod;
			*(volatile u32 *)(base + DMA_Y_MODIF_SRC) =
				d->src_y_mod;
			*(volatile u32 *)(base + DMA_SRC_CUR_ADDR) =
				d->src_addr;
			*(volatile u32 *)(base + DMA_SRC_MASK) =
				d->src_data_mask;

			reg = __raw_in_le16(&d->src_access);
			__raw_out_le32(base + DMA_SRC_ACCESS, reg);
			reg = __raw_in_le16(&d->dst_access);
			__raw_out_le32(base + DMA_DST_ACCESS, reg);

			*(volatile u32 *)(base + DMA_NXT_DESCR) = d->next_ptr;

			reg = __raw_in_le16(&d->dst_x_ctr);
			__raw_out_le32(base + DMA_X_DST_COUNT, reg);
			reg = __raw_in_le16(&d->dst_y_ctr);
			__raw_out_le32(base + DMA_Y_DST_COUNT, reg);

			*(volatile u32 *)(base + DMA_X_MODIF_DST) =
				d->dst_x_mod;
			*(volatile u32 *)(base + DMA_Y_MODIF_DST) =
				d->dst_y_mod;
			*(volatile u32 *)(base + DMA_DST_CUR_ADDR) =
				d->dst_addr;
		}
		__asm__ __volatile__(PPC_MBAR);
		set_bit(GPDMA_CH_KICKED, &dmac->job.state);
		gpdma_dump_channel(dmac);

		if (dmac->direction == DMA_TO_DEVICE) {
			__raw_out_le32(base + DMA_CHANNEL_CONFIG,
				       DMA_CONFIG_DSC_LOAD);
		} else
			*(volatile u32 *)(base + DMA_CHANNEL_CONFIG) =
				d->ch_config;

		spin_unlock_irqrestore(&dmac->lock, flags);
	} else
		dev_warn(dmac->engine->dev,
			 "%s when ch not queued\n",
			 __func__);
}

static inline void gpdma_handle(struct gpdma_channel *dmac,
				enum dma_status status, int tmo)
{
	unsigned long flags;

	spin_lock_irqsave(&dmac->lock, flags);
	set_bit(GPDMA_CH_IDLE, &dmac->job.state);
	dmac->last_completed = dmac->desc.cookie;
	dmac->status = status;
	dmac->direction = DMA_BIDIRECTIONAL;
	spin_unlock_irqrestore(&dmac->lock, flags);

	if (dmac->desc.callback && !tmo)
		dmac->desc.callback(dmac->desc.callback_param);

	dev_dbg(dmac->engine->dev,
		"%s done cookie %d status %d\n",
		__func__, dmac->last_completed, status);
}

static void flush_channel_job(struct gpdma_channel *dmac, int tmo)
{
	if (test_and_clear_bit(GPDMA_CH_TX_SUBMIT, &dmac->job.state)) {
		/* not on any list - just nack and drop */
		free_descriptor(&dmac->engine->desc, dmac->job.d);
		dmac->job.d = NULL;
		gpdma_handle(dmac, DMA_ERROR, tmo);
	} else if (test_and_clear_bit(GPDMA_CH_TX_QUEUED, &dmac->job.state) ||
		   test_and_clear_bit(GPDMA_CH_KICKED, &dmac->job.state)) {
		/* On job or pending list, unlink, drop and Nack */
		list_del_init(&dmac->job.node);
		free_descriptor(&dmac->engine->desc, dmac->job.d);
		dmac->job.d = NULL;
		gpdma_handle(dmac, DMA_ERROR, tmo);
	} else if (test_and_clear_bit(GPDMA_CH_DONE, &dmac->job.state)) {
		list_del_init(&dmac->job.node);
		free_descriptor(&dmac->engine->desc, dmac->job.d);
		dmac->job.d = NULL;
		gpdma_handle(dmac, dmac->job.dma_status, tmo);
	}
}

static void __job_tasklet(unsigned long data)
{
	struct gpdma_engine *engine = (struct gpdma_engine *)data;
	struct gpdma_chan_job *job, *cur;
	int i, maxloop = 10, loop = 0;

purge_job_queue:
	for (i = 0; i < MAX_GPDMA_CHANNELS; i++) {
		struct gpdma_channel *dmac = &engine->channel[i];

		if (test_and_clear_bit(GPDMA_CH_TX_SUBMIT, &dmac->job.state)) {
			dev_dbg(engine->dev,
				"%s: Move ch %d job to job queue\n",
				__func__, i);
			list_add_tail(&dmac->job.node, &engine->job);
			set_bit(GPDMA_CH_TX_QUEUED, &dmac->job.state);
		}
		if (test_and_clear_bit(GPDMA_CH_DONE, &dmac->job.state)) {
			dev_dbg(engine->dev,
				"%s: Ack ch %d job status %d and remove from pending queue\n",
				__func__, i, dmac->job.dma_status);
			list_del_init(&dmac->job.node);
			free_descriptor(&dmac->engine->desc, dmac->job.d);
			dmac->job.d = NULL;
			gpdma_handle(dmac, dmac->job.dma_status, 0);
		}
	}
	if (list_empty(&engine->job))
		return;
	/* Give jobs bypassed earlier a break */
	if ((engine->prio > MAX_GPDMA_CHANNELS) &&
		!list_empty(&engine->pending))
		return;
	if (loop > maxloop || !(test_bit(GPDMA_INIT, &engine->state)))
		return;
	list_for_each_entry_safe(job, cur, &engine->job, node) {
		if (list_empty(&engine->pending)) {
			if (engine->prio) {
				/**
				 * prioritize waiters, we have at least one
				 * pick the oldest.
				 */
				if (!job->prio)
					continue;
				engine->prio = 0;
			} /* else, pick job first in line */
		} else if (job->gpreg != __raw_in_le32(engine->gpreg)) {
			/**
			 * Ongoing job(s) - we can accept new jobs
			 * but only if high address bits match
			 * Flag any by-passed job in order not to
			 * starve them completely
			 */
			engine->prio++;
			job->prio++;
			continue;
		} /* else pick job with match addr-hbits */

		/* init gpreg */
		dev_dbg(engine->dev,
			"%s init gpreg old %08x new %08x\n",
			__func__, __raw_in_le32(engine->gpreg), job->gpreg);
		__raw_out_le32(engine->gpreg, job->gpreg);
		list_del_init(&job->node);
		list_add_tail(&job->node, &engine->pending);
		dev_dbg(engine->dev,
			"%s: Move job from job to pending queue\n",
			__func__);
		__kick_channel(engine, job);
		break;
	}
	/* See if we can squeeze some more in there */
	loop++;
	goto purge_job_queue;
}

static inline void gpdma_sched_job_handler(struct gpdma_engine *engine)
{
	if (unlikely(!test_bit(GPDMA_INIT, &engine->state)))
		return;

	if (likely(test_and_set_bit(TASKLET_STATE_SCHED,
				    &engine->job_task.state) == 0)) {
		__tasklet_schedule(&engine->job_task);
	}
}

static irqreturn_t gpdma_isr(int _irq, void *_dmac)
{
	struct gpdma_channel *dmac = _dmac;
	unsigned long flags;
	void __iomem *base = BASE(dmac);
	u32 status, err;
	int i = 0;

	spin_lock_irqsave(&dmac->lock, flags);

	status = __raw_in_le32(base + DMA_STATUS);
	err = status & DMA_STATUS_ERROR;

	while (status & (DMA_STATUS_CH_WAITING | DMA_STATUS_CH_ACTIVE)) {
		status = __raw_in_le32(base + DMA_STATUS);
		i++;
		if (i > 2000)
			break;
	}
	if (i > 2000)
		dev_warn(dmac->engine->dev,
			 "%s: DMA ch state not cleared after %d loops\n",
			 __func__, i);

	__raw_out_le32(base + DMA_CHANNEL_CONFIG, 0);
	__raw_out_le32((base + DMA_CHANNEL_CONFIG), DMA_CONFIG_CLEAR_FIFO);
	spin_unlock_irqrestore(&dmac->lock, flags);

	dev_dbg(dmac->engine->dev,
		"%s: channel status %08x err %08x wait %d\n",
		__func__, status, err, i);

	if (likely(test_and_clear_bit(GPDMA_CH_KICKED, &dmac->job.state))) {
		dmac->job.dma_status = (err ? DMA_ERROR : DMA_SUCCESS);
		set_bit(GPDMA_CH_DONE, &dmac->job.state);
		gpdma_sched_job_handler(dmac->engine);
		return IRQ_HANDLED;
	}
	dev_warn(dmac->engine->dev, "%s IRQ when ch not kicked\n", __func__);
	return IRQ_NONE;
}

static dma_cookie_t gpdma_assign_cookie(struct gpdma_channel *dmac)
{
	dma_cookie_t cookie = dmac->chan.cookie;

	if (++cookie < 0)
		cookie = 1;

	dmac->chan.cookie = dmac->desc.cookie = cookie;
	return cookie;
}
static dma_cookie_t gpdma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct gpdma_channel *dmac = dchan_to_gchan(tx->chan);
	unsigned long flags;
	dma_cookie_t cookie;

	spin_lock_irqsave(&dmac->lock, flags);
	cookie = gpdma_assign_cookie(dmac);
	spin_unlock_irqrestore(&dmac->lock, flags);

	if (likely(test_and_clear_bit(GPDMA_CH_JOB_INIT,
				      &dmac->job.state))) {
		set_bit(GPDMA_CH_TX_SUBMIT, &dmac->job.state);
		gpdma_sched_job_handler(dmac->engine);
	} else
		dev_warn(dmac->engine->dev,
			 "%s when state != job init\n",
			 __func__);

	dev_dbg(dmac->engine->dev,
		"%s assigned cookie %d\n",
		__func__, cookie);

	return cookie;
}


static int gpdma_alloc_chan_resources(struct dma_chan *chan)
{
	struct gpdma_channel *dmac = dchan_to_gchan(chan);

	dma_async_tx_descriptor_init(&dmac->desc, chan);
	dmac->desc.tx_submit = gpdma_tx_submit;
	dmac->desc.flags = DMA_CTRL_ACK;
	dmac->status = DMA_SUCCESS;
	set_bit(GPDMA_CH_IDLE, &dmac->job.state);
	return 0;
}

static void gpdma_free_chan_resources(struct dma_chan *chan)
{
	struct gpdma_channel *dmac = dchan_to_gchan(chan);

	dev_dbg(dmac->engine->dev, "%s\n", __func__);
	gpdma_dump_channel(dmac);
}

static void gpdma_issue_pending(struct dma_chan *chan)
{
	struct gpdma_channel *dmac = dchan_to_gchan(chan);

	gpdma_sched_job_handler(dmac->engine);
}

static enum dma_status gpdma_tx_status(struct dma_chan *chan,
					dma_cookie_t cookie,
					struct dma_tx_state *txstate)
{
	dma_cookie_t last_used = cookie;
	struct gpdma_channel *dmac = dchan_to_gchan(chan);
	enum dma_status ret = 0;

	last_used = chan->cookie;
	ret = dma_async_is_complete(cookie, dmac->last_completed, last_used);
	dma_set_tx_state(txstate, dmac->last_completed, last_used, 0);
	if (ret)
		dev_dbg(dmac->engine->dev,
			"%s ret %d cookie %d last used %d last_completed %d\n",
			__func__, ret, cookie, chan->cookie,
			dmac->last_completed);
	return ret;
}

static struct dma_async_tx_descriptor *gpdma_prep_memcpy(struct dma_chan *chan,
						       dma_addr_t dst,
						       dma_addr_t src,
						       size_t size,
						       unsigned long dma_flags)
{
	struct gpdma_channel *dmac = dchan_to_gchan(chan);
	struct descriptor *d = get_descriptor(&dmac->engine->desc);
	u16 rot_len, x_count, src_size, src_access = 5, dst_access = 5;

	if (!d) {
		dev_dbg(dmac->engine->dev,
			"--- %s --- : No descriptor\n",
			__func__);
		return NULL;
	}
	if (likely(test_and_clear_bit(GPDMA_CH_IDLE, &dmac->job.state))) {

		if (!(size % 16)) {
			src_size = 16;
			src_access |= (4 << 3);
			dst_access |= (4 << 3);
		} else if (!(size % 8)) {
			src_size = 8;
			src_access |= (3 << 3);
			dst_access |= (3 << 3);
		} else if (!(size % 4)) {
			src_size = 4;
			src_access |= (2 << 3);
			dst_access |= (2 << 3);
		} else if (!(size % 2)) {
			src_size = 2;
			src_access |= (1 << 3);
			dst_access |= (1 << 3);
		} else
			src_size = 1;

		x_count = (size/src_size) - 1;
		rot_len = (2 * src_size) - 1;
		src_access |= (rot_len << 6);

		__raw_out_le16(&d->src_x_ctr, x_count);
		__raw_out_le16(&d->src_y_ctr, 0);
		__raw_out_le32(&d->src_x_mod, src_size);
		__raw_out_le32(&d->src_y_mod, 0);
		__raw_out_le32(&d->src_addr, (src & 0xffffffff));
		__raw_out_le32(&d->src_data_mask, 0xffffffff);
		__raw_out_le16(&d->src_access, src_access);
		__raw_out_le16(&d->dst_access, dst_access);
		__raw_out_le32(&d->ch_config, DMA_CONFIG_ONE_SHOT(1));
		__raw_out_le32(&d->next_ptr, 0);
		__raw_out_le16(&d->dst_x_ctr, x_count);
		__raw_out_le16(&d->dst_y_ctr, 0);
		__raw_out_le32(&d->dst_x_mod, src_size);
		__raw_out_le32(&d->dst_y_mod, 0);
		__raw_out_le32(&d->dst_addr, (dst & 0xffffffff));

		dmac->job.d = d;
		dmac->job.src = src;
		dmac->job.dst = dst;
		dmac->job.prio = 0;
		dmac->job.gpreg = (u64)(((src >> 32) & 0x3F) |
				  (((dst >> 32) & 0x3F) << 8));
		dmac->job.size = size;
		set_bit(GPDMA_CH_JOB_INIT, &dmac->job.state);

		return &dmac->desc;
	} else
		dev_warn(dmac->engine->dev,
			 "%s when Chan not idle\n",
			 __func__);

	free_descriptor(&dmac->engine->desc, d);
	return NULL;
}
static void reset_engine(struct gpdma_engine *engine)
{
	int i;

	clear_bit(GPDMA_INIT, &engine->state);
	__asm__ __volatile__(PPC_MBAR);

	/**
	 * Wait until all channels are out of critical regions
	 */
	for (i = 0; i < MAX_GPDMA_CHANNELS; i++)
		free_irq(engine->channel[i].irq, &engine->channel[i]);

	tasklet_disable(&engine->job_task);
	tasklet_kill(&engine->job_task);

	/* Reset HW */
	soft_reset(engine);

	/**
	 * Flush queues before accepting new jobs
	 */
	for (i = 0; i < MAX_GPDMA_CHANNELS; i++) {
		if (request_irq(engine->channel[i].irq, gpdma_isr,
				IRQF_SHARED, "lsi-dma", &engine->channel[i]))
			dev_err(engine->dev, "failed to request_irq\n");
	}
	tasklet_init(&engine->job_task, __job_tasklet, (unsigned long)engine);
	set_bit(GPDMA_INIT, &engine->state);
	__asm__ __volatile__(PPC_MBAR);

	gpdma_sched_job_handler(engine);
}
static void terminate_channel_job(struct gpdma_channel *dmac, int tmo)
{
	struct gpdma_engine *engine = dmac->engine;

	tasklet_disable(&engine->job_task);

	flush_channel_job(dmac, tmo);

	tasklet_enable(&engine->job_task);
}

static int gpdma_device_control(struct dma_chan *dchan,
				enum dma_ctrl_cmd cmd, unsigned long arg)
{
	struct dma_slave_config *config;
	struct gpdma_channel *dmac = dchan_to_gchan(dchan);

	if (!dmac)
		return -EINVAL;

	switch (cmd) {
	case DMA_TERMINATE_ALL:
		terminate_channel_job(dmac, 1);
		break;

	case DMA_SLAVE_CONFIG:
		config = (struct dma_slave_config *)arg;

		dmac->direction = config->direction;
		break;

	default:
		return -ENXIO;
	}

	return 0;
}

static void gpdma_engine_release(struct kref *kref)
{
	struct gpdma_engine *engine = container_of(kref,
						   struct gpdma_engine,
						   kref);

	kfree(engine);
}

void gpdma_engine_put(struct gpdma_engine *engine)
{
	if (engine)
		kref_put(&engine->kref, gpdma_engine_release);
}

struct gpdma_engine *gpdma_engine_get(struct gpdma_engine *engine)
{
	if (engine)
		kref_get(&engine->kref);

	return engine;
}
static int gpdma_of_remove(struct platform_device *op)
{
	struct gpdma_engine *engine = dev_get_drvdata(&op->dev);
	int i;

	dev_dbg(&op->dev, "%s\n", __func__);

	if (!engine)
		return 0;

	spin_lock(&acp_gpdma_glob_lock);
	acp_gpdma_engine = NULL;
	spin_unlock(&acp_gpdma_glob_lock);

	if (test_bit(GPDMA_INIT, &engine->state))
		dma_async_device_unregister(&engine->dma_device);

	tasklet_disable(&engine->job_task);
	tasklet_kill(&engine->job_task);

	for (i = 0; i < MAX_GPDMA_CHANNELS; i++) {
		struct gpdma_channel *dmac = &engine->channel[i];

		if (dmac->channel == i) /* Initialized ok */
			free_irq(dmac->irq, dmac);
	}
	free_desc_table(&engine->desc);

	iounmap(engine->base);
	iounmap(engine->gbase);
	iounmap(engine->gpreg);
	gpdma_engine_put(engine);
	dev_set_drvdata(&op->dev, NULL);

	return 0;
}

static int __devinit gpdma_of_probe(struct platform_device *op)
{
	struct gpdma_engine *engine;
	struct device_node *child;
	int rc = -ENOMEM;

	dev_dbg(&op->dev, "%s\n", __func__);

	engine = kzalloc(sizeof *engine, GFP_KERNEL);
	if (!engine)
		return -ENOMEM;

	kref_init(&engine->kref);
	engine->dev = engine->dma_device.dev = &op->dev;

	dev_set_drvdata(&op->dev, engine);

	dma_cap_zero(engine->dma_device.cap_mask);
	dma_cap_set(DMA_MEMCPY, engine->dma_device.cap_mask);

	engine->dma_device.device_alloc_chan_resources =
		gpdma_alloc_chan_resources;
	engine->dma_device.device_free_chan_resources =
		gpdma_free_chan_resources;
	engine->dma_device.device_tx_status = gpdma_tx_status;
	engine->dma_device.device_issue_pending = gpdma_issue_pending;
	engine->dma_device.device_prep_dma_memcpy = gpdma_prep_memcpy;
	engine->dma_device.device_control = gpdma_device_control;

	INIT_LIST_HEAD(&engine->dma_device.channels);
	INIT_LIST_HEAD(&engine->job);
	INIT_LIST_HEAD(&engine->pending);
	spin_lock_init(&engine->lock);

	/**
	 * FIXME! Should get these from dtb! (update from LSI coming soon)
	 */
	engine->base = ioremap(0x20004e0000ull, 0x40 * MAX_GPDMA_CHANNELS);
	engine->gbase = ioremap(0x20004e0400ull, 0xc);
	engine->gpreg = ioremap(0x200040c000ull, 0x4);

	dev_dbg(&op->dev,
		"mapped base\t%llx at %p\n",
		0x20004e0000ull,
		engine->base);
	dev_dbg(&op->dev,
		"mapped gbase\t%llx at %p\n",
		0x20004e0400ull,
		engine->gbase);
	dev_dbg(&op->dev,
		"mapped gpreg\t%llx at %p\n",
		0x200040c000ull,
		engine->gpreg);

	if (!engine->base || !engine->gbase || !engine->gpreg)
		goto err_init;

	if (alloc_desc_table(&engine->desc))
		goto err_init;

	soft_reset(engine);

	/* Setup channels */
	for_each_child_of_node(op->dev.of_node, child) {
		if (of_device_is_compatible(child, "gp-dma,acp-dma")) {
			struct resource chan_res;
			struct gpdma_channel *dmac;
			int id;

			rc = of_address_to_resource(child, 0, &chan_res);
			if (rc)
				goto err_init;
			id = ((chan_res.start - 0x4e0000) & 0xfff) >> 6;
			if (id > MAX_GPDMA_CHANNELS)
				goto err_init;
			dmac = &engine->channel[id];
			dmac->channel = id;
			dmac->engine = engine;
			dmac->direction = DMA_BIDIRECTIONAL;
			spin_lock_init(&dmac->lock);
			INIT_LIST_HEAD(&dmac->job.node);
			set_bit(GPDMA_CH_IDLE, &dmac->job.state);
			dmac->chan.device = &engine->dma_device;

			/* Find the IRQ line, if it exists in the dev tree */
			dmac->irq = irq_of_parse_and_map(child, 0);
			dev_dbg(engine->dev,
				"channel %d, irq %d\n",
				id, dmac->irq);
			rc = request_irq(dmac->irq, gpdma_isr,
					 IRQF_SHARED, "lsi-dma", dmac);
			if (rc) {
				dev_err(engine->dev,
					"failed to request_irq, error = %d\n",
					rc);
				goto err_init;
			}

			/* Add the channel to the DMAC list */
			list_add_tail(&dmac->chan.device_node,
				      &engine->dma_device.channels);
		}
	}
	gpdma_dump(engine);

	rc = dma_async_device_register(&engine->dma_device);
	if (rc) {
		dev_err(engine->dev, "unable to register\n");
		goto err_init;
	}
	tasklet_init(&engine->job_task, __job_tasklet, (unsigned long)engine);
	set_bit(GPDMA_INIT, &engine->state);
	spin_lock(&acp_gpdma_glob_lock);
	acp_gpdma_engine = engine;
	spin_unlock(&acp_gpdma_glob_lock);

	return 0;

err_init:
	gpdma_of_remove(op);
	dev_set_drvdata(&op->dev, NULL);
	return rc;
}
static const struct of_device_id gpdma_of_ids[] = {
	{ .compatible = "gp-dma,acp-dma", },
	{ .compatible = "gp-dma,acp-gpdma", },
	{}
};

static struct platform_driver gpdma_of_driver = {
	.driver = {
		.name = "acp-gpdma",
		.owner = THIS_MODULE,
		.of_match_table = gpdma_of_ids,
	},
	.probe = gpdma_of_probe,
	.remove = gpdma_of_remove,
};

/**
 * acp_gpdma_reset_engine
 *
 * Perform soft reset procedure on DMA Engine.
 *
 * Needed occasionally to work around nasty
 * bug ACP3400 sRIO HW.
 */
void acp_gpdma_reset_engine(void)
{
	struct gpdma_engine *engine;
	unsigned long flags;

	if (!try_module_get(THIS_MODULE))
		return;
	spin_lock(&acp_gpdma_glob_lock);
	engine = gpdma_engine_get(acp_gpdma_engine);
	spin_unlock(&acp_gpdma_glob_lock);

	if (!engine) {
		module_put(THIS_MODULE);
		return;
	}

	/* Reset all */
	spin_lock_irqsave(&engine->lock, flags);
	reset_engine(engine);
	spin_unlock_irqrestore(&engine->lock, flags);

	gpdma_engine_put(acp_gpdma_engine);
	module_put(THIS_MODULE);
}
EXPORT_SYMBOL_GPL(acp_gpdma_reset_engine);

static __init int gpdma_init(void)
{
	pr_info("ACP GP-DMA driver\n");
	return platform_driver_register(&gpdma_of_driver);
}

static void __exit gpdma_exit(void)
{
	platform_driver_unregister(&gpdma_of_driver);
}

subsys_initcall(gpdma_init);
module_exit(gpdma_exit);

MODULE_DESCRIPTION("ACP GP-DMA driver");
MODULE_LICENSE("GPL");
