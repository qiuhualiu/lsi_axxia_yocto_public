/*
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#ifndef __DMA_ACP_GPDMA_H
#define __DMA_ACP_GPDMA_H

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#define MAX_GPDMA_CHANNELS	4
#define GPDMA_MAX_DESCRIPTORS	20
#define GPDMA_MAGIC		(0xabcd1234UL)

#define DMA_X_SRC_COUNT			0x00
#define DMA_Y_SRC_COUNT			0x04
#define DMA_X_MODIF_SRC			0x08
#define DMA_Y_MODIF_SRC			0x0c
#define DMA_SRC_CUR_ADDR		0x10
#define DMA_SRC_ACCESS			0x14
#define   DMA_SRC_ACCESS_BURST_TYPE		(1<<15)
#define   DMA_SRC_ACCESS_TAIL_LENGTH(x)		(((x) & 0xF) << 11)
#define   DMA_SRC_ACCESS_ROTATOR_LENGTH(x)	(((x) & 1F) << 6)
#define   DMA_SRC_ACCESS_SRC_SIZE(x)		(((x) & 7) << 3)
#define   DMA_SRC_ACCESS_SRC_BURST(x)		(((x) & 7) << 0)
#define DMA_SRC_MASK			0x18
#define DMA_X_DST_COUNT			0x1c
#define DMA_Y_DST_COUNT			0x20
#define DMA_X_MODIF_DST			0x24
#define DMA_Y_MODIF_DST			0x28
#define DMA_DST_CUR_ADDR		0x2C
#define DMA_DST_ACCESS			0x30
#define   DMA_DST_ACCESS_DST_SIZE(x)		(((x) & 7) << 3)
#define   DMA_DST_ACCESS_DST_BURST(x)		(((x) & 7) << 0)
#define DMA_NXT_DESCR			0x34
#define DMA_CHANNEL_CONFIG		0x38
#define   DMA_CONFIG_DST_SPACE(x)		(((x) & 7) << 26)
#define   DMA_CONFIG_SRC_SPACE(x)		(((x) & 7) << 23)
#define   DMA_CONFIG_EXT_PRIORITY		(1<<22)
#define   DMA_CONFIG_PRIORITY_ROW		(1<<21)
#define   DMA_CONFIG_PRIORITY			(1<<20)
#define   DMA_CONFIG_CHANNEL_PRIORITY(x)	(((x) & 3) << 16)
#define   DMA_CONFIG_LAST_BLOCK			(1<<15)
#define   DMA_CONFIG_CLEAR_FIFO			(1<<14)
#define   DMA_CONFIG_START_MEM_LOAD		(1<<13)
#define   DMA_CONFIG_STOP_DST_EOB		(1<<11)
#define   DMA_CONFIG_INT_DST_EOT		(1<<7)
#define   DMA_CONFIG_INT_DST_EOB		(1<<6)
#define   DMA_CONFIG_TX_EN			(1<<1)
#define   DMA_CONFIG_CHAN_EN			(1<<0)
#define DMA_STATUS			0x3C
#define   DMA_STATUS_CH_PAUS_WR_EN		(1<<16)
#define   DMA_STATUS_ERR_ACC_DST_DL		(1<<15)
#define   DMA_STATUS_ERR_ACC_SRC_DL		(1<<14)
#define   DMA_STATUS_ERR_ACC_DST		(1<<13)
#define   DMA_STATUS_ERR_ACC_SRC		(1<<12)
#define   DMA_STATUS_ERR_FLOW			(1<<8)
#define   DMA_STATUS_CH_PAUSE			(1<<7)
#define   DMA_STATUS_CH_WAITING			(1<<5)
#define   DMA_STATUS_CH_ACTIVE			(1<<4)
#define   DMA_STATUS_TR_COMPLETE		(1<<3)
#define   DMA_STATUS_BLK_COMPLETE		(1<<2)

#define DMA_STATUS_ERROR (			\
		DMA_STATUS_ERR_ACC_DST_DL |	\
		DMA_STATUS_ERR_ACC_SRC_DL |	\
		DMA_STATUS_ERR_ACC_DST |	\
		DMA_STATUS_ERR_ACC_SRC |	\
		DMA_STATUS_ERR_FLOW)

#define DMA_CONFIG_ONE_SHOT(__ext) (					\
		DMA_CONFIG_DST_SPACE((__ext)) |				\
		DMA_CONFIG_SRC_SPACE((__ext)) |				\
		DMA_CONFIG_LAST_BLOCK |					\
		DMA_CONFIG_INT_DST_EOT |				\
		DMA_CONFIG_TX_EN |					\
		DMA_CONFIG_CHAN_EN)

#define DMA_CONFIG_DSC_LOAD (DMA_CONFIG_START_MEM_LOAD | DMA_CONFIG_CHAN_EN)

#define DMA_STATUS_CLEAR (			\
		DMA_STATUS_CH_PAUS_WR_EN |	\
		DMA_STATUS_TR_COMPLETE |	\
		DMA_STATUS_BLK_COMPLETE)

#define GEN_STAT		0x0
#define GEN_CONFIG		0x4
#define   GEN_CONFIG_EXT_MEM		(1<<19)
#define SOFT_RESET		0x8

#define GPDMA_GEN_STAT(__p) ((__p)->gbase + GEN_STAT)
#define GPDMA_GEN_CONFIG(__p) ((__p)->gbase + GEN_CONFIG)
#define GPDMA_SOFT_RESET(__p) ((__p)->gbase + SOFT_RESET)

#define GPDMA_CH_IDLE		0U
#define GPDMA_CH_JOB_INIT	1U
#define GPDMA_CH_TX_SUBMIT	2U
#define GPDMA_CH_TX_QUEUED	3U
#define GPDMA_CH_KICKED		4U
#define GPDMA_CH_DONE		5U

struct descriptor {
	u16 src_x_ctr;
	u16 src_y_ctr;
	u32 src_x_mod;
	u32 src_y_mod;
	u32 src_addr;
	u32 src_data_mask;
	u16 src_access;
	u16 dst_access;
	u32 ch_config;
	u32 next_ptr;
	u16 dst_x_ctr;
	u16 dst_y_ctr;
	u32 dst_x_mod;
	u32 dst_y_mod;
	u32 dst_addr;
};

struct gpdma_engine;

struct gpdma_chan_job {
	volatile unsigned long          state;
	struct list_head                node;
	struct descriptor               *d; /* may be a list someday... */
	dma_addr_t                      src;
	dma_addr_t                      dst;
	u32                             gpreg;
	int                             prio;
	int size;
	enum dma_status                 dma_status;
};

struct gpdma_channel {
	struct gpdma_engine		*engine;
	unsigned int			channel;
	int				irq;

	struct dma_chan			chan;

	spinlock_t			lock;
	struct gpdma_chan_job           job;
	struct dma_async_tx_descriptor	desc;
	dma_cookie_t			last_completed;
	enum dma_status			status;
	enum dma_data_direction         direction;
};

struct gpdma_desc {
	void                            *va;
	u32                             order;
	dma_addr_t                      phys;
	int                             next_di;
	struct descriptor               *stack[GPDMA_MAX_DESCRIPTORS];
};
#define GPDMA_INIT                      0U
#define GPDMA_TERMINATE                 1U
struct gpdma_engine {
	volatile unsigned long          state;
	struct kref                     kref;
	struct device			*dev;
	struct device_dma_parameters	dma_parms;
	struct dma_device		dma_device;
	struct gpdma_channel		channel[MAX_GPDMA_CHANNELS];
	struct gpdma_desc               desc;
	struct list_head                job;
	struct list_head                pending;
	struct tasklet_struct           job_task;
	int                             prio;
	spinlock_t			lock;
	void __iomem			*base;
	void __iomem			*gbase;
	void __iomem			*gpreg;
};


#if __GNUC__ < 4 || (__GNUC__ == 4 && __GNUC_MINOR__ == 0)

static inline void __raw_out_le32(volatile u32 *addr, const u32 val)
{
	__asm__ __volatile__ ("stwbrx %1,0,%2" \
			      : "=m" (*addr) : "r" (val), \
			      "r" (addr) : "memory");
}
static inline u32 __raw_in_le32(const volatile u32 *addr)
{
	u32 val;

	__asm__ __volatile__ ("lwbrx %0,0,%1" \
			      : "=r" (val) : "r" (addr), \
			      "m" (*addr) : "memory");
	return val;
}
static inline void __raw_out_le16(volatile u16 *addr, const u32 val)
{
	__asm__ __volatile__ ("sthbrx %1,0,%2" \
			      : "=m" (*addr) : "r" (val), \
			      "r" (addr) : "memory");
}
static inline u32 __raw_in_le16(const volatile u16 *addr)
{
	u32 val;

	__asm__ __volatile__ ("lhbrx %0,0,%1" \
			      : "=r" (val) : "r" (addr), \
			      "m" (*addr) : "memory");
	return val;
}

#else
static inline void __raw_out_le32(volatile u32 *addr, const u32 val)
{
	__asm__ __volatile__ ("stwbrx %1,%y0" \
			      : "=Z" (*addr) : "r" (val) : "memory");
}
static inline u32 __raw_in_le32(const volatile __u32 *addr)
{
	u32 val;

	__asm__ __volatile__ ("lwbrx %0,%y1" \
			      : "=r" (val) : "Z" (*addr) : "memory");
	return val;
}
static inline void __raw_out_le16(volatile u16 *addr, const u32 val)
{
	__asm__ __volatile__ ("sthbrx %1,%y0" \
			      : "=Z" (*addr) : "r" (val) : "memory");
}
static inline u32 __raw_in_le16(const volatile __u16 *addr)
{
	u32 val;

	__asm__ __volatile__ ("lhbrx %0,%y1" \
			      : "=r" (val) : "Z" (*addr) : "memory");
	return val;
}

#endif

static inline void __iomem *BASE(struct gpdma_channel *dmac)
{
	return dmac->engine->base + dmac->channel * 0x40;
}

#define dchan_to_gchan(n) container_of(n, struct gpdma_channel, chan)
#define job_to_gchan(n) container_of(n, struct gpdma_channel, job)
#define desc_to_engine(n) container_of(n, struct gpdma_engine, desc)

#endif
