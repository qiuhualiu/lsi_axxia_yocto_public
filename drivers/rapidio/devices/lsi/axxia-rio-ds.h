/*
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;  if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#ifndef _AXXIA_RIO_DS_H_
#define _AXXIA_RIO_DS_H_

#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/rio_dio.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>

#define DS_DEBUG 1
#define USE_IOCTRL 1

/******************************************************************************
    #defines
******************************************************************************/
#define RIO_DS_DATA_BUF_1K                  (1<<10)
#define RIO_DS_DATA_BUF_2K                  (1<<11)
#define RIO_DS_DATA_BUF_4K                  (1<<12)
#define RIO_DS_DATA_BUF_8K                  (1<<13)
#define RIO_DS_DATA_BUF_16K                 (1<<14)
#define RIO_DS_DATA_BUF_32K                 (1<<15)
#define RIO_DS_DATA_BUF_64K                 (1<<16)

#define RIO_MAX_NUM_OBDS_DSE                16
#define RIO_MAX_NUM_IBDS_DSE				16
#define RIO_MAX_NUM_IBDS_VSID_M             32

#define RC_TBD                              (-1)

#define RIO_DS_TRUE                         1
#define RIO_DS_FALSE                        0

#define IOCTL_BUF_SIZE                      4096

/*
** Data Streaming registers
*/
#define DSE_WAKEUP                   (2)
#define DSE_ENABLE                   (1)
#define RAB_OBDSE_CTRL(n)            (RAB_REG_BASE + (0x2d28 + (0x10*(n))))
#define RAB_OBDSE_STAT(n)            (RAB_REG_BASE + (0x2d28 + (0x10*(n)))+0x4)
#define RAB_OBDSE_DESC_ADDR(n)       (RAB_REG_BASE + (0x2d28 + (0x10*(n)))+0x8)

#define RAB_IBVIRT_M_STAT(n)         (RAB_REG_BASE + (0x2ef0 + (0x4*(n))))

#define RAB_IBDSE_CTRL(n) \
	(RAB_REG_BASE + (0x2a20 + (0x8 * (n))))
#define RAB_IBDSE_STAT(n) \
	(RAB_REG_BASE + (0x2a20 + (0x8 * (n))) + 0x4)

#define RAB_IBDS_VSID_ADDR_LOW(n)    (RAB_REG_BASE + (0x2b28 + (0x8*(n))))
#define RAB_IBDS_VSID_ADDR_HI(n)     (RAB_REG_BASE + (0x2b28 + (0x8*(n)))+0x4)

#define RAB_IBDS_VSID_ALIAS          (RAB_REG_BASE + 0x2a1c)

#define RIO_DS_DESC_ALIGNMENT        (1 << 5)

/* stats */
#define IB_VIRT_M_STAT_ERROR_MASK           0x3FC
#define IB_VIRT_M_STAT_PDU_DROPPED	    (1 << 9)
#define IB_VIRT_M_STAT_SEG_LOSS		    (1 << 8)
#define IB_VIRT_M_STAT_MTU_LEN_MIS_ERR	    (1 << 7)
#define IB_VIRT_M_STAT_PDU_LEN_MIS_ERR      (1 << 6)
#define IB_VIRT_M_STAT_TRANS_ERR	    (1 << 5)
#define IB_VIRT_M_STAT_UPDATE_ERR	    (1 << 4)
#define IB_VIRT_M_STAT_TIMEOUT_ERR	    (1 << 3)
#define IB_VIRT_M_STAT_FETCH_ERR            (1 << 2)

#define IB_DSE_DESC_ERROR_MASK          0xC00
#define IB_DSE_DESC_AXI_ERR             (1 << 11)
#define IB_DSE_DESC_DS_ERR              (1 << 10)
#define IB_DSE_DESC_DONE                (1 << 9)

#define IB_DSE_VSID_IN_USED             0x3F
#define IB_DSE_STAT_TRANS_PENDING       (1 << 6)
#define IB_DSE_STAT_TIMEOUT             (1 << 7)

#define OB_DSE_STAT_ERROR_MASK		    0x3C
#define OB_DSE_STAT_TRANS_ERR	  	    (1 << 5)
#define OB_DSE_STAT_UPDATE_ERR		    (1 << 4)
#define OB_DSE_STAT_DESC_ERR                (1 << 3)
#define OB_DSE_STAT_FETCH_ERR               (1 << 2)

#define OB_DSE_DESC_ERROR_MASK          0x400
#define OB_HDR_DESC_AXI_ERR             (1 << 10)
#define OB_HDR_DESC_DONE                (1 << 8)


/* data streaming dtb related information */
struct rio_ds_dtb_info{
    int ds_enabled;
    int num_inb_virtaul_m;      /* number of inbound virtual M */
    int num_outb_dses;          /* number of outbound DSEs */
    int inb_num_data_descs;     /* number of inbound data descriptors */
    int outb_num_hdr_descs;     /* number of outbound header descriptors */
    int outb_num_data_descs;    /* number of outbound data descriptors */
};

/*
** The following data structure defines data streaming descriptors
**
**  The HW requires that the desc_addr has to be 8 words alignment, thus
**      additional words are added for SW usage
*/
/* outbound data descriptor */
struct rio_ods_data_desc{
    u32 dw0;
    u32 dw1;
    u32 dw2;
    u32 dw3;

    /* SW usage */
    u32 virt_data_buf;
    u32 sw1;
    u32 sw2;
    u32 sw3;
};

/* inbound data descriptor */
struct rio_ids_data_desc{
    u32 dw0;
    u32 dw1;
    u32 dw2;
    u32 dw3;
    u32 dw4;

    /* SW usage */
    u32 virt_data_buf;
    u32 usr_virt_data_buf;
    u32 sw2;
};

/*
** The following data structure defines data streaming header descriptors
**  only used in outbound
*/
struct rio_ds_hdr_desc{
    u32 dw0;
    u32 dw1;
    u32 dw2;
    u32 dw3;
    u32 dw4;

    /* SW usage */
    u32 virt_data_buf;

#ifdef USE_IOCTRL
    u32 remaining_data_len;  /* TO_BE_REMOVED */
    u32 offset;
#else
    u32 sw1;
    u32 sw2;
#endif
};

/*
** OBDS DSE configuration
*/
struct rio_obds_dse_cfg {
    u8         in_use;     /* if the DSE is in_use */
    u8         cos;
    u16        dest_id;
    u16        stream_id;
    spinlock_t lock;
    char       name[16];

    /* header descriptor */
    u16        num_hdr_desc_free;
    u16        max_num_hdr_desc;
    u16        hdr_desc_offset;
    u16        hdr_head;
    u16        hdr_head_reserved;
    u16        hdr_tail;
    u16        num_hdr_desc_reserved;
    u16        num_data_desc_per_hdr_desc;
    u16        numDataBufsCopied;

    /* data descriptor */
    u16        num_data_desc_free;
    u16        max_num_data_desc;
    u16        data_desc_offset;
    u16        data_head;
    u16        data_head_reserved;
    u16        data_tail;
    u16        num_data_desc_reserved;

};

/*
** IBDS configuration
*/
struct ibds_virt_m_cfg {
    spinlock_t lock;
    u32        in_use;     /* if the DSE is in_use */
    u8         cos;
    u16        dest_id;
    u16        stream_id;
    u16        source_id;
    char       name[16];

    u16        num_desc_free;
    u16        max_num_data_desc;
    u16        data_desc_offset;
    u16        data_head;
    u16        data_tail;
    u16        user_buf_index_head;
    u16        user_buf_index_tai;
    u16        is_desc_chain_tran_completed;
    u16        remaining_data_len;
    u16        pdu_len;
    u16        data_head_reserved;
    void       *ptr_data_buf_start_addr;

};

/* Outbound data stream stats */
struct rio_ds_obds_dse_stats {
    u32	       num_desc_chain_transferred;
    u32	       num_desc_transferred;
    u32        num_desc_err;
    u32        num_desc_fetch_err;
    u32        num_desc_data_transaction_err;
    u32        num_desc_update_err;
    u32        num_desc_axi_err;
};

/* Inbound data stream stats */
struct rio_ds_ibds_vsid_m_stats {
    u32	       num_desc_chain_transferred;
    u32        num_desc_transferred;
    u32        num_desc_fetch_err;
    u32        num_timeout_err;
    u32        num_desc_update_err;
    u32	       num_data_transaction_err;
    u32        num_mtu_len_mismatch_err;
    u32        num_pdu_dropped;
    u32        num_pdu_len_mismatch_err;
    u32        num_dropped_pdu;
    u32        num_segment_loss;
    u32        num_desc_axi_err;
    u32        num_desc_ds_err;
};

/*
** The following data structure defines private data used by data streaming
** feature
*/
struct rio_ds_priv {
    /* IBDS */
    u32				max_pdu_len;
    u16                         num_ibds_dses;
    u16                         num_ibds_virtual_m;
    u16                         num_ibds_data_desc;
    struct rio_ids_data_desc    *ptr_ibds_data_desc;
    struct ibds_virt_m_cfg	ibds_vsid_m_cfg[RIO_MAX_NUM_IBDS_VSID_M];

    /* OBDS */
    u16                         num_obds_dses;
    u16                         num_obds_hdr_desc;
    u16                         num_obds_data_desc;
    struct rio_obds_dse_cfg     obds_dse_cfg[RIO_MAX_NUM_OBDS_DSE];

    struct rio_ds_hdr_desc      *ptr_obds_hdr_desc;
    struct rio_ods_data_desc    *ptr_obds_data_desc;

    struct rio_irq_handler      ob_dse_irq[RIO_MAX_NUM_OBDS_DSE];
    struct rio_irq_handler      ib_dse_vsid_irq[RIO_MAX_NUM_IBDS_VSID_M];

    struct rio_ds_ibds_vsid_m_stats ib_vsid_m_stats[RIO_MAX_NUM_IBDS_VSID_M];
    struct rio_ds_obds_dse_stats        ob_dse_stats[RIO_MAX_NUM_OBDS_DSE];

};

/* open an OBDS data stream */
extern int axxia_open_ob_data_stream(
    struct rio_mport    *mport,
    void                *dev_id,
    int                 dest_id,
    int                 stream_id,
    int                 cos,
    int                 num_pdus,
    int                 pdu_length);

extern int open_ob_data_stream(
    struct rio_mport    *mport,
    void                *dev_id,
    int                 dest_id,
    int                 stream_id,
    int                 cos,
    int                 num_pdus,
    int                 pdu_length);

/* add user's data */
extern int axxia_add_ob_data_stream(
    struct rio_mport        *mport,
    int                     dest_id,
    int                     stream_id,
    int                     cos,
    void                    *buffer,
    int                     data_len);

/* handle outbound data streaming DSE interrupt */
void ob_dse_irq_handler(struct rio_irq_handler *h, u32 state);

/* handle inbound VSID interrupt */
void ib_dse_vsid_m_irq_handler(struct rio_irq_handler *h, u32 state);

/* open IBDS data stream */
int axxia_open_ib_data_stream(
    struct rio_mport    *mport,
    void                *dev_id,
    int		        source_id,
    int 	        cos,
    int		        num_pdus,
    int                 desc_size);

/* add IBDS data buffer */
int axxia_add_ibds_buffer(
    struct rio_mport    *mport,
    int                 source_id,
    int                 cos,
    void                *buf);

/* get IBDS data */
void *axxia_get_ibds_data(
    struct rio_mport    *mport,
    int                 source_id,
    int                 cos,
    int                 *ptr_pdu_length,
    int                 *ptr_stream_id);

/* convert VSID to internal VSID */
int axxio_virt_vsid_convert(
    u32     vsid,
    u32     alias_reg,
    u16     *ptr_virt_vsid);

/* close IBDS data streaming */
int axxia_close_ib_data_stream(
    struct rio_mport    *mport,
    int                 source_id,
    int                 cos);

/* close OBDS data streaming */
int axxia_close_ob_data_stream(
    struct rio_mport    *mport,
    int                 dest_id,
    int                 stream_id,
    int                 cos);


/* allocate data streaming buffers */
int axxia_mem_alloc(
    struct rio_ds_priv      *ptr_ds_priv,
    struct rio_ds_dtb_info  *ptr_ds_dtb_info);

/* free data streaming buffers */
int axxia_ds_free(struct rio_ds_priv      *ptr_ds_priv);

/* configure the data streaming data structures */
int axxia_cfg_ds(
    struct rio_mport        *mport,
    struct rio_ds_dtb_info  *ptr_ds_dtb_info);

void release_ob_ds(struct rio_irq_handler *h);

void release_ib_ds(struct rio_irq_handler *h);

#endif /* _AXXIA_RIO_DS_H_ */
