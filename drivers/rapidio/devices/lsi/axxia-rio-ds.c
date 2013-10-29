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

/* #define DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/of_platform.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/dmapool.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/io.h>
/* #include <asm/machdep.h> */
#include <linux/uaccess.h>

#include "axxia-rio.h"
#include "axxia-rio-irq.h"
#include "axxia-rio-ds.h"

#define HW_55XX 1

/* #define ALLOC_BUF_BY_KERNEL 1 */

static inline void __ib_virt_m_dbg(
	struct rio_ds_ibds_vsid_m_stats *ptr_ib_stats,
	u32 virt_m_stat);

static inline void __ob_dse_dbg(
	struct rio_ds_obds_dse_stats *ptr_ob_stats,
	u32 dse_stat);

static inline void __ob_dse_dw_dbg(
	struct rio_ds_obds_dse_stats *ptr_ob_stats,
	u32 dw0);

static inline void __ib_dse_dw_dbg(
	struct rio_ds_ibds_vsid_m_stats *ptr_ib_stats,
	u32 dw0);


static inline void __ib_virt_m_dbg(
	struct rio_ds_ibds_vsid_m_stats *ptr_ib_stats,
	u32 virt_m_stat)
{
	if (virt_m_stat & IB_VIRT_M_STAT_ERROR_MASK) {
		if (virt_m_stat & IB_VIRT_M_STAT_PDU_DROPPED)
			ptr_ib_stats->num_dropped_pdu++;

		if (virt_m_stat & IB_VIRT_M_STAT_SEG_LOSS)
			ptr_ib_stats->num_segment_loss++;

		if (virt_m_stat & IB_VIRT_M_STAT_MTU_LEN_MIS_ERR)
			ptr_ib_stats->num_mtu_len_mismatch_err++;

		if (virt_m_stat & IB_VIRT_M_STAT_PDU_LEN_MIS_ERR)
			ptr_ib_stats->num_pdu_len_mismatch_err++;

		if (virt_m_stat & IB_VIRT_M_STAT_TRANS_ERR)
			ptr_ib_stats->num_data_transaction_err++;

		if (virt_m_stat & IB_VIRT_M_STAT_UPDATE_ERR)
			ptr_ib_stats->num_desc_update_err++;

		if (virt_m_stat & IB_VIRT_M_STAT_TIMEOUT_ERR)
			ptr_ib_stats->num_timeout_err++;

		if (virt_m_stat & IB_VIRT_M_STAT_FETCH_ERR)
			ptr_ib_stats->num_desc_fetch_err++;
	}
}

static inline void __ob_dse_dbg(
	struct rio_ds_obds_dse_stats *ptr_ob_stats,
	u32 dse_stat)
{
	if (dse_stat & OB_DSE_STAT_ERROR_MASK) {
		if (dse_stat & OB_DSE_STAT_TRANS_ERR)
			ptr_ob_stats->num_desc_data_transaction_err++;

		if (dse_stat & OB_DSE_STAT_UPDATE_ERR)
			ptr_ob_stats->num_desc_update_err++;

		if (dse_stat & OB_DSE_STAT_DESC_ERR)
			ptr_ob_stats->num_desc_err++;

		if (dse_stat & OB_DSE_STAT_FETCH_ERR)
			ptr_ob_stats->num_desc_fetch_err++;
	}
}

static inline void __ob_dse_dw_dbg(
	struct rio_ds_obds_dse_stats *ptr_ob_stats,
	u32 dw0)
{
	if (dw0 & OB_DSE_DESC_ERROR_MASK) {
		if (dw0 & OB_HDR_DESC_AXI_ERR)
			ptr_ob_stats->num_desc_axi_err++;
	}
	if (dw0 & OB_HDR_DESC_DONE)
		ptr_ob_stats->num_desc_transferred++;
}

static inline void __ib_dse_dw_dbg(
	struct rio_ds_ibds_vsid_m_stats *ptr_ib_stats,
	u32 dw0)
{
	if (dw0 & IB_DSE_DESC_ERROR_MASK) {
		if (dw0 & IB_DSE_DESC_AXI_ERR)
			ptr_ib_stats->num_desc_axi_err++;

		if (dw0 & IB_DSE_DESC_DS_ERR)
			ptr_ib_stats->num_desc_ds_err++;
	}

	if (dw0 & IB_DSE_DESC_DONE)
		ptr_ib_stats->num_desc_transferred++;
}

/*****************************************************************************
 * axxia_open_ob_data_stream -
 *
 *  This function sets up the descriptor chain to an available OBDS engine.
 *  It programs each header and data descriptors associated with the stream.
 *
 * @mport:	pointer to the master port
 * @destId:	destination ID of the data stream
 * @streamId:   traffic stream ID
 * @cos:	class of service of the stream
 * @numPDUs:	number of PDUs in the stream
 * @pduLen:	PDU length in bytes of the segmented PDU
 * @isEnableEngine: if to enable the DS engine after setting up the chain
 *
 * Returns %0 on success
 ****************************************************************************/
int axxia_open_ob_data_stream(
	struct rio_mport    	*mport,
	void			*dev_id,
	int		 	dest_id,
	int		 	stream_id,
	int		 	cos,
	int		 	num_pdus,
	int		 	pdu_length)
{
	int		 rc = 0;

	axxia_api_lock();

	open_ob_data_stream(mport, dev_id, dest_id, stream_id, cos,
				num_pdus, pdu_length);

	axxia_api_unlock();

	return rc;
}
EXPORT_SYMBOL(axxia_open_ob_data_stream);

/*****************************************************************************
 * open_ob_data_stream -
 *
 *  This function sets up the descriptor chain to an available OBDS engine.
 *  It programs each header and data descriptors associated with the stream.
 *
 * @mport:	pointer to the master port
 * @dev_id:	device specific pointer to pass on event
 * @dest_id:	destination ID of the data stream
 * @stream_id:  traffic stream ID
 * @cos:	class of service of the stream
 * @num_pdus:   number of PDUs in the stream
 * @pdu_length: PDU length in bytes of the segmented PDU
 *
 *  To keep the correct order of the stream, it is recommended that same
 *  stream_id goes to the same DSE descriptor chain.  However, different
 *  stream_ids can go to the same descriptor chain.  To make it simple,
 *  we make stream_id goes to the (stream_id % 16) descriptor chain.
 *
 * Returns %0 on success
 ****************************************************************************/
int open_ob_data_stream(
	struct rio_mport    	*mport,
	void		    	*dev_id,
	int		 	dest_id,
	int		 	stream_id,
	int		 	cos,
	int		 	num_pdus,
	int		 	pdu_length)
{
	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv  *ptr_ds_priv = &(priv->ds_priv_data);
	struct rio_obds_dse_cfg *ptr_dse_cfg;
	struct rio_ds_hdr_desc  *ptr_hdr_desc;
	struct rio_irq_handler *h;
	u8  dse_id;
	u8  find_ava_dse = RIO_DS_FALSE;
	u16 num_hdr_desc_needed;
	u32 h_dw0, h_dw1, h_dw2, h_dw3;
	unsigned long dse_chain_start_addr_phy;
			/* physical address of dse_chain start */
	unsigned long next_desc_ptr_phy;
	u16 pdu_len_per_hdr_desc;
#ifdef ALLOC_BUF_BY_KERNEL
	void *ptr_virt_data_buf;
	u32  data_buf_high;
	u16 i;
	u32 h_dw4;
	unsigned long data_buf_phy;
#endif
	u16 hdr_head_reserved, hdr_index, desc_offset;
	u32 num_data_buf_needed, total_head_reserved;
	u32 des_chain_start_addr_phy_low, des_chain_start_addr_phy_hi;
	u32 dse_ctrl;
	u32 next_desc_high;

	int rc = 0;

	/* sanity checks - TBD */

	/* TBD - ASR_SPINLOCK_INTERRUPT_DISABLE */

	/*
	**  If this is an existing data stream, select a previously matched
	**  DSE to process;
	**  If this is a new data stream, find an available DSE to process.
	*/
	for (dse_id = 0; dse_id < ptr_ds_priv->num_obds_dses; dse_id++) {
		ptr_dse_cfg = &(ptr_ds_priv->obds_dse_cfg[dse_id]);
		if ((ptr_dse_cfg->in_use == RIO_DS_TRUE)    &&
		    (ptr_dse_cfg->dest_id == dest_id)       &&
		    (ptr_dse_cfg->stream_id == stream_id)   &&
		    (ptr_dse_cfg->cos == cos)) {
			find_ava_dse = RIO_DS_TRUE;
			break;
		}
	}

	if (find_ava_dse == RIO_DS_FALSE) {
		for (dse_id = 0;
		     dse_id < ptr_ds_priv->num_obds_dses;
		     dse_id++) {
			if (ptr_ds_priv->obds_dse_cfg[dse_id].in_use ==
				RIO_DS_FALSE) {
				find_ava_dse = RIO_DS_TRUE;
				break;
			}
		}
	}

	/* No DSE available, return error */
	if (find_ava_dse == RIO_DS_FALSE)
		return -EINVAL;

	/* get the DSE cfg */
	ptr_dse_cfg = &(ptr_ds_priv->obds_dse_cfg[dse_id]);

#ifdef FUTURE
	h = &(priv->ds_priv_data.ob_dse_irq[dse_id]);
	if (test_bit(RIO_IRQ_ENABLED, &h->state))
		return -EBUSY;
#endif

	/*
	** NOTE: For the first attemp, just use single descriptor case.
	**
	*/
	pdu_len_per_hdr_desc = pdu_length;

	num_hdr_desc_needed = num_pdus;

	/* check if there are enough free header descriptors */
	if (ptr_dse_cfg->num_hdr_desc_free < num_hdr_desc_needed)
		return -ENOMEM;

	num_data_buf_needed = num_hdr_desc_needed;

	hdr_head_reserved = ptr_dse_cfg->hdr_head_reserved;

	/* get chain start address */
	dse_chain_start_addr_phy =
	  virt_to_phys((void *)
		(&(ptr_ds_priv->ptr_obds_hdr_desc[hdr_head_reserved])));

	/* NOTE: There are two descriptors in the outbound data streaming
	*        (OBDS).
	*
	*  If a PDU length buffer memory is available, then a single descriptor
	*  (header) can be used.
	*  The single header descriptor points to the continous memory which
	*  can hold one PDU data.
	*
	*  If a PDU length buffer is not available, then both header and data
	*  descriptors are needed.  The header descriptor points to the first
	*  data descriptor and data descriptors are chained together.  Each
	*  data descriptor points to a 4K of continuous memory.
	*
	*  For the initial implementation, we only focus on single descriptor
	*  (header descriptor) case.
	*/
	total_head_reserved = 0;

	for (hdr_index = 0; hdr_index < num_hdr_desc_needed; hdr_index++) {
#ifdef ALLOC_BUF_BY_KERNEL
		/* allocate buffers */
		ptr_virt_data_buf = kzalloc(pdu_length, GFP_KERNEL);

		if (ptr_virt_data_buf == NULL) {
			/* free previous allocated buffers */
			for (i = 0; i < total_head_reserved; i++) {
				if (ptr_dse_cfg->hdr_head_reserved == 0)
					desc_offset =
					(ptr_dse_cfg->max_num_hdr_desc - 1);
				else
					desc_offset =
					ptr_dse_cfg->hdr_head_reserved--;

				ptr_hdr_desc =
				&(ptr_ds_priv->ptr_obds_hdr_desc[desc_offset]);

				kfree((void *)ptr_hdr_desc->virt_data_buf);
			}

			ptr_dse_cfg->hdr_head_reserved = hdr_head_reserved;
			ptr_dse_cfg->num_hdr_desc_free++;
		}
#endif

		total_head_reserved++;

		ptr_hdr_desc =
		  &(ptr_ds_priv->ptr_obds_hdr_desc[hdr_head_reserved]);

		/*
		** program the header descriptor word0
		** The int_enable and valid bits are programmed when data
		** is ready to be sent.
		*/
		h_dw0 = 0;
		/* dest_id - [31:16]  */
		h_dw0 |= ((dest_id << 16) & 0xFFFF0000);
		/* descriptor type - bit 2 (1 - header descriptor) */
		h_dw0 |= ((1 << 2) & 0x4);
		/* single_desc - bit 3 (1 - single descriptor) */
		h_dw0 |= ((1 << 3) & 0x8);
		/* next_desc_ptr_valid - bit 1 */
		if (hdr_index != (num_hdr_desc_needed-1))
			h_dw0 |= 2;

		/* program the header descriptor word1 */
		h_dw1 = 0;
		/* stream_id - bits [0:16] */
		h_dw1 |= (stream_id & 0xFFFF);
		/* pdu_len - [17:31] 000 - 64KB  */
		if (pdu_length != RIO_DS_DATA_BUF_64K)
			h_dw1 |= ((pdu_length << 16) & 0xFFFF0000);

		/* program the header descritpor word2 */
		h_dw2 = 0;
		/* cos - bits [2:9] */
		h_dw2 |= ((cos << 2) & 0x3FC);

		/* if this is not the last descriptor */
		if (hdr_index != (num_hdr_desc_needed - 1)) {
			/* program the header descriptor word3 next_desc_addr */
			if (hdr_head_reserved ==
				(ptr_dse_cfg->max_num_hdr_desc - 1))
				desc_offset = ptr_dse_cfg->hdr_desc_offset;
			else
				desc_offset = (ptr_dse_cfg->hdr_desc_offset +
				(hdr_head_reserved + 1));

			next_desc_ptr_phy = virt_to_phys((void *)
			(&(ptr_ds_priv->ptr_obds_hdr_desc[desc_offset])));
		} else {
			/*
			** A Null descriptor is defined as a descriptor with
			** following descriptor fields value:
			**  descriptor valid = 1
			**  next descriptor pointer valid = 1 ?
			**  next descriptor address = 0
			*/
			next_desc_ptr_phy = 0;
		}

#ifdef ALLOC_BUF_BY_KERNEL
		data_buf_phy = virt_to_phys((void *)ptr_virt_data_buf);

		/*
		** data_address - 38-bit AXI addressing
		** data_address[0:31] - h_dw4 [0:31]
		** data_address[32:37] - h_dw2[22:27]
		*/
		h_dw4 = (u32)((data_buf_phy) & 0xFFFFFFFF);

		data_buf_high = (((u64)data_buf_phy >> 32) & 0x3F);
		h_dw2 |= (((data_buf_high) << 22) & 0xFC00000);
#endif

		/*
		** The next_desc_addr - 38-bit AXI addressing
		**  next_desc_addr[37] - h_dw2 [28]
		**  next_desc_addr[36:5] - h_dw3[0:31] - next_desc_address has
		**                                       to be 8 words aligned
		*/
		h_dw3 = (u32)((next_desc_ptr_phy >> 5) & 0xFFFFFFFF);
		next_desc_high = ((u64)next_desc_ptr_phy >> 37) & 0x1;
		h_dw2 |= ((next_desc_high << 28) & 0x10000000);

		/* write into the header descriptors */
		ptr_hdr_desc->dw0 = h_dw0;
		ptr_hdr_desc->dw1 = h_dw1;
		ptr_hdr_desc->dw2 = h_dw2;
		ptr_hdr_desc->dw3 = h_dw3;

#ifdef ALLOC_BUF_BY_KERNEL
		ptr_hdr_desc->dw4 = h_dw4;
		ptr_hdr_desc->remaining_data_len = pdu_length;
		ptr_hdr_desc->offset = 0;
		/* record the data_addr for future usage */
		ptr_hdr_desc->virt_data_buf = (u32)ptr_virt_data_buf;
#endif

		hdr_head_reserved++;

		if (ptr_dse_cfg->hdr_head_reserved ==
			ptr_dse_cfg->max_num_hdr_desc){
			ptr_dse_cfg->hdr_head_reserved = 0;
		}
		ptr_dse_cfg->num_hdr_desc_free--;
	}

	/* update the DSE */
	ptr_dse_cfg->in_use = RIO_DS_TRUE;
	ptr_dse_cfg->dest_id = dest_id;
	ptr_dse_cfg->stream_id = stream_id;
	ptr_dse_cfg->cos = cos;
	ptr_dse_cfg->hdr_head_reserved = hdr_head_reserved;
	ptr_dse_cfg->num_hdr_desc_reserved += num_hdr_desc_needed;

	/*
	** program chain start address
	**
	**  The OBDSE_DESC_ADDR reg holds lower 32 bits of the descriptor
	**  chain address
	**  The RAB_OBDSE_CTRL register holds the upper bit
	**
	**  There is also a requirement for the start of the descriptor chain
	**  address, it has to be 8 words aligned. TBD
	*/
	des_chain_start_addr_phy_low =
		(dse_chain_start_addr_phy >> 5) & 0xFFFFFFFF;
	des_chain_start_addr_phy_hi =
		(((u64)dse_chain_start_addr_phy >> 37)) & 0x1;

	__rio_local_read_config_32(mport, RAB_OBDSE_CTRL(dse_id), &dse_ctrl);
	dse_ctrl |= ((des_chain_start_addr_phy_hi << 31) & 0x80000000);
	__rio_local_write_config_32(mport, RAB_OBDSE_CTRL(dse_id), dse_ctrl);
	__rio_local_write_config_32(mport, RAB_OBDSE_DESC_ADDR(dse_id),
				des_chain_start_addr_phy_low);

	/*
	** Create IRQ handler and enable data stream irq
	*/
	h = &(ptr_ds_priv->ob_dse_irq[dse_id]);

	sprintf(ptr_dse_cfg->name, "obds-%d", dse_id);
	rc = alloc_irq_handler(h, (void *)ptr_dse_cfg, ptr_dse_cfg->name);

	return rc;
}

/*****************************************************************************
 * axxia_add_ob_data_stream -
 *
 *  This function copies data to an OBDS data buffer.
 *
 * @mport:	  pointer to the master port
 * @dest_id:	 destination ID of the data stream
 * @stream_id:       traffic stream ID
 * @cos:	    class of service of the stream
 * @buffer:	 pointer to where the data is stored
 * @dataLength:     data length in bytes to be copied
 *
 * Returns %0 on success
 ****************************************************************************/
int axxia_add_ob_data_stream(
	struct rio_mport	*mport,
	int		     	dest_id,
	int		     	stream_id,
	int		     	cos,
	void		    	*buffer,
	int		     	data_len)
{
	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv      *ptr_ds_priv;
	struct rio_obds_dse_cfg *ptr_dse_cfg;
	struct rio_ds_hdr_desc  *ptr_hdr_desc;
	u16     dse_id;
	u8      find_ava_dse = RIO_DS_FALSE;
	u32     dse_ctrl;
#ifndef ALLOC_BUF_BY_KERNEL
	u32  data_buf_high;
	unsigned long data_buf_phy;
#endif
	u32	    pdu_length;

	int rc = 0;

	/* sanity check - TBD */

	ptr_ds_priv = &(priv->ds_priv_data);

	/* find the engine that is used to process the data stream */
	for (dse_id = 0; dse_id < (ptr_ds_priv->num_obds_dses); dse_id++) {
		ptr_dse_cfg = &(ptr_ds_priv->obds_dse_cfg[dse_id]);

		if ((ptr_dse_cfg->in_use == RIO_DS_TRUE)    &&
		    (ptr_dse_cfg->dest_id == dest_id)       &&
		    (ptr_dse_cfg->stream_id == stream_id)   &&
		    (ptr_dse_cfg->cos == cos)) {
			find_ava_dse = RIO_DS_TRUE;
			break;
		}
	}

	/* if there is no matched DSE, return error */
	if (find_ava_dse == RIO_DS_FALSE)
		return -EINVAL;

	/* handle single descriptor for now one PDU a time */
	/* multi-descriptors case TBD */
	ptr_hdr_desc = &(ptr_ds_priv->ptr_obds_hdr_desc[ptr_dse_cfg->hdr_head]);

	/* copy the data to the buffer */
	/* How to handle 8K limitation of the ioctrl( ) TBD */
	pdu_length = (ptr_hdr_desc->dw1 >> 16) & 0xFFFF;
#ifdef ALLOC_BUF_BY_KERNEL

	memcpy((void *)(ptr_hdr_desc->virt_data_buf + ptr_hdr_desc->offset),
		buffer, data_len);

	if (ptr_hdr_desc->remaining_data_len <= data_len) {
		/* set the en_int and valid bit of the header descriptor */
		ptr_hdr_desc->dw0 |= 0x1;
		ptr_hdr_desc->dw0 |= ((1 << 5) & 0x20);

		/* start, wake up the engine */
		__rio_local_read_config_32(mport, RAB_OBDSE_CTRL(dse_id),
					&dse_ctrl);
		dse_ctrl |= DSE_WAKEUP | DSE_ENABLE;
		__rio_local_write_config_32(mport, RAB_OBDSE_CTRL(dse_id),
					dse_ctrl);

		ptr_dse_cfg->hdr_head++;

		ptr_dse_cfg->num_hdr_desc_reserved--;
	}
	ptr_hdr_desc->remaining_data_len -= data_len;
	ptr_hdr_desc->offset += data_len;
#else

	/*
	** The kernel driver expect that the higher driver allocates the data
	**  buffer and use the buffer. In the OBDS interrupt service routine,
	**  the kernel frees the buffer.
	**
	**  Thus, there is no extra copy needed here.
	**  memcpy((void *)(ptr_hdr_desc->virt_data_buf), buffer, data_len);
	*/
	/* Check if the data length is equal to the pdu length in the
	** axxia_open_ob_data_stream( )
	*/
	if (data_len < pdu_length)
		return -EINVAL;

	/* Program the data buffer in the descriptor */
	data_buf_phy = virt_to_phys(buffer);

	/*
	** data_address - 38-bit AXI addressing
	** data_address[0:31] - h_dw4 [0:31]
	** data_address[32:37] - h_dw2[22:27]
	*/
	ptr_hdr_desc->dw4 = (u32)((data_buf_phy) & 0xFFFFFFFF);

	data_buf_high = (((u64)data_buf_phy >> 32) & 0x3F);
	ptr_hdr_desc->dw2 |= (((data_buf_high) << 22) & 0xFC00000);

	/* set the en_int and valid bit of the header descriptor */
	ptr_hdr_desc->dw0 |= 0x1;
	ptr_hdr_desc->dw0 |= ((1 << 5) & 0x20);

	/* start, wake up the engine */
	__rio_local_read_config_32(mport, RAB_OBDSE_CTRL(dse_id), &dse_ctrl);
	dse_ctrl |= DSE_WAKEUP | DSE_ENABLE;
	__rio_local_write_config_32(mport, RAB_OBDSE_CTRL(dse_id), dse_ctrl);

	ptr_dse_cfg->hdr_head++;

	ptr_dse_cfg->num_hdr_desc_reserved--;
#endif

	return rc;
}
EXPORT_SYMBOL(axxia_add_ob_data_stream);

/**
 * ob_dse_irq_handler - Outbound data streaming interrupt handler
 * --- Called in threaded irq handler ---
 * @h: Pointer to interrupt-specific data
 *
 * Handles outbound data streaming interrupts.  Executes a callback,
 * if available, on each successfully sent data stream.
 *
*/
void ob_dse_irq_handler(struct rio_irq_handler *h, u32 state)
{
	struct rio_mport *mport = h->mport;
	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv      *ptr_ds_priv;
	struct rio_obds_dse_cfg *ptr_dse_cfg;
	struct rio_ds_hdr_desc  *ptr_hdr_desc = h->data;
	u32 dse_stat, dse_id;
	u16 hdr_tail;
	unsigned long flags;

	/* Find the DSE that gets interrupted, CNTLZW found the upper
	** bit first */
	dse_id = 31 - CNTLZW(state);

	/* find out DSE stats */
	__rio_local_read_config_32(mport, RAB_OBDSE_STAT(dse_id), &dse_stat);

	ptr_ds_priv = &(priv->ds_priv_data);

	/**
	 * Wait for all pending transactions to finish before doing descriptor
	 * updates
	 */
	ptr_dse_cfg = &(ptr_ds_priv->obds_dse_cfg[dse_id]);
	spin_lock_irqsave(&ptr_dse_cfg->lock, flags);

	/*
	** It is possible that one DSE handles multiple data streams,
	** thus the error condition does not reflect a specific descriptor
	** condition. We log the DSE stats but report per descriptor error
	** condition.
	*/
	/* check DSE registers for error reports */
	__ob_dse_dbg(&(ptr_ds_priv->ob_dse_stats[dse_id]), dse_stat);

	/* process all completed transactions - bit 1 - descriptor transaction
	** completed */
	if (dse_stat & 0x2) {
		if (ptr_dse_cfg->hdr_tail == ptr_dse_cfg->hdr_head) {
			spin_unlock_irqrestore(&ptr_dse_cfg->lock, flags);
			return;
		}

		while (ptr_dse_cfg->hdr_tail != ptr_dse_cfg->hdr_head) {
			hdr_tail = ptr_dse_cfg->hdr_tail;
			ptr_hdr_desc =
			&(ptr_ds_priv->ptr_obds_hdr_desc[hdr_tail]);

			__ob_dse_dw_dbg(&(ptr_ds_priv->ob_dse_stats[dse_id]),
				ptr_hdr_desc->dw0);

			/*
			** check if the descriptor is done - bit 8
			**
			**  Since the IRQ process the descriptor one-by-one,
			**  if the descriptor pointed by tail is not processed,
			**  the following ones are not processed either.
			*/
			if ((ptr_hdr_desc->dw0 & OB_HDR_DESC_DONE) == 0)
				break;

			/* set the valid bit to be zero */
			ptr_hdr_desc->dw0 &= 0xFFFFFFFE;

			/* free the buffer */
			kfree((void *)ptr_hdr_desc->virt_data_buf);

			ptr_dse_cfg->data_tail++;

			if (ptr_dse_cfg->data_tail >
				(ptr_dse_cfg->max_num_hdr_desc - 1)) {
				ptr_dse_cfg->data_tail = 0;
			}

			/* the descriptor has been processed, but there is
			** error occurred */
			if ((ptr_hdr_desc->dw0 & OB_HDR_DESC_AXI_ERR)) {
				__ob_dse_dw_dbg(
					&(ptr_ds_priv->ob_dse_stats[dse_id]),
					ptr_hdr_desc->dw0);
				/* callback function - TBD */
			}
		}
	}

	spin_unlock_irqrestore(&ptr_dse_cfg->lock, flags);
	return;
}

/*****************************************************************************
 * axxia_close_ob_data_stream -
 *
 *  This function resets variables associated with a OBDS.
 *
 * @mport:	 pointer to the master port
 * @destId:	 destination ID of the data stream
 * @streamId:    traffic stream ID
 * @cos:	 class of service of the stream
 *
 * Returns %0 on success
 ****************************************************************************/
int axxia_close_ob_data_stream(
	struct rio_mport    	*mport,
	int		 	dest_id,
	int		 	stream_id,
	int		 	cos)
{
	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv      *ptr_ds_priv = &(priv->ds_priv_data);
	struct rio_obds_dse_cfg *ptr_dse_cfg;
	struct rio_ds_hdr_desc  *ptr_hdr_desc;
	u16    dse_id, i, offset;
	u8     find_ava_dse = RIO_DS_FALSE;
	u32    dse_ctrl;

	axxia_api_lock();

	for (dse_id = 0; dse_id < ptr_ds_priv->num_obds_dses; dse_id++) {
		ptr_dse_cfg = &(ptr_ds_priv->obds_dse_cfg[dse_id]);
		if ((ptr_dse_cfg->in_use == RIO_DS_TRUE)    &&
		    (ptr_dse_cfg->dest_id == dest_id)       &&
		    (ptr_dse_cfg->stream_id == stream_id)   &&
		    (ptr_dse_cfg->cos == cos)) {
			find_ava_dse = RIO_DS_TRUE;
			break;
		}
	}

	if (find_ava_dse == RIO_DS_FALSE) {
		axxia_api_unlock();
		return 0;
	}

	/* reset variables */
	ptr_dse_cfg->in_use = RIO_DS_FALSE;
	ptr_dse_cfg->data_head = 0;
	ptr_dse_cfg->data_tail = 0;
	ptr_dse_cfg->data_head_reserved = 0;
	ptr_dse_cfg->hdr_head = 0;
	ptr_dse_cfg->hdr_tail = 0;
	ptr_dse_cfg->hdr_head_reserved = 0;
	ptr_dse_cfg->num_hdr_desc_free = ptr_dse_cfg->max_num_hdr_desc;
	ptr_dse_cfg->num_data_desc_free = ptr_dse_cfg->max_num_data_desc;

	/*
	** Since one DSE handles one data streaming, reset the corresponding
	** descriptors now
	*/
	offset = ptr_dse_cfg->hdr_desc_offset;
	for (i = 0; i < ptr_dse_cfg->max_num_hdr_desc; i++) {
		ptr_hdr_desc = &(ptr_ds_priv->ptr_obds_hdr_desc[i + offset]);

		/* set valid bit to be zero */
		ptr_hdr_desc->dw0 &= 0xFFFFFFFE;

		/* free the data buffer */
		if ((void *)ptr_hdr_desc->virt_data_buf != NULL)
			kfree((void *)ptr_hdr_desc->virt_data_buf);
	}

	/* Disable the corresponding DSE */
	__rio_local_read_config_32(mport, RAB_OBDSE_CTRL(dse_id), &dse_ctrl);
	dse_ctrl &= 0xFFFFFFFE;
	__rio_local_write_config_32(mport, RAB_OBDSE_CTRL(dse_id), dse_ctrl);

	axxia_api_unlock();

	return 0;
}
EXPORT_SYMBOL(axxia_close_ob_data_stream);

/**
 * ib_dse_vsid_m_irq_handler - Inbound data streaming interrupt handler
 * --- Called in threaded irq handler ---
 * @h: Pointer to interrupt-specific data
 *
 * Handles outbound data streaming interrupts.  Executes a callback,
 * if available, on each successfully sent data stream
 *
*/
void ib_dse_vsid_m_irq_handler(struct rio_irq_handler *h, u32 state)
{
	struct rio_mport *mport = h->mport;
	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv      *ptr_ds_priv = &(priv->ds_priv_data);
	struct ibds_virt_m_cfg  *ptr_virt_m_cfg;
	struct rio_ids_data_desc    *ptr_data_desc;
	u32 dse_stat, vsid_m_stats;
	u8  virt_vsid, dse_id;
	u16 data_tail;
	u8  found_dse = RIO_DS_FALSE;
	unsigned long flags;

	virt_vsid = 31 - CNTLZW(state);

	__rio_local_read_config_32(mport,
				RAB_IBVIRT_M_STAT(virt_vsid),
				&vsid_m_stats);

	/* check if the chain transfer complete */
	ptr_virt_m_cfg = &(ptr_ds_priv->ibds_vsid_m_cfg[virt_vsid]);

	spin_lock_irqsave(&ptr_virt_m_cfg->lock, flags);

	/* check errors */
	__ib_virt_m_dbg(&(ptr_ds_priv->ib_vsid_m_stats[virt_vsid]),
			vsid_m_stats);

	/* find the engine that handles this VSID */
	for (dse_id = 0; dse_id < RIO_MAX_NUM_IBDS_DSE; dse_id++) {
		__rio_local_read_config_32(mport,
					RAB_IBDSE_STAT(dse_id),
					&dse_stat);

		if ((dse_stat & IB_DSE_VSID_IN_USED) == virt_vsid) {
			found_dse = RIO_DS_TRUE;
			break;
		}
	}

	/* could not find the DSE that processes the VSID */
	if (found_dse == RIO_DS_FALSE) {
		spin_unlock_irqrestore(&ptr_virt_m_cfg->lock, flags);
			return;
	}

	if (vsid_m_stats & IB_VIRT_M_STAT_FETCH_ERR) {
		/*
		** If transaction pending bit is not set, a timeout is also
		** not set, which means that PDU was successfully written
		** into AXI memory and nothing needs to be done.
		** If transaction pending bit is set or timeout is set,
		** engine needs to be reset.  After disabling engine, when
		** transaction pending gets reset, engine is ready to be
		** enabled again.
		*/
		if ((dse_stat & IB_DSE_STAT_TRANS_PENDING)  ||
			(dse_stat & IB_DSE_STAT_TIMEOUT)) {
			/*
			** BZ43821 - SW workaround for the IBDS descriptor
			** fetch error.  When S/W sees the descriptor fetch
			** error being indicated in status bits, introduce
			** a delay and then disable the engine and enable
			** the engine again.  With this change, the next
			** incoming packet for that engine would not get
			** corrupted.
			*/
			ndelay(5);

			/* Disable the engine */
			__rio_local_write_config_32(mport,
						RAB_IBDSE_CTRL(dse_id),
						0);

			/* Should wait till the pending bit is reset? - TBD */

			/* Enable the engine again */
			__rio_local_write_config_32(mport,
						RAB_IBDSE_CTRL(dse_id),
						1);
		}
	}

	/* In case of timeout error, if not alreaday disabled, descriptor
	**	prefetch logic should be disabled and associated descriptor
	**	start address needs to be set for VSID PDUs to be
	**	eassembled again. Engine should be disabled, once
	** 	transaction pending gets reset, engine can be enabled again.
	**	TBD
	*/

	/* process the completed transactions */
	if (vsid_m_stats & 0x1) {
		if (ptr_virt_m_cfg->data_tail == ptr_virt_m_cfg->data_head) {
			spin_unlock_irqrestore(&ptr_virt_m_cfg->lock, flags);
			return;
		}

		while (ptr_virt_m_cfg->data_tail != ptr_virt_m_cfg->data_head) {
			data_tail = ptr_virt_m_cfg->data_tail;
			ptr_data_desc =
				&(ptr_ds_priv->ptr_ibds_data_desc[data_tail]);

			__ib_dse_dw_dbg(
				&(ptr_ds_priv->ib_vsid_m_stats[virt_vsid]),
				ptr_data_desc->dw0);

			if ((ptr_data_desc->dw0 & IB_DSE_DESC_DONE) == 0)
				break;

			/* set the valid bit to be zero */
			ptr_data_desc->dw0 &= 0xFFFFFFFE;

			ptr_virt_m_cfg->data_tail++;

			if (ptr_virt_m_cfg->data_tail >
				(ptr_virt_m_cfg->max_num_data_desc - 1)) {
				ptr_virt_m_cfg->data_tail = 0;
			}

			/* callback function - TBD */
		}
	}

	spin_unlock_irqrestore(&ptr_virt_m_cfg->lock, flags);

	return;
}

/*****************************************************************************
 * axxia_open_ib_data_stream -
 *
 *  This function sets up the descriptor chain to an internal VSID M.
 *  The internal VSID is calculated through sourceId, cos and
 *  RAB_IBDS_VSID_ALIAS register.
 *
 * @mport:	pointer to the master port
 * @sourceId:   source ID of the data stream
 * @cos:	class of service of the stream
 * @numPDUs:	number of PDUs in the stream
 * @descSize:   PDU size this descriptor can handle. If the PDU size exceeds
 *		descriptor size (but less than maximum PDU size), appropriate
 *		error bit is set and interrupt is generated if enabled.
 *		PDU data beyond descriptor size is not written in the AXI
 *		memory to ensure that there is no corruption of data under
 *		this error case.
 *
 * Returns %0 on success
 ****************************************************************************/
int axxia_open_ib_data_stream(
	struct rio_mport    	*mport,
	void			*dev_id,
	int			source_id,
	int 		    	cos,
	int			num_pdus,
	int		 	desc_size)
{
	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv  *ptr_ds_priv = &(priv->ds_priv_data);
	struct ibds_virt_m_cfg  *ptr_virt_m_cfg;
	struct rio_ids_data_desc *ptr_data_desc;
	struct rio_irq_handler *h;
	u32     alias_reg;
	u32     vsid;
	u16     virt_vsid, num_data_desc_needed, data_head_reserved,
		total_head_reserved;
	unsigned long     	desc_chain_start_addr_phy, next_desc_addr_phy;

	u32     next_desc_addr_hi, vsid_addr_reg;
	u16     data_index, desc_offset;

	int rc = 0;

#ifdef ALLOC_BUF_BY_KERNEL
	unsigned long   data_addr_phy;
	u32 data_addr_hi;
	u16 i;
	void    *ptr_virt_data_buf;
#endif

	axxia_api_lock();

	/* TBD ASR_SPINLOCK_INTERRUPT_DISABLE(&priv->ioLock, lflags); */

	/* find the mapping between incoming VSID and internal VSID */
	__rio_local_read_config_32(mport, RAB_IBDS_VSID_ALIAS, &alias_reg);

	/* VSID = {16'b SourceID, 8'bCOS} */
	vsid = ((source_id & 0xFFFF) << 16) | (cos & 0xFF);

#ifndef HW_55XX
	alias_reg = 0x543210;  /* NING - TO_BE_REMOVED */
#endif

	/* calculate the virtual M index */
	(void)axxio_virt_vsid_convert(vsid, alias_reg, &virt_vsid);

	if (virt_vsid >= RIO_MAX_NUM_IBDS_VSID_M)
		return RC_TBD;

	/*
	** In the IBDS, the descriptor size allocated must be greater than
	** or equal to the PDU length.  The descriptor size can be 1K, 2K,
	** 4K, 8K, 16K, 32K, or 64K.
	*/
	/* get a internal VSID M based on virt_vsid */
	ptr_virt_m_cfg = &(ptr_ds_priv->ibds_vsid_m_cfg[virt_vsid]);

	ptr_virt_m_cfg->in_use = RIO_DS_TRUE;

	/* set up the descriptor chain for the internal VSID */
	num_data_desc_needed = num_pdus;

	if (num_data_desc_needed > ptr_virt_m_cfg->num_desc_free)
		return -ENOMEM;

	data_head_reserved = ptr_virt_m_cfg->data_head;

	desc_chain_start_addr_phy =
	virt_to_phys((void *)
		&(ptr_ds_priv->ptr_ibds_data_desc[data_head_reserved]));

	total_head_reserved = 0;

	for (data_index = 0; data_index < num_data_desc_needed; data_index++) {

#ifdef ALLOC_BUF_BY_KERNEL
		ptr_virt_data_buf = kzalloc(desc_size, GFP_KERNEL);
		if (ptr_virt_data_buf == NULL) {
			/* free previous allocated buffers */
			for (i = 0; i < total_head_reserved; i++) {
				if (ptr_virt_m_cfg->data_head_reserved == 0) {
					desc_offset =
					(ptr_virt_m_cfg->max_num_data_desc - 1);
				} else {
					desc_offset =
					ptr_virt_m_cfg->data_head_reserved--;
				}

				ptr_data_desc =
				&(ptr_ds_priv->ptr_ibds_data_desc[desc_offset]);

				kfree((void *)ptr_data_desc->virt_data_buf);
			}

			return -ENOMEM;
		}
#endif
		ptr_virt_m_cfg->data_head_reserved = data_head_reserved;
		ptr_virt_m_cfg->num_desc_free++;

		total_head_reserved++;

		ptr_data_desc =
		&(ptr_ds_priv->ptr_ibds_data_desc[data_head_reserved]);

		/* init the data descriptor */
		memset((void *)ptr_data_desc,
			0,
			sizeof(struct rio_ids_data_desc));

		/* dw0 - desc_size, bits [4:6] - the desc_size is not
		**                               actual size, it is numbered,
		**                               e.g. 0 - 64K, 1 - 1K etc. */
		ptr_data_desc->dw0 |= ((1 << 4) & 0x70);
		/* dw0 - source_id, bits [16:31] */
		ptr_data_desc->dw0 |= ((source_id << 16) & 0xFFFF0000);

		if (data_index < (num_data_desc_needed - 1)) {
			/* dw0 - set next_desc_ptr_valid (bit 1) to 1
			**       if it is not the last one */
			ptr_data_desc->dw0 |= 0x2;
		}

		/* find the next_desc_addr */
		if (data_head_reserved ==
			(ptr_virt_m_cfg->max_num_data_desc - 1)) {
			desc_offset = ptr_virt_m_cfg->data_desc_offset;
		} else {
			desc_offset =
			(ptr_virt_m_cfg->data_desc_offset +
			data_head_reserved
			+ 1);
		}

		/* cos - bit[16:23] */
		ptr_data_desc->dw2 |= ((cos << 16) & 0xFF0000);
		/*
		** next_desc_addr - 38-bit AXI addressing
		**  next_desc_addr[37] - dw2[24]
		**  next_desc_addr[36:5] - dw4[31:0]
		*/
		next_desc_addr_phy =
		virt_to_phys((void *)
				&ptr_ds_priv->ptr_ibds_data_desc[desc_offset]);
		next_desc_addr_hi = ((u64)next_desc_addr_phy >> 37) & 0x1;

		ptr_data_desc->dw4 =
		((u64)next_desc_addr_phy << 5) & 0xFFFFFFFF;
		ptr_data_desc->dw2 |= (next_desc_addr_hi << 24) & 0x1000000;

#ifdef ALLOC_BUF_BY_KERNEL
		/*
		** data_addr - 38-bit AXI addressing
		**  data_addr[31:0] - dw3[31:0]
		**  data_addr[37:32] - dw2[31:26]
		*/
		ptr_data_desc->virt_data_buf = (u32)ptr_virt_data_buf;

		data_addr_phy =
		virt_to_phys((void *)ptr_data_desc->virt_data_buf);

		ptr_data_desc->dw3 = ((u64)data_addr_phy & 0xFFFFFFFF);
		data_addr_hi = ((u64)data_addr_phy >> 32) & 0x3F;
		ptr_data_desc->dw2 |= (data_addr_hi << 26) & 0xFC000000;
#endif
		data_head_reserved++;

		ptr_virt_m_cfg->data_head_reserved++;
		if (ptr_virt_m_cfg->data_head_reserved ==
			ptr_virt_m_cfg->max_num_data_desc)
			ptr_virt_m_cfg->data_head_reserved = 0;

		ptr_virt_m_cfg->num_desc_free--;
	}

	/* update the virt_m_cfg */
	if (num_data_desc_needed > 0) {
		ptr_virt_m_cfg->in_use = 1;
		ptr_virt_m_cfg->cos = cos;
		ptr_virt_m_cfg->source_id = source_id;

		/*
		** desc_chain_start_addr - 38-bit AXI address
		**  M_LOW_ADDR[31:0] - chain_addr[36:5] - has to be 8 bytes
		**                                        alignment
		**  M_HIGH_ADDR[0] - chain_addr[37]
		*/
		/* program the start address of the descriptor chain */
		vsid_addr_reg = (desc_chain_start_addr_phy >> 5) & 0xFFFFFFFF;
		__rio_local_write_config_32(mport,
			RAB_IBDS_VSID_ADDR_LOW(virt_vsid), vsid_addr_reg);

		__rio_local_read_config_32(mport,
			RAB_IBDS_VSID_ADDR_HI(virt_vsid), &vsid_addr_reg);
		vsid_addr_reg |= (((u64)desc_chain_start_addr_phy >> 37) & 0x1);
		__rio_local_write_config_32(mport,
			RAB_IBDS_VSID_ADDR_HI(virt_vsid), vsid_addr_reg);
	}

	/* register IRQ */
	h = &(ptr_ds_priv->ib_dse_vsid_irq[virt_vsid]);

	sprintf(ptr_virt_m_cfg->name, "ibds-%d", virt_vsid);

	rc = alloc_irq_handler(h, (void *)ptr_virt_m_cfg, ptr_virt_m_cfg->name);

	axxia_api_unlock();

	return rc;
}
EXPORT_SYMBOL(axxia_open_ib_data_stream);

/*****************************************************************************
 * axxia_add_ibds_buffer -
 *
 *  This function adds buffer to the AXXIA inbound data stream queue.
 *
 * @mport:	pointer to the master port
 * @sourceId:	source ID of the data stream
 * @cos:	class of service of the stream
 * @buffer:	pointer to where the data is copied to
 *
 * Returns %0 on success
 ****************************************************************************/
int axxia_add_ibds_buffer(
	struct rio_mport   *mport,
	int		   source_id,
	int		   cos,
	void		  *buf)
{
	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv      *ptr_ds_priv = &(priv->ds_priv_data);
	struct ibds_virt_m_cfg  *ptr_virt_m_cfg;
	struct rio_ids_data_desc *ptr_data_desc;
	u32		     	m_id;
	u8		      	found_one = RIO_DS_FALSE;

#ifndef ALLOC_BUF_BY_KERNEL
	unsigned long   data_addr_phy;
	u32 data_addr_hi;
#endif

	if (buf == NULL)
		return -EINVAL;

	/* search through the virtual M table to find the one that has
	**  the same source_id and cos */
	for (m_id = 0; m_id < RIO_MAX_NUM_IBDS_VSID_M; m_id++) {
		ptr_virt_m_cfg = &(ptr_ds_priv->ibds_vsid_m_cfg[m_id]);

		if ((ptr_virt_m_cfg->source_id == source_id)    &&
		    (ptr_virt_m_cfg->cos == cos)		&&
		    (ptr_virt_m_cfg->in_use == RIO_DS_TRUE)) {
			found_one = RIO_DS_TRUE;
			break;
		}
	}

	if (found_one == RIO_DS_FALSE)
		return RC_TBD;

	/* put user's buffer into the corresponding descriptors */
	ptr_data_desc =
	&(ptr_ds_priv->ptr_ibds_data_desc[ptr_virt_m_cfg->user_buf_index_head]);

	ptr_data_desc->usr_virt_data_buf = (u32)buf;

#ifndef ALLOC_BUF_BY_KERNEL
	data_addr_phy =
		virt_to_phys((void *)ptr_data_desc->virt_data_buf);

	ptr_data_desc->dw3 = ((u64)data_addr_phy & 0xFFFFFFFF);
	data_addr_hi = ((u64)data_addr_phy >> 32) & 0x3F;
	ptr_data_desc->dw2 |= (data_addr_hi << 26) & 0xFC000000;

	ptr_data_desc->virt_data_buf = (u32)buf;
#endif

	if (ptr_virt_m_cfg->user_buf_index_head ==
		(ptr_virt_m_cfg->max_num_data_desc - 1)) {
		ptr_virt_m_cfg->user_buf_index_head = 0;
	} else {
		ptr_virt_m_cfg->user_buf_index_head++;
	}

	return 0;
}
EXPORT_SYMBOL(axxia_add_ibds_buffer);

/*****************************************************************************
 * axxia_get_ibds_data -
 *
 *  This function gets IBDS data from data buffer. The ioctl() can only
 *  transfer up to 4KB each time, thus, if a PDU length is larger than
 *  4KB, an application must call this API multiple times.
 *
 *  The maximum PDU length data is copied to the pre-allocated 64K buffer
 *  in init time.
 *
 * @mport:	pointer to the master port
 * @sourceId:	source ID of the data stream
 * @cos:	class of service of the stream
 * @buffer:	pointer to where the data is copied to
 * @streamId:	pointer to where the streamID is saved
 * @pPduLength:	PDU length of the data stream
 * @pDataLenth:	data length in this transfer
 * @pRemainingDataSize: remaining data in bytes to be expected
 *
 * Returns %0 on success
 ****************************************************************************/
void *axxia_get_ibds_data(
	struct rio_mport   *mport,
	int		   source_id,
	int		   cos,
	int		   *ptr_pdu_length,
	int		   *ptr_stream_id)
{
	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv      *ptr_ds_priv = &(priv->ds_priv_data);
	struct ibds_virt_m_cfg  *ptr_virt_m_cfg;
	struct rio_ids_data_desc *ptr_data_desc;
	u32		    m_id;
	u8		    found_one = RIO_DS_FALSE;
	void		    *user_buf;

#ifndef HW_55XX
	u32		    i;
	char		    *ptr_data;
#endif

	/* search through the virtual M table to find the one that
	** has the same source_id and cos */
	for (m_id = 0; m_id < RIO_MAX_NUM_IBDS_VSID_M; m_id++) {
		ptr_virt_m_cfg = &(ptr_ds_priv->ibds_vsid_m_cfg[m_id]);

		if ((ptr_virt_m_cfg->source_id == source_id)    &&
		    (ptr_virt_m_cfg->cos == cos)		&&
		    (ptr_virt_m_cfg->in_use == RIO_DS_TRUE)) {
			found_one = RIO_DS_TRUE;
			break;
		}
	}

	if (found_one == RIO_DS_FALSE)
		return NULL;

#ifndef HW_55XX
	ptr_virt_m_cfg->is_desc_chain_tran_completed = 1;
#endif
	/*
	** To keep it simple, the software waits till "Descriptor Chain
	** Transfer Completed" bit is set in RAB_IBDSE_VSID_M_STAT register.
	**
	** The is_desc_chain_tran_completed is set in the IBDS interrupt
	** routine.
	**
	** Under current ioctrl() implementation, only 4KB is copied
	*/
	if (ptr_virt_m_cfg->is_desc_chain_tran_completed == RIO_DS_TRUE) {
		/* get a data descriptor */
		if (ptr_virt_m_cfg->data_tail >
			(ptr_virt_m_cfg->max_num_data_desc - 1)) {
			return NULL;
		}

		/* get the data buffer */
		ptr_data_desc =
		&(ptr_ds_priv->ptr_ibds_data_desc[ptr_virt_m_cfg->data_tail]);

		/* check if the source_id and cos matches */
		if ((((ptr_data_desc->dw0 >> 16) & 0xFFFF) != source_id) ||
		    ((ptr_data_desc->dw2 & 0xFF0000) >> 16) != cos) {
			return NULL;
		}

		/* since we will change the ioctrl() into sysfs, limit the
		** buffer size to be 8K for now */
		ptr_virt_m_cfg->pdu_len =
			((ptr_data_desc->dw1 & 0xFFFF0000) >> 16);

#ifndef HW_55XX
		ptr_virt_m_cfg->pdu_len = 64;
		ptr_data = (char *)ptr_data_desc->virt_data_buf;
		for (i = 0; i < ptr_virt_m_cfg->pdu_len; i++)
			ptr_data[i] = i;
#endif

#ifdef ALLOC_BUF_BY_KERNEL
		user_buf = (void *)ptr_data_desc->usr_virt_data_buf;
#else
		user_buf = (void *)ptr_data_desc->virt_data_buf;
#endif
		if (user_buf == NULL) {
			*ptr_pdu_length = 0;
			return NULL;
		} else {
#ifdef ALLOC_BUF_BY_KERNEL
			memcpy((void *)(user_buf),
				(void *)(ptr_data_desc->virt_data_buf),
				ptr_ds_priv->max_pdu_len);

			/* free the buffer */
			kfree((void *)ptr_data_desc->virt_data_buf);
#endif

			*ptr_pdu_length = ptr_virt_m_cfg->pdu_len;

			/* stream_id is the lower 16-bit of word1 */
			*ptr_stream_id = (ptr_data_desc->dw1) & 0xFFFF;

			ptr_virt_m_cfg->data_tail++;

			ptr_virt_m_cfg->is_desc_chain_tran_completed = 0;

			return user_buf;
		}
	}

	return NULL;
}
EXPORT_SYMBOL(axxia_get_ibds_data);

/*****************************************************************************
 * axxia_close_ib_data_stream -
 *
 *  This function resets variables associated with a IBDS data stream.
 *
 * @mport:	  pointer to the master port
 * @sourceId:     source ID of the data stream
 * @cos:	  class of service of the stream
 *
 * Returns %0 on success
 ****************************************************************************/
int axxia_close_ib_data_stream(
	struct rio_mport *mport,
	int		 source_id,
	int		 cos)
{
	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv  *ptr_ds_priv = &(priv->ds_priv_data);
	struct ibds_virt_m_cfg  *ptr_virt_m_cfg;
	struct rio_ids_data_desc *ptr_data_desc;
	u8		      find_ava_virt_m = RIO_DS_FALSE;
	u8      i;
	u16     offset;

	axxia_api_lock();

	for (i = 0; i < (ptr_ds_priv->num_ibds_virtual_m); i++) {
		ptr_virt_m_cfg = &(ptr_ds_priv->ibds_vsid_m_cfg[i]);

		if ((ptr_virt_m_cfg->in_use == RIO_DS_TRUE)     &&
		    (ptr_virt_m_cfg->source_id == source_id)    &&
		    (ptr_virt_m_cfg->cos == cos)) {
			find_ava_virt_m = RIO_DS_TRUE;
			break;
		}
	}

	if (find_ava_virt_m == RIO_DS_FALSE) {
		axxia_api_unlock();
		return 0;
	}

	/* reset variables */
	ptr_virt_m_cfg->in_use = RIO_DS_FALSE;
	ptr_virt_m_cfg->data_head = 0;
	ptr_virt_m_cfg->data_tail = 0;
	ptr_virt_m_cfg->user_buf_index_head = 0;
	ptr_virt_m_cfg->data_head_reserved = 0;
	ptr_virt_m_cfg->is_desc_chain_tran_completed = RIO_DS_FALSE;
	ptr_virt_m_cfg->num_desc_free = ptr_virt_m_cfg->max_num_data_desc;

	/* reset the data descriptors */
	offset = ptr_virt_m_cfg->data_desc_offset;
	for (i = 0; i < (ptr_virt_m_cfg->max_num_data_desc); i++) {
		ptr_data_desc = &(ptr_ds_priv->ptr_ibds_data_desc[i + offset]);

		/* set valid bit 0 to be zero */
		ptr_data_desc->dw0 &= 0xFFFFFFFE;

		/* free the data buffer */
		if ((void *)ptr_data_desc->virt_data_buf != NULL) {
			kfree((void *)ptr_data_desc->virt_data_buf);
			ptr_data_desc->virt_data_buf = 0;
		}
	}

	axxia_api_unlock();

	return 0;
}
EXPORT_SYMBOL(axxia_close_ib_data_stream);

/*****************************************************************************
 * axxio_virt_vsid_convert -
 *
 *  This function converts the VISD {16'bSourceID, 8'b cos} to the internal
 *      virtual VSID.  Please refer to Table 133 of rio_axi_datasheet.pdf
 *      for detail information.
 *
 * @vsid:	  incoming VSID
 * @alias_reg:    incoming VSID to internal VSID mapping configuration
 *
 * @ptr_virt_vsid:pointer to where the internal VSID is stored
 *
 * Returns %0 on success
 ****************************************************************************/
int axxio_virt_vsid_convert(
	u32     vsid,
	u32     alias_reg,
	u16     *ptr_virt_vsid)
{
	u32    virt_vsid = 0;
	u32    bit_field = 0;
	u32    vsid_select;
	u32    temp_vsid;

	/* get AVSID[0] select from bit0 to bit 15 */
	vsid_select = alias_reg & 0xF;
	bit_field = ((vsid & (1<<vsid_select)) >> vsid_select) & 0x1;
	virt_vsid = bit_field;

	/* get AVSID[1] select from bit0 to bit 15 */
	vsid_select = (alias_reg & 0xF0) >> 4;
	bit_field = ((vsid & (1<<vsid_select)) >> vsid_select) & 0x1;
	virt_vsid |= (bit_field << 1);

	/* get AVSID[2] select from bit0 to bit 15 */
	vsid_select = (alias_reg & 0xF00) >> 8;
	bit_field = ((vsid & (1<<vsid_select)) >> vsid_select) & 0x1;
	virt_vsid |= (bit_field << 2);

	temp_vsid = (vsid >> 8) & 0xFFFF;

	/* get AVSID[3] select from bit8 to bit 23 */
	vsid_select = (alias_reg & 0xF000) >> 12;
	bit_field = ((temp_vsid & (1<<vsid_select)) >> vsid_select) & 0x1;
	virt_vsid |= (bit_field << 3);

	/* get AVSID[4] select from bit8 to bit 23 */
	vsid_select = (alias_reg & 0xF0000) >> 16;
	bit_field = ((temp_vsid & (1<<vsid_select)) >> vsid_select) & 0x1;
	virt_vsid |= (bit_field << 4);

	/* get AVSID[4] select from bit8 to bit 23 */
	vsid_select = (alias_reg & 0xF00000) >> 20;
	bit_field = ((temp_vsid & (1<<vsid_select)) >> vsid_select) & 0x1;
	virt_vsid |= (bit_field << 5);

	*ptr_virt_vsid = virt_vsid;

	return 0;
}

/*****************************************************************************
 * asr_init_ds - initialize data streaming variables and allocate buffers
 *
 * @priv: private data of the master port
 *
 * Returns %0 on success
 ****************************************************************************/
int axxia_mem_alloc_ds(
	struct rio_ds_priv  *ptr_ds_priv)
{
	void		    *ptr;
	u32		    temp;

	/* allocate IBDS data descriptor - needs 8 words alignment */
	ptr_ds_priv->ptr_ibds_data_desc = NULL;
	ptr = kzalloc((ptr_ds_priv->num_ibds_data_desc *
			sizeof(struct rio_ids_data_desc) +
			RIO_DS_DESC_ALIGNMENT),
			GFP_KERNEL);
	if (ptr == NULL) {
		return -ENOMEM;
	} else {
		temp = (u32)(ptr) % (RIO_DS_DESC_ALIGNMENT);
		if (temp)
			ptr = (char *)ptr + (RIO_DS_DESC_ALIGNMENT - temp);

		/* check if the ptr is 8 word alignment */
		if (((unsigned int)ptr & 0x1F) != 0)
			return -ENOMEM;
		ptr_ds_priv->ptr_ibds_data_desc = ptr;
	}

	/* allocate OBDS header descriptor */
	ptr_ds_priv->ptr_obds_hdr_desc = NULL;
	ptr = kzalloc((ptr_ds_priv->num_obds_hdr_desc *
			sizeof(struct rio_ds_hdr_desc) +
			RIO_DS_DESC_ALIGNMENT),
			GFP_KERNEL);

	if (ptr == NULL) {
		axxia_ds_free(ptr_ds_priv);
		return -ENOMEM;
	} else {
		temp = (u32)(ptr) % (RIO_DS_DESC_ALIGNMENT);
		if (temp)
			ptr = (char *)ptr + (RIO_DS_DESC_ALIGNMENT - temp);

		/* check if the ptr is 8 word alignment */
		if (((unsigned int)ptr & 0x1F) != 0)
			return -ENOMEM;
		ptr_ds_priv->ptr_obds_hdr_desc = ptr;
	}

	/* allocate OBDS data descriptor */
	ptr_ds_priv->ptr_obds_data_desc = NULL;
	ptr = kzalloc((ptr_ds_priv->num_obds_hdr_desc *
			sizeof(struct rio_ods_data_desc) +
			RIO_DS_DESC_ALIGNMENT),
			GFP_KERNEL);
	if (ptr == NULL) {
		axxia_ds_free(ptr_ds_priv);
		return -ENOMEM;
	} else {
		temp = (u32)(ptr) % (RIO_DS_DESC_ALIGNMENT);
		if (temp)
			ptr = (char *)ptr + (RIO_DS_DESC_ALIGNMENT - temp);

		/* check if the ptr is 8 word alignment */
		if (((unsigned int)ptr & 0x1F) != 0)
			return -ENOMEM;
		ptr_ds_priv->ptr_obds_data_desc = ptr;
	}

	return 0;
}

/*****************************************************************************
 * asr_ds_free - free buffers allocated for the data streaming feature
 *
 * @priv: private data of the master port
 *
 * Returns %0 on success
 ****************************************************************************/
int
axxia_ds_free(
	struct rio_ds_priv      *ptr_ds_priv)
{
	/* IBDS data descriptors */
	if (ptr_ds_priv->ptr_ibds_data_desc != NULL)
		kfree(ptr_ds_priv->ptr_ibds_data_desc);

	/* OBDS header descriptors */
	if (ptr_ds_priv->ptr_obds_hdr_desc != NULL)
		kfree(ptr_ds_priv->ptr_obds_hdr_desc);

	/* OBDS data descriptors */
	if (ptr_ds_priv->ptr_obds_data_desc != NULL)
		kfree(ptr_ds_priv->ptr_obds_data_desc);

	return 0;
}

/*****************************************************************************
 * axxia_cfg_ds - configure OBDS variables
 *
 * @mport: the master port
 * @ptr_ds_dtb_info: pointer to where data streaming dtb info is stored
 *
 * Returns %0 on success
 ****************************************************************************/
int axxia_cfg_ds(
	struct rio_mport	*mport,
	struct rio_ds_dtb_info  *ptr_ds_dtb_info)
{
	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv      *ptr_ds_priv = &(priv->ds_priv_data);
	struct rio_obds_dse_cfg *ptr_dse_cfg;
	struct ibds_virt_m_cfg  *ptr_virt_m_cfg;
	u32	 i;

	ptr_ds_priv->num_ibds_data_desc = ptr_ds_dtb_info->inb_num_data_descs;
	ptr_ds_priv->num_ibds_virtual_m = ptr_ds_dtb_info->num_inb_virtaul_m;

	ptr_ds_priv->num_obds_dses = ptr_ds_dtb_info->num_outb_dses;
	ptr_ds_priv->num_obds_hdr_desc = ptr_ds_dtb_info->outb_num_hdr_descs;
	ptr_ds_priv->num_obds_data_desc = ptr_ds_dtb_info->outb_num_data_descs;

	/* set some default values - can be configured later through
	**                           user's API */
	ptr_ds_priv->max_pdu_len = RIO_DS_DATA_BUF_64K;

	/* allocate memory */
	axxia_mem_alloc_ds(ptr_ds_priv);

	/* initialize each DSEs */
	for (i = 0; i < ptr_ds_priv->num_obds_dses; i++) {
		ptr_dse_cfg = &(ptr_ds_priv->obds_dse_cfg[i]);

		ptr_dse_cfg->in_use = RIO_DS_FALSE;

		/* initialize header descriptor variables */
		ptr_dse_cfg->max_num_hdr_desc =
			(ptr_ds_priv->num_obds_hdr_desc /
				ptr_ds_priv->num_obds_dses);

		ptr_dse_cfg->num_hdr_desc_free =
			ptr_dse_cfg->max_num_hdr_desc;
		ptr_dse_cfg->hdr_desc_offset =
			i * (ptr_dse_cfg->max_num_hdr_desc);
		ptr_dse_cfg->hdr_head = 0;
		ptr_dse_cfg->hdr_tail = 0;

		/* initialize data descriptor variables */
		ptr_dse_cfg->max_num_data_desc =
			(ptr_ds_priv->num_obds_data_desc /
				ptr_ds_priv->num_obds_dses);

		ptr_dse_cfg->num_data_desc_free =
			ptr_dse_cfg->max_num_data_desc;

		ptr_dse_cfg->data_desc_offset =
			(i * ptr_dse_cfg->num_data_desc_free);
		ptr_dse_cfg->data_head = 0;
		ptr_dse_cfg->data_tail = 0;
	}

	/* initialized each virtual VSID M */
	for (i = 0; i < ptr_ds_priv->num_ibds_virtual_m; i++) {
		ptr_virt_m_cfg = &(ptr_ds_priv->ibds_vsid_m_cfg[i]);

		ptr_virt_m_cfg->in_use = RIO_DS_FALSE;

		ptr_virt_m_cfg->num_desc_free =
			(ptr_ds_priv->num_ibds_data_desc /
				ptr_ds_priv->num_ibds_virtual_m);

		ptr_virt_m_cfg->max_num_data_desc =
			ptr_virt_m_cfg->num_desc_free;

		ptr_virt_m_cfg->data_desc_offset =
			(i * ptr_virt_m_cfg->num_desc_free);

		ptr_virt_m_cfg->data_head = 0;
		ptr_virt_m_cfg->data_tail = 0;
	}

	/* initialize the SW related header/data descriptor variables */
	memset((void *)ptr_ds_priv->ptr_ibds_data_desc,
		0,
		(sizeof(struct rio_ids_data_desc) *
			(ptr_ds_priv->num_ibds_data_desc)));

	memset((void *)ptr_ds_priv->ptr_obds_hdr_desc,
		0,
		(sizeof(struct rio_ds_hdr_desc) *
			(ptr_ds_priv->num_obds_hdr_desc)));

	memset((void *)ptr_ds_priv->ptr_obds_data_desc,
		0,
		(sizeof(struct rio_ods_data_desc) *
			(ptr_ds_priv->num_obds_data_desc)));

	return 0;
}

/*****************************************************************************
 * release_ob_ds - TBD
 *
 *  This is currently a stub function to be called in axxia_rio_port_irq_init().
 *
 *  This function counts number of enties in a linked list
 *
 * @h:
 *
 * Returns %0 on success
 ****************************************************************************/
void release_ob_ds(struct rio_irq_handler *h)
{
	return;
}

/*****************************************************************************
 * release_ib_ds - TBD
 *
 *  This is currently a stub function to be called in axxia_rio_port_irq_init().
 *
 *  This function counts number of enties in a linked list
 *
 * @h:
 *
 * Returns %0 on success
 ****************************************************************************/
void release_ib_ds(struct rio_irq_handler *h)
{
	return;
}
