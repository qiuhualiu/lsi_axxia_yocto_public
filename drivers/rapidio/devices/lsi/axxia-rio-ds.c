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

/* #define HW_55XX 1 */

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

/**
 * DS_CNTLZW - Count leading zeros word
 * @val: value from which count number of leading zeros
 *
 * Return: number of zeros
 */
static inline u32 DS_CNTLZW(u32 val)
{
	int n = 0;
	if (val == 0)
		return 32;
	if ((val & 0xFFFF0000) == 0)
		n += 16; val = val << 16;
		/* 1111 1111 1111 1111 0000 0000 0000 0000
		// 16 bits from left are zero! so we omit 16 left bits */
	if ((val & 0xFF000000) == 0)
		n = n + 8; val = val << 8;
		/* 8 left bits are 0 */
	if ((val & 0xF0000000) == 0)
		n = n + 4; val = val << 4;
		/* 4 left bits are 0 */
	if ((val & 0xC0000000) == 0)
		n = n + 2, val = val << 2;
		/* 110000....0 2 left bits are zero */
	if ((val & 0x80000000) == 0)
		n = n + 1, val = val << 1;
		/* first left bit is zero */
	return n;
}


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
 * axxia_data_stream_global_cfg -
 *
 *  This function sets global configuration of data streaming related
 *	registers.
 *
 * @mport:	pointer to the master port
 * @max_pdu_length:   Maximum PDU in size supported by the destination
 *			endpoint. It is a 16-bit field, 0 indicates 64KB.
 * @seg_support:      Number of segmentation context supported by
 *			the destination endpoint. It is a 16-bit field.
 * @mtu:	Maximum Transmission Unit - controls the data payload size for
 *			segments of an encapsulated PDU.
 * @ibds_avsid_mapping:	this field definesmapping from incoming VSID
 *			(Combination of Source ID and COS) to internal
 *			(aliased) VSID.
 *
 * Returns %0 on success
 ****************************************************************************/
int axxia_data_stream_global_cfg(
	struct rio_mport    *mport,
	int			max_pdu_length,
	int			seg_support,
	int			mtu,
	int			ibds_avsid_mapping)
{
	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv  *ptr_ds_priv = &(priv->ds_priv_data);
	struct ibds_virt_m_cfg  *ptr_virt_m_cfg;
	struct rio_obds_dse_cfg *ptr_dse_cfg;
	int		reg_val;
	u16		max_pdu_len, mtu_value;
	int	i;

	/* sanity check */
	if ((max_pdu_length > (1<<16))	||
		(mtu < 32)					||
		(mtu > 256)) {
		return -EINVAL;
	}

	/* The global configuration can only be done if there is no OBDSE or
	**	IBDS ALIAS M is used.
	*/
	for (i = 0; i < RIO_MAX_NUM_IBDS_VSID_M; i++) {
		ptr_virt_m_cfg = &(ptr_ds_priv->ibds_vsid_m_cfg[i]);
		if (ptr_virt_m_cfg->in_use == RIO_DS_TRUE)
			return -EINVAL;
	}

	for (i = 0; i < RIO_MAX_NUM_OBDS_DSE; i++) {
		ptr_dse_cfg = &(ptr_ds_priv->obds_dse_cfg[i]);
		if (ptr_dse_cfg->in_use == RIO_DS_TRUE)
			return -EINVAL;
	}

	/* Data Streaming Information Capability Register */
	/* max_pdu_len is a 16-bit field in HW, thus 0 indicates 64KB */
	reg_val = 0;
	reg_val |= ((seg_support << 16) & 0xFFFF0000);
	if (max_pdu_length == (1<<16))
		max_pdu_len = 0;
	else
		max_pdu_len =  max_pdu_length;

	reg_val |= (max_pdu_len & 0xFFFF);
	__rio_local_write_config_32(mport,
							GRIO_DSI_CAR,
							reg_val);

	/*
	** Data Streaming Logical Layer Control Command and Status Register
	**	MTU bits [31:24] 8 - 32 bytes, 9 - 36 bytes etc.
	*/
	reg_val = 0;
	mtu_value = mtu / 4;
	reg_val |= ((mtu_value << 24) & 0xFF000000);
	__rio_local_write_config_32(mport,
						GRIO_DSLL_CCSR,
						reg_val);

	/* IBDS alias mapping register */
	reg_val = 0;
	reg_val |= (ibds_avsid_mapping & 0xFFFFFFF);
	__rio_local_write_config_32(mport,
						RAB_IBDS_VSID_ALIAS,
						reg_val);

	/* save information in the system */
	ptr_ds_priv->max_pdu_len = max_pdu_len;
	ptr_ds_priv->seg_support = seg_support;
	ptr_ds_priv->mtu = mtu;
	ptr_ds_priv->ibds_avsid_mapping = ibds_avsid_mapping;


	return 0;
}
EXPORT_SYMBOL(axxia_data_stream_global_cfg);

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
	struct rio_mport    *mport,
	void		*dev_id,
	int			dse_id,
	int			num_header_entries,
	int			num_data_entries)
{
	int		 rc = 0;

	axxia_api_lock();

	rc = open_ob_data_stream(mport,
				dev_id,
				dse_id,
				num_header_entries,
				num_data_entries);

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
	int			dse_id,
	int			num_header_entries,
	int			num_data_entries)
{

	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv  *ptr_ds_priv = &(priv->ds_priv_data);
	struct rio_obds_dse_cfg *ptr_dse_cfg;
	u32		    temp;
	void		*ptr;
	struct rio_irq_handler *h;
	u32 des_chain_start_addr_phy_low, des_chain_start_addr_phy_hi;
	u32 dse_ctrl;
	unsigned long dse_chain_start_addr_phy;

	int rc = 0;


	/* Check if the dse_id is in use */
	ptr_dse_cfg = &(ptr_ds_priv->obds_dse_cfg[dse_id]);

	if (ptr_dse_cfg->in_use)
		return -EINVAL;

	/* allocate header descriptor buffers */
	if (!num_header_entries) {
		return -EINVAL;
	} else {
		ptr = kzalloc((num_header_entries *
					  sizeof(struct rio_ds_hdr_desc) +
					  RIO_DS_DESC_ALIGNMENT),
					  GFP_KERNEL);
		if (!ptr) {
			return -ENOMEM;
		} else {
			temp = (u32)(ptr) % (RIO_DS_DESC_ALIGNMENT);
			if (temp)
				ptr = (char *)ptr +
					(RIO_DS_DESC_ALIGNMENT -
					temp);

			/* check if the ptr is 8 word alignment */
			if (((unsigned int)ptr & 0x1F) != 0) {
				/* free the buffer */
				kfree(ptr);

				return -ENOMEM;
			}

			ptr_dse_cfg->ptr_obds_hdr_desc = ptr;
		}
	}

	/* allocate data descriptors */
	if (num_data_entries) {
		ptr = kzalloc((num_data_entries *
					   sizeof(struct rio_ods_data_desc) +
					   RIO_DS_DESC_ALIGNMENT),
					   GFP_KERNEL);

		if (!ptr)
			return -ENOMEM;
		else{
			temp = (u32)(ptr) % (RIO_DS_DESC_ALIGNMENT);
			if (temp)
				ptr = (char *)ptr +
					(RIO_DS_DESC_ALIGNMENT -
					temp);

			/* check if the ptr is 8 word alignment */
			if (((unsigned int)ptr & 0x1F) != 0) {
				/* free the buffer */
				kfree(ptr);
				return -ENOMEM;
			}

			ptr_dse_cfg->ptr_obds_data_desc = ptr;
		}
	}

	ptr_dse_cfg->in_use = RIO_DS_TRUE;

	/* get the chain start address */
	dse_chain_start_addr_phy =
		virt_to_phys((void *)(ptr_dse_cfg->ptr_obds_hdr_desc));

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

	h = &(ptr_ds_priv->ob_dse_irq[dse_id]);

	sprintf(ptr_dse_cfg->name, "obds-%d", dse_id);
	rc = alloc_irq_handler(h, (void *)ptr_dse_cfg, ptr_dse_cfg->name);

	if (rc == 0) {
		ptr_dse_cfg->max_num_hdr_desc = num_header_entries;
		ptr_dse_cfg->num_hdr_desc_free = num_header_entries;

		ptr_dse_cfg->max_num_data_desc = num_data_entries;
		ptr_dse_cfg->num_data_desc_free = num_data_entries;


		/* enable the DSE */
		/* start, wake up the engine */
		__rio_local_read_config_32(mport,
						RAB_OBDSE_CTRL(dse_id),
						&dse_ctrl);
		dse_ctrl |= DSE_ENABLE;
		__rio_local_write_config_32(mport,
						RAB_OBDSE_CTRL(dse_id),
						dse_ctrl);

	}


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
	int				priority,
	int				is_hdr_desc,
	void		    	*buffer,
	int		     	data_len)
{
	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv      *ptr_ds_priv;
	struct rio_obds_dse_cfg *ptr_dse_cfg;
	struct rio_ds_hdr_desc  *ptr_hdr_desc;
	u16		hdr_write_ptr, next_desc_index;
	u16     dse_id;
	u32     dse_ctrl, dse_stat;
	u32		next_desc_high, data_buf_high;
	unsigned long	next_desc_ptr_phy, data_buf_phy;
	int rc = 0;

	/* sanity check - TBD */

	ptr_ds_priv = &(priv->ds_priv_data);

	/*
	** There are maximum of 16 DSEs, each DSE can handle one
	**	descriptor chain, however
	**	different stream_id can be chained in the same
	**	descriptor chain.
	*/
	dse_id = (stream_id % (ptr_ds_priv->num_obds_dses));
	ptr_dse_cfg = &(ptr_ds_priv->obds_dse_cfg[dse_id]);


	/* if the DSE has not been configured, return an error */
	if (ptr_dse_cfg->in_use == RIO_DS_FALSE)
		return -EINVAL;

	/* check if there is a space for the new data */
	if (is_hdr_desc) {

		if (ptr_dse_cfg->num_hdr_desc_free == 0)
			return -ENOMEM;

		/* get the header descriptor */
		hdr_write_ptr = ptr_dse_cfg->hdr_write_ptr;
		ptr_hdr_desc = &(ptr_dse_cfg->ptr_obds_hdr_desc[hdr_write_ptr]);

		/*
		** program the header descriptor word0
		** The int_enable and valid bits are programmed when data
		** is ready to be sent.
		*/
		ptr_hdr_desc->dw0 = 0;
		/* dest_id - [31:16]  */
		ptr_hdr_desc->dw0 |= ((dest_id << 16) & 0xFFFF0000);
		/* descriptor type - bit 2 (1 - header descriptor) */
		ptr_hdr_desc->dw0 |= ((1 << 2) & 0x4);
		/* single_desc - bit 3 (1 - single descriptor) */
		ptr_hdr_desc->dw0 |= ((1 << 3) & 0x8);

		#if 0
		/* next_desc_ptr_valid - bit 1 */
		if (hdr_index != (num_hdr_desc_needed-1))
			h_dw0 |= 2;
		#endif

		/* program the header descriptor word1 */
		ptr_hdr_desc->dw1 = 0;
		/* stream_id - bits [0:16] */
		ptr_hdr_desc->dw1 |= (stream_id & 0xFFFF);
		/*
		** pdu_len - [17:31] 000 - 64KB
		** if it is 64KB,the dw1 field is programmed as 0
		*/
		if (data_len != RIO_DS_DATA_BUF_64K)
			ptr_hdr_desc->dw1 |= ((data_len << 16) & 0xFFFF0000);

		/* program the header descritpor word2 */
		ptr_hdr_desc->dw2 = 0;
		/* cos - bits [2:9] */
		ptr_hdr_desc->dw2 |= ((cos << 2) & 0x3FC);
		/* priority - bits [1:0] */
		ptr_hdr_desc->dw2 |= (priority & 0x3);

		if (ptr_dse_cfg->hdr_write_ptr ==
			(ptr_dse_cfg->max_num_hdr_desc - 1)) {
			next_desc_index = 0;
		} else {
			next_desc_index = ptr_dse_cfg->hdr_write_ptr + 1;
		}

		next_desc_ptr_phy = virt_to_phys((void *)
		(&(ptr_dse_cfg->ptr_obds_hdr_desc[next_desc_index])));

		/*
		** The next_desc_addr - 38-bit AXI addressing
		**  next_desc_addr[37] - h_dw2 [28]
		**  next_desc_addr[36:5] - h_dw3[0:31] - next_desc_address has
		**                                       to be 8 words aligned
		*/
		ptr_hdr_desc->dw3 =
			(u32)((next_desc_ptr_phy >> 5) & 0xFFFFFFFF);
		next_desc_high = ((u64)next_desc_ptr_phy >> 37) & 0x1;
		ptr_hdr_desc->dw2 |= ((next_desc_high << 28) & 0x10000000);

		/* program the data buffer in the descriptor */
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

		ptr_dse_cfg->hdr_write_ptr++;

		ptr_dse_cfg->num_hdr_desc_free--;
	} else {
		/* header and data descriptor combination support TBD */
		return -EINVAL;
	}

	/* check if the DSE is in sleep mode, if it is, wake up */
	/* find out DSE stats */
	__rio_local_read_config_32(mport, RAB_OBDSE_STAT(dse_id), &dse_stat);

	if (dse_stat & OB_DSE_STAT_SLEEPING) {
		/* start, wake up the engine */
		__rio_local_read_config_32(mport,
						RAB_OBDSE_CTRL(dse_id),
						&dse_ctrl);
		dse_ctrl |= DSE_WAKEUP;
		__rio_local_write_config_32(mport,
						RAB_OBDSE_CTRL(dse_id),
						dse_ctrl);
	}

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
	u16 hdr_read_ptr;
	unsigned long flags;

	/* find the DSE that gets interrupted, DS_CNTLZW found the upper
	** bit first */
	dse_id = 31 - DS_CNTLZW(state);

	/* find out DSE stats */
	__rio_local_read_config_32(mport, RAB_OBDSE_STAT(dse_id), &dse_stat);


	ptr_ds_priv = &(priv->ds_priv_data);

	/**
	 * Wait for all pending transactions to finish before doing descriptor
	 * updates
	 */
	ptr_dse_cfg = &(ptr_ds_priv->obds_dse_cfg[dse_id]);
	spin_lock_irqsave(&ptr_dse_cfg->lock, flags);

	/* disable interrupt? TBD */

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
		if (ptr_dse_cfg->hdr_read_ptr == ptr_dse_cfg->hdr_write_ptr) {
			spin_unlock_irqrestore(&ptr_dse_cfg->lock, flags);
			return;
		}

		while (ptr_dse_cfg->hdr_read_ptr !=
			ptr_dse_cfg->hdr_write_ptr) {

			hdr_read_ptr = ptr_dse_cfg->hdr_write_ptr;
			ptr_hdr_desc =
			&(ptr_dse_cfg->ptr_obds_hdr_desc[hdr_read_ptr]);

			__ob_dse_dw_dbg(&(ptr_ds_priv->ob_dse_stats[dse_id]),
				ptr_hdr_desc->dw0);

			/*
			** check if the descriptor is done - bit 8
			**
			**  Since the IRQ process the descriptor one-by-one,
			**  if the descriptor pointed by tail is not processed,
			**  the following ones are not processed either.
			**
			**	Only header descriptors are supported now.
			**	The combination of header descriptor and data
			**		descriptor are TBD.
			*/
			if ((ptr_hdr_desc->dw0 & OB_HDR_DESC_DONE) == 0)
				break;

			/* set the valid bit to be zero */
			ptr_hdr_desc->dw0 &= 0xFFFFFFFE;

			/* free the buffer */
			kfree((void *)ptr_hdr_desc->virt_data_buf);

			ptr_dse_cfg->data_read_ptr++;

			if (ptr_dse_cfg->data_read_ptr ==
				(ptr_dse_cfg->max_num_hdr_desc - 1)) {
				ptr_dse_cfg->data_read_ptr = 0;
			}

			/* the descriptor has been processed, but there is
			** error occurred */
			if ((ptr_hdr_desc->dw0 & OB_HDR_DESC_AXI_ERR)) {
				__ob_dse_dw_dbg(
					&(ptr_ds_priv->ob_dse_stats[dse_id]),
					ptr_hdr_desc->dw0);
				/* callback function - TBD */
			}

			ptr_dse_cfg->num_hdr_desc_free++;
		}
	}

	/* clear the interrupt bit? - TBD */

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
	int		 	dse_id)
{
	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv      *ptr_ds_priv = &(priv->ds_priv_data);
	struct rio_obds_dse_cfg *ptr_dse_cfg;
	u32    dse_ctrl;

	axxia_api_lock();

	ptr_dse_cfg = &(ptr_ds_priv->obds_dse_cfg[dse_id]);

	if (ptr_dse_cfg->in_use == RIO_DS_FALSE) {
		axxia_api_unlock();
		return 0;
	}

	/* reset variables */
	ptr_dse_cfg->in_use = RIO_DS_FALSE;
	ptr_dse_cfg->data_read_ptr = 0;
	ptr_dse_cfg->data_write_ptr = 0;
	ptr_dse_cfg->hdr_read_ptr = 0;
	ptr_dse_cfg->data_write_ptr = 0;
	ptr_dse_cfg->num_hdr_desc_free = 0;
	ptr_dse_cfg->num_data_desc_free = 0;

	/* free header and data descriptor */
	if (ptr_dse_cfg->ptr_obds_hdr_desc != NULL)
		kfree(ptr_dse_cfg->ptr_obds_hdr_desc);

	if (ptr_dse_cfg->ptr_obds_data_desc != NULL)
		kfree(ptr_dse_cfg->ptr_obds_data_desc);

	/* Disable the corresponding DSE */
	__rio_local_read_config_32(mport, RAB_OBDSE_CTRL(dse_id), &dse_ctrl);
	dse_ctrl &= 0xFFFFFFFE;
	__rio_local_write_config_32(mport, RAB_OBDSE_CTRL(dse_id), dse_ctrl);

	axxia_api_unlock();

	return 0;
}
EXPORT_SYMBOL(axxia_close_ob_data_stream);

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
 * @desc_dbuf_size:   PDU size this descriptor can handle.
 *		If the PDU size exceeds
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
	int			desc_dbuf_size,
	int			num_entries)
{
	int rc = 0;

	axxia_api_lock();

	rc = open_ib_data_stream(mport,
							 dev_id,
							 source_id,
							 cos,
							 desc_dbuf_size,
							 num_entries);
	axxia_api_unlock();

	return rc;
}
EXPORT_SYMBOL(axxia_open_ib_data_stream);

/*****************************************************************************
 * open_ib_data_stream -
 *
 *  This function sets up the descriptor chain to an internal VSID M.
 *  The internal VSID is calculated through sourceId, cos and
 *  RAB_IBDS_VSID_ALIAS register.
 *
 * @mport:	pointer to the master port
 * @dev_id: Device specific pointer to pass on event
 * @source_id:   source ID of the data stream
 * @cos:	class of service of the stream
 * @desc_dbuf_size:   PDU size this descriptor can handle.
 *		If the PDU size exceeds
 *		descriptor size (but less than maximum PDU size), appropriate
 *		error bit is set and interrupt is generated if enabled.
 *		PDU data beyond descriptor size is not written in the AXI
 *		memory to ensure that there is no corruption of data under
 *		this error case.
 * @num_entries: number of entries in this descriptor chain
 *
 * Returns %0 on success
 ****************************************************************************/
int open_ib_data_stream(
	struct rio_mport    	*mport,
	void			*dev_id,
	int			source_id,
	int 		    	cos,
	int			desc_dbuf_size,
	int			num_entries)
{
	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv  *ptr_ds_priv = &(priv->ds_priv_data);
	struct ibds_virt_m_cfg  *ptr_virt_m_cfg;
	struct rio_ids_data_desc *ptr_data_desc;
	struct rio_irq_handler *h;
	void	*ptr;
	u32		temp;
	u32     alias_reg;
	u32     vsid, i, next_desc_offset;
	u16     virt_vsid;
	unsigned long     	desc_chain_start_addr_phy, next_desc_addr_phy;

	u32     next_desc_addr_hi, vsid_addr_reg;

	int rc = 0;

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

	if (ptr_virt_m_cfg->in_use == RIO_DS_TRUE)
		return -EINVAL;

	/* allocate data descriptor buffers */
	ptr = kzalloc(num_entries * sizeof(struct rio_ids_data_desc) +
					sizeof(struct rio_ids_data_desc) +
					RIO_DS_DESC_ALIGNMENT,
					GFP_KERNEL);
	if (ptr == NULL) {
		return -ENOMEM;
	} else {
		temp = (u32)(ptr) % (RIO_DS_DESC_ALIGNMENT);
		if (temp)
			ptr = (char *)ptr + (RIO_DS_DESC_ALIGNMENT - temp);

		/* check if the ptr is 8 word alignment */
		if (((unsigned int)ptr & 0x1F) != 0) {
			kfree(ptr);
			return -ENOMEM;
		}

		ptr_virt_m_cfg->ptr_ibds_data_desc =
			(struct rio_ids_data_desc *)ptr;
	}

	ptr_virt_m_cfg->in_use = RIO_DS_TRUE;

	ptr_virt_m_cfg->num_desc_free = num_entries;

	/* chain the data descriptors */
	for (i = 0; i < num_entries; i++) {
		ptr_data_desc = &(ptr_virt_m_cfg->ptr_ibds_data_desc[i]);

		/* init the data descriptor */
		memset((void *)ptr_data_desc,
			0,
			sizeof(struct rio_ids_data_desc));

		/* dw0 - desc_size, bits [4:6]
		**	the desc_size is not actual size, it is numbered value
		**	0 - 64KB, 1 - 1K, 2 - 2K, 3 - 4k, 4 - 8K, 5 - 16K
		**	6 - 32K, 7 - 64K
		*/
		ptr_data_desc->dw0 |= ((1 << 4) & 0x70);
		/* dw0 - source_id, bits [16:31] */
		ptr_data_desc->dw0 |= ((source_id << 16) & 0xFFFF0000);

		ptr_data_desc->dw0 |= 0x2;

		if (i == (num_entries-1))
			next_desc_offset = 0;
		else
			next_desc_offset = i + 1;

		/* cos - bit[16:23] */
		ptr_data_desc->dw2 |= ((cos << 16) & 0xFF0000);
		/*
		** next_desc_addr - 38-bit AXI addressing
		**  next_desc_addr[37] - dw2[24]
		**  next_desc_addr[36:5] - dw4[31:0]
		*/
		next_desc_addr_phy =
		virt_to_phys((void *)
			&ptr_virt_m_cfg->ptr_ibds_data_desc[next_desc_offset]);
		next_desc_addr_hi = ((u64)next_desc_addr_phy >> 37) & 0x1;

		ptr_data_desc->dw4 =
		((u64)next_desc_addr_phy << 5) & 0xFFFFFFFF;

		ptr_data_desc->dw2 |= (next_desc_addr_hi << 24) & 0x1000000;
	}

	ptr_virt_m_cfg->cos = cos;
	ptr_virt_m_cfg->source_id = source_id;
	ptr_virt_m_cfg->desc_dbuf_size = desc_dbuf_size;

	desc_chain_start_addr_phy =
	virt_to_phys((void *)
		&(ptr_virt_m_cfg->ptr_ibds_data_desc[0]));

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


	/* register IRQ */
	h = &(ptr_ds_priv->ib_dse_vsid_irq[virt_vsid]);

	sprintf(ptr_virt_m_cfg->name, "ibds-%d", virt_vsid);

	rc = alloc_irq_handler(h, (void *)ptr_virt_m_cfg, ptr_virt_m_cfg->name);

	return rc;
}


/*****************************************************************************
 * axxia_add_ibds_buffer -
 *
 *  This function adds buffer to the AXXIA inbound data stream queue.
 *
 * @mport:	pointer to the master port
 * @source_id:	source ID of the data stream
 * @cos:	class of service of the stream
 * @buf:	pointer to where the data is copied to
 * @buf_size: buffer size of the added buffer
 *
 * Returns %0 on success
 ****************************************************************************/
int axxia_add_ibds_buffer(
	struct rio_mport   *mport,
	int		   source_id,
	int		   cos,
	void		  *buf,
	int			buf_size)
{
	struct rio_priv *priv = mport->priv;
	struct rio_ds_priv      *ptr_ds_priv = &(priv->ds_priv_data);
	struct ibds_virt_m_cfg  *ptr_virt_m_cfg;
	struct rio_ids_data_desc *ptr_data_desc;
	u32		     	m_id;
	u8		      	found_one = RIO_DS_FALSE;

    unsigned long   data_addr_phy;
    u32 data_addr_hi;

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

	/* check if the buf_size is smaller than the desc_dbuf_size */
	if (buf_size != ptr_virt_m_cfg->desc_dbuf_size)
		return -EINVAL;

	/*
	** check if there are descriptors left
	**
	**	Since the driver uses circular ring and linked the data
	**	descriptor during open_data_stream( ) function.
	**   If end_of_chain bit is not used, SW needs to always keep
	**		one data descriptor invalid, so that the HW will
	**		not overwrite the data buffer when SW can not
	**		keep up with. When SW can not keep up with,
	**		the HW will drop the newly received data.
	*/
	if (ptr_virt_m_cfg->num_desc_free == 1)
		return -ENOMEM;

	/* put user's buffer into the corresponding descriptors */
	ptr_data_desc =
	&(ptr_virt_m_cfg->ptr_ibds_data_desc[ptr_virt_m_cfg->buf_add_ptr]);

	ptr_data_desc->virt_data_buf = (u32)buf;

    data_addr_phy =
		virt_to_phys((void *)ptr_data_desc->virt_data_buf);

    ptr_data_desc->dw3 = ((u64)data_addr_phy & 0xFFFFFFFF);
    data_addr_hi = ((u64)data_addr_phy >> 32) & 0x3F;
    ptr_data_desc->dw2 |= (data_addr_hi << 26) & 0xFC000000;

	/* clear all the status bits that may be set before */
	ptr_data_desc->dw0 &= ~(IB_DSE_DESC_DONE);
	ptr_data_desc->dw0 &= ~(IB_DSE_DESC_AXI_ERR);
	ptr_data_desc->dw0 &= ~(IB_DSE_DESC_DS_ERR);

	/* the buf_add_ptr is determined by number of free descriptors */
    ptr_virt_m_cfg->buf_add_ptr++;

	ptr_virt_m_cfg->num_desc_free--;

	/* set the valid bit to be 1 */
	ptr_data_desc->dw0 |= 0x1;

	return 0;
}
EXPORT_SYMBOL(axxia_add_ibds_buffer);

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
    u16 data_write_ptr, i;
    u8  found_dse = RIO_DS_FALSE;
	unsigned long flags;

    virt_vsid = 31 - DS_CNTLZW(state);

    __rio_local_read_config_32(mport,
				RAB_IBVIRT_M_STAT(virt_vsid),
				&vsid_m_stats);

    /* check if the chain transfer complete */
    ptr_virt_m_cfg = &(ptr_ds_priv->ibds_vsid_m_cfg[virt_vsid]);

    spin_lock_irqsave(&ptr_virt_m_cfg->lock, flags);

	/* disable the interrupt ? - TBD */

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
	** If transaction pending bit is not set an timeout is also not set,
	**	that means that PDU was successfully written into AXI memory
	**      and nothing needs to be done.
	** If transaction pending bit is set or timeout is set, engine needs
	** 	to be reset. After disabling engine, when transaction pending
	**	gets reset, engine is ready to be enabled again.
	*/
	if ((dse_stat & IB_DSE_STAT_TRANS_PENDING)  ||
		(dse_stat & IB_DSE_STAT_TIMEOUT)) {
		/*
		** BZ43821 - SW workaround for the IBDS descriptor fetch error
		** When S/W sees the descriptor fetch error being indicated in
		**  status bits, introduce a delay and then disable the engine
		**  and enable the engine again.
		** With this the next incoming packet for that engine would
		**    not get corrupted.
		*/
		ndelay(5);

		/* disable the engine */
		__rio_local_write_config_32(mport,
					RAB_IBDSE_CTRL(dse_id),
					0);

		/* should wait till the pending bit is reset? - TBD */

		/* enable the engine again */
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

    /* process maximum number of MAX_NUM_PROC_IBDS_DESC transactions */
	data_write_ptr = ptr_virt_m_cfg->data_write_ptr;
	for (i = 0; i < MAX_NUM_PROC_IBDS_DESC; i++) {
		/* check if the done bit is set */
		ptr_data_desc =
		&(ptr_virt_m_cfg->ptr_ibds_data_desc[data_write_ptr]);

		if ((ptr_data_desc->dw0 & IB_DSE_DESC_DONE) != 0) {

			ptr_virt_m_cfg->num_hw_written_bufs++;

			if (data_write_ptr ==
				(ptr_virt_m_cfg->max_num_data_desc-1))
				data_write_ptr = 0;
			else
				data_write_ptr++;

			__ib_dse_dw_dbg(
			&(ptr_ds_priv->ib_vsid_m_stats[virt_vsid]),
			ptr_data_desc->dw0);

			/* set the valid bit to be invalid */
			ptr_data_desc->dw0 &= 0xFFFFFFFE;

		} else {
			break;
		}
	}

	ptr_virt_m_cfg->data_write_ptr = data_write_ptr;

	/* call back - TBD */

	/* clear the interrupt bit? - TBD */

    spin_unlock_irqrestore(&ptr_virt_m_cfg->lock, flags);

    return;
}

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
	u32		    m_id, data_read_ptr;
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
	ptr_virt_m_cfg->num_hw_written_bufs = 2;
#endif

	/* check if the there are buffers that are written - semaphore ?*/
	if (ptr_virt_m_cfg->num_hw_written_bufs < 1)
		return NULL;

	data_read_ptr = ptr_virt_m_cfg->data_read_ptr;

	/* get the data descriptor */
	ptr_data_desc =
		&(ptr_virt_m_cfg->ptr_ibds_data_desc[data_read_ptr]);

	/* check if the source_id and cos matches */
	if ((((ptr_data_desc->dw0 >> 16) & 0xFFFF) != source_id) ||
		((ptr_data_desc->dw2 & 0xFF0000) >> 16) != cos) {
		return NULL;
	}

#ifndef HW_55XX
		*ptr_pdu_length = 64;
		ptr_data = (char *)ptr_data_desc->virt_data_buf;
		for (i = 0; i < (*ptr_pdu_length); i++)
			ptr_data[i] = i;
#endif

	user_buf = (void *)ptr_data_desc->virt_data_buf;

	if (user_buf == NULL) {
		*ptr_pdu_length = 0;
		return NULL;
	} else {
#ifdef HW_55XX
		*ptr_pdu_length = ((ptr_data_desc->dw1 & 0xFFFF0000) >> 16);
		*ptr_stream_id = (ptr_data_desc->dw1) & 0xFFFF;
#endif

		if (ptr_virt_m_cfg->data_read_ptr ==
			(ptr_virt_m_cfg->max_num_data_desc - 1)) {
			ptr_virt_m_cfg->data_read_ptr = 0;
		} else {
			ptr_virt_m_cfg->data_read_ptr++;
		}

		ptr_virt_m_cfg->num_hw_written_bufs--;
		ptr_virt_m_cfg->num_desc_free++;

		return user_buf;
	}
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
	u8		      find_ava_virt_m = RIO_DS_FALSE;
	u8      i;

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
	ptr_virt_m_cfg->data_read_ptr = 0;
	ptr_virt_m_cfg->data_write_ptr = 0;
	ptr_virt_m_cfg->num_desc_free = 0;

	/* free the data descriptor pointer */
	if (ptr_virt_m_cfg->ptr_ibds_data_desc != NULL)
		kfree(ptr_virt_m_cfg->ptr_ibds_data_desc);

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

	ptr_ds_priv->num_ibds_data_desc = ptr_ds_dtb_info->inb_num_data_descs;
	ptr_ds_priv->num_ibds_virtual_m = ptr_ds_dtb_info->num_inb_virtaul_m;

	/* set some default values - can be configured later through
	**                           user's API */
	ptr_ds_priv->max_pdu_len = RIO_DS_DATA_BUF_64K;

	/* HW configuration - TBD */

	/*
	** check if the ASIC supports data streaming feature.
	** This has to be called in the axxia-rio.c after
	**	calling function rio_priv_dtb_setup( )
	**
	**	check RAB_VER register
	*/
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
