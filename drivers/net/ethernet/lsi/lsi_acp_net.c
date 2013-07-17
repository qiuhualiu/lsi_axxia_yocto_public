#ifdef CONFIG_ARM

/*
 * drivers/net/ethernet/lsi/lsi_acp_net.c
 *
 * Copyright (C) 2009 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 *  USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/in.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/proc_fs.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/dma-mapping.h>

#include <linux/uaccess.h>
#include <linux/io.h>
#include <asm/dma.h>

/*#include <asm/acp3400-version.h>*/

extern int acp_mdio_read(unsigned long,
			 unsigned long,
			 unsigned short *);
extern int acp_mdio_write(unsigned long,
			  unsigned long,
			  unsigned short);

/* Base Addresses of the RX, TX, and DMA Registers. */
static void *rx_base;
static void *tx_base;
static void *dma_base;

/*
  =====================================================================
  Debug counts (readable from the external host).
  =====================================================================
*/

#define LSINET_COUNTS

#ifdef LSINET_COUNTS

unsigned long lsinet_counts[] = {
	0, 0, 0, 0,		/* ISR */
	0, 0, 0, 0, 0, 0,	/* HST */
	0, 0, 0, 0, 0, 0, 0,	/* RX */
	0, 0, 0, 0		/* POL */
};
EXPORT_SYMBOL(lsinet_counts);

#define LSINET_COUNTS_INC(index) { ++lsinet_counts[(index)]; }

#else  /* LSINET_DEBUG_COUNTS */

#define LSINET_COUNTS_INC(index)

#endif /* LSINET_DEBUG_COUNTS */

#define LSINET_COUNTS_ISR_START	0
#define LSINET_COUNTS_ISR_TX	1
#define LSINET_COUNTS_ISR_RX	2
#define LSINET_COUNTS_ISR_DONE	3
#define LSINET_COUNTS_HST_START	4
#define LSINET_COUNTS_HST_RCLM	5
#define LSINET_COUNTS_HST_SNDG	6
#define LSINET_COUNTS_HST_SNT	7
#define LSINET_COUNTS_HST_OOD	8
#define LSINET_COUNTS_HST_DONE	9
#define LSINET_COUNTS_RX_START	10
#define LSINET_COUNTS_RX_PKT	11
#define LSINET_COUNTS_RX_ERR	12
#define LSINET_COUNTS_RX_GOOD	13
#define LSINET_COUNTS_RX_SENT	14
#define LSINET_COUNTS_RX_DRPD	15
#define LSINET_COUNTS_RX_DONE	16
#define LSINET_COUNTS_POL_START	17
#define LSINET_COUNTS_POL_PKT	18
#define LSINET_COUNTS_POL_RNBL	19
#define LSINET_COUNTS_POL_DONE	20

/*
  ====================================================================
  ====================================================================
  ====================================================================
  Trace/Debug/Warn/Error Macros
  ====================================================================
  ====================================================================
  ====================================================================
*/

/* -- TRACE -------------------------------------------------------- */

#undef TRACE
/*#define TRACE*/
#define TRACE_PRINTK
#ifdef TRACE
#ifdef TRACE_PRINTK
#define TRACE_BEGINNING() (					\
	printk(KERN_DEBUG "nic:%d:%s:Beginning\n",		\
	       smp_processor_id(), __func__))
#define TRACE_ENDING() (					\
	printk(KERN_DEBUG "nic:%d:%s:Ending\n",			\
	       smp_processor_id(), __func__))
#else
#define TRACE_BEGINNING() TRACER_POST("Beginning");
#define TRACE_ENDING() TRACER_POST("Ending");
#endif
#else
#define TRACE_BEGINNING(format, args...)
#define TRACE_ENDING(format, args...)
#endif

/* -- DEBUG -------------------------------------------------------- */

#undef DEBUG
/*#define DEBUG*/
#if defined(DEBUG)
#define DEBUG_PRINT(format, args...)					\
	do {								\
		printk(KERN_DEBUG "appnic:%d - DEBUG - ", __LINE__);	\
		printk(KERN_DEBUG format, ##args);			\
	} while (0);
#else
#define DEBUG_PRINT(format, args...)
#endif

#undef PHY_DEBUG
/*#define PHY_DEBUG*/
#if defined(PHY_DEBUG)
#define PHY_DEBUG_PRINT(format, args...)				\
	do {								\
		printk(KERN_ERR "net:%d - PHY_DEBUG - ", __LINE__);	\
		printk(KERN_ERR format, ##args);			\
	} while (0);
#else
#define PHY_DEBUG_PRINT(format, args...)
#endif

/* -- WARN ---------------------------------------------------------- */

#undef WARN
#define WARN
#if defined(WARN)
#define WARN_PRINT(format, args...)					\
	do {								\
		printk(KERN_DEBUG "appnic:%d - WARN - ", __LINE__);	\
		printk(KERN_DEBUG format, ##args);			\
	} while (0);
#else
#define WARN_PRINT(format, args...)
#endif

/* -- ERROR --------------------------------------------------------- */

#define ERROR_PRINT(format, args...)					\
	do {								\
		printk(KERN_ERR "%s:%s:%d - ERROR - ",			\
		       __FILE__, __func__, __LINE__);			\
		printk(format, ##args);					\
} while (0);

/*
  ======================================================================
  ======================================================================
  Optimizations
  ======================================================================
  ======================================================================
*/

#define DISABLE_TX_INTERRUPTS
/*#define PRELOAD_RX_BUFFERS*/

#undef DMA_CACHABLE
/*#define DMA_CACHABLE*/

/*
  =============================================================================
  NAPI Support (new and newer)...
  =============================================================================
*/

#define LSINET_NAPI
#define LSINET_NAPI_WEIGHT 64

/*
  ======================================================================
  ======================================================================
  ======================================================================
  Access and Access Logging
  ======================================================================
  ======================================================================
  ======================================================================
*/
/*
#define LOG_MAC_ACCESS
#define LOG_PHY_ACCESS
*/
/*
  ======================================================================
  ======================================================================
  ======================================================================
  PHY
  ======================================================================
  ======================================================================
  ======================================================================
*/

#undef PHYLESS
/*#define PHYLESS*/

#ifndef PHYLESS

/* -- control -- */

#define PHY_CONTROL 0x00
/*#define __LITLE_ENDIAN*/

typedef union {
	unsigned short raw;

	struct {
#if 0
		unsigned short soft_reset:1;
		unsigned short loop_back:1;
		unsigned short force100:1;	/* speedBit0 */
		unsigned short autoneg_enable:1;
		unsigned short power_down:1;
		unsigned short isolate:1;
		unsigned short restart_autoneg:1;
		unsigned short full_duplex:1;	/* duplex */
		unsigned short collision_test:1;
		unsigned short unused:7;
#else  /* __BIG_ENDIAN */
		unsigned short unused:7;
		unsigned short collision_test:1;
		unsigned short full_duplex:1;	/* duplex */
		unsigned short restart_autoneg:1;
		unsigned short isolate:1;
		unsigned short power_down:1;
		unsigned short autoneg_enable:1;
		unsigned short force100:1;	/* speedBit0 */
		unsigned short loop_back:1;
		unsigned short soft_reset:1;
#endif /* __BIG_ENDIAN */
	} __attribute__ ((packed)) bits;
} __attribute__ ((packed)) phy_control_t;

/* -- status -- */

#define PHY_STATUS 0x01

typedef union {
	unsigned short raw;

	struct {
#if 0
		unsigned short t4_capable:1;
		unsigned short tx_fdx_capable:1;
		unsigned short tx_capable:1;
		unsigned short bt_fdx_capable:1;
		unsigned short tenbt_capable:1;
		unsigned short unused:4;
		unsigned short mf_pream_suppress:1;
		unsigned short autoneg_comp:1;	/* autoNegDone */
		unsigned short remote_fault:1;	/* remoutFault */
		unsigned short autoneg_capable:1;
		unsigned short link_status:1;	/* linkStatus */
		unsigned short jabber_detect:1;
		unsigned short extd_reg_capable:1;
#else
		unsigned short extd_reg_capable:1;
		unsigned short jabber_detect:1;
		unsigned short link_status:1;	/* linkStatus */
		unsigned short autoneg_capable:1;
		unsigned short remote_fault:1;	/* remoutFault */
		unsigned short autoneg_comp:1;	/* autoNegDone */
		unsigned short mf_pream_suppress:1;
		unsigned short unused2:4;
		unsigned short tenbt_capable:1;
		unsigned short bt_fdx_capable:1;
		unsigned short tx_capable:1;
		unsigned short tx_fdx_capable:1;
		unsigned short t4_capable:1;
#endif /* __BIG_ENDIAN */
	} __attribute__ ((packed)) bits;
} __attribute__ ((packed)) phy_status_t;

/* -- id_high -- */

#define PHY_ID_HIGH 0x02

typedef union {
	unsigned short raw;

	struct {
		unsigned short id:16;
	} __attribute__ ((packed)) bits;
} __attribute__ ((packed)) phy_id_high_t;

/* -- id_low -- */

#define PHY_ID_LOW  0x03

typedef union {
	unsigned short raw;

	struct {
#if 0
		unsigned short id:6;
		unsigned short model:6;
		unsigned short revision:4;
#else  /* __BIG_ENDIAN */
		unsigned short revision:4;
		unsigned short model:6;
		unsigned short id:6;
#endif /* __BIG_ENDIAN */
	} __attribute__ ((packed)) bits;
} __attribute__ ((packed)) phy_id_low_t;

/* -- autoneg_advertise  -- */

#define PHY_AUTONEG_ADVERTISE 0x04

/* -- link_partner_ability -- */

#define PHY_LINK_PARTNER_ABILITY 0x05

/* -- -- */

#define BC_PHY_INTERRUPT_ 0x1a

typedef union {

	unsigned short raw;

	struct {

		unsigned short status             : 1;
		unsigned short link_change        : 1;
		unsigned short speed_change       : 1;
		unsigned short duplex_change      : 1;
		unsigned short                    : 4;
		unsigned short mask               : 1;
		unsigned short link_mask          : 1;
		unsigned short speed_mask         : 1;
		unsigned short duplex_mask        : 1;
		unsigned short                    : 2;
		unsigned short enable             : 1;
		unsigned short duplex_led_enable  : 1;

	} __attribute__ ( ( packed ) ) bits;

} __attribute__ ( ( packed ) ) bc_phy_interrupt_t;

/* -- -- */

#define M_PHY_INTERRUPT_ 0x1b

typedef union {

	unsigned short raw;

	struct {

		unsigned short link_up                 : 1;
		unsigned short remote_fault            : 1;
		unsigned short link_down               : 1;
		unsigned short link_partner_ack        : 1;
		unsigned short parallel_detect         : 1;
		unsigned short page_receive            : 1;
		unsigned short receive_error           : 1;
		unsigned short jabber                  : 1;
		unsigned short enable_link_up          : 1;
		unsigned short enable_remote_fault     : 1;
		unsigned short enable_link_down        : 1;
		unsigned short enable_link_partner_ack : 1;
		unsigned short enable_parallel_detect  : 1;
		unsigned short enable_page_receive     : 1;
		unsigned short enable_receive_error    : 1;
		unsigned short enable_jabber           : 1;

	} __attribute__ ( ( packed ) ) bits;

} __attribute__ ( ( packed ) ) m_phy_interrupt_t;

/* -- -- */

#define MICREL_PHY_AUXILIARY_CONTROL_STATUS	0x1f
#define BC_AUX_ERROR_AND_GEN_STATUS_REG		0x1C
#define BC_PHY_ID_HIGH_ID			0x40
#define BC_PHY_ID_LOW_ID			0x61E4
#define BC_PHY_ID_LOW_MODEL			0x1e

typedef union {
	unsigned short raw;

	struct {
		unsigned short unused:3;
		unsigned short energy:1;
		unsigned short force_link:1;
		unsigned short power_saving:1;
		unsigned short interrupt_level:1;
		unsigned short jabber_enable:1;
		unsigned short autoneg_indication:1;
		unsigned short enable_pause:1;
		unsigned short isolate:1;
		unsigned short op_mode_indication:3;
		unsigned short unused2:2;
	} bits;
} micrel_phy_auxiliary_control_status_t;

/*
  Auto-negotiation Advertisement Values.
*/

#define PHY_AUTONEG_ADVERTISE_100FULL	0x101
#define PHY_AUTONEG_ADVERTISE_100	0x081
#define PHY_AUTONEG_ADVERTISE_10FULL	0x041
#define PHY_AUTONEG_ADVERTISE_10	0x021

#define UNKNOWN_PHY_ 0x0
#define BCM5221_PHY_ 0x1
#define MICREL_PHY_  0x2

static int phy_type_ = UNKNOWN_PHY_;

static int phy_read_(int, int, unsigned short *);
static int phy_write_(int, int, unsigned short);
static int phy_speed_(int);
static int phy_duplex_(int);
static int phy_renegotiate_(int);
static int phy_enable_(struct net_device *);

#endif /* PHYLESS */

/*
  ======================================================================
  ======================================================================
  ======================================================================

  NIC Interface

  ======================================================================
  ======================================================================
  ======================================================================
*/

static int enable_(struct net_device *);

/*
  ======================================================================
  ======================================================================
  ======================================================================

  Device Data Structures

  ======================================================================
  ======================================================================
  ======================================================================
*/


typedef struct {

#ifndef CONFIG_ARM

	/* Word 0 */
	unsigned long unused:24;
	/* big endian to little endian */
	unsigned long byte_swapping_on:1;
	unsigned long error:1;
	unsigned long interrupt_on_completion:1;
	unsigned long end_of_packet:1;
	unsigned long start_of_packet:1;
	unsigned long write:1;
	/* 00=Fill|01=Block|10=Scatter */
	unsigned long transfer_type:2;

	/* Word 1 */
	unsigned long pdu_length:16;
	unsigned long data_transfer_length:16;

	/* Word 2 */
	unsigned long target_memory_address;

	/* Word 3 */
	unsigned long host_data_memory_pointer;

#else

	/* Word 0 */
	/* 00=Fill|01=Block|10=Scatter */
	unsigned long transfer_type:2;
	unsigned long write:1;
	unsigned long start_of_packet:1;
	unsigned long end_of_packet:1;
	unsigned long interrupt_on_completion:1;
	unsigned long error:1;
	/* big endian to little endian */
	unsigned long byte_swapping_on:1;
	unsigned long unused:24;

	/* Word 1 */
	unsigned long data_transfer_length:16;
	unsigned long pdu_length:16;

	/* Word 2 */
	unsigned long target_memory_address;

	/* Word 3 */
	unsigned long host_data_memory_pointer;

#endif

} __attribute__ ((packed)) appnic_dma_descriptor_t;

typedef union {

	unsigned long raw;

	struct {
#ifndef CONFIG_ARM
		unsigned long unused:11;
		unsigned long generation_bit:1;
		unsigned long offset:20;
#else
		unsigned long offset:20;
		unsigned long generation_bit:1;
		unsigned long unused:11;
#endif
	} __attribute__ ((packed)) bits;

} __attribute__ ((packed)) appnic_queue_pointer_t;

/*
  =============================================================================
  The Device Struction
  =============================================================================
*/

typedef struct {

	/* net_device */
	struct net_device *device;

	/* Addresses, Interrupt, and PHY stuff. */
	unsigned long rx_base;
	unsigned long tx_base;
	unsigned long dma_base;
	unsigned long tx_interrupt;
	unsigned long rx_interrupt;
	unsigned long dma_interrupt;
	unsigned long mdio_clock;
	unsigned long phy_address;
	unsigned long ad_value;
	unsigned char mac_addr[6];

#ifdef LSINET_NAPI
	/* napi */
	struct napi_struct napi;
#endif /* LSINET_ENABLE_NAPI */

	/* statistics */
	struct net_device_stats stats;

	/*
	 * DMA-able memory.
	 */

	int dma_alloc_size;
	void *dma_alloc;
	dma_addr_t dma_alloc_dma;
	int dma_alloc_offset;

	/* tail pointers */
	volatile appnic_queue_pointer_t *rx_tail;
	dma_addr_t rx_tail_dma;
	volatile appnic_queue_pointer_t *tx_tail;
	dma_addr_t tx_tail_dma;

	/* descriptors */
	appnic_dma_descriptor_t *rx_desc;
	dma_addr_t rx_desc_dma;
	unsigned rx_num_desc;
	appnic_dma_descriptor_t *tx_desc;
	dma_addr_t tx_desc_dma;
	unsigned tx_num_desc;

	/* buffers */
	unsigned rx_buf_sz;
	unsigned rx_buf_per_desc;
	void *rx_buf;
	dma_addr_t rx_buf_dma;
	unsigned tx_buf_sz;
	unsigned tx_buf_per_desc;
	void *tx_buf;
	dma_addr_t tx_buf_dma;

	/*
	 * The local pointers
	 */

	appnic_queue_pointer_t rx_tail_copy;
	appnic_queue_pointer_t rx_head;

	appnic_queue_pointer_t tx_tail_copy;
	appnic_queue_pointer_t tx_head;

	/*
	 * Polling Mode?
	 */

	int polling;

	/*
	 * Spin Lock
	 */

	spinlock_t lock;
	spinlock_t extra_lock;

	/*
	 * TEMP: semaphores for locking Tx/Rx operations
	 */

	struct mutex tx_sem;
	struct mutex rx_sem;
	struct mutex poll_sem;

} appnic_device_t;

#define DESCRIPTOR_GRANULARITY 64
#define BUFFER_ALIGNMENT 64

#define ALIGN64B(address) \
	((((unsigned long) (address) + (64UL - 1UL)) & ~(64UL - 1UL)))

#define ALIGN64B_OFFSET(address) \
	(ALIGN64B(address) - (unsigned long) (address))

#define APPNIC_NAME "appnic"

/*
 * Overview
 * --------
 *
 * Register offset decoding is as follows:
 *
 * Bit(s) Description
 *
 * 16:15  define the Channel.  There is only one; therefore, 00.
 * 14:12  define the MAC within the channel.  Only one so 000.
 * 11:10  define the register "space" as follows:
 * 00 = fast ethernet MACmw.l 06000000 ffffffff 3200000
 * 10 = global
 * 11 = interrupt
 * 9: 2  register
 * 1: 0  always 00, 32 bit registers only.
 *
 * Receive registers start at the base address.  Transmit registers start
 * at 0x20000 above the base address.  DMA start at a completely different
 * base address (in this case 0x8000000 above the base).
 *
*/

/*
  ======================================================================
  ======================================================================
  ======================================================================

  Registers.

  ======================================================================
  ======================================================================
  ======================================================================
*/

/* SMII Status ------------------------------------------------------ */

#define APPNIC_RX_SMII_STATUS		(rx_base + 0x10)
#define APPNIC_RX_SMII_STATUS_SPEED	0x01
#define APPNIC_RX_SMII_STATUS_DUPLEX	0x02
#define APPNIC_RX_SMII_STATUS_LINK	0x04
#define APPNIC_RX_SMII_STATUS_JABBER	0x08
#define APPNIC_RX_SMII_STATUS_FCD	0x10 /* False Carrier Detect */

#define SMII_SPEED_100(smii_status_) \
	(0 != (smii_status_ & APPNIC_RX_SMII_STATUS_SPEED))
#define SMII_DUPLEX(smii_status_) \
	(0 != (smii_status_ & APPNIC_RX_SMII_STATUS_DUPLEX))
#define SMII_LINK(smii_status_) \
	(0 != (smii_status_ & APPNIC_RX_SMII_STATUS_LINK))
#define SMII_JABBER(smii_status_) \
	(0 != (smii_status_ & APPNIC_RX_SMII_STATUS_JABBER))

/* Receive Configuration -------------------------------------------- */

#define APPNIC_RX_CONF	   (rx_base + 0x004c)
#define APPNIC_RX_CONF_ENABLE   0x0001
/* Pass Any Packet */
#define APPNIC_RX_CONF_PAP	0x0002
#define APPNIC_RX_CONF_JUMBO9K  0x0008
#define APPNIC_RX_CONF_STRIPCRC 0x0010
/* Accept All MAC Types */
#define APPNIC_RX_CONF_AMT	0x0020
/* Accept Flow Control */
#define APPNIC_RX_CONF_AFC	0x0040
/* Enable VLAN */
#define APPNIC_RX_CONF_VLAN	0x0200
/* RX MAC Speed, 1=100MBS */
#define APPNIC_RX_CONF_SPEED    0x0800
/* 1=Duplex Mode */
#define APPNIC_RX_CONF_DUPLEX   0x1000
/* 1=Enable */
#define APPNIC_RX_CONF_LINK	0x2000
/*
 * Determines the action taken when the FE MAC
 * receives an FC packet in FD mode.
 */
#define APPNIC_RX_CONF_RXFCE    0x4000
/*
 * Controls the insertion of FC packets
 * by the MAC transmitter.
 */
#define APPNIC_RX_CONF_TXFCE    0x8000

/* Receive Stat Overflow -------------------------------------------- */

#define APPNIC_RX_STAT_OVERFLOW (rx_base + 0x278)

/* Receive Stat Undersize ------------------------------------------- */

#define APPNIC_RX_STAT_UNDERSIZE (rx_base + 0x280)

/* Receive Stat Oversize -------------------------------------------- */

#define APPNIC_RX_STAT_OVERSIZE (rx_base + 0x2b8)

/* Receive Stat Multicast ------------------------------------------- */

#define APPNIC_RX_STAT_MULTICAST (rx_base + 0x2d0)

/* Receive Stat Packet OK ------------------------------------------- */

#define APPNIC_RX_STAT_PACKET_OK (rx_base + 0x2c0)

/* Receive Stat CRC Error ------------------------------------------- */

#define APPNIC_RX_STAT_CRC_ERROR (rx_base + 0x2c8)

/* Receive Stat Align Error ----------------------------------------- */

#define APPNIC_RX_STAT_ALIGN_ERROR (rx_base + 0x2e8)

/* Receive Ethernet Mode -------------------------------------------- */

#define APPNIC_RX_MODE (rx_base + 0x0800)
#define APPNIC_RX_MODE_ETHERNET_MODE_ENABLE 0x00001

/* Receive Soft Reset ----------------------------------------------- */

#define APPNIC_RX_SOFT_RESET (rx_base + 0x0808)
#define APPNIC_RX_SOFT_RESET_MAC_0 0x00001

/* Receive Internal Interrupt Control ------------------------------- */

#define APPNIC_RX_INTERNAL_INTERRUPT_CONTROL (rx_base + 0xc00)
#define APPNIC_RX_INTERNAL_INTERRUPT_CONTROL_MAC_0 0x1

/* Receive External Interrupt Control ------------------------------- */

#define APPNIC_RX_EXTERNAL_INTERRUPT_CONTROL (rx_base + 0xc04)
#define APPNIC_RX_EXTERNAL_INTERRUPT_CONTROL_MAC_0_HIGH_LOW 0x10
#define APPNIC_RX_EXTERNAL_INTERRUPT_CONTROL_MAC_0 0x1

/* Receive Interrupt Status ----------------------------------------- */

#define APPNIC_RX_INTERRUPT_STATUS (rx_base + 0xc20)
#define APPNIC_RX_INTERRUPT_EXTERNAL_STATUS_MAC_0 0x10
#define APPNIC_RX_INTERRUPT_INTERNAL_STATUS_MAC_0 0x1

/* Transmit Watermark ----------------------------------------------- */

#define APPNIC_TX_WATERMARK (tx_base + 0x18)
#define APPNIC_TX_WATERMARK_TXCONFIG_DTPA_ASSERT 0x8000
#define APPNIC_TX_WATERMARK_TXCONFIG_DTPA_DISABLE 0x4000
#define APPNIC_TX_WATERMARK_TXCONFIG_DTPA_WATER_MARK_HIGH 0x3f00
#define APPNIC_TX_WATERMARK_TXCONFIG_DTPA_WATER_MARK_LOW 0x3f

/* Swap Source Address Registers ------------------------------------ */

#define APPNIC_SWAP_SOURCE_ADDRESS_2 (tx_base + 0x20)
#define APPNIC_SWAP_SOURCE_ADDRESS_1 (tx_base + 0x24)
#define APPNIC_SWAP_SOURCE_ADDRESS_0 (tx_base + 0x28)

/* Transmit Extended Configuration ---------------------------------- */

#define APPNIC_TX_EXTENDED_CONF (tx_base + 0x30)
#define APPNIC_TX_EXTENDED_CONF_TRANSMIT_COLLISION_WATERMARK_LEVEL 0xf000
#define APPNIC_TX_EXTENDED_CONF_EXCESSIVE_DEFFERED_PACKET_DROP 0x200
#define APPNIC_TX_EXTENDED_CONF_JUMBO9K 0x100
#define APPNIC_TX_EXTENDED_CONF_LATE_COLLISION_WINDOW_COUNT 0xff

/* Transmit Half Duplex Configuration ------------------------------- */

#define APPNIC_TX_HALF_DUPLEX_CONF (tx_base + 0x34)
#define APPNIC_TX_HALF_DUPLEX_CONF_RANDOM_SEED_VALUE 0xff

/* Transmit Configuration ------------------------------------------- */

#define APPNIC_TX_CONF			(tx_base + 0x0050)
#define APPNIC_TX_CONF_ENABLE_SWAP_SA	0x8000
#define APPNIC_TX_CONF_LINK		0x2000
#define APPNIC_TX_CONF_DUPLEX		0x1000
#define APPNIC_TX_CONF_SPEED		0x0800
#define APPNIC_TX_CONF_XBK_RST_RX_NTX	0x0600
#define APPNIC_TX_CONF_IFG		0x01f0
#define APPNIC_TX_CONF_APP_CRC_ENABLE	0x0004
#define APPNIC_TX_CONF_PAD_ENABLE	0x0002
#define APPNIC_TX_CONF_ENABLE		0x0001

#define TX_CONF_SET_IFG(tx_configuration_, ifg_)		\
	do {							\
		(tx_configuration_) &= ~APPNIC_TX_CONF_IFG;	\
		(tx_configuration_) |= ((ifg_ & 0x1f) << 4);	\
	} while (0);

/* Transmit Time Value Configuration -------------------------------- */

#define APPNIC_TX_TIME_VALUE_CONF (tx_base + 0x5c)
#define APPNIC_TX_TIME_VALUE_CONF_PAUSE_VALUE 0xffff

/* Transmit Stat Underrun ------------------------------------------- */

#define APPNIC_TX_STAT_UNDERRUN (tx_base + 0x300)

/* Transmit Stat Packet OK ------------------------------------------ */

#define APPNIC_TX_STAT_PACKET_OK (tx_base + 0x318)

/* Transmit Stat Undersize ------------------------------------------ */

#define APPNIC_TX_STAT_UNDERSIZE (tx_base + 0x350)

/* Transmit Status Late Collision ----------------------------------- */

#define APPNIC_TX_STATUS_LATE_COLLISION (tx_base + 0x368)

/* Transmit Status Excessive Collision ------------------------------ */

#define APPNIC_TX_STATUS_EXCESSIVE_COLLISION (tx_base + 0x370)

/* Transmit Stat Collision Above Watermark -------------------------- */

#define APPNIC_TX_STAT_COLLISION_ABOVE_WATERMARK (tx_base + 0x380)

/* Transmit Mode ---------------------------------------------------- */

#define APPNIC_TX_MODE (tx_base + 0x800)
#define APPNIC_TX_MODE_ETHERNET_MODE_ENABLE 0x1

/* Transmit Soft Reset ---------------------------------------------- */

#define APPNIC_TX_SOFT_RESET (tx_base + 0x808)
#define APPNIC_TX_SOFT_RESET_MAC_0 0x1

/* Transmit Interrupt Control --------------------------------------- */

#define APPNIC_TX_INTERRUPT_CONTROL (tx_base + 0xc00)
#define APPNIC_TX_INTERRUPT_CONTROL_MAC_0 0x1

/* Transmit Interrupt Status ---------------------------------------- */

#define APPNIC_TX_INTERRUPT_STATUS (tx_base + 0xc20)
#define APPNIC_TX_INTERRUPT_STATUS_MAC_0 0x1

/* */

#define APPNIC_DMA_PCI_CONTROL (dma_base + 0x00)

/* */

#define APPNIC_DMA_CONTROL (dma_base + 0x08)

/* DMA Interrupt Status --------------------------------------------- */

#define APPNIC_DMA_INTERRUPT_STATUS (dma_base + 0x18)
#define APPNIC_DMA_INTERRUPT_STATUS_RX 0x2
#define APPNIC_DMA_INTERRUPT_STATUS_TX 0x1
#define RX_INTERRUPT(dma_interrupt_status_) \
	(0 != (dma_interrupt_status_ & APPNIC_DMA_INTERRUPT_STATUS_RX))
#define TX_INTERRUPT(dma_interrupt_status_) \
	(0 != (dma_interrupt_status_ & APPNIC_DMA_INTERRUPT_STATUS_TX))

/* DMA Interrupt Enable --------------------------------------------- */

#define APPNIC_DMA_INTERRUPT_ENABLE (dma_base + 0x1c)
#define APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE 0x2
#define APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT 0x1

/* DMA Receive Queue Base Address ----------------------------------- */

#define APPNIC_DMA_RX_QUEUE_BASE_ADDRESS (dma_base + 0x30)

/* DMA Receive Queue Size ------------------------------------------- */

#define APPNIC_DMA_RX_QUEUE_SIZE (dma_base + 0x34)

/* DMA Transmit Queue Base Address ---------------------------------- */

#define APPNIC_DMA_TX_QUEUE_BASE_ADDRESS (dma_base + 0x38)

/* DMA Transmit Queue Size ------------------------------------------ */

#define APPNIC_DMA_TX_QUEUE_SIZE (dma_base + 0x3c)

/* DMA Recevie Tail Pointer Address --------------------------------- */

#define APPNIC_DMA_RX_TAIL_POINTER_ADDRESS (dma_base + 0x48)

/* DMA Transmit Tail Pointer Address -------------------------------- */

#define APPNIC_DMA_TX_TAIL_POINTER_ADDRESS (dma_base + 0x4c)

/* DMA Receive Head Pointer ----------------------------------------- */

#define APPNIC_DMA_RX_HEAD_POINTER			(dma_base + 0x50)
#define APPNIC_DMA_RX_HEAD_POINTER_GB			0x100000
#define APPNIC_DMA_RX_HEAD_POINTER_POINTER		0x0fffff

/* DMA Receive Tail Pointer Local Copy ------------------------------ */

#define APPNIC_DMA_RX_TAIL_POINTER_LOCAL_COPY		(dma_base + 0x54)
#define APPNIC_DMA_RX_TAIL_POINTER_LOCAL_COPY_GB	0x100000
#define APPNIC_DMA_RX_TAIL_POINTER_LOCAL_COPY_POINTER	0x0fffff

/* DMA Transmit Head Pointer ---------------------------------------- */

#define APPNIC_DMA_TX_HEAD_POINTER			(dma_base + 0x58)
#define APPNIC_DMA_TX_HEAD_POINTER_GB			0x100000
#define APPNIC_DMA_TX_HEAD_POINTER_POINTER		0x0fffff

/* DMA Transmit Tail Pointer Local Copy ----------------------------- */

#define APPNIC_DMA_TX_TAIL_POINTER_LOCAL_COPY		(dma_base + 0x5c)
#define APPNIC_DMA_TX_TAIL_POINTER_LOCAL_COPY_GB	0x100000
#define APPNIC_DMA_TX_TAIL_POINTER_LOCAL_COPY_POINTER	0x0fffff

/*
  ======================================================================
  ======================================================================
  ======================================================================

  Access

  N.B. On the APP, DMA transfers from the NIC MUST USE THE MEMORY
  ALIAS AT 0x60000000!

  ======================================================================
  ======================================================================
  ======================================================================
*/

#ifdef CONFIG_ACP

#define readio(address) in_le32((u32 *) (address))

#define writeio(value, address) out_le32((u32 *) (address), (value));

static inline void
readdescriptor(unsigned long address, appnic_dma_descriptor_t *descriptor)
{
	unsigned long *from = (unsigned long *) address;
	unsigned long *to = (unsigned long *) descriptor;

	*to++ = swab32(*from++);
	*to++ = swab32(*from++);
	*to++ = swab32(*from++);
	*to++ = swab32(*from++);
	return;
}

static inline void
writedescriptor(unsigned long address,
		 const appnic_dma_descriptor_t *descriptor)
{
	unsigned long *to = (unsigned long *) address;
	unsigned long *from = (unsigned long *) descriptor;

	*to++ = swab32(*from++);
	*to++ = swab32(*from++);
	*to++ = swab32(*from++);
	*to++ = swab32(*from++);
	return;
}

static inline appnic_queue_pointer_t
swab_queue_pointer(const appnic_queue_pointer_t *old_queue)
{
	appnic_queue_pointer_t new_queue;
	new_queue.raw = swab32(old_queue->raw);
	return new_queue;
}

#define SWAB_QUEUE_POINTER(pointer) \
swab_queue_pointer((const appnic_queue_pointer_t *) (pointer))

#else

#define readio(address) readl((address))
#define writeio(value, address) writel((value), (address))

static inline void
readdescriptor(unsigned long address, appnic_dma_descriptor_t *descriptor)
{
	memcpy(descriptor, (void *) address, sizeof(appnic_dma_descriptor_t));
	return;
}

static inline void
writedescriptor(unsigned long address,
		 const appnic_dma_descriptor_t *descriptor)
{
	memcpy((void *) address, descriptor, sizeof(appnic_dma_descriptor_t));
	return;
}

static inline appnic_queue_pointer_t
swab_queue_pointer(const appnic_queue_pointer_t *old_queue)
{
	return *old_queue;
}

#define SWAB_QUEUE_POINTER(pointer) \
swab_queue_pointer((const appnic_queue_pointer_t *)(pointer))

#endif /* CONFIG_ACP */

#ifdef LOG_MAC_ACCESS

static unsigned long read_mac_(unsigned int address)
{
	unsigned long value_ = readio(address);
	printk(KERN_INFO "-MAC-  0x%04x => 0x%08lx\n",
	       (address & 0x1fff), value_);
	return value_;
}

static void write_mac_(unsigned long value, unsigned int address)
{
	printk(KERN_INFO "-MAC-  0x%04x <= 0x%08lx\n",
	       (address & 0x1fff), value);
	writeio(value, address);
	return;
}

#else  /* ! LOG_MAC_ACCESS */

#define read_mac_(address) readio((address))
#define write_mac_(value, address) writeio((value), (address))

#endif

#ifndef PHYLESS
#ifdef LOG_PHY_ACCESS

static unsigned long read_phy_(unsigned int address)
{
	unsigned long value_ = readl(address);
	printk(KERN_INFO "-PHY- HOST + 0x%04x => 0x%08lx\n",
	       (address & 0xff), value_);
	return value_;
}

static void write_phy_(unsigned long value, unsigned int address)
{
	printk(KERN_INFO "-PHY- HOST + 0x%04x <= 0x%08lx\n",
	       (address & 0xff), value);
	writel(value, address);
	return;

}

#else  /* ! LOG_PHY_ACCESS */

#define read_phy_(address)		readl((address))
#define write_phy_(value, address)	writel(value, address)

#endif
#endif /* PHYLESS */

/*
  ======================================================================
  ======================================================================
  ======================================================================

  Module Information

  ======================================================================
  ======================================================================
  ======================================================================
*/

MODULE_AUTHOR("John Jacques");
MODULE_DESCRIPTION("Agere APP3xx ethernet driver");
MODULE_LICENSE("GPL");

/*
 *  ----- Note On Buffer Space -----
 *
 *  Minimum number of descriptors is 64 for the receiver and 64 for the
 *  transmitter; therefore, 2048 bytes (16 bytes each).
 *  This driver uses the following parameters,
 *  all of which may be set on the command line if this drivers is used
 *  as a module.
 *
 *  - rx_num_desc : Number of receive descriptors. This  must be a multiple
 *                  of 64.
 *  - tx_num_desc : Number of transmit descriptors. This must be a multiple
 *                  of 64.
 *
 *  The scheme used will be as follows:
 *
 *  - num_[rt]x_desc will be adjusted to be a multiple of 64 (if necessary).
 *  - An skb (with the data area 64 byte aligned) will be allocated for each rx
 *    descriptor.
 */

/*
 * Receiver
 */

int rx_num_desc = (CONFIG_LSI_NET_NUM_RX_DESC * DESCRIPTOR_GRANULARITY);
module_param(rx_num_desc, int, 0);
MODULE_PARM_DESC(rx_num_desc, "appnic : Number of receive descriptors");

int rx_buf_sz = CONFIG_LSI_NET_RX_BUF_SZ;
module_param(rx_buf_sz, int, 0);
MODULE_PARM_DESC(rx_buf_sz, "appnic : Receive buffer size");

/*
 * Transmitter
 */

int tx_num_desc = (CONFIG_LSI_NET_NUM_TX_DESC * DESCRIPTOR_GRANULARITY);
module_param(tx_num_desc, int, 0);
MODULE_PARM_DESC(tx_num_desc, "appnic : Number of receive descriptors");

int tx_buf_sz = CONFIG_LSI_NET_TX_BUF_SZ;
module_param(tx_buf_sz, int, 0);
MODULE_PARM_DESC(tx_buf_sz, "Appnic : Receive buffer size");

/*
 * Only 1 device is possible...
 */

struct net_device *this_net_device;

static unsigned long dropped_by_stack_ = 0;
static unsigned long out_of_tx_descriptors_ = 0;
static unsigned long transmit_interrupts_ = 0;
#ifdef LSINET_NAPI
static unsigned long receive_interrupts_ = 0;
#endif

#define APPNIC_TIMER_PERIOD 5
#ifndef PHYLESS
static void appnic_timer_handler_(unsigned long);
static struct timer_list appnic_timer_;
#endif /* PHYLESS */

/*
  ======================================================================
  ======================================================================
  ======================================================================

  Utility Functions

  ======================================================================
  ======================================================================
  ======================================================================
*/

/*
  ----------------------------------------------------------------------
  dump_packet

  The format is so you can import to WireShark...
*/

void
dump_packet(const char *header, void *packet, int length)
{
	char buffer[256];
	char *string;
	unsigned long offset = 0;
	int i;
	unsigned char *data = packet;

	printk("---- %s\n", header);

	while (0 < length) {
		int this_line;

		string = buffer;
		string += sprintf(string, "%06lx ", offset);
		this_line = (16 > length) ? length : 16;

		for (i = 0; i < this_line; ++i) {
			string += sprintf(string, "%02x ", *data++);
			--length;
			++offset;
		}

		printk("%s\n", buffer);
	}

	printk("\n");
}

/*
  ------------------------------------------------------------------------------
  dump_registers
*/

static void
dump_registers(struct net_device *device)
{
	appnic_device_t *adapter = netdev_priv(device);
	int rc;
	int i;

	unsigned long phy_registers[] =	{
		1, 15, 0x1e, 0x1f
	};

	void *rx_registers[] = {
		APPNIC_RX_CONF,
		APPNIC_RX_STAT_OVERFLOW,
		APPNIC_RX_STAT_UNDERSIZE,
		APPNIC_RX_STAT_OVERSIZE,
		APPNIC_RX_STAT_MULTICAST,
		APPNIC_RX_STAT_PACKET_OK,
		APPNIC_RX_STAT_CRC_ERROR,
		APPNIC_RX_STAT_ALIGN_ERROR,
	};

	void *tx_registers[] = {
		APPNIC_TX_CONF,
		APPNIC_TX_STAT_UNDERRUN,
		APPNIC_TX_STAT_PACKET_OK,
		APPNIC_TX_STAT_UNDERSIZE,
		APPNIC_TX_STATUS_LATE_COLLISION,
		APPNIC_TX_STATUS_EXCESSIVE_COLLISION,
		APPNIC_TX_STAT_COLLISION_ABOVE_WATERMARK
	};

	/*
	  PHY
	*/

	printk(KERN_ERR
	       "%s:%d - PHY Registers\n", __FILE__, __LINE__);

	for (i = 0; i < sizeof(phy_registers) / sizeof(unsigned long); ++i) {
		unsigned short value;

		rc = acp_mdio_read(adapter->phy_address,
				   phy_registers[i], &value);

		if (rc) {
			printk(KERN_ERR
			       "%s:%d - Error reading PHY register %lu!\n",
			       __FILE__, __LINE__, phy_registers[i]);
		} else {
			printk(KERN_ERR
			       "0x%08lx: 0x%08x\n", phy_registers[i], value);
		}
	}

	/*
	  MAC RX
	*/

	printk(KERN_ERR
	       "%s:%d - MAC RX Registers\n", __FILE__, __LINE__);

	for (i = 0; i < sizeof(rx_registers) / sizeof(unsigned long); ++i) {
		unsigned long value;

		value = readl(rx_registers[i]);
		printk(KERN_ERR
		       "0x%08lx: 0x%08lx\n", rx_registers[i], value);
	}

	/*
	  MAC TX
	*/

	printk(KERN_ERR
	       "%s:%d - MAC TX Registers\n", __FILE__, __LINE__);

	for (i = 0; i < sizeof(tx_registers) / sizeof(unsigned long); ++i) {
		unsigned long value;

		value = readl(tx_registers[i]);
		printk(KERN_ERR
		       "0x%08x: 0x%08x\n", tx_registers[i], value);
	}

	/*
	  MAC DMA
	*/

	return;
}

/*
  ----------------------------------------------------------------------
  dump_descriptor
*/

static void dump_descriptor(const char *title,
			    appnic_dma_descriptor_t *descriptor) {

	printk("-- %s -- descriptor at 0x%p\n", title, descriptor );
	printk("        byte_swapping_on=%d\n" \
	       " interrupt_on_completion=%d\n"	\
	       "           end_of_packet=%d\n"	\
	       "         start_of_packet=%d\n"	\
	       "                   write=%d\n"	  \
	       "           transfer_type=0x%x\n"  \
	       "              pdu_length=0x%x\n"  \
	       "    data_transfer_length=0x%x\n"  \
	       "   target_memory_address=0x%x\n"	\
	       "host_data_memory_pointer=0x%x\n",
	       (unsigned int ) ( descriptor->byte_swapping_on ),
	       (unsigned int ) ( descriptor->interrupt_on_completion ),
	       (unsigned int ) ( descriptor->end_of_packet ),
	       (unsigned int ) ( descriptor->start_of_packet ),
	       (unsigned int ) ( descriptor->write ),
	       (unsigned int ) ( descriptor->transfer_type ),
	       (unsigned int ) ( descriptor->pdu_length ),
	       (unsigned int ) ( descriptor->data_transfer_length ),
	       (unsigned int ) ( descriptor->target_memory_address ),
	       (unsigned int ) ( descriptor->host_data_memory_pointer ) );

}

/*
  ------------------------------------------------------------------------------
  dump_descriptors
*/

static void
dump_descriptors(struct net_device *device)
{
	appnic_device_t *adapter = netdev_priv(device);
	appnic_dma_descriptor_t *descriptor;
	char buffer[256];
	int i;

	printk("Address/Size: RX 0x%p/0x%x TX 0x%p/0x%x\n",
	       adapter->rx_desc, adapter->rx_num_desc,
	       adapter->tx_desc, adapter->tx_num_desc);

	printk("RX Pointers: tail=0b%d:0x%x tail_copy=0b%d:0x%x head=0b%d:0x%x\n",
	       adapter->rx_tail->bits.generation_bit,
	       adapter->rx_tail->bits.offset,
	       adapter->rx_tail_copy.bits.generation_bit,
	       adapter->rx_tail_copy.bits.offset,
	       adapter->rx_head.bits.generation_bit,
	       adapter->rx_head.bits.offset);

#if 0
	printk("----RX Descriptors\n");
	descriptor = adapter->rx_desc;

	for (i = 0; i < adapter->rx_num_desc; ++i) {
		sprintf(buffer, "RX:%d", i);
		dump_descriptor(buffer, descriptor++);
	}
#endif

	printk("TX Pointers: tail=0b%d:0x%x tail_copy=0b%d:0x%x head=0b%d:0x%x\n",
	       adapter->tx_tail->bits.generation_bit,
	       adapter->tx_tail->bits.offset,
	       adapter->tx_tail_copy.bits.generation_bit,
	       adapter->tx_tail_copy.bits.offset,
	       adapter->tx_head.bits.generation_bit,
	       adapter->tx_head.bits.offset);
	printk("----TX Descriptors\n");
	descriptor = adapter->tx_desc;

	for (i = 0; i < adapter->tx_num_desc; ++i) {
		sprintf(buffer, "TX:%d", i);
		dump_descriptor(buffer, descriptor++);
	}
}

/*
  ----------------------------------------------------------------------
  clear_statistics_
*/

static void clear_statistics_(appnic_device_t *device)
{
	int waste_;

	/*
	 * Clear memory.
	 */

	memset((void *) &(device->stats), 0,
		sizeof(struct net_device_stats));

	/*
	 * Clear counters.
	 */

	waste_ = read_mac_(APPNIC_RX_STAT_PACKET_OK); /* rx_packets */
	waste_ = read_mac_(APPNIC_TX_STAT_PACKET_OK); /* tx_packets */

	/* rx_bytes kept by driver. */
	/* tx_bytes kept by driver. */
	/* rx_errors will be the sum of the rx errors available. */
	/* tx_errors will be the sum of the tx errors available. */
	/* rx_dropped (unable to allocate skb) will be maintained by driver */
	/* tx_dropped (unable to allocate skb) will be maintained by driver */

	/* multicast */

	waste_ = read_mac_(APPNIC_RX_STAT_MULTICAST);

	/* collisions will be the sum of the three following. */

	waste_ = read_mac_(APPNIC_TX_STATUS_LATE_COLLISION);
	waste_ = read_mac_(APPNIC_TX_STATUS_EXCESSIVE_COLLISION);
	waste_ = read_mac_(APPNIC_TX_STAT_COLLISION_ABOVE_WATERMARK);

	/* rx_length_errors will be the sum of the two following. */

	waste_ = read_mac_(APPNIC_RX_STAT_UNDERSIZE);
	waste_ = read_mac_(APPNIC_RX_STAT_OVERSIZE);

	/* rx_over_errors (out of descriptors?) maintained by the driver. */
	/* rx_crc_errors */

	waste_ = read_mac_(APPNIC_RX_STAT_CRC_ERROR);

	/* rx_frame_errors */

	waste_ = read_mac_(APPNIC_RX_STAT_ALIGN_ERROR);

	/* rx_fifo_errors */

	waste_ = read_mac_(APPNIC_RX_STAT_OVERFLOW);

	/* rx_missed will not be maintained. */
	/* tx_aborted_errors will be maintained by the driver. */
	/* tx_carrier_errors will not be maintained. */
	/* tx_fifo_errors */

	waste_ = read_mac_(APPNIC_TX_STAT_UNDERRUN);

	/* tx_heartbeat_errors */
	/* tx_window_errors */

	/* rx_compressed will not be maintained. */
	/* tx_compressed will not be maintained. */

	/*
	 * That's all.
	 */

	return;
}

/*
 * ----------------------------------------------------------------------
 * get_hw_statistics_
 *
 *  -- NOTES --
 *
 *  1) The hardware clears the statistics registers after a read.
 */

static void get_hw_statistics_(appnic_device_t *device)
{
	unsigned long flags_;

	/* tx_packets */

	device->stats.tx_packets += read_mac_(APPNIC_TX_STAT_PACKET_OK);

	/* multicast */

	device->stats.multicast += read_mac_(APPNIC_RX_STAT_MULTICAST);

	/* collision */

	device->stats.collisions += read_mac_(APPNIC_TX_STATUS_LATE_COLLISION);
	device->stats.collisions +=
	read_mac_(APPNIC_TX_STATUS_EXCESSIVE_COLLISION);
	device->stats.collisions +=
	read_mac_(APPNIC_TX_STAT_COLLISION_ABOVE_WATERMARK);

	/* rx_length_errors */

	device->stats.rx_length_errors += read_mac_(APPNIC_RX_STAT_UNDERSIZE);
	device->stats.rx_length_errors += read_mac_(APPNIC_RX_STAT_OVERSIZE);

	/* tx_fifo_errors */

	device->stats.tx_fifo_errors += read_mac_(APPNIC_TX_STAT_UNDERRUN);

	/*
	 * Lock this section out so the statistics maintained by the driver
	 * don't get clobbered.
	 */


	spin_lock_irqsave(&device->lock, flags_);

	device->stats.rx_errors +=
		(device->stats.rx_length_errors +
		 device->stats.rx_crc_errors +
		 device->stats.rx_frame_errors +
		 device->stats.rx_fifo_errors +
		 device->stats.rx_dropped +
		 device->stats.rx_over_errors);

	device->stats.rx_dropped = 0;
	device->stats.rx_over_errors = 0;

	device->stats.tx_errors +=
		(device->stats.tx_fifo_errors +
		 device->stats.tx_aborted_errors);
	device->stats.tx_aborted_errors = 0;

	spin_unlock_irqrestore(&device->lock, flags_);

	/*
	 * That's all.
	 */

	return;
}

/*
 * ----------------------------------------------------------------------
 * queue_initialized_
 *
 * Returns the number of descriptors that are ready to receive packets
 * or are waiting to transmit packets.  (from tail to head).
 */

static int queue_initialized_(appnic_queue_pointer_t head,
			      appnic_queue_pointer_t tail,
			      int size)
{
	int initialized;

	/* Calculate the number of descriptors currently initialized. */

	if (head.bits.generation_bit == tail.bits.generation_bit) {
		/* same generation */
		initialized = (head.bits.offset - tail.bits.offset);
	} else {
		/* different generation */
		initialized = head.bits.offset +
			(size * sizeof(appnic_dma_descriptor_t) -
			 tail.bits.offset);
	}

	/* number of descriptors is offset / sizeof(a descriptor) */
	initialized /= sizeof(appnic_dma_descriptor_t);

	return initialized;
}

/*
 * ----------------------------------------------------------------------
 * queue_uninitialzed_
 *
 * Returns the number of unused/uninitialized descriptors. (from head to tail).
*/

static int queue_uninitialized_(appnic_queue_pointer_t head,
				appnic_queue_pointer_t tail,
				int size)
{
	int allocated_;

	/* calculate the number of descriptors currently unused/uninitialized */

	if (head.bits.generation_bit == tail.bits.generation_bit) {
		/* same generation. */
		allocated_ = ((size * sizeof(appnic_dma_descriptor_t)) -
			 head.bits.offset) + tail.bits.offset;
	} else {
		/* different generation. */
		allocated_ = tail.bits.offset - head.bits.offset;
	}

	/* number of descriptors is offset / sizeof(a descriptor). */
	allocated_ /= sizeof(appnic_dma_descriptor_t);

	/* that's all */
	return allocated_;
}

/*
 * ----------------------------------------------------------------------
 * queue_increment_
 */

static void queue_increment_(appnic_queue_pointer_t *queue,
			     int number_of_descriptors)
{
	queue->bits.offset += sizeof(appnic_dma_descriptor_t);

	if ((number_of_descriptors * sizeof(appnic_dma_descriptor_t)) ==
		queue->bits.offset) {

		queue->bits.offset = 0;
		queue->bits.generation_bit =
			(0 == queue->bits.generation_bit) ? 1 : 0;
	}

	return;
}

/*
 * ----------------------------------------------------------------------
 * queue_decrement_
 */

static void queue_decrement_(appnic_queue_pointer_t *queue,
			     int number_of_descriptors)
{
	if (0 == queue->bits.offset) {
		queue->bits.offset =
			((number_of_descriptors - 1) *
			 sizeof(appnic_dma_descriptor_t));
		queue->bits.generation_bit =
			(0 == queue->bits.generation_bit) ? 1 : 0;
	} else {
		queue->bits.offset -= sizeof(appnic_dma_descriptor_t);
	}

	return;
}


#ifndef PHYLESS

static void appnic_timer_handler_(unsigned long __opaque)
{
	struct net_device *device_ = (struct net_device *) __opaque;

	PHY_DEBUG_PRINT("Handling Timer Expiration.\n");
	enable_(device_);
	appnic_timer_.expires = jiffies + (APPNIC_TIMER_PERIOD * HZ);
	add_timer(&appnic_timer_);
}

#endif

/*
 * ----------------------------------------------------------------------
 * enable_
 *
 * -- NOTES --
 *
 * 1) Does not change the default values in the extended and
 *    half-duplex configuration registers.
 */

static int enable_(struct net_device *device)
{
	int return_code_ = 1;
	int carrier_state_ = 0;
	unsigned long rx_configuration_;
	unsigned long tx_configuration_ = 0;
	phy_status_t phy_status_;
	appnic_device_t *apnd = netdev_priv(device);

	rx_configuration_ =
		(APPNIC_RX_CONF_STRIPCRC |
		 APPNIC_RX_CONF_RXFCE |
		 APPNIC_RX_CONF_TXFCE);
	tx_configuration_ =
		(APPNIC_TX_CONF_ENABLE_SWAP_SA |
		 APPNIC_TX_CONF_APP_CRC_ENABLE |
		 APPNIC_TX_CONF_PAD_ENABLE);

	TX_CONF_SET_IFG(tx_configuration_, 0xf);

	DEBUG_PRINT("Enabling the interface.\n");

	/*
	 * Setup the receive and transmit configuration registers (using smii
	 * status to set speed/duplex and check the link status).
	 */

	if ((0 == phy_read_(apnd->phy_address, PHY_STATUS, &phy_status_.raw)) &&
	    (0 == phy_read_(apnd->phy_address, PHY_STATUS, &phy_status_.raw))) {

		PHY_DEBUG_PRINT("phy_status_.raw=0x%x phy address= 0x%x\n",
				phy_status_.raw, apnd->phy_address);
		PHY_DEBUG_PRINT("phy_status_.bits.link_status=0x%x\n",
				phy_status_.bits.link_status);
		PHY_DEBUG_PRINT("phy_status_.bits.autoneg_comp=0x%x\n",
				phy_status_.bits.autoneg_comp);

		if (1 == phy_status_.bits.autoneg_comp) {
			if (1 == phy_status_.bits.link_status) {
				if (1 == phy_speed_(apnd->phy_address)) {
					rx_configuration_ |=
						APPNIC_RX_CONF_SPEED;
					tx_configuration_ |=
						APPNIC_TX_CONF_SPEED;
					PHY_DEBUG_PRINT(
					 "RX/TX conf after phy_speed\n");
				}

				if (1 == phy_duplex_(apnd->phy_address)) {
					rx_configuration_ |=
						APPNIC_RX_CONF_DUPLEX;
					tx_configuration_ |=
						APPNIC_TX_CONF_DUPLEX;
					PHY_DEBUG_PRINT(
					 "RX/TX conf after phy_duplex\n");
				}

				rx_configuration_ |=
					(APPNIC_RX_CONF_ENABLE |
					 APPNIC_RX_CONF_LINK);
				tx_configuration_ |=
					(APPNIC_TX_CONF_LINK |
					 APPNIC_TX_CONF_ENABLE);
				return_code_ = 0;
				carrier_state_ = 1;
			} else {
				netif_carrier_off(device);
			}
		} else {
			netif_carrier_off(device);
		}
	} else {
		ERROR_PRINT("phy_read_() failed!\n");
	}

	if (rx_configuration_ != read_mac_(APPNIC_RX_CONF)) {
		PHY_DEBUG_PRINT("Writing APPNIC_RX_CONF: rx_config: 0x%x\n",
				rx_configuration_);
		write_mac_(rx_configuration_, APPNIC_RX_CONF);
	}

	if (tx_configuration_ != read_mac_(APPNIC_TX_CONF)) {
		PHY_DEBUG_PRINT("Writing APPNIC_TX_CONF: tx_config: 0x%x\n",
				tx_configuration_);
		write_mac_(tx_configuration_, APPNIC_TX_CONF);
	}

	if (0 != carrier_state_) {
		PHY_DEBUG_PRINT("Writing netif_carrier_on to device.\n");
		netif_carrier_on(device);
	} else {
		 netif_carrier_off(device);
	}

	return return_code_;
}

/*
 * ----------------------------------------------------------------------
 * disable_
 */

static void disable_(void)
{
	unsigned long tx_configuration_;
	unsigned long rx_configuration_;

	DEBUG_PRINT("Disabling the interface.\n");

	rx_configuration_ = read_mac_(APPNIC_RX_CONF);
	rx_configuration_ &= ~APPNIC_RX_CONF_ENABLE;
	write_mac_(rx_configuration_, APPNIC_RX_CONF);

	tx_configuration_ = read_mac_(APPNIC_TX_CONF);
	tx_configuration_ &= ~APPNIC_TX_CONF_ENABLE;

	write_mac_(tx_configuration_, APPNIC_TX_CONF);

	/* that's all. */
	return;
}

void disable_nic_(void)
{
	disable_();
}

/*
  ======================================================================
  ======================================================================
  ======================================================================
  PHY interface (BCM5221)
  ======================================================================
  ======================================================================
  ======================================================================
*/

#ifndef PHYLESS

/*
 * ----------------------------------------------------------------------
 * phy_read_
 *
 * Returns -1 if unsuccessful, the (short) value otherwise.
 */

static int
phy_read_(int phy, int reg, unsigned short *value)
{
	return acp_mdio_read(phy, reg, value);
}

/*
 * ----------------------------------------------------------------------
 * phy_write_
 */

static int
phy_write_(int phy, int reg, unsigned short value)
{
	return acp_mdio_write(phy, reg, value);
}

/*
 * ----------------------------------------------------------------------
 * phy_speed_
 *
 * Returns the speed (1=100, 0=10) or an error (-1).
 */

static int
phy_speed_(int phy)
{
	int rc;
	unsigned short oui;
	unsigned short aux;

	rc = phy_read_(phy, PHY_ID_HIGH, &oui);

	if (rc)
		goto error;

	if (0x22 == oui) {
		rc = phy_read_(phy, 0x1e, &aux);

		if (rc)
			goto error;

		aux &= 0x7;

		if (0x6 == aux || 0x2 == aux)
			goto return_100;
		else
			goto return_10;
	} else {
		rc = phy_read_(phy, BC_AUX_ERROR_AND_GEN_STATUS_REG, &aux);

		if (rc)
			goto error;

		if (aux & 2)
			goto return_100;
		else
			goto return_10;
	}

error:
	ERROR_PRINT("phy_speed_() failed!\n");
	return -1;

return_10:
	return 0;

return_100:
	return 1;
}

/*
 * ----------------------------------------------------------------------
 * phy_duplex_
 *
 * Returns duplex status (1=full duplex, 0=half duplex) or an error (-1).
 */

static int
phy_duplex_(int phy)
{
	int rc;
	unsigned short oui;
	unsigned short aux;

	rc = phy_read_(phy, PHY_ID_HIGH, &oui);

	if (rc)
		goto error;

	if (0x22 == oui) {
		rc = phy_read_(phy, 0x1e, &aux);

		if (rc)
			goto error;

		aux &= 0x7;

		if (0x6 == aux || 0x5 == aux)
			goto return_full;
		else
			goto return_half;
	} else {
		rc = phy_read_(phy, BC_AUX_ERROR_AND_GEN_STATUS_REG, &aux);

		if (rc)
			goto error;

		if (aux & 2)
			goto return_full;
		else
			goto return_half;
	}

error:
	ERROR_PRINT("phy_duplex_() failed!\n");
	return -1;

return_half:
	return 0;

return_full:
	return 1;
}

/*
 * ----------------------------------------------------------------------
 * phy_renegotiate_
 */

static int
phy_renegotiate_(int phy)
{
	phy_control_t control;
	phy_status_t status;
	int autoneg_retries = 4;
	int autoneg_complete_retries = 20;

	printk(KERN_INFO "Initiating Auto Negotiation");
	phy_write_(phy, PHY_AUTONEG_ADVERTISE, 0x61);
#if defined(PHY_DEBUG)
	/*Debug Code */
	phy_read_(phy, PHY_STATUS, &status.raw);
	PHY_DEBUG_PRINT("PHY Status raw: 0x%x\n Listing all regs:\n",
			status.raw);

	{
		unsigned short regI;
		int count;
		for (count = 0x0; count <= 0x1F; count++) {
			phy_read_(phy, count, &regI);
			PHY_DEBUG_PRINT("Register: 0x%x reads: 0x%x\n",
					count, regI);
			}
	}
	/* End Debug code*/
#endif /*End PHY_DEBUG */
	do {
		phy_read_(phy, PHY_CONTROL, &control.raw);
		PHY_DEBUG_PRINT("control.raw read to: 0x%x\n", control.raw);
		control.bits.restart_autoneg = 1;
		phy_write_(phy, PHY_CONTROL, control.raw);
		PHY_DEBUG_PRINT("control.raw Written to: 0x%x\n", control.raw);
		do {
			mdelay(500);
			phy_read_(phy, PHY_STATUS, &status.raw);
			PHY_DEBUG_PRINT(" PHY_STATUS raw: 0x%x\n", status.raw);
		} while ((0 < --autoneg_complete_retries) &&
			 (0 == status.bits.autoneg_comp));

		if (0 != status.bits.autoneg_comp)
			break;

		autoneg_complete_retries = 8;
		printk(KERN_INFO ".");
	} while (0 < --autoneg_retries);

	if (0 == status.bits.autoneg_comp || 0 == autoneg_retries) {
		printk(KERN_INFO "Auto Negotiation Failed\n");
		return -1;
	}

	printk(KERN_INFO "Auto Negotiation Succeeded\n");
	return 0;
}

/*
 * ----------------------------------------------------------------------
 * phy_enable_
 */

static int phy_enable_(struct net_device *device)
{
	appnic_device_t *apnd = netdev_priv(device);
	phy_id_high_t phy_id_high_;
	phy_id_low_t phy_id_low_;
	unsigned char phyaddr_string_[40];

	if (0 == phy_read_(apnd->phy_address, PHY_ID_HIGH, &phy_id_high_.raw)) {
		PHY_DEBUG_PRINT("Read PHY_ID_HIGH as 0x%x on mdio addr 0x%x.\n",
		phy_id_high_.raw, apnd->phy_address);
	}

	if (0 == phy_read_(apnd->phy_address, PHY_ID_LOW, &phy_id_low_.raw)) {
		PHY_DEBUG_PRINT("Read PHY_ID_LOW as 0x%x on mdio addr 0x%x.\n",
		phy_id_low_.raw, apnd->phy_address);
	}

	phy_renegotiate_(apnd->phy_address);

	return 0;
}

#endif /* PHYLESS */

/*
  ======================================================================
  ======================================================================
  ======================================================================

  Linux Network Driver Interface

  ======================================================================
  ======================================================================
  ======================================================================
*/

/*
 * ----------------------------------------------------------------------
 * handle_transmit_interrupt_
 */

static void handle_transmit_interrupt_(struct net_device *device)
{
	appnic_device_t *dev_ = netdev_priv(device);

	TRACE_BEGINNING();
	DEBUG_PRINT("tail=0x%lx tail_copy=0x%lx head=0x%lx\n",
		    dev_->tx_tail->raw, dev_->tx_tail_copy.raw,
		    dev_->tx_head.raw);

	/*
	 * The hardware's tail pointer should be one descriptor (or more)
	 * ahead of software's copy.
	 */

	while (0 < queue_initialized_(SWAB_QUEUE_POINTER(dev_->tx_tail),
	       dev_->tx_tail_copy, dev_->tx_num_desc)) {
		queue_increment_(&dev_->tx_tail_copy, dev_->tx_num_desc);
	}

	DEBUG_PRINT("tail=0x%lx tail_copy=0x%lx head=0x%lx\n",
		    dev_->tx_tail->raw, dev_->tx_tail_copy.raw,
		    dev_->tx_head.raw);
	TRACE_ENDING();

	return;
}

/* static DECLARE_MUTEX(rpm); */

/*
 * ----------------------------------------------------------------------
 * lsinet_rx_packet
 */

void
lsinet_rx_packet(struct net_device *device)
{
	appnic_device_t *adapter = netdev_priv(device);
	appnic_dma_descriptor_t descriptor;
	struct sk_buff *sk_buff_;
	unsigned bytes_copied_ = 0;
	unsigned error_ = 0;
	int return_code_;
	unsigned long ok_, overflow_, crc_, align_;

	/*
	 * TEMP HACK:
	 * should use down_interruptible
	 */
	spin_lock(&adapter->extra_lock);
	TRACE_BEGINNING();
	DEBUG_PRINT("head=0x%lx tail=0x%lx tail_copy=0x%lx\n",
	adapter->rx_head.raw, adapter->rx_tail->raw,
	adapter->rx_tail_copy.raw);
	readdescriptor(((unsigned long) adapter->rx_desc +
		       adapter->rx_tail_copy.bits.offset), &descriptor);

	sk_buff_ = dev_alloc_skb(1600);

	if ((struct sk_buff *) 0 == sk_buff_) {
		ERROR_PRINT("dev_alloc_skb() failed! Dropping packet.\n");
		TRACE_ENDING();
		spin_unlock(&adapter->extra_lock);
		return;
	}

	/*dump_registers(device);*/
	ok_ = read_mac_(APPNIC_RX_STAT_PACKET_OK);
	overflow_ = read_mac_(APPNIC_RX_STAT_OVERFLOW);
	crc_ = read_mac_(APPNIC_RX_STAT_CRC_ERROR);
	align_ = read_mac_(APPNIC_RX_STAT_ALIGN_ERROR);

	/*
	 * Copy the received packet into the skb.
	 */

	while (0 < queue_initialized_(SWAB_QUEUE_POINTER(adapter->rx_tail),
	       adapter->rx_tail_copy, adapter->rx_num_desc)) {

#ifdef PRELOAD_RX_BUFFERS
		{
			unsigned char *buffer_;
			buffer_ = skb_put(sk_buff_, descriptor.pdu_length);
			memcmp(buffer_, buffer_, descriptor.pdu_length);
			memcpy((void *) buffer_,
			       (void *) (descriptor.host_data_memory_pointer +
				 adapter->dma_alloc_offset),
			       descriptor.pdu_length);
		}
#else  /* PRELOAD_RX_BUFFERS */
		memcpy((void *) skb_put(sk_buff_,
					descriptor.pdu_length),
		       (void *) (descriptor.host_data_memory_pointer +
			adapter->dma_alloc_offset),
			descriptor.pdu_length);
#endif /* PRELOAD_RX_BUFFERS */
		bytes_copied_ += descriptor.pdu_length;
		descriptor.data_transfer_length = adapter->rx_buf_per_desc;
		writedescriptor(((unsigned long) adapter->rx_desc +
				adapter->rx_tail_copy.bits.offset),
				&descriptor);
		if (0 != descriptor.error)
			error_ = 1;
		queue_increment_(&adapter->rx_tail_copy,
				 adapter->rx_num_desc);
		if (0 != descriptor.end_of_packet)
			break;
		readdescriptor(((unsigned long) adapter->rx_desc +
				adapter->rx_tail_copy.bits.offset),
			       &descriptor);
	}

	if (0 == descriptor.end_of_packet) {
		ERROR_PRINT("No end of packet! %lu/%lu/%lu/%lu\n",
			    ok_, overflow_, crc_, align_);
		BUG();
		dev_kfree_skb(sk_buff_);

	} else {
		if (0 == error_) {
			struct ethhdr *ethhdr_ =
				(struct ethhdr *) sk_buff_->data;
			unsigned char broadcast_[] = { 0xff, 0xff, 0xff,
						       0xff, 0xff, 0xff };
			unsigned char multicast_[] = { 0x01, 0x00 };

			LSINET_COUNTS_INC(LSINET_COUNTS_RX_GOOD);

			if ((0 == memcmp((const void *) &(ethhdr_->h_dest[0]),
					(const void *) &(device->dev_addr[0]),
					sizeof(ethhdr_->h_dest))) ||
					 (0 == memcmp((const void *)
					  &(ethhdr_->h_dest[0]),
					(const void *) &(broadcast_[0]),
					sizeof(ethhdr_->h_dest))) ||
					(0 == memcmp((const void *)
					  &(ethhdr_->h_dest[0]),
					(const void *) &(multicast_[0]),
					sizeof(multicast_)))) {

				adapter->stats.rx_bytes += bytes_copied_;
				++adapter->stats.rx_packets;
				sk_buff_->dev = device;
				sk_buff_->protocol = eth_type_trans(sk_buff_,
								    device);
#ifdef LSINET_NAPI
				LSINET_COUNTS_INC(LSINET_COUNTS_RX_SENT);
				return_code_ = netif_receive_skb(sk_buff_);
#else
LSINET_COUNTS_INC(LSINET_COUNTS_RX_SENT);
				return_code_ = netif_rx(sk_buff_);
#endif

				if (NET_RX_DROP == return_code_) {
					++dropped_by_stack_;
					LSINET_COUNTS_INC
						(LSINET_COUNTS_RX_DRPD);
				}
			} else {
#if 0
				dump_packet("Dropped Packet",
					    sk_buff_->data,
					    sk_buff_->len); /* ZZZ */
#endif
				dev_kfree_skb(sk_buff_);
			}
		} else {
			LSINET_COUNTS_INC(LSINET_COUNTS_RX_ERR);

			dev_kfree_skb(sk_buff_);

			if (0 != overflow_)
				++adapter->stats.rx_fifo_errors;
			else if (0 != crc_)
				++adapter->stats.rx_crc_errors;
			else if (0 != align_)
				++adapter->stats.rx_frame_errors;
		}
	}

	DEBUG_PRINT("head=0x%lx tail=0x%lx tail_copy=0x%lx\n",
		    adapter->rx_head.raw, adapter->rx_tail->raw,
		    adapter->rx_tail_copy.raw);
	TRACE_ENDING();

	/* TEMP */
	spin_unlock(&adapter->extra_lock);

	/* that's all */
	return;
}

/*
 * ============================================================================
 * lsinet_rx_packets
 */

static int
lsinet_rx_packets(struct net_device *device, int max)
{
	appnic_device_t *adapter = netdev_priv(device);
	appnic_queue_pointer_t queue;
	int updated_head_pointer = 0;
	int packets = 0;

	queue.raw = adapter->rx_tail_copy.raw;

	/* Receive Packets */

	while (0 < queue_initialized_(SWAB_QUEUE_POINTER(adapter->rx_tail),
					queue, adapter->rx_num_desc)) {
		appnic_dma_descriptor_t descriptor;

		readdescriptor(((unsigned long) adapter->rx_desc +
				  queue.bits.offset),
				&descriptor);

		if (0 != descriptor.end_of_packet) {
			LSINET_COUNTS_INC(LSINET_COUNTS_RX_PKT);
			lsinet_rx_packet(device);
			++packets;
			queue.raw = adapter->rx_tail_copy.raw;

			if ((-1 != max) && (packets == max))
				break;
		} else {
			queue_increment_(&queue, adapter->rx_num_desc);
		}
	}

	/* Update the Head Pointer */

	while (1 < queue_uninitialized_(adapter->rx_head, adapter->rx_tail_copy,
					 adapter->rx_num_desc)) {

		appnic_dma_descriptor_t descriptor;

		readdescriptor(((unsigned long) adapter->rx_desc +
				  adapter->rx_head.bits.offset), &descriptor);
		descriptor.data_transfer_length = adapter->rx_buf_per_desc;
		descriptor.write = 1;
		descriptor.pdu_length = 0;
		descriptor.start_of_packet = 0;
		descriptor.end_of_packet = 0;
		descriptor.interrupt_on_completion = 1;
		writedescriptor(((unsigned long) adapter->rx_desc +
				   adapter->rx_head.bits.offset),
				 &descriptor);
		queue_increment_(&adapter->rx_head, adapter->rx_num_desc);
		updated_head_pointer = 1;
	}

	if (0 != updated_head_pointer)
		write_mac_(adapter->rx_head.raw, APPNIC_DMA_RX_HEAD_POINTER);

	return packets;
}

#ifdef LSINET_NAPI

/*
 * ============================================================================
 * lsinet_poll
 */

static int
lsinet_poll(struct napi_struct *napi, int budget)
{
	appnic_device_t *adapter = container_of(napi, appnic_device_t, napi);
	struct net_device *device = adapter->device;
	appnic_queue_pointer_t queue_;
	int status;

	int cur_budget = budget;
	int done;
	unsigned long dma_interrupt_status_;

	LSINET_COUNTS_INC(LSINET_COUNTS_POL_START);

	TRACE_BEGINNING();
	DEBUG_PRINT("head=0x%x tail=0x%x tail_copy=0x%x\n",
		    adapter->rx_head.raw, adapter->rx_tail->raw,
		    adapter->rx_tail_copy.raw);
	queue_.raw = adapter->rx_tail_copy.raw;

	done = 1;

	do {

		/* Acknowledge the RX interrupt. */
		write_mac_(~APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE,
			   APPNIC_DMA_INTERRUPT_STATUS);

		cur_budget -= lsinet_rx_packets(device, cur_budget);
		if (0 == cur_budget)
			break;

		dma_interrupt_status_ = read_mac_(APPNIC_DMA_INTERRUPT_STATUS);

	} while ((RX_INTERRUPT(dma_interrupt_status_)) && cur_budget);

	if (done) {
		LSINET_COUNTS_INC(LSINET_COUNTS_POL_RNBL);
		napi_complete(napi);
		/* re-enable receive interrupts */
		write_mac_((APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE |
			   APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT),
		APPNIC_DMA_INTERRUPT_ENABLE);
	}

	DEBUG_PRINT("head=0x%x tail=0x%x tail_copy=0x%x\n",
		    adapter->rx_head.raw, adapter->rx_tail->raw,
		    adapter->rx_tail_copy.raw);
	TRACE_ENDING();

	LSINET_COUNTS_INC(LSINET_COUNTS_POL_DONE);
	status = (done ? 0 : 1);
	return status;
}

#else /* Not using NAPI, define the Rx interrupt handler instead */

/*
 * ----------------------------------------------------------------------
 * handle_receive_interrupt_
 */

static void handle_receive_interrupt_(struct net_device *device)
{
	appnic_device_t *dev_ = netdev_priv(device);
	appnic_queue_pointer_t queue_;

	LSINET_COUNTS_INC(LSINET_COUNTS_RX_START);
	TRACE_BEGINNING();
	DEBUG_PRINT("head=0x%lx tail=0x%lx tail_copy=0x%lx\n",
		    dev_->rx_head.raw, dev_->rx_tail->raw,
		    dev_->rx_tail_copy.raw);
	queue_.raw = dev_->rx_tail_copy.raw;
	lsinet_rx_packets(device, -1);
	DEBUG_PRINT("head=0x%lx tail=0x%lx tail_copy=0x%lx\n",
		    dev_->rx_head.raw, dev_->rx_tail->raw,
		    dev_->rx_tail_copy.raw);
	TRACE_ENDING();
	LSINET_COUNTS_INC(LSINET_COUNTS_RX_DONE);

	return;
}
#endif /* else not NAPI */

/*
 * ----------------------------------------------------------------------
 * appnic_isr_
 */

static irqreturn_t appnic_isr_(int irq, void *device_id)
{
	struct net_device *device_ = (struct net_device *) device_id;
	appnic_device_t *dev_ = netdev_priv(device_);
	unsigned long dma_interrupt_status_;
	unsigned long flags;
	appnic_device_t *appnic_device = netdev_priv(device_);

	TRACE_BEGINNING();
	LSINET_COUNTS_INC(LSINET_COUNTS_ISR_START);

	/* Acquire the lock */
	spin_lock_irqsave(&dev_->lock, flags);

#if !defined(PHYLESS) && !defined(CONFIG_ACP)
	if (appnic_device->rx_interrupt == irq) {

		PHY_DEBUG_PRINT("Handling PHY interrupt.\n");

		if (BCM5221_PHY_ == phy_type_) {

			bc_phy_interrupt_t bc_phy_interrupt_;

			(void) phy_read_(appnic_device->phy_address,
					 BC_PHY_INTERRUPT_,
					 &bc_phy_interrupt_.raw);
			bc_phy_interrupt_.raw = 0;
			bc_phy_interrupt_.bits.enable = 1;
			(void) phy_write_(appnic_device->phy_address,
					  BC_PHY_INTERRUPT_,
					  bc_phy_interrupt_.raw);

		} else if (MICREL_PHY_ == phy_type_) {

			m_phy_interrupt_t m_phy_interrupt_;

			(void) phy_read_(appnic_device->phy_address,
					 M_PHY_INTERRUPT_,
					 &m_phy_interrupt_.raw);
			(void) phy_write_(appnic_device->phy_address,
					  M_PHY_INTERRUPT_,
					  m_phy_interrupt_.raw);

		}

		write_mac_(0, APPNIC_RX_INTERRUPT_STATUS);
		enable_(device_);

	} else {
#endif /* PHYLESS */

		/* get the status */
		dma_interrupt_status_ = read_mac_(APPNIC_DMA_INTERRUPT_STATUS);
#ifdef LSINET_NAPI
		/* NAPI - don't ack RX interrupt */
		write_mac_(APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE,
			   APPNIC_DMA_INTERRUPT_STATUS);
#else
		write_mac_(0, APPNIC_DMA_INTERRUPT_STATUS);
#endif

		/* Handle interrupts */

		if (TX_INTERRUPT(dma_interrupt_status_)) {
			LSINET_COUNTS_INC(LSINET_COUNTS_ISR_TX);

			/* transmition complete */
			++transmit_interrupts_;
			handle_transmit_interrupt_(device_);
		}

		if (RX_INTERRUPT(dma_interrupt_status_)) {
			LSINET_COUNTS_INC(LSINET_COUNTS_ISR_RX);

#ifdef LSINET_NAPI
			++receive_interrupts_;
			if (napi_schedule_prep(&dev_->napi)) {

				/*
				 * Disable RX interrupts and tell the
				 * system we've got work
				 */

				write_mac_(APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT,
					   APPNIC_DMA_INTERRUPT_ENABLE);
				__napi_schedule(&dev_->napi);
			} else {
				ERROR_PRINT("NAPI bug! Int while in poll\n");
				write_mac_(APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT,
					   APPNIC_DMA_INTERRUPT_ENABLE);
			}
#else
			/* Receive complete */
			handle_receive_interrupt_(device_);
#endif
		}
#if !defined(PHYLESS) && !defined(CONFIG_ACP)
	}
#endif /* PHYLESS */

	/* release the lock */
	spin_unlock_irqrestore(&dev_->lock, flags);

	LSINET_COUNTS_INC(LSINET_COUNTS_ISR_DONE);
	TRACE_ENDING();

	return IRQ_HANDLED;
}

/*
 * ----------------------------------------------------------------------
 * appnic_open
 *
 * Opens the interface.  The interface is opened whenever ifconfig
 * activates it.  The open method should register any system resource
 * it needs (I/O ports, IRQ, DMA, etc.) turn on the hardware, and
 * increment the module usage count.
 */

int appnic_open(struct net_device *device)
{
	int return_code_ = 0;
#if defined(CONFIG_ARCH_APP3)
	unsigned long gpio_mux_ = readl(APP3XX_HB_CONF_BASE + 0x18);
#endif

	/* enable the receiver and transmitter */
	if (0 != enable_(device)) {
		ERROR_PRINT("Unable to enable the interface.\n");
		disable_();
		return -EBUSY;
	}

#ifdef LSINET_NAPI
	{
		appnic_device_t *adapter = netdev_priv(device);
		napi_enable(&adapter->napi);
	}
#endif /* LSINET_NAPI */

	/* install the interrupt handlers */
	return_code_ = request_irq(device->irq, appnic_isr_, IRQF_DISABLED,
				   APPNIC_NAME, device);

	if (0 != return_code_) {
		ERROR_PRINT("request_irq() for %d failed, returned 0x%x/%d\n",
			    device->irq, return_code_, return_code_);
		return return_code_;
	}

	/* enable interrupts */
	write_mac_((APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE |
		   APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT),
		   APPNIC_DMA_INTERRUPT_ENABLE);

	{
#ifndef PHYLESS
		/*    char phy_string_ [256];  */
		/*    int use_interrupts_ = 1; */
		appnic_device_t *dev_ = netdev_priv(device);
#endif

#ifdef CONFIG_ARCH_APP3
		PHY_DEBUG_PRINT("phy_type_=0x%x gpio_mux_=0x%x\n",
				phy_type_, gpio_mux_);

		if ((BCM5221_PHY_ != phy_type_) ||
		    (0x04000000 != (gpio_mux_ & 0x04000000)))
			use_interrupts_ = 0;
#endif

#ifndef PHYLESS

#ifndef CONFIG_ACP
	WARN_PRINT("PHY is in polling mode.\n");
#endif
	init_timer(&appnic_timer_);
	appnic_timer_.expires = jiffies + (APPNIC_TIMER_PERIOD * HZ);
	appnic_timer_.data = (unsigned long) device;
	appnic_timer_.function = appnic_timer_handler_;
	add_timer(&appnic_timer_);
	dev_->polling = 1;

#endif /* PHYLESS */

	}

	/* let the OS know we are ready to send packets */
	netif_start_queue(device);

	/* that's all */
	return 0;
}

/*
 * ----------------------------------------------------------------------
 * appnic_stop
 *
 * Stops the interface.  The interface is stopped when it is brought
 * down; operations performed at open time should be reversed.
 */

int appnic_stop(struct net_device *device)
{
	int return_code_ = 0;

	DEBUG_PRINT("Stopping the interface.\n");

	/*
	 * Indicate to the OS that no more packets should be sent.
	 */

	netif_stop_queue(device);

	/*
	 * Stop the receiver and transmitter.
	 */

	disable_();

	/* Disable NAPI. */
#ifdef LSINET_NAPI
	{
		appnic_device_t *adapter = netdev_priv(device);
		napi_disable(&adapter->napi);
	}
#endif

	/*
	 * Free the interrupts.
	 */

	free_irq(device->irq, device);

#ifndef PHYLESS

	{
#ifndef CONFIG_ACP
		appnic_device_t *dev_ = netdev_priv(device);
		if (0 != dev_->polling)
			del_timer(&appnic_timer_);
		else
			free_irq(dev_->rx_interrupt, device);
#else
		del_timer(&appnic_timer_);
#endif
	}

#endif /* PHYLESS */

	/*
	 * That's all.
	 */

	return return_code_;

}

/*
 * ----------------------------------------------------------------------
 * appnic_hard_start_xmit
 *
 * The method initiates the transmission of a packet.  The full packet
 * (protocol headers and all) is contained in a socket buffer (sk_buff)
 * structure.
 *
 * ----- NOTES -----
 *
 * 1) This will not get called again by the kernel until it returns.
 */

int
appnic_hard_start_xmit(struct sk_buff *skb,
		       struct net_device *device)
{
	appnic_device_t *adapter = netdev_priv(device);
	int length_;
	int buf_per_desc_;

	LSINET_COUNTS_INC(LSINET_COUNTS_HST_START);
	length_ = skb->len < ETH_ZLEN ? ETH_ZLEN : skb->len;
	buf_per_desc_ = adapter->tx_buf_sz / adapter->tx_num_desc;

	/*dump_registers(device);*/
	/*dump_descriptors(device);*/
	/*dump_packet("TX Packet", (void *)skb->data, skb->len);*/

	/*
	 * If enough transmit descriptors are available, copy and transmit.
	 */

	while (((length_ / buf_per_desc_) + 1) >=
		queue_uninitialized_(adapter->tx_head,
				     SWAB_QUEUE_POINTER(adapter->tx_tail),
				     adapter->tx_num_desc)) {
		udelay(1000);
		LSINET_COUNTS_INC(LSINET_COUNTS_HST_RCLM);
		handle_transmit_interrupt_(device);
	}

	if (((length_ / buf_per_desc_) + 1) <
		queue_uninitialized_(adapter->tx_head,
				     SWAB_QUEUE_POINTER(adapter->tx_tail),
				     adapter->tx_num_desc)) {
		int bytes_copied_ = 0;
		appnic_dma_descriptor_t descriptor;

		LSINET_COUNTS_INC(LSINET_COUNTS_HST_SNDG);
		readdescriptor(((unsigned long) adapter->tx_desc +
			adapter->tx_head.bits.offset), &descriptor);
		descriptor.start_of_packet = 1;

		while (bytes_copied_ < length_) {
			descriptor.write = 1;
			descriptor.pdu_length = length_;

			if ((length_ - bytes_copied_) > buf_per_desc_) {
				memcpy((void *)
				       (descriptor.host_data_memory_pointer +
					adapter->dma_alloc_offset),
				       (void *) ((unsigned long) skb->data +
						 bytes_copied_),
				       buf_per_desc_);
				descriptor.data_transfer_length = buf_per_desc_;
				descriptor.end_of_packet = 0;
				descriptor.interrupt_on_completion = 0;
				bytes_copied_ += buf_per_desc_;
			} else {
				memcpy((void *)
				       (descriptor.host_data_memory_pointer +
					adapter->dma_alloc_offset),
				       (void *) ((unsigned long) skb->data +
						 bytes_copied_),
				       (length_ - bytes_copied_));
				descriptor.data_transfer_length =
				 (length_ - bytes_copied_);
				descriptor.end_of_packet = 1;
#ifdef DISABLE_TX_INTERRUPTS
				descriptor.interrupt_on_completion = 0;
#else  /* DISABLE_TX_INTERRUPTS */
				descriptor.interrupt_on_completion = 1;
#endif /* DISABLE_TX_INTERRUPTS */
				bytes_copied_ = length_;
			}

			adapter->stats.tx_bytes += bytes_copied_;
			writedescriptor(((unsigned long) adapter->tx_desc +
				adapter->tx_head.bits.offset), &descriptor);
			queue_increment_(&adapter->tx_head,
					 adapter->tx_num_desc);
			readdescriptor(((unsigned long) adapter->tx_desc +
				adapter->tx_head.bits.offset), &descriptor);
			descriptor.start_of_packet = 0;
		}

		write_mac_(adapter->tx_head.raw, APPNIC_DMA_TX_HEAD_POINTER);
		device->trans_start = jiffies;
		LSINET_COUNTS_INC(LSINET_COUNTS_HST_SNT);
	} else {
		++out_of_tx_descriptors_;
		LSINET_COUNTS_INC(LSINET_COUNTS_HST_OOD);
		ERROR_PRINT("No transmit descriptors available!\n");
	}

	/* Free the socket buffer */
	dev_kfree_skb(skb);

	LSINET_COUNTS_INC(LSINET_COUNTS_HST_DONE);
	return 0;
}

/*
 * ----------------------------------------------------------------------
 * appnic_net_device_stats
 *
 * Whenever an application needs to get statistics for the interface,
 * this method is called.  This happens, for example, when ifconfig or
 * nstat -i is run.
 */

struct net_device_stats *appnic_get_stats(struct net_device *device)
{
	appnic_device_t *device_ = netdev_priv(device);

	/*
	 * Update the statistics structure.
	 */

	get_hw_statistics_(device_);

	/*
	 * That's all.
	 */

	return &device_->stats;
}

/*
 * ----------------------------------------------------------------------
 * appnic_set_mac_address
 */

static int
appnic_set_mac_address(struct net_device *device, void *data)
{
	struct sockaddr *address_ = data;
	unsigned long swap_source_address_;

	if (netif_running(device))
		return -EBUSY;

	DEBUG_PRINT("Setting MAC to %02x:%02x:%02x:%02x:%02x:%02x\n",
		    address_->sa_data[0], address_->sa_data[1],
		    address_->sa_data[2], address_->sa_data[3],
		    address_->sa_data[4], address_->sa_data[5]);

	memcpy(device->dev_addr, address_->sa_data, 6);
	memcpy(device->perm_addr, address_->sa_data, 6);

	swap_source_address_ = ((address_->sa_data[4]) << 8) |
				address_->sa_data[5];
	write_mac_(swap_source_address_, APPNIC_SWAP_SOURCE_ADDRESS_2);
	swap_source_address_ = ((address_->sa_data[2]) << 8) |
				address_->sa_data[3];
	write_mac_(swap_source_address_, APPNIC_SWAP_SOURCE_ADDRESS_1);
	swap_source_address_ = ((address_->sa_data[0]) << 8) |
				address_->sa_data[1];
	write_mac_(swap_source_address_, APPNIC_SWAP_SOURCE_ADDRESS_0);
	memcpy(device->dev_addr, address_->sa_data, device->addr_len);

	return 0;
}

/*
  ======================================================================
  ======================================================================
  ======================================================================

  ETHTOOL Operations

  ======================================================================
  ======================================================================
  ======================================================================
*/

/*
 * ----------------------------------------------------------------------
 * appnic_get_settings
 */

static int
appnic_get_settings(struct net_device *device, struct ethtool_cmd *command)
{
	appnic_device_t *apnd = netdev_priv(device);

	memset(command, 0, sizeof(struct ethtool_cmd));

	/* What the hardware supports. */

	command->supported = (SUPPORTED_10baseT_Half |
			      SUPPORTED_10baseT_Full |
			      SUPPORTED_100baseT_Half |
			      SUPPORTED_100baseT_Full);

#ifndef PHYLESS

	/* What is currently advertised. */

	{
		unsigned short ad_value_;

		if (0 != phy_read_(apnd->phy_address,
				   PHY_AUTONEG_ADVERTISE,
				   &ad_value_)) {
			ERROR_PRINT("PHY read failed!");
			return -EIO;

		}

		switch (ad_value_) {
		case 0x1e1:
			command->advertising = (ADVERTISED_100baseT_Full |
						ADVERTISED_100baseT_Half |
						ADVERTISED_10baseT_Full |
						ADVERTISED_10baseT_Half);
			break;

		case 0xe1:
			command->advertising = (ADVERTISED_100baseT_Half |
						ADVERTISED_10baseT_Full |
						ADVERTISED_10baseT_Half);
			break;

		case 0x61:
			command->advertising = (ADVERTISED_10baseT_Full |
						ADVERTISED_10baseT_Half);
			break;

		case 0x41:
		default:
			command->advertising = (ADVERTISED_10baseT_Half);
			break;
		}
	}

	/* The current speed. */

	{
		int speed_;

		speed_ = phy_speed_(apnd->phy_address);
		if (-1 == speed_) {
			ERROR_PRINT("PHY read failed!");
			return -EIO;
		} else if (1 == speed_) {
			command->speed = SPEED_100;
		} else {
			command->speed = SPEED_10;
		}
	}

	/* Is the current link duplex? */

	{
		int duplex_;

		duplex_ = phy_duplex_(apnd->phy_address);
		if (-1 == duplex_) {
			ERROR_PRINT("PHY read failed!");
			return -EIO;
		} else if (1 == duplex_) {
			command->duplex = DUPLEX_FULL;
		} else {
			command->duplex = DUPLEX_HALF;
		}

	}

#endif /* PHYLESS */

	/* Is autoneg enabled? */
	command->autoneg = AUTONEG_ENABLE;

	/* Return success. */
	return 0;
}

/*
 * Fill in the struture...
 */

static const struct ethtool_ops appnic_ethtool_ops = {
	.get_settings = appnic_get_settings
};


/*
  ======================================================================
  ======================================================================
  ======================================================================

  Linux Module Interface.

  ======================================================================
  ======================================================================
  ======================================================================
*/

static const struct net_device_ops appnic_netdev_ops = {
	.ndo_open = appnic_open,
	.ndo_stop = appnic_stop,
	.ndo_get_stats = appnic_get_stats,
	.ndo_set_mac_address = appnic_set_mac_address,
	.ndo_start_xmit = appnic_hard_start_xmit,
};

/*
 * ----------------------------------------------------------------------
 * appnic_init
 */

int
appnic_init(struct net_device *device)
{
	appnic_device_t *adapter = netdev_priv(device);

	TRACE_BEGINNING();

	/*
	 * Reset the MAC
	 */

	write_mac_(0x80000000, APPNIC_DMA_PCI_CONTROL);

	/*
	 * Allocate memory and initialize the descriptors
	 */

	{
		void *dma_offset_;

		/*
		 * fixup num_[rt]x_desc
		 */

		if (0 != (rx_num_desc % DESCRIPTOR_GRANULARITY)) {
			WARN_PRINT("rx_num_desc was not a multiple of %d.\n",
			DESCRIPTOR_GRANULARITY);
			rx_num_desc +=
			 DESCRIPTOR_GRANULARITY -
			  (rx_num_desc % DESCRIPTOR_GRANULARITY);
		}

		adapter->rx_num_desc = rx_num_desc;

		if (0 != (tx_num_desc % DESCRIPTOR_GRANULARITY)) {
			WARN_PRINT("tx_num_desc was not a multiple of %d.\n",
			DESCRIPTOR_GRANULARITY);
			tx_num_desc +=
			 DESCRIPTOR_GRANULARITY -
			 (tx_num_desc % DESCRIPTOR_GRANULARITY);
		}

		adapter->tx_num_desc = tx_num_desc;

		DEBUG_PRINT("rx_num_desc=%d tx_num_desc=%d\n",
			    rx_num_desc, tx_num_desc);

		/*
		 * up [rt]x_buf_sz. Must be some multiple of 64 bytes
		 * per descriptor.
		 */

		if (0 != (rx_buf_sz %
			(BUFFER_ALIGNMENT * rx_num_desc))) {
			WARN_PRINT("rx_buf_sz was not a multiple of %d.\n",
			(BUFFER_ALIGNMENT * rx_num_desc));
			rx_buf_sz +=
			 (BUFFER_ALIGNMENT * rx_num_desc) -
			  (rx_buf_sz % (BUFFER_ALIGNMENT *
			   rx_num_desc));
		}

		adapter->rx_buf_sz = rx_buf_sz;

		if (0 != (tx_buf_sz % (BUFFER_ALIGNMENT * tx_num_desc))) {
			WARN_PRINT("tx_buf_sz was not a multiple of %d.\n",
				   (BUFFER_ALIGNMENT * tx_num_desc));
		tx_buf_sz += (BUFFER_ALIGNMENT * tx_num_desc) -
			(tx_buf_sz % (BUFFER_ALIGNMENT * tx_num_desc));
		}

		adapter->tx_buf_sz = tx_buf_sz;

		DEBUG_PRINT("rx_buf_sz=%d tx_buf_sz=%d\n",
			    rx_buf_sz, tx_buf_sz);

		/*
		 * Allocate dma-able memory
		 */

		adapter->dma_alloc_size =
		 /* The tail pointers (rx and tx) */
		 (sizeof(appnic_queue_pointer_t) * 2) +
		 /* The RX descriptor ring (and padding to allow
		  * 64 byte alignment) */
		 (sizeof(appnic_dma_descriptor_t) *
		  adapter->rx_num_desc) +
		 (DESCRIPTOR_GRANULARITY) +
		 /* The TX descriptor ring (and padding...) */
		 (sizeof(appnic_dma_descriptor_t) *
		  adapter->tx_num_desc) +
		 (DESCRIPTOR_GRANULARITY) +
		 /* The RX buffer (and padding...) */
		 (adapter->rx_buf_sz) + (BUFFER_ALIGNMENT) +
		 /* The TX buffer (and padding...) */
		 (adapter->tx_buf_sz) + (BUFFER_ALIGNMENT);

#ifdef DMA_CACHABLE

		adapter->dma_alloc =
		 (void *)kmalloc(adapter->dma_alloc_size, GFP_KERNEL);

		if ((void *)0 == adapter->dma_alloc) {
			ERROR_PRINT("Could not allocate %d bytes of DMA-able memory!\n",
				    adapter->dma_alloc_size);
			kfree(adapter);
			TRACE_ENDING();
			return -ENOMEM;
		}

		adapter->dma_alloc_dma = virt_to_phys(adapter->dma_alloc);

#else
		/*
		 * This needs to be set to something sane for
		 * dma_alloc_coherent()
		 */

#if defined(CONFIG_ARM)
		adapter->dma_alloc = (void *)
			dma_alloc_coherent(NULL,
					   adapter->dma_alloc_size,
					   &adapter->dma_alloc_dma,
					   GFP_KERNEL);
#else
		device->dev.archdata.dma_ops = &dma_direct_ops;

		adapter->dma_alloc = (void *)
			dma_alloc_coherent(&device->dev,
					   adapter->dma_alloc_size,
					   &adapter->dma_alloc_dma,
					   GFP_KERNEL);
#endif

		if ((void *) 0 == adapter->dma_alloc) {
			ERROR_PRINT("Could not allocate %d bytes of DMA-able memory!\n",
				    adapter->dma_alloc_size);
			kfree(adapter);
			TRACE_ENDING();
			return -ENOMEM;
		}
#endif

		adapter->dma_alloc_offset = (int) adapter->dma_alloc -
			(int) adapter->dma_alloc_dma;
		dma_offset_ = adapter->dma_alloc;
		DEBUG_PRINT("Allocated %d bytes at 0x%08lx(0x%08lx), offset=0x%x.\n",
			    adapter->dma_alloc_size,
			    (unsigned long) adapter->dma_alloc,
			    (unsigned long) adapter->dma_alloc_dma,
			    adapter->dma_alloc_offset);

		/*
		 * Initialize the tail pointers
		 */

		adapter->rx_tail = (appnic_queue_pointer_t *) dma_offset_;
		adapter->rx_tail_dma = (int) adapter->rx_tail -
			(int) adapter->dma_alloc_offset;
		printk("%s:%d - rx_tail=0x%08x rx_tail_dma=0x%08x\n",
		       __FILE__, __LINE__,
		       adapter->rx_tail, adapter->rx_tail_dma); /* ZZZ */
		dma_offset_ += sizeof(appnic_queue_pointer_t);
		memset((void *) adapter->rx_tail, 0,
		       sizeof(appnic_queue_pointer_t));
		DEBUG_PRINT("rx_tail=0x%08lx\n",
			    (unsigned long) adapter->rx_tail);

		adapter->tx_tail = (appnic_queue_pointer_t *) dma_offset_;
		adapter->tx_tail_dma = (int) adapter->tx_tail -
			(int) adapter->dma_alloc_offset;
		dma_offset_ += sizeof(appnic_queue_pointer_t);
		memset((void *) adapter->tx_tail, 0,
		       sizeof(appnic_queue_pointer_t));
		DEBUG_PRINT("tx_tail=0x%08lx\n",
			    (unsigned long) adapter->tx_tail);

		/*
		 * Initialize the descriptor pointers
		 */

		DEBUG_PRINT("rx_tail=0x%08lx\n",
			    (unsigned long) adapter->rx_tail);
		adapter->rx_desc = (appnic_dma_descriptor_t *)
			ALIGN64B(dma_offset_);
		DEBUG_PRINT("rx_desc=0x%08lx\n",
			    (unsigned long) adapter->rx_desc);
		adapter->rx_desc_dma = (int) adapter->rx_desc -
			(int) adapter->dma_alloc_offset;
		dma_offset_ += (sizeof(appnic_dma_descriptor_t) *
			adapter->rx_num_desc) + (DESCRIPTOR_GRANULARITY);
		memset((void *) adapter->rx_desc, 0,
		       (sizeof(appnic_dma_descriptor_t) *
			 adapter->rx_num_desc));

		DEBUG_PRINT("rx_tail=0x%08lx\n",
			    (unsigned long) adapter->rx_tail);
		adapter->tx_desc = (appnic_dma_descriptor_t *)
			ALIGN64B(dma_offset_);
		DEBUG_PRINT("tx_desc=0x%08lx\n",
			    (unsigned long) adapter->tx_desc);
		adapter->tx_desc_dma = (int) adapter->tx_desc -
			(int) adapter->dma_alloc_offset;
		dma_offset_ += (sizeof(appnic_dma_descriptor_t) *
			adapter->tx_num_desc) + (DESCRIPTOR_GRANULARITY);
		memset((void *) adapter->tx_desc, 0,
		       (sizeof(appnic_dma_descriptor_t) *
			adapter->tx_num_desc));

		/*
		 * Initialize the buffer pointers
		 */

		DEBUG_PRINT("rx_tail=0x%08lx\n",
			    (unsigned long) adapter->rx_tail);
		DEBUG_PRINT("Initializing the RX buffer pointers, dma_offset=0x%lx/0x%lx\n",
			    (unsigned long) dma_offset_,
			    (unsigned long) ALIGN64B(dma_offset_));
		adapter->rx_buf = (void *) ALIGN64B(dma_offset_);
		adapter->rx_buf_dma = (int) adapter->rx_buf -
			(int) adapter->dma_alloc_offset;
		adapter->rx_buf_per_desc =
			adapter->rx_buf_sz / adapter->rx_num_desc;

		dma_offset_ += (adapter->rx_buf_sz) +
			(BUFFER_ALIGNMENT);

		DEBUG_PRINT("rx_tail=0x%08lx\n",
			    (unsigned long) adapter->rx_tail);
		DEBUG_PRINT("Initializing the TX buffer pointers, dma_offset=0x%lx/0x%lx\n",
			    (unsigned long) dma_offset_,
			    (unsigned long) ALIGN64B(dma_offset_));
		adapter->tx_buf = (void *) ALIGN64B(dma_offset_);
		adapter->tx_buf_dma = (int) adapter->tx_buf -
			 (int) adapter->dma_alloc_offset;
		adapter->tx_buf_per_desc =
			 adapter->tx_buf_sz / adapter->tx_num_desc;
		dma_offset_ += (adapter->tx_buf_sz) + (BUFFER_ALIGNMENT);

		DEBUG_PRINT("rx_tail=0x%08lx\n",
			    (unsigned long) adapter->rx_tail);

		/*
		 * Initialize the descriptors
		 */

		{
			int index_;
			unsigned long buf_ =
			 (unsigned long) adapter->rx_buf_dma;
			appnic_dma_descriptor_t descriptor;

			DEBUG_PRINT("Initializing RX descriptors at 0x%lx\n",
				    buf_);

			for (index_ = 0;
			     index_ < adapter->rx_num_desc;
			     ++index_) {
				memset((void *) &descriptor, 0,
				       sizeof(appnic_dma_descriptor_t));
				descriptor.write = 1;
				descriptor.interrupt_on_completion = 1;
				descriptor.host_data_memory_pointer =
					buf_;
				descriptor.data_transfer_length =
					adapter->rx_buf_per_desc;

				writedescriptor(((unsigned long)
					adapter->rx_desc + (index_ *
					sizeof(appnic_dma_descriptor_t))),
					&descriptor);

				buf_ += adapter->rx_buf_per_desc;
			}

			buf_ = (unsigned long) adapter->tx_buf_dma;
			DEBUG_PRINT("Initializing RX descriptors at 0x%lx\n",
				    buf_);

			for (index_ = 0;
			     index_ < adapter->tx_num_desc;
			     ++index_) {
				memset((void *) &descriptor, 0,
				       sizeof(appnic_dma_descriptor_t));
				descriptor.write = 1;
				descriptor.interrupt_on_completion = 1;
				descriptor.host_data_memory_pointer =
					buf_;

				writedescriptor(((unsigned long)
					adapter->tx_desc + (index_ *
					 sizeof(appnic_dma_descriptor_t))),
					&descriptor);

				buf_ += adapter->tx_buf_per_desc;
			}
		}

		DEBUG_PRINT("rx_tail=0x%08lx\n",
			(unsigned long) adapter->rx_tail);
		DEBUG_PRINT("Initializing spinlocks and semaphores.\n");

		/*
		 * Initialize the spinlock.
		 */

		spin_lock_init(&adapter->lock);
		spin_lock_init(&adapter->extra_lock);

		/*
		 * TEMP: Initialize the semaphores
		 */
		mutex_init(&adapter->rx_sem);
		mutex_init(&adapter->tx_sem);
		mutex_init(&adapter->poll_sem);

	}

	/*
	 * Take MAC out of reset
	 */

	DEBUG_PRINT("rx_tail=0x%08lx\n",
		    (unsigned long) adapter->rx_tail);
	DEBUG_PRINT("Enabling the MAC");
	write_mac_(0x0, APPNIC_RX_SOFT_RESET);
	write_mac_(0x1, APPNIC_RX_MODE);
	write_mac_(0x0, APPNIC_TX_SOFT_RESET);
	write_mac_(0x1, APPNIC_TX_MODE);
	/*write_mac_(0x300a, APPNIC_TX_WATERMARK);*/
	write_mac_(0x7f007f, APPNIC_TX_WATERMARK);
	write_mac_(0x1, APPNIC_TX_HALF_DUPLEX_CONF);
	write_mac_(0xffff, APPNIC_TX_TIME_VALUE_CONF);
	write_mac_(0x1, APPNIC_TX_INTERRUPT_CONTROL);
	write_mac_(0x5275, APPNIC_TX_EXTENDED_CONF);
	write_mac_(0x1, APPNIC_RX_INTERNAL_INTERRUPT_CONTROL);
	write_mac_(0x1, APPNIC_RX_EXTERNAL_INTERRUPT_CONTROL);
	write_mac_(0x40010000, APPNIC_DMA_PCI_CONTROL);
	write_mac_(0x30000, APPNIC_DMA_CONTROL);
	writeio(0x280044, dma_base + 0x60);
	writeio(0xc0, dma_base + 0x64);

	/*
	 * Set the MAC address
	 */

	printk(KERN_WARNING
	       "MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
	       device->dev_addr[0], device->dev_addr[1], device->dev_addr[2],
	       device->dev_addr[3], device->dev_addr[4], device->dev_addr[5]);

	write_mac_(((device->dev_addr[4]) << 8) |
		    (device->dev_addr[5]),
		   APPNIC_SWAP_SOURCE_ADDRESS_2);
	write_mac_(((device->dev_addr[2]) << 8) |
		    (device->dev_addr[3]),
		   APPNIC_SWAP_SOURCE_ADDRESS_1);
	write_mac_(((device->dev_addr[0]) << 8) |
		    (device->dev_addr[1]),
		   APPNIC_SWAP_SOURCE_ADDRESS_0);

	DEBUG_PRINT("rx_tail=0x%08lx\n",
		    (unsigned long) adapter->rx_tail);

	/*
	 * Initialize the queue pointers.
	 */

	{
		/*
		 * Receiver
		 */

		memset((void *) &adapter->rx_tail_copy,
		       0, sizeof(appnic_queue_pointer_t));
		memset((void *) &adapter->rx_head,
		       0, sizeof(appnic_queue_pointer_t));

		write_mac_(adapter->rx_desc_dma,
			   APPNIC_DMA_RX_QUEUE_BASE_ADDRESS);
		write_mac_((adapter->rx_num_desc *
			    sizeof(appnic_dma_descriptor_t)) / 1024,
			   APPNIC_DMA_RX_QUEUE_SIZE);

		/*
		 * Indicate that all of the receive descriptors
		 * are ready
		 */

		adapter->rx_head.bits.offset =
			(adapter->rx_num_desc - 1) *
			 sizeof(appnic_dma_descriptor_t);
		write_mac_(adapter->rx_tail_dma,
			   APPNIC_DMA_RX_TAIL_POINTER_ADDRESS);

		/*
		 * N.B.
		 *
		 * The boot loader may have used the NIC.  If so, the
		 * tail pointer must be read and the head pointer (and
		 * local copy of the tail) based on it.
		*/

		DEBUG_PRINT("Initializing RX tail: adapter=0x%lx\n",
			    (unsigned long) adapter);
		DEBUG_PRINT("Initializing RX tail: adapter->rx_tail=0x%lx\n",
			    (unsigned long) adapter->rx_tail);
		DEBUG_PRINT("Initializing RX tail: adapter->rx_tail->raw=0x%lx\n",
			    (unsigned long) adapter->rx_tail->raw);
		adapter->rx_tail->raw =
		  read_mac_(APPNIC_DMA_RX_TAIL_POINTER_LOCAL_COPY);
		adapter->rx_tail_copy.raw = adapter->rx_tail->raw;
		adapter->rx_head.raw = adapter->rx_tail->raw;
		queue_decrement_(&adapter->rx_head, adapter->rx_num_desc);
		adapter->rx_head.bits.generation_bit =
		  (0 == adapter->rx_head.bits.generation_bit) ? 1 : 0;
		write_mac_(adapter->rx_head.raw,
			   APPNIC_DMA_RX_HEAD_POINTER);

		/*
		 * Transmitter
		 */

		memset((void *) &adapter->tx_tail_copy,
			0, sizeof(appnic_queue_pointer_t));
		memset((void *) &adapter->tx_head,
		       0, sizeof(appnic_queue_pointer_t));

		write_mac_(adapter->tx_desc_dma,
			   APPNIC_DMA_TX_QUEUE_BASE_ADDRESS);
		write_mac_((adapter->tx_num_desc *
			    sizeof(appnic_dma_descriptor_t)) / 1024,
			   APPNIC_DMA_TX_QUEUE_SIZE);
		DEBUG_PRINT("Writing 0x%lx to APPNIC_DMA_TX_TAIL_POINTER_ADDRESS\n",
			    (unsigned long) adapter->tx_tail_dma);
		write_mac_(adapter->tx_tail_dma,
			   APPNIC_DMA_TX_TAIL_POINTER_ADDRESS);

		/*
		 * N.B.
		 *
		 * The boot loader may have used the NIC.  If so, the
		 * tail pointer must be read and the head pointer (and
		 * local copy of the tail) based on it.
		 */

		DEBUG_PRINT("Initializing TX tail pointer at 0x%lx/0x%lx.\n",
			    (unsigned long) adapter->tx_tail,
			    (unsigned long) adapter->tx_tail->raw);
		adapter->tx_tail->raw =
		  read_mac_(APPNIC_DMA_TX_TAIL_POINTER_LOCAL_COPY);
		adapter->tx_tail_copy.raw = adapter->tx_tail->raw;
		adapter->tx_head.raw = adapter->tx_tail->raw;
		write_mac_(adapter->tx_head.raw,
			   APPNIC_DMA_TX_HEAD_POINTER);

	}

	/* Clear statistics */

	{
		appnic_device_t *device_ = netdev_priv(device);
		clear_statistics_(device_);
	}

	/* Initialize the PHY */

#ifndef PHYLESS
	if (0 != phy_enable_(device))
		WARN_PRINT("Failed to initialize the PHY!\n");
#endif /* PHYLESS */

	/* Fill in the net_device structure */

	DEBUG_PRINT("Filling in the device structure.\n");

	ether_setup(device);
#ifdef CONFIG_ACP
	device->irq = irq_create_mapping(NULL, 33);
	if (NO_IRQ == device) {
		ERROR_PRINT("irq_create_mapping() failed\n");
		return -EBUSY;
	}

	if (0 != irq_set_irq_type(device->irq, IRQ_TYPE_LEVEL_HIGH)) {
		ERROR_PRINT("set_irq_type() failed\n");
		return -EBUSY;
	}
#else
	device->irq = adapter->dma_interrupt;
#endif

	device->netdev_ops = &appnic_netdev_ops;

	SET_ETHTOOL_OPS(device, &appnic_ethtool_ops);
#ifdef LSINET_NAPI
	memset((void *) &adapter->napi, 0, sizeof(struct napi_struct));
	netif_napi_add(device, &adapter->napi,
		       lsinet_poll, LSINET_NAPI_WEIGHT);
	adapter->device = device;
#endif

	/* That's all */
	TRACE_ENDING();
	return 0;
}

/*
 * ----------------------------------------------------------------------
 * appnic_read_proc_
 */

static int
appnic_read_proc_(char *page, char **start, off_t offset,
		  int count, int *eof, void *data)
{
	int length_;

	length_ = sprintf(page, "-- appnic.c -- Profiling is disabled\n");

	/* That's all */
	return length_;
}

/*
 * ----------------------------------------------------------------------
 * lsinet_init
 */

int
lsinet_init(void)
{
	int rc = 0;
	struct net_device *device;
	struct device_node *np = NULL;
	const u32 *field;
	appnic_device_t *appnic_device;
	u64 value64;
	u64 dt_size;
	u32 value32;
	int length;

	TRACE_BEGINNING();

	/* Allocate space for the device. */

	device = alloc_etherdev(sizeof(appnic_device_t));
	if ((struct net_device *)0 == device) {
		ERROR_PRINT("Couldn't allocate net device.");
		rc = -ENOMEM;
		goto out;
	}


	this_net_device = device; /* For /proc/reads. */
	appnic_device = netdev_priv(device);

	/*
	 * Get the physical addresses, interrupt number, etc. from the
	 * device tree.  If no entry exists (older boot loader...) just
	 * use the pre-devicetree method.
	 */

	np = of_find_node_by_type(np, "network");

	while (np && !of_device_is_compatible(np, "acp-femac"))
		np = of_find_node_by_type(np, "network");

	if (!np)
		goto device_tree_failed;

#if defined(CONFIG_ARM)
	rx_base = of_iomap(np, 0);
	tx_base = of_iomap(np, 1);
	dma_base = of_iomap(np, 2);
#else
	field = of_get_property(np, "reg", NULL);

	if (!field)
		goto device_tree_failed;

	value64 = of_translate_address(np, field);	
	value32 = (u32)field[1];
	field += 2;
	rx_base = ioremap(value64, value32);
	netdev_dbg(device, "rx_base=0x%x/0x%llx\n", rx_base, value64);
	value64 = of_translate_address(np, field);
	value32 = (u32)field[1];
	field += 2;
	tx_base = ioremap(value64, value32);
	value64 = of_translate_address(np, field);
	value32 = (u32)field[1];
	field += 2;
	dma_base = ioremap(value64, value32);
#endif

	appnic_device->rx_base = (unsigned long)rx_base;
	appnic_device->tx_base = (unsigned long)tx_base;
	appnic_device->dma_base = (unsigned long)dma_base;

#if defined(CONFIG_ARM)
	appnic_device->tx_interrupt = irq_of_parse_and_map(np, 0);
	appnic_device->rx_interrupt = irq_of_parse_and_map(np, 1);
	appnic_device->dma_interrupt = irq_of_parse_and_map(np, 2);
#else
	field = of_get_property(np, "interrupts", NULL);

	if (!field)
		goto device_tree_failed;

	appnic_device->dma_interrupt = field[0];
#endif

	field = of_get_property(np, "mdio-clock", NULL);

	if (!field)
		goto device_tree_failed;

	appnic_device->mdio_clock = field[0];

	field = of_get_property(np, "phy-address", NULL);

	if (!field)
		goto device_tree_failed;

	appnic_device->phy_address = field[0];

	field = of_get_property(np, "ad-value", NULL);

	if (!field)
		goto device_tree_failed;

	appnic_device->ad_value = field[0];

	field = of_get_property(np, "mac-address", &length);

	if (!field || 6 != length) {
		goto device_tree_failed;
	} else {
		int i;
		u8 *value;

		value = (u8 *)field;

		for (i = 0; i < 6; ++i)
			appnic_device->mac_addr[i] = value[i];
	}

	memcpy(device->dev_addr, &appnic_device->mac_addr[0], 6);
	memcpy(device->perm_addr, &appnic_device->mac_addr[0], 6);

	goto device_tree_succeeded;

device_tree_failed:
	printk(KERN_WARNING "%s:%d - Unable to read the device tree!\n",
	       __FILE__, __LINE__);
	iounmap(rx_base);
	iounmap(tx_base);
	iounmap(dma_base);
	rc = -EINVAL;
	goto out;

device_tree_succeeded:

	/* Initialize the device. */

	rc = appnic_init(device);
	if (0 != rc) {
		ERROR_PRINT("appnic_init() failed: %d\n", rc);
		rc = -ENODEV;
		goto out;
	}

	strcpy(this_net_device->name, "eth%d");

	/* Register the device. */
	rc = register_netdev(this_net_device);
	if (0 != rc) {
		ERROR_PRINT("register_netdev() failed: %d\n", rc);

		rc = -ENODEV;
		goto out;
	}

	/* Create the /proc entry. */
	create_proc_read_entry("driver/appnic", 0, NULL,
				appnic_read_proc_, NULL);

out:

	TRACE_ENDING();
	return rc;
}

module_init(lsinet_init);

/*
 * ----------------------------------------------------------------------
 * lsinet_exit
 */

void __exit
lsinet_exit(void)
{
	TRACE_BEGINNING();
	remove_proc_entry("driver/appnic", NULL);
	unregister_netdev(this_net_device);
	TRACE_ENDING();

	return;
}

module_exit(lsinet_exit);


#else

/*
 * drivers/net/ethernet/lsi/lsi_acp_net.c
 *
 * Copyright (C) 2013 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 *
 * NOTES:
 *
 * 1) This driver parses the DTB for driver specific settings. A few of
 *    them can be overriden by setting environment variables in U-boot:
 *
 *    ethaddr - MAC address of interface, in xx:xx:xx:xx:xx:xx format
 *
 *    ad_value - PHY advertise value. Can be set to one of these or multiple
 *               can be OR'ed together. If not set, the driver defaults to
 *               the OR'ed quantity of all four (0x1e1).
 *
 *               0x101 - 100/Full
 *               0x81  - 100/Half
 *               0x41  - 10/Full
 *               0x21  - 10/Half
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/in.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>

#include <linux/uaccess.h>
#include <linux/io.h>
#include <asm/dma.h>

#include <asm/lsi/acp_ncr.h>
#include "lsi_acp_net.h"

#define LSI_DRV_NAME           "acp-femac"
#define LSI_MDIO_NAME          "acp-femac-mdio"
#define LSI_DRV_VERSION        "2013-04-30"

MODULE_AUTHOR("John Jacques");
MODULE_DESCRIPTION("LSI ACP-FEMAC Ethernet driver");
MODULE_LICENSE("GPL");

/* Base Addresses of the RX, TX, and DMA Registers. */
static void *rx_base;
static void *tx_base;
static void *dma_base;

/*
 * ----------------------------------------------------------------------
 * appnic_mii_read
 *
 * Returns -EBUSY if unsuccessful, the (short) value otherwise.
 */

static int appnic_mii_read(struct mii_bus *bus, int phy, int reg)
{
	unsigned short value;

	/* Always returns success, so no need to check return status. */
	acp_mdio_read(phy, reg, &value);

	return (int)value;
}

/*
 * ----------------------------------------------------------------------
 * appnic_mii_write
 */

static int appnic_mii_write(struct mii_bus *bus, int phy, int reg, u16 val)
{
	return acp_mdio_write(phy, reg, val);
}

/*
 * ----------------------------------------------------------------------
 * appnic_handle_link_change
 *
 * Called periodically when PHY is in polling mode.
 */

static void appnic_handle_link_change(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	struct phy_device *phydev = pdata->phy_dev;
	int status_change = 0;
	unsigned long rx_configuration;
	unsigned long tx_configuration = 0;

	rx_configuration =
		(APPNIC_RX_CONF_STRIPCRC |
		 APPNIC_RX_CONF_RXFCE |
		 APPNIC_RX_CONF_TXFCE);
	tx_configuration =
		(APPNIC_TX_CONF_ENABLE_SWAP_SA |
		 APPNIC_TX_CONF_APP_CRC_ENABLE |
		 APPNIC_TX_CONF_PAD_ENABLE);

	TX_CONF_SET_IFG(tx_configuration, 0xf);

	if (phydev->link) {
		if ((pdata->speed != phydev->speed) ||
		    (pdata->duplex != phydev->duplex)) {

			if (phydev->duplex) {
				rx_configuration |= APPNIC_RX_CONF_DUPLEX;
				tx_configuration |= APPNIC_TX_CONF_DUPLEX;
			}
			if (phydev->speed == SPEED_100) {
				rx_configuration |= APPNIC_RX_CONF_SPEED;
				tx_configuration |= APPNIC_TX_CONF_SPEED;
			}

			rx_configuration |= (APPNIC_RX_CONF_ENABLE |
					     APPNIC_RX_CONF_LINK);
			tx_configuration |= (APPNIC_TX_CONF_LINK |
					     APPNIC_TX_CONF_ENABLE);

			pdata->speed = phydev->speed;
			pdata->duplex = phydev->duplex;
			status_change = 1;
		}
	}
	if (phydev->link != pdata->link) {
		if (!phydev->link) {
			pdata->speed = 0;
			pdata->duplex = -1;
		}
		pdata->link = phydev->link;
		status_change = 1;
	}

	if (status_change) {
		if (phydev->link) {
			netif_carrier_on(dev);
			netdev_info(dev, "link up (%d/%s)\n",
				    phydev->speed,
				    phydev->duplex == DUPLEX_FULL ?
				    "Full" : "Half");
		} else {
			netif_carrier_off(dev);
			netdev_info(dev, "link down\n");
		}

		if (rx_configuration != read_mac(APPNIC_RX_CONF))
			write_mac(rx_configuration, APPNIC_RX_CONF);

		if (tx_configuration != read_mac(APPNIC_TX_CONF))
			write_mac(tx_configuration, APPNIC_TX_CONF);
	}

	return;
}

/*
 * ----------------------------------------------------------------------
 * appnic_mii_probe
 */

static int appnic_mii_probe(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	struct phy_device *phydev = NULL;
	int ret;

	/* Find the first phy */
	phydev = phy_find_first(pdata->mii_bus);
	if (!phydev) {
		netdev_err(dev, " no PHY found\n");
		return -ENODEV;
	}

	ret = phy_connect_direct(dev, phydev,
				 &appnic_handle_link_change, 0,
				 PHY_INTERFACE_MODE_MII);

	if (ret) {
		netdev_err(dev, "Could not attach to PHY\n");
		return ret;
	}

	netdev_info(dev,
		    "attached PHY driver [%s] (mii_bus:phy_addr=%s, irq=%d)\n",
		    phydev->drv->name, dev_name(&phydev->dev), phydev->irq);

	/* Mask with MAC supported features */
	phydev->supported &= PHY_BASIC_FEATURES;
	phydev->advertising = mii_adv_to_ethtool_adv_t(pdata->ad_value);

	pdata->link = 0;
	pdata->speed = 0;
	pdata->duplex = -1;
	pdata->phy_dev = phydev;

	pr_info("%s: PHY initialized successfully", LSI_DRV_NAME);
	return 0;
}

/*
 * ----------------------------------------------------------------------
 * appnic_mii_init
 */

static int __devinit appnic_mii_init(struct platform_device *pdev,
				     struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	int i, err = -ENXIO;

	pdata->mii_bus = mdiobus_alloc();
	if (!pdata->mii_bus) {
		err = -ENOMEM;
		goto err_out_1;
	}

	pdata->mii_bus->name = LSI_MDIO_NAME;
	snprintf(pdata->mii_bus->id, MII_BUS_ID_SIZE, "%s-%x",
		 pdev->name, pdev->id);
	pdata->mii_bus->priv = pdata;
	pdata->mii_bus->read = appnic_mii_read;
	pdata->mii_bus->write = appnic_mii_write;
	pdata->mii_bus->irq = pdata->phy_irq;
	for (i = 0; i < PHY_MAX_ADDR; ++i)
		pdata->mii_bus->irq[i] = PHY_POLL;

	if (mdiobus_register(pdata->mii_bus)) {
		pr_warn("%s: Error registering mii bus", LSI_DRV_NAME);
		goto err_out_free_bus_2;
	}

	if (appnic_mii_probe(dev) < 0) {
		pr_warn("%s: Error registering mii bus", LSI_DRV_NAME);
		goto err_out_unregister_bus_3;
	}

	return 0;

err_out_unregister_bus_3:
	mdiobus_unregister(pdata->mii_bus);
err_out_free_bus_2:
	mdiobus_free(pdata->mii_bus);
err_out_1:
	return err;
}

/*
  ======================================================================
  NIC Interface
  ======================================================================
*/

#define DESCRIPTOR_GRANULARITY 64
#define BUFFER_ALIGNMENT 64

#define ALIGN64B(address) \
	((((unsigned long) (address) + (64UL - 1UL)) & ~(64UL - 1UL)))

#define ALIGN64B_OFFSET(address) \
	(ALIGN64B(address) - (unsigned long) (address))

/*
 *  ----- Note On Buffer Space -----
 *
 *  Minimum number of descriptors is 64 for the receiver and 64 for the
 *  transmitter; therefore, 2048 bytes (16 bytes each).
 *  This driver uses the following parameters, all of which may be set on
 *  the command line if this drivers is used as a module.
 *
 *  - rx_num_desc : Number of receive descriptors. This  must be a multiple
 *                  of 64.
 *  - tx_num_desc : Number of transmit descriptors. This must be a multiple
 *                  of 64.
 *
 *  The scheme used will be as follows:
 *
 *  - num_[rt]x_desc will be adjusted to be a multiple of 64 (if necessary).
 *  - An skb (with the data area 64 byte aligned) will be allocated for each rx
 *    descriptor.
 */

/*
 * Receiver
 */

int rx_num_desc = (CONFIG_LSI_NET_NUM_RX_DESC * DESCRIPTOR_GRANULARITY);
module_param(rx_num_desc, int, 0);
MODULE_PARM_DESC(rx_num_desc, "appnic : Number of receive descriptors");

int rx_buf_sz = CONFIG_LSI_NET_RX_BUF_SZ;
module_param(rx_buf_sz, int, 0);
MODULE_PARM_DESC(rx_buf_sz, "appnic : Receive buffer size");

/*
 * Transmitter
 */

int tx_num_desc = (CONFIG_LSI_NET_NUM_TX_DESC * DESCRIPTOR_GRANULARITY);
module_param(tx_num_desc, int, 0);
MODULE_PARM_DESC(tx_num_desc, "appnic : Number of receive descriptors");

int tx_buf_sz = CONFIG_LSI_NET_TX_BUF_SZ;
module_param(tx_buf_sz, int, 0);
MODULE_PARM_DESC(tx_buf_sz, "Appnic : Receive buffer size");

static unsigned long dropped_by_stack;
static unsigned long out_of_tx_descriptors;
static unsigned long transmit_interrupts;
static unsigned long receive_interrupts;

/*
  ======================================================================
  Utility Functions
  ======================================================================
*/

/*
  ----------------------------------------------------------------------
  clear_statistics
*/

static void clear_statistics(struct appnic_device *pdata)
{
	int waste;

	/*
	 * Clear memory.
	 */

	memset((void *) &(pdata->stats), 0, sizeof(struct net_device_stats));

	/*
	 * Clear counters.
	 */

	waste = read_mac(APPNIC_RX_STAT_PACKET_OK); /* rx_packets */
	waste = read_mac(APPNIC_TX_STAT_PACKET_OK); /* tx_packets */

	/* rx_bytes kept by driver. */
	/* tx_bytes kept by driver. */
	/* rx_errors will be the sum of the rx errors available. */
	/* tx_errors will be the sum of the tx errors available. */
	/* rx_dropped (unable to allocate skb) will be maintained by driver */
	/* tx_dropped (unable to allocate skb) will be maintained by driver */

	/* multicast */

	waste = read_mac(APPNIC_RX_STAT_MULTICAST);

	/* collisions will be the sum of the three following. */

	waste = read_mac(APPNIC_TX_STATUS_LATE_COLLISION);
	waste = read_mac(APPNIC_TX_STATUS_EXCESSIVE_COLLISION);
	waste = read_mac(APPNIC_TX_STAT_COLLISION_ABOVE_WATERMARK);

	/* rx_length_errors will be the sum of the two following. */

	waste = read_mac(APPNIC_RX_STAT_UNDERSIZE);
	waste = read_mac(APPNIC_RX_STAT_OVERSIZE);

	/* rx_over_errors (out of descriptors?) maintained by the driver. */
	/* rx_crc_errors */

	waste = read_mac(APPNIC_RX_STAT_CRC_ERROR);

	/* rx_frame_errors */

	waste = read_mac(APPNIC_RX_STAT_ALIGN_ERROR);

	/* rx_fifo_errors */

	waste = read_mac(APPNIC_RX_STAT_OVERFLOW);

	/* rx_missed will not be maintained. */
	/* tx_aborted_errors will be maintained by the driver. */
	/* tx_carrier_errors will not be maintained. */
	/* tx_fifo_errors */

	waste = read_mac(APPNIC_TX_STAT_UNDERRUN);

	/* tx_heartbeat_errors */
	/* tx_window_errors */

	/* rx_compressed will not be maintained. */
	/* tx_compressed will not be maintained. */

	/*
	 * That's all.
	 */

	return;
}

/*
 * ----------------------------------------------------------------------
 * get_hw_statistics
 *
 *  -- NOTES --
 *
 *  1) The hardware clears the statistics registers after a read.
 */

static void get_hw_statistics(struct appnic_device *pdata)
{
	unsigned long flags;

	/* tx_packets */

	pdata->stats.tx_packets += read_mac(APPNIC_TX_STAT_PACKET_OK);

	/* multicast */

	pdata->stats.multicast += read_mac(APPNIC_RX_STAT_MULTICAST);

	/* collision */

	pdata->stats.collisions += read_mac(APPNIC_TX_STATUS_LATE_COLLISION);
	pdata->stats.collisions +=
		read_mac(APPNIC_TX_STATUS_EXCESSIVE_COLLISION);
	pdata->stats.collisions +=
	read_mac(APPNIC_TX_STAT_COLLISION_ABOVE_WATERMARK);

	/* rx_length_errors */

	pdata->stats.rx_length_errors += read_mac(APPNIC_RX_STAT_UNDERSIZE);
	pdata->stats.rx_length_errors += read_mac(APPNIC_RX_STAT_OVERSIZE);

	/* tx_fifo_errors */

	pdata->stats.tx_fifo_errors += read_mac(APPNIC_TX_STAT_UNDERRUN);

	/*
	 * Lock this section out so the statistics maintained by the driver
	 * don't get clobbered.
	 */

	spin_lock_irqsave(&pdata->dev_lock, flags);

	pdata->stats.rx_errors +=
		(pdata->stats.rx_length_errors +
		 pdata->stats.rx_crc_errors +
		 pdata->stats.rx_frame_errors +
		 pdata->stats.rx_fifo_errors +
		 pdata->stats.rx_dropped +
		 pdata->stats.rx_over_errors);

	pdata->stats.rx_dropped = 0;
	pdata->stats.rx_over_errors = 0;

	pdata->stats.tx_errors += (pdata->stats.tx_fifo_errors +
				   pdata->stats.tx_aborted_errors);
	pdata->stats.tx_aborted_errors = 0;

	spin_unlock_irqrestore(&pdata->dev_lock, flags);

	/*
	 * That's all.
	 */

	return;
}

/*
 * ----------------------------------------------------------------------
 * queue_initialized
 *
 * Returns the number of descriptors that are ready to receive packets
 * or are waiting to transmit packets.  (from tail to head).
 */

static int queue_initialized(union appnic_queue_pointer head,
			     union appnic_queue_pointer tail,
			     int size)
{
	int initialized;

	/* Calculate the number of descriptors currently initialized. */
	if (head.bits.generation_bit == tail.bits.generation_bit) {
		/* same generation */
		initialized = (head.bits.offset - tail.bits.offset);
	} else {
		/* different generation */
		initialized = head.bits.offset +
			(size * sizeof(struct appnic_dma_descriptor) -
			 tail.bits.offset);
	}

	/* Number of descriptors is offset / sizeof(a descriptor). */
	initialized /= sizeof(struct appnic_dma_descriptor);

	return initialized;
}

/*
 * ----------------------------------------------------------------------
 * queue_uninitialzed
 *
 * Returns the number of unused/uninitialized descriptors. (from head to tail).
*/

static int queue_uninitialized(union appnic_queue_pointer head,
			       union appnic_queue_pointer tail,
			       int size)
{
	int allocated;

	/* Calculate the number of descriptors currently unused/uninitialized */
	if (head.bits.generation_bit == tail.bits.generation_bit)
		/* Same generation. */
		allocated = ((size * sizeof(struct appnic_dma_descriptor)) -
			 head.bits.offset) + tail.bits.offset;
	else
		/* Different generation. */
		allocated = tail.bits.offset - head.bits.offset;

	/* Number of descriptors is offset / sizeof(a descriptor). */
	allocated /= sizeof(struct appnic_dma_descriptor);

	/* That's all. */
	return allocated;
}

/*
 * ----------------------------------------------------------------------
 * queue_increment
 */

static void queue_increment(union appnic_queue_pointer *queue,
			    int number_of_descriptors)
{
	queue->bits.offset += sizeof(struct appnic_dma_descriptor);

	if ((number_of_descriptors * sizeof(struct appnic_dma_descriptor)) ==
		queue->bits.offset) {

		queue->bits.offset = 0;
		queue->bits.generation_bit =
			(0 == queue->bits.generation_bit) ? 1 : 0;
	}

	return;
}

/*
 * ----------------------------------------------------------------------
 * queue_decrement
 */

static void queue_decrement(union appnic_queue_pointer *queue,
			    int number_of_descriptors)
{
	if (0 == queue->bits.offset) {
		queue->bits.offset =
			((number_of_descriptors - 1) *
			 sizeof(struct appnic_dma_descriptor));
		queue->bits.generation_bit =
			(0 == queue->bits.generation_bit) ? 1 : 0;
	} else {
		queue->bits.offset -= sizeof(struct appnic_dma_descriptor);
	}

	return;
}

/*
 * ----------------------------------------------------------------------
 * disable_rx_tx
 */

static void disable_rx_tx(void)
{
	unsigned long tx_configuration;
	unsigned long rx_configuration;

	pr_info("%s: Disabling the interface.\n", LSI_DRV_NAME);

	rx_configuration = read_mac(APPNIC_RX_CONF);
	rx_configuration &= ~APPNIC_RX_CONF_ENABLE;
	write_mac(rx_configuration, APPNIC_RX_CONF);

	tx_configuration = read_mac(APPNIC_TX_CONF);
	tx_configuration &= ~APPNIC_TX_CONF_ENABLE;

	write_mac(tx_configuration, APPNIC_TX_CONF);

	/* That's all. */
	return;
}


/*
  ======================================================================
  Linux Network Driver Interface
  ======================================================================
*/

/*
 * ----------------------------------------------------------------------
 * handle_transmit_interrupt
 */

static void handle_transmit_interrupt(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);

	/*
	 * The hardware's tail pointer should be one descriptor (or more)
	 * ahead of software's copy.
	 */

	while (0 < queue_initialized(SWAB_QUEUE_POINTER(pdata->tx_tail),
				     pdata->tx_tail_copy, pdata->tx_num_desc)) {
		queue_increment(&pdata->tx_tail_copy, pdata->tx_num_desc);
	}

	return;
}

/*
 * ----------------------------------------------------------------------
 * lsinet_rx_packet
 */

static void lsinet_rx_packet(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	struct appnic_dma_descriptor descriptor;
	struct sk_buff *sk_buff;
	unsigned bytes_copied = 0;
	unsigned error_num = 0;
	unsigned long ok_stat, overflow_stat, crc_stat, align_stat;

	spin_lock(&pdata->extra_lock);

	readdescriptor(((unsigned long)pdata->rx_desc +
			pdata->rx_tail_copy.bits.offset), &descriptor);

	sk_buff = dev_alloc_skb(1600);

	if ((struct sk_buff *)0 == sk_buff) {
		pr_err("%s: dev_alloc_skb() failed! Dropping packet.\n",
		       LSI_DRV_NAME);
		spin_unlock(&pdata->extra_lock);
		return;
	}

	ok_stat = read_mac(APPNIC_RX_STAT_PACKET_OK);
	overflow_stat = read_mac(APPNIC_RX_STAT_OVERFLOW);
	crc_stat = read_mac(APPNIC_RX_STAT_CRC_ERROR);
	align_stat = read_mac(APPNIC_RX_STAT_ALIGN_ERROR);

	/*
	 * Copy the received packet into the skb.
	 */

	while (0 < queue_initialized(SWAB_QUEUE_POINTER(pdata->rx_tail),
				pdata->rx_tail_copy, pdata->rx_num_desc)) {

#ifdef CONFIG_PRELOAD_RX_BUFFERS
		{
			unsigned char *buffer;
			buffer = skb_put(sk_buff, descriptor.pdu_length);
			memcmp(buffer, buffer, descriptor.pdu_length);
			memcpy((void *)buffer,
			       (void *)(descriptor.host_data_memory_pointer +
				 pdata->dma_alloc_offset_rx),
			       descriptor.pdu_length);
		}
#else
		memcpy((void *)skb_put(sk_buff, descriptor.pdu_length),
		       (void *)(descriptor.host_data_memory_pointer +
				pdata->dma_alloc_offset_rx),
		       descriptor.pdu_length);
#endif
		bytes_copied += descriptor.pdu_length;
		descriptor.data_transfer_length = pdata->rx_buf_per_desc;
		writedescriptor(((unsigned long)pdata->rx_desc +
					pdata->rx_tail_copy.bits.offset),
				&descriptor);
		if (0 != descriptor.error)
			error_num = 1;
		queue_increment(&pdata->rx_tail_copy, pdata->rx_num_desc);
		if (0 != descriptor.end_of_packet)
			break;
		readdescriptor(((unsigned long)pdata->rx_desc +
					pdata->rx_tail_copy.bits.offset),
			       &descriptor);
	}

	if (0 == descriptor.end_of_packet) {
		pr_err("%s: No end of packet! %lu/%lu/%lu/%lu\n",
		       LSI_DRV_NAME, ok_stat, overflow_stat,
		       crc_stat, align_stat);
		BUG();
		dev_kfree_skb(sk_buff);

	} else {
		if (0 == error_num) {
			struct ethhdr *ethhdr =
				(struct ethhdr *) sk_buff->data;
			unsigned char broadcast[] = { 0xff, 0xff, 0xff,
						      0xff, 0xff, 0xff };
			unsigned char multicast[] = { 0x01, 0x00 };

			if ((0 == memcmp((const void *)&(ethhdr->h_dest[0]),
					 (const void *)&(dev->dev_addr[0]),
					 sizeof(ethhdr->h_dest))) ||
			    (0 == memcmp((const void *)&(ethhdr->h_dest[0]),
					 (const void *) &(broadcast[0]),
					 sizeof(ethhdr->h_dest))) ||
			    (0 == memcmp((const void *)&(ethhdr->h_dest[0]),
					 (const void *) &(multicast[0]),
					 sizeof(multicast)))) {

				pdata->stats.rx_bytes += bytes_copied;
				++pdata->stats.rx_packets;
				sk_buff->dev = dev;
				sk_buff->protocol = eth_type_trans(sk_buff,
								   dev);
				if (netif_receive_skb(sk_buff) == NET_RX_DROP)
					++dropped_by_stack;
			} else {
				dev_kfree_skb(sk_buff);
			}
		} else {
			dev_kfree_skb(sk_buff);

			if (0 != overflow_stat)
				++pdata->stats.rx_fifo_errors;
			else if (0 != crc_stat)
				++pdata->stats.rx_crc_errors;
			else if (0 != align_stat)
				++pdata->stats.rx_frame_errors;
		}
	}

	spin_unlock(&pdata->extra_lock);

	/* That's all. */
	return;
}

/*
 * ----------------------------------------------------------------------
 * lsinet_rx_packets
 */

static int lsinet_rx_packets(struct net_device *dev, int max)
{
	struct appnic_device *pdata = netdev_priv(dev);
	union appnic_queue_pointer queue;
	int updated_head_pointer = 0;
	int packets = 0;

	queue.raw = pdata->rx_tail_copy.raw;

	/* Receive Packets */

	while (0 < queue_initialized(SWAB_QUEUE_POINTER(pdata->rx_tail),
				     queue, pdata->rx_num_desc)) {
		struct appnic_dma_descriptor descriptor;

		readdescriptor(((unsigned long)pdata->rx_desc +
				  queue.bits.offset),
				&descriptor);

		if (0 != descriptor.end_of_packet) {
			lsinet_rx_packet(dev);
			++packets;
			queue.raw = pdata->rx_tail_copy.raw;

			if ((-1 != max) && (packets == max))
				break;
		} else {
			queue_increment(&queue, pdata->rx_num_desc);
		}
	}

	/* Update the Head Pointer */

	while (1 < queue_uninitialized(pdata->rx_head,
				       pdata->rx_tail_copy,
				       pdata->rx_num_desc)) {

		struct appnic_dma_descriptor descriptor;

		readdescriptor(((unsigned long)pdata->rx_desc +
				  pdata->rx_head.bits.offset), &descriptor);
		descriptor.data_transfer_length = pdata->rx_buf_per_desc;
		descriptor.write = 1;
		descriptor.pdu_length = 0;
		descriptor.start_of_packet = 0;
		descriptor.end_of_packet = 0;
		descriptor.interrupt_on_completion = 1;
		writedescriptor(((unsigned long)pdata->rx_desc +
				   pdata->rx_head.bits.offset),
				 &descriptor);
		queue_increment(&pdata->rx_head, pdata->rx_num_desc);
		updated_head_pointer = 1;
	}

	if (0 != updated_head_pointer)
		write_mac(pdata->rx_head.raw, APPNIC_DMA_RX_HEAD_POINTER);

	return packets;
}

/*
 * ----------------------------------------------------------------------
 * lsinet_poll
 */

static int lsinet_poll(struct napi_struct *napi, int budget)
{
	struct appnic_device *pdata =
		container_of(napi, struct appnic_device, napi);
	struct net_device *dev = pdata->device;
	union appnic_queue_pointer queue;

	int cur_budget = budget;
	unsigned long dma_interrupt_status;

	queue.raw = pdata->rx_tail_copy.raw;

	do {
		/* Acknowledge the RX interrupt. */
		write_mac(~APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE,
			   APPNIC_DMA_INTERRUPT_STATUS);

		cur_budget -= lsinet_rx_packets(dev, cur_budget);
		if (0 == cur_budget)
			break;

		dma_interrupt_status = read_mac(APPNIC_DMA_INTERRUPT_STATUS);

	} while ((RX_INTERRUPT(dma_interrupt_status)) && cur_budget);

	napi_complete(napi);

	/*
	 * Re-enable receive interrupts (and preserve
	 * the already enabled TX interrupt).
	 */
	write_mac((APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE |
		   APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT),
		  APPNIC_DMA_INTERRUPT_ENABLE);

	return 0;
}

/*
 * ----------------------------------------------------------------------
 * appnic_isr
 */

static irqreturn_t appnic_isr(int irq, void *device_id)
{
	struct net_device *dev = (struct net_device *)device_id;
	struct appnic_device *pdata = netdev_priv(dev);
	unsigned long dma_interrupt_status;
	unsigned long flags;

	/* Acquire the lock */
	spin_lock_irqsave(&pdata->dev_lock, flags);

	/* Get the status. */
	dma_interrupt_status = read_mac(APPNIC_DMA_INTERRUPT_STATUS);

	/* NAPI - don't ack RX interrupt */
	write_mac(APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE,
		  APPNIC_DMA_INTERRUPT_STATUS);

	/* Handle interrupts */
	if (TX_INTERRUPT(dma_interrupt_status)) {
		/* transmition complete */
		++transmit_interrupts;
		handle_transmit_interrupt(dev);
	}

	if (RX_INTERRUPT(dma_interrupt_status)) {
		++receive_interrupts;
		if (napi_schedule_prep(&pdata->napi)) {

			/*
			 * Disable RX interrupts and tell the
			 * system we've got work
			 */
			write_mac(APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT,
				  APPNIC_DMA_INTERRUPT_ENABLE);
			__napi_schedule(&pdata->napi);
		} else {
			write_mac(APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT,
				  APPNIC_DMA_INTERRUPT_ENABLE);
		}
	}

	/* Release the lock */
	spin_unlock_irqrestore(&pdata->dev_lock, flags);

	return IRQ_HANDLED;
}

#ifdef CONFIG_NET_POLL_CONTROLLER

/*
 * ----------------------------------------------------------------------
 * appnic_poll_controller
 *
 * Polling receive - used by netconsole and other diagnostic tools
 * to allow network i/o with interrupts disabled.
 */

static void appnic_poll_controller(struct net_device *dev)
{
	disable_irq(dev->irq);
	appnic_isr(dev->irq, dev);
	enable_irq(dev->irq);
}

#endif


/*
 * ----------------------------------------------------------------------
 * appnic_open
 *
 * Opens the interface.  The interface is opened whenever ifconfig
 * activates it.  The open method should register any system resource
 * it needs (I/O ports, IRQ, DMA, etc.) turn on the hardware, and
 * increment the module usage count.
 */

static int appnic_open(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	int return_code = 0;

	/* Bring the PHY up */
	phy_start(pdata->phy_dev);

	/* Enable NAPI */
	napi_enable(&pdata->napi);

	/* Install the interrupt handlers */
	return_code = request_irq(dev->irq, appnic_isr, IRQF_DISABLED,
				   LSI_DRV_NAME, dev);
	if (0 != return_code) {
		pr_err("%s: request_irq() failed, returned 0x%x/%d\n",
		       LSI_DRV_NAME, return_code, return_code);
		return return_code;
	}

	/* Enable interrupts */
	write_mac((APPNIC_DMA_INTERRUPT_ENABLE_RECEIVE |
		   APPNIC_DMA_INTERRUPT_ENABLE_TRANSMIT),
		   APPNIC_DMA_INTERRUPT_ENABLE);

	/* Let the OS know we are ready to send packets */
	netif_start_queue(dev);

	/* That's all */
	return 0;
}

/*
 * ----------------------------------------------------------------------
 * appnic_stop
 *
 * Stops the interface.  The interface is stopped when it is brought
 * down; operations performed at open time should be reversed.
 */

static int appnic_stop(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);

	pr_info("%s: Stopping the interface.\n", LSI_DRV_NAME);

	/* Disable all device interrupts */
	write_mac(0, APPNIC_DMA_INTERRUPT_ENABLE);
	free_irq(dev->irq, dev);

	/* Indicate to the OS that no more packets should be sent.  */
	netif_stop_queue(dev);
	napi_disable(&pdata->napi);

	/* Stop the receiver and transmitter. */
	disable_rx_tx();

	/* Bring the PHY down. */
	if (pdata->phy_dev)
		phy_stop(pdata->phy_dev);

	/* That's all. */
	return 0;
}

/*
 * ----------------------------------------------------------------------
 * appnic_hard_start_xmit
 *
 * The method initiates the transmission of a packet.  The full packet
 * (protocol headers and all) is contained in a socket buffer (sk_buff)
 * structure.
 *
 * ----- NOTES -----
 *
 * 1) This will not get called again by the kernel until it returns.
 */

static int appnic_hard_start_xmit(struct sk_buff *skb,
		       struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	int length;
	int buf_per_desc;

	length = skb->len < ETH_ZLEN ? ETH_ZLEN : skb->len;
	buf_per_desc = pdata->tx_buf_sz / pdata->tx_num_desc;

	/*
	 * If enough transmit descriptors are available, copy and transmit.
	 */

	while (((length / buf_per_desc) + 1) >=
		queue_uninitialized(pdata->tx_head,
				    SWAB_QUEUE_POINTER(pdata->tx_tail),
				    pdata->tx_num_desc)) {
		handle_transmit_interrupt(dev);
	}

	if (((length / buf_per_desc) + 1) <
		queue_uninitialized(pdata->tx_head,
				    SWAB_QUEUE_POINTER(pdata->tx_tail),
				    pdata->tx_num_desc)) {
		int bytes_copied = 0;
		struct appnic_dma_descriptor descriptor;

		readdescriptor(((unsigned long)pdata->tx_desc +
				pdata->tx_head.bits.offset), &descriptor);
		descriptor.start_of_packet = 1;

		while (bytes_copied < length) {
			descriptor.write = 1;
			descriptor.pdu_length = length;

			if ((length - bytes_copied) > buf_per_desc) {
				memcpy((void *)
					(descriptor.host_data_memory_pointer +
					 pdata->dma_alloc_offset_tx),
				       (void *) ((unsigned long) skb->data +
					bytes_copied),
					buf_per_desc);
				descriptor.data_transfer_length = buf_per_desc;
				descriptor.end_of_packet = 0;
				descriptor.interrupt_on_completion = 0;
				bytes_copied += buf_per_desc;
			} else {
				memcpy((void *)
					(descriptor.host_data_memory_pointer +
					 pdata->dma_alloc_offset_tx),
				       (void *) ((unsigned long) skb->data +
					bytes_copied),
					(length - bytes_copied));
				descriptor.data_transfer_length =
				 (length - bytes_copied);
				descriptor.end_of_packet = 1;
#ifdef CONFIG_DISABLE_TX_INTERRUPTS
				descriptor.interrupt_on_completion = 0;
#else
				descriptor.interrupt_on_completion = 1;
#endif
				bytes_copied = length;
			}

			pdata->stats.tx_bytes += bytes_copied;
			writedescriptor(((unsigned long) pdata->tx_desc +
				pdata->tx_head.bits.offset), &descriptor);
			queue_increment(&pdata->tx_head, pdata->tx_num_desc);
			readdescriptor(((unsigned long)pdata->tx_desc +
					 pdata->tx_head.bits.offset),
					&descriptor);
			descriptor.start_of_packet = 0;
		}

		write_mac(pdata->tx_head.raw, APPNIC_DMA_TX_HEAD_POINTER);
		dev->trans_start = jiffies;
	} else {
		++out_of_tx_descriptors;
		pr_err("%s: No transmit descriptors available!\n",
		       LSI_DRV_NAME);
	}

	/* Free the socket buffer */
	dev_kfree_skb(skb);

	return 0;
}

/*
 * ----------------------------------------------------------------------
 * appnic_net_device_stats
 *
 * Whenever an application needs to get statistics for the interface,
 * this method is called.  This happens, for example, when ifconfig or
 * nstat -i is run.
 */

static struct net_device_stats *appnic_get_stats(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);

	/*
	 * Update the statistics structure.
	 */

	get_hw_statistics(pdata);

	/*
	 * That's all.
	 */

	return &pdata->stats;
}

/*
 * ----------------------------------------------------------------------
 * appnic_set_mac_address
 */

static int appnic_set_mac_address(struct net_device *dev, void *data)
{
	struct sockaddr *address = data;
	unsigned long swap_source_address;

	if (netif_running(dev))
		return -EBUSY;

	memcpy(dev->dev_addr, address->sa_data, 6);
	memcpy(dev->perm_addr, address->sa_data, 6);

	swap_source_address = ((address->sa_data[4]) << 8) |
			       address->sa_data[5];
	write_mac(swap_source_address, APPNIC_SWAP_SOURCE_ADDRESS_2);
	swap_source_address = ((address->sa_data[2]) << 8) |
			address->sa_data[3];
	write_mac(swap_source_address, APPNIC_SWAP_SOURCE_ADDRESS_1);
	swap_source_address = ((address->sa_data[0]) << 8) |
			       address->sa_data[1];
	write_mac(swap_source_address, APPNIC_SWAP_SOURCE_ADDRESS_0);
	memcpy(dev->dev_addr, address->sa_data, dev->addr_len);

	return 0;
}

/*
  ======================================================================
  ETHTOOL Operations
  ======================================================================
*/

/*
 * ----------------------------------------------------------------------
 * appnic_get_drvinfo
 */

static void appnic_get_drvinfo(struct net_device *dev,
			       struct ethtool_drvinfo *info)
{
	strcpy(info->driver, LSI_DRV_NAME);
	strcpy(info->version, LSI_DRV_VERSION);
}

/*
 * ----------------------------------------------------------------------
 * appnic_get_settings
 */

static int appnic_get_settings(struct net_device *dev,
			       struct ethtool_cmd *cmd)
{
	struct appnic_device *pdata = netdev_priv(dev);
	struct phy_device *phydev = pdata->phy_dev;

	if (!phydev)
		return -ENODEV;

	return phy_ethtool_gset(phydev, cmd);
}

/*
 * Fill in the struture...
 */

static const struct ethtool_ops appnic_ethtool_ops = {
	.get_drvinfo = appnic_get_drvinfo,
	.get_settings = appnic_get_settings
};


/*
  ======================================================================
  Linux Module Interface.
  ======================================================================
*/

static const struct net_device_ops appnic_netdev_ops = {
	.ndo_open = appnic_open,
	.ndo_stop = appnic_stop,
	.ndo_get_stats = appnic_get_stats,
	.ndo_set_mac_address = appnic_set_mac_address,
	.ndo_start_xmit = appnic_hard_start_xmit,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = appnic_poll_controller,
#endif

};

/*
 * ----------------------------------------------------------------------
 * appnic_init
 */

int appnic_init(struct net_device *dev)
{
	struct appnic_device *pdata = netdev_priv(dev);
	void *dma_offset;
	int index;
	unsigned long buf;
	struct appnic_dma_descriptor descriptor;
	struct sockaddr address;

	/*
	 * Reset the MAC
	 */

	write_mac(0x80000000, APPNIC_DMA_PCI_CONTROL);

	/*
	 * Allocate memory and initialize the descriptors
	 */


	/*
	 * fixup num_[rt]x_desc
	 */

	if (0 != (rx_num_desc % DESCRIPTOR_GRANULARITY)) {
		pr_warn("%s: rx_num_desc was not a multiple of %d.\n",
			LSI_DRV_NAME, DESCRIPTOR_GRANULARITY);
		rx_num_desc += DESCRIPTOR_GRANULARITY -
				(rx_num_desc % DESCRIPTOR_GRANULARITY);
	}

	pdata->rx_num_desc = rx_num_desc;

	if (0 != (tx_num_desc % DESCRIPTOR_GRANULARITY)) {
		pr_warn("%s: tx_num_desc was not a multiple of %d.\n",
			LSI_DRV_NAME, DESCRIPTOR_GRANULARITY);
		tx_num_desc += DESCRIPTOR_GRANULARITY -
			(tx_num_desc % DESCRIPTOR_GRANULARITY);
	}

	pdata->tx_num_desc = tx_num_desc;

	/*
	 * up [rt]x_buf_sz. Must be some multiple of 64 bytes
	 * per descriptor.
	 */

	if (0 != (rx_buf_sz % (BUFFER_ALIGNMENT * rx_num_desc))) {
		pr_warn("%s: rx_buf_sz was not a multiple of %d.\n",
			LSI_DRV_NAME, (BUFFER_ALIGNMENT * rx_num_desc));
		rx_buf_sz += (BUFFER_ALIGNMENT * rx_num_desc) -
				(rx_buf_sz % (BUFFER_ALIGNMENT * rx_num_desc));
	}

	pdata->rx_buf_sz = rx_buf_sz;

	if (0 != (tx_buf_sz % (BUFFER_ALIGNMENT * tx_num_desc))) {
		pr_warn("%s: tx_buf_sz was not a multiple of %d.\n",
			LSI_DRV_NAME, (BUFFER_ALIGNMENT * tx_num_desc));
		tx_buf_sz += (BUFFER_ALIGNMENT * tx_num_desc) -
			(tx_buf_sz % (BUFFER_ALIGNMENT * tx_num_desc));
	}

	pdata->tx_buf_sz = tx_buf_sz;

	/*
	 * Allocate dma-able memory. Broken into smaller parts to keep
	 * from allocating a single large chunk of memory, but not too
	 * small since mappings obtained from dma_alloc_coherent() have
	 * a minimum size of one page.
	 */

	pdata->dma_alloc_size =
		/* The tail pointers (rx and tx) */
		(sizeof(union appnic_queue_pointer) * 2) +
		/* The RX descriptor ring (and padding to allow
		 * 64 byte alignment) */
		(sizeof(struct appnic_dma_descriptor) * pdata->rx_num_desc) +
		(DESCRIPTOR_GRANULARITY) +
		/* The TX descriptor ring (and padding...) */
		(sizeof(struct appnic_dma_descriptor) * pdata->tx_num_desc) +
		(DESCRIPTOR_GRANULARITY);

	pdata->dma_alloc_size_rx =
		/* The RX buffer (and padding...) */
		(pdata->rx_buf_sz) + (BUFFER_ALIGNMENT);

	pdata->dma_alloc_size_tx =
		/* The TX buffer (and padding...) */
		(pdata->tx_buf_sz) + (BUFFER_ALIGNMENT);


	/*
	 * This needs to be set to something sane for
	 * dma_alloc_coherent()
	 */

	dev->dev.archdata.dma_ops = &dma_direct_ops;

	pdata->dma_alloc = (void *)dma_alloc_coherent(&dev->dev,
						    pdata->dma_alloc_size,
						    &pdata->dma_alloc_dma,
						    GFP_KERNEL);

	if ((void *)0 == pdata->dma_alloc) {
		pr_err("%s: Could not allocate %d bytes of DMA-able memory!\n",
		       LSI_DRV_NAME, pdata->dma_alloc_size);
		kfree(pdata);
		return -ENOMEM;
	}

	pdata->dma_alloc_offset = (int)pdata->dma_alloc -
					(int)pdata->dma_alloc_dma;

	pdata->dma_alloc_rx = (void *)dma_alloc_coherent(&dev->dev,
						    pdata->dma_alloc_size_rx,
						    &pdata->dma_alloc_dma_rx,
						    GFP_KERNEL);

	if ((void *)0 == pdata->dma_alloc_rx) {
		pr_err("%s: Could not allocate %d bytes of DMA-able memory!\n",
		       LSI_DRV_NAME, pdata->dma_alloc_size_rx);
		dma_free_coherent(&dev->dev, pdata->dma_alloc_size,
				  pdata->dma_alloc, pdata->dma_alloc_dma);
		kfree(pdata);
		return -ENOMEM;
	}

	pdata->dma_alloc_offset_rx = (int)pdata->dma_alloc_rx -
					(int)pdata->dma_alloc_dma_rx;

	pdata->dma_alloc_tx = (void *)dma_alloc_coherent(&dev->dev,
						    pdata->dma_alloc_size_tx,
						    &pdata->dma_alloc_dma_tx,
						    GFP_KERNEL);

	if ((void *)0 == pdata->dma_alloc_tx) {
		pr_err("%s: Could not allocate %d bytes of DMA-able memory!\n",
		       LSI_DRV_NAME, pdata->dma_alloc_size_tx);
		dma_free_coherent(&dev->dev, pdata->dma_alloc_size,
				  pdata->dma_alloc, pdata->dma_alloc_dma);
		dma_free_coherent(&dev->dev, pdata->dma_alloc_size_rx,
				  pdata->dma_alloc_rx, pdata->dma_alloc_dma_rx);
		kfree(pdata);
		return -ENOMEM;
	}

	pdata->dma_alloc_offset_tx = (int)pdata->dma_alloc_tx -
					(int)pdata->dma_alloc_dma_tx;

	/*
	 * Initialize the tail pointers
	 */

	dma_offset = pdata->dma_alloc;

	pdata->rx_tail = (union appnic_queue_pointer *)dma_offset;
	pdata->rx_tail_dma = (int)pdata->rx_tail - (int)pdata->dma_alloc_offset;
	memset((void *)pdata->rx_tail, 0,
	       sizeof(union appnic_queue_pointer));

	dma_offset += sizeof(union appnic_queue_pointer);

	pdata->tx_tail = (union appnic_queue_pointer *)dma_offset;
	pdata->tx_tail_dma = (int)pdata->tx_tail - (int)pdata->dma_alloc_offset;
	memset((void *)pdata->tx_tail, 0, sizeof(union appnic_queue_pointer));

	dma_offset += sizeof(union appnic_queue_pointer);

	/*
	 * Initialize the descriptor pointers
	 */

	pdata->rx_desc = (struct appnic_dma_descriptor *)ALIGN64B(dma_offset);
	pdata->rx_desc_dma = (int)pdata->rx_desc - (int)pdata->dma_alloc_offset;
	memset((void *)pdata->rx_desc, 0,
	       (sizeof(struct appnic_dma_descriptor) * pdata->rx_num_desc));

	dma_offset += (sizeof(struct appnic_dma_descriptor) *
			pdata->rx_num_desc) + (DESCRIPTOR_GRANULARITY);

	pdata->tx_desc = (struct appnic_dma_descriptor *)ALIGN64B(dma_offset);
	pdata->tx_desc_dma = (int)pdata->tx_desc - (int)pdata->dma_alloc_offset;
	memset((void *)pdata->tx_desc, 0,
	       (sizeof(struct appnic_dma_descriptor) * pdata->tx_num_desc));

	/*
	 * Initialize the buffer pointers
	 */

	dma_offset = pdata->dma_alloc_rx;

	pdata->rx_buf = (void *)ALIGN64B(dma_offset);
	pdata->rx_buf_dma = (int)pdata->rx_buf -
				(int)pdata->dma_alloc_offset_rx;
	pdata->rx_buf_per_desc = pdata->rx_buf_sz / pdata->rx_num_desc;

	dma_offset = pdata->dma_alloc_tx;

	pdata->tx_buf = (void *)ALIGN64B(dma_offset);
	pdata->tx_buf_dma = (int)pdata->tx_buf -
				(int)pdata->dma_alloc_offset_tx;
	pdata->tx_buf_per_desc = pdata->tx_buf_sz / pdata->tx_num_desc;

	/*
	 * Initialize the descriptors
	 */

	buf = (unsigned long)pdata->rx_buf_dma;
	for (index = 0; index < pdata->rx_num_desc; ++index) {
		memset((void *) &descriptor, 0,
		       sizeof(struct appnic_dma_descriptor));
		descriptor.write = 1;
		descriptor.interrupt_on_completion = 1;
		descriptor.host_data_memory_pointer = buf;
		descriptor.data_transfer_length = pdata->rx_buf_per_desc;

		writedescriptor(((unsigned long)pdata->rx_desc + (index *
				sizeof(struct appnic_dma_descriptor))),
				&descriptor);

		buf += pdata->rx_buf_per_desc;
	}

	buf = (unsigned long)pdata->tx_buf_dma;

	for (index = 0; index < pdata->tx_num_desc; ++index) {
		memset((void *) &descriptor, 0,
		       sizeof(struct appnic_dma_descriptor));
		descriptor.write = 1;
		descriptor.interrupt_on_completion = 1;
		descriptor.host_data_memory_pointer = buf;

		writedescriptor(((unsigned long)pdata->tx_desc + (index *
				 sizeof(struct appnic_dma_descriptor))),
				&descriptor);

		buf += pdata->tx_buf_per_desc;
	}

	/*
	 * Initialize the spinlocks.
	 */

	spin_lock_init(&pdata->dev_lock);
	spin_lock_init(&pdata->extra_lock);

	/*
	 * Take MAC out of reset
	 */

	write_mac(0x0, APPNIC_RX_SOFT_RESET);
	write_mac(0x1, APPNIC_RX_MODE);
	write_mac(0x0, APPNIC_TX_SOFT_RESET);
	write_mac(0x1, APPNIC_TX_MODE);
	if (is_asic())
		write_mac(0x300a, APPNIC_TX_WATERMARK);
	else
		write_mac(0xc00096, APPNIC_TX_WATERMARK);
	write_mac(0x1, APPNIC_TX_HALF_DUPLEX_CONF);
	write_mac(0xffff, APPNIC_TX_TIME_VALUE_CONF);
	write_mac(0x1, APPNIC_TX_INTERRUPT_CONTROL);
	write_mac(0x5275, APPNIC_TX_EXTENDED_CONF);
	write_mac(0x1, APPNIC_RX_INTERNAL_INTERRUPT_CONTROL);
	write_mac(0x1, APPNIC_RX_EXTERNAL_INTERRUPT_CONTROL);
	write_mac(0x40010000, APPNIC_DMA_PCI_CONTROL);
	write_mac(0x30000, APPNIC_DMA_CONTROL);
	out_le32(dma_base + 0x60, 0x280044);
	out_le32(dma_base + 0x64, 0xc0);

	/*
	 * Set the MAC address
	 */

	memcpy(&(address.sa_data[0]), dev->dev_addr, 6);
	appnic_set_mac_address(dev, &address);

	/*
	 * Initialize the queue pointers.
	 */

	/*
	 * Receiver
	 */

	memset((void *)&pdata->rx_tail_copy, 0,
	       sizeof(union appnic_queue_pointer));
	memset((void *)&pdata->rx_head, 0,
	       sizeof(union appnic_queue_pointer));

	write_mac(pdata->rx_desc_dma, APPNIC_DMA_RX_QUEUE_BASE_ADDRESS);
	write_mac((pdata->rx_num_desc *
		   sizeof(struct appnic_dma_descriptor)) / 1024,
		  APPNIC_DMA_RX_QUEUE_SIZE);

	/*
	 * Indicate that all of the receive descriptors
	 * are ready
	 */

	pdata->rx_head.bits.offset = (pdata->rx_num_desc - 1) *
					sizeof(struct appnic_dma_descriptor);
	write_mac(pdata->rx_tail_dma, APPNIC_DMA_RX_TAIL_POINTER_ADDRESS);

	/*
	 * N.B.
	 *
	 * The boot loader may have used the NIC.  If so, the
	 * tail pointer must be read and the head pointer (and
	 * local copy of the tail) based on it.
	 */

	pdata->rx_tail->raw =
		  read_mac(APPNIC_DMA_RX_TAIL_POINTER_LOCAL_COPY);
	pdata->rx_tail_copy.raw = pdata->rx_tail->raw;
	pdata->rx_head.raw = pdata->rx_tail->raw;
	queue_decrement(&pdata->rx_head, pdata->rx_num_desc);
	pdata->rx_head.bits.generation_bit =
		  (0 == pdata->rx_head.bits.generation_bit) ? 1 : 0;
	write_mac(pdata->rx_head.raw, APPNIC_DMA_RX_HEAD_POINTER);

	/*
	 * Transmitter
	 */

	memset((void *) &pdata->tx_tail_copy, 0,
	       sizeof(union appnic_queue_pointer));
	memset((void *) &pdata->tx_head, 0,
	       sizeof(union appnic_queue_pointer));

	write_mac(pdata->tx_desc_dma, APPNIC_DMA_TX_QUEUE_BASE_ADDRESS);
	write_mac((pdata->tx_num_desc *
		   sizeof(struct appnic_dma_descriptor)) / 1024,
		  APPNIC_DMA_TX_QUEUE_SIZE);
	write_mac(pdata->tx_tail_dma, APPNIC_DMA_TX_TAIL_POINTER_ADDRESS);

	/*
	 * N.B.
	 *
	 * The boot loader may have used the NIC.  If so, the
	 * tail pointer must be read and the head pointer (and
	 * local copy of the tail) based on it.
	 */

	pdata->tx_tail->raw = read_mac(APPNIC_DMA_TX_TAIL_POINTER_LOCAL_COPY);
	pdata->tx_tail_copy.raw = pdata->tx_tail->raw;
	pdata->tx_head.raw = pdata->tx_tail->raw;
	write_mac(pdata->tx_head.raw, APPNIC_DMA_TX_HEAD_POINTER);

	/* Clear statistics */

	clear_statistics(pdata);

	/* Fill in the net_device structure */

	ether_setup(dev);
	dev->irq = irq_create_mapping(NULL, pdata->interrupt);
	if (NO_IRQ == dev->irq) {
		pr_err("%s: irq_create_mapping() failed\n", LSI_DRV_NAME);
		return -EBUSY;
	}

	if (0 != irq_set_irq_type(dev->irq, IRQ_TYPE_LEVEL_HIGH)) {
		pr_err("%s: set_irq_type() failed\n", LSI_DRV_NAME);
		return -EBUSY;
	}

	dev->netdev_ops = &appnic_netdev_ops;

	SET_ETHTOOL_OPS(dev, &appnic_ethtool_ops);
	memset((void *) &pdata->napi, 0, sizeof(struct napi_struct));
	netif_napi_add(dev, &pdata->napi,
		       lsinet_poll, LSINET_NAPI_WEIGHT);
	pdata->device = dev;

	/* That's all */
	return 0;
}

/*
 * ----------------------------------------------------------------------
 * appnic_read_proc
 */

static int
appnic_read_proc(char *page, char **start, off_t offset,
		 int count, int *eof, void *data)
{
	int length;

	length = sprintf(page, "-- appnic.c -- Profiling is disabled\n");

	/* That's all */
	return length;
}

/*
 * ----------------------------------------------------------------------
 * appnic_probe_config_dt
 */

#ifdef CONFIG_OF
static int __devinit appnic_probe_config_dt(struct net_device *dev,
					    struct device_node *np)
{
	struct appnic_device *pdata = netdev_priv(dev);
	const u32 *field;
	u64 value64;
	u32 value32;
	int length;

	if (!np)
		return -ENODEV;

	field = of_get_property(np, "enabled", NULL);

	if (!field || (field && (0 == *field)))
		return -EINVAL;

	field = of_get_property(np, "reg", NULL);

	if (!field) {
		pr_err("%s: Couldn't get \"reg\" property.", LSI_DRV_NAME);
		return -EINVAL;
	}

	value64 = of_translate_address(np, field);
	value32 = field[1];
	field += 2;
	rx_base = ioremap(value64, value32);
	pdata->rx_base = (unsigned long)rx_base;
	value64 = of_translate_address(np, field);
	value32 = field[1];
	field += 2;
	tx_base = ioremap(value64, value32);
	pdata->tx_base = (unsigned long)tx_base;
	value64 = of_translate_address(np, field);
	value32 = field[1];
	field += 2;
	dma_base = ioremap(value64, value32);
	pdata->dma_base = (unsigned long)dma_base;

	field = of_get_property(np, "interrupts", NULL);
	if (!field)
		goto device_tree_failed;
	else
		pdata->interrupt = field[0];

	field = of_get_property(np, "mdio-clock", NULL);
	if (!field)
		goto device_tree_failed;
	else
		pdata->mdio_clock = field[0];

	field = of_get_property(np, "phy-address", NULL);
	if (!field)
		goto device_tree_failed;
	else
		pdata->phy_address = field[0];

	field = of_get_property(np, "ad-value", NULL);
	if (!field)
		goto device_tree_failed;
	else
		pdata->ad_value = field[0];

	field = of_get_property(np, "mac-address", &length);
	if (!field || 6 != length) {
		goto device_tree_failed;
	} else {
		int i;
		u8 *value;

		value = (u8 *)field;

		for (i = 0; i < 6; ++i)
			pdata->mac_addr[i] = value[i];
	}

	memcpy(dev->dev_addr, &pdata->mac_addr[0], 6);
	memcpy(dev->perm_addr, &pdata->mac_addr[0], 6);

	return 0;

device_tree_failed:
	pr_err("%s: Reading Device Tree Failed\n", LSI_DRV_NAME);
	iounmap(rx_base);
	iounmap(tx_base);
	iounmap(dma_base);
	return -EINVAL;
}
#else
static inline int appnic_probe_config_dt(struct net_device *dev,
					 struct device_node *np)
{
	return -ENODEV;
}
#endif /* CONFIG_OF */

/*
 * ----------------------------------------------------------------------
 * appnic_drv_probe
 */

static int __devinit appnic_drv_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device_node *np = pdev->dev.of_node;
	struct net_device *dev;
	struct appnic_device *pdata;

	pr_info("%s: LSI(R) 10/100 Network Driver - version %s\n",
		LSI_DRV_NAME, LSI_DRV_VERSION);

	/* Allocate space for the device. */

	dev = alloc_etherdev(sizeof(struct appnic_device));
	if (!dev) {
		pr_err("%s: Couldn't allocate net device.\n", LSI_DRV_NAME);
		rc = -ENOMEM;
		goto out;
	}

	SET_NETDEV_DEV(dev, &pdev->dev);
	platform_set_drvdata(pdev, dev);

	pdata = netdev_priv(dev);

	/*
	 * Get the physical addresses, interrupt number, etc. from the
	 * device tree.  If no entry exists (older boot loader...) just
	 * use the pre-devicetree method.
	 */

	rc = appnic_probe_config_dt(dev, np);

	if (rc == -EINVAL) {
		goto out;
	} else if (rc == -ENODEV) {

#ifdef CONFIG_MTD_NAND_EP501X_UBOOTENV

		/*
		 * Attempt to get device settings from the DTB failed, so
		 * try to grab the ethernet MAC from the u-boot environment
		 * and use hard-coded values for device base addresses.
		 */

		unsigned char ethaddr_string[20];

		if (0 != ubootenv_get("ethaddr", ethaddr_string)) {
			pr_err("%s: Could not read ethernet address!\n",
			       LSI_DRV_NAME);
			return -EFAULT;
		} else {

			u8 mac_address[6];
			int i = 0;
			char *string = ethaddr_string;

			while ((0 != string) && (6 > i)) {
				char *value;
				unsigned long res;
				value = strsep(&string, ":");
				if (kstrtoul(value, 16, &res))
					return -EBUSY;
				mac_address[i++] = (u8)res;
			}

			memcpy(dev->dev_addr, mac_address, 6);
			memcpy(dev->perm_addr, mac_address, 6);
			dev->addr_len = 6;

			pr_info("%s: Using Static Addresses and Interrupts",
				LSI_DRV_NAME);
			rx_base = ioremap(0x002000480000ULL, 0x1000);
			pdata->rx_base =
			 (unsigned long)ioremap(0x002000480000ULL, 0x1000);
			tx_base = ioremap(0x002000481000ULL, 0x1000);
			pdata->tx_base =
			(unsigned long)ioremap(0x002000481000ULL, 0x1000);
			dma_base = ioremap(0x002000482000ULL, 0x1000);
			pdata->dma_base =
			 (unsigned long)ioremap(0x002000482000ULL, 0x1000);
			pdata->interrupt = 33;
		}
#else
		/* Neither dtb info nor ubootenv driver found. */
		pr_err("%s: Could not read ethernet address!", LSI_DRV_NAME);
		return -EBUSY;
#endif

	}

#ifdef CONFIG_MTD_NAND_EP501X_UBOOTENV

	{
		unsigned char uboot_env_string[20];

		/* Override ad_value with u-boot environment variable if set. */
		if (0 == ubootenv_get("ad_value", uboot_env_string)) {
			/*
			 * Assume ad_value is always entered as a hex value,
			 * since u-boot defaults this value as hex.
			 */
			unsigned long res;
			if (kstrtoul(uboot_env_string, 16, &res))
				return -EBUSY;
			pdata->ad_value = res;
		}
	}

#endif
	/* ad_value should never be 0. */
	if (pdata->ad_value == 0) {
		pdata->ad_value = 0x1e1;
		pr_err("%s: Set ad_value to default of 0x%lx\n",
		       LSI_DRV_NAME, pdata->ad_value);
	}

	/* Initialize the device. */
	rc = appnic_init(dev);
	if (0 != rc) {
		pr_err("%s: appnic_init() failed: %d\n", LSI_DRV_NAME, rc);
		rc = -ENODEV;
		goto out;
	}

	/* Register the device. */
	rc = register_netdev(dev);
	if (0 != rc) {
		pr_err("%s: register_netdev() failed: %d\n", LSI_DRV_NAME, rc);
		rc = -ENODEV;
		goto out;
	}

	/* Initialize the PHY. */
	rc = appnic_mii_init(pdev, dev);
	if (rc) {
		pr_warn("%s: Failed to initialize PHY", LSI_DRV_NAME);
		rc = -ENODEV;
		goto out;
	}

	/* Create the /proc entry. */
	create_proc_read_entry("driver/appnic", 0, NULL,
				appnic_read_proc, NULL);

out:
	return rc;
}

/*
 * ----------------------------------------------------------------------
 * appnic_drv_remove
 */

static int __devexit appnic_drv_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct appnic_device *pdata;

	pr_info("%s: Stopping driver", LSI_DRV_NAME);

	remove_proc_entry("driver/appnic", NULL);

	if (dev) {
		pdata = netdev_priv(dev);
		if (pdata->phy_dev)
			phy_disconnect(pdata->phy_dev);
		mdiobus_unregister(pdata->mii_bus);
		mdiobus_free(pdata->mii_bus);
		platform_set_drvdata(pdev, NULL);
		unregister_netdev(dev);
		free_irq(dev->irq, dev);
		dma_free_coherent(&dev->dev, pdata->dma_alloc_size,
				  pdata->dma_alloc, pdata->dma_alloc_dma);
		dma_free_coherent(&dev->dev, pdata->dma_alloc_size_rx,
				  pdata->dma_alloc_rx, pdata->dma_alloc_dma_rx);
		dma_free_coherent(&dev->dev, pdata->dma_alloc_size_tx,
				  pdata->dma_alloc_tx, pdata->dma_alloc_dma_tx);
		free_netdev(dev);
	}

	iounmap(rx_base);
	iounmap(tx_base);
	iounmap(dma_base);

	return 0;
}

static const struct of_device_id appnic_dt_ids[] = {
	{ .compatible = "acp-femac", }
};
MODULE_DEVICE_TABLE(of, appnic_dt_ids);

static struct platform_driver appnic_driver = {
	.probe = appnic_drv_probe,
	.remove = __devexit_p(appnic_drv_remove),
	.driver = {
		.name   = LSI_DRV_NAME,
		.owner  = THIS_MODULE,
		.pm     = NULL,
		.of_match_table = appnic_dt_ids,
	},
};

/* Entry point for loading the module */
static int __init appnic_init_module(void)
{
	return platform_driver_register(&appnic_driver);
}

/* Entry point for unloading the module */
static void __exit appnic_cleanup_module(void)
{
	platform_driver_unregister(&appnic_driver);
}

module_init(appnic_init_module);
module_exit(appnic_cleanup_module);

#endif
