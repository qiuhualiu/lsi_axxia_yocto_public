/*
 * PCI-Express support for LSI Axxia 3400 parts
 *
 */
#ifndef __LSI_PPC4XX_PCI_H__
#define __LSI_PPC4XX_PCI_H__

#include <asm/lsi_ppc4xx_pcie.h>

enum lsi_pei_irq_bits {
	PEI_SLAVE_WRITE_TO_IRQ,
	PEI_SLAVE_WRITE_CMP_IRQ,
	PEI_SLAVE_WRITE_ID_IRQ,
	PEI_MSG_DROP_IRQ,
	PEI_MSG_RX_IRQ,
	PEI_MREAD_4K_ERR_IRQ,
	PEI_SLAVE_READ_ERR_IRQ,
	PEI_SLAVE_READ_TO_IRQ,
	PEI_MWRITE_PKT_DROP_IRQ,
	PEI_MWRITE_BAR_IRQ,
	PEI_MREAD_PKT_DROP_IRQ,
	PEI_MREAD_BAR_IRQ,
	PEI_LINK_OR_TL_IRQ,
	PEI_T2A_EGR_ERR_IRQ,
	PEI_T2A_REC_IRQ,
	PEI_T2A_CPL_TO_IRQ,
	PEI_T2A_INGR_IRQ,
	PEI_T2A_INDP_ERR_IRQ,
	PEI_T2A_INDP_OTH_IRQ,
	PEI_T2A_PAR_IRQ,
	PEI_RESP_ERR_IRQ,
	PEI_FIFO_ERR_IRQ,
	PEI_AHB_BUSS_ERR_IRQ,
	PEI_MREAD_LENGTH_IRQ,
	PEI_ERR_MSG_RCVD_IRQ,
	PEI_PME_TO_ACK_IRQ,
	PEI_DEASSERT_INT_RCVD_IRQ,
	PEI_ASSER_INT_RCVD_IRQ,
	PEI_LINK_DOWN_IRQ,
	PEI_PME_OFF_IRQ,
	PEI_SYS_ERR_IRQ,
	PEI_MSI_RX_IRQ,
	PEI_IRQS
};

/*
 * LSI PCI Express bridge register definitions
 *
 * Internal cfg space root-complex mode:
 * offset 0: 256 bytes internal cfg space out of which
 * the first 64 bytes maps to the standardized
 * cfg space for header type 1 (PCI-to-PCI bridges)
 * use inlude/linux/pci_reg.h for this part
 *
 * hm.. adress space appear to be 0x8000 bytes for each PEI
 * but DS indicates only 0x4000 are actually used.
 */

#define PEI_INT_CFG_SIZE            0x4000
#define PEI_EXT_CFG_SIZE            0x100000

 /*
  * PCI Express bridge registers
  */
#define PCIE_EC_CAP_HDR		0x100
#define UNC_ERR_STATUS          0x104
#define UNC_ERR_MASK            0x108
#define UNC_ERR_SEVERITY        0x10c
#define CORR_ERR_STATUS         0x110
#define CORR_ERR_MASK           0x114
#define ERR_CAP_CTRL            0x118
#define HEADER_LOG1             0x11c
#define HEADER_LOG2             0x120
#define HEADER_LOG3             0x124
#define HEADER_LOG4             0x128
#define ROOT_ERR_STATUS         0x130
#define ERR_SRC_ID              0x134
#define VC_CH_CAP_HDR           0x140
#define PORT_VC_CAP1            0x144
#define PORT_VC_CAP2            0x148
#define PORT_VC_CTRL_STATUS     0x14c
#define VC_RES_CAP              0x150
#define VC_RES_CTRL             0x154
#define VC_RES_STATUS           0x158

/*
 * PEI configuration register definitions
 */
#define PEI_CONFIG              0x1000
#define  PEI_AXI_IF_RDY         0x40000
#define  PEI_RESET_EN           0x1
#define PEI_STATUS              0x1004
#define  PEI_LSTATE             0x3f00
#define   PEI_LSTATE_UP         0x0b00
#define  PEI_PIN_CFG            0x18
#define   PEI_RC                0x18
#define   PEI_EP                0x8
#define  PEI_RESET_PRG          0x1
#define PEI_CORE_DBG            0x1008
#define PEI_MPAGE_HIGH          0x1010
#define PEI_MPAGE_LOW           0x1014
/* 8 * 64bit registers */
#define PEI_MPAGE_UPPER(n) (PEI_MPAGE_HIGH + (n * 8))
#define PEI_MPAGE_LOWER(n) (PEI_MPAGE_LOW + (n * 8))

#define PEI_TPAGE_BAR0_BASE     0x1050
/* 8 * 32bit registers */
#define PEI_TPAGE_BAR0(n) (PEI_TPAGE_BAR0_BASE + (n * 4))
#define  PEI_AXI_SIZE_32        0x0
#define  PEI_AXI_SIZE_128       0x80000000
#define PEI_TPAGE_BAR1_BASE     0x1070
/* 8 * 32bit registers */
#define PEI_TPAGE_BAR1(n) (PEI_TPAGE_BAR1_BASE + (n * 4))

#define PEI_TPAGE_BAR2_BASE     0x1090
/* 8 * 32bit registers */
#define PEI_TPAGE_BAR2(n) (PEI_TPAGE_BAR2_BASE + (n * 4))

#define PEI_MSG_IN_FIFO         0x10b0
#define PEI_MSG_IN_FIFO_STATUS  0x10b4
#define PEI_MSG_OUT             0x10b8
#define PEI_IRQ_STATUS          0x10c0
#define  PEI_IRQ_T2A_IGR_ERR    0x00020000
#define PEI_IRQ_EN              0x10c4
#define PEI_IRQ_FRC             0x10c8
#define PEI_PHY_STA0            0x10cc
#define PEI_PHY_STA1            0x10d0
#define PEI_PHY_CTRL0           0x10d4
#define PEI_PHY_CTRL1           0x10d8
#define PEI_PHY_CTRL2           0x10dc

#define PEI_T2A_INDP_ERR        0x1170

/*
 * and many more, tbd...
 */
#endif /* __LSI_PPC4XX_PCI_H__ */
