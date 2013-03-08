/*
 * PCI-Express support for LSI Axxia 3400 parts
 *
 */
#ifndef __LSI_PPC4XX_PCIE_H__
#define __LSI_PPC4XX_PCIE_H__

#ifdef CONFIG_ACP_PCIE_HOTPLUG
extern int lsi_ppc4xx_pci_init_hw(struct pci_controller *hose);
extern int lsi_ppc4xx_pci_activate(struct pci_controller *hose);
extern void lsi_ppc4xx_pci_deactivate(struct pci_controller *hose);
#else
static inline int lsi_ppc4xx_pci_init_hw(struct pci_controller *hose)
{
	return 0;
}
static inline int lsi_ppc4xx_pci_activate(struct pci_controller *hose)
{
	return 0;
}
static inline void lsi_ppc4xx_pci_deactivate(struct pci_controller *hose)
{
	return;
}
#endif
extern int __init lsi_ppc4xx_pci_sysfs_init(struct pci_controller *hose);

#endif
