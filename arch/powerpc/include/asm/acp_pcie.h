/*
 * PCI-Express hotplug support for LSI Axxia 3400 parts
 *
 */
#ifndef __ACP_PCIE_H__
#define __ACP_PCIE_H__

#include <linux/pci.h>

#if defined(CONFIG_ACP_PCIE_HOTPLUG) && defined(CONFIG_PPC4xx_PCI_EXPRESS)

extern int __init acp_pci_sysfs_init(struct pci_controller *hose);
extern int acp_pci_rescan_bus(int pei);
extern void acp_pci_remove_bus(int pei);

#else
static inline int acp_pci_sysfs_init(struct pci_controller *hose)
{
	return 0;
}
static inline int acp_pci_rescan_bus(int pei)
{
	return 0;
}
static inline void acp_pci_remove_bus(int pei)
{
}
#endif

#endif
