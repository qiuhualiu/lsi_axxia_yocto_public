/*
 *
 * hotplug support for PCIe
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/stat.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/delay.h>

#include <asm/lsi_ppc4xx_pcie.h>
#include <asm/acp_pcie.h>

#if defined(CONFIG_ACP_PCIE_HOTPLUG) && defined(CONFIG_PPC4xx_PCI_EXPRESS)

static DEFINE_MUTEX(pci_remove_rescan_mutex);

static int __port_rescan_buses(struct pci_bus *bus)
{
	struct pci_controller *hose = pci_bus_to_host(bus);
	int rc = -EFAULT;
	int loop;

	if (!hose)
		return -EINVAL;

	mutex_lock(&pci_remove_rescan_mutex);
	/* eager beaver... */
	for (loop = 0; loop<40; loop++) {
		rc = lsi_ppc4xx_pci_init_hw(hose);
		if (rc) {
			mutex_unlock(&pci_remove_rescan_mutex);
			return rc;
		}
		rc = lsi_ppc4xx_pci_activate(hose);
		if (rc == 0)
			break;
		msleep(20);
	}
	pr_info("link state %d after %d loops\n", rc, loop);
	if (rc) {
		mutex_unlock(&pci_remove_rescan_mutex);
		return rc;
	}
	hose->last_busno = 0xff;
	pci_rescan_bus(hose->bus);
	pcibios_finish_adding_to_bus(hose->bus);
	pr_info("__port_rescan_buses - DONE\n");
	mutex_unlock(&pci_remove_rescan_mutex);
	return rc;
}
static void __pcibios_remove_pci_devices(struct pci_bus *bus)
{
 	struct pci_dev *dev, *tmp;
	struct pci_bus *child_bus;

	/* First go down child busses */
	list_for_each_entry(child_bus, &bus->children, node)
		__pcibios_remove_pci_devices(child_bus);

	pr_info("PCI: Removing devices on bus %04x:%02x\n",
		pci_domain_nr(bus),  bus->number);
	list_for_each_entry_safe(dev, tmp, &bus->devices, bus_list) {
		pr_info("     * Removing %s...\n", pci_name(dev));
 		pci_stop_and_remove_bus_device(dev);
 	}
}
static void __port_remove_buses(struct pci_bus *bus)
{
	struct pci_controller *hose = pci_bus_to_host(bus);

	mutex_lock(&pci_remove_rescan_mutex);
	__pcibios_remove_pci_devices(bus);
	iosync();
	lsi_ppc4xx_pci_deactivate(hose);
	mutex_unlock(&pci_remove_rescan_mutex);
}


int acp_pci_rescan_bus(int pei)
{
	int rc = -EFAULT;
	struct pci_bus *bus = pci_find_bus(pei, 0);

	if (!bus)
		return rc;

	rc = __port_rescan_buses(bus);
	return rc;
}
EXPORT_SYMBOL(acp_pci_rescan_bus);
void acp_pci_remove_bus(int pei)
{
	struct pci_bus *bus = pci_find_bus(pei, 0);

	if (!bus)
		return;

	__port_remove_buses(bus);
}
EXPORT_SYMBOL(acp_pci_remove_bus);

static ssize_t bus_rescan_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct pci_bus *bus = to_pci_bus(dev);
	struct pci_controller *hose = pci_bus_to_host(bus);
	unsigned long val;
	int rc;

	if (strict_strtoul(buf, 0, &val) < 0)
		return -EINVAL;
	if (!hose)
		return -EINVAL;

	if (!val)
		return count;

	rc = __port_rescan_buses(bus);

	if (rc)
		return rc;
	return count;
}
static DEVICE_ATTR(rescan, S_IWUGO, NULL, bus_rescan_store);

static ssize_t bus_remove_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct pci_bus *bus = to_pci_bus(dev);
	struct pci_controller *hose = pci_bus_to_host(bus);
	unsigned long val;

	if (strict_strtoul(buf, 0, &val) < 0)
		return -EINVAL;
	if (!hose)
		return -EINVAL;

	if (!val)
		return count;

	__port_remove_buses(bus);

	return count;
}
static DEVICE_ATTR(remove, S_IWUGO, NULL, bus_remove_store);

static struct attribute *pciex_attributes[] = {
	&dev_attr_rescan.attr,
	&dev_attr_remove.attr,
	NULL
};

static struct attribute_group pciex_attribute_group = {
	.name = "hotplug",
	.attrs = pciex_attributes,
};

int __devinit acp_pci_sysfs_init(struct pci_controller *hose)
{
	int rc = 0;

	rc = sysfs_create_group(&hose->bus->dev.kobj, &pciex_attribute_group);
	return rc;
}
EXPORT_SYMBOL(acp_pci_sysfs_init);
#endif
