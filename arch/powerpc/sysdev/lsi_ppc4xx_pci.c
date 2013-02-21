/*
 * PCI-Express support for LSI Axxia 3400 parts
 *
 * Based on earlier code:
 *
 * Mainly from ppc4xx_pci.c and LSI patches created by john.jacques@lsi.com:
 *
 *
 * PCI Express in LSI ACP chip is not IBM 4xx compliant.
 * e.g. DCR access to PCI devices are not supported
 * and in root-complex mode the internal and external configuration
 * space do not share a common base address.
 * The dtb is utilized as follows:
 * base address for internal configuration space is setup using
 * the utl_regs property
 * base address for external configuration space is setup using
 * the cfg_space property.
 * Internal cfg space root-complex mode:
 * offset 0: 256 bytes internal cfg space out of which
 * the first 64 bytes maps to the standardized
 * cfg space for header type 1 (PCI-to-PCI bridges)
 * see inlude/linux/pci_reg.h
 * offset 0x100 - 0x158: PCI Express Enhanced Cap. registers
 * see arch/powerpc/sysdev/lsi_ppc4xx_pci.h
 * offset 0x1000 - 0x7fff: PEI configuration registers
 * see arch/powerpc/sysdev/lsi_ppc4xx_pci.h
 */

#include <linux/export.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/bootmem.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <asm/io.h>
#include <asm/pci-bridge.h>
#include <asm/machdep.h>
#include <asm/dcr.h>
#include <asm/dcr-regs.h>
#include <mm/mmu_decl.h>

#include "lsi_ppc4xx_pci.h"

static int dma_offset_set;

#define U64_TO_U32_LOW(val)	((u32)((val) & 0x00000000ffffffffULL))
#define U64_TO_U32_HIGH(val)	((u32)((val) >> 32))

#define RES_TO_U32_LOW(val)	\
	((sizeof(resource_size_t) > sizeof(u32)) ? U64_TO_U32_LOW(val) : (val))
#define RES_TO_U32_HIGH(val)	\
	((sizeof(resource_size_t) > sizeof(u32)) ? U64_TO_U32_HIGH(val) : (0))

#ifdef CONFIG_ACP_PCIE_HOTPLUG
/**
 * Can't be __init mem, may be used after init phase if
 * hootplug is supported
 */
static void __devinit fixup_acp_pci_bridge(struct pci_dev *dev)
#else
static void __init fixup_acp_pci_bridge(struct pci_dev *dev)
#endif
{
	/* if we aren't a PCIe don't bother */
	if (!pci_find_capability(dev, PCI_CAP_ID_EXP))
		return;

	/*
	 * Set the class appropriately for a bridge device
	 */
	dev->class = PCI_CLASS_BRIDGE_HOST << 8;

	/*
	 * Make the bridge transparent
	 */
	dev->transparent = 1;

	return;
}
DECLARE_PCI_FIXUP_HEADER(0x1000, PCI_ANY_ID, fixup_acp_pci_bridge);

static int __init lsi_ppc4xx_parse_dma_ranges(struct pci_controller *hose,
					  void __iomem *reg,
					  struct resource *res)
{
	u64 size;
	const u32 *ranges;
	int rlen;
	int pna = of_n_addr_cells(hose->dn);
	int np = pna + 5;

	/* Default */
	res->start = 0;
	size = 0x80000000;
	res->end = size - 1;
	res->flags = IORESOURCE_MEM | IORESOURCE_PREFETCH;

	/* Get dma-ranges property */
	ranges = of_get_property(hose->dn, "dma-ranges", &rlen);
	if (ranges == NULL)
		goto out;

	/* Walk it */
	while ((rlen -= np * 4) >= 0) {
		u32 pci_space = ranges[0];
		u64 pci_addr = of_read_number(ranges + 1, 2);
		u64 cpu_addr = of_translate_dma_address(hose->dn, ranges + 3);
		size = of_read_number(ranges + pna + 3, 2);
		ranges += np;

		if (cpu_addr == OF_BAD_ADDR || size == 0)
			continue;

		/* We only care about memory */
		if ((pci_space & 0x03000000) != 0x02000000)
			continue;

		/* We currently only support memory at 0, and pci_addr
		 * within 32 bits space
		 */
		if (cpu_addr != 0 || pci_addr > 0xffffffff) {
			pr_debug("%s: Ignored unsupported dma range"
				 " 0x%016llx...0x%016llx -> 0x%016llx\n",
				 hose->dn->full_name,
				 pci_addr, pci_addr + size - 1, cpu_addr);
			continue;
		}

		/* Check if not prefetchable */
		if (!(pci_space & 0x40000000))
			res->flags &= ~IORESOURCE_PREFETCH;


		/* Use that */
		res->start = pci_addr;
		/* Beware of 32 bits resources */
		if (sizeof(resource_size_t) == sizeof(u32) &&
		    (pci_addr + size) > 0x100000000ull)
			res->end = 0xffffffff;
		else
			res->end = res->start + size - 1;
		break;
	}

	/* We only support one global DMA offset */
	if (dma_offset_set && pci_dram_offset != size) {
		pr_err("%s: dma-ranges(s) mismatch\n",
		       hose->dn->full_name);
		return -ENXIO;
	}

	/* Check that we can fit all of memory as we don't support
	 * DMA bounce buffers
	 */
	if (size < total_memory) {
		pr_err("%s: dma-ranges too small "
		       "(size=%llx total_memory=%llx)\n",
		       hose->dn->full_name, size, (u64)total_memory);
		return -ENXIO;
	}

	/* Check we are a power of 2 size and that base is a multiple of size*/
	if ((size & (size - 1)) != 0  ||
	    (res->start & (size - 1)) != 0) {
		pr_err("%s: dma-ranges unaligned\n",
		       hose->dn->full_name);
		return -ENXIO;
	}

	/* Check that we are fully contained within 32 bits space */
	if (res->end > 0xffffffff) {
		pr_err("%s: dma-ranges outside of 32 bits space\n",
		       hose->dn->full_name);
		return -ENXIO;
	}
 out:
	dma_offset_set = 1;
	/* In ACP DMA base is size of inbound window.
	 */
	pci_dram_offset = size;

	pr_debug("4xx PCI DMA offset set to 0x%08lx\n",
		 pci_dram_offset);
	return 0;
}

#ifdef CONFIG_PPC4xx_PCI_EXPRESS

/*
 * 4xx PCI-Express part
 *
 *
 */

#define MAX_PCIE_BUS_MAPPED	0x40

struct lsi_ppc4xx_stat {
	atomic_t t2a_igr_err;
	atomic_t t2a_igr_state;
};

struct lsi_ppc4xx_pciex_port
{
	struct pci_controller	*hose;
	struct device_node	*node;
	unsigned int		index;
	int			endpoint;
	int			link;
	int			has_ibpre;
	struct resource		cfg_space;
	struct resource		utl_regs;
	void __iomem		*utl_base;
	int                     irq_no;
	struct lsi_ppc4xx_stat  stat;
};

static struct lsi_ppc4xx_pciex_port *lsi_ppc4xx_pciex_ports;
static unsigned int lsi_ppc4xx_pciex_port_count;

struct lsi_ppc4xx_pciex_hwops
{
	int (*core_init)(struct device_node *np);
	int (*port_init_hw)(struct lsi_ppc4xx_pciex_port *port);
	int (*port_activate)(struct lsi_ppc4xx_pciex_port *port);
	void (*port_deactivate)(struct lsi_ppc4xx_pciex_port *port);
};

static struct lsi_ppc4xx_pciex_hwops *lsi_ppc4xx_pciex_hwops;

#ifdef CONFIG_44x
static int __devinit ppc476fp_pciex_core_init(struct device_node *np)
{
	/* return 3 ports */
	return 3;
}
#ifdef CONFIG_ACP_PCIE_HOTPLUG
/**
 * Can't be __init mem, may be used after init phase if
 * hootplug is supported
 */
static int __devinit ppc476fp_pciex_port_init_hw(struct lsi_ppc4xx_pciex_port *port)
#else
static int __init ppc476fp_pciex_port_init_hw(struct lsi_ppc4xx_pciex_port *port)
#endif
{
	void __iomem *mbase = NULL;
	u32 cfg, state;
	int loop = 0;
	int rc = 0;

	if (!port->endpoint) {
		/* setting up as root complex */
		mbase = port->utl_base;

		state = in_le32(mbase + PEI_STATUS);
		if ((state & PEI_PIN_CFG) != PEI_RC) {
			pr_err("ACP device is not PCI Root Complex! status = 0x%08x\n",
			       state);
			return -EFAULT;
		}

		/* make sure the link is up */
		if ((state & PEI_LSTATE) != PEI_LSTATE_UP) {
			/* Reset once and see if link status gets OK. */
			pr_debug("PCI link in bad state - resetting\n");

			cfg = in_le32(mbase + PEI_CONFIG);
			cfg |= PEI_RESET_EN;
			out_le32(mbase + PEI_CONFIG, cfg);

			do {
				udelay(10);
				cfg = in_le32(mbase + PEI_CONFIG);
				state = in_le32(mbase + PEI_STATUS);
				loop++;
			} while( ((cfg & PEI_RESET_EN) || (state & PEI_RESET_PRG)) &&
				 loop < 500);

			state = in_le32(mbase + PEI_STATUS);
			if ((state & PEI_LSTATE) != PEI_LSTATE_UP) {
#ifndef CONFIG_ACP_PCIE_HOTPLUG
				/**
				 * Just give up if we don't support hotplug
				 */
				rc = -EFAULT;
#endif
				pr_debug("PCI link state still bad = 0x%x, after %d us\n",
					state, loop * 10);
			} else
				pr_debug("PCI link state OK = 0x%x\n", state);
		} else {
			pr_debug("PCI link state OK = 0x%x\n", state);
		}
	}
	return rc;
}

#ifdef CONFIG_ACP_PCIE_HOTPLUG
/**
 * Can't be __init mem, may be used after init phase if
 * hootplug is supported
 */
static int ppc476fp_pciex_activate(struct lsi_ppc4xx_pciex_port *port)
#else
static int __init ppc476fp_pciex_activate(struct lsi_ppc4xx_pciex_port *port)
#endif
{
	void __iomem *mbase = port->utl_base;
	u32 val, state;

	state = in_le32(mbase + PEI_STATUS);
	if ((state & PEI_LSTATE) == PEI_LSTATE_UP) {
		pr_debug("PCI link state is up enable AXI IF\n");
		/* for v2 we need to set the 'axi_interface_rdy' bit */
		val = in_le32(mbase + PEI_CONFIG);
		val |= PEI_AXI_IF_RDY;
		out_le32(mbase + PEI_CONFIG, val);
		return 0;
	} else
		pr_debug("PCI link state is down - AXI IF disabled\n");
	return -EFAULT;
}
#ifdef CONFIG_ACP_PCIE_HOTPLUG
/**
 * Can't be __init mem, may be used after init phase if
 * hootplug is supported
 */
static void ppc476fp_pciex_deactivate(struct lsi_ppc4xx_pciex_port *port)
#else
static void __init ppc476fp_pciex_deactivate(struct lsi_ppc4xx_pciex_port *port)
#endif
{
	void __iomem *mbase = port->utl_base;
	u32 val;

	val = in_le32(mbase + PEI_CONFIG);
	val &= ~PEI_AXI_IF_RDY;
	out_le32(mbase + PEI_CONFIG, val);
}

#ifdef CONFIG_ACP_PCIE_HOTPLUG
/**
 * Can't be __init mem, may be used after init phase if
 * hootplug is supported
 */
static struct lsi_ppc4xx_pciex_hwops ppc476fp_pcie_hwops __devinitdata =
#else
static struct lsi_ppc4xx_pciex_hwops ppc476fp_pcie_hwops __initdata =
#endif
{
   .core_init = ppc476fp_pciex_core_init,
   .port_init_hw = ppc476fp_pciex_port_init_hw,
   .port_activate = ppc476fp_pciex_activate,
   .port_deactivate = ppc476fp_pciex_deactivate,
};
#ifdef CONFIG_ACP_PCIE_HOTPLUG
int lsi_ppc4xx_pci_init_hw(struct pci_controller *hose)
{
	struct lsi_ppc4xx_pciex_port *port =
		&lsi_ppc4xx_pciex_ports[hose->indirect_type];

	if (!port)
		return -EINVAL;

	if (lsi_ppc4xx_pciex_hwops->port_init_hw)
		return lsi_ppc4xx_pciex_hwops->port_init_hw(port);

	return 0;
}
EXPORT_SYMBOL(lsi_ppc4xx_pci_init_hw);

int lsi_ppc4xx_pci_activate(struct pci_controller *hose)
{
	struct lsi_ppc4xx_pciex_port *port =
		&lsi_ppc4xx_pciex_ports[hose->indirect_type];

	if (!port)
		return -EINVAL;

	if (lsi_ppc4xx_pciex_hwops->port_activate)
		return lsi_ppc4xx_pciex_hwops->port_activate(port);

	return 0;
}
EXPORT_SYMBOL(lsi_ppc4xx_pci_activate);

void lsi_ppc4xx_pci_deactivate(struct pci_controller *hose)
{
	struct lsi_ppc4xx_pciex_port *port =
		&lsi_ppc4xx_pciex_ports[hose->indirect_type];

	if (!port)
		return;

	if (lsi_ppc4xx_pciex_hwops->port_deactivate)
		lsi_ppc4xx_pciex_hwops->port_deactivate(port);
}
EXPORT_SYMBOL(lsi_ppc4xx_pci_deactivate);
#endif

#endif /* CONFIG_44x */

/* Check that the core has been initied and if not, do it */
static int __devinit lsi_ppc4xx_pciex_check_core_init(struct device_node *np)
{
	static int core_init;
	int count = -ENODEV;

	if (core_init++)
		return 0;

#ifdef CONFIG_44x
	if (of_device_is_compatible(np, "lsi,plb-pciex-476fp")) {
		lsi_ppc4xx_pciex_hwops = &ppc476fp_pcie_hwops;
	}
#endif /* CONFIG_44x    */
	if (lsi_ppc4xx_pciex_hwops == NULL) {
		pr_err("PCIE: unknown host type %s\n",
		       np->full_name);
		return -ENODEV;
	}

	count = lsi_ppc4xx_pciex_hwops->core_init(np);
	if (count > 0) {
		lsi_ppc4xx_pciex_ports =
			kzalloc(count * sizeof(struct lsi_ppc4xx_pciex_port),
				GFP_KERNEL);
		if (lsi_ppc4xx_pciex_ports) {
			lsi_ppc4xx_pciex_port_count = count;
			return 0;
		}
		pr_err("PCIE: failed to allocate ports array\n");
		return -ENOMEM;
	}
	return -ENODEV;
}

static int __init lsi_ppc4xx_pciex_port_init(struct lsi_ppc4xx_pciex_port *port)
{
	int rc = 0;

	/*
	 * Map Internal CFG space (root complex mode)
	 *
	 */
	port->utl_base = ioremap(port->utl_regs.start, PEI_INT_CFG_SIZE);

	BUG_ON(port->utl_base == NULL);
	pr_debug("PCIE %s\n", port->node->full_name);
	pr_debug("Internal config space %u bytes mapped at: root %p, from %llx\n",
		 PEI_INT_CFG_SIZE, (void *)port->utl_base, port->utl_regs.start);

		/* Init HW */
	if (lsi_ppc4xx_pciex_hwops->port_init_hw)
		rc = lsi_ppc4xx_pciex_hwops->port_init_hw(port);
	if (rc)
		goto fail;

	return rc;
fail:
	iounmap(port->utl_base);
	return rc;
}

static int lsi_ppc4xx_pciex_validate_bdf(struct lsi_ppc4xx_pciex_port *port,
				     struct pci_bus *bus,
				     unsigned int devfn)
{
	static int message;

	/* Endpoint can not generate upstream(remote) config cycles */
	if (port->endpoint && bus->number != port->hose->first_busno) {
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	/* Check we are within the mapped range */
	if (bus->number > port->hose->last_busno) {
		if (!message) {
			pr_debug("Warning! Probing bus %u"
				 " out of range !\n", bus->number);
			message++;
		}
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	/* The root complex has only one device / function */
	if (bus->number == port->hose->first_busno && devfn != 0) {
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	/* The other side of the RC has only one device as well */
	if (bus->number == (port->hose->first_busno + 1) &&
	    PCI_SLOT(devfn) != 0) {
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	return 0;
}

static void __iomem *lsi_ppc4xx_pciex_get_config_base(struct lsi_ppc4xx_pciex_port *port,
						      struct pci_bus *bus,
						      unsigned int devfn)
{
	unsigned mpage;
	u32 addr;
	int dev, fn;
	int cfg_type;
	int relbus;

	/* Remove the casts when we finally remove the stupid volatile
	 * in struct pci_controller
	 */
	if (bus->number == port->hose->first_busno)
		return (void __iomem *)port->hose->cfg_addr;

	relbus = bus->number - (port->hose->first_busno + 1);

	/*
	 * Set MPAGE0 to map config access for this BDF
	 */

	dev = ( (devfn & 0xf8) >> 3);
	fn = devfn & 0x7;

	if (dev > 31)
		return NULL;
	if ((fn > 3) || (bus->number > 63))
		return NULL;

	if (relbus && (bus->number != bus->primary))
		cfg_type = 1;
	else
		cfg_type = 0;

	/* build the mpage register */
	mpage = (bus->number << 11) |
		(dev << 6) |
		(cfg_type << 5 ) |
		0x11; /* enable MPAGE for configuration access */

	mpage |= (fn << 17);

	addr = ((u32) port->hose->cfg_addr) + PEI_MPAGE_LOWER(7);
	out_le32((u32 *) addr, mpage);

	return (void __iomem *)port->hose->cfg_data;
}

static int lsi_ppc4xx_pciex_read_config(struct pci_bus *bus, unsigned int devfn,
					int offset, int len, u32 *val)
{
	struct pci_controller *hose = (struct pci_controller *) bus->sysdata;
	struct lsi_ppc4xx_pciex_port *port = &lsi_ppc4xx_pciex_ports[hose->indirect_type];
	void __iomem *addr;
	u32 bus_addr;
	u32 val32, state;
	u32 mcsr;
	int bo = offset & 0x3;
	int rc = PCIBIOS_SUCCESSFUL;

	BUG_ON(hose != port->hose);

	if (lsi_ppc4xx_pciex_validate_bdf(port, bus, devfn) != 0) {
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	addr = lsi_ppc4xx_pciex_get_config_base(port, bus, devfn);
	/* check link state mabye - as we now register bridges even if the link is down...
	 */
	state = in_le32((void __iomem *)hose->cfg_addr + PEI_STATUS);

	if (!addr || ((state & PEI_LSTATE) != PEI_LSTATE_UP)) {
		*val = 0;
		pr_debug("%s: Will pass on this one as link training is not done\n"
			 "try rescan later when end-point is up\n", __func__);
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	/*
	 * Reading from configuration space of non-existing device can
	 * generate transaction errors. For the read duration we suppress
	 * assertion of machine check exceptions to avoid those.
	 */
	mtmsr( mfmsr() & ~(MSR_ME));
	__asm__ __volatile__(PPC_MSYNC);

	/*
	 * addressing is different for local config access vs.
	 * access through the mapped config space.
	 */
	if (bus->number == port->hose->first_busno)
		bus_addr = (u32) addr + (offset & 0xfffffffc);
	else {

		/*
		 * mapped config space only supports 32-bit access
		 *
		 *  AXI address[3:0] is not used at all.
		 *  AXI address[9:4] becomes register number.
		 *  AXI address[13:10] becomes Ext. register number
		 *  AXI address[17:14] becomes 1st DWBE for configuration read only.
		 *  AXI address[29:27] is used to select one of 8 Mpage registers.
		 */

		bus_addr = (u32) addr + (offset << 2);

		switch (len) {
		case 1:
			bus_addr |= ((1 << bo) ) << 14;
			break;
		case 2:
			bus_addr |= ((3 << bo) ) << 14;
			break;
		default:
			bus_addr |= ( 0xf ) << 14;
			break;
		}

	}

	/*
	 * do the read
	 */

	val32 = in_le32((u32 *)bus_addr);

	switch (len) {
	case 1:
		*val = (val32 >> (bo * 8)) & 0xff;
		break;
	case 2:
		*val = (val32 >> (bo * 8)) & 0xffff;
		break;
	default:
		*val = val32;
		break;
	}

	__asm__ __volatile__(PPC_MSYNC);

	mcsr = mfspr(SPRN_MCSR);
	if ( mcsr != 0) {
		mtspr(SPRN_MCSR, 0);
		__asm__ __volatile__(PPC_MSYNC);

		pr_debug("acp_read_config  : bus=%3d [%3d..%3d] devfn=0x%04x"
			 " offset=0x%04x len=%d, addr=0x%08x machine check!! "
			 "0x%08x\n",
			 bus->number, hose->first_busno, hose->last_busno,
			 devfn, offset, len, bus_addr, mcsr);
		*val = 0;
		rc = PCIBIOS_DEVICE_NOT_FOUND;
	} else {
		pr_debug("acp_read_config  : bus=%3d [%3d..%3d] devfn=0x%04x"
			 " offset=0x%04x len=%d, addr=0x%08x val=0x%08x\n",
			 bus->number, hose->first_busno, hose->last_busno,
			 devfn, offset, len, bus_addr, *val);
	}

	/* re-enable machine checks */
	mtmsr(mfmsr() | (MSR_ME) );
	__asm__ __volatile__(PPC_MSYNC);

	return rc;
}

static int lsi_ppc4xx_pciex_write_config(struct pci_bus *bus, unsigned int devfn,
					 int offset, int len, u32 val)
{
	struct pci_controller *hose = (struct pci_controller *) bus->sysdata;
	struct lsi_ppc4xx_pciex_port *port = &lsi_ppc4xx_pciex_ports[hose->indirect_type];
	void __iomem *addr;
	u32 bus_addr, state;

	if (lsi_ppc4xx_pciex_validate_bdf(port, bus, devfn) != 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	addr = lsi_ppc4xx_pciex_get_config_base(port, bus, devfn);
	/* check link state mabye - as we now register bridges even if the link is down...
	 */
	state = in_le32((void __iomem *)hose->cfg_addr + PEI_STATUS);

	if (!addr || ((state & PEI_LSTATE) != PEI_LSTATE_UP))
		return PCIBIOS_DEVICE_NOT_FOUND;

	/*
	 * addressing is different for local config access vs.
	 * access through the mapped config space. We need to
	 * translate the offset for mapped config access
	 */
	if (bus->number == port->hose->first_busno) {
		/* the local ACP RC only supports 32-bit dword config access,
		 * so if this is a byte or 16-bit word access we need to
		 * perform a read-modify write
		 */
		if (len == 4)
			bus_addr = (u32) addr + offset;
		else {
			int bs = ((offset & 0x3) * 8);
			u32 val32;

			bus_addr = (u32) addr + (offset & 0xfffffffc);
			val32 = in_le32((u32 *)bus_addr);

			if (len == 2)
				val32 = (val32 & ~(0xffff << bs) ) | ((val & 0xffff) << bs);
			else
				val32 = (val32 & ~(0xff << bs) ) | ((val & 0xff) << bs);

			val = val32;
			len = 4;
		}

	} else {
		bus_addr = (u32) addr + (offset << 2) + (offset & 0x3);
	}
	pr_debug("acp_write_config : bus=%3d [%3d..%3d] devfn=0x%04x"
		 " offset=0x%04x len=%d, addr=0x%08x val=0x%08x\n",
		 bus->number, hose->first_busno, hose->last_busno,
		 devfn, offset, len, bus_addr, val);

	switch (len) {
	case 1:
		out_8((u8 *)(bus_addr), val);
		break;
	case 2:
		out_le16((u16 *)(bus_addr), val);
		break;
	default:
		out_le32((u32 *)(bus_addr), val);
		break;
	}
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops lsi_ppc4xx_pciex_pci_ops =
{
	.read  = lsi_ppc4xx_pciex_read_config,
	.write = lsi_ppc4xx_pciex_write_config,
};

static int __init lsi_ppc4xx_setup_one_pciex_POM(struct lsi_ppc4xx_pciex_port	*port,
					     struct pci_controller	*hose,
					     void __iomem		*mbase,
					     u64			plb_addr,
					     u64			pci_addr,
					     u64			size,
					     unsigned int		flags,
					     int			index)
{
	u32 lah, lal, pciah, pcial, sa;
	int i, num_pages;
	u32 mpage_lower;

	if (!is_power_of_2(size) ||
	    (index < 2 && size < 0x100000) ||
	    (index == 2 && size < 0x100) ||
	    (plb_addr & (size - 1)) != 0) {
		pr_err("%s: Resource out of range\n",
		       hose->dn->full_name);
		return -1;
	}

	/* Calculate register values */
	lah = RES_TO_U32_HIGH(plb_addr);
	lal = RES_TO_U32_LOW(plb_addr);
	pciah = RES_TO_U32_HIGH(pci_addr);
	pcial = RES_TO_U32_LOW(pci_addr);
	sa = (0xffffffffu << ilog2(size)) | 0x1;

	/* ACP X1 setup MPAGE registers */


	pr_debug("setting outbound window %d with "
		 "plb_add=0x%012llx, pci_addr=0x%012llx, size=0x%012llx\n",
		 index, plb_addr, pci_addr, size);

	/*
	 * MPAGE7 is dedicated to config access, so we only
	 *  have 7 128MB pages available for memory i/o.
	 *  Calculate how many pages we need
	 */
	if (size > (7 * 0x08000000)) {
		pr_err("%s: Resource size 0x%012llx out of range\n",
		       hose->dn->full_name, size);
		return -1;
	}

	num_pages = ( (size - 1) >> 27) + 1;
	for (i = 0; i < num_pages; i++) {
		mpage_lower = (pcial & 0xf8000000) | 1;
		out_le32( mbase + PEI_MPAGE_UPPER(i), pciah);
		out_le32( mbase + PEI_MPAGE_LOWER(i), mpage_lower);
		pcial += 0x08000000;
	}
	return 0;
}

static void __init lsi_ppc4xx_configure_pciex_POMs(struct lsi_ppc4xx_pciex_port *port,
					       struct pci_controller *hose,
					       void __iomem *mbase)
{
	int i, j, found_isa_hole = 0;

	/* Setup outbound memory windows */
	for (i = j = 0; i < 3; i++) {
		struct resource *res = &hose->mem_resources[i];

		/* we only care about memory windows */
		if (!(res->flags & IORESOURCE_MEM))
			continue;
		if (j > 1) {
			pr_err("%s: Too many ranges\n",
			       port->node->full_name);
			break;
		}

		/* Configure the resource */
		if (lsi_ppc4xx_setup_one_pciex_POM(port, hose, mbase,
					       res->start,
					       res->start - hose->pci_mem_offset,
					       res->end + 1 - res->start,
					       res->flags,
					       j) == 0) {
			j++;

			/* If the resource PCI address is 0 then we have our
			 * ISA memory hole
			 */
			if (res->start == hose->pci_mem_offset)
				found_isa_hole = 1;
		}
	}

	/* Handle ISA memory hole if not already covered
	 * sais somewhere else that it's not supported -
	 * do we need this check?
	 */
	if (j <= 1 && !found_isa_hole && hose->isa_mem_size)
		if (lsi_ppc4xx_setup_one_pciex_POM(port, hose, mbase,
					       hose->isa_mem_phys, 0,
					       hose->isa_mem_size, 0, j) == 0)
			pr_debug("%s: Legacy ISA memory support enabled\n",
				 hose->dn->full_name);

	/* Configure IO, always 64K starting at 0. We hard wire it to 64K !
	 * Note also that it -has- to be region index 2 on this HW
	 */
	if (hose->io_resource.flags & IORESOURCE_IO) {
		lsi_ppc4xx_setup_one_pciex_POM(port, hose, mbase,
					   hose->io_base_phys, 0,
					   0x10000, IORESOURCE_IO, 2);
	}
}

static void __init lsi_ppc4xx_configure_pciex_PIMs(struct lsi_ppc4xx_pciex_port *port,
						   struct pci_controller *hose,
						   void __iomem *mbase,
						   struct resource *res)
{
	resource_size_t size = res->end - res->start + 1;
	u64 sa;
	int i;

	if (port->endpoint) {
		resource_size_t ep_addr = 0;
		resource_size_t ep_size = 32 << 20;

		/* Currently we map a fixed 64MByte window to PLB address
		 * 0 (SDRAM). This should probably be configurable via a dts
		 * property.
		 */

		/* Calculate window size */
		sa = (0xffffffffffffffffull << ilog2(ep_size));;

		/* TODO: */

		out_le32(mbase + PCI_BASE_ADDRESS_0, RES_TO_U32_LOW(ep_addr));
		out_le32(mbase + PCI_BASE_ADDRESS_1, RES_TO_U32_HIGH(ep_addr));
	} else {
		/* Calculate window size */
		sa = (0xffffffffffffffffull << ilog2(size));;
		if (res->flags & IORESOURCE_PREFETCH)
			sa |= 0x8;

		pr_debug("configure inbound mapping from 0x%012llx-0x%012llx (0x%08llx bytes)\n",
			 res->start, res->end, size);

		out_le32(mbase + PCI_BASE_ADDRESS_0, RES_TO_U32_LOW(size));
		out_le32(mbase + PCI_BASE_ADDRESS_1, RES_TO_U32_HIGH(size));

		/*
		 * set up the TPAGE registers
		 *
		 * We set the MSB of each TPAGE to select 128-bit AXI access.
		 * For the address field we simply program an incrementing value
		 * to map consecutive pages
		 */
		for (i = 0; i < 8; i++)
			out_le32(mbase + PEI_TPAGE_BAR0(i), (PEI_AXI_SIZE_128 + i));
	}

	/* Enable I/O, Mem, and Busmaster cycles */
	out_le16(mbase + PCI_COMMAND,
		 in_le16(mbase + PCI_COMMAND) |
		 PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);
}

static irqreturn_t
acp_pcie_isr(int irq, void *arg)
{
   struct pci_controller *hose = (struct pci_controller *) arg;
   void __iomem *mbase = (void __iomem *)hose->cfg_addr;

   u32 intr_status;
   u32 intr_enb;
   u32 msg_fifo_stat;
   u32 msg_fifo_info;
   u8 externalPciIntr = 0;

   /* read the PEI interrupt status register */
   intr_status = in_le32(mbase + PEI_IRQ_STATUS);
   intr_enb = in_le32(mbase + PEI_IRQ_EN);

   /* check if this is a PCIe message from an external device */
   if (intr_status & 0x00000010)
   {
      externalPciIntr = 1;
      pr_debug("ACP_PCIE_ISR %d: got external error interrupt 0x%08x\n",
	       hose->indirect_type, intr_status);

      msg_fifo_stat = in_le32(mbase + PEI_MSG_IN_FIFO_STATUS);

      /* loop until the message fifo is empty */
      while ( (msg_fifo_stat & 0x01) == 0)  {

         u8 bus, dev, fn;
         u8 msg_type;
         msg_fifo_info = in_le32(mbase + PEI_MSG_IN_FIFO);

         bus = (msg_fifo_info >> 16 ) & 0xff;
         dev = (msg_fifo_info >> 11 ) & 0x1f;
         fn = (msg_fifo_info >> 8 ) & 0x07;
         msg_type = msg_fifo_info & 0xff;

	 /* print out the BDF and message type.
          * We ignore the common message types.
          */
         switch (msg_type)
         {
            case 0x20: /*    assert_INTa */
            case 0x21: /*    assert_INTb */
            case 0x22: /*    assert_INTc */
            case 0x23: /*    assert_INTd */
            case 0x24: /* de-assert_INTa */
            case 0x25: /* de-assert_INTb */
            case 0x26: /* de-assert_INTc */
            case 0x27: /* de-assert_INTd */

            /* do nothing */
            break;

            default:
            pr_debug("BDF %02x:%02x.%x sent msgtype 0x%02x\n", bus, dev, fn, msg_type);
            break;
         }
         /* re-read fifo status */
         msg_fifo_stat = in_le32(mbase + PEI_MSG_IN_FIFO_STATUS);
      }
   }
   else
   {
      /* ignore the common interrupts, still need to figure out what they all mean */
      if (intr_status & 0xf3ffffab)
      {
         u32 t2a_err_stat;
         u32 t2a_other_err_stat;
         u32 int_enb;
         u32 linkStatus;
	 struct lsi_ppc4xx_pciex_port *port = &lsi_ppc4xx_pciex_ports[hose->indirect_type];

         pr_debug("ACP_PCIE_ISR %d: got PEI error interrupt 0x%08x\n",
		  hose->indirect_type, intr_status);

         linkStatus = in_le32(mbase + 0x117c);

         pr_debug("link_status (0x117c) = 0x%08x\n", linkStatus);

         if (intr_status & PEI_IRQ_T2A_IGR_ERR)
         {
            t2a_err_stat = in_le32(mbase + PEI_T2A_INDP_ERR);
	    atomic_set(&port->stat.t2a_igr_state, t2a_err_stat);
	    atomic_inc(&port->stat.t2a_igr_err);
         }

         if (intr_status & 0x00040000)
         {
            t2a_other_err_stat = in_le32(mbase + 0x1174);
            int_enb = in_le32(mbase + PEI_IRQ_EN);
            int_enb &= 0xfffbffff;
            out_le32(mbase + PEI_IRQ_EN, int_enb);
         }

         if (intr_status & 0x00000800)
         {
            int_enb = in_le32(mbase + PEI_IRQ_EN);
            int_enb &= 0xfffff7ff;
            out_le32(mbase + PEI_IRQ_EN, int_enb);
         }
      }
   }

   /*
    *  We clear all the interrupts in the PEI status, even though
    *  interrupts from external devices have not yet been handled.
    *  That should be okay, since the PCI IRQ in the MPIC won't be
    *  re-enabled until all external handlers have been called.
    */
   out_le32(mbase + PEI_IRQ_STATUS, intr_status);

   return (externalPciIntr ? IRQ_NONE : IRQ_HANDLED);
}

static int __init lsi_ppc4xx_setup_irq(struct pci_controller *hose)
{
	void __iomem *mbase = (void __iomem *)hose->cfg_addr;
	struct lsi_ppc4xx_pciex_port *port = &lsi_ppc4xx_pciex_ports[hose->indirect_type];
        const u32 *imap, *tmp;
        int imaplen, intsize;
	int ph_offs = 4;
	int irq_offs = 5;
        struct device_node *iic;

        /*
	 * Weird! LSI doesn't use the dtb - hardcode IRQ 29 shared for
	 * each device. Irq type not set by LSI??? Unable to find
	 * correct info from docs...
	 * dtb probably completely brooken - tmp. use only first entry
	 * (there are 4 of them/bridge) and see what happens.
	 * NB! dtb choose lvl high but accord. to comment choose sense???
	 * LSI original patch has commented out set type lvl-high
	 * No idea what to make of this!!!
	 */
        tmp = of_get_property(hose->dn, "#interrupt-cells", NULL);
        if (tmp == NULL)
                return NO_IRQ;
        intsize = *tmp;
        imap = of_get_property(hose->dn, "interrupt-map", &imaplen);
        if (imap == NULL || imaplen < (intsize + 1))
                return NO_IRQ;
	/* PIC handle at imap word 4 */
	iic = of_find_node_by_phandle(imap[ph_offs]);
	if (iic == NULL)
		return NO_IRQ;
	/* and now we need the intsize for the PIC */
	tmp = of_get_property(iic, "#interrupt-cells", NULL);
	if (tmp == NULL) {
		of_node_put(iic);
		return NO_IRQ;
	}
	intsize = *tmp;
	/* HW irq at word 5 and type at word 6 */
	port->irq_no = irq_create_of_mapping(iic, &imap[irq_offs], intsize);
	if ( port->irq_no == NO_IRQ) {
		pr_err("irq_create_mapping failed!!!\n");
		of_node_put(iic);
		return NO_IRQ;
	}
	if (request_irq(port->irq_no, acp_pcie_isr, IRQF_SHARED, "pciex", hose)) {
		pr_err("request_irq failed!!!!\n");
		of_node_put(iic);
		return NO_IRQ;
	}
	of_node_put(iic);
	/* unmask PEI interrupts */
	out_le32(mbase + PEI_IRQ_EN, 0xffffffff);

	return 0;
}
static void __init lsi_ppc4xx_pciex_port_setup_hose(struct lsi_ppc4xx_pciex_port *port)
{
	struct resource dma_window;
	struct pci_controller *hose = NULL;
	const int *bus_range;
	int primary = 0, busses;
	void __iomem *mbase = NULL, *cfg_data = NULL;

	/* Check if primary bridge */
	if (of_get_property(port->node, "primary", NULL))
		primary = 1;

	/* Get bus range if any */
	bus_range = of_get_property(port->node, "bus-range", NULL);

	/* Allocate the host controller data structure */
	hose = pcibios_alloc_controller(port->node);
	if (!hose)
		goto fail;

	/* We stick the port number in "indirect_type" so the config space
	 * ops can retrieve the port data structure easily
	 */
	hose->indirect_type = port->index;

	/* Get bus range */
	hose->first_busno = bus_range ? bus_range[0] : 0x0;
	hose->last_busno = bus_range ? bus_range[1] : 0xff;

	busses = hose->last_busno - hose->first_busno; /* This is off by 1 */
	if (busses > MAX_PCIE_BUS_MAPPED) {
		busses = MAX_PCIE_BUS_MAPPED;
		hose->last_busno = hose->first_busno + busses;
	}

	if (!port->endpoint) {
		/*
		 * map the bottom page of PCIe memory for config space access
		 */
		cfg_data = ioremap(port->cfg_space.start, PEI_EXT_CFG_SIZE);
		if (cfg_data == NULL) {
			pr_err("%s: Can't map external config space !",
			       port->node->full_name);
			goto fail;
		}
		hose->cfg_data = cfg_data;
	}
	/*
	 * The internal config space has already been mapped, so
	 * just re-use that virtual address.
	 */
	hose->cfg_addr = port->utl_base;

	pr_debug("PCIE %s, bus %d..%d\n", port->node->full_name,
		 hose->first_busno, hose->last_busno);
	pr_debug("     config space mapped at: root @0x%p, other @0x%p\n",
		 hose->cfg_addr, hose->cfg_data);

	/* Setup config space */
	hose->ops = &lsi_ppc4xx_pciex_pci_ops;
	port->hose = hose;
	mbase = (void __iomem *)hose->cfg_addr;

	if (!port->endpoint) {
		/*
		 * Set bus numbers on our root port
		 */
		out_8(mbase + PCI_PRIMARY_BUS, hose->first_busno);
		out_8(mbase + PCI_SECONDARY_BUS, hose->first_busno + 1);
		out_8(mbase + PCI_SUBORDINATE_BUS, hose->last_busno);
	}

	/* Parse outbound mapping resources */
	pci_process_bridge_OF_ranges(hose, port->node, primary);

	/* Parse inbound mapping resources */
	if (lsi_ppc4xx_parse_dma_ranges(hose, mbase, &dma_window) != 0)
		goto fail;

	/* Configure outbound ranges POMs */
	lsi_ppc4xx_configure_pciex_POMs(port, hose, mbase);

	/* Configure inbound ranges PIMs */
	lsi_ppc4xx_configure_pciex_PIMs(port, hose, mbase, &dma_window);

	/*
	 * Setup IRQ handler
	 */
	if (lsi_ppc4xx_setup_irq(hose) != 0)
		goto fail;

	if (lsi_ppc4xx_pciex_hwops->port_activate)
		lsi_ppc4xx_pciex_hwops->port_activate(port);

	if (!port->endpoint) {
		pr_debug("PCIE%d: successfully set as root-complex\n",
			 port->index);
	}
	return;
 fail:
	if (hose)
		pcibios_free_controller(hose);
	if (cfg_data)
		iounmap(cfg_data);
	if (mbase)
		iounmap(mbase);
}

static void __init lsi_ppc4xx_probe_pciex_bridge(struct device_node *np)
{
	struct lsi_ppc4xx_pciex_port *port;
	const u32 *pval;
	int portno;
	const char *val;

	/* First, proceed to core initialization as we assume there's
	 * only one PCIe core in the system
	 */
	if (lsi_ppc4xx_pciex_check_core_init(np))
		return;

	/* Get the port number from the device-tree */
	pval = of_get_property(np, "port", NULL);
	if (pval == NULL) {
		pr_err("PCIE: Can't find port number for %s\n",
		       np->full_name);
		return;
	}
	portno = *pval;
	if (portno >= lsi_ppc4xx_pciex_port_count) {
		pr_err("PCIE: port number out of range for %s\n",
		       np->full_name);
		return;
	}
	port = &lsi_ppc4xx_pciex_ports[portno];
	port->index = portno;

	/*
	 * Check if device is enabled
	 */
	if (!of_device_is_available(np)) {
		pr_debug("PCIE%d: Port disabled via device-tree\n", port->index);
		return;
	}
	/* Check if device_type property is set to "pci" or "pci-endpoint".
	 * Resulting from this setup this PCIe port will be configured
	 * as root-complex or as endpoint.
	 */
	port->node = of_node_get(np);
	val = of_get_property(port->node, "device_type", NULL);
	if (!val) {
		pr_err("PCIE%d: device_type not found via device-tree\n", port->index);
		return;
	}
	if (!strcmp(val, "pci-endpoint")) {
		port->endpoint = 1;
	} else if (!strcmp(val, "pci")) {
		port->endpoint = 0;
	} else {
		pr_err("PCIE: missing or incorrect device_type for %s\n",
		       np->full_name);
		return;
	}
	/* Fetch the external config space registers address */
	if (of_address_to_resource(np, 0, &port->cfg_space)) {
		pr_err("%s: Can't get PCI-E config space !",
		       np->full_name);
		return;
	}
	/* Fetch host bridge internal registers address */
	if (of_address_to_resource(np, 1, &port->utl_regs)) {
		pr_err("%s: Can't get UTL register base !",
		       np->full_name);
		return;
	}

	/* Initialize the port specific registers */
	if (lsi_ppc4xx_pciex_port_init(port)) {
		pr_err("PCIE%d: Port init failed\n", port->index);
		return;
	}
	/* Setup the linux hose data structure */
	lsi_ppc4xx_pciex_port_setup_hose(port);
}
#endif /* CONFIG_PPC4xx_PCI_EXPRESS */

static int __init lsi_ppc4xx_pci_find_bridges(void)
{
	struct device_node *np;

	pci_add_flags(PCI_ENABLE_PROC_DOMAINS | PCI_COMPAT_DOMAIN_0);

#ifdef CONFIG_PPC4xx_PCI_EXPRESS
	for_each_compatible_node(np, NULL, "lsi,plb-pciex") {

		pr_debug("%s: found lsi,plb-pciex node\n", __func__);
		lsi_ppc4xx_probe_pciex_bridge(np);
	}
#endif

	return 0;
}
arch_initcall(lsi_ppc4xx_pci_find_bridges);
/*
 * sysfs extras
 */
#ifdef CONFIG_PPC4xx_PCI_EXPRESS

static ssize_t pci_show_dev(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct pci_bus *bus = to_pci_bus(dev);
	struct pci_controller *hose = pci_bus_to_host(bus);

	if (hose)
		return sprintf(buf, "hose bus_range[%x - %x]\n",
			       hose->first_busno, hose->last_busno);
	return 0;
}
static DEVICE_ATTR(bus_range, S_IRUGO, pci_show_dev, NULL);


static ssize_t pci_show_stat(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct pci_bus *bus = to_pci_bus(dev);
	struct pci_controller *hose = pci_bus_to_host(bus);

	if (hose) {
		struct lsi_ppc4xx_pciex_port *port = &lsi_ppc4xx_pciex_ports[hose->indirect_type];
		char *str = buf;

		str += sprintf(str, "t2a_igr_state %8.8x\n",
			       (unsigned)atomic_read(&port->stat.t2a_igr_state));
		str += sprintf(str, "t2a_igr_err %d\n",
			       atomic_read(&port->stat.t2a_igr_err));
		return str - buf;
	}
	return 0;
}
static DEVICE_ATTR(stats, S_IRUGO, pci_show_stat, NULL);


static struct attribute *pciex_attributes[] = {
	&dev_attr_bus_range.attr,
	&dev_attr_stats.attr,
	NULL
};

static struct attribute_group pciex_attribute_group = {
	.name = "statistics",
	.attrs = pciex_attributes,
};
#endif

int __devinit lsi_ppc4xx_pci_sysfs_init(struct pci_controller *hose)
{
	int rc = 0;

#ifdef CONFIG_PPC4xx_PCI_EXPRESS
	rc = sysfs_create_group(&hose->bus->dev.kobj, &pciex_attribute_group);
#endif
	return rc;
}
EXPORT_SYMBOL(lsi_ppc4xx_pci_sysfs_init);
