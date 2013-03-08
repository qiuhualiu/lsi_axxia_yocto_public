/*
 * RapidIO support for LSI Axxia 3400 parts
 *
 */
#ifndef __ASM_ACP3400_RIO_H__
#define __ASM_ACP3400_RIO_H__

/* ACP RIO debug stuff */

extern int acp3400_rio_apio_enable(struct rio_mport *mport, u32 mask, u32 bits);
extern int acp3400_rio_apio_disable(struct rio_mport *mport);
extern int acp3400_rio_rpio_enable(struct rio_mport *mport, u32 mask, u32 bits);
extern int acp3400_rio_rpio_disable(struct rio_mport *mport);

#endif
