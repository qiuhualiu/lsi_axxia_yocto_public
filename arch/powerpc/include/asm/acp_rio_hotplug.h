/*
 * RapidIO support for LSI Axxia 3400 parts
 *
 */
#ifndef __ACP_RIO_HOTPLUG_H__
#define __ACP_RIO_HOTPLUG_H__

#ifdef CONFIG_RAPIDIO_HOTPLUG
extern int acp_rio_sysfs_init(void);
extern int acp_rio_remove_net(int hostid, unsigned tmo_ms);
extern int acp_rio_scan_net(int hostid, unsigned tmo_ms);
extern int acp_rio_rescan_net(int hostid, unsigned tmo_ms);
extern int acp_rio_scan_dsp(int destid);
extern int acp_rio_remove_dsp(int destid);
extern int acp_rio_rescan_dsp(int destid);
#else
static inline int acp_rio_sysfs_init(void) { return 0; }
static inline int acp_rio_remove_net(int hostid, unsigned tmo_ms) { return 0; }
static inline int acp_rio_scan_net(int hostid, unsigned tmo_ms) { return 0; }
static inline int acp_rio_rescan_net(int hostid, unsigned tmo_ms) { return 0; }
static inline int acp_rio_scan_dsp(int destid) { return 0; }
static inline int acp_rio_remove_dsp(int destid) { return 0; }
static inline int acp_rio_rescan_dsp(int destid) { return 0; }
#endif

#ifdef CONFIG_RAPIDIO_STATIC_DESTID
extern void acp_rio_install_destid_cb(void);
#else
static inline void acp_rio_install_destid_cb(void) {}
#endif

#if defined(CONFIG_ACP3400_DUS)

/* #define HERMES_I2C_DEBUG */

#ifdef HERMES_I2C_DEBUG
struct srds_data {
	u8 addr;
	u32 data;
};

int hermes_readback(u32 snid, u32 *blob, u32 size, struct srds_data *rb);
int hermes_serdes(u32 snid, u32 *serdesParam, u32 size, struct srds_data *rb);
#else
int hermes_serdes(u32 snid, u32 *serdesParam, u32 size);
#endif
void hermes_sync_i2c_bus(void);

#else /* #if defined(CONFIG_ACP3400_DUS) */

static inline int hermes_serdes(u32 snid, u32 *serdesParam, u32 size) { return 0; }

#endif

#endif
