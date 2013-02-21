/*
 * @file ncr_cd.h
 * @section DESCRIPTION
 *
 * Header file for the SSP char device driver
 *
 */
#ifndef NCR_CD_H_
#define NCR_CD_H_

#include <linux/ioctl.h>

#define NCR_MAJOR 64

#define NCRWRITE32	_IOWR(NCR_MAJOR, 1, int)
#define NCRREAD32	_IOWR(NCR_MAJOR, 2, int)


/* Number of scull devices */
#ifndef NCR_COUNT
#define NCR_COUNT 1
#endif

/* Name of the scull driver */
#ifndef NCR_NAME
#define NCR_NAME "ncr"
#endif

/* First minor number */
#ifndef NCR_MINOR_START
#define NCR_MINOR_START 0
#endif

#define NCR_BUFSZ 1024



/**
 * Enum for the ioctl calls
 */
typedef struct NCR_IOC_TRANSFER {
	unsigned long region;
	unsigned long offset;
	unsigned long *buffer;
} ncr_ioc_transfer;

#endif
