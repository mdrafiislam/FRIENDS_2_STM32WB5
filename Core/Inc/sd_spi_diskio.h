#ifndef SD_SPI_DISKIO_H
#define SD_SPI_DISKIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "diskio.h"

#define SD_SPI_DRIVE 0U

void SD_SPI_Reset(void);


DSTATUS SD_SPI_initialize(void);
DSTATUS SD_SPI_status(void);
DRESULT SD_SPI_read(BYTE *buff, LBA_t sector, UINT count);
DRESULT SD_SPI_write(const BYTE *buff, LBA_t sector, UINT count);
DRESULT SD_SPI_ioctl(BYTE cmd, void *buff);

#ifdef __cplusplus
}
#endif

#endif /* SD_SPI_DISKIO_H */
