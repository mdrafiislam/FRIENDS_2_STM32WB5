#include "diskio.h"
#include "sd_spi_diskio.h"

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != SD_SPI_DRIVE)
    {
        return STA_NOINIT;
    }

    /*
     * Force the custom SPI driver back to an uninitialized state
     * before every explicit disk initialization attempt.
     */
    SD_SPI_Reset();

    return SD_SPI_initialize();
}

DSTATUS disk_status(BYTE pdrv)
{
    return (pdrv == SD_SPI_DRIVE) ? SD_SPI_status() : STA_NOINIT;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    return (pdrv == SD_SPI_DRIVE) ? SD_SPI_read(buff, sector, count) : RES_PARERR;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    return (pdrv == SD_SPI_DRIVE) ? SD_SPI_write(buff, sector, count) : RES_PARERR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    return (pdrv == SD_SPI_DRIVE) ? SD_SPI_ioctl(cmd, buff) : RES_PARERR;
}
