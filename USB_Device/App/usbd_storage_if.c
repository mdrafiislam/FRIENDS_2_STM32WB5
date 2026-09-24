#include "usbd_storage_if.h"
#include "diskio.h"
#include "sd_spi_diskio.h"
#include <string.h>

#define STORAGE_LUN_NBR                  1U
#define STORAGE_BLK_SIZ                  512U

static int8_t STORAGE_Init_FS(uint8_t lun);
static int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
static int8_t STORAGE_IsReady_FS(uint8_t lun);
static int8_t STORAGE_IsWriteProtected_FS(uint8_t lun);
static int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_GetMaxLun_FS(void);

static int8_t STORAGE_Inquirydata_FS[] = {
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5U),
  0x00,
  0x00,
  0x00,
  'F', 'R', 'I', 'E', 'N', 'D', 'S', ' ',
  'S', 'D', ' ', 'C', 'a', 'r', 'd', ' ',
  'S', 't', 'o', 'r', 'a', 'g', 'e', ' ',
  '1', '.', '0', '0'
};

USBD_StorageTypeDef USBD_Storage_Interface_fops_FS = {
  STORAGE_Init_FS,
  STORAGE_GetCapacity_FS,
  STORAGE_IsReady_FS,
  STORAGE_IsWriteProtected_FS,
  STORAGE_Read_FS,
  STORAGE_Write_FS,
  STORAGE_GetMaxLun_FS,
  STORAGE_Inquirydata_FS
};

static int8_t STORAGE_Init_FS(uint8_t lun)
{
  (void)lun;
  return (disk_initialize(SD_SPI_DRIVE) == 0U) ? 0 : -1;
}

static int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  DWORD sector_count = 0U;

  if ((lun >= STORAGE_LUN_NBR) || (block_num == NULL) || (block_size == NULL)) {
    return -1;
  }

  if (disk_ioctl(SD_SPI_DRIVE, GET_SECTOR_COUNT, &sector_count) != RES_OK) {
    return -1;
  }

  *block_num = (sector_count > 0U) ? (uint32_t)(sector_count - 1U) : 0U;
  *block_size = STORAGE_BLK_SIZ;
  return 0;
}

static int8_t STORAGE_IsReady_FS(uint8_t lun)
{
  if (lun >= STORAGE_LUN_NBR) {
    return -1;
  }

  return (disk_status(SD_SPI_DRIVE) == 0U) ? 0 : -1;
}

static int8_t STORAGE_IsWriteProtected_FS(uint8_t lun)
{
  (void)lun;
  return 0;
}

static int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  if ((lun >= STORAGE_LUN_NBR) || (buf == NULL)) {
    return -1;
  }

  return (disk_read(SD_SPI_DRIVE, buf, blk_addr, blk_len) == RES_OK) ? 0 : -1;
}

static int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  if ((lun >= STORAGE_LUN_NBR) || (buf == NULL)) {
    return -1;
  }

  return (disk_write(SD_SPI_DRIVE, buf, blk_addr, blk_len) == RES_OK) ? 0 : -1;
}

static int8_t STORAGE_GetMaxLun_FS(void)
{
  return (STORAGE_LUN_NBR - 1U);
}
