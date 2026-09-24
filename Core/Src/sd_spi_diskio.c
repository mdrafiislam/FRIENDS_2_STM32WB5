#include "sd_spi_diskio.h"
#include "main.h"
#include <string.h>

extern SPI_HandleTypeDef hspi2;

#define SD_DUMMY_BYTE          0xFFU
#define SD_BLOCK_SIZE          512U
#define SD_CMD_TIMEOUT_MS      500U
#define SD_READ_TIMEOUT_MS     500U
#define SD_WRITE_TIMEOUT_MS    500U

#define SD_CMD0                0U
#define SD_CMD1                1U
#define SD_CMD8                8U
#define SD_CMD9                9U
#define SD_CMD12               12U
#define SD_CMD16               16U
#define SD_CMD17               17U
#define SD_CMD18               18U
#define SD_CMD24               24U
#define SD_CMD25               25U
#define SD_CMD55               55U
#define SD_CMD58               58U
#define SD_ACMD41              41U

#define SD_TOKEN_START_BLOCK   0xFEU
#define SD_TOKEN_MULTI_WRITE   0xFCU
#define SD_TOKEN_STOP_TRAN     0xFDU

#define SD_TYPE_MMC            0x01U
#define SD_TYPE_SD1            0x02U
#define SD_TYPE_SD2            0x04U
#define SD_TYPE_BLOCK          0x08U

static volatile DSTATUS sd_status = STA_NOINIT;
static uint8_t sd_card_type = 0U;

static void SD_CS_Select(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

static void SD_CS_Deselect(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

static uint8_t SD_SPI_TxRx(uint8_t data)
{
    uint8_t rx = SD_DUMMY_BYTE;

    (void)HAL_SPI_TransmitReceive(&hspi2, &data, &rx, 1U, 100U);
    return rx;
}

static void SD_SPI_ClockBytes(uint16_t count)
{
    while (count-- > 0U) {
        (void)SD_SPI_TxRx(SD_DUMMY_BYTE);
    }
}

static HAL_StatusTypeDef SD_SPI_ReceiveBytes(uint8_t *buff, uint16_t len, uint32_t timeout_ms)
{
    uint8_t tx = SD_DUMMY_BYTE;

    while (len-- > 0U) {
        if (HAL_SPI_TransmitReceive(&hspi2, &tx, buff, 1U, timeout_ms) != HAL_OK) {
            return HAL_ERROR;
        }
        buff++;
    }

    return HAL_OK;
}

static void SD_SPI_SetPrescaler(uint32_t prescaler)
{
    (void)HAL_SPI_DeInit(&hspi2);
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.BaudRatePrescaler = prescaler;
    (void)HAL_SPI_Init(&hspi2);
}

static uint8_t SD_WaitReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t response;

    do {
        response = SD_SPI_TxRx(SD_DUMMY_BYTE);
        if (response == 0xFFU) {
            return 1U;
        }
    } while ((HAL_GetTick() - start) < timeout_ms);

    return 0U;
}

static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
    uint8_t response;
    uint8_t packet[6];

    if (cmd & 0x80U) {
        cmd &= 0x7FU;
        response = SD_SendCmd(SD_CMD55, 0U);
        if (response > 1U) {
            return response;
        }
    }

    SD_CS_Deselect();
    SD_SPI_TxRx(SD_DUMMY_BYTE);
    SD_CS_Select();
    if (SD_WaitReady(SD_CMD_TIMEOUT_MS) == 0U) {
        return 0xFFU;
    }

    packet[0] = (uint8_t)(0x40U | cmd);
    packet[1] = (uint8_t)(arg >> 24);
    packet[2] = (uint8_t)(arg >> 16);
    packet[3] = (uint8_t)(arg >> 8);
    packet[4] = (uint8_t)arg;
    packet[5] = 0x01U;
    if (cmd == SD_CMD0) {
        packet[5] = 0x95U;
    } else if (cmd == SD_CMD8) {
        packet[5] = 0x87U;
    }

    (void)HAL_SPI_Transmit(&hspi2, packet, sizeof(packet), 100U);
    if (cmd == SD_CMD12) {
        SD_SPI_TxRx(SD_DUMMY_BYTE);
    }

    for (uint8_t attempts = 10U; attempts > 0U; attempts--) {
        response = SD_SPI_TxRx(SD_DUMMY_BYTE);
        if ((response & 0x80U) == 0U) {
            return response;
        }
    }

    return response;
}

static uint8_t SD_ReadDataBlock(uint8_t *buff, uint16_t len)
{
    uint32_t start = HAL_GetTick();
    uint8_t token;

    do {
        token = SD_SPI_TxRx(SD_DUMMY_BYTE);
        if (token == SD_TOKEN_START_BLOCK) {
            break;
        }
    } while ((HAL_GetTick() - start) < SD_READ_TIMEOUT_MS);

    if (token != SD_TOKEN_START_BLOCK) {
        return 0U;
    }

    if (SD_SPI_ReceiveBytes(buff, len, SD_READ_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }
    SD_SPI_TxRx(SD_DUMMY_BYTE);
    SD_SPI_TxRx(SD_DUMMY_BYTE);
    return 1U;
}

static uint8_t SD_WriteDataBlock(const uint8_t *buff, uint8_t token)
{
    uint8_t response;
    uint8_t crc[2] = { SD_DUMMY_BYTE, SD_DUMMY_BYTE };

    if (SD_WaitReady(SD_WRITE_TIMEOUT_MS) == 0U) {
        return 0U;
    }

    SD_SPI_TxRx(token);
    if (token != SD_TOKEN_STOP_TRAN) {
        (void)HAL_SPI_Transmit(&hspi2, (uint8_t *)buff, SD_BLOCK_SIZE, SD_WRITE_TIMEOUT_MS);
        (void)HAL_SPI_Transmit(&hspi2, crc, sizeof(crc), SD_WRITE_TIMEOUT_MS);
        response = SD_SPI_TxRx(SD_DUMMY_BYTE);
        if ((response & 0x1FU) != 0x05U) {
            return 0U;
        }
    }

    return 1U;
}

DSTATUS SD_SPI_initialize(void)
{
    uint8_t response;
    uint8_t ocr[4];
    uint32_t start;
    uint8_t type = 0U;

    SD_CS_Deselect();
    SD_SPI_SetPrescaler(SPI_BAUDRATEPRESCALER_256);

    /*
     * Allow a newly inserted card and its supply rail to settle.
     * The card may still be powered independently of the MCU.
     */
    HAL_Delay(100U);

    /* At least 74 clock pulses with CS high are required. */
    SD_SPI_ClockBytes(20U);

    uint8_t cmd0_response = 0xFFU;
    uint32_t cmd0_start = HAL_GetTick();

    do
    {
        cmd0_response = SD_SendCmd(SD_CMD0, 0U);

        if (cmd0_response == 1U)
        {
            break;
        }

        SD_CS_Deselect();
        SD_SPI_ClockBytes(2U);
        HAL_Delay(10U);

    } while ((HAL_GetTick() - cmd0_start) < 1000U);

    if (cmd0_response == 1U)
    {
        start = HAL_GetTick();
        if (SD_SendCmd(SD_CMD8, 0x1AAU) == 1U) {
            for (uint8_t i = 0U; i < 4U; i++) {
                ocr[i] = SD_SPI_TxRx(SD_DUMMY_BYTE);
            }
            if ((ocr[2] == 0x01U) && (ocr[3] == 0xAAU)) {
                do {
                    response = SD_SendCmd(0x80U | SD_ACMD41, 0x40000000U);
                } while ((response != 0U) && ((HAL_GetTick() - start) < 1000U));

                if ((response == 0U) && (SD_SendCmd(SD_CMD58, 0U) == 0U)) {
                    for (uint8_t i = 0U; i < 4U; i++) {
                        ocr[i] = SD_SPI_TxRx(SD_DUMMY_BYTE);
                    }
                    type = (ocr[0] & 0x40U) ? (SD_TYPE_SD2 | SD_TYPE_BLOCK) : SD_TYPE_SD2;
                }
            }
        } else {
            if (SD_SendCmd(0x80U | SD_ACMD41, 0U) <= 1U) {
                type = SD_TYPE_SD1;
                do {
                    response = SD_SendCmd(0x80U | SD_ACMD41, 0U);
                } while ((response != 0U) && ((HAL_GetTick() - start) < 1000U));
            } else {
                type = SD_TYPE_MMC;
                do {
                    response = SD_SendCmd(SD_CMD1, 0U);
                } while ((response != 0U) && ((HAL_GetTick() - start) < 1000U));
            }
            if ((response != 0U) || (SD_SendCmd(SD_CMD16, SD_BLOCK_SIZE) != 0U)) {
                type = 0U;
            }
        }
    }

    sd_card_type = type;
    SD_CS_Deselect();
    SD_SPI_TxRx(SD_DUMMY_BYTE);

    if (type != 0U) {
        sd_status &= (DSTATUS)~STA_NOINIT;
        SD_SPI_SetPrescaler(SPI_BAUDRATEPRESCALER_8);
    } else {
        sd_status = STA_NOINIT;
    }

    return sd_status;
}

DSTATUS SD_SPI_status(void)
{
    return sd_status;
}

DRESULT SD_SPI_read(BYTE *buff, LBA_t sector, UINT count)
{
    if ((buff == NULL) || (count == 0U)) {
        return RES_PARERR;
    }
    if (sd_status & STA_NOINIT) {
        return RES_NOTRDY;
    }
    if ((sd_card_type & SD_TYPE_BLOCK) == 0U) {
        sector *= SD_BLOCK_SIZE;
    }

    if (count == 1U) {
        if ((SD_SendCmd(SD_CMD17, (uint32_t)sector) == 0U) && SD_ReadDataBlock(buff, SD_BLOCK_SIZE)) {
            count = 0U;
        }
    } else {
        if (SD_SendCmd(SD_CMD18, (uint32_t)sector) == 0U) {
            do {
                if (SD_ReadDataBlock(buff, SD_BLOCK_SIZE) == 0U) {
                    break;
                }
                buff += SD_BLOCK_SIZE;
            } while (--count > 0U);
            (void)SD_SendCmd(SD_CMD12, 0U);
        }
    }

    SD_CS_Deselect();
    SD_SPI_TxRx(SD_DUMMY_BYTE);
    return (count == 0U) ? RES_OK : RES_ERROR;
}

DRESULT SD_SPI_write(const BYTE *buff, LBA_t sector, UINT count)
{
    if ((buff == NULL) || (count == 0U)) {
        return RES_PARERR;
    }
    if (sd_status & STA_NOINIT) {
        return RES_NOTRDY;
    }
    if ((sd_card_type & SD_TYPE_BLOCK) == 0U) {
        sector *= SD_BLOCK_SIZE;
    }

    if (count == 1U) {
        if ((SD_SendCmd(SD_CMD24, (uint32_t)sector) == 0U) && SD_WriteDataBlock(buff, SD_TOKEN_START_BLOCK)) {
            count = 0U;
        }
    } else {
        if ((sd_card_type & SD_TYPE_SD1) || (sd_card_type & SD_TYPE_SD2)) {
            (void)SD_SendCmd(0x80U | 23U, count);
        }
        if (SD_SendCmd(SD_CMD25, (uint32_t)sector) == 0U) {
            do {
                if (SD_WriteDataBlock(buff, SD_TOKEN_MULTI_WRITE) == 0U) {
                    break;
                }
                buff += SD_BLOCK_SIZE;
            } while (--count > 0U);
            if (SD_WriteDataBlock(NULL, SD_TOKEN_STOP_TRAN) == 0U) {
                count = 1U;
            }
        }
    }

    SD_CS_Deselect();
    SD_SPI_TxRx(SD_DUMMY_BYTE);
    return (count == 0U) ? RES_OK : RES_ERROR;
}

DRESULT SD_SPI_ioctl(BYTE cmd, void *buff)
{
    DRESULT result = RES_ERROR;
    uint8_t csd[16];
    DWORD sector_count;

    if (sd_status & STA_NOINIT) {
        return RES_NOTRDY;
    }

    switch (cmd) {
    case CTRL_SYNC:
        SD_CS_Select();
        result = SD_WaitReady(SD_WRITE_TIMEOUT_MS) ? RES_OK : RES_ERROR;
        SD_CS_Deselect();
        SD_SPI_TxRx(SD_DUMMY_BYTE);
        break;

    case GET_SECTOR_SIZE:
        *(WORD *)buff = SD_BLOCK_SIZE;
        result = RES_OK;
        break;

    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 1U;
        result = RES_OK;
        break;

    case GET_SECTOR_COUNT:
        if ((buff != NULL) && (SD_SendCmd(SD_CMD9, 0U) == 0U) && SD_ReadDataBlock(csd, sizeof(csd))) {
            if ((csd[0] >> 6) == 1U) {
                sector_count = (DWORD)(csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63U) << 16) + 1U) << 10;
            } else {
                uint8_t n = (uint8_t)((csd[5] & 15U) + ((csd[10] & 128U) >> 7) + ((csd[9] & 3U) << 1) + 2U);
                sector_count = (DWORD)(csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3U) << 10) + 1U;
                sector_count <<= (n - 9U);
            }
            *(DWORD *)buff = sector_count;
            result = RES_OK;
        }
        SD_CS_Deselect();
        SD_SPI_TxRx(SD_DUMMY_BYTE);
        break;

    default:
        result = RES_PARERR;
        break;
    }

    return result;
}

void SD_SPI_Reset(void)
{
    sd_status = STA_NOINIT;
    sd_card_type = 0U;

    SD_CS_Deselect();

    (void)HAL_SPI_DeInit(&hspi2);
    HAL_Delay(2U);
    (void)HAL_SPI_Init(&hspi2);

    SD_CS_Deselect();
}
