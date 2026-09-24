/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/p2p_server_app.c
  * @author  MCD Application Team
  * @brief   Peer to peer Server Application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "p2p_server_app.h"
#include "stm32_seq.h"
#include "ble_notify_len.h"   /* USER: shared notify length, see that header */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Notification period. 1000 = 1Hz for bring up, drop to 50 for 20 Hz (one notification per magnetomenter sample)
#define P2PS_APP_NOTIFY_PERIOD_MS 50U
/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint8_t counter = 0U;
static uint32_t last_send_ms = 0U;
static uint8_t notifications_enabled = 0U;
static uint16_t sequence = 0U;

typedef struct __attribute__((packed))
{
	uint8_t version;
	uint8_t flags;
	uint16_t sequence;
	uint32_t timestamp_ms;

	int16_t s1_x;
	int16_t s1_y;
	int16_t s1_z;

	int16_t s2_x;
	int16_t s2_y;
	int16_t s2_z;

	/* v3: zeroed vector difference (s1 - s2) - boot baseline, per axis.
	 * s1/s2 above stay RAW - this is an addition, not a replacement, so
	 * sphere fits and per-sensor inspection still work over BLE.
	 * Valid only when flags bit1 is set; until the baseline is captured
	 * these carry the raw s1 - s2 pass-through. See Core/Inc/mag_zero.h. */
	int16_t z_x;
	int16_t z_y;
	int16_t z_z;
}BleMagPacket;

typedef struct __attribute__((packed))
{
    uint8_t  version;        /* 2 = puff event */
    uint8_t  flags;          /* bit0: 0 = START, 1 = END */
    uint16_t puff_id;
    uint32_t start_ms;
    uint16_t duration_ms;
    uint16_t mod_hz;         /* duty-cycle modulation, 0 if none */
    uint16_t carrier_hz;     /* edges/s, ~20000 if a converter is present */
    uint16_t therm1_raw;
    uint16_t therm2_raw;
    uint16_t reserved;
    /* Padding to BLE_NOTIFY_PACKET_LEN. Both packets go out through the same
     * fixed-length P2PS_STM_App_Update_Char(), so a shorter struct here would
     * make that call read past the end of this object. Sized, not guessed. */
    uint8_t  reserved2[BLE_NOTIFY_PACKET_LEN - 20U];
} BlePuffPacket;

static BlePuffPacket pending_puff;
static uint8_t pending_puff_valid = 0U;


// Both packets travel through the same fixed-length P2PS_STM_App_Update_Char(),
// whose length is BLE_NOTIFY_PACKET_LEN (Core/Inc/ble_notify_len.h). If either
// struct ever stops matching it the build breaks here, instead of silently
// truncating the packet or reading past the end of the struct.
typedef char BleMagPacket_size_must_match_notify_len[
    (sizeof(BleMagPacket) == BLE_NOTIFY_PACKET_LEN) ? 1 : -1
];
typedef char BlePuffPacket_size_must_match_notify_len[
    (sizeof(BlePuffPacket) == BLE_NOTIFY_PACKET_LEN) ? 1 : -1
];

static BleMagPacket latest_sample;
static uint8_t latest_sample_valid = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
// Writes latest sample
void P2PS_APP_SetMagSample(int16_t s1_x, int16_t s1_y, int16_t s1_z,
                           int16_t s2_x, int16_t s2_y, int16_t s2_z,
                           int16_t z_x, int16_t z_y, int16_t z_z,
                           uint8_t baseline_valid);

void P2PS_APP_SetPuffEvent(uint8_t is_end, uint16_t puff_id, uint32_t start_ms,
                           uint16_t duration_ms, uint16_t mod_hz, uint16_t carrier_hz,
                           uint16_t t1, uint16_t t2);

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_STM_App_Notification_1 */

/* USER CODE END P2PS_STM_App_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_STM_App_Notification_P2P_Evt_Opcode */

/* USER CODE END P2PS_STM_App_Notification_P2P_Evt_Opcode */

    case P2PS_STM__NOTIFY_ENABLED_EVT:
/* USER CODE BEGIN P2PS_STM__NOTIFY_ENABLED_EVT */
    	notifications_enabled = 1U;
/* USER CODE END P2PS_STM__NOTIFY_ENABLED_EVT */
      break;

    case P2PS_STM_NOTIFY_DISABLED_EVT:
/* USER CODE BEGIN P2PS_STM_NOTIFY_DISABLED_EVT */
    	notifications_enabled = 0U;
/* USER CODE END P2PS_STM_NOTIFY_DISABLED_EVT */
      break;

    case P2PS_STM_WRITE_EVT:
/* USER CODE BEGIN P2PS_STM_WRITE_EVT */

/* USER CODE END P2PS_STM_WRITE_EVT */
      break;

    default:
/* USER CODE BEGIN P2PS_STM_App_Notification_default */

/* USER CODE END P2PS_STM_App_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_STM_App_Notification_2 */

/* USER CODE END P2PS_STM_App_Notification_2 */
  return;
}

void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_APP_Notification_1 */

/* USER CODE END P2PS_APP_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_APP_Notification_P2P_Evt_Opcode */

/* USER CODE END P2PS_APP_Notification_P2P_Evt_Opcode */
  case PEER_CONN_HANDLE_EVT :
/* USER CODE BEGIN PEER_CONN_HANDLE_EVT */

/* USER CODE END PEER_CONN_HANDLE_EVT */
    break;

    case PEER_DISCON_HANDLE_EVT :
/* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */
notifications_enabled = 0U;
latest_sample_valid = 0U;
/* USER CODE END PEER_DISCON_HANDLE_EVT */
    break;

    default:
/* USER CODE BEGIN P2PS_APP_Notification_default */

/* USER CODE END P2PS_APP_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_APP_Notification_2 */

/* USER CODE END P2PS_APP_Notification_2 */
  return;
}

void P2PS_APP_Init(void)
{
/* USER CODE BEGIN P2PS_APP_Init */

/* USER CODE END P2PS_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void P2PS_APP_Process(void)
{
    uint32_t now_ms = HAL_GetTick();
    if (notifications_enabled == 0U) { return; }

    if (pending_puff_valid != 0U)
    {
        if (P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&pending_puff) == BLE_STATUS_SUCCESS)
        {
            pending_puff_valid = 0U;
        }
        return;   /* one notification per pass; mag sample goes next time */
    }

    if (latest_sample_valid == 0U) { return; }
    if ((uint32_t)(now_ms - last_send_ms) < P2PS_APP_NOTIFY_PERIOD_MS) { return; }

  latest_sample.sequence = sequence;

	tBleStatus status = P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&latest_sample);

	if (status == BLE_STATUS_SUCCESS)
  // Advance only on success. If CPU2's TX pool was full, we leave
  // last_send_ms alone so the next loop pass retries immediately
  // instead of dropping the sample.
	{
		sequence++;
    last_send_ms = now_ms;
	}
}

void P2PS_APP_SetMagSample(int16_t s1_x, int16_t s1_y, int16_t s1_z,
                           int16_t s2_x, int16_t s2_y, int16_t s2_z,
                           int16_t z_x, int16_t z_y, int16_t z_z,
                           uint8_t baseline_valid) {
  latest_sample.version = 3U;   /* v3: adds z_x,z_y,z_z; v1 was 20 bytes, v3 is 26 */
  /* bit0 stays reserved for puff_active, as in v1. bit1 is new: it tells the
   * host whether z_* is baseline-corrected or still the raw s1 - s2
   * pass-through, so a capture started before the baseline completes is not
   * silently mistaken for zeroed data. */
  latest_sample.flags = (uint8_t)((baseline_valid != 0U) ? 0x02U : 0x00U);
  latest_sample.timestamp_ms = HAL_GetTick();
  latest_sample.s1_x = s1_x;
  latest_sample.s1_y = s1_y;
  latest_sample.s1_z = s1_z;
  latest_sample.s2_x = s2_x;
  latest_sample.s2_y = s2_y;
  latest_sample.s2_z = s2_z;
  latest_sample.z_x  = z_x;
  latest_sample.z_y  = z_y;
  latest_sample.z_z  = z_z;

  latest_sample_valid = 1U;
}

void P2PS_APP_SetPuffEvent(uint8_t is_end, uint16_t puff_id, uint32_t start_ms,
                           uint16_t duration_ms, uint16_t mod_hz, uint16_t carrier_hz,
                           uint16_t t1, uint16_t t2)
{
    pending_puff.version     = 2U;
    pending_puff.flags       = is_end ? 1U : 0U;
    pending_puff.puff_id     = puff_id;
    pending_puff.start_ms    = start_ms;
    pending_puff.duration_ms = duration_ms;
    pending_puff.mod_hz      = mod_hz;
    pending_puff.carrier_hz  = carrier_hz;
    pending_puff.therm1_raw  = t1;
    pending_puff.therm2_raw  = t2;
    pending_puff.reserved    = 0U;
    pending_puff_valid = 1U;
}

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/


/* USER CODE END FD_LOCAL_FUNCTIONS*/
