/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ff.h"
#include "usbd_cdc_if.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "diskio.h"
#include "p2p_server_app.h" // BLE notify functions
#include "mag_zero.h"       // boot-time zeroing of the s1 - s2 vector difference
#include "boot_dfu.h"       // 'b' command: software entry into the USB DFU bootloader
#include "usbd_core.h"      // USBD_Stop/USBD_DeInit for a clean detach before DFU
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

IPCC_HandleTypeDef hipcc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile int16_t x1, sensor1_y, z1, x2, y2, z2;
static uint8_t usb_startup_sent = 0U;

// Previous serial_sensor_print_enabled = 0U flag removed when BLE was implemented
// Replaced with an enum to split the streaming/printing magnetometer data command
// IMPORTANT: switch modes only with the plot stopped. SerialPlot binds its channel 
// count at capture start, so changing column count mid-stream garbles it.
typedef enum {
  STREAM_OFF = 0, /* 'x' */
  STREAM_FULL,    /* 'p' - 10 channels, the SerialPlot bench view */
  STREAM_MAG,     /* 'm' magnitudes only, quick compare */
  STREAM_VEC      /* 'v' - 3 channels, signed zeroed axes z_x,z_y,z_z */
} SerialStreamMode;
static SerialStreamMode serial_stream_mode = STREAM_OFF;

static uint32_t usb_status_last_ms = 0U;

/* Mag-delta zeroing. mag_zero_announced gates the one-shot CDC report so the
 * baseline is printed exactly once per capture rather than every loop pass. */
static uint8_t mag_zero_announced = 0U;

// SD Card Variables
static FATFS sd_fs;
static FIL sd_puff_file;
static FIL sd_mag_file;
static uint32_t current_puff_id = 0U;
static uint8_t sd_faulted = 0U;
static uint8_t sd_mounted = 0U;
static uint8_t sd_logging = 0U;
static uint32_t sd_last_sync_ms = 0U;

#define MAG_BUF_LEN 64U

typedef struct {
	char timestamp[32];
	uint32_t ms;
	uint32_t puff_id;
	uint32_t puff_active;
	int16_t s1_x, s1_y, s1_z;
	uint32_t s1_mag;
	int16_t s2_x, s2_y, s2_z;
	uint32_t s2_mag;
	int16_t z_x, z_y, z_z;      /* (s1 - s2) - boot baseline, per axis. Earth cancels in
	                             * s1 - s2; the baseline removes the residual fixed
	                             * offset difference b1 - b2. See mag_zero.h. */
	uint32_t z_mag;             /* |(s1 - s2) - baseline|, the differential the two-sensor
	                             * design is built on. Non-negative by construction. */
}MagLogSample;

// Magnetometer data buffers
static MagLogSample mag_buffer1[MAG_BUF_LEN];
static MagLogSample mag_buffer2[MAG_BUF_LEN];
static MagLogSample *mag_fill_buf = mag_buffer1;
static MagLogSample *mag_write_buf = mag_buffer2;
static uint32_t mag_fill_count = 0U;
static uint32_t mag_write_count = 0U;
static uint8_t mag_buffer_ready = 0U;
static uint8_t mag_overrun = 0U;

// Puff Detection Variables
volatile uint8_t puff_active = 0U;
volatile uint8_t puff_started = 0U;
volatile uint8_t puff_ended = 0U;
volatile uint32_t puff_start_ms = 0U;
volatile uint32_t puff_end_ms = 0U;
volatile uint32_t puff_duration_ms = 0U;
volatile uint32_t last_rf_edge_ms = 0U;
volatile uint32_t rf_edge_count = 0U;
volatile uint32_t rf_rising_count = 0U;
volatile uint32_t rf_falling_count = 0U;
volatile uint32_t last_capture = 0U;

/* RF Correction Variables */
#define RF_GAP_TICKS 8192U
volatile uint32_t rf_gap_count = 0U;
volatile uint32_t rf_overcapture = 0U;
volatile uint32_t rf_latency_max = 0U;
volatile uint32_t rf_per_min = 0xFFFFFFFFU;
volatile uint32_t rf_per_max = 0U;

// Duty cycle measurement
#define RF_DUTY_HYST 8
volatile uint32_t rf_duty_min = 0xFFFFFFFFU;
volatile uint32_t rf_duty_max = 0U;
volatile int32_t rf_duty_acc = 0;
volatile uint32_t rf_duty_cross = 0U;
volatile uint8_t rf_duty_state = 0U;
// 0 idle, 1 skip first cycle, 2 seed the mean, 3 collecting
volatile uint8_t rf_meas_state = 0U;

static uint32_t puff_start_edge_count = 0U;
static uint32_t puff_start_rising_count = 0U;
static uint32_t puff_start_falling_count = 0U;

#define RF_INACTIVITY_TIMEOUT_MS 1000U

/* Thermistor Variables*/
// Mirroring the old MSP design: each divider is powered by
// a GPIO enable line (active-high) only during a reading,
// the powered back off.
//

#define THERM1_ADC_CHANNEL ADC_CHANNEL_5
#define THERM2_ADC_CHANNEL ADC_CHANNEL_4
#define THERM_SETTLE_MS 2U // divider power on settle before sampling
#define THERM_ADC_TIMEOUT_MS 10U // time wait for one ADC conversion before giving up, prevents hang

static uint16_t therm1_start = 0U, therm1_end = 0U;
static uint16_t therm2_start = 0U, therm2_end = 0U;

/**
  * @brief Claim CLK48 for USB so the BLE stack on CPU2 cannot take it away.
  *
  * CLK48 is a single 48 MHz domain feeding BOTH the USB peripheral and the RNG.
  * CPU1 owns PLLSAI1; CPU2 owns HSI48 and needs the RNG for BLE crypto and
  * random addresses. Once the BLE stack is up, CPU2 asserts ownership: it
  * resets CCIPR.CLK48SEL to its default (00 = HSI48) and powers HSI48 down.
  *
  * Measured on this board with the stock CubeMX config (USB on PLLSAI1):
  *
  *   after MX_USB_Device_Init   CCIPR 0x14000000 (PLLSAI1)  CRRCR 0x8500  ok
  *   after MX_APPE_Init         CCIPR 0x14000000 (PLLSAI1)  CRRCR 0x8500  ok
  *   +3 s, BLE stack running    CCIPR 0x10000000 (HSI48)    CRRCR 0x8500  DEAD
  *
  * HSI48ON/HSI48RDY (CRRCR bits 0/1) are clear throughout, so USB does not get
  * an inaccurate clock - it gets none at all. The D+ pull-up is asserted by
  * USBD_Start() and does not need CLK48, so the host still sees a device, but
  * EP0 cannot answer: Windows reports "Device Descriptor Request Failed".
  *
  * Switching USB to HSI48 + CRS does NOT work: CPU2 powers that oscillator off
  * (CRRCR goes 0x8503 -> 0x8500 at the 3 s mark). PLLSAI1 is the right target
  * precisely because CPU2 has no interest in it.
  *
  * hw_conf.h documents the arbitration mechanism:
  *   "Index of the semaphore used to manage the CLK48 clock configuration.
  *    When the USB is required, this semaphore shall be taken before
  *    configuring the CLK48 for USB."
  *
  * Call AFTER PeriphCommonClock_Config() and BEFORE MX_USB_Device_Init().
  * ADC is unaffected - it stays on PLLSAI1_R (CCIPR.ADCSEL never moves).
  */
static void USB_ClockConfig_ClaimCLK48(void)
{
  /* Tell CPU2 that CPU1 owns the CLK48 configuration. Held for the lifetime of
   * the application - USB is always enumerated in this design. */
  (void)HAL_HSEM_FastTake(CFG_HW_CLK48_CONFIG_SEMID);

  __HAL_RCC_USB_CONFIG(RCC_USBCLKSOURCE_PLLSAI1);
}

/**
  * @brief Belt-and-braces: restore CLK48SEL if anything clears it.
  *
  * One register read per main-loop pass. Verified NOT load-bearing - an
  * instrumented counter stayed at zero across long runs with BLE connected, so
  * the semaphore alone is holding CPU2 off. Kept as cheap insurance because
  * CPU2 is a closed binary that ST updates, and this failure mode is invisible
  * until someone plugs in USB.
  */
static void USB_ClockGuard(void)
{
  if (__HAL_RCC_GET_USB_SOURCE() != RCC_USBCLKSOURCE_PLLSAI1)
  {
    __HAL_RCC_USB_CONFIG(RCC_USBCLKSOURCE_PLLSAI1);
  }
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
static void MX_IPCC_Init(void);
static void MX_RF_Init(void);
/* USER CODE BEGIN PFP */
static void CS1_Select(void);
static void CS1_Deselect(void);
static void CS2_Select(void);
static void CS2_Deselect(void);
static HAL_StatusTypeDef LIS2MDL_DebugWhoAmI(const char *name,
    GPIO_TypeDef *cs_port, uint16_t cs_pin, volatile uint8_t *who_am_i,
    void (*cs_select)(void), void (*cs_deselect)(void));
static uint32_t LIS2MDL_MagnitudeRaw(int16_t x, int16_t y, int16_t z);

static void SDLog_ProcessCommand(void);
static void SDLog_Start(void);
static void SDLog_Stop(const char *reason);
static void SDLog_Eject(void);
static void Cmd_EnterBootloader(void);
static void SDLog_Abort(const char *operration, FRESULT result);

static void SDLog_WriteSample(uint32_t now_ms,
    int16_t s1_x, int16_t s1_y, int16_t s1_z, uint32_t s1_mag,
    int16_t s2_x, int16_t s2_y, int16_t s2_z, uint32_t s2_mag,
    int16_t z_x, int16_t z_y, int16_t z_z, uint32_t z_mag);
static void SDLog_WritePuffEvent(
		const char *event,
		uint32_t start_ms,
		uint32_t end_ms,
		uint32_t duration_ms,
		uint32_t edges,
		uint32_t rising,
		uint32_t falling,
		uint32_t t1_raw,
		uint32_t t2_raw);

static FRESULT SDLog_Mount(void);

static void SDLog_Service(void);

static FRESULT SDLog_NextFileName(char *name, size_t name_size);
static FRESULT SDLog_FindNextRun(char *puff_name, size_t puff_size, char *mag_name, size_t mag_size);
static void SDLog_FlushMagBuffers(void);

static void SDLog_PrintResult(const char *operation, FRESULT result);
static HAL_StatusTypeDef USB_CDC_Print(const char *text);

/* Puff Operations */
static void start_puff(void);
static void end_puff(void);
static void PuffLogService(void);
static void testLeds(void);
static void testRF_status(void);
static void getTimestamp(char *buf, size_t len);
static void LED_IdlePurple(void);

/* Thermistor Read Helpers */
static uint16_t Thermistor_ReadChannel(uint32_t channel, GPIO_TypeDef *en_port, uint16_t en_pin);
static void Thermistor_ReadBoth(uint16_t *t1_raw, uint16_t *t2_raw);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Must stay the first statement of main(): the DFU jump relies on the chip
   * still being in post-reset state (no HAL_Init, no PLL, no SysTick, CPU2
   * not yet booted). Returns immediately on a normal boot. See boot_dfu.h. */
  BootDFU_CheckAndJump();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */
  /* Must run after PeriphCommonClock_Config() and before MX_USB_Device_Init().
   * Claims CLK48 via HSEM 5 so the BLE stack on CPU2 cannot power it down.
   * See USB_ClockConfig_ClaimCLK48() for the measurements behind this. */
  USB_ClockConfig_ClaimCLK48();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_USB_Device_Init();
  MX_RF_Init();

  /* USER CODE BEGIN 2 */
  // One time ADC1 self calibration before first use. Improves thermistor read accuracy, non fatal.
  // If fail, magnetometer/SD logging  still work
  (void)HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  // Override the automatically CubeMX generation, if it happens
  __HAL_TIM_SET_CAPTUREPOLARITY(
		  &htim2,
		  TIM_CHANNEL_1,
		  TIM_INPUTCHANNELPOLARITY_RISING);
  __HAL_TIM_SET_CAPTUREPOLARITY(
		  &htim2,
		  TIM_CHANNEL_2,
		  TIM_INPUTCHANNELPOLARITY_FALLING);

  /* Channel 2 captures without its own interrupt; channel 1 drives the ISR. */
  if (HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
      Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
      Error_Handler();
  }

  volatile uint8_t id1 = 0, id2 = 0;

  CS1_Deselect();
  CS2_Deselect();

  /* Enable separate SDO/MISO before the first read. LIS2MDL defaults to 3-wire SPI. */
  LIS2MDL_SPI_WriteReg(LIS2MDL_CFG_REG_C, LIS2MDL_CFG_REG_C_VAL, CS1_Select, CS1_Deselect);
  LIS2MDL_SPI_WriteReg(LIS2MDL_CFG_REG_C, LIS2MDL_CFG_REG_C_VAL, CS2_Select, CS2_Deselect);

  /* The LIS2MDL powers up in 3-wire SPI and the CFG_REG_C writes above switch
   * it to 4-wire. That first transaction after power-up is not always reliable,
   * so retry WHO_AM_I before concluding a part is missing. A single bad read
   * here used to skip LIS2MDL_Init() entirely, leaving the sensor in its
   * power-on idle state where it never sets ZYXDA - which reads downstream as
   * a permanent HAL_BUSY and silently gates off both USB streaming and BLE. */
  for (uint8_t attempt = 0U; attempt < 5U; attempt++)
  {
    if (id1 != LIS2MDL_WHO_AM_I) {
      LIS2MDL_SPI_WriteReg(LIS2MDL_CFG_REG_C, LIS2MDL_CFG_REG_C_VAL, CS1_Select, CS1_Deselect);
      LIS2MDL_DebugWhoAmI("sensor1", MAG_CS1_GPIO_Port, MAG_CS1_Pin, &id1, CS1_Select, CS1_Deselect);
    }
    if (id2 != LIS2MDL_WHO_AM_I) {
      LIS2MDL_SPI_WriteReg(LIS2MDL_CFG_REG_C, LIS2MDL_CFG_REG_C_VAL, CS2_Select, CS2_Deselect);
      LIS2MDL_DebugWhoAmI("sensor2", MAG_CS2_GPIO_Port, MAG_CS2_Pin, &id2, CS2_Select, CS2_Deselect);
    }
    if ((id1 == LIS2MDL_WHO_AM_I) && (id2 == LIS2MDL_WHO_AM_I)) { break; }
    HAL_Delay(10);
  }

  /* Initialise both unconditionally. A part that failed WHO_AM_I may still be
   * present and working; skipping init guarantees it never produces data. */
  LIS2MDL_Init(CS1_Select, CS1_Deselect);
  LIS2MDL_Init(CS2_Select, CS2_Deselect);

  /* Arm the mag-delta baseline capture. MagZero_Feed() self-arms if this is
   * missing, so this is explicitness rather than a requirement. */
  MagZero_Arm();
  mag_zero_announced = 0U;

  if ((id1 == LIS2MDL_WHO_AM_I) && (id2 == LIS2MDL_WHO_AM_I)) {
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  }


  // After both sensor initialization
  LED_IdlePurple();
  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_APPE_Process();

    /* USER CODE BEGIN 3 */
    USB_ClockGuard();     // restore CLK48SEL ownership to CPU1 if CPU2 clears it despite HSEM 5
    USB_Device_Service(); // switches to USB serial if in cdc mode
    {
      uint8_t restore_reason = USB_Device_TakeCDCRestoredMessage();
      if (restore_reason == 1U) {
        (void)USB_CDC_Print("USB mass storage ejected; COM port mode restored\r\n");
      } else if (restore_reason == 2U) {
        (void)USB_CDC_Print("USB disconnect detected; COM port mode restored\r\n");
      }
    }

    if (usb_startup_sent == 0U) {
      if (USB_CDC_Print("USB CDC startup test\r\n") == HAL_OK) {
        usb_startup_sent = 1U;
      }
    }
    SDLog_ProcessCommand(); // continually checks for commands typed over serial

    /* Reads both sensors */
    HAL_StatusTypeDef s1 = LIS2MDL_ReadXYZ((int16_t *)&x1, (int16_t *)&sensor1_y, (int16_t *)&z1, CS1_Select, CS1_Deselect);
    HAL_StatusTypeDef s2 = LIS2MDL_ReadXYZ((int16_t *)&x2, (int16_t *)&y2, (int16_t *)&z2, CS2_Select, CS2_Deselect);

    /* LIS2MDL_ReadXYZ returns HAL_BUSY to mean "no new sample yet", not an SPI
     * error. Brief HAL_BUSY is normal - this loop polls at 20 Hz and the ODR is
     * also 20 Hz, so we sometimes ask a hair early. Sustained HAL_BUSY means the
     * part has fallen out of continuous mode, so re-init instead of requiring a
     * reflash. Everything downstream is gated on both reads being HAL_OK. */
    {
      static uint32_t s1_busy_since = 0U, s2_busy_since = 0U;
      uint32_t busy_now = HAL_GetTick();

      /* A re-init restarts that part's offset-cancellation state, which can
       * shift b1 - b2 and invalidate the captured baseline. Deliberately NOT
       * auto-rebaselining here: that would silently move the reference mid-run,
       * and if a target is present at that moment it would be absorbed into the
       * new zero. Warn and let the operator decide when to send 'z'. */
      if (s1 == HAL_BUSY) {
        if (s1_busy_since == 0U) { s1_busy_since = busy_now; }
        else if ((uint32_t)(busy_now - s1_busy_since) > 1000U) {
          LIS2MDL_Init(CS1_Select, CS1_Deselect);
          s1_busy_since = 0U;
          if (MagZero_IsReady() != 0U) {
            (void)USB_CDC_Print("sensor1 re-init; mag delta baseline may be stale, send z to re-zero\r\n");
          }
        }
      } else { s1_busy_since = 0U; }

      if (s2 == HAL_BUSY) {
        if (s2_busy_since == 0U) { s2_busy_since = busy_now; }
        else if ((uint32_t)(busy_now - s2_busy_since) > 1000U) {
          LIS2MDL_Init(CS2_Select, CS2_Deselect);
          s2_busy_since = 0U;
          if (MagZero_IsReady() != 0U) {
            (void)USB_CDC_Print("sensor2 re-init; mag delta baseline may be stale, send z to re-zero\r\n");
          }
        }
      } else { s2_busy_since = 0U; }
    }

    if ((s1 == HAL_OK) && (s2 == HAL_OK))
    {
      uint32_t now_ms = HAL_GetTick();

      /* P2PS_APP_SetMagSample() has moved below, after the zeroed delta is
       * computed - it now carries z_x,z_y,z_z as well and cannot run before
       * they exist. */

      uint32_t mag1 = LIS2MDL_MagnitudeRaw((int16_t)x1, (int16_t)sensor1_y, (int16_t)z1);
      uint32_t mag2 = LIS2MDL_MagnitudeRaw((int16_t)x2, (int16_t)y2, (int16_t)z2);

      int32_t mag12_diff = (int32_t)mag1 - (int32_t)mag2;   /* |s1| - |s2|: magnitude difference */
      int32_t mag21_diff = (int32_t)mag2 - (int32_t)mag1;

      /* Vector difference s1 - s2. Earth is common to both sensors and cancels;
       * what remains is the fixed offset difference plus any nearby source.
       * Saturating rather than casting: if one part rails near a strong magnet
       * while the other does not, the difference spans +/-65534 and a plain
       * (int16_t) cast wraps it to a large value of the wrong sign, which would
       * show up as a spurious spike in z_mag. */
      int16_t d_x = MagZero_Sat16((int32_t)x1 - (int32_t)x2);
      int16_t d_y = MagZero_Sat16((int32_t)sensor1_y - (int32_t)y2);
      int16_t d_z = MagZero_Sat16((int32_t)z1 - (int32_t)z2);

      /* Boot zeroing. s1 - s2 = (k1-k2)*B + (b1-b2); the (b1-b2) term is a
       * constant vector because the ambient field B is common to both parts, so
       * a single baseline stays valid as the board is moved. The (k1-k2)*B term
       * does not cancel - see mag_zero.h for the magnitude of that residual and
       * for the axis-alignment ASSUMPTION this relies on. */
      if ((MagZero_Feed(d_x, d_y, d_z) != 0U) && (mag_zero_announced == 0U)) {
        int16_t b_x, b_y, b_z;
        char zero_line[96];
        int zero_len;
        MagZero_GetBaseline(&b_x, &b_y, &b_z);
        zero_len = snprintf(zero_line, sizeof(zero_line),
                            "Mag delta zeroed: baseline %d,%d,%d LSB\r\n",
                            b_x, b_y, b_z);
        if ((zero_len > 0) && ((size_t)zero_len < sizeof(zero_line))) {
          (void)USB_CDC_Print(zero_line);
        }
        mag_zero_announced = 1U;
      }

      /* Zeroed delta. These, not the raw d_*, are what gets streamed and logged,
       * so the logged axes stay consistent with the logged magnitude. The SD CSV
       * and the USB streams carry these under the names z_x,z_y,z_z,z_mag.
       * NOTE: the BLE characteristic still carries RAW per-sensor values, and
       * friends_ble_logger.py / live_mag_ble.py derive their own d_x..d_mag from
       * those, so a BLE-side capture is NOT baseline-corrected and will not match
       * an SD capture. The names differ (d_* vs z_*) to keep that visible.
       * NOTE: the baseline is reported over CDC only, so the raw pre-zero delta
       * is not recoverable from the SD file. */
      int16_t z_x, z_y, z_z;
      MagZero_Apply(d_x, d_y, d_z, &z_x, &z_y, &z_z);
      uint32_t z_mag = LIS2MDL_MagnitudeRaw(z_x, z_y, z_z);   /* |(s1-s2) - baseline| */

      /* Hand the sample to BLE. v3 carries raw s1, raw s2 AND the zeroed delta,
       * so a BLE capture now matches an SD capture in the z_* columns. The
       * baseline-valid flag lets the host distinguish "zeroed" from "still
       * capturing", rather than guessing from the values. */
      P2PS_APP_SetMagSample((int16_t)x1, (int16_t)sensor1_y, (int16_t)z1,
                            (int16_t)x2, (int16_t)y2, (int16_t)z2,
                            z_x, z_y, z_z, MagZero_IsReady());

      // Case-switch for streaming over USB or BLE
      switch (serial_stream_mode)
      {
      case STREAM_FULL:
      {
        char usb_line[128];
        /* SerialPlot ASCII format: one line per sample, bare comma-separated
         * numbers only. Channel order:
         * s1_x,s1_y,s1_z,s1_mag,s2_x,s2_y,s2_z,s2_mag,mag12_diff,mag21_diff,z_x,z_y,z_z,z_mag */
        int usb_len = snprintf(usb_line, sizeof(usb_line),
               "%d,%d,%d,%lu,%d,%d,%d,%lu,%ld,%ld,%d,%d,%d,%lu\r\n",
               x1, sensor1_y, z1, (unsigned long)mag1,
               x2, y2, z2, (unsigned long)mag2, (long)mag12_diff, (long)mag21_diff,
               z_x, z_y, z_z, (unsigned long)z_mag);
        if ((usb_len > 0) && ((size_t)usb_len < sizeof(usb_line))) {
          (void)USB_CDC_Print(usb_line);
        }
        break;
      }

      case STREAM_MAG:
      {
        char usb_line[64];
        /* Channel order: s1_mag,s2_mag,mag12_diff,z_mag
         * mag12_diff and z_mag side by side: |s1|-|s2| versus the zeroed
         * |(s1-s2) - baseline|. Note z_mag is non-negative by construction. */
        int usb_len = snprintf(usb_line, sizeof(usb_line),
               "%lu,%lu,%ld,%lu\r\n",
               (unsigned long)mag1, (unsigned long)mag2, (long)mag12_diff, (unsigned long)z_mag);
        if ((usb_len > 0) && ((size_t)usb_len < sizeof(usb_line))) {
          (void)USB_CDC_Print(usb_line);
        }
        break;
      }

      case STREAM_VEC:
      {
        char usb_line[32];
        /* Channel order: z_x,z_y,z_z - the signed, baseline-subtracted vector
         * difference s1 - s2. These are the channels that actually cross zero:
         * z_mag is a magnitude and is non-negative by construction, so it idles
         * near +3 LSB on noise alone and cannot average to zero. These three
         * are zero-mean, so they average to zero and show direction.
         * Widest possible line is "-32768,-32768,-32768\r\n" = 22 chars + NUL. */
        int usb_len = snprintf(usb_line, sizeof(usb_line),
               "%d,%d,%d\r\n", z_x, z_y, z_z);
        if ((usb_len > 0) && ((size_t)usb_len < sizeof(usb_line))) {
          (void)USB_CDC_Print(usb_line);
        }
        break;
      }

      case STREAM_OFF:
      default:
        break;
      }

      if (sd_logging != 0U)
      {
        SDLog_WriteSample(now_ms, (int16_t)x1, (int16_t)sensor1_y, (int16_t)z1, mag1,
                          (int16_t)x2, (int16_t)y2, (int16_t)z2, mag2,
                          z_x, z_y, z_z, z_mag);
      }
    }
    else if ((HAL_GetTick() - usb_status_last_ms) >= 1000U) // if either sensor isn't reading
        {
          char usb_line[96];
          int usb_len = snprintf(usb_line, sizeof(usb_line),
                 "LIS2MDL read status: sensor1=%ld sensor2=%ld\r\n",
                 (long)s1, (long)s2);
          if ((usb_len > 0) && ((size_t)usb_len < sizeof(usb_line)))
          {
            (void)USB_CDC_Print(usb_line);
          }
          usb_status_last_ms = HAL_GetTick();
        }

    PuffLogService(); //monitor and watch for puffs
    SDLog_Service(); //log if there's anything to log
    P2PS_APP_Process(); // notify latest sample over BLE if subscribed.
    HAL_Delay(50);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI1
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLLSAI1.PLLN = 24;
  PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_USBCLK|RCC_PLLSAI1_ADCCLK;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_HSE_DIV1024;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

}

/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
static void MX_RF_Init(void)
{

  /* USER CODE BEGIN RF_Init 0 */

  /* USER CODE END RF_Init 0 */

  /* USER CODE BEGIN RF_Init 1 */

  /* USER CODE END RF_Init 1 */
  /* USER CODE BEGIN RF_Init 2 */

  /* USER CODE END RF_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
  hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  if (HAL_SPI_DeInit(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;

  /* PWM input mode: the rising edge on TI1 resets the counter, so CCR1
     captures the period and CCR2 (indirect, falling) the high time. */
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;

  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIMEx_RemapConfig(&htim2, TIM_TIM2_ITR_USB) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* Same pin (TI1) routed to channel 2, opposite edge -> high time. */
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|LED_GREEN_Pin|THERMISTOR2_Pin|THERMISTOR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MAG_CS1_Pin|MAG_CS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_GREEN_Pin THERMISTOR2_Pin THERMISTOR1_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin|THERMISTOR2_Pin|THERMISTOR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MAG_CS1_Pin MAG_CS2_Pin */
  GPIO_InitStruct.Pin = MAG_CS1_Pin|MAG_CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(
      GPIOA,
      MAG_CS1_Pin | MAG_CS2_Pin,
      GPIO_PIN_SET);

  HAL_GPIO_WritePin(
      SD_CS_GPIO_Port,
      SD_CS_Pin,
      GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Both Edge polarity timer capture for PA15, which holds the RF signal
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance != TIM2) ||
        (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1))
    {
        return;
    }

    uint32_t now_ms = HAL_GetTick();

    if ((TIM2->SR & TIM_SR_CC1OF) != 0U)
    {
    	rf_overcapture++;
    	TIM2->SR = ~TIM_SR_CC1OF; // rc_w0, writing 0 clears
    }

    /*
     * PWM input mode. The rising edge on TI1 resets the counter, so CCR1
     * holds the full period of the cycle that just ended and CCR2 holds that
     * same cycle's high time.
     */
    uint32_t period = TIM2->CCR1;
    uint32_t high = TIM2->CCR2;

    last_capture = period;

    if (rf_meas_state >= 2U)
    {
    	if (period < rf_per_min) { rf_per_min = period; }
    	if (period > rf_per_max) { rf_per_max = period; }
    	if (period > RF_GAP_TICKS) { rf_gap_count++; }

    	if ((period >= 64U) && (high <= period))
    	{
    		uint32_t duty = (high * 1000U) / period;

    		if (duty < rf_duty_min) { rf_duty_min = duty; }
    		if (duty > rf_duty_max) { rf_duty_max = duty; }

    		if (rf_meas_state == 2U)
    		{
    			rf_duty_acc = (int32_t) duty * 1024; // seed the running mean
    			rf_duty_state = 0U;
    			rf_meas_state = 3U;
    		}
    		else
    		{
    			int32_t mean = rf_duty_acc / 1024;
    			rf_duty_acc += (int32_t)duty - mean;

    			if ((rf_duty_state == 0U) && ((int32_t)duty > (mean + RF_DUTY_HYST)))
    			{
    				rf_duty_state = 1U;
    				rf_duty_cross++;
    			}
    			else if ((rf_duty_state == 1U) && ((int32_t)duty < (mean - RF_DUTY_HYST)))
				{
    				rf_duty_state = 0U;
    				rf_duty_cross++;
				}
    		}
    	}

    }

    else if (rf_meas_state == 1U)
    {
    	rf_meas_state = 2U; // discard the first cycle, it spans the idle gap
    }

    last_rf_edge_ms = now_ms;
    rf_edge_count++;
    rf_falling_count++;

    {
           /* The counter restarted at this edge, so CNT is elapsed ISR time. */
           uint32_t lat = TIM2->CNT;
           if (lat > rf_latency_max) { rf_latency_max = lat; }
     }


    /*
     * Do not perform USB or SD operations here.
     * Tell the main loop that it needs to begin a puff.
     */
    if ((puff_active == 0U) && (puff_started == 0U))
    {
        puff_start_ms = now_ms;
        puff_started = 1U;
    }
}

static void CS1_Select(void)   { HAL_GPIO_WritePin(MAG_CS2_GPIO_Port, MAG_CS2_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(MAG_CS1_GPIO_Port, MAG_CS1_Pin, GPIO_PIN_RESET); }
static void CS1_Deselect(void) { HAL_GPIO_WritePin(MAG_CS1_GPIO_Port, MAG_CS1_Pin, GPIO_PIN_SET); }
static void CS2_Select(void)   { HAL_GPIO_WritePin(MAG_CS1_GPIO_Port, MAG_CS1_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(MAG_CS2_GPIO_Port, MAG_CS2_Pin, GPIO_PIN_RESET); }
static void CS2_Deselect(void) { HAL_GPIO_WritePin(MAG_CS2_GPIO_Port, MAG_CS2_Pin, GPIO_PIN_SET); }

static uint32_t LIS2MDL_MagnitudeRaw(int16_t x, int16_t y, int16_t z)
{
    double xd = (double)x;
    double yd = (double)y;
    double zd = (double)z;

    return (uint32_t)sqrt((xd * xd) + (yd * yd) + (zd * zd));
}

static void SDLog_ProcessCommand(void)
{
    char command[16];
    char *p = command;

    if (CDC_ReadCommand(command, sizeof(command)) == 0U) {
        return;
    }

    while ((*p != '\0') && isspace((unsigned char)*p)) {
        p++;
    }

    switch ((char)tolower((unsigned char)*p)) {
    case 'd':
        SDLog_Start();
        break;
    case 's':
        SDLog_Stop("SD logging stopped\r\n");
        break;
    case 'e':
        SDLog_Eject();
        break;
    case 'b':
        Cmd_EnterBootloader();   /* does not return unless cancelled */
        break;
    case 'p':
        serial_stream_mode = STREAM_FULL;
        (void)USB_CDC_Print("Serial streaming: full (10 ch)\r\n");
        break;
    case 'm':
        serial_stream_mode = STREAM_MAG;
        (void)USB_CDC_Print("Serial streaming: magnitudes (3 ch)\r\n");
        break;
    case 'v':
        /* Signed zeroed axes. Separate from 'm' so neither existing view
         * changes channel count. */
        serial_stream_mode = STREAM_VEC;
        (void)USB_CDC_Print("Serial streaming: signed zeroed vector z_x,z_y,z_z (3 ch)\r\n");
        if (MagZero_IsReady() == 0U) {
            (void)USB_CDC_Print("NOTE: baseline not captured yet; values are raw s1-s2 until it is\r\n");
        }
        break;
    case 'x':
        serial_stream_mode = STREAM_OFF;
        (void)USB_CDC_Print("Serial streaming disabled\r\n");
        break;
    case 'z':
        /* Re-capture the mag-delta baseline. Needed when the board powered up
         * with a target already near it, since the boot capture would have
         * absorbed that target into the reference.
         * Duration is deliberately not quoted: the capture needs
         * MAG_ZERO_DISCARD + MAG_ZERO_SAMPLES samples that both parts returned
         * HAL_OK for, and this loop polls at the same 20 Hz as the ODR, so it
         * misses samples at an unpredictable rate. Wait for the confirmation
         * line instead of a stopwatch. */
        MagZero_Arm();
        mag_zero_announced = 0U;
        if (sd_logging != 0U) {
            /* The z_* columns shift discontinuously at this point and the CSV
             * carries no marker for it. Say so loudly. */
            (void)USB_CDC_Print("WARNING: re-zeroing while logging; z_* columns "
                                "shift mid-file with no marker in the CSV\r\n");
        }
        (void)USB_CDC_Print("Mag delta re-zeroing; hold still until confirmed\r\n");
        break;
    default:
        (void)USB_CDC_Print("Unknown command. Use d=start, s=stop, e=eject, "
                            "p=print full, m=print magnitudes, v=print signed "
                            "vector, x=stop print, z=zero mag delta, "
                            "b=USB DFU bootloader\r\n");
        break;

    }
}

static FRESULT SDLog_Mount(void)
{
    FRESULT result;
    DSTATUS status;

    /*
     * Detach any stale FatFs registration before rebuilding
     * the disk and filesystem state.
     */
    (void)f_mount(NULL, "", 0U);
    sd_mounted = 0U;

    /*
     * disk_initialize() performs the driver-specific reset and
     * card initialization inside diskio.c.
     *
     * Drive 0 is the SD card in this project.
     */
    status = disk_initialize(0U);

    if ((status & STA_NOINIT) != 0U)
    {
        return FR_NOT_READY;
    }

    result = f_mount(&sd_fs, "", 1U);

    if (result == FR_OK)
    {
        sd_mounted = 1U;
    }

    return result;
}

static FRESULT SDLog_NextFileName(char *name, size_t name_size)
{
    FILINFO info;
    FRESULT result;

    for (uint32_t index = 1U; index <= 9999U; index++) {
        int len = snprintf(name, name_size, "test_%lu.txt", (unsigned long)index);
        if ((len <= 0) || ((size_t)len >= name_size)) {
            return FR_INVALID_NAME;
        }

        result = f_stat(name, &info);
        if (result == FR_NO_FILE) {
            return FR_OK;
        }
        if (result != FR_OK) {
            return result;
        }
    }

    return FR_DENIED;
}

static FRESULT SDLog_FindNextRun(char *puff_name, size_t puff_size, char *mag_name, size_t mag_size)
{
	// This function will return one five things:
	// An invalid name error. a puff f_stat, a  mag_fstat, or an
	FILINFO info;

	for (uint32_t i = 1U; i<=999U; i++)
	{
		int puff_len = snprintf(puff_name, puff_size, "RUN%04lu_PUFF.csv", (unsigned long)i);
		int mag_len = snprintf(mag_name, mag_size, "RUN%04lu_MAG.csv", (unsigned long)i);

		// Return Invalid error code if f
		if((puff_len <= 0) || ((size_t)puff_len >= puff_size) || (mag_len <= 0) || ((size_t)mag_len >= mag_size))
		{
			return FR_INVALID_NAME;
		}
		FRESULT puff_result = f_stat(puff_name, &info);
		FRESULT mag_result = f_stat(mag_name, &info);
		if ((puff_result != FR_OK) && (puff_result != FR_NO_FILE))
		{
			return puff_result;
		}
		if ((mag_result != FR_OK) && (mag_result != FR_NO_FILE))
		{
			return mag_result;
		}
		if ((puff_result == FR_NO_FILE) && (mag_result == FR_NO_FILE))
		{
			return FR_OK;
		}
	}
	return FR_DENIED;
}

static void SDLog_Start(void)
{
	// Updated version of previous which only logged testX mag data
    FRESULT result;
    UINT written;
    char puff_filename[32];
    char mag_filename[32];

    char message[128];

    // Declare headers for puff events and magnetometer readings
    static const char puff_header[] = "date,time,puff_id,event,start_ms,end_ms,duration_ms,rf_edges,rising_edges,falling_edges,thermistor1,thermistor2\r\n";
    static const char mag_header[] = "date,time,ms,puff_id,puff_active,s1_x,s1_y,s1_z,s1_mag,s2_x,s2_y,s2_z,s2_mag,z_x,z_y,z_z,z_mag\r\n";

    if (sd_logging != 0U) {
        (void)USB_CDC_Print("SD logging already active\r\n");
        return;
    }

    // Check mount
    result = SDLog_Mount();
    if (result != FR_OK) {
        SDLog_PrintResult("SD mount failed", result);
        return;
    }
    // find conjunct puff and mag filenames
    result = SDLog_FindNextRun(puff_filename, sizeof(puff_filename), mag_filename, sizeof(mag_filename));
    if (result != FR_OK) {
        SDLog_PrintResult("Finding next SD run failed", result);
        return;
    }

    // WRITE TO OR CREATE puff file
    result = f_open(&sd_puff_file, puff_filename, FA_WRITE | FA_CREATE_NEW);
    if (result != FR_OK) {
        	SDLog_PrintResult("SD puff file open failed", result);
        	return;
        }

    // WRITE TO OR CREATE mag file, closing puff file if it fails
    result = f_open(&sd_mag_file, mag_filename, FA_WRITE | FA_CREATE_NEW);
    if (result != FR_OK) {
    		(void)f_close(&sd_puff_file); // CLOSE the already created puff file
            SDLog_PrintResult("SD mag filename failed", result);
            return;
        }

    // write header to puff file
    result = f_write(&sd_puff_file, puff_header, (UINT)(sizeof(puff_header) - 1U), &written);
    if ((result != FR_OK) || (written != (UINT)(sizeof(puff_header) - 1U)))
    {
    	// Close both files
        (void)f_close(&sd_puff_file);
        (void)f_close(&sd_mag_file);
        // error out
        SDLog_PrintResult("Puff header write failed", (result == FR_OK) ? FR_DISK_ERR : result);
        return;
    }
    written = 0U;
    // write mag header to magnetometer file
    result = f_write(&sd_mag_file, mag_header, (UINT)(sizeof(mag_header) - 1U), &written);
    if ((result != FR_OK) || (written != (UINT)(sizeof(mag_header) - 1U)))
    {
            (void)f_close(&sd_puff_file);
            (void)f_close(&sd_mag_file);
            SDLog_PrintResult("Mag header write failed", (result == FR_OK) ? FR_DISK_ERR : result);
            return;
    }

    // Sync files
    result = f_sync(&sd_puff_file);
    if(result == FR_OK)
    {
    	result = f_sync(&sd_mag_file); // try mag file
    }
    if (result != FR_OK)
    {
    	(void)f_close(&sd_puff_file);
    	(void)f_close(&sd_mag_file);
    	SDLog_PrintResult("Initial SD sync failed", result);
    	return;
    }
    // set all related flags to false and ready double buffer
    current_puff_id = 0U;
    mag_fill_buf = mag_buffer1;
    mag_write_buf = mag_buffer2;
    mag_fill_count = 0U;
    mag_write_count = 0U;
    mag_buffer_ready = 0U;
    mag_overrun = 0U;
    sd_faulted = 0U;
    sd_logging = 1U;
    sd_last_sync_ms = HAL_GetTick();

    int len = snprintf(message, sizeof(message), "SD logging started: %s and %s\r\n", puff_filename, mag_filename);
    if ((len > 0) && ((size_t)len < sizeof(message))) {
        (void)USB_CDC_Print(message);
    }
}

static void SDLog_Abort(const char *operation, FRESULT result)
{
	sd_faulted = 1U;
	sd_logging = 0U;
	(void)f_close(&sd_puff_file);
	(void)f_close(&sd_mag_file);
	(void)f_mount(NULL, "", 0U); // Ater a disk error, don't leave sd_mounted = 1U, require fresh mount, otherwise this function assums that the card is still mounted
	sd_mounted = 0U;
	SDLog_PrintResult(operation, result);

}

static void SDLog_Stop(const char *reason)
{

    if (sd_logging == 0U) {
        if (reason != NULL) {
            (void)USB_CDC_Print("SD logging is not active\r\n");
        }
        return;
    }

    SDLog_FlushMagBuffers();
    if (sd_logging == 0U)
    {
    	return;
    }

    FRESULT puff_sync = f_sync(&sd_puff_file);
    FRESULT mag_sync = f_sync(&sd_mag_file);
    FRESULT puff_close = f_close(&sd_puff_file);
    FRESULT mag_close = f_close(&sd_mag_file);
    sd_logging = 0U;

    FRESULT final_result = FR_OK;
    if(puff_sync != FR_OK) final_result = puff_sync;
    else if(mag_sync != FR_OK) final_result = mag_sync;
    else if(puff_close != FR_OK) final_result = puff_close;
    else if(mag_close !=  FR_OK) final_result = mag_close;

    sd_logging = 0U;

    if (final_result != FR_OK)
    {
    	sd_faulted = 1U;
    	SDLog_PrintResult("SD close/sync failed", final_result);
    	return;
    }
    if (reason != NULL)
    {
    	(void)USB_CDC_Print(reason);
    }
}

static void SDLog_Eject(void)
{
	sd_faulted = 0U;

    if (sd_logging != 0U) {
        SDLog_Stop(NULL);

        if (sd_faulted != 0U)
        {
        	(void)USB_CDC_Print("Eject canceled, SD files did not close safely\r\n");
        	return;
        }
    }

    // Report unmount error, CANCEL EJECT if unmount fails, do not allow Windows and FatFs access SD card after unsuccessful unmount
    if (sd_mounted != 0U)
    {
        FRESULT result = f_mount(NULL, "", 0U);

        if ((result != FR_OK) && (result !=FR_NOT_ENABLED))
        {
            SDLog_PrintResult("SD unmount failed", result);
            (void)USB_CDC_Print(
                "Eject cancelled; SD remains owned by STM32\r\n");
            return;
        }

        sd_mounted = 0U;
    }

    (void)USB_CDC_Print("SD closed and unmounted; switching to USB mass storage. Reset the board to return to CDC mode.");
    HAL_Delay(300U);
    if (USB_Device_SwitchToMSC() == 0U) {(void)USB_CDC_Print("USB mass storage switch failed; CDC restored");}
}

/* 'b' command: close SD files, detach USB cleanly, then reset into the ST
 * system-memory bootloader (USB DFU). The actual flag + reset + jump live in
 * boot_dfu.c (user-owned). Mirrors SDLog_Eject's close-then-print-then-wait
 * pattern and USB_Device_SwitchToMSC's Stop/DeInit/delay detach timing, both
 * of which are already proven on this board. */
static void Cmd_EnterBootloader(void)
{
    sd_faulted = 0U;

    if (sd_logging != 0U) {
        SDLog_Stop(NULL);

        if (sd_faulted != 0U) {
            (void)USB_CDC_Print("DFU cancelled, SD files did not close safely\r\n");
            return;
        }
    }

    /* Silence streaming so the notice is not stuck behind a pending sample. */
    serial_stream_mode = STREAM_OFF;

    for (uint32_t tries = 0U; tries < 50U; tries++) {
        if (USB_CDC_Print("Entering USB DFU bootloader; COM port will disconnect. "
                          "Power-cycle the board after flashing.\r\n") == HAL_OK) {
            break;
        }
        HAL_Delay(2U);
    }
    HAL_Delay(300U);                 /* let the CDC IN transfer reach the host */

    (void)USBD_Stop(&hUsbDeviceFS);  /* drop the D+ pull-up: host sees detach */
    (void)USBD_DeInit(&hUsbDeviceFS);
    HAL_Delay(250U);

    BootDFU_RequestAndReset();       /* no return */
}



static void SDLog_WriteSample(uint32_t now_ms,
    int16_t s1_x, int16_t s1_y, int16_t s1_z, uint32_t s1_mag,
    int16_t s2_x, int16_t s2_y, int16_t s2_z, uint32_t s2_mag,
    int16_t z_x, int16_t z_y, int16_t z_z, uint32_t z_mag)
{

	if (sd_logging == 0U)
	{
		return;
	}
	if (mag_fill_count >= MAG_BUF_LEN)
	{
		if (mag_buffer_ready != 0U)
		{
			mag_overrun = 1U;
			return;
		}
		MagLogSample *tmp = mag_write_buf;
		mag_write_buf = mag_fill_buf;
		mag_write_count = mag_fill_count;
		mag_fill_buf = tmp;
		mag_fill_count = 0U;
		mag_buffer_ready = 1U;
	}

	MagLogSample *sample = &mag_fill_buf[mag_fill_count++];
	getTimestamp(sample->timestamp, sizeof(sample->timestamp));
	sample->ms = now_ms;
	sample->puff_id = current_puff_id;
	sample->puff_active = puff_active;
	sample->s1_x = s1_x;
	sample->s1_y = s1_y;
	sample->s1_z = s1_z;
	sample->s1_mag = s1_mag;
	sample->s2_x = s2_x;
	sample->s2_y = s2_y;
	sample->s2_z = s2_z;
	sample->s2_mag = s2_mag;
	sample->z_x = z_x;
	sample->z_y = z_y;
	sample->z_z = z_z;
	sample->z_mag = z_mag;

}

static void SDLog_Service(void)
{
	if((sd_logging == 0U) || (mag_buffer_ready == 0U))
	{
		return;
	}
	char line[192];
	for (uint32_t i =0U; i < mag_write_count; i++)
	{
		MagLogSample *sample = &mag_write_buf[i];
		int len = snprintf(line, sizeof(line), "%s,%lu,%lu,%u,%d,%d,%d,%lu,%d,%d,%d,%lu,%d,%d,%d,%lu\r\n",
		            							sample->timestamp,
												(unsigned long)sample->ms,
												(unsigned long)sample->puff_id,
												(unsigned int)sample->puff_active,
												sample->s1_x, sample->s1_y, sample->s1_z,
												(unsigned long)sample->s1_mag,
												sample->s2_x, sample->s2_y, sample->s2_z,
												(unsigned long)sample->s2_mag,
												sample->z_x, sample->z_y, sample->z_z,
												(unsigned long)sample->z_mag);
		if((len <= 0) || ((size_t)len >= sizeof(line)))
		{
			SDLog_Abort("MAG line format failed", FR_INT_ERR);
			return;
		}
		UINT written = 0U;
		FRESULT result = f_write(&sd_mag_file,line,(UINT)len,&written);
		if ((result != FR_OK) || (written != (UINT)len))
		{
			SDLog_Abort("MAG write failed", (result == FR_OK) ? FR_DISK_ERR : result);
			return;
		}
	}

	mag_write_count = 0U;
	mag_buffer_ready = 0U;

	if ((HAL_GetTick() - sd_last_sync_ms) >= 1000U)
	{
		FRESULT result = f_sync(&sd_mag_file);
		if (result == FR_OK)
		{
			result = f_sync(&sd_puff_file);
		}
		if (result != FR_OK)
		{
			SDLog_Abort("Periodic SD sync failed", result);
			return;
		}
		sd_last_sync_ms = HAL_GetTick(); // UPdate last sync time after successful syncs
	}

	if (mag_overrun != 0U)
	{
		mag_overrun = 0U;
		(void)USB_CDC_Print("Warning: MAG buffer overrun, samples dropped");
	}
}

static void SDLog_FlushMagBuffers(void)
{
	if(sd_logging == 0U)
	{
		return;
	}

	if (mag_buffer_ready != 0U)
	{
		SDLog_Service();
		if(sd_logging == 0U)
		{
			return;
		}
	}

	if (mag_fill_count > 0U)
	{
		MagLogSample *tmp = mag_write_buf;
		mag_write_buf = mag_fill_buf;
		mag_write_count = mag_fill_count;
		mag_fill_buf = tmp;
		mag_fill_count = 0U;
		mag_buffer_ready = 1U;
		SDLog_Service();
	}
}

static void SDLog_WritePuffEvent(const char *event, uint32_t start_ms, uint32_t end_ms, uint32_t duration_ms, uint32_t edges, uint32_t rising, uint32_t falling, uint32_t t1_raw, uint32_t t2_raw)
{
	if (sd_logging == 0U)
	{
		return;
	}

	char timestamp[32];
	char line[192];
	getTimestamp(timestamp, sizeof(timestamp));

	int len = snprintf(line, sizeof(line), "%s,%lu,%s,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n", timestamp,
																				(unsigned long)current_puff_id,
																				event,
																				(unsigned long)start_ms,
																				(unsigned long)end_ms,
																				(unsigned long)duration_ms,
																				(unsigned long)edges,
																				(unsigned long)rising,
																				(unsigned long)falling,
																				(unsigned long)t1_raw,
																				(unsigned long)t2_raw);
	if ((len <= 0) || ((size_t)len >= sizeof(line)))
	{
		SDLog_Abort("PUFF line format failed", FR_INT_ERR);
		return;
	}

	UINT written = 0U;
	FRESULT result = f_write(&sd_puff_file, line, (UINT)len, &written);
	if ((result != FR_OK) || (written != (UINT)len))
	{
		SDLog_Abort("PUFF write failed", (result == FR_OK) ? FR_DISK_ERR : result);
	}

}

// PUFF DETECTION

static void PuffLogService(void)
{

    /*
     * The ISR sets puff_started.
     * The main loop performs printing and SD writes.
     */
    if ((puff_started != 0U) && (puff_active == 0U))
    {

        start_puff();
        puff_started = 0U; // IMPORTANT: Clear this flag AFTER start_puff returns in order to avoid re-arming the flag and leave stale puff_start_ms behind
    }

    // Read ticks HERE, not at the top of the function.
    // Doing this
    uint32_t last_ms = last_rf_edge_ms;
    uint32_t now_ms = HAL_GetTick();
    uint32_t idle_ms = now_ms - last_ms;

    if (idle_ms > 0x7FFFFFFFU)
    {
    	idle_ms = 0U; // edge landed after the tick was sampled
    }

    /* if puff active, GPIO15 high and time since last edge is greater than the timeout period, end the puff*/
    if ((puff_active != 0U) &&
        (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET) &&
        ((idle_ms >= RF_INACTIVITY_TIMEOUT_MS)))
    {
        end_puff();
    }
}

static void start_puff(void)
{
    current_puff_id++;

    puff_active = 1U;
    puff_ended = 0U;

    rf_per_min = 0xFFFFFFFFU;
    rf_per_max = 0U;
    rf_gap_count = 0U;
    rf_duty_min = 0xFFFFFFFFU;
    rf_duty_max = 0U;
    rf_duty_cross = 0U;
    rf_duty_state = 0U;
    rf_overcapture = 0U;
    rf_latency_max = 0U;
    rf_meas_state = 1U;


    /*
     * puff_start_ms was recorded by the ISR at the first edge.
     */
    puff_start_edge_count = rf_edge_count;
    puff_start_rising_count = rf_rising_count;
    puff_start_falling_count = rf_falling_count;

    // Unset purple and turn green
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    char message[96];
    int len = snprintf(
        message,
        sizeof(message),
        "PUFF %lu START at %lu ms\r\n",
        (unsigned long)current_puff_id,
        (unsigned long)puff_start_ms);

    if ((len > 0) && ((size_t)len < sizeof(message)))
    {
        (void)USB_CDC_Print(message);
    }

    Thermistor_ReadBoth(&therm1_start, &therm2_start);

    SDLog_WritePuffEvent(
        "START",
        puff_start_ms,
        0U,
        0U,
        0U,
        0U,
        0U,
		therm1_start,
		therm2_start);
}

static void end_puff(void)
{
    /*
     * Use the last actual RF transition as the signal endpoint.
     */
    puff_end_ms = last_rf_edge_ms;
    puff_duration_ms = puff_end_ms - puff_start_ms;

    uint32_t edges = rf_edge_count - puff_start_edge_count;
    uint32_t rising = rf_rising_count - puff_start_rising_count;
    uint32_t falling = rf_falling_count - puff_start_falling_count;

    puff_ended = 1U;
    // unset green and return to purple
    if (puff_ended == 1U) { LED_IdlePurple(); }

    /* Writing for both CDC and SD Logging */
    char message[128];
    int len = snprintf(
        message,
        sizeof(message),
        "PUFF %lu END duration=%lu ms edges=%lu rise=%lu fall=%lu\r\n",
        (unsigned long)current_puff_id,
        (unsigned long)puff_duration_ms,
        (unsigned long)edges,
        (unsigned long)rising,
        (unsigned long)falling);

    if ((len > 0) && ((size_t)len < sizeof(message)))
    {
        (void)USB_CDC_Print(message);
    }

    rf_meas_state = 0U;

    if (rf_per_max != 0U)
    {
        char line[256];
        uint32_t spread = (rf_duty_max >= rf_duty_min)
                          ? (rf_duty_max - rf_duty_min) : 0U;
        /* two crossings per modulation cycle */
        uint32_t mod_hz = (puff_duration_ms > 0U)
                          ? ((rf_duty_cross * 500UL) / puff_duration_ms) : 0UL;

        int n = snprintf(line, sizeof(line),
            "PUFF %lu PWM per=%lu..%lu (%lu.%lu..%lu.%lu us) duty=%lu..%lu pm spread=%lu long=%lu cross=%lu (%lu Hz) ovf=%lu lat=%lu\r\n",
            (unsigned long)current_puff_id,
            (unsigned long)rf_per_min,
            (unsigned long)rf_per_max,
            (unsigned long)(rf_per_min / 64U), (unsigned long)((rf_per_min * 10U / 64U) % 10U),
            (unsigned long)(rf_per_max / 64U), (unsigned long)((rf_per_max * 10U / 64U) % 10U),
            (unsigned long)rf_duty_min,
            (unsigned long)rf_duty_max,
            (unsigned long)spread,
            (unsigned long)rf_gap_count,
            (unsigned long)rf_duty_cross,
            (unsigned long)mod_hz,
            (unsigned long)rf_overcapture,
            (unsigned long)rf_latency_max);
        if ((n > 0) && ((size_t)n < sizeof(line))) { (void)USB_CDC_Print(line); }
    }


    Thermistor_ReadBoth(&therm1_end, &therm2_end); // read both end values

    SDLog_WritePuffEvent(
        "END",
        puff_start_ms,
        puff_end_ms,
        puff_duration_ms,
        edges,
        rising,
        falling,
		therm1_end,
		therm2_end);

    // puff active cleared at END after all I/O so that edges arriving during SD write cannot arm the next puff
    puff_active = 0U;
}

// Thermistor Functions
static uint16_t Thermistor_ReadChannel(uint32_t channel, GPIO_TypeDef *en_port, uint16_t en_pin)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	uint16_t value = 0U;

	// Power the divider (active-high) and let the RC node settle
	HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_SET);
	HAL_Delay(THERM_SETTLE_MS);

	/* sConfig block: runtime channel selections */
	sConfig.Channel = channel; // picks mux input
	sConfig.Rank = ADC_REGULAR_RANK_1; // puts it first in the sequence
	sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5; // specified long window for the 100k resistor
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;

	// applies sConfig selection to the ADC
	if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) == HAL_OK)
	{
		if (HAL_ADC_Start(&hadc1) == HAL_OK)
		{
			if (HAL_ADC_PollForConversion(&hadc1, THERM_ADC_TIMEOUT_MS) == HAL_OK)
			{
				value = (uint16_t)HAL_ADC_GetValue(&hadc1); //grabs 12-bit result from polled read
			}

		}
		(void)HAL_ADC_Stop(&hadc1); // Return ADC to idle
		__HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_EOS); // clear adc flags, not mandatory, but keeps peripheral clean
	}

	HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_RESET); // Power divider bck off to save energy
	return value;
}

static void Thermistor_ReadBoth(uint16_t *t1_raw, uint16_t *t2_raw)
{
	/* Reads t1 and t2 and hand both back through pointers*/
	uint16_t v1 = Thermistor_ReadChannel(THERM1_ADC_CHANNEL, THERMISTOR1_GPIO_Port, THERMISTOR1_Pin);
	uint16_t v2 = Thermistor_ReadChannel(THERM2_ADC_CHANNEL, THERMISTOR2_GPIO_Port, THERMISTOR2_Pin);

	if (t1_raw != NULL) { *t1_raw = v1; }
	if (t2_raw != NULL) { *t2_raw = v2; }
}

static void LED_IdlePurple(void)
{
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
}

static void getTimestamp(char *buf, size_t len)
{
    RTC_TimeTypeDef time = {0};
    RTC_DateTypeDef date = {0};

    if ((buf == NULL) || (len == 0U))
    {
        return;
    }

    /*
     * Read time first, then date.
     * Reading the date unlocks the RTC shadow registers.
     */
    if ((HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK) ||
        (HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK))
    {
        (void)snprintf(buf, len, "RTC_ERROR");
        return;
    }

    (void)snprintf(
        buf,
        len,
        "20%02u-%02u-%02u,%02u:%02u:%02u",
        (unsigned int)date.Year,
        (unsigned int)date.Month,
        (unsigned int)date.Date,
        (unsigned int)time.Hours,
        (unsigned int)time.Minutes,
        (unsigned int)time.Seconds);
}

static void SDLog_PrintResult(const char *operation, FRESULT result)
{
    char message[64];
    const char *detail = "";

    if (result == FR_NO_FILESYSTEM) detail = "(no FAT/exFAT filesystem)";
    else if (result == FR_NOT_READY) detail = "(card/drive not ready)";
    else if (result == FR_WRITE_PROTECTED) detail = " (write protected/read-only)";
    else if (result == FR_DISK_ERR) detail = " (physical disk I/O error)";

    int len = snprintf(message, sizeof(message), "%s: FR=%u%s\r\n", operation, (unsigned int)result, detail);

    if ((len > 0) && ((size_t)len < sizeof(message))) {
        (void)USB_CDC_Print(message);
    }
}

static HAL_StatusTypeDef USB_CDC_Print(const char *text)
{
    static uint8_t tx_buf[256];
    size_t len;
    USBD_CDC_HandleTypeDef *hcdc;

    if (text == NULL) {
        return HAL_ERROR;
    }
    if (USB_Device_GetMode() != USB_DEVICE_MODE_CDC) {
        return HAL_BUSY;
    }

    len = strlen(text);
    if (len > UINT16_MAX) {
        return HAL_ERROR;
    }
    if ((hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) || (hUsbDeviceFS.pClassData == NULL)) {
        return HAL_BUSY;
    }
    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc->TxState != 0U) {
        return HAL_BUSY;
    }
    memcpy(tx_buf, text, len);

    return (CDC_Transmit_FS(tx_buf, (uint16_t)len) == 0U) ? HAL_OK : HAL_BUSY;
}

static HAL_StatusTypeDef LIS2MDL_DebugWhoAmI(const char *name,
    GPIO_TypeDef *cs_port, uint16_t cs_pin, volatile uint8_t *who_am_i,
    void (*cs_select)(void), void (*cs_deselect)(void))
{
    HAL_StatusTypeDef last = HAL_ERROR;
    uint8_t value = 0U;

    (void)name;
    (void)cs_port;
    (void)cs_pin;

    for (uint32_t attempt = 1U; attempt <= 5U; attempt++) {
        CS1_Deselect();
        CS2_Deselect();
        value = 0U;
        last = LIS2MDL_CheckWhoAmI(&value, cs_select, cs_deselect);
        *who_am_i = value;

        HAL_Delay(2);
    }

    CS1_Deselect();
    CS2_Deselect();
    return last;
}

HAL_StatusTypeDef LIS2MDL_SPI_WriteReg(uint8_t reg, uint8_t data,
    void (*cs_sel)(void), void (*cs_desel)(void))
{
    uint8_t tx[2] = { reg & 0x7F, data };
    cs_sel();
    HAL_StatusTypeDef r = HAL_SPI_Transmit(&hspi1, tx, 2, 10);
    cs_desel();
    return r;
}

HAL_StatusTypeDef LIS2MDL_SPI_ReadReg(uint8_t reg, uint8_t *data,
    void (*cs_sel)(void), void (*cs_desel)(void))
{
    uint8_t cmd = reg | 0x80;
    cs_sel();
    HAL_StatusTypeDef r = HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    if (r == HAL_OK) {
        r = HAL_SPI_Receive(&hspi1, data, 1, 10);
    }
    cs_desel();
    return r;
}

HAL_StatusTypeDef LIS2MDL_SPI_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len,
    void (*cs_sel)(void), void (*cs_desel)(void))
{
    uint8_t cmd = reg | 0x80;
    cs_sel();
    HAL_StatusTypeDef r = HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    if (r == HAL_OK) {
        r = HAL_SPI_Receive(&hspi1, buf, len, 10);
    }
    cs_desel();
    return r;
}

HAL_StatusTypeDef LIS2MDL_CheckWhoAmI(volatile uint8_t *who_am_i,
    void (*cs_sel)(void), void (*cs_desel)(void))
{
    return LIS2MDL_SPI_ReadReg(LIS2MDL_WHO_AM_I_REG, (uint8_t *)who_am_i, cs_sel, cs_desel);
}

HAL_StatusTypeDef LIS2MDL_Init(void (*cs_sel)(void), void (*cs_desel)(void))
{
    HAL_StatusTypeDef r;
    r = LIS2MDL_SPI_WriteReg(LIS2MDL_CFG_REG_C, LIS2MDL_CFG_REG_C_VAL, cs_sel, cs_desel);
    if (r != HAL_OK) return r;
    /* CFG_REG_B before CFG_REG_A. Writing it while the part is still in idle
     * means the first continuous-mode sample already has offset cancellation
     * and the LPF active, rather than the first 1/ODR arriving uncancelled.
     * LIS2MDL DS12144 Rev 6 section 8.6, Tables 26/27. */
    r = LIS2MDL_SPI_WriteReg(LIS2MDL_CFG_REG_B, LIS2MDL_CFG_REG_B_VAL, cs_sel, cs_desel);
    if (r != HAL_OK) return r;
    r = LIS2MDL_SPI_WriteReg(LIS2MDL_CFG_REG_A, LIS2MDL_CFG_REG_A_VAL, cs_sel, cs_desel);
    if (r != HAL_OK) return r;
    return LIS2MDL_SPI_WriteReg(LIS2MDL_CFG_REG_C, LIS2MDL_CFG_REG_C_VAL, cs_sel, cs_desel);
}

HAL_StatusTypeDef LIS2MDL_ReadXYZ(int16_t *x, int16_t *y, int16_t *z,
    void (*cs_sel)(void), void (*cs_desel)(void))
{
    uint8_t status;
    uint8_t raw[6];
    HAL_StatusTypeDef r = LIS2MDL_SPI_ReadReg(LIS2MDL_STATUS_REG, &status, cs_sel, cs_desel);
    if (r != HAL_OK) return r;
    if ((status & LIS2MDL_STATUS_ZYXDA) == 0U) return HAL_BUSY;

    r = LIS2MDL_SPI_ReadRegs(LIS2MDL_OUTX_L_REG, raw, 6, cs_sel, cs_desel);
    if (r != HAL_OK) return r;
    *x = (int16_t)((raw[1] << 8) | raw[0]);
    *y = (int16_t)((raw[3] << 8) | raw[2]);
    *z = (int16_t)((raw[5] << 8) | raw[4]);
    return HAL_OK;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
