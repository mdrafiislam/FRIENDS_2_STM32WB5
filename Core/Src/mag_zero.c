/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mag_zero.c
  * @brief   Boot-time zeroing of the two-magnetometer vector difference.
  *
  * USER-OWNED FILE. CubeMX has never generated this and must never own it.
  *
  * Deliberately free of any HAL or peripheral dependency: main.c owns the SPI
  * transactions and the static CS1_Select/CS2_Select helpers, and feeds samples
  * in here. That keeps the chip-select statics private to main.c and makes this
  * unit pure arithmetic.
  *
  * See mag_zero.h for the derivation and for the documented ASSUMPTION about
  * the mechanical alignment of the two parts.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "mag_zero.h"
#include <stddef.h>     /* NULL */

/* Accumulator. int32 is sufficient: each term is a difference of two int16
 * readings, so |term| <= 65534, and MAG_ZERO_SAMPLES * 65534 = 4194176 for the
 * default 64 samples - three orders of magnitude inside INT32_MAX. */
static int32_t  mz_acc_x;
static int32_t  mz_acc_y;
static int32_t  mz_acc_z;
static uint16_t mz_count;
static uint16_t mz_discard;
static int16_t  mz_base_x;
static int16_t  mz_base_y;
static int16_t  mz_base_z;
static uint8_t  mz_ready;
/* Guards the discard window on the very first capture. Statics zero-initialise,
 * which would leave mz_discard at 0 and skip the discard on the boot capture if
 * the caller forgot MagZero_Arm(). Self-arming on the first sample removes that
 * footgun without depending on call order in main(). */
static uint8_t  mz_init;

int16_t MagZero_Sat16(int32_t v)
{
    if (v >  32767) { return  (int16_t)32767; }
    if (v < -32768) { return  (int16_t)(-32768); }
    return (int16_t)v;
}

/* Rounded division. Truncation would leave up to 1 LSB of residual on every
 * axis, which shows up directly as a non-zero idle magnitude. */
static int16_t MagZero_RoundDiv(int32_t sum, uint16_t n)
{
    int32_t half;
    int32_t q;

    if (n == 0U) { return 0; }

    half = (int32_t)(n / 2U);
    q = (sum >= 0) ? ((sum + half) / (int32_t)n)
                   : ((sum - half) / (int32_t)n);
    return MagZero_Sat16(q);
}

void MagZero_Arm(void)
{
    mz_acc_x   = 0;
    mz_acc_y   = 0;
    mz_acc_z   = 0;
    mz_count   = 0U;
    mz_discard = MAG_ZERO_DISCARD;
    mz_ready   = 0U;
    mz_init    = 1U;
    /* Baseline intentionally left at its previous value. MagZero_Apply() keys
     * off mz_ready and passes through while re-capturing, so a stale baseline
     * is never applied. */
}

uint8_t MagZero_IsReady(void)
{
    return mz_ready;
}

uint8_t MagZero_Feed(int16_t d_x, int16_t d_y, int16_t d_z)
{
    if (mz_init == 0U) { MagZero_Arm(); }

    if (mz_ready != 0U) { return 1U; }

    if (mz_discard != 0U) {
        mz_discard--;
        return 0U;
    }

    mz_acc_x += (int32_t)d_x;
    mz_acc_y += (int32_t)d_y;
    mz_acc_z += (int32_t)d_z;
    mz_count++;

    if (mz_count < MAG_ZERO_SAMPLES) { return 0U; }

    mz_base_x = MagZero_RoundDiv(mz_acc_x, mz_count);
    mz_base_y = MagZero_RoundDiv(mz_acc_y, mz_count);
    mz_base_z = MagZero_RoundDiv(mz_acc_z, mz_count);
    mz_ready  = 1U;
    return 1U;
}

void MagZero_Apply(int16_t d_x, int16_t d_y, int16_t d_z,
                   int16_t *z_x, int16_t *z_y, int16_t *z_z)
{
    if ((z_x == NULL) || (z_y == NULL) || (z_z == NULL)) { return; }

    if (mz_ready == 0U) {
        /* Pass through until the baseline is valid, so the stream is never
         * corrected by a half-captured reference. */
        *z_x = d_x;
        *z_y = d_y;
        *z_z = d_z;
        return;
    }

    *z_x = MagZero_Sat16((int32_t)d_x - (int32_t)mz_base_x);
    *z_y = MagZero_Sat16((int32_t)d_y - (int32_t)mz_base_y);
    *z_z = MagZero_Sat16((int32_t)d_z - (int32_t)mz_base_z);
}

void MagZero_GetBaseline(int16_t *b_x, int16_t *b_y, int16_t *b_z)
{
    if ((b_x == NULL) || (b_y == NULL) || (b_z == NULL)) { return; }

    if (mz_ready == 0U) {
        *b_x = 0;
        *b_y = 0;
        *b_z = 0;
        return;
    }

    *b_x = mz_base_x;
    *b_y = mz_base_y;
    *b_z = mz_base_z;
}
