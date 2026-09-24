/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mag_zero.h
  * @brief   Boot-time zeroing of the two-magnetometer vector difference.
  *
  * USER-OWNED FILE. CubeMX has never generated this and must never own it.
  * Placement rule 1: new functions and data belong in a file CubeMX cannot
  * overwrite, so this survives regeneration without USER CODE fences.
  *
  * Rationale. Model each part as s_i = k_i * B + b_i, where B is the ambient
  * field common to both, k_i the per-part scale and b_i the per-part offset.
  * Then
  *     s1 - s2 = (k1 - k2) * B + (b1 - b2)
  * The (b1 - b2) term is a constant vector independent of board orientation,
  * so averaging s1 - s2 once at power-up and subtracting it thereafter keeps
  * the difference near zero as the board is moved.
  *
  * The (k1 - k2) * B term does NOT cancel and rotates with the board. The
  * LIS2MDL sensitivity tolerance is -7% / +7% (LIS2MDL DS12144 Rev 6 Table 2,
  * So = 1.5 mgauss/LSB), so two parts may differ by up to ~14%. Against
  * Earth's ~500 mG (~333 LSB) that leaves up to ~45 LSB of orientation-
  * dependent residual that no additive baseline can remove. A gain match is
  * required if the target signal is not comfortably above that.
  *
  * ASSUMPTION: both LIS2MDL parts are mounted in the same orientation so their
  * X/Y/Z axes are parallel. If one is rotated relative to the other, B does
  * not cancel in s1 - s2 at all and a rotation matrix is needed instead of a
  * vector subtract.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef MAG_ZERO_H
#define MAG_ZERO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Number of samples averaged into the baseline. 64 at ODR = 20 Hz is 3.2 s,
 * which is exactly one set-pulse period with CFG_REG_B[Set_FREQ] = 0 ("set
 * pulse is released every 64 ODR", LIS2MDL DS12144 Rev 6 Table 27), so the
 * average spans a whole offset-cancellation cycle rather than part of one. */
#define MAG_ZERO_SAMPLES        64U

/* Discard this many samples after arming before accumulating. Covers the
 * offset-cancellation turn-on time of 9.4 ms + 1/ODR in high-resolution mode
 * (LIS2MDL DS12144 Rev 6 Table 11) and, when re-zeroing by hand, gives the
 * operator's hand time to leave the board alone. */
#define MAG_ZERO_DISCARD        8U

/* Arm (or re-arm) a capture. Discards MAG_ZERO_DISCARD samples, then averages
 * MAG_ZERO_SAMPLES of them into the baseline. */
void    MagZero_Arm(void);

/* 1 once the baseline is valid, 0 while still capturing. */
uint8_t MagZero_IsReady(void);

/* Feed one raw delta sample (s1 - s2, per axis). Returns 1 if the baseline is
 * ready after this call, 0 if still capturing. Cheap and idempotent once
 * ready, so it is safe to call unconditionally every loop pass. */
uint8_t MagZero_Feed(int16_t d_x, int16_t d_y, int16_t d_z);

/* Apply the baseline. Before the baseline is ready this passes the raw delta
 * through unchanged, so the output is never a half-captured reference. */
void    MagZero_Apply(int16_t d_x, int16_t d_y, int16_t d_z,
                      int16_t *z_x, int16_t *z_y, int16_t *z_z);

/* Read back the captured baseline. Zeros while not ready. Log this alongside
 * the zeroed data so the raw delta stays recoverable. */
void    MagZero_GetBaseline(int16_t *b_x, int16_t *b_y, int16_t *b_z);

/* Clamp an int32 into int16 instead of letting it wrap. Exposed because the
 * raw per-axis delta s1 - s2 spans +/-65534 when the two readings sit at
 * opposite rails, which a plain (int16_t) cast wraps into a large value of the
 * wrong sign. Saturating instead keeps the magnitude monotonic. */
int16_t MagZero_Sat16(int32_t v);

#ifdef __cplusplus
}
#endif

#endif /* MAG_ZERO_H */
