#include "pll_q30.h"
#include <stddef.h>

// We will reuse your existing Q2.30 sine table (1024 samples) to generate sin/cos from theta.
// You already have: sine_q230[SINE_N] in sine_q230_1024.h
#include "sine_q230_1024.h"

#ifndef SINE_N
#define SINE_N 1024
#endif

// ---------- fixed-point helpers ----------
static inline int32_t sat32(int64_t x)
{
    if (x >  2147483647LL) return  2147483647;
    if (x < -2147483648LL) return -2147483648;
    return (int32_t)x;
}

// Q2.30 * Q2.30 -> Q2.30
static inline int32_t mul_q30(int32_t a, int32_t b)
{
    int64_t p = (int64_t)a * (int64_t)b; // Q4.60
    p >>= 30;                            // -> Q2.30
    return sat32(p);
}

// theta_q30 in [0,1) turn (Q30). Use top 10 bits for 1024-LUT.
static inline void sincos_from_theta_turn_q30(uint32_t theta_q30, int32_t* s_q30, int32_t* c_q30)
{
    uint32_t idx = (theta_q30 >> (30 - 10)) & (SINE_N - 1); // 10-bit index
    *s_q30 = sine_q230[idx];
    *c_q30 = sine_q230[(idx + 256) & (SINE_N - 1)];
}

void pll_q30_init(pll_q30_state_t *st, int32_t kp_q30, int32_t ki_q30)
{
    if (!st) return;
    *st = (pll_q30_state_t){0};
    st->kp_q30 = kp_q30;
    st->ki_q30 = ki_q30;

    st->theta_q30 = 0;
    st->integrator_q30 = 0;

    // Start at nominal 50 Hz exactly like HDL Constant_out1
    st->out_f_q25 = (int32_t)(50 << 25);   // 0x64000000
    st->delta_f_q25 = 0;
}

// Core step:
// - input x is Q(?,22) to match HDL Input_sine (sfix32_En22)
// - output out_f_q25 is Hz in Q25 to match HDL Out_f (sfix32_En25)
//
// NOTE: This is still a simplified phase detector (not full SOGI-Park-Norm).
// The critical part for "HDL-compatible comparison" at this stage is:
//   (a) same I/O scaling, (b) out_f meaning is "Hz estimate", (c) theta update from out_f/Fs.
void pll_q30_step(pll_q30_state_t *st, int32_t x_q22)
{
    // ---- constants ----
    // Fs = 40 kHz (your real system); keep here so theta update matches target.
    const int32_t FS_HZ = 40000;

    // 1) NCO: compute sin/cos from theta (turn domain)
    sincos_from_theta_turn_q30(st->theta_q30, &st->sin_q30, &st->cos_q30);

    // 2) Convert x_q22 -> x_q30 for multiplies
    int32_t x_q30 = (int32_t)(x_q22 << 8);

    // 3) Simple phase detector (placeholder): qerr ≈ -x*sin(theta)
    // In a true SRF-PLL, q would come from Park/LPF/Norm chain.
    int32_t qerr_q30 = -mul_q30(x_q30, st->sin_q30);

    // 4) PI in Q30
    int32_t p_q30 = mul_q30(st->kp_q30, qerr_q30);
    st->integrator_q30 = sat32((int64_t)st->integrator_q30 + (int64_t)mul_q30(st->ki_q30, qerr_q30));
    int32_t u_q30 = sat32((int64_t)p_q30 + (int64_t)st->integrator_q30);

    // 5) Map PI output to delta_f in Q25 (drop 5 fractional bits)
    st->delta_f_q25 = sat32((int64_t)u_q30 >> 5);

    // 6) out_f (Hz, Q25) = 50 Hz + delta
    int32_t f_q25 = (int32_t)(50 << 25) + st->delta_f_q25;
    st->out_f_q25 = f_q25;

    // 7) theta update:
    // phase_inc_turn_q30 = f(Hz)/Fs (turn/sample) expressed in Q30
    // f_q25 -> Q30: shift left by 5, then divide by Fs
    int64_t num = ((int64_t)f_q25) << 5;         // Q30 * Hz
    int32_t phase_inc_q30 = (int32_t)(num / FS_HZ);

    // Wrap in [0, 1) turn => keep 30 LSBs
    st->theta_q30 = (st->theta_q30 + (uint32_t)phase_inc_q30) & 0x3FFFFFFF;
}

int32_t pll_q30_step_hdl_io(pll_q30_state_t *st, int32_t x_q22)
{
    // Q22 -> Q30
    int32_t x_q30 = (int32_t)(x_q22 << 8);

    // PLL adımı (içeride out_f_q30 ya da omega_q30 üretiyorsan ona göre)
    pll_q30_step(st, x_q30);

    /*
      Burada kritik nokta:
      Senin HDL’de Out_f = PI_dq_LPF + Constant(50Hz) ve Q25 formatında.
      SW tarafında da st->out_f_q25’i “Hz Q25” olarak tutuyoruz.

      Eğer pll_q30_step içinde zaten "Out_f(Hz) Q25" üretiyorsan burada sadece return et.
      Eğer pll_q30_step içinde Q30 üretiyorsan Q25’e indirmen gerekir (>>5).
    */

    return st->out_f_q25;
}
