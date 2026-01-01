#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // PI gains in Q2.30
    int32_t kp_q30;
    int32_t ki_q30;

    // Internal phase accumulator in "turns", Q2.30, range [0, 1)
    uint32_t theta_q30;

    // PI integrator in Q2.30
    int32_t integrator_q30;

    // sin/cos (Q2.30) computed from theta
    int32_t sin_q30;
    int32_t cos_q30;

    // Output frequency estimate in Hz, Q(?,25) to match HDL Out_f (sfix32_En25)
    int32_t out_f_q25;

    // Optional: delta component (Hz) in Q25
    int32_t delta_f_q25;
} pll_q30_state_t;

// Fs is compile-time for now (you can make it runtime if needed)
void pll_q30_init(pll_q30_state_t *st, int32_t kp_q30, int32_t ki_q30);
void pll_q30_step(pll_q30_state_t *st, int32_t x_q22);

int32_t pll_q30_step_hdl_io(pll_q30_state_t *st, int32_t x_q22);

#ifdef __cplusplus
}
#endif
