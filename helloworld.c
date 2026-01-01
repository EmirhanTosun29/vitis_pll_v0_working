/******************************************************************************
* Copyright (C) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/
/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include <stdint.h>

#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xil_io.h"
#include "sleep.h"

#include "pll_q30.h"
#include "sine_q230_1024.h"



// ---------------- BRAM base ----------------
#ifndef XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR
  #define BRAM_BASE_ADDR   0xC0000000UL
#else
  #define BRAM_BASE_ADDR   XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR
#endif

// ---------------- GPIO probe (AXI GPIO) ----------------
// AXI GPIO register map (channel 1): 0x00 DATA, 0x04 TRI (1=input, 0=output)
#ifndef XPAR_AXI_GPIO_0_BASEADDR
  #error "AXI GPIO base address not found. Check xparameters.h (XPAR_AXI_GPIO_0_BASEADDR)."
#endif

#define GPIO_BASE   XPAR_AXI_GPIO_0_BASEADDR
#define GPIO_DATA   (GPIO_BASE + 0x00)
#define GPIO_TRI    (GPIO_BASE + 0x04)

static inline void probe_init(void) { Xil_Out32(GPIO_TRI, 0x0); } // all outputs
static inline void probe_hi(void)   { Xil_Out32(GPIO_DATA, 0x1); }
static inline void probe_lo(void)   { Xil_Out32(GPIO_DATA, 0x0); }


// ---------------- Cycle counter (RV32/RV64) ----------------
static inline uint64_t rdcycle64(void)
{
#if (__riscv_xlen == 32)
    uint32_t hi0, lo, hi1;
    do {
        asm volatile ("rdcycleh %0" : "=r"(hi0));
        asm volatile ("rdcycle  %0" : "=r"(lo));
        asm volatile ("rdcycleh %0" : "=r"(hi1));
    } while (hi0 != hi1);
    return ((uint64_t)hi0 << 32) | (uint64_t)lo;
#else
    uint64_t v;
    asm volatile ("rdcycle %0" : "=r"(v));
    return v;
#endif
}

// ---------------- Pretty print without float ----------------
// Print signed Qn as decimal with 6 fractional digits.
static void print_qn(const char* tag, int32_t x, int frac_bits)
{
    int32_t sign = 0;
    if (x < 0) { sign = 1; x = -x; }

    int32_t ip = x >> frac_bits;
    uint32_t fp = (uint32_t)(x & ((1u << frac_bits) - 1u));

    // scale fractional to 6 digits
    uint64_t frac6 = ((uint64_t)fp * 1000000ull) >> frac_bits;

    if (sign) xil_printf("%s=-%ld.%06lu", tag, (long)ip, (unsigned long)frac6);
    else      xil_printf("%s=%ld.%06lu",  tag, (long)ip, (unsigned long)frac6);
}

static void dump_word(const char* tag, int idx, uint32_t v)
{
    xil_printf("%s i=%d v=0x%08lx\r\n", tag, idx, (unsigned long)v);
}


int main()
{
    init_platform();
    probe_init();

    xil_printf("\r\n=== SW PLL benchmark (HDL-compatible I/O) + SETTLE ===\r\n");
    xil_printf("BRAM_BASE = 0x%08lx\r\n", (unsigned long)BRAM_BASE_ADDR);

    volatile uint32_t *bram = (volatile uint32_t*)BRAM_BASE_ADDR;

    // 1) BRAM sanity pattern
    xil_printf("Pattern test...\r\n");
    for (int i=0; i<16; i++) bram[i] = 0xA5A50000u + (uint32_t)i;
    for (int i=0; i<16; i++) dump_word("RD", i, bram[i]);

    // 2) Write sine table to BRAM as "sfix32_En22" (Q22) -> Q30 >> 8
    xil_printf("Writing sine table to BRAM as Q22 (from Q30>>8)... N=%d\r\n", SINE_N);
    for (int i=0; i<SINE_N; i++) {
        int32_t q30 = sine_q230[i];
        int32_t q22 = (q30 >> 8);                // Q2.30 -> Q?.22
        bram[i] = (uint32_t)q22;
    }

    xil_printf("Key points (Q22):\r\n");
    dump_word("SINE", 0,   bram[0]);
    dump_word("SINE", 256, bram[256]); // +1.0 -> 0x00400000
    dump_word("SINE", 512, bram[512]);
    dump_word("SINE", 768, bram[768]); // -1.0 -> 0xFFC00000

    // 3) PLL init (kp=0.5, ki=0.00125 in Q2.30)
    pll_q30_state_t st;
    pll_q30_init(&st, 0x20000000, 0x00147AE1);

    // ---------------- Input frequency emulation (phase accumulator) ----------------
    // We want FIN_HZ at FS_HZ using 1024-LUT in BRAM.
    // Use 32-bit phase where top 10 bits select LUT index.
    const uint32_t FS_HZ  = 40000u;   // senin asıl hedefin
    const uint32_t FIN_HZ = 50u;

    // phase_step = FIN/FS * 2^32
    const uint32_t phase_step = (uint32_t)(((uint64_t)FIN_HZ << 32) / (uint64_t)FS_HZ);
    uint32_t phase = 0;

    // 4) SETTLE: measurement dışı yerleşme süresi
    const int SETTLE_SAMPLES = 20000;   // 0.5 s @ 40kHz (istersen 40000 yap -> 1.0 s)
    xil_printf("Settle running... samples=%d (%.3f s)\r\n",
               SETTLE_SAMPLES, (float)SETTLE_SAMPLES/(float)FS_HZ);

    for (int i=0; i<SETTLE_SAMPLES; i++) {
        uint32_t idx = phase >> (32 - 10);       // 0..1023
        int32_t  x_q22 = (int32_t)bram[idx];     // "ADC" sample in Q22
        pll_q30_step(&st, x_q22);         // senin HDL-uyumlu step fonksiyonun
        phase += phase_step;
    }

    // 5) BENCHMARK
    const int N = 1024;   // ölçüm penceresi
    uint64_t t0, t1;

    probe_hi();
    t0 = rdcycle64();

    for (int i=0; i<N; i++) {
        uint32_t idx = phase >> (32 - 10);
        int32_t  x_q22 = (int32_t)bram[idx];
        pll_q30_step(&st, x_q22);
        phase += phase_step;
    }

    t1 = rdcycle64();
    probe_lo();

    uint64_t cyc = (t1 - t0);
    xil_printf("cycles = %lu (N=%d)  cycles/sample = %lu\r\n",
               (unsigned long)cyc, N, (unsigned long)(cyc / (uint64_t)N));

    // 6) End state print (measurement dışı)
    xil_printf("theta_q30=0x%08lx  sin=0x%08lx cos=0x%08lx  Out_f(Q25)=0x%08lx\r\n",
               (unsigned long)st.theta_q30,
               (unsigned long)st.sin_q30,
               (unsigned long)st.cos_q30,
               (unsigned long)st.out_f_q25);
    // 6) End state
    xil_printf("theta_q30=0x%08lx  sin=0x%08lx cos=0x%08lx  Out_f(Q25)=0x%08lx\r\n",
               (unsigned long)st.theta_q30,
               (unsigned long)st.sin_q30,
               (unsigned long)st.cos_q30,
               (unsigned long)st.out_f_q25);

    // Print interpretations:
    // theta: Q30 in turns
    print_qn("theta(turn)", (int32_t)st.theta_q30, 30); xil_printf("\r\n");

    // sin/cos: Q30
    print_qn("sin", st.sin_q30, 30); xil_printf("   ");
    print_qn("cos", st.cos_q30, 30); xil_printf("\r\n");

    // Out_f: Q25 in Hz (HDL-compatible)
    print_qn("Out_f(Hz)", st.out_f_q25, 25); xil_printf("\r\n");

    
    cleanup_platform();
    return 0;
}
