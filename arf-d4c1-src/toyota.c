// Reference source: D4C1-Labs/Flipper-ARF lib/subghz/protocols/toyota.c (main)
// Dual-variant Toyota KeeLoq decoder. Ported to c-toyota.hpp for ARFPORT SubCarRx.
// VARIANT A Corolla 433: PWM pairs LS=0/SL=1, te 400/800, 68 bits.
// VARIANT B Tundra 315: NRZ, preamble ~200/390 + sync ~1938us, midpoint 287us, 67 bits.
// generic.data: [63:32]=hop [31:4]=serial(28) [3:0]=button ; cnt=variant(0/1)
// (full original retained in git history of D4C1-Labs/Flipper-ARF)
