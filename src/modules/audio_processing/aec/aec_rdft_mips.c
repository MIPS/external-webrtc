/*
 * http://www.kurims.kyoto-u.ac.jp/~ooura/fft.html
 * Copyright Takuya OOURA, 1996-2001
 *
 * You may use, copy, modify and distribute this code for any purpose (include
 * commercial use) and without fee. Please refer to this package when you modify
 * this code.
 *
 * Changes by the WebRTC authors:
 *    - Trivial type modifications.
 *    - Minimal code subset to do rdft of length 128.
 *    - Optimizations because of known length.
 *
 *  All changes are covered by the WebRTC license and IP grant:
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "aec_rdft.h"
#include "typedefs.h"
#include <math.h>


void bitrv2_32_mips(int n, int *ip, float *a) {
    // n is 32
    float xr, xi, yr, yi;

    ip[0] = 0;
    ip[1] = 16;

    xr = a[4];
    xi = a[5];
    yr = a[8];
    yi = a[9];
    a[4] = yr;
    a[5] = yi;
    a[8] = xr;
    a[9] = xi;

    xr = a[16];
    xi = a[17];
    yr = a[2];
    yi = a[3];
    a[16] = yr;
    a[17] = yi;
    a[2] = xr;
    a[3] = xi;

    xr = a[20];
    xi = a[21];
    yr = a[10];
    yi = a[11];
    a[20] = yr;
    a[21] = yi;
    a[10] = xr;
    a[11] = xi;

    xr = a[24];
    xi = a[25];
    yr = a[6];
    yi = a[7];
    a[24] = yr;
    a[25] = yi;
    a[6] = xr;
    a[7] = xi;

    xr = a[28];
    xi = a[29];
    yr = a[14];
    yi = a[15];
    a[28] = yr;
    a[29] = yi;
    a[14] = xr;
    a[15] = xi;

    xr = a[22];
    xi = a[23];
    yr = a[26];
    yi = a[27];
    a[22] = yr;
    a[23] = yi;
    a[26] = xr;
    a[27] = xi;
}

void bitrv2_128_mips(int n, int *ip, float *a) {
    // n is 128
    float xr, xi, yr, yi;

    ip[0] = 0;
    ip[1] = 64;
    ip[2] = 32;
    ip[3] = 96;

    xr = a[8];
    xi = a[9];
    yr = a[16];
    yi = a[17];
    a[8] = yr;
    a[9] = yi;
    a[16] = xr;
    a[17] = xi;

    xr = a[64];
    xi = a[65];
    yr = a[2];
    yi = a[3];
    a[64] = yr;
    a[65] = yi;
    a[2] = xr;
    a[3] = xi;

    xr = a[72];
    xi = a[73];
    yr = a[18];
    yi = a[19];
    a[72] = yr;
    a[73] = yi;
    a[18] = xr;
    a[19] = xi;

    xr = a[80];
    xi = a[81];
    yr = a[10];
    yi = a[11];
    a[80] = yr;
    a[81] = yi;
    a[10] = xr;
    a[11] = xi;

    xr = a[88];
    xi = a[89];
    yr = a[26];
    yi = a[27];
    a[88] = yr;
    a[89] = yi;
    a[26] = xr;
    a[27] = xi;

    xr = a[74];
    xi = a[75];
    yr = a[82];
    yi = a[83];
    a[74] = yr;
    a[75] = yi;
    a[82] = xr;
    a[83] = xi;

    xr = a[32];
    xi = a[33];
    yr = a[4];
    yi = a[5];
    a[32] = yr;
    a[33] = yi;
    a[4] = xr;
    a[5] = xi;

    xr = a[40];
    xi = a[41];
    yr = a[20];
    yi = a[21];
    a[40] = yr;
    a[41] = yi;
    a[20] = xr;
    a[21] = xi;

    xr = a[48];
    xi = a[49];
    yr = a[12];
    yi = a[13];
    a[48] = yr;
    a[49] = yi;
    a[12] = xr;
    a[13] = xi;

    xr = a[56];
    xi = a[57];
    yr = a[28];
    yi = a[29];
    a[56] = yr;
    a[57] = yi;
    a[28] = xr;
    a[29] = xi;

    xr = a[34];
    xi = a[35];
    yr = a[68];
    yi = a[69];
    a[34] = yr;
    a[35] = yi;
    a[68] = xr;
    a[69] = xi;

    xr = a[42];
    xi = a[43];
    yr = a[84];
    yi = a[85];
    a[42] = yr;
    a[43] = yi;
    a[84] = xr;
    a[85] = xi;

    xr = a[50];
    xi = a[51];
    yr = a[76];
    yi = a[77];
    a[50] = yr;
    a[51] = yi;
    a[76] = xr;
    a[77] = xi;

    xr = a[58];
    xi = a[59];
    yr = a[92];
    yi = a[93];
    a[58] = yr;
    a[59] = yi;
    a[92] = xr;
    a[93] = xi;

    xr = a[44];
    xi = a[45];
    yr = a[52];
    yi = a[53];
    a[44] = yr;
    a[45] = yi;
    a[52] = xr;
    a[53] = xi;

    xr = a[96];
    xi = a[97];
    yr = a[6];
    yi = a[7];
    a[96] = yr;
    a[97] = yi;
    a[6] = xr;
    a[7] = xi;

    xr = a[104];
    xi = a[105];
    yr = a[22];
    yi = a[23];
    a[104] = yr;
    a[105] = yi;
    a[22] = xr;
    a[23] = xi;

    xr = a[112];
    xi = a[113];
    yr = a[14];
    yi = a[15];
    a[112] = yr;
    a[113] = yi;
    a[14] = xr;
    a[15] = xi;

    xr = a[120];
    xi = a[121];
    yr = a[30];
    yi = a[31];
    a[120] = yr;
    a[121] = yi;
    a[30] = xr;
    a[31] = xi;

    xr = a[98];
    xi = a[99];
    yr = a[70];
    yi = a[71];
    a[98] = yr;
    a[99] = yi;
    a[70] = xr;
    a[71] = xi;

    xr = a[106];
    xi = a[107];
    yr = a[86];
    yi = a[87];
    a[106] = yr;
    a[107] = yi;
    a[86] = xr;
    a[87] = xi;

    xr = a[114];
    xi = a[115];
    yr = a[78];
    yi = a[79];
    a[114] = yr;
    a[115] = yi;
    a[78] = xr;
    a[79] = xi;

    xr = a[122];
    xi = a[123];
    yr = a[94];
    yi = a[95];
    a[122] = yr;
    a[123] = yi;
    a[94] = xr;
    a[95] = xi;

    xr = a[100];
    xi = a[101];
    yr = a[38];
    yi = a[39];
    a[100] = yr;
    a[101] = yi;
    a[38] = xr;
    a[39] = xi;

    xr = a[108];
    xi = a[109];
    yr = a[54];
    yi = a[55];
    a[108] = yr;
    a[109] = yi;
    a[54] = xr;
    a[55] = xi;

    xr = a[116];
    xi = a[117];
    yr = a[46];
    yi = a[47];
    a[116] = yr;
    a[117] = yi;
    a[46] = xr;
    a[47] = xi;

    xr = a[124];
    xi = a[125];
    yr = a[62];
    yi = a[63];
    a[124] = yr;
    a[125] = yi;
    a[62] = xr;
    a[63] = xi;

    xr = a[110];
    xi = a[111];
    yr = a[118];
    yi = a[119];
    a[110] = yr;
    a[111] = yi;
    a[118] = xr;
    a[119] = xi;
}

#if defined(MIPS32_LE) & defined(MIPS_FPU_LE)
void cft1st_128_mips(float *a) {
  float wk1r, wk1i, wk2r, wk2i, wk3r, wk3i;
  float x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;

  float f0, f1, f2, f3, f4, f5, f6, f7;
  __asm__ volatile(
        ".set push                                                              \n\t"
        ".set noreorder                                                         \n\t"
        "lwc1       %[f0],          0(%[a])                                     \n\t"
        "lwc1       %[f1],          4(%[a])                                     \n\t"
        "lwc1       %[f2],          8(%[a])                                     \n\t"
        "lwc1       %[f3],          12(%[a])                                    \n\t"
        "lwc1       %[f4],          16(%[a])                                    \n\t"
        "lwc1       %[f5],          20(%[a])                                    \n\t"
        "lwc1       %[f6],          24(%[a])                                    \n\t"
        "lwc1       %[f7],          28(%[a])                                    \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "sub.s      %[f4],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f5],          %[x0i],         %[x2i]                      \n\t"
        "sub.s      %[f2],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f3],          %[x1i],         %[x3r]                      \n\t"
        "add.s      %[f6],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f7],          %[x1i],         %[x3r]                      \n\t"

        "swc1       %[f0],          0(%[a])                                     \n\t"
        "swc1       %[f1],          4(%[a])                                     \n\t"
        "swc1       %[f2],          8(%[a])                                     \n\t"
        "swc1       %[f3],          12(%[a])                                    \n\t"
        "swc1       %[f4],          16(%[a])                                    \n\t"
        "swc1       %[f5],          20(%[a])                                    \n\t"
        "swc1       %[f6],          24(%[a])                                    \n\t"
        "swc1       %[f7],          28(%[a])                                    \n\t"

        "lwc1       %[f0],          32(%[a])                                    \n\t"
        "lwc1       %[f1],          36(%[a])                                    \n\t"
        "lwc1       %[f2],          40(%[a])                                    \n\t"
        "lwc1       %[f3],          44(%[a])                                    \n\t"
        "lwc1       %[f4],          48(%[a])                                    \n\t"
        "lwc1       %[f5],          52(%[a])                                    \n\t"
        "lwc1       %[f6],          56(%[a])                                    \n\t"
        "lwc1       %[f7],          60(%[a])                                    \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"

        "lwc1       %[wk2r],        8(%[rdft_w])                                \n\t"

        "add.s      %[f3],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f2],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f6],          %[x3i],         %[x1r]                      \n\t"
        "sub.s      %[f7],          %[x3r],         %[x1i]                      \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "sub.s      %[x1r],         %[f2],          %[f3]                       \n\t"
        "add.s      %[x1i],         %[f3],          %[f2]                       \n\t"
        "sub.s      %[x3r],         %[f7],          %[f6]                       \n\t"
        "add.s      %[x3i],         %[f7],          %[f6]                       \n\t"
        "sub.s      %[f4],          %[x0r],         %[x2r]                      \n\t"
        "mul.s      %[f2],          %[wk2r],        %[x1r]                      \n\t"
        "mul.s      %[f3],          %[wk2r],        %[x1i]                      \n\t"
        "mul.s      %[f6],          %[wk2r],        %[x3r]                      \n\t"
        "mul.s      %[f7],          %[wk2r],        %[x3i]                      \n\t"

        "sub.s      %[f5],          %[x2i],         %[x0i]                      \n\t"

        "swc1       %[f0],          32(%[a])                                    \n\t"
        "swc1       %[f1],          36(%[a])                                    \n\t"
        "swc1       %[f2],          40(%[a])                                    \n\t"
        "swc1       %[f3],          44(%[a])                                    \n\t"
        "swc1       %[f5],          48(%[a])                                    \n\t"
        "swc1       %[f4],          52(%[a])                                    \n\t"
        "swc1       %[f6],          56(%[a])                                    \n\t"
        "swc1       %[f7],          60(%[a])                                    \n\t"

        "lwc1       %[f0],          64(%[a])                                    \n\t"
        "lwc1       %[f1],          68(%[a])                                    \n\t"
        "lwc1       %[f2],          72(%[a])                                    \n\t"
        "lwc1       %[f3],          76(%[a])                                    \n\t"
        "lwc1       %[f4],          80(%[a])                                    \n\t"
        "lwc1       %[f5],          84(%[a])                                    \n\t"
        "lwc1       %[f6],          88(%[a])                                    \n\t"
        "lwc1       %[f7],          92(%[a])                                    \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "lwc1       %[wk2i],        12(%[rdft_w])                               \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "lwc1       %[wk1r],        16(%[rdft_w])                               \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "lwc1       %[wk3r],        8(%[rdft_wk3ri_first])                      \n\t"

        "mul.s      %[x3r],         %[wk2r],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "lwc1       %[wk1i],        20(%[rdft_w])                               \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"
        "lwc1       %[wk3i],        12(%[rdft_wk3ri_first])                     \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[wk1r],        %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2i],        %[f0]                       \n\t"
        "sub.s      %[x3r],         %[x3r],         %[wk1r]                     \n\t"
        "add.s      %[x3i],         %[x3i],         %[f0]                       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[x0r],         %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[x2r],         %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[x0r]                      \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[x2r]                      \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmsub.s    %[x3r],         %[x3r],         %[wk2i],        %[f1]       \n\t"
        "madd.s     %[x3i],         %[x3i],         %[wk2i],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          64(%[a])                                    \n\t"
        "swc1       %[f1],          68(%[a])                                    \n\t"
        "swc1       %[x1r],         72(%[a])                                    \n\t"
        "swc1       %[x1i],         76(%[a])                                    \n\t"
        "swc1       %[x3r],         80(%[a])                                    \n\t"
        "swc1       %[x3i],         84(%[a])                                    \n\t"
        "swc1       %[f6],          88(%[a])                                    \n\t"
        "swc1       %[f7],          92(%[a])                                    \n\t"

        "lwc1       %[f0],          96(%[a])                                    \n\t"
        "lwc1       %[f1],          100(%[a])                                   \n\t"
        "lwc1       %[f2],          104(%[a])                                   \n\t"
        "lwc1       %[f3],          108(%[a])                                   \n\t"
        "lwc1       %[f4],          112(%[a])                                   \n\t"
        "lwc1       %[f5],          116(%[a])                                   \n\t"
        "lwc1       %[f6],          120(%[a])                                   \n\t"
        "lwc1       %[f7],          124(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "lwc1       %[wk1r],        24(%[rdft_w])                               \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "lwc1       %[wk3r],        8(%[rdft_wk3ri_second])                     \n\t"

        "mul.s      %[x3r],         %[wk2i],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"

        "lwc1       %[wk1i],        28(%[rdft_w])                               \n\t"
        "lwc1       %[wk3i],        12(%[rdft_wk3ri_second])                    \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[wk1r],        %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2r],        %[f0]                       \n\t"
        "add.s      %[x3r],         %[x3r],         %[wk1r]                     \n\t"
        "neg.s      %[x3r],         %[x3r]                                      \n\t"
        "sub.s      %[x3i],         %[f0],          %[x3i]                      \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[x0r],         %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[x2r],         %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[x0r]                      \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[x2r]                      \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmadd.s    %[x3r],         %[x3r],         %[wk2r],        %[f1]       \n\t"
        "msub.s     %[x3i],         %[x3i],         %[wk2r],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          96(%[a])                                    \n\t"
        "swc1       %[f1],          100(%[a])                                   \n\t"
        "swc1       %[x1r],         104(%[a])                                   \n\t"
        "swc1       %[x1i],         108(%[a])                                   \n\t"
        "swc1       %[x3r],         112(%[a])                                   \n\t"
        "swc1       %[x3i],         116(%[a])                                   \n\t"
        "swc1       %[f6],          120(%[a])                                   \n\t"
        "swc1       %[f7],          124(%[a])                                   \n\t"

        "lwc1       %[f0],          128(%[a])                                   \n\t"
        "lwc1       %[f1],          132(%[a])                                   \n\t"
        "lwc1       %[f2],          136(%[a])                                   \n\t"
        "lwc1       %[f3],          140(%[a])                                   \n\t"
        "lwc1       %[f4],          144(%[a])                                   \n\t"
        "lwc1       %[f5],          148(%[a])                                   \n\t"
        "lwc1       %[f6],          152(%[a])                                   \n\t"
        "lwc1       %[f7],          156(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "lwc1       %[wk2r],        16(%[rdft_w])                               \n\t"
        "lwc1       %[wk2i],        20(%[rdft_w])                               \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "lwc1       %[wk1r],        32(%[rdft_w])                               \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "lwc1       %[wk3r],        16(%[rdft_wk3ri_first])                     \n\t"

        "mul.s      %[x3r],         %[wk2r],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "lwc1       %[wk1i],        36(%[rdft_w])                               \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"
        "lwc1       %[wk3i],        20(%[rdft_wk3ri_first])                     \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[wk1r],        %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2i],        %[f0]                       \n\t"
        "sub.s      %[x3r],         %[x3r],         %[wk1r]                     \n\t"
        "add.s      %[x3i],         %[x3i],         %[f0]                       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[x0r],         %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[x2r],         %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[x0r]                      \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[x2r]                      \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmsub.s    %[x3r],         %[x3r],         %[wk2i],        %[f1]       \n\t"
        "madd.s     %[x3i],         %[x3i],         %[wk2i],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          128(%[a])                                   \n\t"
        "swc1       %[f1],          132(%[a])                                   \n\t"
        "swc1       %[x1r],         136(%[a])                                   \n\t"
        "swc1       %[x1i],         140(%[a])                                   \n\t"
        "swc1       %[x3r],         144(%[a])                                   \n\t"
        "swc1       %[x3i],         148(%[a])                                   \n\t"
        "swc1       %[f6],          152(%[a])                                   \n\t"
        "swc1       %[f7],          156(%[a])                                   \n\t"

        "lwc1       %[f0],          160(%[a])                                   \n\t"
        "lwc1       %[f1],          164(%[a])                                   \n\t"
        "lwc1       %[f2],          168(%[a])                                   \n\t"
        "lwc1       %[f3],          172(%[a])                                   \n\t"
        "lwc1       %[f4],          176(%[a])                                   \n\t"
        "lwc1       %[f5],          180(%[a])                                   \n\t"
        "lwc1       %[f6],          184(%[a])                                   \n\t"
        "lwc1       %[f7],          188(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "lwc1       %[wk1r],        40(%[rdft_w])                               \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "lwc1       %[wk3r],        16(%[rdft_wk3ri_second])                    \n\t"

        "mul.s      %[x3r],         %[wk2i],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"

        "lwc1       %[wk1i],        44(%[rdft_w])                               \n\t"
        "lwc1       %[wk3i],        20(%[rdft_wk3ri_second])                    \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[wk1r],        %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2r],        %[f0]                       \n\t"
        "add.s      %[x3r],         %[x3r],         %[wk1r]                     \n\t"
        "neg.s      %[x3r],         %[x3r]                                      \n\t"
        "sub.s      %[x3i],         %[f0],          %[x3i]                      \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[x0r],         %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[x2r],         %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[x0r]                      \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[x2r]                      \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmadd.s    %[x3r],         %[x3r],         %[wk2r],        %[f1]       \n\t"
        "msub.s     %[x3i],         %[x3i],         %[wk2r],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          160(%[a])                                   \n\t"
        "swc1       %[f1],          164(%[a])                                   \n\t"
        "swc1       %[x1r],         168(%[a])                                   \n\t"
        "swc1       %[x1i],         172(%[a])                                   \n\t"
        "swc1       %[x3r],         176(%[a])                                   \n\t"
        "swc1       %[x3i],         180(%[a])                                   \n\t"
        "swc1       %[f6],          184(%[a])                                   \n\t"
        "swc1       %[f7],          188(%[a])                                   \n\t"

        "lwc1       %[f0],          192(%[a])                                   \n\t"
        "lwc1       %[f1],          196(%[a])                                   \n\t"
        "lwc1       %[f2],          200(%[a])                                   \n\t"
        "lwc1       %[f3],          204(%[a])                                   \n\t"
        "lwc1       %[f4],          208(%[a])                                   \n\t"
        "lwc1       %[f5],          212(%[a])                                   \n\t"
        "lwc1       %[f6],          216(%[a])                                   \n\t"
        "lwc1       %[f7],          220(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "lwc1       %[wk2r],        24(%[rdft_w])                               \n\t"
        "lwc1       %[wk2i],        28(%[rdft_w])                               \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "lwc1       %[wk1r],        48(%[rdft_w])                               \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "lwc1       %[wk3r],        24(%[rdft_wk3ri_first])                     \n\t"

        "mul.s      %[x3r],         %[wk2r],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "lwc1       %[wk1i],        52(%[rdft_w])                               \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"
        "lwc1       %[wk3i],        28(%[rdft_wk3ri_first])                     \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[wk1r],        %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2i],        %[f0]                       \n\t"
        "sub.s      %[x3r],         %[x3r],         %[wk1r]                     \n\t"
        "add.s      %[x3i],         %[x3i],         %[f0]                       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[x0r],         %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[x2r],         %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[x0r]                      \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[x2r]                      \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmsub.s    %[x3r],         %[x3r],         %[wk2i],        %[f1]       \n\t"
        "madd.s     %[x3i],         %[x3i],         %[wk2i],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          192(%[a])                                   \n\t"
        "swc1       %[f1],          196(%[a])                                   \n\t"
        "swc1       %[x1r],         200(%[a])                                   \n\t"
        "swc1       %[x1i],         204(%[a])                                   \n\t"
        "swc1       %[x3r],         208(%[a])                                   \n\t"
        "swc1       %[x3i],         212(%[a])                                   \n\t"
        "swc1       %[f6],          216(%[a])                                   \n\t"
        "swc1       %[f7],          220(%[a])                                   \n\t"

        "lwc1       %[f0],          224(%[a])                                   \n\t"
        "lwc1       %[f1],          228(%[a])                                   \n\t"
        "lwc1       %[f2],          232(%[a])                                   \n\t"
        "lwc1       %[f3],          236(%[a])                                   \n\t"
        "lwc1       %[f4],          240(%[a])                                   \n\t"
        "lwc1       %[f5],          244(%[a])                                   \n\t"
        "lwc1       %[f6],          248(%[a])                                   \n\t"
        "lwc1       %[f7],          252(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "lwc1       %[wk1r],        56(%[rdft_w])                               \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "lwc1       %[wk3r],        24(%[rdft_wk3ri_second])                    \n\t"

        "mul.s      %[x3r],         %[wk2i],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"

        "lwc1       %[wk1i],        60(%[rdft_w])                               \n\t"
        "lwc1       %[wk3i],        28(%[rdft_wk3ri_second])                    \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[wk1r],        %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2r],        %[f0]                       \n\t"
        "add.s      %[x3r],         %[x3r],         %[wk1r]                     \n\t"
        "neg.s      %[x3r],         %[x3r]                                      \n\t"
        "sub.s      %[x3i],         %[f0],          %[x3i]                      \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[x0r],         %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[x2r],         %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[x0r]                      \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[x2r]                      \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmadd.s    %[x3r],         %[x3r],         %[wk2r],        %[f1]       \n\t"
        "msub.s     %[x3i],         %[x3i],         %[wk2r],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          224(%[a])                                   \n\t"
        "swc1       %[f1],          228(%[a])                                   \n\t"
        "swc1       %[x1r],         232(%[a])                                   \n\t"
        "swc1       %[x1i],         236(%[a])                                   \n\t"
        "swc1       %[x3r],         240(%[a])                                   \n\t"
        "swc1       %[x3i],         244(%[a])                                   \n\t"
        "swc1       %[f6],          248(%[a])                                   \n\t"
        "swc1       %[f7],          252(%[a])                                   \n\t"

        "lwc1       %[f0],          256(%[a])                                   \n\t"
        "lwc1       %[f1],          260(%[a])                                   \n\t"
        "lwc1       %[f2],          264(%[a])                                   \n\t"
        "lwc1       %[f3],          268(%[a])                                   \n\t"
        "lwc1       %[f4],          272(%[a])                                   \n\t"
        "lwc1       %[f5],          276(%[a])                                   \n\t"
        "lwc1       %[f6],          280(%[a])                                   \n\t"
        "lwc1       %[f7],          284(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "lwc1       %[wk2r],        32(%[rdft_w])                               \n\t"
        "lwc1       %[wk2i],        36(%[rdft_w])                               \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "lwc1       %[wk1r],        64(%[rdft_w])                               \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "lwc1       %[wk3r],        32(%[rdft_wk3ri_first])                     \n\t"

        "mul.s      %[x3r],         %[wk2r],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "lwc1       %[wk1i],        68(%[rdft_w])                               \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"
        "lwc1       %[wk3i],        36(%[rdft_wk3ri_first])                     \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[wk1r],        %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2i],        %[f0]                       \n\t"
        "sub.s      %[x3r],         %[x3r],         %[wk1r]                     \n\t"
        "add.s      %[x3i],         %[x3i],         %[f0]                       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[x0r],         %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[x2r],         %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[x0r]                      \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[x2r]                      \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmsub.s    %[x3r],         %[x3r],         %[wk2i],        %[f1]       \n\t"
        "madd.s     %[x3i],         %[x3i],         %[wk2i],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          256(%[a])                                   \n\t"
        "swc1       %[f1],          260(%[a])                                   \n\t"
        "swc1       %[x1r],         264(%[a])                                   \n\t"
        "swc1       %[x1i],         268(%[a])                                   \n\t"
        "swc1       %[x3r],         272(%[a])                                   \n\t"
        "swc1       %[x3i],         276(%[a])                                   \n\t"
        "swc1       %[f6],          280(%[a])                                   \n\t"
        "swc1       %[f7],          284(%[a])                                   \n\t"

        "lwc1       %[f0],          288(%[a])                                   \n\t"
        "lwc1       %[f1],          292(%[a])                                   \n\t"
        "lwc1       %[f2],          296(%[a])                                   \n\t"
        "lwc1       %[f3],          300(%[a])                                   \n\t"
        "lwc1       %[f4],          304(%[a])                                   \n\t"
        "lwc1       %[f5],          308(%[a])                                   \n\t"
        "lwc1       %[f6],          312(%[a])                                   \n\t"
        "lwc1       %[f7],          316(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "lwc1       %[wk1r],        72(%[rdft_w])                               \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "lwc1       %[wk3r],        32(%[rdft_wk3ri_second])                    \n\t"

        "mul.s      %[x3r],         %[wk2i],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"

        "lwc1       %[wk1i],        76(%[rdft_w])                               \n\t"
        "lwc1       %[wk3i],        36(%[rdft_wk3ri_second])                    \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[wk1r],        %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2r],        %[f0]                       \n\t"
        "add.s      %[x3r],         %[x3r],         %[wk1r]                     \n\t"
        "neg.s      %[x3r],         %[x3r]                                      \n\t"
        "sub.s      %[x3i],         %[f0],          %[x3i]                      \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[x0r],         %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[x2r],         %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[x0r]                      \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[x2r]                      \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmadd.s    %[x3r],         %[x3r],         %[wk2r],        %[f1]       \n\t"
        "msub.s     %[x3i],         %[x3i],         %[wk2r],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          288(%[a])                                   \n\t"
        "swc1       %[f1],          292(%[a])                                   \n\t"
        "swc1       %[x1r],         296(%[a])                                   \n\t"
        "swc1       %[x1i],         300(%[a])                                   \n\t"
        "swc1       %[x3r],         304(%[a])                                   \n\t"
        "swc1       %[x3i],         308(%[a])                                   \n\t"
        "swc1       %[f6],          312(%[a])                                   \n\t"
        "swc1       %[f7],          316(%[a])                                   \n\t"

        "lwc1       %[f0],          320(%[a])                                   \n\t"
        "lwc1       %[f1],          324(%[a])                                   \n\t"
        "lwc1       %[f2],          328(%[a])                                   \n\t"
        "lwc1       %[f3],          332(%[a])                                   \n\t"
        "lwc1       %[f4],          336(%[a])                                   \n\t"
        "lwc1       %[f5],          340(%[a])                                   \n\t"
        "lwc1       %[f6],          344(%[a])                                   \n\t"
        "lwc1       %[f7],          348(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "lwc1       %[wk2r],        40(%[rdft_w])                               \n\t"
        "lwc1       %[wk2i],        44(%[rdft_w])                               \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "lwc1       %[wk1r],        80(%[rdft_w])                               \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "lwc1       %[wk3r],        40(%[rdft_wk3ri_first])                     \n\t"

        "mul.s      %[x3r],         %[wk2r],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "lwc1       %[wk1i],        84(%[rdft_w])                               \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"
        "lwc1       %[wk3i],        44(%[rdft_wk3ri_first])                     \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[wk1r],        %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2i],        %[f0]                       \n\t"
        "sub.s      %[x3r],         %[x3r],         %[wk1r]                     \n\t"
        "add.s      %[x3i],         %[x3i],         %[f0]                       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[x0r],         %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[x2r],         %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[x0r]                      \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[x2r]                      \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmsub.s    %[x3r],         %[x3r],         %[wk2i],        %[f1]       \n\t"
        "madd.s     %[x3i],         %[x3i],         %[wk2i],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          320(%[a])                                   \n\t"
        "swc1       %[f1],          324(%[a])                                   \n\t"
        "swc1       %[x1r],         328(%[a])                                   \n\t"
        "swc1       %[x1i],         332(%[a])                                   \n\t"
        "swc1       %[x3r],         336(%[a])                                   \n\t"
        "swc1       %[x3i],         340(%[a])                                   \n\t"
        "swc1       %[f6],          344(%[a])                                   \n\t"
        "swc1       %[f7],          348(%[a])                                   \n\t"

        "lwc1       %[f0],          352(%[a])                                   \n\t"
        "lwc1       %[f1],          356(%[a])                                   \n\t"
        "lwc1       %[f2],          360(%[a])                                   \n\t"
        "lwc1       %[f3],          364(%[a])                                   \n\t"
        "lwc1       %[f4],          368(%[a])                                   \n\t"
        "lwc1       %[f5],          372(%[a])                                   \n\t"
        "lwc1       %[f6],          376(%[a])                                   \n\t"
        "lwc1       %[f7],          380(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "lwc1       %[wk1r],        88(%[rdft_w])                               \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "lwc1       %[wk3r],        40(%[rdft_wk3ri_second])                    \n\t"

        "mul.s      %[x3r],         %[wk2i],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"

        "lwc1       %[wk1i],        92(%[rdft_w])                               \n\t"
        "lwc1       %[wk3i],        44(%[rdft_wk3ri_second])                    \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[wk1r],        %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2r],        %[f0]                       \n\t"
        "add.s      %[x3r],         %[x3r],         %[wk1r]                     \n\t"
        "neg.s      %[x3r],         %[x3r]                                      \n\t"
        "sub.s      %[x3i],         %[f0],          %[x3i]                      \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[x0r],         %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[x2r],         %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[x0r]                      \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[x2r]                      \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmadd.s    %[x3r],         %[x3r],         %[wk2r],        %[f1]       \n\t"
        "msub.s     %[x3i],         %[x3i],         %[wk2r],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          352(%[a])                                   \n\t"
        "swc1       %[f1],          356(%[a])                                   \n\t"
        "swc1       %[x1r],         360(%[a])                                   \n\t"
        "swc1       %[x1i],         364(%[a])                                   \n\t"
        "swc1       %[x3r],         368(%[a])                                   \n\t"
        "swc1       %[x3i],         372(%[a])                                   \n\t"
        "swc1       %[f6],          376(%[a])                                   \n\t"
        "swc1       %[f7],          380(%[a])                                   \n\t"

        "lwc1       %[f0],          384(%[a])                                   \n\t"
        "lwc1       %[f1],          388(%[a])                                   \n\t"
        "lwc1       %[f2],          392(%[a])                                   \n\t"
        "lwc1       %[f3],          396(%[a])                                   \n\t"
        "lwc1       %[f4],          400(%[a])                                   \n\t"
        "lwc1       %[f5],          404(%[a])                                   \n\t"
        "lwc1       %[f6],          408(%[a])                                   \n\t"
        "lwc1       %[f7],          412(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "lwc1       %[wk2r],        48(%[rdft_w])                               \n\t"
        "lwc1       %[wk2i],        52(%[rdft_w])                               \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "lwc1       %[wk1r],        96(%[rdft_w])                               \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "lwc1       %[wk3r],        48(%[rdft_wk3ri_first])                     \n\t"

        "mul.s      %[x3r],         %[wk2r],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "lwc1       %[wk1i],        100(%[rdft_w])                              \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"
        "lwc1       %[wk3i],        52(%[rdft_wk3ri_first])                     \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[wk1r],        %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2i],        %[f0]                       \n\t"
        "sub.s      %[x3r],         %[x3r],         %[wk1r]                     \n\t"
        "add.s      %[x3i],         %[x3i],         %[f0]                       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[x0r],         %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[x2r],         %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[x0r]                      \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[x2r]                      \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmsub.s    %[x3r],         %[x3r],         %[wk2i],        %[f1]       \n\t"
        "madd.s     %[x3i],         %[x3i],         %[wk2i],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          384(%[a])                                   \n\t"
        "swc1       %[f1],          388(%[a])                                   \n\t"
        "swc1       %[x1r],         392(%[a])                                   \n\t"
        "swc1       %[x1i],         396(%[a])                                   \n\t"
        "swc1       %[x3r],         400(%[a])                                   \n\t"
        "swc1       %[x3i],         404(%[a])                                   \n\t"
        "swc1       %[f6],          408(%[a])                                   \n\t"
        "swc1       %[f7],          412(%[a])                                   \n\t"

        "lwc1       %[f0],          416(%[a])                                   \n\t"
        "lwc1       %[f1],          420(%[a])                                   \n\t"
        "lwc1       %[f2],          424(%[a])                                   \n\t"
        "lwc1       %[f3],          428(%[a])                                   \n\t"
        "lwc1       %[f4],          432(%[a])                                   \n\t"
        "lwc1       %[f5],          436(%[a])                                   \n\t"
        "lwc1       %[f6],          440(%[a])                                   \n\t"
        "lwc1       %[f7],          444(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "lwc1       %[wk1r],        104(%[rdft_w])                              \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "lwc1       %[wk3r],        48(%[rdft_wk3ri_second])                    \n\t"

        "mul.s      %[x3r],         %[wk2i],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"

        "lwc1       %[wk1i],        108(%[rdft_w])                              \n\t"
        "lwc1       %[wk3i],        52(%[rdft_wk3ri_second])                    \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[wk1r],        %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2r],        %[f0]                       \n\t"
        "add.s      %[x3r],         %[x3r],         %[wk1r]                     \n\t"
        "neg.s      %[x3r],         %[x3r]                                      \n\t"
        "sub.s      %[x3i],         %[f0],          %[x3i]                      \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[x0r],         %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[x2r],         %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[x0r]                      \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[x2r]                      \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmadd.s    %[x3r],         %[x3r],         %[wk2r],        %[f1]       \n\t"
        "msub.s     %[x3i],         %[x3i],         %[wk2r],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          416(%[a])                                   \n\t"
        "swc1       %[f1],          420(%[a])                                   \n\t"
        "swc1       %[x1r],         424(%[a])                                   \n\t"
        "swc1       %[x1i],         428(%[a])                                   \n\t"
        "swc1       %[x3r],         432(%[a])                                   \n\t"
        "swc1       %[x3i],         436(%[a])                                   \n\t"
        "swc1       %[f6],          440(%[a])                                   \n\t"
        "swc1       %[f7],          444(%[a])                                   \n\t"

        "lwc1       %[f0],          448(%[a])                                   \n\t"
        "lwc1       %[f1],          452(%[a])                                   \n\t"
        "lwc1       %[f2],          456(%[a])                                   \n\t"
        "lwc1       %[f3],          460(%[a])                                   \n\t"
        "lwc1       %[f4],          464(%[a])                                   \n\t"
        "lwc1       %[f5],          468(%[a])                                   \n\t"
        "lwc1       %[f6],          472(%[a])                                   \n\t"
        "lwc1       %[f7],          476(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "lwc1       %[wk2r],        56(%[rdft_w])                               \n\t"
        "lwc1       %[wk2i],        60(%[rdft_w])                               \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "lwc1       %[wk1r],        112(%[rdft_w])                              \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "lwc1       %[wk3r],        56(%[rdft_wk3ri_first])                     \n\t"

        "mul.s      %[x3r],         %[wk2r],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "lwc1       %[wk1i],        116(%[rdft_w])                              \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"
        "lwc1       %[wk3i],        60(%[rdft_wk3ri_first])                     \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[wk1r],        %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2i],        %[f0]                       \n\t"
        "sub.s      %[x3r],         %[x3r],         %[wk1r]                     \n\t"
        "add.s      %[x3i],         %[x3i],         %[f0]                       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[x0r],         %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[x2r],         %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[x0r]                      \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[x2r]                      \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmsub.s    %[x3r],         %[x3r],         %[wk2i],        %[f1]       \n\t"
        "madd.s     %[x3i],         %[x3i],         %[wk2i],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          448(%[a])                                   \n\t"
        "swc1       %[f1],          452(%[a])                                   \n\t"
        "swc1       %[x1r],         456(%[a])                                   \n\t"
        "swc1       %[x1i],         460(%[a])                                   \n\t"
        "swc1       %[x3r],         464(%[a])                                   \n\t"
        "swc1       %[x3i],         468(%[a])                                   \n\t"
        "swc1       %[f6],          472(%[a])                                   \n\t"
        "swc1       %[f7],          476(%[a])                                   \n\t"

        "lwc1       %[f0],          480(%[a])                                   \n\t"
        "lwc1       %[f1],          484(%[a])                                   \n\t"
        "lwc1       %[f2],          488(%[a])                                   \n\t"
        "lwc1       %[f3],          492(%[a])                                   \n\t"
        "lwc1       %[f4],          496(%[a])                                   \n\t"
        "lwc1       %[f5],          500(%[a])                                   \n\t"
        "lwc1       %[f6],          504(%[a])                                   \n\t"
        "lwc1       %[f7],          508(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "lwc1       %[wk1r],        120(%[rdft_w])                              \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "lwc1       %[wk3r],        56(%[rdft_wk3ri_second])                    \n\t"

        "mul.s      %[x3r],         %[wk2i],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"

        "lwc1       %[wk1i],        124(%[rdft_w])                              \n\t"
        "lwc1       %[wk3i],        60(%[rdft_wk3ri_second])                    \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[wk1r],        %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2r],        %[f0]                       \n\t"
        "add.s      %[x3r],         %[x3r],         %[wk1r]                     \n\t"
        "neg.s      %[x3r],         %[x3r]                                      \n\t"
        "sub.s      %[x3i],         %[f0],          %[x3i]                      \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[x0r],         %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[x2r],         %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[x0r]                      \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[x2r]                      \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmadd.s    %[x3r],         %[x3r],         %[wk2r],        %[f1]       \n\t"
        "msub.s     %[x3i],         %[x3i],         %[wk2r],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          480(%[a])                                   \n\t"
        "swc1       %[f1],          484(%[a])                                   \n\t"
        "swc1       %[x1r],         488(%[a])                                   \n\t"
        "swc1       %[x1i],         492(%[a])                                   \n\t"
        "swc1       %[x3r],         496(%[a])                                   \n\t"
        "swc1       %[x3i],         500(%[a])                                   \n\t"
        "swc1       %[f6],          504(%[a])                                   \n\t"
        "swc1       %[f7],          508(%[a])                                   \n\t"
        ".set pop                                                               \n\t"

        : [f0] "=&f" (f0), [f1] "=&f" (f1), [f2] "=&f" (f2),
          [f3] "=&f" (f3), [f4] "=&f" (f4), [f5] "=&f" (f5),
          [f6] "=&f" (f6), [f7] "=&f" (f7), [x0r] "=&f" (x0r),
          [x0i] "=&f" (x0i), [x1r] "=&f" (x1r), [x1i] "=&f" (x1i),
          [x2r] "=&f" (x2r), [x2i] "=&f" (x2i), [x3r] "=&f" (x3r),
          [x3i] "=&f" (x3i), [wk1r] "=&f" (wk1r), [wk1i] "=&f" (wk1i),
          [wk2r] "=&f" (wk2r), [wk2i] "=&f" (wk2i), [wk3r] "=&f" (wk3r),
          [wk3i] "=&f" (wk3i)
        : [a] "r" (a), [rdft_w] "r" (rdft_w), [rdft_wk3ri_first] "r" (rdft_wk3ri_first),
          [rdft_wk3ri_second] "r" (rdft_wk3ri_second)
        : "memory"
  );
}

void cftmdl_128_mips(float *a) {
  float wk1r, wk1i, wk2r, wk2i, wk3r, wk3i;
  float x0r, x0i, x1r, x1i, x2r, x2i, x3r, x3i;

  float f0, f1, f2, f3, f4, f5, f6, f7;

    __asm__ volatile(
        ".set push                                          \n\t"
        ".set noreorder                                     \n\t"
        "lwc1       %[f0],      0(%[a])                     \n\t"
        "lwc1       %[f1],      4(%[a])                     \n\t"
        "lwc1       %[f2],      32(%[a])                    \n\t"
        "lwc1       %[f3],      36(%[a])                    \n\t"
        "lwc1       %[f4],      64(%[a])                    \n\t"
        "lwc1       %[f5],      68(%[a])                    \n\t"
        "lwc1       %[f6],      96(%[a])                    \n\t"
        "lwc1       %[f7],      100(%[a])                   \n\t"

        "add.s      %[x0r],     %[f0],      %[f2]           \n\t"
        "add.s      %[x0i],     %[f1],      %[f3]           \n\t"
        "add.s      %[x2r],     %[f4],      %[f6]           \n\t"
        "add.s      %[x2i],     %[f5],      %[f7]           \n\t"
        "sub.s      %[x1r],     %[f0],      %[f2]           \n\t"
        "sub.s      %[x1i],     %[f1],      %[f3]           \n\t"
        "sub.s      %[x3r],     %[f4],      %[f6]           \n\t"
        "sub.s      %[x3i],     %[f5],      %[f7]           \n\t"

        "add.s      %[f0],      %[x0r],     %[x2r]          \n\t"
        "add.s      %[f1],      %[x0i],     %[x2i]          \n\t"
        "sub.s      %[f4],      %[x0r],     %[x2r]          \n\t"
        "sub.s      %[f5],      %[x0i],     %[x2i]          \n\t"
        "sub.s      %[f2],      %[x1r],     %[x3i]          \n\t"
        "add.s      %[f3],      %[x1i],     %[x3r]          \n\t"
        "add.s      %[f6],      %[x1r],     %[x3i]          \n\t"
        "sub.s      %[f7],      %[x1i],     %[x3r]          \n\t"

        "swc1       %[f0],      0(%[a])                     \n\t"
        "swc1       %[f1],      4(%[a])                     \n\t"
        "swc1       %[f2],      32(%[a])                    \n\t"
        "swc1       %[f3],      36(%[a])                    \n\t"
        "swc1       %[f4],      64(%[a])                    \n\t"
        "swc1       %[f5],      68(%[a])                    \n\t"
        "swc1       %[f6],      96(%[a])                    \n\t"
        "swc1       %[f7],      100(%[a])                   \n\t"

        "lwc1       %[f0],      8(%[a])                     \n\t"
        "lwc1       %[f1],      12(%[a])                    \n\t"
        "lwc1       %[f2],      40(%[a])                    \n\t"
        "lwc1       %[f3],      44(%[a])                    \n\t"
        "lwc1       %[f4],      72(%[a])                    \n\t"
        "lwc1       %[f5],      76(%[a])                    \n\t"
        "lwc1       %[f6],      104(%[a])                   \n\t"
        "lwc1       %[f7],      108(%[a])                   \n\t"

        "add.s      %[x0r],     %[f0],      %[f2]           \n\t"
        "add.s      %[x0i],     %[f1],      %[f3]           \n\t"
        "add.s      %[x2r],     %[f4],      %[f6]           \n\t"
        "add.s      %[x2i],     %[f5],      %[f7]           \n\t"
        "sub.s      %[x1r],     %[f0],      %[f2]           \n\t"
        "sub.s      %[x1i],     %[f1],      %[f3]           \n\t"
        "sub.s      %[x3r],     %[f4],      %[f6]           \n\t"
        "sub.s      %[x3i],     %[f5],      %[f7]           \n\t"

        "add.s      %[f0],      %[x0r],     %[x2r]          \n\t"
        "add.s      %[f1],      %[x0i],     %[x2i]          \n\t"
        "sub.s      %[f4],      %[x0r],     %[x2r]          \n\t"
        "sub.s      %[f5],      %[x0i],     %[x2i]          \n\t"
        "sub.s      %[f2],      %[x1r],     %[x3i]          \n\t"
        "add.s      %[f3],      %[x1i],     %[x3r]          \n\t"
        "add.s      %[f6],      %[x1r],     %[x3i]          \n\t"
        "sub.s      %[f7],      %[x1i],     %[x3r]          \n\t"

        "swc1       %[f0],      8(%[a])                     \n\t"
        "swc1       %[f1],      12(%[a])                    \n\t"
        "swc1       %[f2],      40(%[a])                    \n\t"
        "swc1       %[f3],      44(%[a])                    \n\t"
        "swc1       %[f4],      72(%[a])                    \n\t"
        "swc1       %[f5],      76(%[a])                    \n\t"
        "swc1       %[f6],      104(%[a])                   \n\t"
        "swc1       %[f7],      108(%[a])                   \n\t"

        "lwc1       %[f0],      16(%[a])                    \n\t"
        "lwc1       %[f1],      20(%[a])                    \n\t"
        "lwc1       %[f2],      48(%[a])                    \n\t"
        "lwc1       %[f3],      52(%[a])                    \n\t"
        "lwc1       %[f4],      80(%[a])                    \n\t"
        "lwc1       %[f5],      84(%[a])                    \n\t"
        "lwc1       %[f6],      112(%[a])                   \n\t"
        "lwc1       %[f7],      116(%[a])                   \n\t"

        "add.s      %[x0r],     %[f0],      %[f2]           \n\t"
        "add.s      %[x0i],     %[f1],      %[f3]           \n\t"
        "add.s      %[x2r],     %[f4],      %[f6]           \n\t"
        "add.s      %[x2i],     %[f5],      %[f7]           \n\t"
        "sub.s      %[x1r],     %[f0],      %[f2]           \n\t"
        "sub.s      %[x1i],     %[f1],      %[f3]           \n\t"
        "sub.s      %[x3r],     %[f4],      %[f6]           \n\t"
        "sub.s      %[x3i],     %[f5],      %[f7]           \n\t"

        "add.s      %[f0],      %[x0r],     %[x2r]          \n\t"
        "add.s      %[f1],      %[x0i],     %[x2i]          \n\t"
        "sub.s      %[f4],      %[x0r],     %[x2r]          \n\t"
        "sub.s      %[f5],      %[x0i],     %[x2i]          \n\t"
        "sub.s      %[f2],      %[x1r],     %[x3i]          \n\t"
        "add.s      %[f3],      %[x1i],     %[x3r]          \n\t"
        "add.s      %[f6],      %[x1r],     %[x3i]          \n\t"
        "sub.s      %[f7],      %[x1i],     %[x3r]          \n\t"

        "swc1       %[f0],      16(%[a])                    \n\t"
        "swc1       %[f1],      20(%[a])                    \n\t"
        "swc1       %[f2],      48(%[a])                    \n\t"
        "swc1       %[f3],      52(%[a])                    \n\t"
        "swc1       %[f4],      80(%[a])                    \n\t"
        "swc1       %[f5],      84(%[a])                    \n\t"
        "swc1       %[f6],      112(%[a])                   \n\t"
        "swc1       %[f7],      116(%[a])                   \n\t"

        "lwc1       %[f0],      24(%[a])                    \n\t"
        "lwc1       %[f1],      28(%[a])                    \n\t"
        "lwc1       %[f2],      56(%[a])                    \n\t"
        "lwc1       %[f3],      60(%[a])                    \n\t"
        "lwc1       %[f4],      88(%[a])                    \n\t"
        "lwc1       %[f5],      92(%[a])                    \n\t"
        "lwc1       %[f6],      120(%[a])                   \n\t"
        "lwc1       %[f7],      124(%[a])                   \n\t"

        "add.s      %[x0r],     %[f0],      %[f2]           \n\t"
        "add.s      %[x0i],     %[f1],      %[f3]           \n\t"
        "add.s      %[x2r],     %[f4],      %[f6]           \n\t"
        "add.s      %[x2i],     %[f5],      %[f7]           \n\t"
        "sub.s      %[x1r],     %[f0],      %[f2]           \n\t"
        "sub.s      %[x1i],     %[f1],      %[f3]           \n\t"
        "sub.s      %[x3r],     %[f4],      %[f6]           \n\t"
        "sub.s      %[x3i],     %[f5],      %[f7]           \n\t"

        "add.s      %[f0],      %[x0r],     %[x2r]          \n\t"
        "add.s      %[f1],      %[x0i],     %[x2i]          \n\t"
        "sub.s      %[f4],      %[x0r],     %[x2r]          \n\t"
        "sub.s      %[f5],      %[x0i],     %[x2i]          \n\t"
        "sub.s      %[f2],      %[x1r],     %[x3i]          \n\t"
        "add.s      %[f3],      %[x1i],     %[x3r]          \n\t"
        "add.s      %[f6],      %[x1r],     %[x3i]          \n\t"
        "sub.s      %[f7],      %[x1i],     %[x3r]          \n\t"

        "swc1       %[f0],      24(%[a])                    \n\t"
        "swc1       %[f1],      28(%[a])                    \n\t"
        "swc1       %[f2],      56(%[a])                    \n\t"
        "swc1       %[f3],      60(%[a])                    \n\t"
        "swc1       %[f4],      88(%[a])                    \n\t"
        "swc1       %[f5],      92(%[a])                    \n\t"
        "swc1       %[f6],      120(%[a])                   \n\t"
        "swc1       %[f7],      124(%[a])                   \n\t"
        ".set pop                                           \n\t"

        : [f0] "=&f" (f0), [f1] "=&f" (f1), [f2] "=&f" (f2),
          [f3] "=&f" (f3), [f4] "=&f" (f4), [f5] "=&f" (f5),
          [f6] "=&f" (f6), [f7] "=&f" (f7), [x0r] "=&f" (x0r),
          [x0i] "=&f" (x0i), [x1r] "=&f" (x1r), [x1i] "=&f" (x1i),
          [x2r] "=&f" (x2r), [x2i] "=&f" (x2i), [x3r] "=&f" (x3r),
          [x3i] "=&f" (x3i)
        : [a] "r" (a)
        : "memory"
    );
    wk2r = rdft_w[2];
    __asm__ volatile(
        ".set push                                      \n\t"
        ".set noreorder                                 \n\t"
        "lwc1   %[f0],      128(%[a])                   \n\t"
        "lwc1   %[f1],      132(%[a])                   \n\t"
        "lwc1   %[f2],      160(%[a])                   \n\t"
        "lwc1   %[f3],      164(%[a])                   \n\t"
        "lwc1   %[f4],      192(%[a])                   \n\t"
        "lwc1   %[f5],      196(%[a])                   \n\t"
        "lwc1   %[f6],      224(%[a])                   \n\t"
        "lwc1   %[f7],      228(%[a])                   \n\t"

        "sub.s  %[x1r],     %[f0],      %[f2]           \n\t"
        "sub.s  %[x1i],     %[f1],      %[f3]           \n\t"
        "sub.s  %[x3r],     %[f4],      %[f6]           \n\t"
        "sub.s  %[x3i],     %[f5],      %[f7]           \n\t"
        "add.s  %[x0r],     %[f0],      %[f2]           \n\t"
        "add.s  %[x0i],     %[f1],      %[f3]           \n\t"
        "add.s  %[x2r],     %[f4],      %[f6]           \n\t"
        "add.s  %[x2i],     %[f5],      %[f7]           \n\t"

        "sub.s  %[f0],      %[x1r],     %[x3i]          \n\t"
        "add.s  %[f1],      %[x1i],     %[x3r]          \n\t"
        "sub.s  %[f2],      %[x3r],     %[x1i]          \n\t"
        "add.s  %[f3],      %[x3i],     %[x1r]          \n\t"
        "add.s  %[f4],      %[x0r],     %[x2r]          \n\t"
        "add.s  %[f5],      %[x0i],     %[x2i]          \n\t"
        "sub.s  %[f6],      %[f0],      %[f1]           \n\t"
        "add.s  %[f0],      %[f0],      %[f1]           \n\t"
        "sub.s  %[f7],      %[f2],      %[f3]           \n\t"
        "add.s  %[f2],      %[f2],      %[f3]           \n\t"
        "sub.s  %[f1],      %[x2i],     %[x0i]          \n\t"
        "mul.s  %[f6],      %[f6],      %[wk2r]         \n\t"
        "mul.s  %[f0],      %[f0],      %[wk2r]         \n\t"
        "sub.s  %[f3],      %[x0r],     %[x2r]          \n\t"
        "mul.s  %[f7],      %[f7],      %[wk2r]         \n\t"
        "mul.s  %[f2],      %[f2],      %[wk2r]         \n\t"

        "swc1   %[f4],      128(%[a])                   \n\t"
        "swc1   %[f5],      132(%[a])                   \n\t"
        "swc1   %[f6],      160(%[a])                   \n\t"
        "swc1   %[f0],      164(%[a])                   \n\t"
        "swc1   %[f1],      192(%[a])                   \n\t"
        "swc1   %[f3],      196(%[a])                   \n\t"
        "swc1   %[f7],      224(%[a])                   \n\t"
        "swc1   %[f2],      228(%[a])                   \n\t"

        "lwc1   %[f0],      136(%[a])                   \n\t"
        "lwc1   %[f1],      140(%[a])                   \n\t"
        "lwc1   %[f2],      168(%[a])                   \n\t"
        "lwc1   %[f3],      172(%[a])                   \n\t"
        "lwc1   %[f4],      200(%[a])                   \n\t"
        "lwc1   %[f5],      204(%[a])                   \n\t"
        "lwc1   %[f6],      232(%[a])                   \n\t"
        "lwc1   %[f7],      236(%[a])                   \n\t"

        "sub.s  %[x1r],     %[f0],      %[f2]           \n\t"
        "sub.s  %[x1i],     %[f1],      %[f3]           \n\t"
        "sub.s  %[x3r],     %[f4],      %[f6]           \n\t"
        "sub.s  %[x3i],     %[f5],      %[f7]           \n\t"
        "add.s  %[x0r],     %[f0],      %[f2]           \n\t"
        "add.s  %[x0i],     %[f1],      %[f3]           \n\t"
        "add.s  %[x2r],     %[f4],      %[f6]           \n\t"
        "add.s  %[x2i],     %[f5],      %[f7]           \n\t"

        "sub.s  %[f0],      %[x1r],     %[x3i]          \n\t"
        "add.s  %[f1],      %[x1i],     %[x3r]          \n\t"
        "sub.s  %[f2],      %[x3r],     %[x1i]          \n\t"
        "add.s  %[f3],      %[x3i],     %[x1r]          \n\t"
        "add.s  %[f4],      %[x0r],     %[x2r]          \n\t"
        "add.s  %[f5],      %[x0i],     %[x2i]          \n\t"
        "sub.s  %[f6],      %[f0],      %[f1]           \n\t"
        "add.s  %[f0],      %[f0],      %[f1]           \n\t"
        "sub.s  %[f7],      %[f2],      %[f3]           \n\t"
        "add.s  %[f2],      %[f2],      %[f3]           \n\t"
        "sub.s  %[f1],      %[x2i],     %[x0i]          \n\t"
        "mul.s  %[f6],      %[f6],      %[wk2r]         \n\t"
        "mul.s  %[f0],      %[f0],      %[wk2r]         \n\t"
        "sub.s  %[f3],      %[x0r],     %[x2r]          \n\t"
        "mul.s  %[f7],      %[f7],      %[wk2r]         \n\t"
        "mul.s  %[f2],      %[f2],      %[wk2r]         \n\t"

        "swc1   %[f4],      136(%[a])                   \n\t"
        "swc1   %[f5],      140(%[a])                   \n\t"
        "swc1   %[f6],      168(%[a])                   \n\t"
        "swc1   %[f0],      172(%[a])                   \n\t"
        "swc1   %[f1],      200(%[a])                   \n\t"
        "swc1   %[f3],      204(%[a])                   \n\t"
        "swc1   %[f7],      232(%[a])                   \n\t"
        "swc1   %[f2],      236(%[a])                   \n\t"

        "lwc1   %[f0],      144(%[a])                   \n\t"
        "lwc1   %[f1],      148(%[a])                   \n\t"
        "lwc1   %[f2],      176(%[a])                   \n\t"
        "lwc1   %[f3],      180(%[a])                   \n\t"
        "lwc1   %[f4],      208(%[a])                   \n\t"
        "lwc1   %[f5],      212(%[a])                   \n\t"
        "lwc1   %[f6],      240(%[a])                   \n\t"
        "lwc1   %[f7],      244(%[a])                   \n\t"

        "sub.s  %[x1r],     %[f0],      %[f2]           \n\t"
        "sub.s  %[x1i],     %[f1],      %[f3]           \n\t"
        "sub.s  %[x3r],     %[f4],      %[f6]           \n\t"
        "sub.s  %[x3i],     %[f5],      %[f7]           \n\t"
        "add.s  %[x0r],     %[f0],      %[f2]           \n\t"
        "add.s  %[x0i],     %[f1],      %[f3]           \n\t"
        "add.s  %[x2r],     %[f4],      %[f6]           \n\t"
        "add.s  %[x2i],     %[f5],      %[f7]           \n\t"

        "sub.s  %[f0],      %[x1r],     %[x3i]          \n\t"
        "add.s  %[f1],      %[x1i],     %[x3r]          \n\t"
        "sub.s  %[f2],      %[x3r],     %[x1i]          \n\t"
        "add.s  %[f3],      %[x3i],     %[x1r]          \n\t"
        "add.s  %[f4],      %[x0r],     %[x2r]          \n\t"
        "add.s  %[f5],      %[x0i],     %[x2i]          \n\t"
        "sub.s  %[f6],      %[f0],      %[f1]           \n\t"
        "add.s  %[f0],      %[f0],      %[f1]           \n\t"
        "sub.s  %[f7],      %[f2],      %[f3]           \n\t"
        "add.s  %[f2],      %[f2],      %[f3]           \n\t"
        "sub.s  %[f1],      %[x2i],     %[x0i]          \n\t"
        "mul.s  %[f6],      %[f6],      %[wk2r]         \n\t"
        "mul.s  %[f0],      %[f0],      %[wk2r]         \n\t"
        "sub.s  %[f3],      %[x0r],     %[x2r]          \n\t"
        "mul.s  %[f7],      %[f7],      %[wk2r]         \n\t"
        "mul.s  %[f2],      %[f2],      %[wk2r]         \n\t"

        "swc1   %[f4],      144(%[a])                   \n\t"
        "swc1   %[f5],      148(%[a])                   \n\t"
        "swc1   %[f6],      176(%[a])                   \n\t"
        "swc1   %[f0],      180(%[a])                   \n\t"
        "swc1   %[f1],      208(%[a])                   \n\t"
        "swc1   %[f3],      212(%[a])                   \n\t"
        "swc1   %[f7],      240(%[a])                   \n\t"
        "swc1   %[f2],      244(%[a])                   \n\t"

        "lwc1   %[f0],      152(%[a])                   \n\t"
        "lwc1   %[f1],      156(%[a])                   \n\t"
        "lwc1   %[f2],      184(%[a])                   \n\t"
        "lwc1   %[f3],      188(%[a])                   \n\t"
        "lwc1   %[f4],      216(%[a])                   \n\t"
        "lwc1   %[f5],      220(%[a])                   \n\t"
        "lwc1   %[f6],      248(%[a])                   \n\t"
        "lwc1   %[f7],      252(%[a])                   \n\t"

        "sub.s  %[x1r],     %[f0],      %[f2]           \n\t"
        "sub.s  %[x1i],     %[f1],      %[f3]           \n\t"
        "sub.s  %[x3r],     %[f4],      %[f6]           \n\t"
        "sub.s  %[x3i],     %[f5],      %[f7]           \n\t"
        "add.s  %[x0r],     %[f0],      %[f2]           \n\t"
        "add.s  %[x0i],     %[f1],      %[f3]           \n\t"
        "add.s  %[x2r],     %[f4],      %[f6]           \n\t"
        "add.s  %[x2i],     %[f5],      %[f7]           \n\t"

        "sub.s  %[f0],      %[x1r],     %[x3i]          \n\t"
        "add.s  %[f1],      %[x1i],     %[x3r]          \n\t"
        "sub.s  %[f2],      %[x3r],     %[x1i]          \n\t"
        "add.s  %[f3],      %[x3i],     %[x1r]          \n\t"
        "add.s  %[f4],      %[x0r],     %[x2r]          \n\t"
        "add.s  %[f5],      %[x0i],     %[x2i]          \n\t"
        "sub.s  %[f6],      %[f0],      %[f1]           \n\t"
        "add.s  %[f0],      %[f0],      %[f1]           \n\t"
        "sub.s  %[f7],      %[f2],      %[f3]           \n\t"
        "add.s  %[f2],      %[f2],      %[f3]           \n\t"
        "sub.s  %[f1],      %[x2i],     %[x0i]          \n\t"
        "mul.s  %[f6],      %[f6],      %[wk2r]         \n\t"
        "mul.s  %[f0],      %[f0],      %[wk2r]         \n\t"
        "sub.s  %[f3],      %[x0r],     %[x2r]          \n\t"
        "mul.s  %[f7],      %[f7],      %[wk2r]         \n\t"
        "mul.s  %[f2],      %[f2],      %[wk2r]         \n\t"

        "swc1   %[f4],      152(%[a])                   \n\t"
        "swc1   %[f5],      156(%[a])                   \n\t"
        "swc1   %[f6],      184(%[a])                   \n\t"
        "swc1   %[f0],      188(%[a])                   \n\t"
        "swc1   %[f1],      216(%[a])                   \n\t"
        "swc1   %[f3],      220(%[a])                   \n\t"
        "swc1   %[f7],      248(%[a])                   \n\t"
        "swc1   %[f2],      252(%[a])                   \n\t"
        ".set pop                                       \n\t"

        : [f0] "=&f" (f0), [f1] "=&f" (f1), [f2] "=&f" (f2),
          [f3] "=&f" (f3), [f4] "=&f" (f4), [f5] "=&f" (f5),
          [f6] "=&f" (f6), [f7] "=&f" (f7), [x0r] "=&f" (x0r),
          [x0i] "=&f" (x0i), [x1r] "=&f" (x1r), [x1i] "=&f" (x1i),
          [x2r] "=&f" (x2r), [x2i] "=&f" (x2i), [x3r] "=&f" (x3r),
          [x3i] "=&f" (x3i)
        : [a] "r" (a), [wk2r] "f" (wk2r)
        : "memory"
    );
    wk2i = rdft_w[3];
    wk1r = rdft_w[4];
    wk1i = rdft_w[5];
    wk3r = rdft_wk3ri_first[2];
    wk3i = rdft_wk3ri_first[3];

    __asm__ volatile(
        ".set push                                                              \n\t"
        ".set noreorder                                                         \n\t"
        "lwc1       %[f0],          256(%[a])                                   \n\t"
        "lwc1       %[f1],          260(%[a])                                   \n\t"
        "lwc1       %[f2],          288(%[a])                                   \n\t"
        "lwc1       %[f3],          292(%[a])                                   \n\t"
        "lwc1       %[f4],          320(%[a])                                   \n\t"
        "lwc1       %[f5],          324(%[a])                                   \n\t"
        "lwc1       %[f6],          352(%[a])                                   \n\t"
        "lwc1       %[f7],          356(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "mul.s      %[x3r],         %[wk2r],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f1],          %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2i],        %[f0]                       \n\t"
        "sub.s      %[x3r],         %[x3r],         %[f1]                       \n\t"
        "add.s      %[x3i],         %[x3i],         %[f0]                       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[f2],          %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[f5],          %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[f2]                       \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[f5]                       \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmsub.s    %[x3r],         %[x3r],         %[wk2i],        %[f1]       \n\t"
        "madd.s     %[x3i],         %[x3i],         %[wk2i],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          256(%[a])                                   \n\t"
        "swc1       %[f1],          260(%[a])                                   \n\t"
        "swc1       %[x1r],         288(%[a])                                   \n\t"
        "swc1       %[x1i],         292(%[a])                                   \n\t"
        "swc1       %[x3r],         320(%[a])                                   \n\t"
        "swc1       %[x3i],         324(%[a])                                   \n\t"
        "swc1       %[f6],          352(%[a])                                   \n\t"
        "swc1       %[f7],          356(%[a])                                   \n\t"

        "lwc1       %[f0],          264(%[a])                                   \n\t"
        "lwc1       %[f1],          268(%[a])                                   \n\t"
        "lwc1       %[f2],          296(%[a])                                   \n\t"
        "lwc1       %[f3],          300(%[a])                                   \n\t"
        "lwc1       %[f4],          328(%[a])                                   \n\t"
        "lwc1       %[f5],          332(%[a])                                   \n\t"
        "lwc1       %[f6],          360(%[a])                                   \n\t"
        "lwc1       %[f7],          364(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "mul.s      %[x3r],         %[wk2r],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f1],          %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2i],        %[f0]                       \n\t"
        "sub.s      %[x3r],         %[x3r],         %[f1]                       \n\t"
        "add.s      %[x3i],         %[x3i],         %[f0]                       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[f2],          %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[f5],          %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[f2]                       \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[f5]                       \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmsub.s    %[x3r],         %[x3r],         %[wk2i],        %[f1]       \n\t"
        "madd.s     %[x3i],         %[x3i],         %[wk2i],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          264(%[a])                                   \n\t"
        "swc1       %[f1],          268(%[a])                                   \n\t"
        "swc1       %[x1r],         296(%[a])                                   \n\t"
        "swc1       %[x1i],         300(%[a])                                   \n\t"
        "swc1       %[x3r],         328(%[a])                                   \n\t"
        "swc1       %[x3i],         332(%[a])                                   \n\t"
        "swc1       %[f6],          360(%[a])                                   \n\t"
        "swc1       %[f7],          364(%[a])                                   \n\t"

        "lwc1       %[f0],          272(%[a])                                   \n\t"
        "lwc1       %[f1],          276(%[a])                                   \n\t"
        "lwc1       %[f2],          304(%[a])                                   \n\t"
        "lwc1       %[f3],          308(%[a])                                   \n\t"
        "lwc1       %[f4],          336(%[a])                                   \n\t"
        "lwc1       %[f5],          340(%[a])                                   \n\t"
        "lwc1       %[f6],          368(%[a])                                   \n\t"
        "lwc1       %[f7],          372(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "mul.s      %[x3r],         %[wk2r],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f1],          %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2i],        %[f0]                       \n\t"
        "sub.s      %[x3r],         %[x3r],         %[f1]                       \n\t"
        "add.s      %[x3i],         %[x3i],         %[f0]                       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[f2],          %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[f5],          %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[f2]                       \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[f5]                       \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmsub.s    %[x3r],         %[x3r],         %[wk2i],        %[f1]       \n\t"
        "madd.s     %[x3i],         %[x3i],         %[wk2i],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          272(%[a])                                   \n\t"
        "swc1       %[f1],          276(%[a])                                   \n\t"
        "swc1       %[x1r],         304(%[a])                                   \n\t"
        "swc1       %[x1i],         308(%[a])                                   \n\t"
        "swc1       %[x3r],         336(%[a])                                   \n\t"
        "swc1       %[x3i],         340(%[a])                                   \n\t"
        "swc1       %[f6],          368(%[a])                                   \n\t"
        "swc1       %[f7],          372(%[a])                                   \n\t"

        "lwc1       %[f0],          280(%[a])                                   \n\t"
        "lwc1       %[f1],          284(%[a])                                   \n\t"
        "lwc1       %[f2],          312(%[a])                                   \n\t"
        "lwc1       %[f3],          316(%[a])                                   \n\t"
        "lwc1       %[f4],          344(%[a])                                   \n\t"
        "lwc1       %[f5],          348(%[a])                                   \n\t"
        "lwc1       %[f6],          376(%[a])                                   \n\t"
        "lwc1       %[f7],          380(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "mul.s      %[x3r],         %[wk2r],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f1],          %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2i],        %[f0]                       \n\t"
        "sub.s      %[x3r],         %[x3r],         %[f1]                       \n\t"
        "add.s      %[x3i],         %[x3i],         %[f0]                       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[f2],          %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[f5],          %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[f2]                       \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[f5]                       \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmsub.s    %[x3r],         %[x3r],         %[wk2i],        %[f1]       \n\t"
        "madd.s     %[x3i],         %[x3i],         %[wk2i],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          280(%[a])                                   \n\t"
        "swc1       %[f1],          284(%[a])                                   \n\t"
        "swc1       %[x1r],         312(%[a])                                   \n\t"
        "swc1       %[x1i],         316(%[a])                                   \n\t"
        "swc1       %[x3r],         344(%[a])                                   \n\t"
        "swc1       %[x3i],         348(%[a])                                   \n\t"
        "swc1       %[f6],          376(%[a])                                   \n\t"
        "swc1       %[f7],          380(%[a])                                   \n\t"
        ".set pop                                                               \n\t"
        : [f0] "=&f" (f0), [f1] "=&f" (f1), [f2] "=&f" (f2),
          [f3] "=&f" (f3), [f4] "=&f" (f4), [f5] "=&f" (f5),
          [f6] "=&f" (f6), [f7] "=&f" (f7), [x0r] "=&f" (x0r),
          [x0i] "=&f" (x0i), [x1r] "=&f" (x1r), [x1i] "=&f" (x1i),
          [x2r] "=&f" (x2r), [x2i] "=&f" (x2i), [x3r] "=&f" (x3r),
          [x3i] "=&f" (x3i)
        : [a] "r" (a),  [wk1r] "f" (wk1r), [wk1i] "f" (wk1i),
          [wk2r] "f" (wk2r), [wk2i] "f" (wk2i), [wk3r] "f" (wk3r),
          [wk3i] "f" (wk3i)
        : "memory"
    );

    wk1r = rdft_w[6];
    wk1i = rdft_w[7];
    wk3r = rdft_wk3ri_second[2];
    wk3i = rdft_wk3ri_second[3];

    __asm__ volatile(
        ".set push                                                              \n\t"
        ".set noreorder                                                         \n\t"

        "lwc1       %[f0],          384(%[a])                                   \n\t"
        "lwc1       %[f1],          388(%[a])                                   \n\t"
        "lwc1       %[f2],          416(%[a])                                   \n\t"
        "lwc1       %[f3],          420(%[a])                                   \n\t"
        "lwc1       %[f4],          448(%[a])                                   \n\t"
        "lwc1       %[f5],          452(%[a])                                   \n\t"
        "lwc1       %[f6],          480(%[a])                                   \n\t"
        "lwc1       %[f7],          484(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "mul.s      %[x3r],         %[wk2i],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f1],          %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2r],        %[f0]                       \n\t"
        "add.s      %[x3r],         %[x3r],         %[f1]                       \n\t"
        "neg.s      %[x3r],         %[x3r]                                      \n\t"
        "sub.s      %[x3i],         %[f0],          %[x3i]                      \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[f2],          %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[f5],          %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[f2]                       \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[f5]                       \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmadd.s    %[x3r],         %[x3r],         %[wk2r],        %[f1]       \n\t"
        "msub.s     %[x3i],         %[x3i],         %[wk2r],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          384(%[a])                                   \n\t"
        "swc1       %[f1],          388(%[a])                                   \n\t"
        "swc1       %[x1r],         416(%[a])                                   \n\t"
        "swc1       %[x1i],         420(%[a])                                   \n\t"
        "swc1       %[x3r],         448(%[a])                                   \n\t"
        "swc1       %[x3i],         452(%[a])                                   \n\t"
        "swc1       %[f6],          480(%[a])                                   \n\t"
        "swc1       %[f7],          484(%[a])                                   \n\t"

        "lwc1       %[f0],          392(%[a])                                   \n\t"
        "lwc1       %[f1],          396(%[a])                                   \n\t"
        "lwc1       %[f2],          424(%[a])                                   \n\t"
        "lwc1       %[f3],          428(%[a])                                   \n\t"
        "lwc1       %[f4],          456(%[a])                                   \n\t"
        "lwc1       %[f5],          460(%[a])                                   \n\t"
        "lwc1       %[f6],          488(%[a])                                   \n\t"
        "lwc1       %[f7],          492(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "mul.s      %[x3r],         %[wk2i],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f1],          %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2r],        %[f0]                       \n\t"
        "add.s      %[x3r],         %[x3r],         %[f1]                       \n\t"
        "neg.s      %[x3r],         %[x3r]                                      \n\t"
        "sub.s      %[x3i],         %[f0],          %[x3i]                      \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[f2],          %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[f5],          %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[f2]                       \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[f5]                       \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmadd.s    %[x3r],         %[x3r],         %[wk2r],        %[f1]       \n\t"
        "msub.s     %[x3i],         %[x3i],         %[wk2r],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          392(%[a])                                   \n\t"
        "swc1       %[f1],          396(%[a])                                   \n\t"
        "swc1       %[x1r],         424(%[a])                                   \n\t"
        "swc1       %[x1i],         428(%[a])                                   \n\t"
        "swc1       %[x3r],         456(%[a])                                   \n\t"
        "swc1       %[x3i],         460(%[a])                                   \n\t"
        "swc1       %[f6],          488(%[a])                                   \n\t"
        "swc1       %[f7],          492(%[a])                                   \n\t"

        "lwc1       %[f0],          400(%[a])                                   \n\t"
        "lwc1       %[f1],          404(%[a])                                   \n\t"
        "lwc1       %[f2],          432(%[a])                                   \n\t"
        "lwc1       %[f3],          436(%[a])                                   \n\t"
        "lwc1       %[f4],          464(%[a])                                   \n\t"
        "lwc1       %[f5],          468(%[a])                                   \n\t"
        "lwc1       %[f6],          496(%[a])                                   \n\t"
        "lwc1       %[f7],          500(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "mul.s      %[x3r],         %[wk2i],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f1],          %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2r],        %[f0]                       \n\t"
        "add.s      %[x3r],         %[x3r],         %[f1]                       \n\t"
        "neg.s      %[x3r],         %[x3r]                                      \n\t"
        "sub.s      %[x3i],         %[f0],          %[x3i]                      \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[f2],          %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[f5],          %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[f2]                       \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[f5]                       \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmadd.s    %[x3r],         %[x3r],         %[wk2r],        %[f1]       \n\t"
        "msub.s     %[x3i],         %[x3i],         %[wk2r],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          400(%[a])                                   \n\t"
        "swc1       %[f1],          404(%[a])                                   \n\t"
        "swc1       %[x1r],         432(%[a])                                   \n\t"
        "swc1       %[x1i],         436(%[a])                                   \n\t"
        "swc1       %[x3r],         464(%[a])                                   \n\t"
        "swc1       %[x3i],         468(%[a])                                   \n\t"
        "swc1       %[f6],          496(%[a])                                   \n\t"
        "swc1       %[f7],          500(%[a])                                   \n\t"

        "lwc1       %[f0],          408(%[a])                                   \n\t"
        "lwc1       %[f1],          412(%[a])                                   \n\t"
        "lwc1       %[f2],          440(%[a])                                   \n\t"
        "lwc1       %[f3],          444(%[a])                                   \n\t"
        "lwc1       %[f4],          472(%[a])                                   \n\t"
        "lwc1       %[f5],          476(%[a])                                   \n\t"
        "lwc1       %[f6],          504(%[a])                                   \n\t"
        "lwc1       %[f7],          508(%[a])                                   \n\t"

        "add.s      %[x0r],         %[f0],          %[f2]                       \n\t"
        "add.s      %[x2r],         %[f4],          %[f6]                       \n\t"
        "add.s      %[x0i],         %[f1],          %[f3]                       \n\t"
        "add.s      %[x2i],         %[f5],          %[f7]                       \n\t"
        "sub.s      %[x1r],         %[f0],          %[f2]                       \n\t"
        "sub.s      %[x1i],         %[f1],          %[f3]                       \n\t"
        "sub.s      %[x3r],         %[f4],          %[f6]                       \n\t"
        "sub.s      %[x3i],         %[f5],          %[f7]                       \n\t"

        "sub.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "sub.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "add.s      %[f2],          %[x1i],         %[x3r]                      \n\t"
        "sub.s      %[f3],          %[x1r],         %[x3i]                      \n\t"
        "add.s      %[f4],          %[x1r],         %[x3i]                      \n\t"
        "sub.s      %[f5],          %[x1i],         %[x3r]                      \n\t"

        "mul.s      %[x3r],         %[wk2i],        %[f0]                       \n\t"
        "mul.s      %[x3i],         %[wk2i],        %[f1]                       \n\t"
        "mul.s      %[x1r],         %[wk1r],        %[f3]                       \n\t"
        "mul.s      %[x1i],         %[wk1r],        %[f2]                       \n\t"
        "mul.s      %[f6],          %[wk3r],        %[f4]                       \n\t"
        "mul.s      %[f7],          %[wk3r],        %[f5]                       \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f1],          %[wk2r],        %[f1]                       \n\t"
        "mul.s      %[f0],          %[wk2r],        %[f0]                       \n\t"
        "add.s      %[x3r],         %[x3r],         %[f1]                       \n\t"
        "neg.s      %[x3r],         %[x3r]                                      \n\t"
        "sub.s      %[x3i],         %[f0],          %[x3i]                      \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "mul.s      %[f2],          %[wk1i],        %[f2]                       \n\t"
        "mul.s      %[f3],          %[wk1i],        %[f3]                       \n\t"
        "mul.s      %[f5],          %[wk3i],        %[f5]                       \n\t"
        "mul.s      %[f4],          %[wk3i],        %[f4]                       \n\t"
        "sub.s      %[x1r],         %[x1r],         %[f2]                       \n\t"
        "add.s      %[x1i],         %[x1i],         %[f3]                       \n\t"
        "sub.s      %[f6],          %[f6],          %[f5]                       \n\t"
        "add.s      %[f7],          %[f7],          %[f4]                       \n\t"
#else
        "nmadd.s    %[x3r],         %[x3r],         %[wk2r],        %[f1]       \n\t"
        "msub.s     %[x3i],         %[x3i],         %[wk2r],        %[f0]       \n\t"
        "add.s      %[f0],          %[x0r],         %[x2r]                      \n\t"
        "add.s      %[f1],          %[x0i],         %[x2i]                      \n\t"
        "nmsub.s    %[x1r],         %[x1r],         %[wk1i],        %[f2]       \n\t"
        "madd.s     %[x1i],         %[x1i],         %[wk1i],        %[f3]       \n\t"
        "nmsub.s    %[f6],          %[f6],          %[wk3i],        %[f5]       \n\t"
        "madd.s     %[f7],          %[f7],          %[wk3i],        %[f4]       \n\t"
#endif
        "swc1       %[f0],          408(%[a])                                   \n\t"
        "swc1       %[f1],          412(%[a])                                   \n\t"
        "swc1       %[x1r],         440(%[a])                                   \n\t"
        "swc1       %[x1i],         444(%[a])                                   \n\t"
        "swc1       %[x3r],         472(%[a])                                   \n\t"
        "swc1       %[x3i],         476(%[a])                                   \n\t"
        "swc1       %[f6],          504(%[a])                                   \n\t"
        "swc1       %[f7],          508(%[a])                                   \n\t"
        ".set pop                                                               \n\t"
        : [f0] "=&f" (f0), [f1] "=&f" (f1), [f2] "=&f" (f2),
          [f3] "=&f" (f3), [f4] "=&f" (f4), [f5] "=&f" (f5),
          [f6] "=&f" (f6), [f7] "=&f" (f7), [x0r] "=&f" (x0r),
          [x0i] "=&f" (x0i), [x1r] "=&f" (x1r), [x1i] "=&f" (x1i),
          [x2r] "=&f" (x2r), [x2i] "=&f" (x2i), [x3r] "=&f" (x3r),
          [x3i] "=&f" (x3i)
        : [a] "r" (a), [wk1r] "f" (wk1r), [wk1i] "f" (wk1i),
          [wk2r] "f" (wk2r), [wk2i] "f" (wk2i), [wk3r] "f" (wk3r),
          [wk3i] "f" (wk3i)
        : "memory"
    );
}

void rftfsub_128_mips(float *a) {
  const float *c = rdft_w + 32;
  float wkr, wki, xr, xi, yr, yi;

  const float temp = 0.5f;
  float aj20=0, aj21=0, ak20=0, ak21=0, ck1=0;
  float *a1 = a;
  float *a2 = a;
  float *c1 = rdft_w + 33;
  float *c2 = c1 + 30;

  __asm__ volatile(
      ".set      push                                             \n\t"
      ".set      noreorder                                        \n\t"

      "lwc1      %[aj20],     8(%[a2])                            \n\t"
      "lwc1      %[ak20],     504(%[a1])                          \n\t"
      "lwc1      %[ck1],      0(%[c2])                            \n\t"
      "lwc1      %[aj21],     12(%[a2])                           \n\t"
      "lwc1      %[ak21],     508(%[a1])                          \n\t"
      "sub.s     %[wkr],      %[temp],      %[ck1]                \n\t"
      "sub.s     %[xr],       %[aj20],      %[ak20]               \n\t"
      "add.s     %[xi],       %[aj21],      %[ak21]               \n\t"
      "lwc1      %[wki],      0(%[c1])                            \n\t"
      "addiu     %[c2],       %[c2],-4                            \n\t"
      "mul.s     %[yr],       %[wkr],       %[xr]                 \n\t"
      "mul.s     %[yi],       %[wkr],       %[xi]                 \n\t"
#if !defined(MIPS32_R2_LE)
      "mul.s     %[xi],       %[wki],       %[xi]                 \n\t"
      "mul.s     %[xr],       %[wki],       %[xr]                 \n\t"
      "sub.s     %[yr],       %[yr],        %[xi]                 \n\t"
      "add.s     %[yi],       %[yi],        %[xr]                 \n\t"
#else
      "nmsub.s   %[yr],       %[yr],        %[wki],     %[xi]     \n\t"
      "madd.s    %[yi],       %[yi],        %[wki],     %[xr]     \n\t"
#endif
      "addiu     %[c1],       %[c1],        4                     \n\t"
      "sub.s     %[aj20],     %[aj20],      %[yr]                 \n\t"
      "sub.s     %[aj21],     %[aj21],      %[yi]                 \n\t"
      "add.s     %[ak20],     %[ak20],      %[yr]                 \n\t"
      "sub.s     %[ak21],     %[ak21],      %[yi]                 \n\t"
      "addiu     %[a2],       %[a2],        8                     \n\t"
      "swc1      %[aj20],     0(%[a2])                            \n\t"
      "swc1      %[aj21],     4(%[a2])                            \n\t"
      "swc1      %[ak20],     504(%[a1])                          \n\t"
      "swc1      %[ak21],     508(%[a1])                          \n\t"
      "addiu     %[a1],       %[a1],        -8                    \n\t"

//15x2 passes:
  "1:                                                             \n\t"
      "lwc1      %[ck1],      0(%[c2])                            \n\t"
      "lwc1      %[aj20],     8(%[a2])                            \n\t"
      "lwc1      %[aj21],     12(%[a2])                           \n\t"
      "lwc1      %[ak20],     504(%[a1])                          \n\t"
      "lwc1      %[ak21],     508(%[a1])                          \n\t"

      "lwc1      $f0,         -4(%[c2])                           \n\t"
      "lwc1      $f2,         16(%[a2])                           \n\t"
      "lwc1      $f3,         20(%[a2])                           \n\t"
      "lwc1      $f8,         496(%[a1])                          \n\t"
      "lwc1      $f7,         500(%[a1])                          \n\t"

      "sub.s     %[wkr],      %[temp],      %[ck1]                \n\t"
      "sub.s     %[xr],       %[aj20],      %[ak20]               \n\t"
      "add.s     %[xi],       %[aj21],      %[ak21]               \n\t"
      "lwc1      %[wki],      0(%[c1])                            \n\t"

      "sub.s     $f0,         %[temp],      $f0                   \n\t"
      "sub.s     $f6,         $f2,          $f8                   \n\t"
      "add.s     $f4,         $f3,          $f7                   \n\t"
      "lwc1      $f5,         4(%[c1])                            \n\t"

      "mul.s     %[yr],       %[wkr],       %[xr]                 \n\t"
      "mul.s     %[yi],       %[wkr],       %[xi]                 \n\t"
      "mul.s     $f1,         $f0,          $f6                   \n\t"
      "mul.s     $f0,         $f0,          $f4                   \n\t"
      "addiu     %[c2],       %[c2],        -8                    \n\t"
#if !defined(MIPS32_R2_LE)
      "mul.s     %[xi],       %[wki],       %[xi]                 \n\t"
      "mul.s     %[xr],       %[wki],       %[xr]                 \n\t"
      "mul.s     $f4,         $f5,          $f4                   \n\t"
      "mul.s     $f6,         $f5,          $f6                   \n\t"
      "sub.s     %[yr],       %[yr],        %[xi]                 \n\t"
      "add.s     %[yi],       %[yi],        %[xr]                 \n\t"
      "sub.s     $f1,         $f1,          $f4                   \n\t"
      "add.s     $f0,         $f0,          $f6                   \n\t"
#else
      "nmsub.s   %[yr],       %[yr],        %[wki],     %[xi]     \n\t"
      "madd.s    %[yi],       %[yi],        %[wki],     %[xr]     \n\t"
      "nmsub.s   $f1,         $f1,          $f5,        $f4       \n\t"
      "madd.s    $f0,         $f0,          $f5,        $f6       \n\t"
#endif
      "addiu     %[c1],       %[c1],        8                     \n\t"

      "sub.s     %[aj20],     %[aj20],      %[yr]                 \n\t"
      "sub.s     %[aj21],     %[aj21],      %[yi]                 \n\t"
      "add.s     %[ak20],     %[ak20],      %[yr]                 \n\t"
      "sub.s     %[ak21],     %[ak21],      %[yi]                 \n\t"

      "sub.s     $f2,         $f2,          $f1                   \n\t"
      "sub.s     $f3,         $f3,          $f0                   \n\t"
      "add.s     $f1,         $f8,          $f1                   \n\t"
      "sub.s     $f0,         $f7,          $f0                   \n\t"

      "swc1      %[aj20],     8(%[a2])                            \n\t"
      "swc1      %[aj21],     12(%[a2])                           \n\t"
      "swc1      %[ak20],     504(%[a1])                          \n\t"
      "swc1      %[ak21],     508(%[a1])                          \n\t"

      "swc1      $f2,         16(%[a2])                           \n\t"
      "swc1      $f3,         20(%[a2])                           \n\t"
      "swc1      $f1,         496(%[a1])                          \n\t"
      "swc1      $f0,         500(%[a1])                          \n\t"

      "addiu     %[a2],       %[a2],        16                    \n\t"
      "bne       %[c2],       %[c],         1b                    \n\t"
      " addiu    %[a1],       %[a1],        -16                   \n\t"

      ".set      pop                                              \n\t"
      : [a]"+r"(a), [c]"+r"(c),[a1]"+r"(a1), [a2]"+r"(a2), [c1]"+r"(c1), [c2]"+r"(c2),
        [wkr]"=&f"(wkr), [wki]"=&f"(wki), [xr]"=&f"(xr), [xi]"=&f"(xi), [yr]"=&f"(yr), [yi]"=&f"(yi),
        [aj20]"=&f"(aj20), [aj21]"=&f"(aj21), [ak20]"=&f"(ak20), [ak21]"=&f"(ak21), [ck1]"=&f"(ck1)
      : [temp]"f"(temp)
      : "memory",
        "$f0","$f1","$f2","$f3","$f4","$f5","$f6","$f7","$f8"
  );
}

void rftbsub_128_mips(float *a) {
  const float *c = rdft_w + 32;
  float wkr, wki, xr, xi, yr, yi;

  a[1] = -a[1];
  a[65] = -a[65];

  const float temp = 0.5f;
  float aj20=0, aj21=0, ak20=0, ak21=0, ck1=0;
  float *a1 = a;
  float *a2 = a;
  float *c1 = rdft_w + 33;
  float *c2 = c1 + 30;

  __asm__ volatile(
      ".set      push                                           \n\t"
      ".set      noreorder                                      \n\t"

      "lwc1      %[aj20],     8(%[a2])                          \n\t"
      "lwc1      %[ak20],     504(%[a1])                        \n\t"
      "lwc1      %[ck1],      0(%[c2])                          \n\t"
      "lwc1      %[aj21],     12(%[a2])                         \n\t"
      "lwc1      %[ak21],     508(%[a1])                        \n\t"
      "sub.s     %[wkr],      %[temp],    %[ck1]                \n\t"
      "sub.s     %[xr],       %[aj20],    %[ak20]               \n\t"
      "add.s     %[xi],       %[aj21],    %[ak21]               \n\t"
      "lwc1      %[wki],      0(%[c1])                          \n\t"
      "addiu     %[c2],       %[c2],       -4                   \n\t"
      "mul.s     %[yr],       %[wkr],     %[xr]                 \n\t"
      "mul.s     %[yi],       %[wkr],     %[xi]                 \n\t"
#if !defined(MIPS32_R2_LE)
      "mul.s     %[xi],       %[wki],     %[xi]                 \n\t"
      "mul.s     %[xr],       %[wki],     %[xr]                 \n\t"
      "add.s     %[yr],       %[yr],      %[xi]                 \n\t"
      "sub.s     %[yi],       %[yi],      %[xr]                 \n\t"
#else
      "madd.s    %[yr],       %[yr],      %[wki],     %[xi]     \n\t"
      "nmsub.s   %[yi],       %[yi],      %[wki],     %[xr]     \n\t"
#endif
      "addiu     %[c1],       %[c1],4                           \n\t"
      "sub.s     %[aj20],     %[aj20],    %[yr]                 \n\t"
      "sub.s     %[aj21],     %[yi],      %[aj21]               \n\t"
      "add.s     %[ak20],     %[ak20],    %[yr]                 \n\t"
      "sub.s     %[ak21],     %[yi],      %[ak21]               \n\t"
      "addiu     %[a2],       %[a2],      8                     \n\t"
      "swc1      %[aj20],     0(%[a2])                          \n\t"
      "swc1      %[aj21],     4(%[a2])                          \n\t"
      "swc1      %[ak20],     504(%[a1])                        \n\t"
      "swc1      %[ak21],     508(%[a1])                        \n\t"
      "addiu     %[a1],       %[a1],      -8                    \n\t"

//15x2 passes:
  "1:                                                           \n\t"
      "lwc1      %[ck1],      0(%[c2])                          \n\t"
      "lwc1      %[aj20],     8(%[a2])                          \n\t"
      "lwc1      %[aj21],     12(%[a2])                         \n\t"
      "lwc1      %[ak20],     504(%[a1])                        \n\t"
      "lwc1      %[ak21],     508(%[a1])                        \n\t"

      "lwc1      $f0,         -4(%[c2])                         \n\t"
      "lwc1      $f2,         16(%[a2])                         \n\t"
      "lwc1      $f3,         20(%[a2])                         \n\t"
      "lwc1      $f8,         496(%[a1])                        \n\t"
      "lwc1      $f7,         500(%[a1])                        \n\t"

      "sub.s     %[wkr],      %[temp],    %[ck1]                \n\t"
      "sub.s     %[xr],       %[aj20],    %[ak20]               \n\t"
      "add.s     %[xi],       %[aj21],    %[ak21]               \n\t"
      "lwc1      %[wki],      0(%[c1])                          \n\t"

      "sub.s     $f0,         %[temp],    $f0                   \n\t"
      "sub.s     $f6,         $f2,        $f8                   \n\t"
      "add.s     $f4,         $f3,        $f7                   \n\t"
      "lwc1      $f5,         4(%[c1])                          \n\t"

      "mul.s     %[yr],       %[wkr],     %[xr]                 \n\t"
      "mul.s     %[yi],       %[wkr],     %[xi]                 \n\t"
      "mul.s     $f1,         $f0,        $f6                   \n\t"
      "mul.s     $f0,         $f0,        $f4                   \n\t"

      "addiu     %[c2],       %[c2],      -8                    \n\t"
#if !defined(MIPS32_R2_LE)
      "mul.s     %[xi],       %[wki],     %[xi]                 \n\t"
      "mul.s     %[xr],       %[wki],     %[xr]                 \n\t"
      "mul.s     $f4,         $f5,        $f4                   \n\t"
      "mul.s     $f6,         $f5,        $f6                   \n\t"
      "add.s     %[yr],       %[yr],      %[xi]                 \n\t"
      "sub.s     %[yi],       %[yi],      %[xr]                 \n\t"
      "add.s     $f1,         $f1,        $f4                   \n\t"
      "sub.s     $f0,         $f0,        $f6                   \n\t"
#else
      "madd.s    %[yr],       %[yr],      %[wki],     %[xi]     \n\t"
      "nmsub.s   %[yi],       %[yi],      %[wki],     %[xr]     \n\t"
      "madd.s    $f1,         $f1,        $f5,        $f4       \n\t"
      "nmsub.s   $f0,         $f0,        $f5,        $f6       \n\t"
#endif
      "addiu     %[c1],       %[c1],      8                     \n\t"

      "sub.s     %[aj20],     %[aj20],    %[yr]                 \n\t"
      "sub.s     %[aj21],     %[yi],      %[aj21]               \n\t"
      "add.s     %[ak20],     %[ak20],    %[yr]                 \n\t"
      "sub.s     %[ak21],     %[yi],      %[ak21]               \n\t"

      "sub.s     $f2,         $f2,        $f1                   \n\t"
      "sub.s     $f3,         $f0,        $f3                   \n\t"
      "add.s     $f1,         $f8,        $f1                   \n\t"
      "sub.s     $f0,         $f0,        $f7                   \n\t"

      "swc1      %[aj20],     8(%[a2])                          \n\t"
      "swc1      %[aj21],     12(%[a2])                         \n\t"
      "swc1      %[ak20],     504(%[a1])                        \n\t"
      "swc1      %[ak21],     508(%[a1])                        \n\t"

      "swc1      $f2,         16(%[a2])                         \n\t"
      "swc1      $f3,         20(%[a2])                         \n\t"
      "swc1      $f1,         496(%[a1])                        \n\t"
      "swc1      $f0,         500(%[a1])                        \n\t"

      "addiu     %[a2],       %[a2],      16                    \n\t"
      "bne       %[c2],       %[c],       1b                    \n\t"
      " addiu    %[a1],       %[a1],      -16                   \n\t"

      ".set      pop                                            \n\t"
      : [a]"+r"(a), [c]"+r"(c),[a1]"+r"(a1), [a2]"+r"(a2), [c1]"+r"(c1), [c2]"+r"(c2),
        [wkr]"=&f"(wkr), [wki]"=&f"(wki), [xr]"=&f"(xr), [xi]"=&f"(xi), [yr]"=&f"(yr), [yi]"=&f"(yi),
        [aj20]"=&f"(aj20), [aj21]"=&f"(aj21), [ak20]"=&f"(ak20), [ak21]"=&f"(ak21), [ck1]"=&f"(ck1)
      : [temp]"f"(temp)
      : "memory",
        "$f0","$f1","$f2","$f3","$f4","$f5","$f6","$f7","$f8"
  );
}
#endif //#if defined(MIPS32_LE) & defined(MIPS_FPU_LE)

void aec_rdft_init_mips(void) {
#if defined(MIPS32_LE) & defined(MIPS_FPU_LE)
  cft1st_128 = cft1st_128_mips;
  cftmdl_128 = cftmdl_128_mips;
  rftfsub_128 = rftfsub_128_mips;
  rftbsub_128 = rftbsub_128_mips;
#endif //#if defined(MIPS32_LE) & defined(MIPS_FPU_LE)
}
