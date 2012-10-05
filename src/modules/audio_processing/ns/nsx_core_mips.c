/*
 *  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "noise_suppression_x.h"

#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "cpu_features_wrapper.h"
#include "nsx_core.h"


// For the noise supression process, synthesis, read out fully processed
// segment, and update synthesis buffer.
void SynthesisUpdate_mips(NsxInst_t* inst,
                          int16_t* out_frame,
                          int16_t gain_factor) {

  int iters = inst->blockLen10ms >> 2;
  int after = inst->blockLen10ms & 3;
  int r0, r1, r2, r3, r4, r5, r6, r7;
  WebRtc_Word16 *window = (WebRtc_Word16*)inst->window;
  int16_t *real = inst->real;
  int16_t *synthBuf = inst->synthesisBuffer;
  int16_t *out = out_frame;
  int sat_pos = 0x7fff;
  int sat_neg = 0xffff8000;
  int block10 = (int)inst->blockLen10ms;
  int anaLen = (int)inst->anaLen;

  __asm__ volatile(
    ".set push                                              \n\t"
    ".set noreorder                                         \n\t"
    "1:                                                     \n\t"
    "blez       %[iters],   2f                              \n\t"
    " nop                                                   \n\t"
    "lh         %[r0],      0(%[window])                    \n\t"
    "lh         %[r1],      0(%[real])                      \n\t"
    "lh         %[r2],      2(%[window])                    \n\t"
    "lh         %[r3],      2(%[real])                      \n\t"
    "lh         %[r4],      4(%[window])                    \n\t"
    "lh         %[r5],      4(%[real])                      \n\t"
    "lh         %[r6],      6(%[window])                    \n\t"
    "lh         %[r7],      6(%[real])                      \n\t"
    "mul        %[r0],      %[r0],          %[r1]           \n\t"
    "mul        %[r2],      %[r2],          %[r3]           \n\t"
    "mul        %[r4],      %[r4],          %[r5]           \n\t"
    "mul        %[r6],      %[r6],          %[r7]           \n\t"
    "addiu      %[r0],      %[r0],          0x2000          \n\t"
    "addiu      %[r2],      %[r2],          0x2000          \n\t"
    "addiu      %[r4],      %[r4],          0x2000          \n\t"
    "addiu      %[r6],      %[r6],          0x2000          \n\t"
    "sra        %[r0],      %[r0],          14              \n\t"
    "sra        %[r2],      %[r2],          14              \n\t"
    "sra        %[r4],      %[r4],          14              \n\t"
    "sra        %[r6],      %[r6],          14              \n\t"
    "mul        %[r0],      %[r0],          %[gain_factor]  \n\t"
    "mul        %[r2],      %[r2],          %[gain_factor]  \n\t"
    "mul        %[r4],      %[r4],          %[gain_factor]  \n\t"
    "mul        %[r6],      %[r6],          %[gain_factor]  \n\t"
    "addiu      %[r0],      %[r0],          0x1000          \n\t"
    "addiu      %[r2],      %[r2],          0x1000          \n\t"
    "addiu      %[r4],      %[r4],          0x1000          \n\t"
    "addiu      %[r6],      %[r6],          0x1000          \n\t"
    "sra        %[r0],      %[r0],          13              \n\t"
    "sra        %[r2],      %[r2],          13              \n\t"
    "sra        %[r4],      %[r4],          13              \n\t"
    "sra        %[r6],      %[r6],          13              \n\t"
    "slt        %[r1],      %[r0],          %[sat_pos]      \n\t"
    "slt        %[r3],      %[r2],          %[sat_pos]      \n\t"
    "slt        %[r5],      %[r4],          %[sat_pos]      \n\t"
    "slt        %[r7],      %[r6],          %[sat_pos]      \n\t"
    "movz       %[r0],      %[sat_pos],     %[r1]           \n\t"
    "movz       %[r2],      %[sat_pos],     %[r3]           \n\t"
    "movz       %[r4],      %[sat_pos],     %[r5]           \n\t"
    "movz       %[r6],      %[sat_pos],     %[r7]           \n\t"
    "lh         %[r1],      0(%[synthBuf])                  \n\t"
    "lh         %[r3],      2(%[synthBuf])                  \n\t"
    "lh         %[r5],      4(%[synthBuf])                  \n\t"
    "lh         %[r7],      6(%[synthBuf])                  \n\t"
    "addu       %[r0],      %[r0],          %[r1]           \n\t"
    "addu       %[r2],      %[r2],          %[r3]           \n\t"
    "addu       %[r4],      %[r4],          %[r5]           \n\t"
    "addu       %[r6],      %[r6],          %[r7]           \n\t"
    "slt        %[r1],      %[r0],          %[sat_pos]      \n\t"
    "slt        %[r3],      %[r2],          %[sat_pos]      \n\t"
    "slt        %[r5],      %[r4],          %[sat_pos]      \n\t"
    "slt        %[r7],      %[r6],          %[sat_pos]      \n\t"
    "movz       %[r0],      %[sat_pos],     %[r1]           \n\t"
    "movz       %[r2],      %[sat_pos],     %[r3]           \n\t"
    "movz       %[r4],      %[sat_pos],     %[r5]           \n\t"
    "movz       %[r6],      %[sat_pos],     %[r7]           \n\t"
    "slt        %[r1],      %[r0],          %[sat_neg]      \n\t"
    "slt        %[r3],      %[r2],          %[sat_neg]      \n\t"
    "slt        %[r5],      %[r4],          %[sat_neg]      \n\t"
    "slt        %[r7],      %[r6],          %[sat_neg]      \n\t"
    "movn       %[r0],      %[sat_neg],     %[r1]           \n\t"
    "movn       %[r2],      %[sat_neg],     %[r3]           \n\t"
    "movn       %[r4],      %[sat_neg],     %[r5]           \n\t"
    "movn       %[r6],      %[sat_neg],     %[r7]           \n\t"
    "sh         %[r0],      0(%[synthBuf])                  \n\t"
    "sh         %[r2],      2(%[synthBuf])                  \n\t"
    "sh         %[r4],      4(%[synthBuf])                  \n\t"
    "sh         %[r6],      6(%[synthBuf])                  \n\t"
    "sh         %[r0],      0(%[out])                       \n\t"
    "sh         %[r2],      2(%[out])                       \n\t"
    "sh         %[r4],      4(%[out])                       \n\t"
    "sh         %[r6],      6(%[out])                       \n\t"
    "addiu      %[window],  %[window],      8               \n\t"
    "addiu      %[real],    %[real],        8               \n\t"
    "addiu      %[synthBuf],%[synthBuf],    8               \n\t"
    "addiu      %[out],     %[out],         8               \n\t"
    "b          1b                                          \n\t"
    " addiu     %[iters],   %[iters],       -1              \n\t"
    "2:                                                     \n\t"
    "blez       %[after],   3f                              \n\t"
    " subu      %[block10], %[anaLen],      %[block10]      \n\t"
    "lh         %[r0],      0(%[window])                    \n\t"
    "lh         %[r1],      0(%[real])                      \n\t"
    "mul        %[r0],      %[r0],          %[r1]           \n\t"
    "addiu      %[window],  %[window],      2               \n\t"
    "addiu      %[real],    %[real],        2               \n\t"
    "addiu      %[r0],      %[r0],          0x2000          \n\t"
    "sra        %[r0],      %[r0],          14              \n\t"
    "mul        %[r0],      %[r0],          %[gain_factor]  \n\t"
    "addiu      %[r0],      %[r0],          0x1000          \n\t"
    "sra        %[r0],      %[r0],          13              \n\t"
    "slt        %[r1],      %[r0],          %[sat_pos]      \n\t"
    "movz       %[r0],      %[sat_pos],     %[r1]           \n\t"
    "lh         %[r1],      0(%[synthBuf])                  \n\t"
    "addu       %[r0],      %[r0],          %[r1]           \n\t"
    "slt        %[r1],      %[r0],          %[sat_pos]      \n\t"
    "movz       %[r0],      %[sat_pos],     %[r1]           \n\t"
    "slt        %[r1],      %[r0],          %[sat_neg]      \n\t"
    "movn       %[r0],      %[sat_neg],     %[r1]           \n\t"
    "sh         %[r0],      0(%[synthBuf])                  \n\t"
    "sh         %[r0],      0(%[out])                       \n\t"
    "addiu      %[synthBuf],%[synthBuf],    2               \n\t"
    "addiu      %[out],     %[out],         2               \n\t"
    "b          2b                                          \n\t"
    " addiu     %[after],   %[after],       -1              \n\t"
    "3:                                                     \n\t"
    "sra        %[iters],   %[block10],     2               \n\t"
    "4:                                                     \n\t"
    "blez       %[iters],   5f                              \n\t"
    " andi      %[after],   %[block10],     3               \n\t"
    "lh         %[r0],      0(%[window])                    \n\t"
    "lh         %[r1],      0(%[real])                      \n\t"
    "lh         %[r2],      2(%[window])                    \n\t"
    "lh         %[r3],      2(%[real])                      \n\t"
    "lh         %[r4],      4(%[window])                    \n\t"
    "lh         %[r5],      4(%[real])                      \n\t"
    "lh         %[r6],      6(%[window])                    \n\t"
    "lh         %[r7],      6(%[real])                      \n\t"
    "mul        %[r0],      %[r0],          %[r1]           \n\t"
    "mul        %[r2],      %[r2],          %[r3]           \n\t"
    "mul        %[r4],      %[r4],          %[r5]           \n\t"
    "mul        %[r6],      %[r6],          %[r7]           \n\t"
    "addiu      %[r0],      %[r0],          0x2000          \n\t"
    "addiu      %[r2],      %[r2],          0x2000          \n\t"
    "addiu      %[r4],      %[r4],          0x2000          \n\t"
    "addiu      %[r6],      %[r6],          0x2000          \n\t"
    "sra        %[r0],      %[r0],          14              \n\t"
    "sra        %[r2],      %[r2],          14              \n\t"
    "sra        %[r4],      %[r4],          14              \n\t"
    "sra        %[r6],      %[r6],          14              \n\t"
    "mul        %[r0],      %[r0],          %[gain_factor]  \n\t"
    "mul        %[r2],      %[r2],          %[gain_factor]  \n\t"
    "mul        %[r4],      %[r4],          %[gain_factor]  \n\t"
    "mul        %[r6],      %[r6],          %[gain_factor]  \n\t"
    "addiu      %[r0],      %[r0],          0x1000          \n\t"
    "addiu      %[r2],      %[r2],          0x1000          \n\t"
    "addiu      %[r4],      %[r4],          0x1000          \n\t"
    "addiu      %[r6],      %[r6],          0x1000          \n\t"
    "sra        %[r0],      %[r0],          13              \n\t"
    "sra        %[r2],      %[r2],          13              \n\t"
    "sra        %[r4],      %[r4],          13              \n\t"
    "sra        %[r6],      %[r6],          13              \n\t"
    "slt        %[r1],      %[r0],          %[sat_pos]      \n\t"
    "slt        %[r3],      %[r2],          %[sat_pos]      \n\t"
    "slt        %[r5],      %[r4],          %[sat_pos]      \n\t"
    "slt        %[r7],      %[r6],          %[sat_pos]      \n\t"
    "movz       %[r0],      %[sat_pos],     %[r1]           \n\t"
    "movz       %[r2],      %[sat_pos],     %[r3]           \n\t"
    "movz       %[r4],      %[sat_pos],     %[r5]           \n\t"
    "movz       %[r6],      %[sat_pos],     %[r7]           \n\t"
    "lh         %[r1],      0(%[synthBuf])                  \n\t"
    "lh         %[r3],      2(%[synthBuf])                  \n\t"
    "lh         %[r5],      4(%[synthBuf])                  \n\t"
    "lh         %[r7],      6(%[synthBuf])                  \n\t"
    "addu       %[r0],      %[r0],          %[r1]           \n\t"
    "addu       %[r2],      %[r2],          %[r3]           \n\t"
    "addu       %[r4],      %[r4],          %[r5]           \n\t"
    "addu       %[r6],      %[r6],          %[r7]           \n\t"
    "slt        %[r1],      %[r0],          %[sat_pos]      \n\t"
    "slt        %[r3],      %[r2],          %[sat_pos]      \n\t"
    "slt        %[r5],      %[r4],          %[sat_pos]      \n\t"
    "slt        %[r7],      %[r6],          %[sat_pos]      \n\t"
    "movz       %[r0],      %[sat_pos],     %[r1]           \n\t"
    "movz       %[r2],      %[sat_pos],     %[r3]           \n\t"
    "movz       %[r4],      %[sat_pos],     %[r5]           \n\t"
    "movz       %[r6],      %[sat_pos],     %[r7]           \n\t"
    "slt        %[r1],      %[r0],          %[sat_neg]      \n\t"
    "slt        %[r3],      %[r2],          %[sat_neg]      \n\t"
    "slt        %[r5],      %[r4],          %[sat_neg]      \n\t"
    "slt        %[r7],      %[r6],          %[sat_neg]      \n\t"
    "movn       %[r0],      %[sat_neg],     %[r1]           \n\t"
    "movn       %[r2],      %[sat_neg],     %[r3]           \n\t"
    "movn       %[r4],      %[sat_neg],     %[r5]           \n\t"
    "movn       %[r6],      %[sat_neg],     %[r7]           \n\t"
    "sh         %[r0],      0(%[synthBuf])                  \n\t"
    "sh         %[r2],      2(%[synthBuf])                  \n\t"
    "sh         %[r4],      4(%[synthBuf])                  \n\t"
    "sh         %[r6],      6(%[synthBuf])                  \n\t"
    "addiu      %[window],  %[window],      8               \n\t"
    "addiu      %[real],    %[real],        8               \n\t"
    "addiu      %[synthBuf],%[synthBuf],    8               \n\t"
    "b          4b                                          \n\t"
    " addiu     %[iters],   %[iters],       -1              \n\t"
    "5:                                                     \n\t"
    "blez       %[after],   6f                              \n\t"
    " nop                                                   \n\t"
    "lh         %[r0],      0(%[window])                    \n\t"
    "lh         %[r1],      0(%[real])                      \n\t"
    "mul        %[r0],      %[r0],          %[r1]           \n\t"
    "addiu      %[window],  %[window],      2               \n\t"
    "addiu      %[real],    %[real],        2               \n\t"
    "addiu      %[r0],      %[r0],          0x2000          \n\t"
    "sra        %[r0],      %[r0],          14              \n\t"
    "mul        %[r0],      %[r0],          %[gain_factor]  \n\t"
    "addiu      %[r0],      %[r0],          0x1000          \n\t"
    "sra        %[r0],      %[r0],          13              \n\t"
    "slt        %[r1],      %[r0],          %[sat_pos]      \n\t"
    "movz       %[r0],      %[sat_pos],     %[r1]           \n\t"
    "lh         %[r1],      0(%[synthBuf])                  \n\t"
    "addu       %[r0],      %[r0],          %[r1]           \n\t"
    "slt        %[r1],      %[r0],          %[sat_pos]      \n\t"
    "movz       %[r0],      %[sat_pos],     %[r1]           \n\t"
    "slt        %[r1],      %[r0],          %[sat_neg]      \n\t"
    "movn       %[r0],      %[sat_neg],     %[r1]           \n\t"
    "sh         %[r0],      0(%[synthBuf])                  \n\t"
    "addiu      %[synthBuf],%[synthBuf],    2               \n\t"
    "b          2b                                          \n\t"
    " addiu     %[after],   %[after],       -1              \n\t"
    "6:                                                     \n\t"
    ".set pop                                               \n\t"
    : [r0] "=&r" (r0), [r1] "=&r" (r1), [r2] "=&r" (r2), [r3] "=&r" (r3),
      [r4] "=&r" (r4), [r5] "=&r" (r5), [r6] "=&r" (r6), [r7] "=&r" (r7),
      [iters] "+r" (iters), [after] "+r" (after), [block10] "+r" (block10),
      [window] "+r" (window), [real] "+r" (real), [synthBuf] "+r" (synthBuf),
      [out] "+r" (out)
    : [gain_factor] "r" (gain_factor), [sat_pos] "r" (sat_pos),
      [sat_neg] "r" (sat_neg), [anaLen] "r" (anaLen)
    : "memory", "hi", "lo"
  );

  // update synthesis buffer
  WEBRTC_SPL_MEMCPY_W16(inst->synthesisBuffer,
                        inst->synthesisBuffer + inst->blockLen10ms,
                        inst->anaLen - inst->blockLen10ms);
  WebRtcSpl_ZerosArrayW16(inst->synthesisBuffer
      + inst->anaLen - inst->blockLen10ms, inst->blockLen10ms);
}

// Update analysis buffer for lower band, and window data before FFT.
void AnalysisUpdate_mips(NsxInst_t* inst,
                         int16_t* out,
                         int16_t* new_speech) {

  // For lower band update analysis buffer.
  WEBRTC_SPL_MEMCPY_W16(inst->analysisBuffer,
                        inst->analysisBuffer + inst->blockLen10ms,
                        inst->anaLen - inst->blockLen10ms);
  WEBRTC_SPL_MEMCPY_W16(inst->analysisBuffer
      + inst->anaLen - inst->blockLen10ms, new_speech, inst->blockLen10ms);

  // Window data before FFT.
  int iters, after;
  int anaLen = inst->anaLen;
  int *window = (int*)inst->window;
  int *anaBuf = (int*)inst->analysisBuffer;
  int *outBuf = (int*)out;
  int r0, r1, r2, r3, r4, r5, r6, r7;

#if defined(__mips_dsp)
  int r8;
  __asm__ volatile(
    ".set push                                                          \n\t"
    ".set noreorder                                                     \n\t"
    "sra            %[iters],       %[anaLen],          3               \n\t"
    "1:                                                                 \n\t"
    "blez           %[iters],       2f                                  \n\t"
    " nop                                                               \n\t"
    "lw             %[r0],          0(%[window])                        \n\t"
    "lw             %[r1],          0(%[anaBuf])                        \n\t"
    "lw             %[r2],          4(%[window])                        \n\t"
    "lw             %[r3],          4(%[anaBuf])                        \n\t"
    "lw             %[r4],          8(%[window])                        \n\t"
    "lw             %[r5],          8(%[anaBuf])                        \n\t"
    "lw             %[r6],          12(%[window])                       \n\t"
    "lw             %[r7],          12(%[anaBuf])                       \n\t"
    "muleq_s.w.phl  %[r8],          %[r0],              %[r1]           \n\t"
    "muleq_s.w.phr  %[r0],          %[r0],              %[r1]           \n\t"
    "muleq_s.w.phl  %[r1],          %[r2],              %[r3]           \n\t"
    "muleq_s.w.phr  %[r2],          %[r2],              %[r3]           \n\t"
    "muleq_s.w.phl  %[r3],          %[r4],              %[r5]           \n\t"
    "muleq_s.w.phr  %[r4],          %[r4],              %[r5]           \n\t"
    "muleq_s.w.phl  %[r5],          %[r6],              %[r7]           \n\t"
    "muleq_s.w.phr  %[r6],          %[r6],              %[r7]           \n\t"
#if (__mips_dsp_rev >= 2)
    "precr_sra_r.ph.w  %[r8],       %[r0],              15              \n\t"
    "precr_sra_r.ph.w  %[r1],       %[r2],              15              \n\t"
    "precr_sra_r.ph.w  %[r3],       %[r4],              15              \n\t"
    "precr_sra_r.ph.w  %[r5],       %[r6],              15              \n\t"
    "sw                %[r8],       0(%[outBuf])                        \n\t"
    "sw                %[r1],       4(%[outBuf])                        \n\t"
    "sw                %[r3],       8(%[outBuf])                        \n\t"
    "sw                %[r5],       12(%[outBuf])                       \n\t"
#else
    "shra_r.w       %[r8],          %[r8],              15              \n\t"
    "shra_r.w       %[r0],          %[r0],              15              \n\t"
    "shra_r.w       %[r1],          %[r1],              15              \n\t"
    "shra_r.w       %[r2],          %[r2],              15              \n\t"
    "shra_r.w       %[r3],          %[r3],              15              \n\t"
    "shra_r.w       %[r4],          %[r4],              15              \n\t"
    "shra_r.w       %[r5],          %[r5],              15              \n\t"
    "shra_r.w       %[r6],          %[r6],              15              \n\t"
    "sll            %[r0],          %[r0],              16              \n\t"
    "sll            %[r2],          %[r2],              16              \n\t"
    "sll            %[r4],          %[r4],              16              \n\t"
    "sll            %[r6],          %[r6],              16              \n\t"
    "packrl.ph      %[r0],          %[r8],              %[r0]           \n\t"
    "packrl.ph      %[r2],          %[r1],              %[r2]           \n\t"
    "packrl.ph      %[r4],          %[r3],              %[r4]           \n\t"
    "packrl.ph      %[r6],          %[r5],              %[r6]           \n\t"
    "sw             %[r0],          0(%[outBuf])                        \n\t"
    "sw             %[r2],          4(%[outBuf])                        \n\t"
    "sw             %[r4],          8(%[outBuf])                        \n\t"
    "sw             %[r6],          12(%[outBuf])                       \n\t"
#endif
    "addiu          %[window],      %[window],          16              \n\t"
    "addiu          %[anaBuf],      %[anaBuf],          16              \n\t"
    "addiu          %[outBuf],      %[outBuf],          16              \n\t"
    "b              1b                                                  \n\t"
    " addiu         %[iters],       %[iters],           -1              \n\t"
    "2:                                                                 \n\t"
    "andi           %[after],       %[anaLen],          7               \n\t"
    "3:                                                                 \n\t"
    "blez           %[after],       4f                                  \n\t"
    " nop                                                               \n\t"
    "lh             %[r0],          0(%[window])                        \n\t"
    "lh             %[r1],          0(%[anaBuf])                        \n\t"
    "mul            %[r0],          %[r0],              %[r1]           \n\t"
    "addiu          %[window],      %[window],          2               \n\t"
    "addiu          %[anaBuf],      %[anaBuf],          2               \n\t"
    "addiu          %[outBuf],      %[outBuf],          2               \n\t"
    "shra_r.w       %[r0],          %[r0],              14              \n\t"
    "sh             %[r0],          -2(%[outBuf])                       \n\t"
    "b              3b                                                  \n\t"
    " addiu         %[after],       %[after],           -1              \n\t"
    "4:                                                                 \n\t"
    ".set pop                                                           \n\t"
    : [r0] "=&r" (r0), [r1] "=&r" (r1), [r2] "=&r" (r2), [r3] "=&r" (r3),
      [r4] "=&r" (r4), [r5] "=&r" (r5), [r6] "=&r" (r6), [r7] "=&r" (r7),
      [iters] "=&r" (iters), [after] "=&r" (after), [window] "+r" (window),
      [anaBuf] "+r" (anaBuf), [outBuf] "+r" (outBuf), [r8] "=&r" (r8)
    : [anaLen] "r" (anaLen)
    : "memory", "hi", "lo"
  );
#else
  __asm__ volatile(
    ".set push                                                          \n\t"
    ".set noreorder                                                     \n\t"
    "sra            %[iters],       %[anaLen],          2               \n\t"
    "1:                                                                 \n\t"
    "blez           %[iters],       2f                                  \n\t"
    " nop                                                               \n\t"
    "lh             %[r0],          0(%[window])                        \n\t"
    "lh             %[r1],          0(%[anaBuf])                        \n\t"
    "lh             %[r2],          2(%[window])                        \n\t"
    "lh             %[r3],          2(%[anaBuf])                        \n\t"
    "lh             %[r4],          4(%[window])                        \n\t"
    "lh             %[r5],          4(%[anaBuf])                        \n\t"
    "lh             %[r6],          6(%[window])                        \n\t"
    "lh             %[r7],          6(%[anaBuf])                        \n\t"
    "mul            %[r0],          %[r0],              %[r1]           \n\t"
    "mul            %[r2],          %[r2],              %[r3]           \n\t"
    "mul            %[r4],          %[r4],              %[r5]           \n\t"
    "mul            %[r6],          %[r6],              %[r7]           \n\t"
    "addiu          %[window],      %[window],          8               \n\t"
    "addiu          %[anaBuf],      %[anaBuf],          8               \n\t"
    "addiu          %[r0],          %[r0],              0x2000          \n\t"
    "addiu          %[r2],          %[r2],              0x2000          \n\t"
    "addiu          %[r4],          %[r4],              0x2000          \n\t"
    "addiu          %[r6],          %[r6],              0x2000          \n\t"
    "sra            %[r0],          %[r0],              14              \n\t"
    "sra            %[r2],          %[r2],              14              \n\t"
    "sra            %[r4],          %[r4],              14              \n\t"
    "sra            %[r6],          %[r6],              14              \n\t"
    "sh             %[r0],          0(%[outBuf])                        \n\t"
    "sh             %[r2],          2(%[outBuf])                        \n\t"
    "sh             %[r4],          4(%[outBuf])                        \n\t"
    "sh             %[r6],          6(%[outBuf])                        \n\t"
    "addiu          %[outBuf],      %[outBuf],          8               \n\t"
    "b              1b                                                  \n\t"
    " addiu         %[iters],       %[iters],           -1              \n\t"
    "2:                                                                 \n\t"
    "andi           %[after],       %[anaLen],          3               \n\t"
    "3:                                                                 \n\t"
    "blez           %[after],       4f                                  \n\t"
    " nop                                                               \n\t"
    "lh             %[r0],          0(%[window])                        \n\t"
    "lh             %[r1],          0(%[anaBuf])                        \n\t"
    "mul            %[r0],          %[r0],              %[r1]           \n\t"
    "addiu          %[window],      %[window],          2               \n\t"
    "addiu          %[anaBuf],      %[anaBuf],          2               \n\t"
    "addiu          %[outBuf],      %[outBuf],          2               \n\t"
    "addiu          %[r0],          %[r0],              0x2000          \n\t"
    "sra            %[r0],          %[r0],              14              \n\t"
    "sh             %[r0],          -2(%[outBuf])                       \n\t"
    "b              3b                                                  \n\t"
    " addiu         %[after],       %[after],           -1              \n\t"
    "4:                                                                 \n\t"
    ".set pop                                                           \n\t"
    : [r0] "=&r" (r0), [r1] "=&r" (r1), [r2] "=&r" (r2), [r3] "=&r" (r3),
      [r4] "=&r" (r4), [r5] "=&r" (r5), [r6] "=&r" (r6), [r7] "=&r" (r7),
      [iters] "=&r" (iters), [after] "=&r" (after), [window] "+r" (window),
      [anaBuf] "+r" (anaBuf), [outBuf] "+r" (outBuf)
    : [anaLen] "r" (anaLen)
    : "memory", "hi", "lo"
  );
#endif
}

void UpdateNoiseEstimate_mips(NsxInst_t* inst, int offset) {
  WebRtc_Word16 tmp16 = 0;
  const WebRtc_Word16 kExp2Const = 11819; // Q13

  tmp16 = WebRtcSpl_MaxValueW16(inst->noiseEstLogQuantile + offset,
                                   inst->magnLen);
  // Guarantee a Q-domain as high as possible and still fit in int16
  inst->qNoise = 14 - (int) WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(
                   kExp2Const, tmp16, 21);

  int magnLen = inst->magnLen;
  int iters, after;
  int *in = (int*)(&inst->noiseEstLogQuantile[offset]);
  int *out = (int*)inst->noiseEstQuantile;
  int r0, r1, r2, r3, r4, r5, r6, r7, r8, r9;
  int k0 = 0x1fffff;
  int k1 = 0x200000;
  int sat_max = 32767;
  int sat_min = -32768;

  __asm__ volatile(
    ".set push                                                  \n\t"
    ".set noreorder                                             \n\t"
    "sra            %[iters],   %[magnLen],     2               \n\t"
    "1:                                                         \n\t"
    "blez           %[iters],   2f                              \n\t"
    " nop                                                       \n\t"
    "lh             %[r0],      0(%[in])                        \n\t"
    "lh             %[r1],      2(%[in])                        \n\t"
    "lh             %[r2],      4(%[in])                        \n\t"
    "lh             %[r3],      6(%[in])                        \n\t"
    "mul            %[r0],      %[r0],          %[kExp2Const]   \n\t"
    "mul            %[r1],      %[r1],          %[kExp2Const]   \n\t"
    "mul            %[r2],      %[r2],          %[kExp2Const]   \n\t"
    "mul            %[r3],      %[r3],          %[kExp2Const]   \n\t"
    "addiu          %[in],      %[in],          8               \n\t"
    "and            %[r4],      %[r0],          %[k0]           \n\t"
    "and            %[r5],      %[r1],          %[k0]           \n\t"
    "and            %[r6],      %[r2],          %[k0]           \n\t"
    "and            %[r7],      %[r3],          %[k0]           \n\t"
    "or             %[r4],      %[r4],          %[k1]           \n\t"
    "or             %[r5],      %[r5],          %[k1]           \n\t"
    "or             %[r6],      %[r6],          %[k1]           \n\t"
    "or             %[r7],      %[r7],          %[k1]           \n\t"
    "sra            %[r0],      %[r0],          21              \n\t"
    "sra            %[r1],      %[r1],          21              \n\t"
    "sra            %[r2],      %[r2],          21              \n\t"
    "sra            %[r3],      %[r3],          21              \n\t"
    "addiu          %[r0],      %[r0],          -21             \n\t"
    "addiu          %[r1],      %[r1],          -21             \n\t"
    "addiu          %[r2],      %[r2],          -21             \n\t"
    "addiu          %[r3],      %[r3],          -21             \n\t"
    "addu           %[r0],      %[r0],          %[qNoise]       \n\t"
    "addu           %[r1],      %[r1],          %[qNoise]       \n\t"
    "addu           %[r2],      %[r2],          %[qNoise]       \n\t"
    "addu           %[r3],      %[r3],          %[qNoise]       \n\t"
    "neg            %[r8],      %[r0]                           \n\t"
    "sra            %[r9],      %[r4],          %[r8]           \n\t"
    "sll            %[r4],      %[r4],          %[r0]           \n\t"
    "slt            %[r0],      %[r0],          $0              \n\t"
    "movn           %[r4],      %[r9],          %[r0]           \n\t"
    "neg            %[r8],      %[r1]                           \n\t"
    "sra            %[r9],      %[r5],          %[r8]           \n\t"
    "sll            %[r5],      %[r5],          %[r1]           \n\t"
    "slt            %[r1],      %[r1],          $0              \n\t"
    "movn           %[r5],      %[r9],          %[r1]           \n\t"
    "neg            %[r8],      %[r2]                           \n\t"
    "sra            %[r9],      %[r6],          %[r8]           \n\t"
    "sll            %[r6],      %[r6],          %[r2]           \n\t"
    "slt            %[r2],      %[r2],          $0              \n\t"
    "movn           %[r6],      %[r9],          %[r2]           \n\t"
    "neg            %[r8],      %[r3]                           \n\t"
    "sra            %[r9],      %[r7],          %[r8]           \n\t"
    "sll            %[r7],      %[r7],          %[r3]           \n\t"
    "slt            %[r3],      %[r3],          $0              \n\t"
    "movn           %[r7],      %[r9],          %[r3]           \n\t"
#if defined(__mips_dsp)
    "shll_s.w       %[r4],      %[r4],          16              \n\t"
    "shll_s.w       %[r5],      %[r5],          16              \n\t"
    "shll_s.w       %[r6],      %[r6],          16              \n\t"
    "shll_s.w       %[r7],      %[r7],          16              \n\t"
    "sra            %[r4],      %[r4],          16              \n\t"
    "sra            %[r5],      %[r5],          16              \n\t"
    "sra            %[r6],      %[r6],          16              \n\t"
    "sra            %[r7],      %[r7],          16              \n\t"
#else
    "slt            %[r0],      %[r4],          %[sat_min]      \n\t"
    "movn           %[r4],      %[sat_min],     %[r0]           \n\t"
    "slt            %[r1],      %[r5],          %[sat_min]      \n\t"
    "movn           %[r5],      %[sat_min],     %[r1]           \n\t"
    "slt            %[r2],      %[r6],          %[sat_min]      \n\t"
    "movn           %[r6],      %[sat_min],     %[r2]           \n\t"
    "slt            %[r3],      %[r7],          %[sat_min]      \n\t"
    "movn           %[r7],      %[sat_min],     %[r3]           \n\t"
    "slt            %[r0],      %[sat_max],     %[r4]           \n\t"
    "movn           %[r4],      %[sat_max],     %[r0]           \n\t"
    "slt            %[r1],      %[sat_max],     %[r5]           \n\t"
    "movn           %[r5],      %[sat_max],     %[r1]           \n\t"
    "slt            %[r2],      %[sat_max],     %[r6]           \n\t"
    "movn           %[r6],      %[sat_max],     %[r2]           \n\t"
    "slt            %[r3],      %[sat_max],     %[r7]           \n\t"
    "movn           %[r7],      %[sat_max],     %[r3]           \n\t"
#endif
    "sh             %[r4],      0(%[out])                       \n\t"
    "sh             %[r5],      2(%[out])                       \n\t"
    "sh             %[r6],      4(%[out])                       \n\t"
    "sh             %[r7],      6(%[out])                       \n\t"
    "addiu          %[out],     %[out],         8               \n\t"
    "b              1b                                          \n\t"
    " addiu         %[iters],   %[iters],       -1              \n\t"
    "2:                                                         \n\t"
    "andi           %[after],   %[magnLen],     3               \n\t"
    "3:                                                         \n\t"
    "blez           %[after],   4f                              \n\t"
    " nop                                                       \n\t"
    "lh             %[r0],      0(%[in])                        \n\t"
    "mul            %[r0],      %[r0],          %[kExp2Const]   \n\t"
    "addiu          %[in],      %[in],          2               \n\t"
    "and            %[r4],      %[r0],          %[k0]           \n\t"
    "or             %[r4],      %[r4],          %[k1]           \n\t"
    "sra            %[r0],      %[r0],          21              \n\t"
    "addiu          %[r0],      %[r0],          -21             \n\t"
    "addu           %[r0],      %[r0],          %[qNoise]       \n\t"
    "neg            %[r8],      %[r0]                           \n\t"
    "sra            %[r9],      %[r4],          %[r8]           \n\t"
    "sll            %[r4],      %[r4],          %[r0]           \n\t"
    "slt            %[r0],      %[r0],          $0              \n\t"
    "movn           %[r4],      %[r9],          %[r0]           \n\t"
    "slt            %[r0],      %[r4],          %[sat_min]      \n\t"
    "movn           %[r4],      %[sat_min],     %[r0]           \n\t"
    "slt            %[r1],      %[r5],          %[sat_min]      \n\t"
    "movn           %[r5],      %[sat_min],     %[r1]           \n\t"
    "sh             %[r4],      0(%[out])                       \n\t"
    "addiu          %[out],     %[out],         2               \n\t"
    "b              3b                                          \n\t"
    " addiu         %[after],   %[after],       -1              \n\t"
    "4:                                                         \n\t"
    ".set pop                                                   \n\t"
    : [r0] "=&r" (r0), [r1] "=&r" (r1), [r2] "=&r" (r2), [r3] "=&r" (r3),
      [r4] "=&r" (r4), [r5] "=&r" (r5), [r6] "=&r" (r6), [r7] "=&r" (r7),
      [r8] "=&r" (r8), [r9] "=&r" (r9), [in] "+r" (in), [out] "+r" (out),
      [iters] "=&r" (iters), [after] "=&r" (after)
    : [k0] "r" (k0), [k1] "r" (k1), [sat_max] "r" (sat_max), [sat_min] "r" (sat_min),
      [kExp2Const] "r" (kExp2Const), [qNoise] "r" (inst->qNoise), [magnLen] "r" (magnLen)
    : "memory", "hi", "lo"
  );
}

void NoiseEstimation_mips(NsxInst_t* inst,
                          uint16_t* magn,
                          uint32_t* noise,
                          int16_t* q_noise) {
  WebRtc_Word16 lmagn[HALF_ANAL_BLOCKL], counter, countDiv;
  WebRtc_Word16 countProd, delta, zeros, frac;
  WebRtc_Word16 log2, tabind, logval, tmp16, tmp16no1, tmp16no2;
  WebRtc_Word16 tmp16no1_1,tmp16no1_2,tmp16no1_3,tmp16no1_4;
  WebRtc_Word16 tmp16no2_1,tmp16no2_2,tmp16no2_3,tmp16no2_4;
  const int16_t log2_const = 22713; // Q15
  const int16_t width_factor = 21845;
  int tmp32_1, tmp32_2, tmp32_3, tmp32_4;
  int i, s, offset;

  WebRtc_Word16 tmp;
  WebRtc_Word16 deltaTemp;
  WebRtc_Word16 log2_1, log2_2, log2_3, log2_4;
  WebRtc_Word16 zeros1, zeros2, zeros3, zeros4;
  WebRtc_Word16 frac1, frac2, frac3, frac4;

  WebRtc_Word16 noiseEstDensityTmp1;
  WebRtc_Word16 noiseEstDensityTmp2;
  WebRtc_Word16 noiseEstDensityTmp3;
  WebRtc_Word16 noiseEstDensityTmp4;
  WebRtc_Word16 noiseEstLogQuantileTmp1;
  WebRtc_Word16 noiseEstLogQuantileTmp2;
  WebRtc_Word16 noiseEstLogQuantileTmp3;
  WebRtc_Word16 noiseEstLogQuantileTmp4;

  tabind = inst->stages - inst->normData;
  assert(tabind < 9);
  assert(tabind > -9);
  if (tabind < 0) {
    logval = -WebRtcNsx_kLogTable[-tabind];
  } else {
    logval = WebRtcNsx_kLogTable[tabind];
  }

  int loop_size = inst->magnLen >> 2;

  // lmagn(i)=log(magn(i))=log(2)*log2(magn(i))
  // magn is in Q(-stages), and the real lmagn values are:
  // real_lmagn(i)=log(magn(i)*2^stages)=log(magn(i))+log(2^stages)
  // lmagn in Q8
  for (i = 0; i < loop_size; i++) {
    tmp16no1_1 = magn[4*i];
    tmp16no1_2 = magn[4*i + 1];
    tmp16no1_3 = magn[4*i + 2];
    tmp16no1_4 = magn[4*i + 3];

    tmp32_1 = (WebRtc_UWord32)tmp16no1_1;
    tmp32_2 = (WebRtc_UWord32)tmp16no1_2;
    tmp32_3 = (WebRtc_UWord32)tmp16no1_3;
    tmp32_4 = (WebRtc_UWord32)tmp16no1_4;

    zeros1 = WebRtcSpl_NormU32(tmp32_1);
    zeros2 = WebRtcSpl_NormU32(tmp32_2);
    zeros3 = WebRtcSpl_NormU32(tmp32_3);
    zeros4 = WebRtcSpl_NormU32(tmp32_4);

    frac1 = (WebRtc_Word16)(((tmp32_1 << zeros1) & 0x7FFFFFFF) >> 23);
    frac2 = (WebRtc_Word16)(((tmp32_2 << zeros2) & 0x7FFFFFFF) >> 23);
    frac3 = (WebRtc_Word16)(((tmp32_3 << zeros3) & 0x7FFFFFFF) >> 23);
    frac4 = (WebRtc_Word16)(((tmp32_4 << zeros4) & 0x7FFFFFFF) >> 23);

    assert(frac1 < 256);
    assert(frac2 < 256);
    assert(frac3 < 256);
    assert(frac4 < 256);

    tmp32_1 = WebRtcNsx_kLogTableFrac[frac1];
    tmp32_2 = WebRtcNsx_kLogTableFrac[frac2];
    tmp32_3 = WebRtcNsx_kLogTableFrac[frac3];
    tmp32_4 = WebRtcNsx_kLogTableFrac[frac4];

    log2_1 = (WebRtc_Word16)(((31 - zeros1) << 8) + tmp32_1);
    log2_2 = (WebRtc_Word16)(((31 - zeros2) << 8) + tmp32_2);
    log2_3 = (WebRtc_Word16)(((31 - zeros3) << 8) + tmp32_3);
    log2_4 = (WebRtc_Word16)(((31 - zeros4) << 8) + tmp32_4);

    tmp32_1 = WEBRTC_SPL_MUL_16_16(log2_1, log2_const);
    tmp32_2 = WEBRTC_SPL_MUL_16_16(log2_2, log2_const);
    tmp32_3 = WEBRTC_SPL_MUL_16_16(log2_3, log2_const);
    tmp32_4 = WEBRTC_SPL_MUL_16_16(log2_4, log2_const);

    tmp16no2_1 = logval;
    tmp16no2_2 = logval;
    tmp16no2_3 = logval;
    tmp16no2_4 = logval;

    log2_1 = (WebRtc_Word16)(tmp32_1>>15);
    log2_2 = (WebRtc_Word16)(tmp32_2>>15);
    log2_3 = (WebRtc_Word16)(tmp32_3>>15);
    log2_4 = (WebRtc_Word16)(tmp32_4>>15);

    if (tmp16no1_1) {
      tmp16no2_1 += log2_1;
    }
    if (tmp16no1_2) {
      tmp16no2_2 += log2_2;
    }
    if (tmp16no1_3) {
      tmp16no2_3 += log2_3;
    }
    if (tmp16no1_4) {
      tmp16no2_4 += log2_4;
    }

    lmagn[4*i] = tmp16no2_1;
    lmagn[4*i + 1] = tmp16no2_2;
    lmagn[4*i + 2] = tmp16no2_3;
    lmagn[4*i + 3] = tmp16no2_4;
  }

  for (i = inst->magnLen & 0xfffc; i < inst->magnLen; i++) {
    if (magn[i]) {
      zeros = WebRtcSpl_NormU32((WebRtc_UWord32)magn[i]);
      frac = (WebRtc_Word16)((((WebRtc_UWord32)magn[i] << zeros)
                              & 0x7FFFFFFF) >> 23);
      // log2(magn(i))
      assert(frac < 256);
      log2 = (WebRtc_Word16)(((31 - zeros) << 8)
                             + WebRtcNsx_kLogTableFrac[frac]);
      // log2(magn(i))*log(2)
      lmagn[i] = (WebRtc_Word16)WEBRTC_SPL_MUL_16_16_RSFT(log2, log2_const, 15);
      // + log(2^stages)
      lmagn[i] += logval;
    } else {
      lmagn[i] = logval;//0;
    }
  }

  // loop over simultaneous estimates
  for (s = 0; s < SIMULT; s++) {
    offset = s * inst->magnLen;

    // Get counter values from state
    counter = inst->noiseEstCounter[s];
    assert(counter < 201);
    countDiv = WebRtcNsx_kCounterDiv[counter];
    countProd = (WebRtc_Word16)WEBRTC_SPL_MUL_16_16(counter, countDiv);

    if (inst->blockIndex < END_STARTUP_LONG) deltaTemp = FACTOR_Q7_STARTUP;
    else deltaTemp = FACTOR_Q7;

    tmp = (WebRtc_Word16)WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(width_factor, countDiv, 15);

    // quant_est(...)
    for (i = 0; i < loop_size; i++)
    {
      noiseEstDensityTmp1 = inst->noiseEstDensity[offset + (4*i)];
      noiseEstDensityTmp2 = inst->noiseEstDensity[offset + (4*i + 1)];
      noiseEstDensityTmp3 = inst->noiseEstDensity[offset + (4*i + 2)];
      noiseEstDensityTmp4 = inst->noiseEstDensity[offset + (4*i + 3)];

      int factor1 = WebRtcSpl_NormW16(noiseEstDensityTmp1);
      int factor2 = WebRtcSpl_NormW16(noiseEstDensityTmp2);
      int factor3 = WebRtcSpl_NormW16(noiseEstDensityTmp3);
      int factor4 = WebRtcSpl_NormW16(noiseEstDensityTmp4);

      tmp32_1 = FACTOR_Q16 >> (14 - factor1);
      tmp32_2 = FACTOR_Q16 >> (14 - factor2);
      tmp32_3 = FACTOR_Q16 >> (14 - factor3);
      tmp32_4 = FACTOR_Q16 >> (14 - factor4);

      if (noiseEstDensityTmp1 > 512) {
        tmp16no1_1 = (int16_t)tmp32_1;
      } else {
        tmp16no1_1 = deltaTemp;
      }

      if (noiseEstDensityTmp2 > 512) {
        tmp16no1_2 = (int16_t)tmp32_2;
      } else {
        tmp16no1_2 = deltaTemp;
      }

      if (noiseEstDensityTmp3 > 512) {
        tmp16no1_3 = (int16_t)tmp32_3;
      } else {
        tmp16no1_3 = deltaTemp;
      }

      if (noiseEstDensityTmp4 > 512) {
        tmp16no1_4 = (int16_t)tmp32_4;
      } else {
        tmp16no1_4 = deltaTemp;
      }

      tmp32_1 = WEBRTC_SPL_MUL_16_16(tmp16no1_1, countDiv);
      tmp32_2 = WEBRTC_SPL_MUL_16_16(tmp16no1_2, countDiv);
      tmp32_3 = WEBRTC_SPL_MUL_16_16(tmp16no1_3, countDiv);
      tmp32_4 = WEBRTC_SPL_MUL_16_16(tmp16no1_4, countDiv);

      noiseEstLogQuantileTmp1 = inst->noiseEstLogQuantile[offset + (4*i)];
      noiseEstLogQuantileTmp2 = inst->noiseEstLogQuantile[offset + (4*i + 1)];
      noiseEstLogQuantileTmp3 = inst->noiseEstLogQuantile[offset + (4*i + 2)];
      noiseEstLogQuantileTmp4 = inst->noiseEstLogQuantile[offset + (4*i + 3)];

      tmp32_1 >>= 14;
      tmp32_2 >>= 14;
      tmp32_3 >>= 14;
      tmp32_4 >>= 14;

      tmp16no1_1 = (WebRtc_Word16)tmp32_1;
      tmp16no1_2 = (WebRtc_Word16)tmp32_2;
      tmp16no1_3 = (WebRtc_Word16)tmp32_3;
      tmp16no1_4 = (WebRtc_Word16)tmp32_4;

      WebRtc_Word16 lmagn1 = lmagn[(4*i)];
      WebRtc_Word16 lmagn2 = lmagn[(4*i + 1)];
      WebRtc_Word16 lmagn3 = lmagn[(4*i + 2)];
      WebRtc_Word16 lmagn4 = lmagn[(4*i + 3)];

      if (lmagn1 > noiseEstLogQuantileTmp1) {
        tmp16no1_1 += 2;
        tmp16no1_1 = WEBRTC_SPL_RSHIFT_W16(tmp16no1_1, 2);
        noiseEstLogQuantileTmp1 += tmp16no1_1;
      } else {
        tmp16no1_1++;
        tmp16no1_1 = WEBRTC_SPL_RSHIFT_W16(tmp16no1_1, 1);
        tmp16no2_1 = (WebRtc_Word16)WEBRTC_SPL_MUL_16_16_RSFT(tmp16no1_1, 3, 1);
        noiseEstLogQuantileTmp1 -= tmp16no2_1;
        if (noiseEstLogQuantileTmp1 < logval) {
          noiseEstLogQuantileTmp1 = logval;
        }
      }

      if (lmagn2 > noiseEstLogQuantileTmp2) {
        tmp16no1_2 += 2;
        tmp16no1_2 = WEBRTC_SPL_RSHIFT_W16(tmp16no1_2, 2);
        noiseEstLogQuantileTmp2 += tmp16no1_2;
      } else {
        tmp16no1_2++;
        tmp16no1_2 = WEBRTC_SPL_RSHIFT_W16(tmp16no1_2, 1);
        tmp16no2_2 = (WebRtc_Word16)WEBRTC_SPL_MUL_16_16_RSFT(tmp16no1_2, 3, 1);
        noiseEstLogQuantileTmp2 -= tmp16no2_2;
        if (noiseEstLogQuantileTmp2 < logval) {
          noiseEstLogQuantileTmp2 = logval;
        }
      }

      if (lmagn3 > noiseEstLogQuantileTmp3) {
        tmp16no1_3 += 2;
        tmp16no1_3 = WEBRTC_SPL_RSHIFT_W16(tmp16no1_3, 2);
        noiseEstLogQuantileTmp3 += tmp16no1_3;
      } else {
        tmp16no1_3++;
        tmp16no1_3 = WEBRTC_SPL_RSHIFT_W16(tmp16no1_3, 1);
        tmp16no2_3 = (WebRtc_Word16)WEBRTC_SPL_MUL_16_16_RSFT(tmp16no1_3, 3, 1);
        noiseEstLogQuantileTmp3 -= tmp16no2_3;
        if (noiseEstLogQuantileTmp3 < logval) {
          noiseEstLogQuantileTmp3 = logval;
        }
      }

      if (lmagn4 > noiseEstLogQuantileTmp4) {
        tmp16no1_4 += 2;
        tmp16no1_4 = WEBRTC_SPL_RSHIFT_W16(tmp16no1_4, 2);
        noiseEstLogQuantileTmp4 += tmp16no1_4;
      } else {
        tmp16no1_4++;
        tmp16no1_4 = WEBRTC_SPL_RSHIFT_W16(tmp16no1_4, 1);
        tmp16no2_4 = (WebRtc_Word16)WEBRTC_SPL_MUL_16_16_RSFT(tmp16no1_4, 3, 1);
        noiseEstLogQuantileTmp4 -= tmp16no2_4;
        if (noiseEstLogQuantileTmp4 < logval) {
          noiseEstLogQuantileTmp4 = logval;
        }
      }

      tmp16no1_1 = lmagn1 - noiseEstLogQuantileTmp1;
      tmp16no1_2 = lmagn2 - noiseEstLogQuantileTmp2;
      tmp16no1_3 = lmagn3 - noiseEstLogQuantileTmp3;
      tmp16no1_4 = lmagn4 - noiseEstLogQuantileTmp4;

      inst->noiseEstLogQuantile[offset + (4*i)]      = noiseEstLogQuantileTmp1;
      inst->noiseEstLogQuantile[offset + (4*i + 1)]  = noiseEstLogQuantileTmp2;
      inst->noiseEstLogQuantile[offset + (4*i + 2)]  = noiseEstLogQuantileTmp3;
      inst->noiseEstLogQuantile[offset + (4*i + 3)]  = noiseEstLogQuantileTmp4;

#if defined(__mips_dsp)
      __asm__ volatile(
        "absq_s.ph  %[tmp16no1_1],     %[tmp16no1_1]                  \n\t"
        "absq_s.ph  %[tmp16no1_2],     %[tmp16no1_2]                  \n\t"
        "absq_s.ph  %[tmp16no1_3],     %[tmp16no1_3]                  \n\t"
        "absq_s.ph  %[tmp16no1_4],     %[tmp16no1_4]                  \n\t"
        : [tmp16no1_1] "+r" (tmp16no1_1), [tmp16no1_2] "+r" (tmp16no1_2),
          [tmp16no1_3] "+r" (tmp16no1_3), [tmp16no1_4] "+r" (tmp16no1_4)
        :
        :
      );
#else
      tmp16no1_1 = WEBRTC_SPL_ABS_W16(tmp16no1_1);
      tmp16no1_2 = WEBRTC_SPL_ABS_W16(tmp16no1_2);
      tmp16no1_3 = WEBRTC_SPL_ABS_W16(tmp16no1_3);
      tmp16no1_4 = WEBRTC_SPL_ABS_W16(tmp16no1_4);

#endif //#if defined(__mips_dsp)

      if (tmp16no1_1 < WIDTH_Q8) {
        tmp16no1_1 = (WebRtc_Word16)WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(noiseEstDensityTmp1, countProd, 15);
        inst->noiseEstDensity[offset + (4*i)] = tmp16no1_1 + tmp;
      }

      if (tmp16no1_2 < WIDTH_Q8) {
        tmp16no1_2 = (WebRtc_Word16)WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(noiseEstDensityTmp2, countProd, 15);
        inst->noiseEstDensity[offset + (4*i + 1)] = tmp16no1_2 + tmp;
      }

      if (tmp16no1_3 < WIDTH_Q8) {
        tmp16no1_3 = (WebRtc_Word16)WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(noiseEstDensityTmp3, countProd, 15);
        inst->noiseEstDensity[offset + (4*i + 2)] = tmp16no1_3 + tmp;
      }

      if (tmp16no1_4 < WIDTH_Q8) {
        tmp16no1_4 = (WebRtc_Word16)WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(noiseEstDensityTmp4, countProd, 15);
        inst->noiseEstDensity[offset + (4*i + 3)] = tmp16no1_4 + tmp;
      }
    }

    // quant_est(...)
    for (i = inst->magnLen & 0xfffc; i < inst->magnLen; i++) {
      // compute delta
      if (inst->noiseEstDensity[offset + i] > 512) {
        // Get the value for delta by shifting intead of dividing.
        int factor = WebRtcSpl_NormW16(inst->noiseEstDensity[offset + i]);
        delta = (int16_t)(FACTOR_Q16 >> (14 - factor));
      } else {
        delta = FACTOR_Q7;
        if (inst->blockIndex < END_STARTUP_LONG) {
          // Smaller step size during startup. This prevents from using
          // unrealistic values causing overflow.
          delta = FACTOR_Q7_STARTUP;
        }
      }

      // update log quantile estimate
      tmp16 = (WebRtc_Word16)WEBRTC_SPL_MUL_16_16_RSFT(delta, countDiv, 14);
      if (lmagn[i] > inst->noiseEstLogQuantile[offset + i]) {
        // +=QUANTILE*delta/(inst->counter[s]+1) QUANTILE=0.25, =1 in Q2
        // CounterDiv=1/(inst->counter[s]+1) in Q15
        tmp16 += 2;
        tmp16no1 = WEBRTC_SPL_RSHIFT_W16(tmp16, 2);
        inst->noiseEstLogQuantile[offset + i] += tmp16no1;
      } else {
        tmp16 += 1;
        tmp16no1 = WEBRTC_SPL_RSHIFT_W16(tmp16, 1);
        // *(1-QUANTILE), in Q2 QUANTILE=0.25, 1-0.25=0.75=3 in Q2
        tmp16no2 = (WebRtc_Word16)WEBRTC_SPL_MUL_16_16_RSFT(tmp16no1, 3, 1);
        inst->noiseEstLogQuantile[offset + i] -= tmp16no2;
        if (inst->noiseEstLogQuantile[offset + i] < logval) {
          // This is the smallest fixed point representation we can
          // have, hence we limit the output.
          inst->noiseEstLogQuantile[offset + i] = logval;
        }
      }

      // update density estimate
      if (WEBRTC_SPL_ABS_W16(lmagn[i] - inst->noiseEstLogQuantile[offset + i])
          < WIDTH_Q8) {
        tmp16no1 = (WebRtc_Word16)WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(
                     inst->noiseEstDensity[offset + i], countProd, 15);
        tmp16no2 = (WebRtc_Word16)WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(
                     width_factor, countDiv, 15);
        inst->noiseEstDensity[offset + i] = tmp16no1 + tmp16no2;
      }
    } // end loop over magnitude spectrum

    if (counter >= END_STARTUP_LONG) {
      inst->noiseEstCounter[s] = 0;
      if (inst->blockIndex >= END_STARTUP_LONG) {
        WebRtcNsx_UpdateNoiseEstimate(inst, offset);
      }
    }
    inst->noiseEstCounter[s]++;

  } // end loop over simultaneous estimates

  // Sequentially update the noise during startup
  if (inst->blockIndex < END_STARTUP_LONG) {
    WebRtcNsx_UpdateNoiseEstimate(inst, offset);
  }

  for (i = 0; i < loop_size; i++)
  {
    tmp16no1_1 = (inst->noiseEstQuantile[4*i]);
    tmp16no1_2 = (inst->noiseEstQuantile[4*i+1]);
    tmp16no1_3 = (inst->noiseEstQuantile[4*i+2]);
    tmp16no1_4 = (inst->noiseEstQuantile[4*i+3]);

    noise[4*i] = (WebRtc_UWord32)tmp16no1_1;
    noise[4*i+1] = (WebRtc_UWord32)tmp16no1_2;
    noise[4*i+2] = (WebRtc_UWord32)tmp16no1_3;
    noise[4*i+3] = (WebRtc_UWord32)tmp16no1_4;
  }

  for (i = inst->magnLen & 0xfffc; i < inst->magnLen; i++) {
    noise[i] = (WebRtc_UWord32)(inst->noiseEstQuantile[i]); // Q(qNoise)
  }

  (*q_noise) = (WebRtc_Word16)inst->qNoise;

}

// Create a complex number buffer (out[]) as the intput (in[]) interleaved with
// zeros, and normalize it.
__inline void CreateComplexBuffer_mips(NsxInst_t* inst,
                                       int16_t* in,
                                       int16_t* out) {

  //inst->anaLen is set to 128 or 256
  int16_t tmp_1, tmp_2, tmp_3, tmp_4, tmp_zero = 0;
  int32_t loop_count = 0;
  __asm__ volatile(
    ".set push                                                              \n\t"
    ".set noreorder                                                         \n\t"
    "1:                                                                     \n\t"
    "lh         %[tmp_1],           0(%[in])                                \n\t"
    "lh         %[tmp_2],           2(%[in])                                \n\t"
    "lh         %[tmp_3],           4(%[in])                                \n\t"
    "lh         %[tmp_4],           6(%[in])                                \n\t"
    "sh         %[tmp_zero],        2(%[out])                               \n\t"
    "sh         %[tmp_zero],        6(%[out])                               \n\t"
    "sh         %[tmp_zero],        10(%[out])                              \n\t"
    "sh         %[tmp_zero],        14(%[out])                              \n\t"
    "addiu      %[in],              %[in],          8                       \n\t"
    "sllv       %[tmp_1],           %[tmp_1],       %[shift_size]           \n\t"
    "sllv       %[tmp_2],           %[tmp_2],       %[shift_size]           \n\t"
    "sllv       %[tmp_3],           %[tmp_3],       %[shift_size]           \n\t"
    "sllv       %[tmp_4],           %[tmp_4],       %[shift_size]           \n\t"
    "addiu      %[loop_count],      %[loop_count],  4                       \n\t"
    "sh         %[tmp_1],           0(%[out])                               \n\t"
    "sh         %[tmp_2],           4(%[out])                               \n\t"
    "sh         %[tmp_3],           8(%[out])                               \n\t"
    "sh         %[tmp_4],           12(%[out])                              \n\t"
    "blt        %[loop_count],      %[loop_size],   1b                      \n\t"
    " addiu     %[out],             %[out],         16                      \n\t"
    ".set pop                                                               \n\t"
    : [in] "+r" (in), [out] "+r" (out), [loop_count] "+r" (loop_count),
      [tmp_1] "=&r" (tmp_1), [tmp_2] "=&r" (tmp_2),
      [tmp_3] "=&r" (tmp_3), [tmp_4] "=&r" (tmp_4)
    : [loop_size] "r" (inst->anaLen), [shift_size] "r" (inst->normData), [tmp_zero] "r" (tmp_zero)
    : "memory"
  );
}

#if defined(__mips_dsp)
// Denormalize the input buffer.
__inline void Denormalize_mips(NsxInst_t* inst, int16_t* in, int factor) {

  int32_t loop_count = 0;
  int16_t tmp_1, tmp_2, tmp_3, tmp_4;
  int shift_size = factor - inst->normData;
  int16_t *real = inst->real;

  if(shift_size > 0){
    __asm__ volatile (
      ".set push                                                              \n\t"
      ".set noreorder                                                         \n\t"
      "1:                                                                     \n\t"
      "lh         %[tmp_1],           0(%[in])                                \n\t"
      "lh         %[tmp_2],           4(%[in])                                \n\t"
      "lh         %[tmp_3],           8(%[in])                                \n\t"
      "lh         %[tmp_4],           12(%[in])                               \n\t"
      "addiu      %[in],              %[in],          16                      \n\t"
      "shllv_s.ph %[tmp_1],           %[tmp_1],       %[shift_size]           \n\t"
      "shllv_s.ph %[tmp_2],           %[tmp_2],       %[shift_size]           \n\t"
      "shllv_s.ph %[tmp_3],           %[tmp_3],       %[shift_size]           \n\t"
      "shllv_s.ph %[tmp_4],           %[tmp_4],       %[shift_size]           \n\t"
      "addiu      %[loop_count],      %[loop_count],  4                       \n\t"
      "sh         %[tmp_1],           0(%[out])                               \n\t"
      "sh         %[tmp_2],           2(%[out])                               \n\t"
      "sh         %[tmp_3],           4(%[out])                               \n\t"
      "sh         %[tmp_4],           6(%[out])                               \n\t"
      "blt        %[loop_count],      %[loop_size],   1b                      \n\t"
      " addiu     %[out],             %[out],         8                       \n\t"
      ".set pop                                                               \n\t"
      : [in] "+r" (in), [out] "+r" (real), [loop_count] "+r" (loop_count),
        [tmp_1] "=&r" (tmp_1), [tmp_2] "=&r" (tmp_2),
        [tmp_3] "=&r" (tmp_3), [tmp_4] "=&r" (tmp_4)
      : [loop_size] "r" (inst->anaLen), [shift_size] "r" (shift_size)
      : "memory"
    );
  }
  else if(shift_size < 0){
    shift_size = inst->normData - factor;
    __asm__ volatile (
      ".set push                                                              \n\t"
      ".set noreorder                                                         \n\t"
      "1:                                                                     \n\t"
      "lh         %[tmp_1],           0(%[in])                                \n\t"
      "lh         %[tmp_2],           4(%[in])                                \n\t"
      "lh         %[tmp_3],           8(%[in])                                \n\t"
      "lh         %[tmp_4],           12(%[in])                               \n\t"
      "shrav.ph   %[tmp_1],           %[tmp_1],       %[shift_size]           \n\t"
      "shrav.ph   %[tmp_2],           %[tmp_2],       %[shift_size]           \n\t"
      "shrav.ph   %[tmp_3],           %[tmp_3],       %[shift_size]           \n\t"
      "shrav.ph   %[tmp_4],           %[tmp_4],       %[shift_size]           \n\t"
      "addiu      %[in],              %[in],          16                      \n\t"
      "addiu      %[loop_count],      %[loop_count],  4                       \n\t"
      "sh         %[tmp_1],           0(%[out])                               \n\t"
      "sh         %[tmp_2],           2(%[out])                               \n\t"
      "sh         %[tmp_3],           4(%[out])                               \n\t"
      "sh         %[tmp_4],           6(%[out])                               \n\t"
      "blt        %[loop_count],      %[loop_size],   1b                      \n\t"
      " addiu     %[out],             %[out],         8                       \n\t"
      ".set pop                                                               \n\t"
      : [in] "+r" (in), [out] "+r" (real), [loop_count] "+r" (loop_count),
        [tmp_1] "=&r" (tmp_1), [tmp_2] "=&r" (tmp_2),
        [tmp_3] "=&r" (tmp_3), [tmp_4] "=&r" (tmp_4)
      : [loop_size] "r" (inst->anaLen), [shift_size] "r" (shift_size)
      : "memory"
    );
  }
  else{
    __asm__ volatile (
      ".set push                                                              \n\t"
      ".set noreorder                                                         \n\t"
      "1:                                                                     \n\t"
      "lh         %[tmp_1],           0(%[in])                                \n\t"
      "lh         %[tmp_2],           4(%[in])                                \n\t"
      "lh         %[tmp_3],           8(%[in])                                \n\t"
      "lh         %[tmp_4],           12(%[in])                               \n\t"
      "addiu      %[in],              %[in],          16                      \n\t"
      "addiu      %[loop_count],      %[loop_count],  4                       \n\t"
      "sh         %[tmp_1],           0(%[out])                               \n\t"
      "sh         %[tmp_2],           2(%[out])                               \n\t"
      "sh         %[tmp_3],           4(%[out])                               \n\t"
      "sh         %[tmp_4],           6(%[out])                               \n\t"
      "blt        %[loop_count],      %[loop_size],   1b                      \n\t"
      " addiu     %[out],             %[out],         8                       \n\t"
      ".set pop                                                               \n\t"
      : [in] "+r" (in), [out] "+r" (real), [loop_count] "+r" (loop_count),
        [tmp_1] "=&r" (tmp_1), [tmp_2] "=&r" (tmp_2),
        [tmp_3] "=&r" (tmp_3), [tmp_4] "=&r" (tmp_4)
      : [loop_size] "r" (inst->anaLen)
      : "memory"
    );
  }

}
#endif //#if defined(__mips_dsp)

// Filter the data in the frequency domain, and create spectrum.
void PrepareSpectrum_mips(NsxInst_t* inst, int16_t* freq_buf) {

  WebRtc_UWord16 *noiseSupFilter = inst->noiseSupFilter;
  int16_t *real = inst->real;
  int16_t *imag = inst->imag;
  int32_t loop_count = 2;
  int16_t tmp_1, tmp_2, tmp_3, tmp_4, tmp_5, tmp_6;
  int16_t tmp16 = (inst->anaLen << 1) - 4;
  int16_t* freq_buf_f = freq_buf;
  int16_t* freq_buf_s = &freq_buf[tmp16];

  __asm__ volatile (
    ".set push                                                              \n\t"
    ".set noreorder                                                         \n\t"
//first sample
    "lh         %[tmp_1],           0(%[noiseSupFilter])                    \n\t"
    "lh         %[tmp_2],           0(%[real])                              \n\t"
    "lh         %[tmp_3],           0(%[imag])                              \n\t"
    "mul        %[tmp_2],           %[tmp_2],             %[tmp_1]          \n\t"
    "mul        %[tmp_3],           %[tmp_3],             %[tmp_1]          \n\t"
    "sra        %[tmp_2],           %[tmp_2],             14                \n\t"
    "sra        %[tmp_3],           %[tmp_3],             14                \n\t"
    "sh         %[tmp_2],           0(%[real])                              \n\t"
    "sh         %[tmp_3],           0(%[imag])                              \n\t"
    "negu       %[tmp_3],           %[tmp_3]                                \n\t"
    "sh         %[tmp_2],           0(%[freq_buf_f])                        \n\t"
    "sh         %[tmp_3],           2(%[freq_buf_f])                        \n\t"
    "addiu      %[real],            %[real],              2                 \n\t"
    "addiu      %[imag],            %[imag],              2                 \n\t"
    "addiu      %[noiseSupFilter],  %[noiseSupFilter],    2                 \n\t"
    "addiu      %[freq_buf_f],      %[freq_buf_f],        4                 \n\t"

    "1:                                                                     \n\t"
    "lh         %[tmp_1],           0(%[noiseSupFilter])                    \n\t"
    "lh         %[tmp_2],           0(%[real])                              \n\t"
    "lh         %[tmp_3],           0(%[imag])                              \n\t"
    "lh         %[tmp_4],           2(%[noiseSupFilter])                    \n\t"
    "lh         %[tmp_5],           2(%[real])                              \n\t"
    "lh         %[tmp_6],           2(%[imag])                              \n\t"
    "mul        %[tmp_2],           %[tmp_2],             %[tmp_1]          \n\t"
    "mul        %[tmp_3],           %[tmp_3],             %[tmp_1]          \n\t"
    "mul        %[tmp_5],           %[tmp_5],             %[tmp_4]          \n\t"
    "mul        %[tmp_6],           %[tmp_6],             %[tmp_4]          \n\t"
    "addiu      %[loop_count],      %[loop_count],        2                 \n\t"
    "sra        %[tmp_2],           %[tmp_2],             14                \n\t"
    "sra        %[tmp_3],           %[tmp_3],             14                \n\t"
    "sra        %[tmp_5],           %[tmp_5],             14                \n\t"
    "sra        %[tmp_6],           %[tmp_6],             14                \n\t"
    "addiu      %[noiseSupFilter],  %[noiseSupFilter],    4                 \n\t"
    "sh         %[tmp_2],           0(%[real])                              \n\t"
    "sh         %[tmp_2],           4(%[freq_buf_s])                        \n\t"
    "sh         %[tmp_3],           0(%[imag])                              \n\t"
    "sh         %[tmp_3],           6(%[freq_buf_s])                        \n\t"
    "negu       %[tmp_3],           %[tmp_3]                                \n\t"
    "sh         %[tmp_5],           2(%[real])                              \n\t"
    "sh         %[tmp_5],           0(%[freq_buf_s])                        \n\t"
    "sh         %[tmp_6],           2(%[imag])                              \n\t"
    "sh         %[tmp_6],           2(%[freq_buf_s])                        \n\t"
    "negu       %[tmp_6],           %[tmp_6]                                \n\t"
    "addiu      %[freq_buf_s],      %[freq_buf_s],        -8                \n\t"
    "addiu      %[real],            %[real],              4                 \n\t"
    "addiu      %[imag],            %[imag],              4                 \n\t"
    "sh         %[tmp_2],           0(%[freq_buf_f])                        \n\t"
    "sh         %[tmp_3],           2(%[freq_buf_f])                        \n\t"
    "sh         %[tmp_5],           4(%[freq_buf_f])                        \n\t"
    "sh         %[tmp_6],           6(%[freq_buf_f])                        \n\t"
    "blt        %[loop_count],      %[loop_size],         1b                \n\t"
    " addiu     %[freq_buf_f],      %[freq_buf_f],        8                 \n\t"

//last two samples:
    "lh         %[tmp_1],           0(%[noiseSupFilter])                    \n\t"
    "lh         %[tmp_2],           0(%[real])                              \n\t"
    "lh         %[tmp_3],           0(%[imag])                              \n\t"
    "lh         %[tmp_4],           2(%[noiseSupFilter])                    \n\t"
    "lh         %[tmp_5],           2(%[real])                              \n\t"
    "lh         %[tmp_6],           2(%[imag])                              \n\t"
    "mul        %[tmp_2],           %[tmp_2],             %[tmp_1]          \n\t"
    "mul        %[tmp_3],           %[tmp_3],             %[tmp_1]          \n\t"
    "mul        %[tmp_5],           %[tmp_5],             %[tmp_4]          \n\t"
    "mul        %[tmp_6],           %[tmp_6],             %[tmp_4]          \n\t"
    "sra        %[tmp_2],           %[tmp_2],             14                \n\t"
    "sra        %[tmp_3],           %[tmp_3],             14                \n\t"
    "sra        %[tmp_5],           %[tmp_5],             14                \n\t"
    "sra        %[tmp_6],           %[tmp_6],             14                \n\t"
    "sh         %[tmp_2],           0(%[real])                              \n\t"
    "sh         %[tmp_2],           4(%[freq_buf_s])                        \n\t"
    "sh         %[tmp_3],           0(%[imag])                              \n\t"
    "sh         %[tmp_3],           6(%[freq_buf_s])                        \n\t"
    "negu       %[tmp_3],           %[tmp_3]                                \n\t"
    "sh         %[tmp_2],           0(%[freq_buf_f])                        \n\t"
    "sh         %[tmp_3],           2(%[freq_buf_f])                        \n\t"
    "sh         %[tmp_5],           4(%[freq_buf_f])                        \n\t"
    "sh         %[tmp_6],           6(%[freq_buf_f])                        \n\t"
    "sh         %[tmp_5],           2(%[real])                              \n\t"
    "sh         %[tmp_6],           2(%[imag])                              \n\t"

    ".set pop                                                               \n\t"
    : [real] "+r" (real), [imag] "+r" (imag),
      [freq_buf_f] "+r" (freq_buf_f), [freq_buf_s] "+r" (freq_buf_s),
      [loop_count] "+r" (loop_count), [noiseSupFilter] "+r" (noiseSupFilter),
      [tmp_1] "=&r" (tmp_1), [tmp_2] "=&r" (tmp_2),
      [tmp_3] "=&r" (tmp_3), [tmp_4] "=&r" (tmp_4),
      [tmp_5] "=&r" (tmp_5), [tmp_6] "=&r" (tmp_6)
    : [loop_size] "r" (inst->anaLen2)
    : "memory", "hi", "lo"
  );

}

void WebRtcNsx_InitMips(void) {
  WebRtcNsx_SynthesisUpdate = SynthesisUpdate_mips;
  WebRtcNsx_AnalysisUpdate = AnalysisUpdate_mips;
  WebRtcNsx_UpdateNoiseEstimate = UpdateNoiseEstimate_mips;
  WebRtcNsx_NoiseEstimation = NoiseEstimation_mips;
  WebRtcNsx_CreateComplexBuffer = CreateComplexBuffer_mips;
  WebRtcNsx_PrepareSpectrum = PrepareSpectrum_mips;
#if defined(__mips_dsp)
  WebRtcNsx_Denormalize = Denormalize_mips;
#endif //#if defined(__mips_dsp)
}
