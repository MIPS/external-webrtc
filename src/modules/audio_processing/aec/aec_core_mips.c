/*
 *  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

/*
 * The core AEC algorithm, which is presented with time-aligned signals.
 */

#include "aec_core.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>  // size_t
#include <stdlib.h>
#include <string.h>

#include "aec_rdft.h"
#include "delay_estimator_wrapper.h"
#include "ring_buffer.h"
#include "typedefs.h"


// Metrics
static const int subCountLen = 4;
static const int countLen = 50;

// Quantities to control H band scaling for SWB input
static const int flagHbandCn = 1; // flag for adding comfort noise in H band
static const float cnScaleHband = (float)0.4; // scale for comfort noise in H band
// Initial bin for averaging nlp gain in low band
static const int freqAvgIc = PART_LEN / 2;

static void ComfortNoise_mips(aec_t *aec, float efw[2][PART_LEN1],
                              complex_t *comfortNoiseHband,
                              const float *noisePow, const float *lambda);

static int CmpFloat(const void *a, const void *b)
{
    const float *da = (const float *)a;
    const float *db = (const float *)b;

    return (*da > *db) - (*da < *db);
}

#if defined(MIPS32_LE) & defined(MIPS_FPU_LE)
void FilterFar_mips(aec_t *aec, float yf[2][PART_LEN1])
{
  int i;
  for (i = 0; i < NR_PART; i++) {
    int xPos = (i + aec->xfBufBlockPos) * PART_LEN1;
    int pos = i * PART_LEN1;
    // Check for wrap
    if (i + aec->xfBufBlockPos >= NR_PART) {
      xPos -= NR_PART*(PART_LEN1);
    }
    float *yf0 = yf[0];
    float *yf1 = yf[1];
    float *aRe = aec->xfBuf[0] + xPos;
    float *aIm = aec->xfBuf[1] + xPos;
    float *bRe = aec->wfBuf[0] + pos;
    float *bIm = aec->wfBuf[1] + pos;
    float f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13;

    __asm__ volatile(
        ".set push                                                      \n\t"
        ".set noreorder                                                 \n\t"
        "lwc1       %[f0],      0(%[aRe])                               \n\t"
        "lwc1       %[f1],      0(%[bRe])                               \n\t"
        "lwc1       %[f2],      0(%[bIm])                               \n\t"
        "lwc1       %[f3],      0(%[aIm])                               \n\t"
        "lwc1       %[f4],      4(%[aRe])                               \n\t"
        "lwc1       %[f5],      4(%[bRe])                               \n\t"
        "lwc1       %[f6],      4(%[bIm])                               \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      4(%[aIm])                               \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      0(%[yf0])                               \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      0(%[yf1])                               \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      4(%[yf0])                               \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      0(%[yf0])                               \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      0(%[yf1])                               \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      4(%[yf0])                               \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      4(%[yf1])                               \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      0(%[yf0])                               \n\t"
        "swc1       %[f3],      0(%[yf1])                               \n\t"
        "swc1       %[f6],      4(%[yf0])                               \n\t"
        "swc1       %[f5],      4(%[yf1])                               \n\t"

        "lwc1       %[f0],      8(%[aRe])                               \n\t"
        "lwc1       %[f1],      8(%[bRe])                               \n\t"
        "lwc1       %[f2],      8(%[bIm])                               \n\t"
        "lwc1       %[f3],      8(%[aIm])                               \n\t"
        "lwc1       %[f4],      12(%[aRe])                              \n\t"
        "lwc1       %[f5],      12(%[bRe])                              \n\t"
        "lwc1       %[f6],      12(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      12(%[aIm])                              \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      8(%[yf0])                               \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      8(%[yf1])                               \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      12(%[yf0])                              \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      8(%[yf0])                               \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      8(%[yf1])                               \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      12(%[yf0])                              \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      12(%[yf1])                              \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      8(%[yf0])                               \n\t"
        "swc1       %[f3],      8(%[yf1])                               \n\t"
        "swc1       %[f6],      12(%[yf0])                              \n\t"
        "swc1       %[f5],      12(%[yf1])                              \n\t"

        "lwc1       %[f0],      16(%[aRe])                              \n\t"
        "lwc1       %[f1],      16(%[bRe])                              \n\t"
        "lwc1       %[f2],      16(%[bIm])                              \n\t"
        "lwc1       %[f3],      16(%[aIm])                              \n\t"
        "lwc1       %[f4],      20(%[aRe])                              \n\t"
        "lwc1       %[f5],      20(%[bRe])                              \n\t"
        "lwc1       %[f6],      20(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      20(%[aIm])                              \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      16(%[yf0])                              \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      16(%[yf1])                              \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      20(%[yf0])                              \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      16(%[yf0])                              \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      16(%[yf1])                              \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      20(%[yf0])                              \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      20(%[yf1])                              \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      16(%[yf0])                              \n\t"
        "swc1       %[f3],      16(%[yf1])                              \n\t"
        "swc1       %[f6],      20(%[yf0])                              \n\t"
        "swc1       %[f5],      20(%[yf1])                              \n\t"

        "lwc1       %[f0],      24(%[aRe])                              \n\t"
        "lwc1       %[f1],      24(%[bRe])                              \n\t"
        "lwc1       %[f2],      24(%[bIm])                              \n\t"
        "lwc1       %[f3],      24(%[aIm])                              \n\t"
        "lwc1       %[f4],      28(%[aRe])                              \n\t"
        "lwc1       %[f5],      28(%[bRe])                              \n\t"
        "lwc1       %[f6],      28(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      28(%[aIm])                              \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      24(%[yf0])                              \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      24(%[yf1])                              \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      28(%[yf0])                              \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      24(%[yf0])                              \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      24(%[yf1])                              \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      28(%[yf0])                              \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      28(%[yf1])                              \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      24(%[yf0])                              \n\t"
        "swc1       %[f3],      24(%[yf1])                              \n\t"
        "swc1       %[f6],      28(%[yf0])                              \n\t"
        "swc1       %[f5],      28(%[yf1])                              \n\t"

        "lwc1       %[f0],      32(%[aRe])                              \n\t"
        "lwc1       %[f1],      32(%[bRe])                              \n\t"
        "lwc1       %[f2],      32(%[bIm])                              \n\t"
        "lwc1       %[f3],      32(%[aIm])                              \n\t"
        "lwc1       %[f4],      36(%[aRe])                              \n\t"
        "lwc1       %[f5],      36(%[bRe])                              \n\t"
        "lwc1       %[f6],      36(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      36(%[aIm])                              \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      32(%[yf0])                              \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      32(%[yf1])                              \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      36(%[yf0])                              \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      32(%[yf0])                              \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      32(%[yf1])                              \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      36(%[yf0])                              \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      36(%[yf1])                              \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      32(%[yf0])                              \n\t"
        "swc1       %[f3],      32(%[yf1])                              \n\t"
        "swc1       %[f6],      36(%[yf0])                              \n\t"
        "swc1       %[f5],      36(%[yf1])                              \n\t"

        "lwc1       %[f0],      40(%[aRe])                              \n\t"
        "lwc1       %[f1],      40(%[bRe])                              \n\t"
        "lwc1       %[f2],      40(%[bIm])                              \n\t"
        "lwc1       %[f3],      40(%[aIm])                              \n\t"
        "lwc1       %[f4],      44(%[aRe])                              \n\t"
        "lwc1       %[f5],      44(%[bRe])                              \n\t"
        "lwc1       %[f6],      44(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      44(%[aIm])                              \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      40(%[yf0])                              \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      40(%[yf1])                              \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      44(%[yf0])                              \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      40(%[yf0])                              \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      40(%[yf1])                              \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      44(%[yf0])                              \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      44(%[yf1])                              \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      40(%[yf0])                              \n\t"
        "swc1       %[f3],      40(%[yf1])                              \n\t"
        "swc1       %[f6],      44(%[yf0])                              \n\t"
        "swc1       %[f5],      44(%[yf1])                              \n\t"

        "lwc1       %[f0],      48(%[aRe])                              \n\t"
        "lwc1       %[f1],      48(%[bRe])                              \n\t"
        "lwc1       %[f2],      48(%[bIm])                              \n\t"
        "lwc1       %[f3],      48(%[aIm])                              \n\t"
        "lwc1       %[f4],      52(%[aRe])                              \n\t"
        "lwc1       %[f5],      52(%[bRe])                              \n\t"
        "lwc1       %[f6],      52(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      52(%[aIm])                              \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      48(%[yf0])                              \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      48(%[yf1])                              \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      52(%[yf0])                              \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      48(%[yf0])                              \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      48(%[yf1])                              \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      52(%[yf0])                              \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      52(%[yf1])                              \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      48(%[yf0])                              \n\t"
        "swc1       %[f3],      48(%[yf1])                              \n\t"
        "swc1       %[f6],      52(%[yf0])                              \n\t"
        "swc1       %[f5],      52(%[yf1])                              \n\t"

        "lwc1       %[f0],      56(%[aRe])                              \n\t"
        "lwc1       %[f1],      56(%[bRe])                              \n\t"
        "lwc1       %[f2],      56(%[bIm])                              \n\t"
        "lwc1       %[f3],      56(%[aIm])                              \n\t"
        "lwc1       %[f4],      60(%[aRe])                              \n\t"
        "lwc1       %[f5],      60(%[bRe])                              \n\t"
        "lwc1       %[f6],      60(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      60(%[aIm])                              \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      56(%[yf0])                              \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      56(%[yf1])                              \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      60(%[yf0])                              \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      56(%[yf0])                              \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      56(%[yf1])                              \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      60(%[yf0])                              \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      60(%[yf1])                              \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      56(%[yf0])                              \n\t"
        "swc1       %[f3],      56(%[yf1])                              \n\t"
        "swc1       %[f6],      60(%[yf0])                              \n\t"
        "swc1       %[f5],      60(%[yf1])                              \n\t"

        "lwc1       %[f0],      64(%[aRe])                              \n\t"
        "lwc1       %[f1],      64(%[bRe])                              \n\t"
        "lwc1       %[f2],      64(%[bIm])                              \n\t"
        "lwc1       %[f3],      64(%[aIm])                              \n\t"
        "lwc1       %[f4],      68(%[aRe])                              \n\t"
        "lwc1       %[f5],      68(%[bRe])                              \n\t"
        "lwc1       %[f6],      68(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      68(%[aIm])                              \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      64(%[yf0])                              \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      64(%[yf1])                              \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      68(%[yf0])                              \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      64(%[yf0])                              \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      64(%[yf1])                              \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      68(%[yf0])                              \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      68(%[yf1])                              \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      64(%[yf0])                              \n\t"
        "swc1       %[f3],      64(%[yf1])                              \n\t"
        "swc1       %[f6],      68(%[yf0])                              \n\t"
        "swc1       %[f5],      68(%[yf1])                              \n\t"

        "lwc1       %[f0],      72(%[aRe])                              \n\t"
        "lwc1       %[f1],      72(%[bRe])                              \n\t"
        "lwc1       %[f2],      72(%[bIm])                              \n\t"
        "lwc1       %[f3],      72(%[aIm])                              \n\t"
        "lwc1       %[f4],      76(%[aRe])                              \n\t"
        "lwc1       %[f5],      76(%[bRe])                              \n\t"
        "lwc1       %[f6],      76(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      76(%[aIm])                              \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      72(%[yf0])                              \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      72(%[yf1])                              \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      76(%[yf0])                              \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      72(%[yf0])                              \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      72(%[yf1])                              \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      76(%[yf0])                              \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      76(%[yf1])                              \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      72(%[yf0])                              \n\t"
        "swc1       %[f3],      72(%[yf1])                              \n\t"
        "swc1       %[f6],      76(%[yf0])                              \n\t"
        "swc1       %[f5],      76(%[yf1])                              \n\t"

        "lwc1       %[f0],      80(%[aRe])                              \n\t"
        "lwc1       %[f1],      80(%[bRe])                              \n\t"
        "lwc1       %[f2],      80(%[bIm])                              \n\t"
        "lwc1       %[f3],      80(%[aIm])                              \n\t"
        "lwc1       %[f4],      84(%[aRe])                              \n\t"
        "lwc1       %[f5],      84(%[bRe])                              \n\t"
        "lwc1       %[f6],      84(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      84(%[aIm])                              \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      80(%[yf0])                              \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      80(%[yf1])                              \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      84(%[yf0])                              \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      80(%[yf0])                              \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      80(%[yf1])                              \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      84(%[yf0])                              \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      84(%[yf1])                              \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      80(%[yf0])                              \n\t"
        "swc1       %[f3],      80(%[yf1])                              \n\t"
        "swc1       %[f6],      84(%[yf0])                              \n\t"
        "swc1       %[f5],      84(%[yf1])                              \n\t"

        "lwc1       %[f0],      88(%[aRe])                              \n\t"
        "lwc1       %[f1],      88(%[bRe])                              \n\t"
        "lwc1       %[f2],      88(%[bIm])                              \n\t"
        "lwc1       %[f3],      88(%[aIm])                              \n\t"
        "lwc1       %[f4],      92(%[aRe])                              \n\t"
        "lwc1       %[f5],      92(%[bRe])                              \n\t"
        "lwc1       %[f6],      92(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      92(%[aIm])                              \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      88(%[yf0])                              \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      88(%[yf1])                              \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      92(%[yf0])                              \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      88(%[yf0])                              \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      88(%[yf1])                              \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      92(%[yf0])                              \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      92(%[yf1])                              \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      88(%[yf0])                              \n\t"
        "swc1       %[f3],      88(%[yf1])                              \n\t"
        "swc1       %[f6],      92(%[yf0])                              \n\t"
        "swc1       %[f5],      92(%[yf1])                              \n\t"

        "lwc1       %[f0],      96(%[aRe])                              \n\t"
        "lwc1       %[f1],      96(%[bRe])                              \n\t"
        "lwc1       %[f2],      96(%[bIm])                              \n\t"
        "lwc1       %[f3],      96(%[aIm])                              \n\t"
        "lwc1       %[f4],      100(%[aRe])                             \n\t"
        "lwc1       %[f5],      100(%[bRe])                             \n\t"
        "lwc1       %[f6],      100(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      100(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      96(%[yf0])                              \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      96(%[yf1])                              \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      100(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      96(%[yf0])                              \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      96(%[yf1])                              \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      100(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      100(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      96(%[yf0])                              \n\t"
        "swc1       %[f3],      96(%[yf1])                              \n\t"
        "swc1       %[f6],      100(%[yf0])                             \n\t"
        "swc1       %[f5],      100(%[yf1])                             \n\t"

        "lwc1       %[f0],      104(%[aRe])                             \n\t"
        "lwc1       %[f1],      104(%[bRe])                             \n\t"
        "lwc1       %[f2],      104(%[bIm])                             \n\t"
        "lwc1       %[f3],      104(%[aIm])                             \n\t"
        "lwc1       %[f4],      108(%[aRe])                             \n\t"
        "lwc1       %[f5],      108(%[bRe])                             \n\t"
        "lwc1       %[f6],      108(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      108(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      104(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      104(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      108(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      104(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      104(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      108(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      108(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      104(%[yf0])                             \n\t"
        "swc1       %[f3],      104(%[yf1])                             \n\t"
        "swc1       %[f6],      108(%[yf0])                             \n\t"
        "swc1       %[f5],      108(%[yf1])                             \n\t"

        "lwc1       %[f0],      112(%[aRe])                             \n\t"
        "lwc1       %[f1],      112(%[bRe])                             \n\t"
        "lwc1       %[f2],      112(%[bIm])                             \n\t"
        "lwc1       %[f3],      112(%[aIm])                             \n\t"
        "lwc1       %[f4],      116(%[aRe])                             \n\t"
        "lwc1       %[f5],      116(%[bRe])                             \n\t"
        "lwc1       %[f6],      116(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      116(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      112(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      112(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      116(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      112(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      112(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      116(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      116(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      112(%[yf0])                             \n\t"
        "swc1       %[f3],      112(%[yf1])                             \n\t"
        "swc1       %[f6],      116(%[yf0])                             \n\t"
        "swc1       %[f5],      116(%[yf1])                             \n\t"

        "lwc1       %[f0],      120(%[aRe])                             \n\t"
        "lwc1       %[f1],      120(%[bRe])                             \n\t"
        "lwc1       %[f2],      120(%[bIm])                             \n\t"
        "lwc1       %[f3],      120(%[aIm])                             \n\t"
        "lwc1       %[f4],      124(%[aRe])                             \n\t"
        "lwc1       %[f5],      124(%[bRe])                             \n\t"
        "lwc1       %[f6],      124(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      124(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      120(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      120(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      124(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      120(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      120(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      124(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      124(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      120(%[yf0])                             \n\t"
        "swc1       %[f3],      120(%[yf1])                             \n\t"
        "swc1       %[f6],      124(%[yf0])                             \n\t"
        "swc1       %[f5],      124(%[yf1])                             \n\t"

        "lwc1       %[f0],      128(%[aRe])                             \n\t"
        "lwc1       %[f1],      128(%[bRe])                             \n\t"
        "lwc1       %[f2],      128(%[bIm])                             \n\t"
        "lwc1       %[f3],      128(%[aIm])                             \n\t"
        "lwc1       %[f4],      132(%[aRe])                             \n\t"
        "lwc1       %[f5],      132(%[bRe])                             \n\t"
        "lwc1       %[f6],      132(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      132(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      128(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      128(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      132(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      128(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      128(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      132(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      132(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      128(%[yf0])                             \n\t"
        "swc1       %[f3],      128(%[yf1])                             \n\t"
        "swc1       %[f6],      132(%[yf0])                             \n\t"
        "swc1       %[f5],      132(%[yf1])                             \n\t"

        "lwc1       %[f0],      136(%[aRe])                             \n\t"
        "lwc1       %[f1],      136(%[bRe])                             \n\t"
        "lwc1       %[f2],      136(%[bIm])                             \n\t"
        "lwc1       %[f3],      136(%[aIm])                             \n\t"
        "lwc1       %[f4],      140(%[aRe])                             \n\t"
        "lwc1       %[f5],      140(%[bRe])                             \n\t"
        "lwc1       %[f6],      140(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      140(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      136(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      136(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      140(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      136(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      136(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      140(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      140(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      136(%[yf0])                             \n\t"
        "swc1       %[f3],      136(%[yf1])                             \n\t"
        "swc1       %[f6],      140(%[yf0])                             \n\t"
        "swc1       %[f5],      140(%[yf1])                             \n\t"

        "lwc1       %[f0],      144(%[aRe])                             \n\t"
        "lwc1       %[f1],      144(%[bRe])                             \n\t"
        "lwc1       %[f2],      144(%[bIm])                             \n\t"
        "lwc1       %[f3],      144(%[aIm])                             \n\t"
        "lwc1       %[f4],      148(%[aRe])                             \n\t"
        "lwc1       %[f5],      148(%[bRe])                             \n\t"
        "lwc1       %[f6],      148(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      148(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      144(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      144(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      148(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      144(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      144(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      148(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      148(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      144(%[yf0])                             \n\t"
        "swc1       %[f3],      144(%[yf1])                             \n\t"
        "swc1       %[f6],      148(%[yf0])                             \n\t"
        "swc1       %[f5],      148(%[yf1])                             \n\t"

        "lwc1       %[f0],      152(%[aRe])                             \n\t"
        "lwc1       %[f1],      152(%[bRe])                             \n\t"
        "lwc1       %[f2],      152(%[bIm])                             \n\t"
        "lwc1       %[f3],      152(%[aIm])                             \n\t"
        "lwc1       %[f4],      156(%[aRe])                             \n\t"
        "lwc1       %[f5],      156(%[bRe])                             \n\t"
        "lwc1       %[f6],      156(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      156(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      152(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      152(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      156(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      152(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      152(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      156(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      156(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      152(%[yf0])                             \n\t"
        "swc1       %[f3],      152(%[yf1])                             \n\t"
        "swc1       %[f6],      156(%[yf0])                             \n\t"
        "swc1       %[f5],      156(%[yf1])                             \n\t"

        "lwc1       %[f0],      160(%[aRe])                             \n\t"
        "lwc1       %[f1],      160(%[bRe])                             \n\t"
        "lwc1       %[f2],      160(%[bIm])                             \n\t"
        "lwc1       %[f3],      160(%[aIm])                             \n\t"
        "lwc1       %[f4],      164(%[aRe])                             \n\t"
        "lwc1       %[f5],      164(%[bRe])                             \n\t"
        "lwc1       %[f6],      164(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      164(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      160(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      160(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      164(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      160(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      160(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      164(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      164(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      160(%[yf0])                             \n\t"
        "swc1       %[f3],      160(%[yf1])                             \n\t"
        "swc1       %[f6],      164(%[yf0])                             \n\t"
        "swc1       %[f5],      164(%[yf1])                             \n\t"

        "lwc1       %[f0],      168(%[aRe])                             \n\t"
        "lwc1       %[f1],      168(%[bRe])                             \n\t"
        "lwc1       %[f2],      168(%[bIm])                             \n\t"
        "lwc1       %[f3],      168(%[aIm])                             \n\t"
        "lwc1       %[f4],      172(%[aRe])                             \n\t"
        "lwc1       %[f5],      172(%[bRe])                             \n\t"
        "lwc1       %[f6],      172(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      172(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      168(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      168(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      172(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      168(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      168(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      172(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      172(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      168(%[yf0])                             \n\t"
        "swc1       %[f3],      168(%[yf1])                             \n\t"
        "swc1       %[f6],      172(%[yf0])                             \n\t"
        "swc1       %[f5],      172(%[yf1])                             \n\t"

        "lwc1       %[f0],      176(%[aRe])                             \n\t"
        "lwc1       %[f1],      176(%[bRe])                             \n\t"
        "lwc1       %[f2],      176(%[bIm])                             \n\t"
        "lwc1       %[f3],      176(%[aIm])                             \n\t"
        "lwc1       %[f4],      180(%[aRe])                             \n\t"
        "lwc1       %[f5],      180(%[bRe])                             \n\t"
        "lwc1       %[f6],      180(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      180(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      176(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      176(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      180(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      176(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      176(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      180(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      180(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      176(%[yf0])                             \n\t"
        "swc1       %[f3],      176(%[yf1])                             \n\t"
        "swc1       %[f6],      180(%[yf0])                             \n\t"
        "swc1       %[f5],      180(%[yf1])                             \n\t"

        "lwc1       %[f0],      184(%[aRe])                             \n\t"
        "lwc1       %[f1],      184(%[bRe])                             \n\t"
        "lwc1       %[f2],      184(%[bIm])                             \n\t"
        "lwc1       %[f3],      184(%[aIm])                             \n\t"
        "lwc1       %[f4],      188(%[aRe])                             \n\t"
        "lwc1       %[f5],      188(%[bRe])                             \n\t"
        "lwc1       %[f6],      188(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      188(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      184(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      184(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      188(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      184(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      184(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      188(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      188(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      184(%[yf0])                             \n\t"
        "swc1       %[f3],      184(%[yf1])                             \n\t"
        "swc1       %[f6],      188(%[yf0])                             \n\t"
        "swc1       %[f5],      188(%[yf1])                             \n\t"

        "lwc1       %[f0],      192(%[aRe])                             \n\t"
        "lwc1       %[f1],      192(%[bRe])                             \n\t"
        "lwc1       %[f2],      192(%[bIm])                             \n\t"
        "lwc1       %[f3],      192(%[aIm])                             \n\t"
        "lwc1       %[f4],      196(%[aRe])                             \n\t"
        "lwc1       %[f5],      196(%[bRe])                             \n\t"
        "lwc1       %[f6],      196(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      196(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      192(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      192(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      196(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      192(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      192(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      196(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      196(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      192(%[yf0])                             \n\t"
        "swc1       %[f3],      192(%[yf1])                             \n\t"
        "swc1       %[f6],      196(%[yf0])                             \n\t"
        "swc1       %[f5],      196(%[yf1])                             \n\t"

        "lwc1       %[f0],      200(%[aRe])                             \n\t"
        "lwc1       %[f1],      200(%[bRe])                             \n\t"
        "lwc1       %[f2],      200(%[bIm])                             \n\t"
        "lwc1       %[f3],      200(%[aIm])                             \n\t"
        "lwc1       %[f4],      204(%[aRe])                             \n\t"
        "lwc1       %[f5],      204(%[bRe])                             \n\t"
        "lwc1       %[f6],      204(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      204(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      200(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      200(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      204(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      200(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      200(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      204(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      204(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      200(%[yf0])                             \n\t"
        "swc1       %[f3],      200(%[yf1])                             \n\t"
        "swc1       %[f6],      204(%[yf0])                             \n\t"
        "swc1       %[f5],      204(%[yf1])                             \n\t"

        "lwc1       %[f0],      208(%[aRe])                             \n\t"
        "lwc1       %[f1],      208(%[bRe])                             \n\t"
        "lwc1       %[f2],      208(%[bIm])                             \n\t"
        "lwc1       %[f3],      208(%[aIm])                             \n\t"
        "lwc1       %[f4],      212(%[aRe])                             \n\t"
        "lwc1       %[f5],      212(%[bRe])                             \n\t"
        "lwc1       %[f6],      212(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      212(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      208(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      208(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      212(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      208(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      208(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      212(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      212(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      208(%[yf0])                             \n\t"
        "swc1       %[f3],      208(%[yf1])                             \n\t"
        "swc1       %[f6],      212(%[yf0])                             \n\t"
        "swc1       %[f5],      212(%[yf1])                             \n\t"

        "lwc1       %[f0],      216(%[aRe])                             \n\t"
        "lwc1       %[f1],      216(%[bRe])                             \n\t"
        "lwc1       %[f2],      216(%[bIm])                             \n\t"
        "lwc1       %[f3],      216(%[aIm])                             \n\t"
        "lwc1       %[f4],      220(%[aRe])                             \n\t"
        "lwc1       %[f5],      220(%[bRe])                             \n\t"
        "lwc1       %[f6],      220(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      220(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      216(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      216(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      220(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      216(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      216(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      220(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      220(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      216(%[yf0])                             \n\t"
        "swc1       %[f3],      216(%[yf1])                             \n\t"
        "swc1       %[f6],      220(%[yf0])                             \n\t"
        "swc1       %[f5],      220(%[yf1])                             \n\t"

        "lwc1       %[f0],      224(%[aRe])                             \n\t"
        "lwc1       %[f1],      224(%[bRe])                             \n\t"
        "lwc1       %[f2],      224(%[bIm])                             \n\t"
        "lwc1       %[f3],      224(%[aIm])                             \n\t"
        "lwc1       %[f4],      228(%[aRe])                             \n\t"
        "lwc1       %[f5],      228(%[bRe])                             \n\t"
        "lwc1       %[f6],      228(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      228(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      224(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      224(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      228(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      224(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      224(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      228(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      228(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      224(%[yf0])                             \n\t"
        "swc1       %[f3],      224(%[yf1])                             \n\t"
        "swc1       %[f6],      228(%[yf0])                             \n\t"
        "swc1       %[f5],      228(%[yf1])                             \n\t"

        "lwc1       %[f0],      232(%[aRe])                             \n\t"
        "lwc1       %[f1],      232(%[bRe])                             \n\t"
        "lwc1       %[f2],      232(%[bIm])                             \n\t"
        "lwc1       %[f3],      232(%[aIm])                             \n\t"
        "lwc1       %[f4],      236(%[aRe])                             \n\t"
        "lwc1       %[f5],      236(%[bRe])                             \n\t"
        "lwc1       %[f6],      236(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      236(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      232(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      232(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      236(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      232(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      232(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      236(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      236(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      232(%[yf0])                             \n\t"
        "swc1       %[f3],      232(%[yf1])                             \n\t"
        "swc1       %[f6],      236(%[yf0])                             \n\t"
        "swc1       %[f5],      236(%[yf1])                             \n\t"

        "lwc1       %[f0],      240(%[aRe])                             \n\t"
        "lwc1       %[f1],      240(%[bRe])                             \n\t"
        "lwc1       %[f2],      240(%[bIm])                             \n\t"
        "lwc1       %[f3],      240(%[aIm])                             \n\t"
        "lwc1       %[f4],      244(%[aRe])                             \n\t"
        "lwc1       %[f5],      244(%[bRe])                             \n\t"
        "lwc1       %[f6],      244(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      244(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "lwc1       %[f2],      240(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      240(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f6],      244(%[yf0])                             \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      240(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      240(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
        "lwc1       %[f6],      244(%[yf0])                             \n\t"
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "lwc1       %[f5],      244(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "swc1       %[f2],      240(%[yf0])                             \n\t"
        "swc1       %[f3],      240(%[yf1])                             \n\t"
        "swc1       %[f6],      244(%[yf0])                             \n\t"
        "swc1       %[f5],      244(%[yf1])                             \n\t"

        "lwc1       %[f0],      248(%[aRe])                             \n\t"
        "lwc1       %[f1],      248(%[bRe])                             \n\t"
        "lwc1       %[f2],      248(%[bIm])                             \n\t"
        "lwc1       %[f3],      248(%[aIm])                             \n\t"
        "lwc1       %[f4],      252(%[aRe])                             \n\t"
        "lwc1       %[f5],      252(%[bRe])                             \n\t"
        "lwc1       %[f6],      252(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
        "lwc1       %[f7],      252(%[aIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f2],          %[f3]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f6],          %[f7]                   \n\t"
        "sub.s      %[f8],      %[f8],          %[f12]                  \n\t"
        "lwc1       %[f2],      248(%[yf0])                             \n\t"
        "add.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "lwc1       %[f3],      248(%[yf1])                             \n\t"
        "sub.s      %[f9],      %[f9],          %[f11]                  \n\t"
#else
        "nmsub.s    %[f8],      %[f8],          %[f2],      %[f3]       \n\t"
        "lwc1       %[f2],      248(%[yf0])                             \n\t"
        "madd.s     %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "lwc1       %[f3],      248(%[yf1])                             \n\t"
        "nmsub.s    %[f9],      %[f9],          %[f6],      %[f7]       \n\t"
#endif
        "lwc1       %[f10],     256(%[aRe])                             \n\t"
        "lwc1       %[f11],     256(%[bRe])                             \n\t"
        "lwc1       %[f12],     256(%[bIm])                             \n\t"
        "lwc1       %[f13],     256(%[aIm])                             \n\t"
        "lwc1       %[f6],      252(%[yf0])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f7],          %[f5]                   \n\t"
        "add.s      %[f4],      %[f4],          %[f12]                  \n\t"
#else
        "madd.s     %[f4],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "mul.s      %[f7],      %[f10],         %[f11]                  \n\t"
        "mul.s      %[f10],     %[f10],         %[f12]                  \n\t"

        "lwc1       %[f5],      252(%[yf1])                             \n\t"
        "add.s      %[f2],      %[f2],          %[f8]                   \n\t"
        "add.s      %[f3],      %[f3],          %[f1]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f12],         %[f13]                  \n\t"
        "mul.s      %[f1],      %[f13],         %[f10]                  \n\t"
        "sub.s      %[f7],      %[f7],          %[f12]                  \n\t"
        "lwc1       %[f12],     256(%[yf0])                             \n\t"
        "add.s      %[f10],     %[f10],         %[f1]                   \n\t"
#else
        "nmsub.s    %[f7],      %[f7],          %[f12],     %[f13]      \n\t"
        "lwc1       %[f12],     256(%[yf0])                             \n\t"
        "madd.s     %[f10],     %[f10],         %[f13],     %[f11]      \n\t"
#endif
        "lwc1       %[f11],     256(%[yf1])                             \n\t"

        "add.s      %[f6],      %[f6],          %[f9]                   \n\t"
        "add.s      %[f5],      %[f5],          %[f4]                   \n\t"
        "add.s      %[f7],      %[f7],          %[f12]                  \n\t"
        "add.s      %[f10],     %[f10],         %[f11]                  \n\t"
        "swc1       %[f2],      248(%[yf0])                             \n\t"
        "swc1       %[f3],      248(%[yf1])                             \n\t"
        "swc1       %[f6],      252(%[yf0])                             \n\t"
        "swc1       %[f5],      252(%[yf1])                             \n\t"
        "swc1       %[f7],      256(%[yf0])                             \n\t"
        "swc1       %[f10],     256(%[yf1])                             \n\t"

        ".set pop                                                       \n\t"
        : [f0] "=&f" (f0), [f1] "=&f" (f1), [f2] "=&f" (f2),
          [f3] "=&f" (f3), [f4] "=&f" (f4), [f5] "=&f" (f5),
          [f6] "=&f" (f6), [f7] "=&f" (f7), [f8] "=&f" (f8),
          [f9] "=&f" (f9), [f10] "=&f" (f10), [f11] "=&f" (f11),
          [f12] "=&f" (f12), [f13] "=&f" (f13)
        : [aRe] "r" (aRe), [aIm] "r" (aIm), [bRe] "r" (bRe),
          [bIm] "r" (bIm), [yf0] "r" (yf0), [yf1] "r" (yf1)
        : "memory"
    );
  }
}


void FilterAdaptation_mips(aec_t *aec, float *fft, float ef[2][PART_LEN1]) {
  int i;
  for (i = 0; i < NR_PART; i++) {
    int xPos = (i + aec->xfBufBlockPos)*(PART_LEN1);
    int pos;
    // Check for wrap
    if (i + aec->xfBufBlockPos >= NR_PART) {
      xPos -= NR_PART * PART_LEN1;
    }

    pos = i * PART_LEN1;
    float *aRe = aec->xfBuf[0] + xPos;
    float *aIm = aec->xfBuf[1] + xPos;
    float *bRe = ef[0];
    float *bIm = ef[1];

    float f0, f1, f2, f3, f4, f5, f6 ,f7, f8, f9, f10, f11, f12;

    __asm__ volatile(
        ".set push                                                      \n\t"
        ".set noreorder                                                 \n\t"
        "lwc1       %[f0],      0(%[aRe])                               \n\t"
        "lwc1       %[f1],      0(%[bRe])                               \n\t"
        "lwc1       %[f2],      0(%[bIm])                               \n\t"
        "lwc1       %[f4],      4(%[aRe])                               \n\t"
        "lwc1       %[f5],      4(%[bRe])                               \n\t"
        "lwc1       %[f6],      4(%[bIm])                               \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      0(%[aIm])                               \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      4(%[aIm])                               \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      8(%[aRe])                               \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      8(%[aRe])                               \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      0(%[fft])                               \n\t"
        "swc1       %[f1],      4(%[fft])                               \n\t"
        "swc1       %[f9],      8(%[fft])                               \n\t"
        "lwc1       %[f1],      8(%[bRe])                               \n\t"
        "swc1       %[f5],      12(%[fft])                              \n\t"

        "lwc1       %[f2],      8(%[bIm])                               \n\t"
        "lwc1       %[f4],      12(%[aRe])                              \n\t"
        "lwc1       %[f5],      12(%[bRe])                              \n\t"
        "lwc1       %[f6],      12(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      8(%[aIm])                               \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      12(%[aIm])                              \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      16(%[aRe])                              \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      16(%[aRe])                              \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      16(%[fft])                              \n\t"
        "swc1       %[f1],      20(%[fft])                              \n\t"
        "swc1       %[f9],      24(%[fft])                              \n\t"
        "lwc1       %[f1],      16(%[bRe])                              \n\t"
        "swc1       %[f5],      28(%[fft])                              \n\t"

        "lwc1       %[f2],      16(%[bIm])                              \n\t"
        "lwc1       %[f4],      20(%[aRe])                              \n\t"
        "lwc1       %[f5],      20(%[bRe])                              \n\t"
        "lwc1       %[f6],      20(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      16(%[aIm])                              \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      20(%[aIm])                              \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      24(%[aRe])                              \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      24(%[aRe])                              \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      32(%[fft])                              \n\t"
        "swc1       %[f1],      36(%[fft])                              \n\t"
        "swc1       %[f9],      40(%[fft])                              \n\t"
        "lwc1       %[f1],      24(%[bRe])                              \n\t"
        "swc1       %[f5],      44(%[fft])                              \n\t"

        "lwc1       %[f2],      24(%[bIm])                              \n\t"
        "lwc1       %[f4],      28(%[aRe])                              \n\t"
        "lwc1       %[f5],      28(%[bRe])                              \n\t"
        "lwc1       %[f6],      28(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      24(%[aIm])                              \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      28(%[aIm])                              \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      32(%[aRe])                              \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      32(%[aRe])                              \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      48(%[fft])                              \n\t"
        "swc1       %[f1],      52(%[fft])                              \n\t"
        "swc1       %[f9],      56(%[fft])                              \n\t"
        "lwc1       %[f1],      32(%[bRe])                              \n\t"
        "swc1       %[f5],      60(%[fft])                              \n\t"

        "lwc1       %[f2],      32(%[bIm])                              \n\t"
        "lwc1       %[f4],      36(%[aRe])                              \n\t"
        "lwc1       %[f5],      36(%[bRe])                              \n\t"
        "lwc1       %[f6],      36(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      32(%[aIm])                              \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      36(%[aIm])                              \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      40(%[aRe])                              \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      40(%[aRe])                              \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      64(%[fft])                              \n\t"
        "swc1       %[f1],      68(%[fft])                              \n\t"
        "swc1       %[f9],      72(%[fft])                              \n\t"
        "lwc1       %[f1],      40(%[bRe])                              \n\t"
        "swc1       %[f5],      76(%[fft])                              \n\t"

        "lwc1       %[f2],      40(%[bIm])                              \n\t"
        "lwc1       %[f4],      44(%[aRe])                              \n\t"
        "lwc1       %[f5],      44(%[bRe])                              \n\t"
        "lwc1       %[f6],      44(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      40(%[aIm])                              \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      44(%[aIm])                              \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      48(%[aRe])                              \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      48(%[aRe])                              \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      80(%[fft])                              \n\t"
        "swc1       %[f1],      84(%[fft])                              \n\t"
        "swc1       %[f9],      88(%[fft])                              \n\t"
        "lwc1       %[f1],      48(%[bRe])                              \n\t"
        "swc1       %[f5],      92(%[fft])                              \n\t"

        "lwc1       %[f2],      48(%[bIm])                              \n\t"
        "lwc1       %[f4],      52(%[aRe])                              \n\t"
        "lwc1       %[f5],      52(%[bRe])                              \n\t"
        "lwc1       %[f6],      52(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      48(%[aIm])                              \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      52(%[aIm])                              \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      56(%[aRe])                              \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      56(%[aRe])                              \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      96(%[fft])                              \n\t"
        "swc1       %[f1],      100(%[fft])                             \n\t"
        "swc1       %[f9],      104(%[fft])                             \n\t"
        "lwc1       %[f1],      56(%[bRe])                              \n\t"
        "swc1       %[f5],      108(%[fft])                             \n\t"

        "lwc1       %[f2],      56(%[bIm])                              \n\t"
        "lwc1       %[f4],      60(%[aRe])                              \n\t"
        "lwc1       %[f5],      60(%[bRe])                              \n\t"
        "lwc1       %[f6],      60(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      56(%[aIm])                              \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      60(%[aIm])                              \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      64(%[aRe])                              \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      64(%[aRe])                              \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      112(%[fft])                             \n\t"
        "swc1       %[f1],      116(%[fft])                             \n\t"
        "swc1       %[f9],      120(%[fft])                             \n\t"
        "lwc1       %[f1],      64(%[bRe])                              \n\t"
        "swc1       %[f5],      124(%[fft])                             \n\t"

        "lwc1       %[f2],      64(%[bIm])                              \n\t"
        "lwc1       %[f4],      68(%[aRe])                              \n\t"
        "lwc1       %[f5],      68(%[bRe])                              \n\t"
        "lwc1       %[f6],      68(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      64(%[aIm])                              \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      68(%[aIm])                              \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      72(%[aRe])                              \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      72(%[aRe])                              \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      128(%[fft])                             \n\t"
        "swc1       %[f1],      132(%[fft])                             \n\t"
        "swc1       %[f9],      136(%[fft])                             \n\t"
        "lwc1       %[f1],      72(%[bRe])                              \n\t"
        "swc1       %[f5],      140(%[fft])                             \n\t"

        "lwc1       %[f2],      72(%[bIm])                              \n\t"
        "lwc1       %[f4],      76(%[aRe])                              \n\t"
        "lwc1       %[f5],      76(%[bRe])                              \n\t"
        "lwc1       %[f6],      76(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      72(%[aIm])                              \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      76(%[aIm])                              \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      80(%[aRe])                              \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      80(%[aRe])                              \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      144(%[fft])                             \n\t"
        "swc1       %[f1],      148(%[fft])                             \n\t"
        "swc1       %[f9],      152(%[fft])                             \n\t"
        "lwc1       %[f1],      80(%[bRe])                              \n\t"
        "swc1       %[f5],      156(%[fft])                             \n\t"

        "lwc1       %[f2],      80(%[bIm])                              \n\t"
        "lwc1       %[f4],      84(%[aRe])                              \n\t"
        "lwc1       %[f5],      84(%[bRe])                              \n\t"
        "lwc1       %[f6],      84(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      80(%[aIm])                              \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      84(%[aIm])                              \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      88(%[aRe])                              \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      88(%[aRe])                              \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      160(%[fft])                             \n\t"
        "swc1       %[f1],      164(%[fft])                             \n\t"
        "swc1       %[f9],      168(%[fft])                             \n\t"
        "lwc1       %[f1],      88(%[bRe])                              \n\t"
        "swc1       %[f5],      172(%[fft])                             \n\t"

        "lwc1       %[f2],      88(%[bIm])                              \n\t"
        "lwc1       %[f4],      92(%[aRe])                              \n\t"
        "lwc1       %[f5],      92(%[bRe])                              \n\t"
        "lwc1       %[f6],      92(%[bIm])                              \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      88(%[aIm])                              \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      92(%[aIm])                              \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      96(%[aRe])                              \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      96(%[aRe])                              \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      176(%[fft])                             \n\t"
        "swc1       %[f1],      180(%[fft])                             \n\t"
        "swc1       %[f9],      184(%[fft])                             \n\t"
        "lwc1       %[f1],      96(%[bRe])                              \n\t"
        "swc1       %[f5],      188(%[fft])                             \n\t"

        "lwc1       %[f2],      96(%[bIm])                              \n\t"
        "lwc1       %[f4],      100(%[aRe])                             \n\t"
        "lwc1       %[f5],      100(%[bRe])                             \n\t"
        "lwc1       %[f6],      100(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      96(%[aIm])                              \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      100(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      104(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      104(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      192(%[fft])                             \n\t"
        "swc1       %[f1],      196(%[fft])                             \n\t"
        "swc1       %[f9],      200(%[fft])                             \n\t"
        "lwc1       %[f1],      104(%[bRe])                             \n\t"
        "swc1       %[f5],      204(%[fft])                             \n\t"

        "lwc1       %[f2],      104(%[bIm])                             \n\t"
        "lwc1       %[f4],      108(%[aRe])                             \n\t"
        "lwc1       %[f5],      108(%[bRe])                             \n\t"
        "lwc1       %[f6],      108(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      104(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      108(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      112(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      112(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      208(%[fft])                             \n\t"
        "swc1       %[f1],      212(%[fft])                             \n\t"
        "swc1       %[f9],      216(%[fft])                             \n\t"
        "lwc1       %[f1],      112(%[bRe])                             \n\t"
        "swc1       %[f5],      220(%[fft])                             \n\t"

        "lwc1       %[f2],      112(%[bIm])                             \n\t"
        "lwc1       %[f4],      116(%[aRe])                             \n\t"
        "lwc1       %[f5],      116(%[bRe])                             \n\t"
        "lwc1       %[f6],      116(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      112(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      116(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      120(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      120(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      224(%[fft])                             \n\t"
        "swc1       %[f1],      228(%[fft])                             \n\t"
        "swc1       %[f9],      232(%[fft])                             \n\t"
        "lwc1       %[f1],      120(%[bRe])                             \n\t"
        "swc1       %[f5],      236(%[fft])                             \n\t"

        "lwc1       %[f2],      120(%[bIm])                             \n\t"
        "lwc1       %[f4],      124(%[aRe])                             \n\t"
        "lwc1       %[f5],      124(%[bRe])                             \n\t"
        "lwc1       %[f6],      124(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      120(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      124(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      128(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      128(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      240(%[fft])                             \n\t"
        "swc1       %[f1],      244(%[fft])                             \n\t"
        "swc1       %[f9],      248(%[fft])                             \n\t"
        "lwc1       %[f1],      128(%[bRe])                             \n\t"
        "swc1       %[f5],      252(%[fft])                             \n\t"

        "lwc1       %[f2],      128(%[bIm])                             \n\t"
        "lwc1       %[f4],      132(%[aRe])                             \n\t"
        "lwc1       %[f5],      132(%[bRe])                             \n\t"
        "lwc1       %[f6],      132(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      128(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      132(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      136(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      136(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      256(%[fft])                             \n\t"
        "swc1       %[f1],      260(%[fft])                             \n\t"
        "swc1       %[f9],      264(%[fft])                             \n\t"
        "lwc1       %[f1],      136(%[bRe])                             \n\t"
        "swc1       %[f5],      268(%[fft])                             \n\t"

        "lwc1       %[f2],      136(%[bIm])                             \n\t"
        "lwc1       %[f4],      140(%[aRe])                             \n\t"
        "lwc1       %[f5],      140(%[bRe])                             \n\t"
        "lwc1       %[f6],      140(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      136(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      140(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      144(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      144(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      272(%[fft])                             \n\t"
        "swc1       %[f1],      276(%[fft])                             \n\t"
        "swc1       %[f9],      280(%[fft])                             \n\t"
        "lwc1       %[f1],      144(%[bRe])                             \n\t"
        "swc1       %[f5],      284(%[fft])                             \n\t"

        "lwc1       %[f2],      144(%[bIm])                             \n\t"
        "lwc1       %[f4],      148(%[aRe])                             \n\t"
        "lwc1       %[f5],      148(%[bRe])                             \n\t"
        "lwc1       %[f6],      148(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      144(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      148(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      152(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      152(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      288(%[fft])                             \n\t"
        "swc1       %[f1],      292(%[fft])                             \n\t"
        "swc1       %[f9],      296(%[fft])                             \n\t"
        "lwc1       %[f1],      152(%[bRe])                             \n\t"
        "swc1       %[f5],      300(%[fft])                             \n\t"

        "lwc1       %[f2],      152(%[bIm])                             \n\t"
        "lwc1       %[f4],      156(%[aRe])                             \n\t"
        "lwc1       %[f5],      156(%[bRe])                             \n\t"
        "lwc1       %[f6],      156(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      152(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      156(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      160(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      160(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      304(%[fft])                             \n\t"
        "swc1       %[f1],      308(%[fft])                             \n\t"
        "swc1       %[f9],      312(%[fft])                             \n\t"
        "lwc1       %[f1],      160(%[bRe])                             \n\t"
        "swc1       %[f5],      316(%[fft])                             \n\t"

        "lwc1       %[f2],      160(%[bIm])                             \n\t"
        "lwc1       %[f4],      164(%[aRe])                             \n\t"
        "lwc1       %[f5],      164(%[bRe])                             \n\t"
        "lwc1       %[f6],      164(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      160(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      164(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      168(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      168(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      320(%[fft])                             \n\t"
        "swc1       %[f1],      324(%[fft])                             \n\t"
        "swc1       %[f9],      328(%[fft])                             \n\t"
        "lwc1       %[f1],      168(%[bRe])                             \n\t"
        "swc1       %[f5],      332(%[fft])                             \n\t"

        "lwc1       %[f2],      168(%[bIm])                             \n\t"
        "lwc1       %[f4],      172(%[aRe])                             \n\t"
        "lwc1       %[f5],      172(%[bRe])                             \n\t"
        "lwc1       %[f6],      172(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      168(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      172(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      176(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      176(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      336(%[fft])                             \n\t"
        "swc1       %[f1],      340(%[fft])                             \n\t"
        "swc1       %[f9],      344(%[fft])                             \n\t"
        "lwc1       %[f1],      176(%[bRe])                             \n\t"
        "swc1       %[f5],      348(%[fft])                             \n\t"

        "lwc1       %[f2],      176(%[bIm])                             \n\t"
        "lwc1       %[f4],      180(%[aRe])                             \n\t"
        "lwc1       %[f5],      180(%[bRe])                             \n\t"
        "lwc1       %[f6],      180(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      176(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      180(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      184(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      184(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      352(%[fft])                             \n\t"
        "swc1       %[f1],      356(%[fft])                             \n\t"
        "swc1       %[f9],      360(%[fft])                             \n\t"
        "lwc1       %[f1],      184(%[bRe])                             \n\t"
        "swc1       %[f5],      364(%[fft])                             \n\t"

        "lwc1       %[f2],      184(%[bIm])                             \n\t"
        "lwc1       %[f4],      188(%[aRe])                             \n\t"
        "lwc1       %[f5],      188(%[bRe])                             \n\t"
        "lwc1       %[f6],      188(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      184(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      188(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      192(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      192(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      368(%[fft])                             \n\t"
        "swc1       %[f1],      372(%[fft])                             \n\t"
        "swc1       %[f9],      376(%[fft])                             \n\t"
        "lwc1       %[f1],      192(%[bRe])                             \n\t"
        "swc1       %[f5],      380(%[fft])                             \n\t"

        "lwc1       %[f2],      192(%[bIm])                             \n\t"
        "lwc1       %[f4],      196(%[aRe])                             \n\t"
        "lwc1       %[f5],      196(%[bRe])                             \n\t"
        "lwc1       %[f6],      196(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      192(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      196(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      200(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      200(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      384(%[fft])                             \n\t"
        "swc1       %[f1],      388(%[fft])                             \n\t"
        "swc1       %[f9],      392(%[fft])                             \n\t"
        "lwc1       %[f1],      200(%[bRe])                             \n\t"
        "swc1       %[f5],      396(%[fft])                             \n\t"

        "lwc1       %[f2],      200(%[bIm])                             \n\t"
        "lwc1       %[f4],      204(%[aRe])                             \n\t"
        "lwc1       %[f5],      204(%[bRe])                             \n\t"
        "lwc1       %[f6],      204(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      200(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      204(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      208(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      208(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      400(%[fft])                             \n\t"
        "swc1       %[f1],      404(%[fft])                             \n\t"
        "swc1       %[f9],      408(%[fft])                             \n\t"
        "lwc1       %[f1],      208(%[bRe])                             \n\t"
        "swc1       %[f5],      412(%[fft])                             \n\t"

        "lwc1       %[f2],      208(%[bIm])                             \n\t"
        "lwc1       %[f4],      212(%[aRe])                             \n\t"
        "lwc1       %[f5],      212(%[bRe])                             \n\t"
        "lwc1       %[f6],      212(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      208(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      212(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      216(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      216(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      416(%[fft])                             \n\t"
        "swc1       %[f1],      420(%[fft])                             \n\t"
        "swc1       %[f9],      424(%[fft])                             \n\t"
        "lwc1       %[f1],      216(%[bRe])                             \n\t"
        "swc1       %[f5],      428(%[fft])                             \n\t"

        "lwc1       %[f2],      216(%[bIm])                             \n\t"
        "lwc1       %[f4],      220(%[aRe])                             \n\t"
        "lwc1       %[f5],      220(%[bRe])                             \n\t"
        "lwc1       %[f6],      220(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      216(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      220(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      224(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      224(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      432(%[fft])                             \n\t"
        "swc1       %[f1],      436(%[fft])                             \n\t"
        "swc1       %[f9],      440(%[fft])                             \n\t"
        "lwc1       %[f1],      224(%[bRe])                             \n\t"
        "swc1       %[f5],      444(%[fft])                             \n\t"

        "lwc1       %[f2],      224(%[bIm])                             \n\t"
        "lwc1       %[f4],      228(%[aRe])                             \n\t"
        "lwc1       %[f5],      228(%[bRe])                             \n\t"
        "lwc1       %[f6],      228(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      224(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      228(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      232(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      232(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      448(%[fft])                             \n\t"
        "swc1       %[f1],      452(%[fft])                             \n\t"
        "swc1       %[f9],      456(%[fft])                             \n\t"
        "lwc1       %[f1],      232(%[bRe])                             \n\t"
        "swc1       %[f5],      460(%[fft])                             \n\t"

        "lwc1       %[f2],      232(%[bIm])                             \n\t"
        "lwc1       %[f4],      236(%[aRe])                             \n\t"
        "lwc1       %[f5],      236(%[bRe])                             \n\t"
        "lwc1       %[f6],      236(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      232(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      236(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      240(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      240(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      464(%[fft])                             \n\t"
        "swc1       %[f1],      468(%[fft])                             \n\t"
        "swc1       %[f9],      472(%[fft])                             \n\t"
        "lwc1       %[f1],      240(%[bRe])                             \n\t"
        "swc1       %[f5],      476(%[fft])                             \n\t"

        "lwc1       %[f2],      240(%[bIm])                             \n\t"
        "lwc1       %[f4],      244(%[aRe])                             \n\t"
        "lwc1       %[f5],      244(%[bRe])                             \n\t"
        "lwc1       %[f6],      244(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      240(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f7],      244(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f10],     %[f3],          %[f2]                   \n\t"
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f11],     %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f10]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f11]                  \n\t"
        "lwc1       %[f0],      248(%[aRe])                             \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "lwc1       %[f0],      248(%[aRe])                             \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
#endif
        "swc1       %[f8],      480(%[fft])                             \n\t"
        "swc1       %[f1],      484(%[fft])                             \n\t"
        "swc1       %[f9],      488(%[fft])                             \n\t"
        "lwc1       %[f1],      248(%[bRe])                             \n\t"
        "swc1       %[f5],      492(%[fft])                             \n\t"

        "lwc1       %[f2],      248(%[bIm])                             \n\t"
        "lwc1       %[f4],      252(%[aRe])                             \n\t"
        "lwc1       %[f5],      252(%[bRe])                             \n\t"
        "lwc1       %[f6],      252(%[bIm])                             \n\t"
        "mul.s      %[f8],      %[f0],          %[f1]                   \n\t"
        "mul.s      %[f0],      %[f0],          %[f2]                   \n\t"
        "lwc1       %[f3],      248(%[aIm])                             \n\t"
        "mul.s      %[f9],      %[f4],          %[f5]                   \n\t"
        "lwc1       %[f10],     256(%[aRe])                             \n\t"
        "lwc1       %[f11],     256(%[bRe])                             \n\t"
        "lwc1       %[f7],      252(%[aIm])                             \n\t"
        "mul.s      %[f4],      %[f4],          %[f6]                   \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f12],     %[f3],          %[f2]                   \n\t"
        "add.s      %[f8],      %[f8],          %[f12]                  \n\t"
#else
        "madd.s     %[f8],      %[f8],          %[f3],      %[f2]       \n\t"
#endif
        "mul.s      %[f10],     %[f10],         %[f11]                  \n\t"
        "lwc1       %[f2],      256(%[aIm])                             \n\t"
        "lwc1       %[f11],     256(%[bIm])                             \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s      %[f1],      %[f3],          %[f1]                   \n\t"
        "mul.s      %[f3],      %[f7],          %[f6]                   \n\t"
        "mul.s      %[f5],      %[f7],          %[f5]                   \n\t"
        "mul.s      %[f11],     %[f2],          %[f11]                  \n\t"
        "sub.s      %[f1],      %[f0],          %[f1]                   \n\t"
        "add.s      %[f9],      %[f9],          %[f3]                   \n\t"
        "sub.s      %[f5],      %[f4],          %[f5]                   \n\t"
        "add.s      %[f10],     %[f10],         %[f11]                  \n\t"
#else
        "nmsub.s    %[f1],      %[f0],          %[f3],      %[f1]       \n\t"
        "madd.s     %[f9],      %[f9],          %[f7],      %[f6]       \n\t"
        "nmsub.s    %[f5],      %[f4],          %[f7],      %[f5]       \n\t"
        "madd.s     %[f10],     %[f10],         %[f2],      %[f11]      \n\t"
#endif
        "swc1       %[f8],      496(%[fft])                             \n\t"
        "swc1       %[f1],      500(%[fft])                             \n\t"
        "swc1       %[f9],      504(%[fft])                             \n\t"
        "swc1       %[f5],      508(%[fft])                             \n\t"
        "swc1       %[f10],     4(%[fft])                               \n\t"
        ".set pop                                                       \n\t"
        : [f0] "=&f" (f0), [f1] "=&f" (f1), [f2] "=&f" (f2),
          [f3] "=&f" (f3), [f4] "=&f" (f4), [f5] "=&f" (f5),
          [f6] "=&f" (f6), [f7] "=&f" (f7), [f8] "=&f" (f8),
          [f9] "=&f" (f9), [f10] "=&f" (f10), [f11] "=&f" (f11),
          [f12] "=&f" (f12)
        : [aRe] "r" (aRe), [aIm] "r" (aIm), [bRe] "r" (bRe),
          [bIm] "r" (bIm), [fft] "r" (fft)
        : "memory"
    );
    aec_rdft_inverse_128(fft);
    memset(fft + PART_LEN, 0, sizeof(float) * PART_LEN);

    // fft scaling
    {
      float scale = 2.0f / PART_LEN2;
      __asm__ volatile(
          ".set push                                        \n\t"
          ".set noreorder                                   \n\t"
          "lwc1     %[f0],  0(%[fft])                       \n\t"
          "lwc1     %[f1],  4(%[fft])                       \n\t"
          "lwc1     %[f2],  8(%[fft])                       \n\t"
          "lwc1     %[f3],  12(%[fft])                      \n\t"
          "mul.s    %[f0],  %[f0],      %[scale]            \n\t"
          "mul.s    %[f1],  %[f1],      %[scale]            \n\t"
          "mul.s    %[f2],  %[f2],      %[scale]            \n\t"
          "mul.s    %[f3],  %[f3],      %[scale]            \n\t"
          "lwc1     %[f4],  16(%[fft])                      \n\t"
          "lwc1     %[f5],  20(%[fft])                      \n\t"
          "lwc1     %[f6],  24(%[fft])                      \n\t"
          "lwc1     %[f7],  28(%[fft])                      \n\t"
          "mul.s    %[f4],  %[f4],      %[scale]            \n\t"
          "mul.s    %[f5],  %[f5],      %[scale]            \n\t"
          "mul.s    %[f6],  %[f6],      %[scale]            \n\t"
          "mul.s    %[f7],  %[f7],      %[scale]            \n\t"
          "swc1     %[f0],  0(%[fft])                       \n\t"
          "swc1     %[f1],  4(%[fft])                       \n\t"
          "swc1     %[f2],  8(%[fft])                       \n\t"
          "swc1     %[f3],  12(%[fft])                      \n\t"
          "swc1     %[f4],  16(%[fft])                      \n\t"
          "swc1     %[f5],  20(%[fft])                      \n\t"
          "swc1     %[f6],  24(%[fft])                      \n\t"
          "swc1     %[f7],  28(%[fft])                      \n\t"

          "lwc1     %[f0],  32(%[fft])                      \n\t"
          "lwc1     %[f1],  36(%[fft])                      \n\t"
          "lwc1     %[f2],  40(%[fft])                      \n\t"
          "lwc1     %[f3],  44(%[fft])                      \n\t"
          "mul.s    %[f0],  %[f0],      %[scale]            \n\t"
          "mul.s    %[f1],  %[f1],      %[scale]            \n\t"
          "mul.s    %[f2],  %[f2],      %[scale]            \n\t"
          "mul.s    %[f3],  %[f3],      %[scale]            \n\t"
          "lwc1     %[f4],  48(%[fft])                      \n\t"
          "lwc1     %[f5],  52(%[fft])                      \n\t"
          "lwc1     %[f6],  56(%[fft])                      \n\t"
          "lwc1     %[f7],  60(%[fft])                      \n\t"
          "mul.s    %[f4],  %[f4],      %[scale]            \n\t"
          "mul.s    %[f5],  %[f5],      %[scale]            \n\t"
          "mul.s    %[f6],  %[f6],      %[scale]            \n\t"
          "mul.s    %[f7],  %[f7],      %[scale]            \n\t"
          "swc1     %[f0],  32(%[fft])                      \n\t"
          "swc1     %[f1],  36(%[fft])                      \n\t"
          "swc1     %[f2],  40(%[fft])                      \n\t"
          "swc1     %[f3],  44(%[fft])                      \n\t"
          "swc1     %[f4],  48(%[fft])                      \n\t"
          "swc1     %[f5],  52(%[fft])                      \n\t"
          "swc1     %[f6],  56(%[fft])                      \n\t"
          "swc1     %[f7],  60(%[fft])                      \n\t"

          "lwc1     %[f0],  64(%[fft])                      \n\t"
          "lwc1     %[f1],  68(%[fft])                      \n\t"
          "lwc1     %[f2],  72(%[fft])                      \n\t"
          "lwc1     %[f3],  76(%[fft])                      \n\t"
          "mul.s    %[f0],  %[f0],      %[scale]            \n\t"
          "mul.s    %[f1],  %[f1],      %[scale]            \n\t"
          "mul.s    %[f2],  %[f2],      %[scale]            \n\t"
          "mul.s    %[f3],  %[f3],      %[scale]            \n\t"
          "lwc1     %[f4],  80(%[fft])                      \n\t"
          "lwc1     %[f5],  84(%[fft])                      \n\t"
          "lwc1     %[f6],  88(%[fft])                      \n\t"
          "lwc1     %[f7],  92(%[fft])                      \n\t"
          "mul.s    %[f4],  %[f4],      %[scale]            \n\t"
          "mul.s    %[f5],  %[f5],      %[scale]            \n\t"
          "mul.s    %[f6],  %[f6],      %[scale]            \n\t"
          "mul.s    %[f7],  %[f7],      %[scale]            \n\t"
          "swc1     %[f0],  64(%[fft])                      \n\t"
          "swc1     %[f1],  68(%[fft])                      \n\t"
          "swc1     %[f2],  72(%[fft])                      \n\t"
          "swc1     %[f3],  76(%[fft])                      \n\t"
          "swc1     %[f4],  80(%[fft])                      \n\t"
          "swc1     %[f5],  84(%[fft])                      \n\t"
          "swc1     %[f6],  88(%[fft])                      \n\t"
          "swc1     %[f7],  92(%[fft])                      \n\t"

          "lwc1     %[f0],  96(%[fft])                      \n\t"
          "lwc1     %[f1],  100(%[fft])                     \n\t"
          "lwc1     %[f2],  104(%[fft])                     \n\t"
          "lwc1     %[f3],  108(%[fft])                     \n\t"
          "mul.s    %[f0],  %[f0],      %[scale]            \n\t"
          "mul.s    %[f1],  %[f1],      %[scale]            \n\t"
          "mul.s    %[f2],  %[f2],      %[scale]            \n\t"
          "mul.s    %[f3],  %[f3],      %[scale]            \n\t"
          "lwc1     %[f4],  112(%[fft])                     \n\t"
          "lwc1     %[f5],  116(%[fft])                     \n\t"
          "lwc1     %[f6],  120(%[fft])                     \n\t"
          "lwc1     %[f7],  124(%[fft])                     \n\t"
          "mul.s    %[f4],  %[f4],      %[scale]            \n\t"
          "mul.s    %[f5],  %[f5],      %[scale]            \n\t"
          "mul.s    %[f6],  %[f6],      %[scale]            \n\t"
          "mul.s    %[f7],  %[f7],      %[scale]            \n\t"
          "swc1     %[f0],  96(%[fft])                      \n\t"
          "swc1     %[f1],  100(%[fft])                     \n\t"
          "swc1     %[f2],  104(%[fft])                     \n\t"
          "swc1     %[f3],  108(%[fft])                     \n\t"
          "swc1     %[f4],  112(%[fft])                     \n\t"
          "swc1     %[f5],  116(%[fft])                     \n\t"
          "swc1     %[f6],  120(%[fft])                     \n\t"
          "swc1     %[f7],  124(%[fft])                     \n\t"

          "lwc1     %[f0],  128(%[fft])                     \n\t"
          "lwc1     %[f1],  132(%[fft])                     \n\t"
          "lwc1     %[f2],  136(%[fft])                     \n\t"
          "lwc1     %[f3],  140(%[fft])                     \n\t"
          "mul.s    %[f0],  %[f0],      %[scale]            \n\t"
          "mul.s    %[f1],  %[f1],      %[scale]            \n\t"
          "mul.s    %[f2],  %[f2],      %[scale]            \n\t"
          "mul.s    %[f3],  %[f3],      %[scale]            \n\t"
          "lwc1     %[f4],  144(%[fft])                     \n\t"
          "lwc1     %[f5],  148(%[fft])                     \n\t"
          "lwc1     %[f6],  152(%[fft])                     \n\t"
          "lwc1     %[f7],  156(%[fft])                     \n\t"
          "mul.s    %[f4],  %[f4],      %[scale]            \n\t"
          "mul.s    %[f5],  %[f5],      %[scale]            \n\t"
          "mul.s    %[f6],  %[f6],      %[scale]            \n\t"
          "mul.s    %[f7],  %[f7],      %[scale]            \n\t"
          "swc1     %[f0],  128(%[fft])                     \n\t"
          "swc1     %[f1],  132(%[fft])                     \n\t"
          "swc1     %[f2],  136(%[fft])                     \n\t"
          "swc1     %[f3],  140(%[fft])                     \n\t"
          "swc1     %[f4],  144(%[fft])                     \n\t"
          "swc1     %[f5],  148(%[fft])                     \n\t"
          "swc1     %[f6],  152(%[fft])                     \n\t"
          "swc1     %[f7],  156(%[fft])                     \n\t"

          "lwc1     %[f0],  160(%[fft])                     \n\t"
          "lwc1     %[f1],  164(%[fft])                     \n\t"
          "lwc1     %[f2],  168(%[fft])                     \n\t"
          "lwc1     %[f3],  172(%[fft])                     \n\t"
          "mul.s    %[f0],  %[f0],      %[scale]            \n\t"
          "mul.s    %[f1],  %[f1],      %[scale]            \n\t"
          "mul.s    %[f2],  %[f2],      %[scale]            \n\t"
          "mul.s    %[f3],  %[f3],      %[scale]            \n\t"
          "lwc1     %[f4],  176(%[fft])                     \n\t"
          "lwc1     %[f5],  180(%[fft])                     \n\t"
          "lwc1     %[f6],  184(%[fft])                     \n\t"
          "lwc1     %[f7],  188(%[fft])                     \n\t"
          "mul.s    %[f4],  %[f4],      %[scale]            \n\t"
          "mul.s    %[f5],  %[f5],      %[scale]            \n\t"
          "mul.s    %[f6],  %[f6],      %[scale]            \n\t"
          "mul.s    %[f7],  %[f7],      %[scale]            \n\t"
          "swc1     %[f0],  160(%[fft])                     \n\t"
          "swc1     %[f1],  164(%[fft])                     \n\t"
          "swc1     %[f2],  168(%[fft])                     \n\t"
          "swc1     %[f3],  172(%[fft])                     \n\t"
          "swc1     %[f4],  176(%[fft])                     \n\t"
          "swc1     %[f5],  180(%[fft])                     \n\t"
          "swc1     %[f6],  184(%[fft])                     \n\t"
          "swc1     %[f7],  188(%[fft])                     \n\t"

          "lwc1     %[f0],  192(%[fft])                     \n\t"
          "lwc1     %[f1],  196(%[fft])                     \n\t"
          "lwc1     %[f2],  200(%[fft])                     \n\t"
          "lwc1     %[f3],  204(%[fft])                     \n\t"
          "mul.s    %[f0],  %[f0],      %[scale]            \n\t"
          "mul.s    %[f1],  %[f1],      %[scale]            \n\t"
          "mul.s    %[f2],  %[f2],      %[scale]            \n\t"
          "mul.s    %[f3],  %[f3],      %[scale]            \n\t"
          "lwc1     %[f4],  208(%[fft])                     \n\t"
          "lwc1     %[f5],  212(%[fft])                     \n\t"
          "lwc1     %[f6],  216(%[fft])                     \n\t"
          "lwc1     %[f7],  220(%[fft])                     \n\t"
          "mul.s    %[f4],  %[f4],      %[scale]            \n\t"
          "mul.s    %[f5],  %[f5],      %[scale]            \n\t"
          "mul.s    %[f6],  %[f6],      %[scale]            \n\t"
          "mul.s    %[f7],  %[f7],      %[scale]            \n\t"
          "swc1     %[f0],  192(%[fft])                     \n\t"
          "swc1     %[f1],  196(%[fft])                     \n\t"
          "swc1     %[f2],  200(%[fft])                     \n\t"
          "swc1     %[f3],  204(%[fft])                     \n\t"
          "swc1     %[f4],  208(%[fft])                     \n\t"
          "swc1     %[f5],  212(%[fft])                     \n\t"
          "swc1     %[f6],  216(%[fft])                     \n\t"
          "swc1     %[f7],  220(%[fft])                     \n\t"

          "lwc1     %[f0],  224(%[fft])                     \n\t"
          "lwc1     %[f1],  228(%[fft])                     \n\t"
          "lwc1     %[f2],  232(%[fft])                     \n\t"
          "lwc1     %[f3],  236(%[fft])                     \n\t"
          "mul.s    %[f0],  %[f0],      %[scale]            \n\t"
          "mul.s    %[f1],  %[f1],      %[scale]            \n\t"
          "mul.s    %[f2],  %[f2],      %[scale]            \n\t"
          "mul.s    %[f3],  %[f3],      %[scale]            \n\t"
          "lwc1     %[f4],  240(%[fft])                     \n\t"
          "lwc1     %[f5],  244(%[fft])                     \n\t"
          "lwc1     %[f6],  248(%[fft])                     \n\t"
          "lwc1     %[f7],  252(%[fft])                     \n\t"
          "mul.s    %[f4],  %[f4],      %[scale]            \n\t"
          "mul.s    %[f5],  %[f5],      %[scale]            \n\t"
          "mul.s    %[f6],  %[f6],      %[scale]            \n\t"
          "mul.s    %[f7],  %[f7],      %[scale]            \n\t"
          "swc1     %[f0],  224(%[fft])                     \n\t"
          "swc1     %[f1],  228(%[fft])                     \n\t"
          "swc1     %[f2],  232(%[fft])                     \n\t"
          "swc1     %[f3],  236(%[fft])                     \n\t"
          "swc1     %[f4],  240(%[fft])                     \n\t"
          "swc1     %[f5],  244(%[fft])                     \n\t"
          "swc1     %[f6],  248(%[fft])                     \n\t"
          "swc1     %[f7],  252(%[fft])                     \n\t"
          ".set pop                                         \n\t"
          : [f0] "=&f" (f0), [f1] "=&f" (f1), [f2] "=&f" (f2),
            [f3] "=&f" (f3), [f4] "=&f" (f4), [f5] "=&f" (f5),
            [f6] "=&f" (f6), [f7] "=&f" (f7)
          : [scale] "f" (scale), [fft] "r" (fft)
          : "memory"
      );
    }
    aec_rdft_forward_128(fft);
    aRe = aec->wfBuf[0] + pos;
    aIm = aec->wfBuf[1] + pos;
    __asm__ volatile(
        ".set push                                              \n\t"
        ".set noreorder                                         \n\t"
        "lwc1   %[f0],      0(%[aRe])                           \n\t"
        "lwc1   %[f1],      0(%[fft])                           \n\t"
        "lwc1   %[f2],      4(%[fft])                           \n\t"
        "lwc1   %[f3],      4(%[aRe])                           \n\t"
        "lwc1   %[f4],      8(%[fft])                           \n\t"
        "lwc1   %[f5],      4(%[aIm])                           \n\t"
        "lwc1   %[f6],      12(%[fft])                          \n\t"
        "lwc1   %[f7],      8(%[aRe])                           \n\t"
        "lwc1   %[f8],      16(%[fft])                          \n\t"
        "lwc1   %[f9],      8(%[aIm])                           \n\t"
        "lwc1   %[f10],     20(%[fft])                          \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      0(%[aRe])                           \n\t"
        "swc1   %[f3],      4(%[aRe])                           \n\t"
        "swc1   %[f5],      4(%[aIm])                           \n\t"
        "swc1   %[f7],      8(%[aRe])                           \n\t"
        "swc1   %[f9],      8(%[aIm])                           \n\t"

        "lwc1   %[f0],      12(%[aRe])                          \n\t"
        "lwc1   %[f1],      24(%[fft])                          \n\t"
        "lwc1   %[f3],      12(%[aIm])                          \n\t"
        "lwc1   %[f4],      28(%[fft])                          \n\t"
        "lwc1   %[f5],      16(%[aRe])                          \n\t"
        "lwc1   %[f6],      32(%[fft])                          \n\t"
        "lwc1   %[f7],      16(%[aIm])                          \n\t"
        "lwc1   %[f8],      36(%[fft])                          \n\t"
        "lwc1   %[f9],      20(%[aRe])                          \n\t"
        "lwc1   %[f10],     40(%[fft])                          \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      12(%[aRe])                          \n\t"
        "swc1   %[f3],      12(%[aIm])                          \n\t"
        "swc1   %[f5],      16(%[aRe])                          \n\t"
        "swc1   %[f7],      16(%[aIm])                          \n\t"
        "swc1   %[f9],      20(%[aRe])                          \n\t"

        "lwc1   %[f0],      20(%[aIm])                          \n\t"
        "lwc1   %[f1],      44(%[fft])                          \n\t"
        "lwc1   %[f3],      24(%[aRe])                          \n\t"
        "lwc1   %[f4],      48(%[fft])                          \n\t"
        "lwc1   %[f5],      24(%[aIm])                          \n\t"
        "lwc1   %[f6],      52(%[fft])                          \n\t"
        "lwc1   %[f7],      28(%[aRe])                          \n\t"
        "lwc1   %[f8],      56(%[fft])                          \n\t"
        "lwc1   %[f9],      28(%[aIm])                          \n\t"
        "lwc1   %[f10],     60(%[fft])                          \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      20(%[aIm])                          \n\t"
        "swc1   %[f3],      24(%[aRe])                          \n\t"
        "swc1   %[f5],      24(%[aIm])                          \n\t"
        "swc1   %[f7],      28(%[aRe])                          \n\t"
        "swc1   %[f9],      28(%[aIm])                          \n\t"

        "lwc1   %[f0],      32(%[aRe])                          \n\t"
        "lwc1   %[f1],      64(%[fft])                          \n\t"
        "lwc1   %[f3],      32(%[aIm])                          \n\t"
        "lwc1   %[f4],      68(%[fft])                          \n\t"
        "lwc1   %[f5],      36(%[aRe])                          \n\t"
        "lwc1   %[f6],      72(%[fft])                          \n\t"
        "lwc1   %[f7],      36(%[aIm])                          \n\t"
        "lwc1   %[f8],      76(%[fft])                          \n\t"
        "lwc1   %[f9],      40(%[aRe])                          \n\t"
        "lwc1   %[f10],     80(%[fft])                          \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      32(%[aRe])                          \n\t"
        "swc1   %[f3],      32(%[aIm])                          \n\t"
        "swc1   %[f5],      36(%[aRe])                          \n\t"
        "swc1   %[f7],      36(%[aIm])                          \n\t"
        "swc1   %[f9],      40(%[aRe])                          \n\t"

        "lwc1   %[f0],      40(%[aIm])                          \n\t"
        "lwc1   %[f1],      84(%[fft])                          \n\t"
        "lwc1   %[f3],      44(%[aRe])                          \n\t"
        "lwc1   %[f4],      88(%[fft])                          \n\t"
        "lwc1   %[f5],      44(%[aIm])                          \n\t"
        "lwc1   %[f6],      92(%[fft])                          \n\t"
        "lwc1   %[f7],      48(%[aRe])                          \n\t"
        "lwc1   %[f8],      96(%[fft])                          \n\t"
        "lwc1   %[f9],      48(%[aIm])                          \n\t"
        "lwc1   %[f10],     100(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      40(%[aIm])                          \n\t"
        "swc1   %[f3],      44(%[aRe])                          \n\t"
        "swc1   %[f5],      44(%[aIm])                          \n\t"
        "swc1   %[f7],      48(%[aRe])                          \n\t"
        "swc1   %[f9],      48(%[aIm])                          \n\t"

        "lwc1   %[f0],      52(%[aRe])                          \n\t"
        "lwc1   %[f1],      104(%[fft])                         \n\t"
        "lwc1   %[f3],      52(%[aIm])                          \n\t"
        "lwc1   %[f4],      108(%[fft])                         \n\t"
        "lwc1   %[f5],      56(%[aRe])                          \n\t"
        "lwc1   %[f6],      112(%[fft])                         \n\t"
        "lwc1   %[f7],      56(%[aIm])                          \n\t"
        "lwc1   %[f8],      116(%[fft])                         \n\t"
        "lwc1   %[f9],      60(%[aRe])                          \n\t"
        "lwc1   %[f10],     120(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      52(%[aRe])                          \n\t"
        "swc1   %[f3],      52(%[aIm])                          \n\t"
        "swc1   %[f5],      56(%[aRe])                          \n\t"
        "swc1   %[f7],      56(%[aIm])                          \n\t"
        "swc1   %[f9],      60(%[aRe])                          \n\t"

        "lwc1   %[f0],      60(%[aIm])                          \n\t"
        "lwc1   %[f1],      124(%[fft])                         \n\t"
        "lwc1   %[f3],      64(%[aRe])                          \n\t"
        "lwc1   %[f4],      128(%[fft])                         \n\t"
        "lwc1   %[f5],      64(%[aIm])                          \n\t"
        "lwc1   %[f6],      132(%[fft])                         \n\t"
        "lwc1   %[f7],      68(%[aRe])                          \n\t"
        "lwc1   %[f8],      136(%[fft])                         \n\t"
        "lwc1   %[f9],      68(%[aIm])                          \n\t"
        "lwc1   %[f10],     140(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      60(%[aIm])                          \n\t"
        "swc1   %[f3],      64(%[aRe])                          \n\t"
        "swc1   %[f5],      64(%[aIm])                          \n\t"
        "swc1   %[f7],      68(%[aRe])                          \n\t"
        "swc1   %[f9],      68(%[aIm])                          \n\t"

        "lwc1   %[f0],      72(%[aRe])                          \n\t"
        "lwc1   %[f1],      144(%[fft])                         \n\t"
        "lwc1   %[f3],      72(%[aIm])                          \n\t"
        "lwc1   %[f4],      148(%[fft])                         \n\t"
        "lwc1   %[f5],      76(%[aRe])                          \n\t"
        "lwc1   %[f6],      152(%[fft])                         \n\t"
        "lwc1   %[f7],      76(%[aIm])                          \n\t"
        "lwc1   %[f8],      156(%[fft])                         \n\t"
        "lwc1   %[f9],      80(%[aRe])                          \n\t"
        "lwc1   %[f10],     160(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      72(%[aRe])                          \n\t"
        "swc1   %[f3],      72(%[aIm])                          \n\t"
        "swc1   %[f5],      76(%[aRe])                          \n\t"
        "swc1   %[f7],      76(%[aIm])                          \n\t"
        "swc1   %[f9],      80(%[aRe])                          \n\t"

        "lwc1   %[f0],      80(%[aIm])                          \n\t"
        "lwc1   %[f1],      164(%[fft])                         \n\t"
        "lwc1   %[f3],      84(%[aRe])                          \n\t"
        "lwc1   %[f4],      168(%[fft])                         \n\t"
        "lwc1   %[f5],      84(%[aIm])                          \n\t"
        "lwc1   %[f6],      172(%[fft])                         \n\t"
        "lwc1   %[f7],      88(%[aRe])                          \n\t"
        "lwc1   %[f8],      176(%[fft])                         \n\t"
        "lwc1   %[f9],      88(%[aIm])                          \n\t"
        "lwc1   %[f10],     180(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      80(%[aIm])                          \n\t"
        "swc1   %[f3],      84(%[aRe])                          \n\t"
        "swc1   %[f5],      84(%[aIm])                          \n\t"
        "swc1   %[f7],      88(%[aRe])                          \n\t"
        "swc1   %[f9],      88(%[aIm])                          \n\t"

        "lwc1   %[f0],      92(%[aRe])                          \n\t"
        "lwc1   %[f1],      184(%[fft])                         \n\t"
        "lwc1   %[f3],      92(%[aIm])                          \n\t"
        "lwc1   %[f4],      188(%[fft])                         \n\t"
        "lwc1   %[f5],      96(%[aRe])                          \n\t"
        "lwc1   %[f6],      192(%[fft])                         \n\t"
        "lwc1   %[f7],      96(%[aIm])                          \n\t"
        "lwc1   %[f8],      196(%[fft])                         \n\t"
        "lwc1   %[f9],      100(%[aRe])                         \n\t"
        "lwc1   %[f10],     200(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      92(%[aRe])                          \n\t"
        "swc1   %[f3],      92(%[aIm])                          \n\t"
        "swc1   %[f5],      96(%[aRe])                          \n\t"
        "swc1   %[f7],      96(%[aIm])                          \n\t"
        "swc1   %[f9],      100(%[aRe])                         \n\t"

        "lwc1   %[f0],      100(%[aIm])                         \n\t"
        "lwc1   %[f1],      204(%[fft])                         \n\t"
        "lwc1   %[f3],      104(%[aRe])                         \n\t"
        "lwc1   %[f4],      208(%[fft])                         \n\t"
        "lwc1   %[f5],      104(%[aIm])                         \n\t"
        "lwc1   %[f6],      212(%[fft])                         \n\t"
        "lwc1   %[f7],      108(%[aRe])                         \n\t"
        "lwc1   %[f8],      216(%[fft])                         \n\t"
        "lwc1   %[f9],      108(%[aIm])                         \n\t"
        "lwc1   %[f10],     220(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      100(%[aIm])                         \n\t"
        "swc1   %[f3],      104(%[aRe])                         \n\t"
        "swc1   %[f5],      104(%[aIm])                         \n\t"
        "swc1   %[f7],      108(%[aRe])                         \n\t"
        "swc1   %[f9],      108(%[aIm])                         \n\t"

        "lwc1   %[f0],      112(%[aRe])                         \n\t"
        "lwc1   %[f1],      224(%[fft])                         \n\t"
        "lwc1   %[f3],      112(%[aIm])                         \n\t"
        "lwc1   %[f4],      228(%[fft])                         \n\t"
        "lwc1   %[f5],      116(%[aRe])                         \n\t"
        "lwc1   %[f6],      232(%[fft])                         \n\t"
        "lwc1   %[f7],      116(%[aIm])                         \n\t"
        "lwc1   %[f8],      236(%[fft])                         \n\t"
        "lwc1   %[f9],      120(%[aRe])                         \n\t"
        "lwc1   %[f10],     240(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      112(%[aRe])                         \n\t"
        "swc1   %[f3],      112(%[aIm])                         \n\t"
        "swc1   %[f5],      116(%[aRe])                         \n\t"
        "swc1   %[f7],      116(%[aIm])                         \n\t"
        "swc1   %[f9],      120(%[aRe])                         \n\t"

        "lwc1   %[f0],      120(%[aIm])                         \n\t"
        "lwc1   %[f1],      244(%[fft])                         \n\t"
        "lwc1   %[f3],      124(%[aRe])                         \n\t"
        "lwc1   %[f4],      248(%[fft])                         \n\t"
        "lwc1   %[f5],      124(%[aIm])                         \n\t"
        "lwc1   %[f6],      252(%[fft])                         \n\t"
        "lwc1   %[f7],      128(%[aRe])                         \n\t"
        "lwc1   %[f8],      256(%[fft])                         \n\t"
        "lwc1   %[f9],      128(%[aIm])                         \n\t"
        "lwc1   %[f10],     260(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      120(%[aIm])                         \n\t"
        "swc1   %[f3],      124(%[aRe])                         \n\t"
        "swc1   %[f5],      124(%[aIm])                         \n\t"
        "swc1   %[f7],      128(%[aRe])                         \n\t"
        "swc1   %[f9],      128(%[aIm])                         \n\t"

        "lwc1   %[f0],      132(%[aRe])                         \n\t"
        "lwc1   %[f1],      264(%[fft])                         \n\t"
        "lwc1   %[f3],      132(%[aIm])                         \n\t"
        "lwc1   %[f4],      268(%[fft])                         \n\t"
        "lwc1   %[f5],      136(%[aRe])                         \n\t"
        "lwc1   %[f6],      272(%[fft])                         \n\t"
        "lwc1   %[f7],      136(%[aIm])                         \n\t"
        "lwc1   %[f8],      276(%[fft])                         \n\t"
        "lwc1   %[f9],      140(%[aRe])                         \n\t"
        "lwc1   %[f10],     280(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      132(%[aRe])                         \n\t"
        "swc1   %[f3],      132(%[aIm])                         \n\t"
        "swc1   %[f5],      136(%[aRe])                         \n\t"
        "swc1   %[f7],      136(%[aIm])                         \n\t"
        "swc1   %[f9],      140(%[aRe])                         \n\t"

        "lwc1   %[f0],      140(%[aIm])                         \n\t"
        "lwc1   %[f1],      284(%[fft])                         \n\t"
        "lwc1   %[f3],      144(%[aRe])                         \n\t"
        "lwc1   %[f4],      288(%[fft])                         \n\t"
        "lwc1   %[f5],      144(%[aIm])                         \n\t"
        "lwc1   %[f6],      292(%[fft])                         \n\t"
        "lwc1   %[f7],      148(%[aRe])                         \n\t"
        "lwc1   %[f8],      296(%[fft])                         \n\t"
        "lwc1   %[f9],      148(%[aIm])                         \n\t"
        "lwc1   %[f10],     300(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      140(%[aIm])                         \n\t"
        "swc1   %[f3],      144(%[aRe])                         \n\t"
        "swc1   %[f5],      144(%[aIm])                         \n\t"
        "swc1   %[f7],      148(%[aRe])                         \n\t"
        "swc1   %[f9],      148(%[aIm])                         \n\t"

        "lwc1   %[f0],      152(%[aRe])                         \n\t"
        "lwc1   %[f1],      304(%[fft])                         \n\t"
        "lwc1   %[f3],      152(%[aIm])                         \n\t"
        "lwc1   %[f4],      308(%[fft])                         \n\t"
        "lwc1   %[f5],      156(%[aRe])                         \n\t"
        "lwc1   %[f6],      312(%[fft])                         \n\t"
        "lwc1   %[f7],      156(%[aIm])                         \n\t"
        "lwc1   %[f8],      316(%[fft])                         \n\t"
        "lwc1   %[f9],      160(%[aRe])                         \n\t"
        "lwc1   %[f10],     320(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      152(%[aRe])                         \n\t"
        "swc1   %[f3],      152(%[aIm])                         \n\t"
        "swc1   %[f5],      156(%[aRe])                         \n\t"
        "swc1   %[f7],      156(%[aIm])                         \n\t"
        "swc1   %[f9],      160(%[aRe])                         \n\t"

        "lwc1   %[f0],      160(%[aIm])                         \n\t"
        "lwc1   %[f1],      324(%[fft])                         \n\t"
        "lwc1   %[f3],      164(%[aRe])                         \n\t"
        "lwc1   %[f4],      328(%[fft])                         \n\t"
        "lwc1   %[f5],      164(%[aIm])                         \n\t"
        "lwc1   %[f6],      332(%[fft])                         \n\t"
        "lwc1   %[f7],      168(%[aRe])                         \n\t"
        "lwc1   %[f8],      336(%[fft])                         \n\t"
        "lwc1   %[f9],      168(%[aIm])                         \n\t"
        "lwc1   %[f10],     340(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      160(%[aIm])                         \n\t"
        "swc1   %[f3],      164(%[aRe])                         \n\t"
        "swc1   %[f5],      164(%[aIm])                         \n\t"
        "swc1   %[f7],      168(%[aRe])                         \n\t"
        "swc1   %[f9],      168(%[aIm])                         \n\t"

        "lwc1   %[f0],      172(%[aRe])                         \n\t"
        "lwc1   %[f1],      344(%[fft])                         \n\t"
        "lwc1   %[f3],      172(%[aIm])                         \n\t"
        "lwc1   %[f4],      348(%[fft])                         \n\t"
        "lwc1   %[f5],      176(%[aRe])                         \n\t"
        "lwc1   %[f6],      352(%[fft])                         \n\t"
        "lwc1   %[f7],      176(%[aIm])                         \n\t"
        "lwc1   %[f8],      356(%[fft])                         \n\t"
        "lwc1   %[f9],      180(%[aRe])                         \n\t"
        "lwc1   %[f10],     360(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      172(%[aRe])                         \n\t"
        "swc1   %[f3],      172(%[aIm])                         \n\t"
        "swc1   %[f5],      176(%[aRe])                         \n\t"
        "swc1   %[f7],      176(%[aIm])                         \n\t"
        "swc1   %[f9],      180(%[aRe])                         \n\t"

        "lwc1   %[f0],      180(%[aIm])                         \n\t"
        "lwc1   %[f1],      364(%[fft])                         \n\t"
        "lwc1   %[f3],      184(%[aRe])                         \n\t"
        "lwc1   %[f4],      368(%[fft])                         \n\t"
        "lwc1   %[f5],      184(%[aIm])                         \n\t"
        "lwc1   %[f6],      372(%[fft])                         \n\t"
        "lwc1   %[f7],      188(%[aRe])                         \n\t"
        "lwc1   %[f8],      376(%[fft])                         \n\t"
        "lwc1   %[f9],      188(%[aIm])                         \n\t"
        "lwc1   %[f10],     380(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      180(%[aIm])                         \n\t"
        "swc1   %[f3],      184(%[aRe])                         \n\t"
        "swc1   %[f5],      184(%[aIm])                         \n\t"
        "swc1   %[f7],      188(%[aRe])                         \n\t"
        "swc1   %[f9],      188(%[aIm])                         \n\t"

        "lwc1   %[f0],      192(%[aRe])                         \n\t"
        "lwc1   %[f1],      384(%[fft])                         \n\t"
        "lwc1   %[f3],      192(%[aIm])                         \n\t"
        "lwc1   %[f4],      388(%[fft])                         \n\t"
        "lwc1   %[f5],      196(%[aRe])                         \n\t"
        "lwc1   %[f6],      392(%[fft])                         \n\t"
        "lwc1   %[f7],      196(%[aIm])                         \n\t"
        "lwc1   %[f8],      396(%[fft])                         \n\t"
        "lwc1   %[f9],      200(%[aRe])                         \n\t"
        "lwc1   %[f10],     400(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      192(%[aRe])                         \n\t"
        "swc1   %[f3],      192(%[aIm])                         \n\t"
        "swc1   %[f5],      196(%[aRe])                         \n\t"
        "swc1   %[f7],      196(%[aIm])                         \n\t"
        "swc1   %[f9],      200(%[aRe])                         \n\t"

        "lwc1   %[f0],      200(%[aIm])                         \n\t"
        "lwc1   %[f1],      404(%[fft])                         \n\t"
        "lwc1   %[f3],      204(%[aRe])                         \n\t"
        "lwc1   %[f4],      408(%[fft])                         \n\t"
        "lwc1   %[f5],      204(%[aIm])                         \n\t"
        "lwc1   %[f6],      412(%[fft])                         \n\t"
        "lwc1   %[f7],      208(%[aRe])                         \n\t"
        "lwc1   %[f8],      416(%[fft])                         \n\t"
        "lwc1   %[f9],      208(%[aIm])                         \n\t"
        "lwc1   %[f10],     420(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      200(%[aIm])                         \n\t"
        "swc1   %[f3],      204(%[aRe])                         \n\t"
        "swc1   %[f5],      204(%[aIm])                         \n\t"
        "swc1   %[f7],      208(%[aRe])                         \n\t"
        "swc1   %[f9],      208(%[aIm])                         \n\t"

        "lwc1   %[f0],      212(%[aRe])                         \n\t"
        "lwc1   %[f1],      424(%[fft])                         \n\t"
        "lwc1   %[f3],      212(%[aIm])                         \n\t"
        "lwc1   %[f4],      428(%[fft])                         \n\t"
        "lwc1   %[f5],      216(%[aRe])                         \n\t"
        "lwc1   %[f6],      432(%[fft])                         \n\t"
        "lwc1   %[f7],      216(%[aIm])                         \n\t"
        "lwc1   %[f8],      436(%[fft])                         \n\t"
        "lwc1   %[f9],      220(%[aRe])                         \n\t"
        "lwc1   %[f10],     440(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      212(%[aRe])                         \n\t"
        "swc1   %[f3],      212(%[aIm])                         \n\t"
        "swc1   %[f5],      216(%[aRe])                         \n\t"
        "swc1   %[f7],      216(%[aIm])                         \n\t"
        "swc1   %[f9],      220(%[aRe])                         \n\t"

        "lwc1   %[f0],      220(%[aIm])                         \n\t"
        "lwc1   %[f1],      444(%[fft])                         \n\t"
        "lwc1   %[f3],      224(%[aRe])                         \n\t"
        "lwc1   %[f4],      448(%[fft])                         \n\t"
        "lwc1   %[f5],      224(%[aIm])                         \n\t"
        "lwc1   %[f6],      452(%[fft])                         \n\t"
        "lwc1   %[f7],      228(%[aRe])                         \n\t"
        "lwc1   %[f8],      456(%[fft])                         \n\t"
        "lwc1   %[f9],      228(%[aIm])                         \n\t"
        "lwc1   %[f10],     460(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      220(%[aIm])                         \n\t"
        "swc1   %[f3],      224(%[aRe])                         \n\t"
        "swc1   %[f5],      224(%[aIm])                         \n\t"
        "swc1   %[f7],      228(%[aRe])                         \n\t"
        "swc1   %[f9],      228(%[aIm])                         \n\t"

        "lwc1   %[f0],      232(%[aRe])                         \n\t"
        "lwc1   %[f1],      464(%[fft])                         \n\t"
        "lwc1   %[f3],      232(%[aIm])                         \n\t"
        "lwc1   %[f4],      468(%[fft])                         \n\t"
        "lwc1   %[f5],      236(%[aRe])                         \n\t"
        "lwc1   %[f6],      472(%[fft])                         \n\t"
        "lwc1   %[f7],      236(%[aIm])                         \n\t"
        "lwc1   %[f8],      476(%[fft])                         \n\t"
        "lwc1   %[f9],      240(%[aRe])                         \n\t"
        "lwc1   %[f10],     480(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      232(%[aRe])                         \n\t"
        "swc1   %[f3],      232(%[aIm])                         \n\t"
        "swc1   %[f5],      236(%[aRe])                         \n\t"
        "swc1   %[f7],      236(%[aIm])                         \n\t"
        "swc1   %[f9],      240(%[aRe])                         \n\t"

        "lwc1   %[f0],      240(%[aIm])                         \n\t"
        "lwc1   %[f1],      484(%[fft])                         \n\t"
        "lwc1   %[f3],      244(%[aRe])                         \n\t"
        "lwc1   %[f4],      488(%[fft])                         \n\t"
        "lwc1   %[f5],      244(%[aIm])                         \n\t"
        "lwc1   %[f6],      492(%[fft])                         \n\t"
        "lwc1   %[f7],      248(%[aRe])                         \n\t"
        "lwc1   %[f8],      496(%[fft])                         \n\t"
        "lwc1   %[f9],      248(%[aIm])                         \n\t"
        "lwc1   %[f10],     500(%[fft])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f5],      %[f5],          %[f6]               \n\t"
        "add.s  %[f7],      %[f7],          %[f8]               \n\t"
        "add.s  %[f9],      %[f9],          %[f10]              \n\t"
        "swc1   %[f0],      240(%[aIm])                         \n\t"
        "swc1   %[f3],      244(%[aRe])                         \n\t"

        "lwc1   %[f0],      252(%[aRe])                         \n\t"
        "lwc1   %[f1],      504(%[fft])                         \n\t"
        "lwc1   %[f3],      252(%[aIm])                         \n\t"
        "lwc1   %[f4],      508(%[fft])                         \n\t"
        "lwc1   %[f6],      256(%[aRe])                         \n\t"
        "swc1   %[f5],      244(%[aIm])                         \n\t"
        "add.s  %[f0],      %[f0],          %[f1]               \n\t"
        "add.s  %[f3],      %[f3],          %[f4]               \n\t"
        "add.s  %[f6],      %[f6],          %[f2]               \n\t"
        "swc1   %[f7],      248(%[aRe])                         \n\t"
        "swc1   %[f9],      248(%[aIm])                         \n\t"
        "swc1   %[f0],      252(%[aRe])                         \n\t"
        "swc1   %[f3],      252(%[aIm])                         \n\t"
        "swc1   %[f6],      256(%[aRe])                         \n\t"
        ".set pop                                               \n\t"
        : [f0] "=&f" (f0), [f1] "=&f" (f1), [f2] "=&f" (f2),
          [f3] "=&f" (f3), [f4] "=&f" (f4), [f5] "=&f" (f5),
          [f6] "=&f" (f6), [f7] "=&f" (f7), [f8] "=&f" (f8),
          [f9] "=&f" (f9), [f10] "=&f" (f10)
        : [aRe] "r" (aRe), [aIm] "r" (aIm), [fft] "r" (fft)
        : "memory"
    );
  }
}


void NonLinearProcessing_mips(aec_t *aec, short *output, short *outputH)
{
    float efw[2][PART_LEN1], dfw[2][PART_LEN1], xfw[2][PART_LEN1];
    complex_t comfortNoiseHband[PART_LEN1];
    float fft[PART_LEN2];
    float scale, dtmp;
    float nlpGainHband;
    int i, j, pos;

    // Coherence and non-linear filter
    float cohde[PART_LEN1], cohxd[PART_LEN1];
    float hNlDeAvg, hNlXdAvg;
    float hNl[PART_LEN1];
    float hNlPref[PREF_BAND_SIZE];
    float hNlFb = 0, hNlFbLow = 0;
    const float prefBandQuant = 0.75f, prefBandQuantLow = 0.5f;
    const int prefBandSize = PREF_BAND_SIZE / aec->mult;
    const int minPrefBand = 4 / aec->mult;

    // Near and error power sums
    float sdSum = 0, seSum = 0;

    // Power estimate smoothing coefficients
    const float gCoh[2][2] = {{0.9f, 0.1f}, {0.93f, 0.07f}};
    const float *ptrGCoh = gCoh[aec->mult - 1];

    // Filter energy
    float wfEnMax = 0, wfEn = 0;
    const int delayEstInterval = 10 * aec->mult;

    float* xfw_ptr = NULL;

    aec->delayEstCtr++;
    if (aec->delayEstCtr == delayEstInterval) {
        aec->delayEstCtr = 0;
    }

    // initialize comfort noise for H band
    memset(comfortNoiseHband, 0, sizeof(comfortNoiseHband));
    nlpGainHband = (float)0.0;
    dtmp = (float)0.0;

    // Measure energy in each filter partition to determine delay.
    // TODO: Spread by computing one partition per block?
    if (aec->delayEstCtr == 0) {
        wfEnMax = 0;
        aec->delayIdx = 0;
        for (i = 0; i < NR_PART; i++) {
            pos = i * PART_LEN1;
            wfEn = 0;

            wfEn += aec->wfBuf[0][pos] * aec->wfBuf[0][pos] +
                    aec->wfBuf[1][pos] * aec->wfBuf[1][pos];
            for (j = 1; j < PART_LEN1; j+=4) {
                wfEn += aec->wfBuf[0][pos + j] * aec->wfBuf[0][pos + j] +
                    aec->wfBuf[1][pos + j] * aec->wfBuf[1][pos + j];
                wfEn += aec->wfBuf[0][pos + j + 1] * aec->wfBuf[0][pos + j + 1] +
                    aec->wfBuf[1][pos + j + 1] * aec->wfBuf[1][pos + j + 1];
                wfEn += aec->wfBuf[0][pos + j + 2] * aec->wfBuf[0][pos + j + 2] +
                    aec->wfBuf[1][pos + j + 2] * aec->wfBuf[1][pos + j + 2];
                wfEn += aec->wfBuf[0][pos + j + 3] * aec->wfBuf[0][pos + j + 3] +
                    aec->wfBuf[1][pos + j + 3] * aec->wfBuf[1][pos + j + 3];
            }

            if (wfEn > wfEnMax) {
                wfEnMax = wfEn;
                aec->delayIdx = i;
            }
        }
    }

    // We should always have at least one element stored in |far_buf|.
    assert(WebRtc_available_read(aec->far_buf_windowed) > 0);
    // NLP
    WebRtc_ReadBuffer(aec->far_buf_windowed, (void**) &xfw_ptr, &xfw[0][0], 1);

    // TODO(bjornv): Investigate if we can reuse |far_buf_windowed| instead of
    // |xfwBuf|.
    // Buffer far.
    memcpy(aec->xfwBuf, xfw_ptr, sizeof(float) * 2 * PART_LEN1);

    // Use delayed far.
    memcpy(xfw, aec->xfwBuf + aec->delayIdx * PART_LEN1, sizeof(xfw));

    // Windowed near fft
    for (i = 0; i < PART_LEN; i+=4) {
        fft[i] = aec->dBuf[i] * sqrtHanning[i];
        fft[i + 1] = aec->dBuf[i + 1] * sqrtHanning[i + 1];
        fft[i + 2] = aec->dBuf[i + 2] * sqrtHanning[i + 2];
        fft[i + 3] = aec->dBuf[i + 3] * sqrtHanning[i + 3];
        fft[PART_LEN + i] = aec->dBuf[PART_LEN + i] * sqrtHanning[PART_LEN - i];
        fft[PART_LEN + i + 1] = aec->dBuf[PART_LEN + i + 1] * sqrtHanning[PART_LEN - i - 1];
        fft[PART_LEN + i + 2] = aec->dBuf[PART_LEN + i + 2] * sqrtHanning[PART_LEN - i - 2];
        fft[PART_LEN + i + 3] = aec->dBuf[PART_LEN + i + 3] * sqrtHanning[PART_LEN - i - 3];
    }

    aec_rdft_forward_128(fft);

    dfw[1][0] = 0;
    dfw[1][PART_LEN] = 0;
    dfw[0][0] = fft[0];
    dfw[0][PART_LEN] = fft[1];
    dfw[0][1] = fft[2];
    dfw[1][1] = fft[3];
    for (i = 2; i < PART_LEN; i+=2) {
        dfw[0][i] = fft[2 * i];
        dfw[1][i] = fft[2 * i + 1];
        dfw[0][i + 1] = fft[2 * (i + 1)];
        dfw[1][i + 1] = fft[2 * (i + 1) + 1];
    }

    // Windowed error fft
    for (i = 0; i < PART_LEN; i+=4) {
        fft[i] = aec->eBuf[i] * sqrtHanning[i];
        fft[i + 1] = aec->eBuf[i + 1] * sqrtHanning[i + 1];
        fft[i + 2] = aec->eBuf[i + 2] * sqrtHanning[i + 2];
        fft[i + 3] = aec->eBuf[i + 3] * sqrtHanning[i + 3];
        fft[PART_LEN + i] = aec->eBuf[PART_LEN + i] * sqrtHanning[PART_LEN - i];
        fft[PART_LEN + i + 1] = aec->eBuf[PART_LEN + i + 1] * sqrtHanning[PART_LEN - i - 1];
        fft[PART_LEN + i + 2] = aec->eBuf[PART_LEN + i + 2] * sqrtHanning[PART_LEN - i - 2];
        fft[PART_LEN + i + 3] = aec->eBuf[PART_LEN + i + 3] * sqrtHanning[PART_LEN - i - 3];
    }

    aec_rdft_forward_128(fft);
    efw[1][0] = 0;
    efw[1][PART_LEN] = 0;
    efw[0][0] = fft[0];
    efw[0][PART_LEN] = fft[1];
    efw[0][1] = fft[2];
    efw[1][1] = fft[3];
    efw[0][2] = fft[4];
    efw[1][2] = fft[5];
    efw[0][3] = fft[6];
    efw[1][3] = fft[7];
    for (i = 4; i < PART_LEN; i+=4) {
        efw[0][i] = fft[2 * i];
        efw[0][i + 1] = fft[2 * (i + 1)];
        efw[0][i + 2] = fft[2 * (i + 2)];
        efw[0][i + 3] = fft[2 * (i + 3)];

        efw[1][i] = fft[2 * i + 1];
        efw[1][i + 1] = fft[2 * (i + 1) + 1];
        efw[1][i + 2] = fft[2 * (i + 2) + 1];
        efw[1][i + 3] = fft[2 * (i + 3) + 1];
    }

    // Smoothed PSD
    for (i = 0; i < PART_LEN1; i++) {
        aec->sd[i] = ptrGCoh[0] * aec->sd[i] + ptrGCoh[1] *
            (dfw[0][i] * dfw[0][i] + dfw[1][i] * dfw[1][i]);
        aec->se[i] = ptrGCoh[0] * aec->se[i] + ptrGCoh[1] *
            (efw[0][i] * efw[0][i] + efw[1][i] * efw[1][i]);
        // We threshold here to protect against the ill-effects of a zero farend.
        // The threshold is not arbitrarily chosen, but balances protection and
        // adverse interaction with the algorithm's tuning.
        // TODO: investigate further why this is so sensitive.
        aec->sx[i] = ptrGCoh[0] * aec->sx[i] + ptrGCoh[1] *
            WEBRTC_SPL_MAX(xfw[0][i] * xfw[0][i] + xfw[1][i] * xfw[1][i], 15);

        aec->sde[i][0] = ptrGCoh[0] * aec->sde[i][0] + ptrGCoh[1] *
            (dfw[0][i] * efw[0][i] + dfw[1][i] * efw[1][i]);
        aec->sde[i][1] = ptrGCoh[0] * aec->sde[i][1] + ptrGCoh[1] *
            (dfw[0][i] * efw[1][i] - dfw[1][i] * efw[0][i]);

        aec->sxd[i][0] = ptrGCoh[0] * aec->sxd[i][0] + ptrGCoh[1] *
            (dfw[0][i] * xfw[0][i] + dfw[1][i] * xfw[1][i]);
        aec->sxd[i][1] = ptrGCoh[0] * aec->sxd[i][1] + ptrGCoh[1] *
            (dfw[0][i] * xfw[1][i] - dfw[1][i] * xfw[0][i]);

        sdSum += aec->sd[i];
        seSum += aec->se[i];
    }

    // Divergent filter safeguard.
    if (aec->divergeState == 0) {
        if (seSum > sdSum) {
            aec->divergeState = 1;
        }
    }
    else {
        if (seSum * 1.05f < sdSum) {
            aec->divergeState = 0;
        }
    }

    if (aec->divergeState == 1) {
        memcpy(efw, dfw, sizeof(efw));
    }

    // Reset if error is significantly larger than nearend (13 dB).
    if (seSum > (19.95f * sdSum)) {
        memset(aec->wfBuf, 0, sizeof(aec->wfBuf));
    }

    // Subband coherence
    for (i = 0; i < PART_LEN1; i++) {
        cohde[i] = (aec->sde[i][0] * aec->sde[i][0] + aec->sde[i][1] * aec->sde[i][1]) /
            (aec->sd[i] * aec->se[i] + 1e-10f);
        cohxd[i] = (aec->sxd[i][0] * aec->sxd[i][0] + aec->sxd[i][1] * aec->sxd[i][1]) /
            (aec->sx[i] * aec->sd[i] + 1e-10f);
    }

    hNlXdAvg = 0;
    for (i = minPrefBand; i < prefBandSize + minPrefBand; i++) {
        hNlXdAvg += cohxd[i];
    }
    hNlXdAvg /= prefBandSize;
    hNlXdAvg = 1 - hNlXdAvg;

    hNlDeAvg = 0;
    for (i = minPrefBand; i < prefBandSize + minPrefBand; i++) {
        hNlDeAvg += cohde[i];
    }
    hNlDeAvg /= prefBandSize;

    if (hNlXdAvg < 0.75f && hNlXdAvg < aec->hNlXdAvgMin) {
        aec->hNlXdAvgMin = hNlXdAvg;
    }

    if (hNlDeAvg > 0.98f && hNlXdAvg > 0.9f) {
        aec->stNearState = 1;
    }
    else if (hNlDeAvg < 0.95f || hNlXdAvg < 0.8f) {
        aec->stNearState = 0;
    }

    if (aec->hNlXdAvgMin == 1) {
        aec->echoState = 0;
        aec->overDrive = aec->minOverDrive;

        if (aec->stNearState == 1) {
            memcpy(hNl, cohde, sizeof(hNl));
            hNlFb = hNlDeAvg;
            hNlFbLow = hNlDeAvg;
        }
        else {
            for (i = 0; i < PART_LEN1; i++) {
                hNl[i] = 1 - cohxd[i];
            }
            hNlFb = hNlXdAvg;
            hNlFbLow = hNlXdAvg;
        }
    }
    else {

        if (aec->stNearState == 1) {
            aec->echoState = 0;
            memcpy(hNl, cohde, sizeof(hNl));
            hNlFb = hNlDeAvg;
            hNlFbLow = hNlDeAvg;
        }
        else {
            aec->echoState = 1;
            for (i = 0; i < PART_LEN1; i++) {
                hNl[i] = WEBRTC_SPL_MIN(cohde[i], 1 - cohxd[i]);
            }

            // Select an order statistic from the preferred bands.
            // TODO: Using quicksort now, but a selection algorithm may be preferred.
            memcpy(hNlPref, &hNl[minPrefBand], sizeof(float) * prefBandSize);
            qsort(hNlPref, prefBandSize, sizeof(float), CmpFloat);
            hNlFb = hNlPref[(int)floor(prefBandQuant * (prefBandSize - 1))];
            hNlFbLow = hNlPref[(int)floor(prefBandQuantLow * (prefBandSize - 1))];
        }
    }

    // Track the local filter minimum to determine suppression overdrive.
    if (hNlFbLow < 0.6f && hNlFbLow < aec->hNlFbLocalMin) {
        aec->hNlFbLocalMin = hNlFbLow;
        aec->hNlFbMin = hNlFbLow;
        aec->hNlNewMin = 1;
        aec->hNlMinCtr = 0;
    }
    aec->hNlFbLocalMin = WEBRTC_SPL_MIN(aec->hNlFbLocalMin + 0.0008f / aec->mult, 1);
    aec->hNlXdAvgMin = WEBRTC_SPL_MIN(aec->hNlXdAvgMin + 0.0006f / aec->mult, 1);

    if (aec->hNlNewMin == 1) {
        aec->hNlMinCtr++;
    }
    if (aec->hNlMinCtr == 2) {
        aec->hNlNewMin = 0;
        aec->hNlMinCtr = 0;
        aec->overDrive = WEBRTC_SPL_MAX(aec->targetSupp /
            ((float)log(aec->hNlFbMin + 1e-10f) + 1e-10f), aec->minOverDrive);
    }

    // Smooth the overdrive.
    if (aec->overDrive < aec->overDriveSm) {
      aec->overDriveSm = 0.99f * aec->overDriveSm + 0.01f * aec->overDrive;
    }
    else {
      aec->overDriveSm = 0.9f * aec->overDriveSm + 0.1f * aec->overDrive;
    }

    WebRtcAec_OverdriveAndSuppress(aec, hNl, hNlFb, efw);

    // Add comfort noise.
    ComfortNoise_mips(aec, efw, comfortNoiseHband, aec->noisePow, hNl);

    // TODO(bjornv): Investigate how to take the windowing below into account if
    // needed.
    if (aec->metricsMode == 1) {
      // Note that we have a scaling by two in the time domain |eBuf|.
      // In addition the time domain signal is windowed before transformation,
      // losing half the energy on the average. We take care of the first
      // scaling only in UpdateMetrics().
      UpdateLevel(&aec->nlpoutlevel, efw);
    }
    // Inverse error fft.
    fft[0] = efw[0][0];
    fft[1] = efw[0][PART_LEN];
    fft[2] = efw[0][1];
    fft[3] = -efw[1][1];
    fft[4] = efw[0][2];
    fft[5] = -efw[1][2];
    fft[6] = efw[0][3];
    fft[7] = -efw[1][3];
    for (i = 4; i < PART_LEN; i+=4) {
        fft[2*i] = efw[0][i];
        fft[2*(i + 1)] = efw[0][i + 1];
        fft[2*(i + 2)] = efw[0][i + 2];
        fft[2*(i + 3)] = efw[0][i + 3];

        fft[2*i + 1] = -efw[1][i];
        fft[2*(i + 1) + 1] = -efw[1][i + 1];
        fft[2*(i + 2) + 1] = -efw[1][i + 2];
        fft[2*(i + 3) + 1] = -efw[1][i + 3];
    }

    aec_rdft_inverse_128(fft);

    // Overlap and add to obtain output.
    scale = 2.0f / PART_LEN2;
#if defined (MIPS_DSP_R1_LE)
    float tempf1, tempf2, tempf3, tempf4;
    int tempi1, tempi2, tempi3, tempi4;
    float *fft_ptr = fft;
    short *out_ptr = output;
#endif

    for (i = 0; i < PART_LEN; i+=4) {
        // fft scaling
        fft[i] *= scale;
        fft[i + 1] *= scale;
        fft[i + 2] *= scale;
        fft[i + 3] *= scale;
        fft[PART_LEN + i] *= scale;
        fft[PART_LEN + i + 1] *= scale;
        fft[PART_LEN + i + 2] *= scale;
        fft[PART_LEN + i + 3] *= scale;
        fft[i] = fft[i]*sqrtHanning[i] + aec->outBuf[i];
        fft[i + 1] = fft[i + 1]*sqrtHanning[i + 1] + aec->outBuf[i + 1];
        fft[i + 2] = fft[i + 2]*sqrtHanning[i + 2] + aec->outBuf[i + 2];
        fft[i + 3] = fft[i + 3]*sqrtHanning[i + 3] + aec->outBuf[i + 3];
        aec->outBuf[i] = fft[PART_LEN + i] * sqrtHanning[PART_LEN - i];
        aec->outBuf[i + 1] = fft[PART_LEN + i + 1] * sqrtHanning[PART_LEN - i - 1];
        aec->outBuf[i + 2] = fft[PART_LEN + i + 2] * sqrtHanning[PART_LEN - i - 2];
        aec->outBuf[i + 3] = fft[PART_LEN + i + 3] * sqrtHanning[PART_LEN - i - 3];

#if defined (MIPS_DSP_R1_LE)
        __asm__ volatile(
            "lwc1       %[tempf1],     0(%[fft_ptr])              \n\t"
            "lwc1       %[tempf2],     4(%[fft_ptr])              \n\t"
            "lwc1       %[tempf3],     8(%[fft_ptr])              \n\t"
            "lwc1       %[tempf4],     12(%[fft_ptr])             \n\t"
            "trunc.w.s  %[tempf1],     %[tempf1]                  \n\t"
            "trunc.w.s  %[tempf2],     %[tempf2]                  \n\t"
            "trunc.w.s  %[tempf3],     %[tempf3]                  \n\t"
            "trunc.w.s  %[tempf4],     %[tempf4]                  \n\t"
            "addiu      %[fft_ptr],    %[fft_ptr],     16         \n\t"
            "addiu      %[out_ptr],    %[out_ptr],     8          \n\t"
            "mfc1       %[tempi1],     %[tempf1]                  \n\t"
            "mfc1       %[tempi2],     %[tempf2]                  \n\t"
            "mfc1       %[tempi3],     %[tempf3]                  \n\t"
            "mfc1       %[tempi4],     %[tempf4]                  \n\t"
            "shll_s.w   %[tempi1],     %[tempi1],      16         \n\t"
            "shll_s.w   %[tempi2],     %[tempi2],      16         \n\t"
            "shll_s.w   %[tempi3],     %[tempi3],      16         \n\t"
            "shll_s.w   %[tempi4],     %[tempi4],      16         \n\t"
            "sra        %[tempi1],     %[tempi1],      16         \n\t"
            "sra        %[tempi2],     %[tempi2],      16         \n\t"
            "sra        %[tempi3],     %[tempi3],      16         \n\t"
            "sra        %[tempi4],     %[tempi4],      16         \n\t"
            "sh         %[tempi1],     -8(%[out_ptr])             \n\t"
            "sh         %[tempi2],     -6(%[out_ptr])             \n\t"
            "sh         %[tempi3],     -4(%[out_ptr])             \n\t"
            "sh         %[tempi4],     -2(%[out_ptr])             \n\t"

            :[out_ptr]"+r"(out_ptr), [fft_ptr]"+r"(fft_ptr),
             [tempi1]"=&r"(tempi1), [tempi2]"=&r"(tempi2), [tempi3]"=&r"(tempi3), [tempi4]"=&r"(tempi4),
             [tempf1]"=&f"(tempf1), [tempf2]"=&f"(tempf2), [tempf3]"=&f"(tempf3), [tempf4]"=&f"(tempf4)
            :
            : "memory"
        );
#else
        // Saturation protection
        output[i] = (short)WEBRTC_SPL_SAT(WEBRTC_SPL_WORD16_MAX, fft[i],
            WEBRTC_SPL_WORD16_MIN);
        output[i + 1] = (short)WEBRTC_SPL_SAT(WEBRTC_SPL_WORD16_MAX, fft[i + 1],
            WEBRTC_SPL_WORD16_MIN);
        output[i + 2] = (short)WEBRTC_SPL_SAT(WEBRTC_SPL_WORD16_MAX, fft[i + 2],
            WEBRTC_SPL_WORD16_MIN);
        output[i + 3] = (short)WEBRTC_SPL_SAT(WEBRTC_SPL_WORD16_MAX, fft[i + 3],
            WEBRTC_SPL_WORD16_MIN);
#endif
    }

    // For H band
    if (aec->sampFreq == 32000) {

        // H band gain
        // average nlp over low band: average over second half of freq spectrum
        // (4->8khz)
        GetHighbandGain(hNl, &nlpGainHband);

        // Inverse comfort_noise
        if (flagHbandCn == 1) {
            fft[0] = comfortNoiseHband[0][0];
            fft[1] = comfortNoiseHband[PART_LEN][0];
            for (i = 1; i < PART_LEN; i++) {
                fft[2*i] = comfortNoiseHband[i][0];
                fft[2*i + 1] = comfortNoiseHband[i][1];
            }
            aec_rdft_inverse_128(fft);
            scale = 2.0f / PART_LEN2;
        }

        // compute gain factor
        for (i = 0; i < PART_LEN; i++) {
            dtmp = (float)aec->dBufH[i];
            dtmp = (float)dtmp * nlpGainHband; // for variable gain

            // add some comfort noise where Hband is attenuated
            if (flagHbandCn == 1) {
                fft[i] *= scale; // fft scaling
                dtmp += cnScaleHband * fft[i];
            }

            // Saturation protection
            outputH[i] = (short)WEBRTC_SPL_SAT(WEBRTC_SPL_WORD16_MAX, dtmp,
                WEBRTC_SPL_WORD16_MIN);
         }
    }

    // Copy the current block to the old position.
    memcpy(aec->dBuf, aec->dBuf + PART_LEN, sizeof(float) * PART_LEN);
    memcpy(aec->eBuf, aec->eBuf + PART_LEN, sizeof(float) * PART_LEN);

    // Copy the current block to the old position for H band
    if (aec->sampFreq == 32000) {
        memcpy(aec->dBufH, aec->dBufH + PART_LEN, sizeof(float) * PART_LEN);
    }

    memmove(aec->xfwBuf + PART_LEN1, aec->xfwBuf, sizeof(aec->xfwBuf) -
        sizeof(complex_t) * PART_LEN1);
}


static void ComfortNoise_mips(aec_t *aec, float efw[2][PART_LEN1],
    complex_t *comfortNoiseHband, const float *noisePow, const float *lambda)
{
    int i, num;
    float rand[PART_LEN];
    float noise, noiseAvg, tmp, tmpAvg;
    WebRtc_Word16 randW16[PART_LEN];
    complex_t u[PART_LEN1];

    const float pi2 = 6.28318530717959f;
    const float pi2t = pi2 / 32768;

    // Generate a uniform random array on [0 1]
    WebRtcSpl_RandUArray(randW16, PART_LEN, &aec->seed);

    WebRtc_Word16 *randWptr = randW16;
    float randTemp, randTemp2, randTemp3, randTemp4;
    short tmp1s, tmp2s, tmp3s, tmp4s;

    for (i = 0; i < PART_LEN; i+=4) {
        __asm__ volatile(
            ".set     push                                                        \n\t"
            ".set     noreorder                                                   \n\t"
            "lh       %[tmp1s],       0(%[randWptr])                              \n\t"
            "lh       %[tmp2s],       2(%[randWptr])                              \n\t"
            "lh       %[tmp3s],       4(%[randWptr])                              \n\t"
            "lh       %[tmp4s],       6(%[randWptr])                              \n\t"
            "mtc1     %[tmp1s],       %[randTemp]                                 \n\t"
            "mtc1     %[tmp2s],       %[randTemp2]                                \n\t"
            "mtc1     %[tmp3s],       %[randTemp3]                                \n\t"
            "mtc1     %[tmp4s],       %[randTemp4]                                \n\t"
            "cvt.s.w  %[randTemp],    %[randTemp]                                 \n\t"
            "cvt.s.w  %[randTemp2],   %[randTemp2]                                \n\t"
            "cvt.s.w  %[randTemp3],   %[randTemp3]                                \n\t"
            "cvt.s.w  %[randTemp4],   %[randTemp4]                                \n\t"
            "addiu    %[randWptr],    %[randWptr],      8                         \n\t"
            "mul.s    %[randTemp],    %[randTemp],      %[pi2t]                   \n\t"
            "mul.s    %[randTemp2],   %[randTemp2],     %[pi2t]                   \n\t"
            "mul.s    %[randTemp3],   %[randTemp3],     %[pi2t]                   \n\t"
            "mul.s    %[randTemp4],   %[randTemp4],     %[pi2t]                   \n\t"
            ".set     pop                                                         \n\t"

            : [randWptr] "+r" (randWptr),
              [randTemp] "=&f" (randTemp),
              [randTemp2] "=&f" (randTemp2),
              [randTemp3] "=&f" (randTemp3),
              [randTemp4] "=&f" (randTemp4),
              [tmp1s] "=&r" (tmp1s),
              [tmp2s] "=&r" (tmp2s),
              [tmp3s] "=&r" (tmp3s),
              [tmp4s] "=&r" (tmp4s)
            : [pi2t] "f" (pi2t)
            : "memory"
        );

        u[i+1][0] = (float)cos(randTemp);
        u[i+1][1] = (float)sin(randTemp);
        u[i+2][0] = (float)cos(randTemp2);
        u[i+2][1] = (float)sin(randTemp2);
        u[i+3][0] = (float)cos(randTemp3);
        u[i+3][1] = (float)sin(randTemp3);
        u[i+4][0] = (float)cos(randTemp4);
        u[i+4][1] = (float)sin(randTemp4);
    }

    // Reject LF noise
    float *u_ptr = &u[1][0];
    float noise2, noise3, noise4;
    float tmp1f, tmp2f, tmp3f, tmp4f, tmp5f, tmp6f, tmp7f, tmp8f;

    u[0][0] = 0;
    u[0][1] = 0;
    for (i = 1; i < PART_LEN1; i+=4) {
        __asm__ volatile(
            ".set     push                                                        \n\t"
            ".set     noreorder                                                   \n\t"
            "lwc1     %[noise],       4(%[noisePow])                              \n\t"
            "lwc1     %[noise2],      8(%[noisePow])                              \n\t"
            "lwc1     %[noise3],      12(%[noisePow])                             \n\t"
            "lwc1     %[noise4],      16(%[noisePow])                             \n\t"

            "sqrt.s   %[noise],       %[noise]                                    \n\t"
            "sqrt.s   %[noise2],      %[noise2]                                   \n\t"
            "sqrt.s   %[noise3],      %[noise3]                                   \n\t"
            "sqrt.s   %[noise4],      %[noise4]                                   \n\t"

            "lwc1     %[tmp1f],       0(%[u_ptr])                                 \n\t"
            "lwc1     %[tmp2f],       4(%[u_ptr])                                 \n\t"
            "lwc1     %[tmp3f],       8(%[u_ptr])                                 \n\t"
            "lwc1     %[tmp4f],       12(%[u_ptr])                                \n\t"
            "lwc1     %[tmp5f],       16(%[u_ptr])                                \n\t"
            "lwc1     %[tmp6f],       20(%[u_ptr])                                \n\t"
            "lwc1     %[tmp7f],       24(%[u_ptr])                                \n\t"
            "lwc1     %[tmp8f],       28(%[u_ptr])                                \n\t"
            "addiu    %[noisePow],    %[noisePow],      16                        \n\t"

            "mul.s    %[tmp1f],       %[tmp1f],         %[noise]                  \n\t"
            "mul.s    %[tmp2f],       %[tmp2f],         %[noise]                  \n\t"
            "mul.s    %[tmp3f],       %[tmp3f],         %[noise2]                 \n\t"
            "mul.s    %[tmp4f],       %[tmp4f],         %[noise2]                 \n\t"
            "mul.s    %[tmp5f],       %[tmp5f],         %[noise3]                 \n\t"
            "mul.s    %[tmp6f],       %[tmp6f],         %[noise3]                 \n\t"

            "swc1     %[tmp1f],       0(%[u_ptr])                                 \n\t"
            "swc1     %[tmp3f],       8(%[u_ptr])                                 \n\t"

            "mul.s    %[tmp8f],       %[tmp8f],         %[noise4]                 \n\t"
            "mul.s    %[tmp7f],       %[tmp7f],         %[noise4]                 \n\t"

            "neg.s    %[tmp2f]                                                    \n\t"
            "neg.s    %[tmp4f]                                                    \n\t"
            "neg.s    %[tmp6f]                                                    \n\t"
            "neg.s    %[tmp8f]                                                    \n\t"

            "swc1     %[tmp5f],       16(%[u_ptr])                                \n\t"
            "swc1     %[tmp7f],       24(%[u_ptr])                                \n\t"
            "swc1     %[tmp2f],       4(%[u_ptr])                                 \n\t"
            "swc1     %[tmp4f],       12(%[u_ptr])                                \n\t"
            "swc1     %[tmp6f],       20(%[u_ptr])                                \n\t"
            "swc1     %[tmp8f],       28(%[u_ptr])                                \n\t"

            "addiu    %[u_ptr],       %[u_ptr],         32                        \n\t"
            ".set     pop                                                         \n\t"

            : [u_ptr] "+r" (u_ptr),  [noisePow] "+r" (noisePow),
              [noise] "=&f" (noise), [noise2] "=&f" (noise2),
              [noise3] "=&f" (noise3), [noise4] "=&f" (noise4),
              [tmp1f] "=&f" (tmp1f), [tmp2f] "=&f" (tmp2f),
              [tmp3f] "=&f" (tmp3f), [tmp4f] "=&f" (tmp4f),
              [tmp5f] "=&f" (tmp5f), [tmp6f] "=&f" (tmp6f),
              [tmp7f] "=&f" (tmp7f), [tmp8f] "=&f" (tmp8f)
            :
            : "memory"
        );

    }
    u[PART_LEN][1] = 0;
    noisePow -= PART_LEN;

    u_ptr = &u[0][0];
    float *u_ptr_end = &u[PART_LEN][0];
    float *efw_ptr_0 = &efw[0][0];
    float *efw_ptr_1 = &efw[1][0];
    float tmp9f, tmp10f;
    const float tmp1c = 1.0;
    const float tmp2c = 0.0;

    __asm__ volatile(
        ".set     push                                                        \n\t"
        ".set     noreorder                                                   \n\t"

      "loop_start:                                                            \n\t"
        "lwc1     %[tmp1f],       0(%[lambda])                                \n\t"
        "lwc1     %[tmp6f],       4(%[lambda])                                \n\t"
        "addiu    %[lambda],      %[lambda],   8                              \n\t"
        "c.lt.s   %[tmp1f],       %[tmp1c]                                    \n\t"
        "bc1f     pass_second                                                 \n\t"
        "nop                                                                  \n\t"
        "c.lt.s   %[tmp6f],       %[tmp1c]                                    \n\t"
        "bc1f     pass_first                                                  \n\t"
        " nop                                                                 \n\t"
      "pass_both:                                                             \n\t"

        "mul.s    %[tmp1f],       %[tmp1f],         %[tmp1f]                  \n\t"
        "mul.s    %[tmp6f],       %[tmp6f],         %[tmp6f]                  \n\t"
        "sub.s    %[tmp1f],       %[tmp1c],         %[tmp1f]                  \n\t"
        "sub.s    %[tmp6f],       %[tmp1c],         %[tmp6f]                  \n\t"
        "sqrt.s   %[tmp1f],       %[tmp1f]                                    \n\t"
        "sqrt.s   %[tmp6f],       %[tmp6f]                                    \n\t"
        "lwc1     %[tmp2f],       0(%[efw_ptr_0])                             \n\t"
        "lwc1     %[tmp3f],       0(%[u_ptr])                                 \n\t"
        "lwc1     %[tmp7f],       4(%[efw_ptr_0])                             \n\t"
        "lwc1     %[tmp8f],       8(%[u_ptr])                                 \n\t"
        "lwc1     %[tmp4f],       0(%[efw_ptr_1])                             \n\t"
        "lwc1     %[tmp5f],       4(%[u_ptr])                                 \n\t"
        "lwc1     %[tmp9f],       4(%[efw_ptr_1])                             \n\t"
        "lwc1     %[tmp10f],      12(%[u_ptr])                                \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s    %[tmp3f],       %[tmp1f],         %[tmp3f]                  \n\t"
        "mul.s    %[tmp5f],       %[tmp1f],         %[tmp5f]                  \n\t"
        "mul.s    %[tmp8f],       %[tmp6f],         %[tmp8f]                  \n\t"
        "mul.s    %[tmp10f],      %[tmp6f],         %[tmp10f]                 \n\t"
        "add.s    %[tmp2f],       %[tmp2f],         %[tmp3f]                  \n\t"
        "add.s    %[tmp4f],       %[tmp4f],         %[tmp5f]                  \n\t"
        "add.s    %[tmp7f],       %[tmp7f],         %[tmp8f]                  \n\t"
        "add.s    %[tmp9f],       %[tmp9f],         %[tmp10f]                 \n\t"
#else
        "madd.s   %[tmp2f],       %[tmp2f],         %[tmp1f],     %[tmp3f]    \n\t"
        "madd.s   %[tmp4f],       %[tmp4f],         %[tmp1f],     %[tmp5f]    \n\t"
        "madd.s   %[tmp7f],       %[tmp7f],         %[tmp6f],     %[tmp8f]    \n\t"
        "madd.s   %[tmp9f],       %[tmp9f],         %[tmp6f],     %[tmp10f]   \n\t"
#endif
        "swc1     %[tmp2f],       0(%[efw_ptr_0])                             \n\t"
        "swc1     %[tmp4f],       0(%[efw_ptr_1])                             \n\t"
        "swc1     %[tmp7f],       4(%[efw_ptr_0])                             \n\t"
        "b        update_ptrs                                                 \n\t"
        " swc1    %[tmp9f],       4(%[efw_ptr_1])                             \n\t"

      "pass_first:                                                            \n\t"
        "mul.s    %[tmp1f],       %[tmp1f],         %[tmp1f]                  \n\t"
        "sub.s    %[tmp1f],       %[tmp1c],         %[tmp1f]                  \n\t"
        "sqrt.s   %[tmp1f],       %[tmp1f]                                    \n\t"
        "lwc1     %[tmp2f],       0(%[efw_ptr_0])                             \n\t"
        "lwc1     %[tmp3f],       0(%[u_ptr])                                 \n\t"
        "lwc1     %[tmp4f],       0(%[efw_ptr_1])                             \n\t"
        "lwc1     %[tmp5f],       4(%[u_ptr])                                 \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s    %[tmp3f],       %[tmp1f],         %[tmp3f]                  \n\t"
        "mul.s    %[tmp5f],       %[tmp1f],         %[tmp5f]                  \n\t"
        "add.s    %[tmp2f],       %[tmp2f],         %[tmp3f]                  \n\t"
        "add.s    %[tmp4f],       %[tmp4f],         %[tmp5f]                  \n\t"
#else
        "madd.s   %[tmp2f],       %[tmp2f],         %[tmp1f],     %[tmp3f]    \n\t"
        "madd.s   %[tmp4f],       %[tmp4f],         %[tmp1f],     %[tmp5f]    \n\t"
#endif
        "swc1     %[tmp2f],       0(%[efw_ptr_0])                             \n\t"
        "b        update_ptrs                                                 \n\t"
        " swc1    %[tmp4f],       0(%[efw_ptr_1])                             \n\t"

      "pass_second:                                                           \n\t"
        "c.lt.s   %[tmp6f],       %[tmp1c]                                    \n\t"
        "bc1f     update_ptrs                                                 \n\t"
        "nop                                                                  \n\t"
        "mul.s    %[tmp6f],       %[tmp6f],         %[tmp6f]                  \n\t"
        "sub.s    %[tmp6f],       %[tmp1c],         %[tmp6f]                  \n\t"
        "sqrt.s   %[tmp6f],       %[tmp6f]                                    \n\t"
        "lwc1     %[tmp7f],       4(%[efw_ptr_0])                             \n\t"
        "lwc1     %[tmp8f],       8(%[u_ptr])                                 \n\t"
        "lwc1     %[tmp9f],       4(%[efw_ptr_1])                             \n\t"
        "lwc1     %[tmp10f],      12(%[u_ptr])                                \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s    %[tmp3f],       %[tmp6f],         %[tmp8f]                  \n\t"
        "mul.s    %[tmp10f],      %[tmp6f],         %[tmp10f]                 \n\t"
        "add.s    %[tmp7f],       %[tmp7f],         %[tmp3f]                  \n\t"
        "add.s    %[tmp9f],       %[tmp9f],         %[tmp10f]                 \n\t"
#else
        "madd.s   %[tmp7f],       %[tmp7f],         %[tmp6f],     %[tmp8f]    \n\t"
        "madd.s   %[tmp9f],       %[tmp9f],         %[tmp6f],     %[tmp10f]   \n\t"
#endif
        "swc1     %[tmp7f],       4(%[efw_ptr_0])                             \n\t"
        "swc1     %[tmp9f],       4(%[efw_ptr_1])                             \n\t"

      "update_ptrs:                                                           \n\t"
        "addiu    %[u_ptr],       %[u_ptr],         16                        \n\t"
        "addiu    %[efw_ptr_0],   %[efw_ptr_0],     8                         \n\t"
        "bne      %[u_ptr],       %[u_ptr_end],     loop_start                \n\t"
        " addiu   %[efw_ptr_1],   %[efw_ptr_1],     8                         \n\t"
        ".set     pop                                                         \n\t"

        : [lambda] "+r" (lambda), [u_ptr] "+r" (u_ptr),
          [efw_ptr_0] "+r" (efw_ptr_0), [efw_ptr_1] "+r" (efw_ptr_1),
          [tmp1f] "=&f" (tmp1f), [tmp2f] "=&f" (tmp2f), [tmp3f] "=&f" (tmp3f),
          [tmp4f] "=&f" (tmp4f), [tmp5f] "=&f" (tmp5f),
          [tmp6f] "=&f" (tmp6f), [tmp7f] "=&f" (tmp7f), [tmp8f] "=&f" (tmp8f),
          [tmp9f] "=&f" (tmp9f), [tmp10f] "=&f" (tmp10f)
        : [tmp1c] "f" (tmp1c), [tmp2c] "f" (tmp2c), [u_ptr_end] "r" (u_ptr_end)
        : "memory"
    );

    lambda -= PART_LEN;
    tmp = sqrtf(WEBRTC_SPL_MAX(1 - lambda[PART_LEN] * lambda[PART_LEN], 0));
    //tmp = 1 - lambda[i];
    efw[0][PART_LEN] += tmp * u[PART_LEN][0];
    efw[1][PART_LEN] += tmp * u[PART_LEN][1];

    // For H band comfort noise
    // TODO: don't compute noise and "tmp" twice. Use the previous results.
    noiseAvg = 0.0;
    tmpAvg = 0.0;
    num = 0;
    if (aec->sampFreq == 32000 && flagHbandCn == 1) {
        for (i = 0; i < PART_LEN; i++) {
            rand[i] = ((float)randW16[i]) / 32768;
        }

        // average noise scale
        // average over second half of freq spectrum (i.e., 4->8khz)
        // TODO: we shouldn't need num. We know how many elements we're summing.
        for (i = PART_LEN1 >> 1; i < PART_LEN1; i++) {
            num++;
            noiseAvg += sqrtf(noisePow[i]);
        }
        noiseAvg /= (float)num;

        // average nlp scale
        // average over second half of freq spectrum (i.e., 4->8khz)
        // TODO: we shouldn't need num. We know how many elements we're summing.
        num = 0;
        for (i = PART_LEN1 >> 1; i < PART_LEN1; i++) {
            num++;
            tmpAvg += sqrtf(WEBRTC_SPL_MAX(1 - lambda[i] * lambda[i], 0));
        }
        tmpAvg /= (float)num;

        // Use average noise for H band
        // TODO: we should probably have a new random vector here.
        // Reject LF noise
        u[0][0] = 0;
        u[0][1] = 0;
        for (i = 1; i < PART_LEN1; i++) {
            tmp = pi2 * rand[i - 1];

            // Use average noise for H band
            u[i][0] = noiseAvg * (float)cos(tmp);
            u[i][1] = -noiseAvg * (float)sin(tmp);
        }
        u[PART_LEN][1] = 0;

        for (i = 0; i < PART_LEN1; i++) {
            // Use average NLP weight for H band
            comfortNoiseHband[i][0] = tmpAvg * u[i][0];
            comfortNoiseHband[i][1] = tmpAvg * u[i][1];
        }
    }
}


void ScaleErrorSignal_mips(aec_t *aec, float ef[2][PART_LEN1])
{
    float *pointEf0;
    float *pointEf1;
    float *pointErrThresh;
    float *pointAecMu;
    float *pointAecxPow;
    float *loop_end;
    float  val1 = 1e-10f;

    float temp1;
    float temp2;
    float temp3;
    float temp4;
    float temp5;
    float temp6;
    float temp7;
    float temp8;
    float temp9;
    float temp10;

    if((PART_LEN1>>1) > 0)
    {
        pointErrThresh = &(aec->errThresh);
        pointAecMu = &(aec->mu);

        pointEf0 = &(ef[0][0]);
        pointEf1 = &(ef[1][0]);
        pointAecxPow = &(aec->xPow[0]);

        loop_end = pointAecxPow + (PART_LEN1 - 1);

        __asm__ volatile (
            ".set   push                                                      \n\t"
            ".set   noreorder                                                 \n\t"
            "1:                                                               \n\t"
            "lwc1   %[temp1],        0(%[pointAecxPow])                       \n\t"
            "lwc1   %[temp7],        4(%[pointAecxPow])                       \n\t"
            "lwc1   %[temp3],        0(%[pointEf1])                           \n\t"
            "lwc1   %[temp2],        0(%[pointEf0])                           \n\t"

            "add.s  %[temp1],        %[temp1],            %[val1]             \n\t"
            "add.s  %[temp7],        %[temp7],            %[val1]             \n\t"
            "lwc1   %[temp9],        4(%[pointEf1])                           \n\t"
            "lwc1   %[temp8],        4(%[pointEf0])                           \n\t"

            "div.s  %[temp3],        %[temp3],            %[temp1]            \n\t"
            "div.s  %[temp2],        %[temp2],            %[temp1]            \n\t"
            "div.s  %[temp9],        %[temp9],            %[temp7]            \n\t"
            "div.s  %[temp8],        %[temp8],            %[temp7]            \n\t"

            "mul.s  %[temp6],        %[temp3],            %[temp3]            \n\t"
            "mul.s  %[temp10],       %[temp9],            %[temp9]            \n\t"
#if !defined(MIPS32_R2_LE)
            "mul.s  %[temp1],        %[temp2],            %[temp2]            \n\t"
            "mul.s  %[temp7],        %[temp8],            %[temp8]            \n\t"
            "add.s  %[temp6],        %[temp6],            %[temp1]            \n\t"
            "add.s  %[temp10],       %[temp10],           %[temp7]            \n\t"
#else
            "madd.s %[temp6],        %[temp6],            %[temp2], %[temp2]  \n\t"
            "madd.s %[temp10],       %[temp10],           %[temp8], %[temp8]  \n\t"
#endif
            "sqrt.s %[temp6],        %[temp6]                                 \n\t"
            "sqrt.s %[temp10],       %[temp10]                                \n\t"

            "lwc1   %[temp4],        0(%[pointErrThresh])                     \n\t"
            "lwc1   %[temp5],        0(%[pointAecMu])                         \n\t"

            "c.lt.s %[temp4],        %[temp6]                                 \n\t"
            "bc1f   2f                                                        \n\t"
            " add.s %[temp6],        %[temp6],            %[val1]             \n\t"
            "div.s  %[temp6],        %[temp4],            %[temp6]            \n\t"
            "mul.s  %[temp2],        %[temp2],            %[temp6]            \n\t"
            "mul.s  %[temp3],        %[temp3],            %[temp6]            \n\t"

            "2:                                                               \n\t"
            "c.lt.s %[temp4],        %[temp10]                                \n\t"
            "bc1f   3f                                                        \n\t"
            " add.s %[temp10],       %[temp10],           %[val1]             \n\t"
            "div.s  %[temp10],       %[temp4],            %[temp10]           \n\t"
            "mul.s  %[temp8],        %[temp8],            %[temp10]           \n\t"
            "mul.s  %[temp9],        %[temp9],            %[temp10]           \n\t"

            "3:                                                               \n\t"
            "mul.s  %[temp2],        %[temp2],            %[temp5]            \n\t"
            "mul.s  %[temp3],        %[temp3],            %[temp5]            \n\t"
            "mul.s  %[temp8],        %[temp8],            %[temp5]            \n\t"
            "mul.s  %[temp9],        %[temp9],            %[temp5]            \n\t"
            "addiu  %[pointAecxPow], %[pointAecxPow],     8                   \n\t"
            "swc1   %[temp2],        0(%[pointEf0])                           \n\t"
            "swc1   %[temp3],        0(%[pointEf1])                           \n\t"
            "swc1   %[temp8],        4(%[pointEf0])                           \n\t"
            "swc1   %[temp9],        4(%[pointEf1])                           \n\t"
            "addiu  %[pointEf0],     %[pointEf0],         8                   \n\t"
            "bne    %[pointAecxPow], %[loop_end],         1b                  \n\t"
            " addiu %[pointEf1],     %[pointEf1],         8                   \n\t"
            ".set   pop                                                       \n\t"

            : [temp1] "=&f" (temp1),
              [temp2] "=&f" (temp2),
              [temp3] "=&f" (temp3),
              [temp4] "=&f" (temp4),
              [temp5] "=&f" (temp5),
              [temp6] "=&f" (temp6),
              [temp7] "=&f" (temp7),
              [temp8] "=&f" (temp8),
              [temp9] "=&f" (temp9),
              [temp10] "=&f" (temp10),
              [pointAecxPow] "+r" (pointAecxPow),
              [pointEf0] "+r" (pointEf0),
              [pointEf1] "+r" (pointEf1)
            : [pointErrThresh] "r" (pointErrThresh),
              [loop_end] "r" (loop_end),
              [pointAecMu] "r" (pointAecMu),
              [val1] "f" (val1)
            : "memory"
        );

    }

    pointEf0 = &(ef[0][64]);
    pointEf1 = &(ef[1][64]);
    pointErrThresh = &(aec->errThresh);
    pointAecMu = &(aec->mu);
    pointAecxPow = &(aec->xPow[64]);

    __asm__ volatile (
        "lwc1   %[temp1],        0(%[pointAecxPow])                       \n\t"
        "lwc1   %[temp2],        0(%[pointEf0])                           \n\t"
        "lwc1   %[temp3],        0(%[pointEf1])                           \n\t"

        "add.s  %[temp1],        %[temp1],            %[val1]             \n\t"
        "div.s  %[temp2],        %[temp2],            %[temp1]            \n\t"
        "div.s  %[temp3],        %[temp3],            %[temp1]            \n\t"

        "lwc1   %[temp4],        0(%[pointErrThresh])                     \n\t"
        "lwc1   %[temp5],        0(%[pointAecMu])                         \n\t"

        "mul.s  %[temp6],        %[temp3],            %[temp3]            \n\t"
#if !defined(MIPS32_R2_LE)
        "mul.s  %[temp1],        %[temp2],            %[temp2]            \n\t"
        "add.s  %[temp6],        %[temp6],            %[temp1]            \n\t"
#else
        "madd.s %[temp6],        %[temp6],            %[temp2],  %[temp2] \n\t"
#endif
        "sqrt.s %[temp6],        %[temp6]                                 \n\t"

        "c.lt.s %[temp4],        %[temp6]                                 \n\t"

        "bc1f   1f                                                        \n\t"
        "add.s  %[temp6],        %[temp6],            %[val1]             \n\t"
        "div.s  %[temp6],        %[temp4],            %[temp6]            \n\t"
        "mul.s  %[temp2],        %[temp2],            %[temp6]            \n\t"
        "mul.s  %[temp3],        %[temp3],            %[temp6]            \n\t"

        "1:                                                               \n\t"
        "mul.s  %[temp2],        %[temp2],            %[temp5]            \n\t"
        "mul.s  %[temp3],        %[temp3],            %[temp5]            \n\t"
        "swc1   %[temp2],        0(%[pointEf0])                           \n\t"
        "swc1   %[temp3],        0(%[pointEf1])                           \n\t"

        : [temp1] "=&f" (temp1),
          [temp2] "=&f" (temp2),
          [temp3] "=&f" (temp3),
          [temp4] "=&f" (temp4),
          [temp5] "=&f" (temp5),
          [temp6] "=&f" (temp6)
        : [pointEf0] "r" (pointEf0),
          [pointEf1] "r" (pointEf1),
          [pointErrThresh] "r" (pointErrThresh),
          [pointAecMu] "r" (pointAecMu),
          [pointAecxPow] "r" (pointAecxPow),
          [val1] "f" (val1)
        : "memory"
    );
}
#endif //#if defined(MIPS32_LE) & defined(MIPS_FPU_LE)

void WebRtcAec_InitAec_Mips(void)
{
#if defined(MIPS32_LE) & defined(MIPS_FPU_LE)
    WebRtcAec_FilterFar = FilterFar_mips;
    WebRtcAec_FilterAdaptation = FilterAdaptation_mips;
    WebRtcAec_NonLinearProcessing = NonLinearProcessing_mips;
    WebRtcAec_ScaleErrorSignal = ScaleErrorSignal_mips;
#endif //#if defined(MIPS32_LE) & defined(MIPS_FPU_LE)
}
