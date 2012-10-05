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
 * This file contains the implementation of function
 * WebRtcSpl_MaxAbsValueW16()
 *
 * The description header can be found in signal_processing_library.h.
 *
 */
#if defined(__mips__)

#include "signal_processing_library.h"

// Maximum absolute value of word16 vector.
WebRtc_Word16 WebRtcSpl_MaxAbsValueW16(const WebRtc_Word16 *vector, WebRtc_Word16 length)
{
    WebRtc_Word32 totMax = 0;
    WebRtc_Word32 tmp32_0, tmp32_1, tmp32_2, tmp32_3;
    int i;

#if defined(__mips_dsp)
    G_CONST WebRtc_Word32 *tmpvec32 = (WebRtc_Word32 *)vector;
    for (i = 0; i < length/16; i++)
    {
        __asm__ volatile (
            "lw         %[tmp32_0],     0(%[tmpvec32])              \n\t"
            "lw         %[tmp32_1],     4(%[tmpvec32])              \n\t"
            "lw         %[tmp32_2],     8(%[tmpvec32])              \n\t"
            "lw         %[tmp32_3],     12(%[tmpvec32])             \n\t"

            "absq_s.ph  %[tmp32_0],     %[tmp32_0]                  \n\t"
            "absq_s.ph  %[tmp32_1],     %[tmp32_1]                  \n\t"
            "cmp.lt.ph  %[totMax],      %[tmp32_0]                  \n\t"
            "pick.ph    %[totMax],      %[tmp32_0],     %[totMax]   \n\t"

            "lw         %[tmp32_0],     16(%[tmpvec32])             \n\t"
            "absq_s.ph  %[tmp32_2],     %[tmp32_2]                  \n\t"
            "cmp.lt.ph  %[totMax],      %[tmp32_1]                  \n\t"
            "pick.ph    %[totMax],      %[tmp32_1],     %[totMax]   \n\t"

            "lw         %[tmp32_1],     20(%[tmpvec32])             \n\t"
            "absq_s.ph  %[tmp32_3],     %[tmp32_3]                  \n\t"
            "cmp.lt.ph  %[totMax],      %[tmp32_2]                  \n\t"
            "pick.ph    %[totMax],      %[tmp32_2],     %[totMax]   \n\t"

            "lw         %[tmp32_2],     24(%[tmpvec32])             \n\t"
            "cmp.lt.ph  %[totMax],      %[tmp32_3]                  \n\t"
            "pick.ph    %[totMax],      %[tmp32_3],     %[totMax]   \n\t"

            "lw         %[tmp32_3],     28(%[tmpvec32])             \n\t"
            "absq_s.ph  %[tmp32_0],     %[tmp32_0]                  \n\t"
            "absq_s.ph  %[tmp32_1],     %[tmp32_1]                  \n\t"
            "cmp.lt.ph  %[totMax],      %[tmp32_0]                  \n\t"
            "pick.ph    %[totMax],      %[tmp32_0],     %[totMax]   \n\t"

            "absq_s.ph  %[tmp32_2],     %[tmp32_2]                  \n\t"
            "cmp.lt.ph  %[totMax],      %[tmp32_1]                  \n\t"
            "pick.ph    %[totMax],      %[tmp32_1],     %[totMax]   \n\t"

            "absq_s.ph  %[tmp32_3],     %[tmp32_3]                  \n\t"
            "cmp.lt.ph  %[totMax],      %[tmp32_2]                  \n\t"
            "pick.ph    %[totMax],      %[tmp32_2],     %[totMax]   \n\t"

            "cmp.lt.ph  %[totMax],      %[tmp32_3]                  \n\t"
            "pick.ph    %[totMax],      %[tmp32_3],     %[totMax]   \n\t"

            "addiu      %[tmpvec32],    %[tmpvec32],    32          \n\t"
            : [tmp32_0] "=&r" (tmp32_0), [tmp32_1] "=&r" (tmp32_1),
              [tmp32_2] "=&r" (tmp32_2), [tmp32_3] "=&r" (tmp32_3),
              [totMax] "+r" (totMax)
            : [tmpvec32] "r" (tmpvec32)
            : "memory"
        );
    }
    __asm__ volatile (
        "rotr       %[tmp32_0],     %[totMax],      16          \n\t"
        "cmp.lt.ph  %[totMax],      %[tmp32_0]                  \n\t"
        "pick.ph    %[totMax],      %[tmp32_0],     %[totMax]   \n\t"
        : [tmp32_0] "=&r" (tmp32_0), [totMax] "+r" (totMax)
        :
    );
#else
    WebRtc_Word32 v16MaxMax = WEBRTC_SPL_WORD16_MAX;
    WebRtc_Word32 r, r1, r2, r3;
    G_CONST WebRtc_Word16 *tmpvector = vector;
    for (i = 0; i < length/16; i++)
    {
        __asm__ volatile (
            "lh     %[tmp32_0],     0(%[tmpvector])                 \n\t"
            "lh     %[tmp32_1],     2(%[tmpvector])                 \n\t"
            "lh     %[tmp32_2],     4(%[tmpvector])                 \n\t"
            "lh     %[tmp32_3],     6(%[tmpvector])                 \n\t"

            "abs    %[tmp32_0],     %[tmp32_0]                      \n\t"
            "abs    %[tmp32_1],     %[tmp32_1]                      \n\t"
            "abs    %[tmp32_2],     %[tmp32_2]                      \n\t"
            "abs    %[tmp32_3],     %[tmp32_3]                      \n\t"

            "slt    %[r],           %[totMax],      %[tmp32_0]      \n\t"
            "movn   %[totMax],      %[tmp32_0],     %[r]            \n\t"
            "slt    %[r1],          %[totMax],      %[tmp32_1]      \n\t"
            "movn   %[totMax],      %[tmp32_1],     %[r1]           \n\t"
            "slt    %[r2],          %[totMax],      %[tmp32_2]      \n\t"
            "movn   %[totMax],      %[tmp32_2],     %[r2]           \n\t"
            "slt    %[r3],          %[totMax],      %[tmp32_3]      \n\t"
            "movn   %[totMax],      %[tmp32_3],     %[r3]           \n\t"

            "lh     %[tmp32_0],     8(%[tmpvector])                 \n\t"
            "lh     %[tmp32_1],     10(%[tmpvector])                \n\t"
            "lh     %[tmp32_2],     12(%[tmpvector])                \n\t"
            "lh     %[tmp32_3],     14(%[tmpvector])                \n\t"

            "abs    %[tmp32_0],     %[tmp32_0]                      \n\t"
            "abs    %[tmp32_1],     %[tmp32_1]                      \n\t"
            "abs    %[tmp32_2],     %[tmp32_2]                      \n\t"
            "abs    %[tmp32_3],     %[tmp32_3]                      \n\t"

            "slt    %[r],           %[totMax],      %[tmp32_0]      \n\t"
            "movn   %[totMax],      %[tmp32_0],     %[r]            \n\t"
            "slt    %[r1],          %[totMax],      %[tmp32_1]      \n\t"
            "movn   %[totMax],      %[tmp32_1],     %[r1]           \n\t"
            "slt    %[r2],          %[totMax],      %[tmp32_2]      \n\t"
            "movn   %[totMax],      %[tmp32_2],     %[r2]           \n\t"
            "slt    %[r3],          %[totMax],      %[tmp32_3]      \n\t"
            "movn   %[totMax],      %[tmp32_3],     %[r3]           \n\t"

            "lh     %[tmp32_0],     16(%[tmpvector])                \n\t"
            "lh     %[tmp32_1],     18(%[tmpvector])                \n\t"
            "lh     %[tmp32_2],     20(%[tmpvector])                \n\t"
            "lh     %[tmp32_3],     22(%[tmpvector])                \n\t"

            "abs    %[tmp32_0],     %[tmp32_0]                      \n\t"
            "abs    %[tmp32_1],     %[tmp32_1]                      \n\t"
            "abs    %[tmp32_2],     %[tmp32_2]                      \n\t"
            "abs    %[tmp32_3],     %[tmp32_3]                      \n\t"

            "slt    %[r],           %[totMax],      %[tmp32_0]      \n\t"
            "movn   %[totMax],      %[tmp32_0],     %[r]            \n\t"
            "slt    %[r1],          %[totMax],      %[tmp32_1]      \n\t"
            "movn   %[totMax],      %[tmp32_1],     %[r1]           \n\t"
            "slt    %[r2],          %[totMax],      %[tmp32_2]      \n\t"
            "movn   %[totMax],      %[tmp32_2],     %[r2]           \n\t"
            "slt    %[r3],          %[totMax],      %[tmp32_3]      \n\t"
            "movn   %[totMax],      %[tmp32_3],     %[r3]           \n\t"

            "lh     %[tmp32_0],     24(%[tmpvector])                \n\t"
            "lh     %[tmp32_1],     26(%[tmpvector])                \n\t"
            "lh     %[tmp32_2],     28(%[tmpvector])                \n\t"
            "lh     %[tmp32_3],     30(%[tmpvector])                \n\t"

            "abs    %[tmp32_0],     %[tmp32_0]                      \n\t"
            "abs    %[tmp32_1],     %[tmp32_1]                      \n\t"
            "abs    %[tmp32_2],     %[tmp32_2]                      \n\t"
            "abs    %[tmp32_3],     %[tmp32_3]                      \n\t"

            "slt    %[r],           %[totMax],      %[tmp32_0]      \n\t"
            "movn   %[totMax],      %[tmp32_0],     %[r]            \n\t"
            "slt    %[r1],          %[totMax],      %[tmp32_1]      \n\t"
            "movn   %[totMax],      %[tmp32_1],     %[r1]           \n\t"
            "slt    %[r2],          %[totMax],      %[tmp32_2]      \n\t"
            "movn   %[totMax],      %[tmp32_2],     %[r2]           \n\t"
            "slt    %[r3],          %[totMax],      %[tmp32_3]      \n\t"
            "movn   %[totMax],      %[tmp32_3],     %[r3]           \n\t"

            "addiu  %[tmpvector],   %[tmpvector],   32              \n\t"
            : [tmp32_0] "=&r" (tmp32_0), [tmp32_1] "=&r" (tmp32_1),
              [tmp32_2] "=&r" (tmp32_2), [tmp32_3] "=&r" (tmp32_3),
              [totMax] "+r" (totMax), [r] "=&r" (r),
              [r1] "=&r" (r1), [r2] "=&r" (r2), [r3] "=&r" (r3)
            : [tmpvector] "r" (tmpvector)
            : "memory"
        );
    }
    __asm__ volatile (
        "slt    %[r],       %[v16MaxMax],   %[totMax]   \n\t"
        "movn   %[totMax],  %[v16MaxMax],   %[r]        \n\t"
        : [totMax] "+r" (totMax), [r] "=&r" (r)
        : [v16MaxMax] "r" (v16MaxMax)
    );
#endif
    return (WebRtc_Word16)totMax;
}
#endif //#if defined(__mips__)
