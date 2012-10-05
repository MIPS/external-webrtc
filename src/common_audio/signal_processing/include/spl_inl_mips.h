/*
 *  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */


// This header file includes the inline functions in
// the fix point signal processing library.

#ifndef WEBRTC_SPL_SPL_INL_MIPS_H_
#define WEBRTC_SPL_SPL_INL_MIPS_H_


static __inline WebRtc_Word32 WEBRTC_SPL_MUL_16_16(WebRtc_Word16 a,
                                                   WebRtc_Word16 b) {
    WebRtc_Word32 value32;

    __asm__ volatile (
        "seh    %[a],           %[a]                \n\t"
        "seh    %[b],           %[b]                \n\t"
        "mul    %[value32],     %[a],   %[b]        \n\t"
        : [value32] "=r" (value32)
        : [a] "r" (a), [b] "r" (b)
        : "hi", "lo"
    );

    return value32;
}

static __inline WebRtc_Word32 WEBRTC_SPL_MUL_16_32_RSFT16(WebRtc_Word16 a,
                                                          WebRtc_Word32 b) {
    WebRtc_Word32 value32, b1, b2;

    __asm__ volatile (
        "andi   %[b2],          %[b],           0xFFFF      \n\t"
        "sra    %[b1],          %[b],           16          \n\t"
        "sra    %[b2],          %[b2],          1           \n\t"
        "mul    %[value32],     %[a],           %[b1]       \n\t"
        "mul    %[b2],          %[a],           %[b2]       \n\t"
        "addiu  %[b2],          %[b2],          0x4000      \n\t"
        "sra    %[b2],          %[b2],          15          \n\t"
        "addu   %[value32],     %[value32],     %[b2]       \n\t"
        : [value32] "=&r" (value32), [b1] "=&r" (b1), [b2] "=&r" (b2)
        : [a] "r" (a), [b] "r" (b)
        : "hi", "lo"
    );

    return value32;
}

static __inline WebRtc_Word32 WEBRTC_SPL_MUL_32_32_RSFT32BI(WebRtc_Word32 a,
                                                            WebRtc_Word32 b) {
    WebRtc_Word32 tmp;

    if ((32767 < a) || (a < 0))
        tmp = WEBRTC_SPL_MUL_16_32_RSFT16(((WebRtc_Word16)(a >> 16)), b);
    tmp += WEBRTC_SPL_MUL_16_32_RSFT16(((WebRtc_Word16)((a & 0x0000FFFF) >> 1)), b) >> 15;

    return tmp;
}

static __inline WebRtc_Word32 WEBRTC_SPL_MUL_32_32_RSFT32(WebRtc_Word32 a,
                                                          WebRtc_Word32 b,
                                                          WebRtc_Word32 c) {
    WebRtc_Word32 tmp;

    __asm__ __volatile__ (
        "precr_sra.ph.w     %[b],   %[a],   0       \n\t"
        "mulq_rs.w          %[tmp], %[b],   %[c]    \n\t"
        : [tmp] "=r" (tmp)
        : [a] "r" (a), [b] "r" (b), [c] "r" (c)
        : "hi", "lo"
    );

    return tmp;
}

static __inline WebRtc_Word16 WebRtcSpl_SatW32ToW16(WebRtc_Word32 value32) {

    __asm__ volatile (
        "shll_s.w   %[value32], %[value32], 16         \n\t"
        "sra        %[value32], %[value32], 16         \n\t"
        : [value32] "+r" (value32)
        :
    );

    WebRtc_Word16 out16 = (WebRtc_Word16) value32;

    return out16;
}

static __inline WebRtc_Word16 WebRtcSpl_AddSatW16(WebRtc_Word16 a,
                                                  WebRtc_Word16 b) {
    WebRtc_Word32 value32;

    __asm__ volatile (
        "addq_s.ph      %[value32],     %[a],   %[b]    \n\t"
        : [value32] "=r" (value32)
        : [a] "r" (a), [b] "r" (b)
    );

    return value32;
}

static __inline WebRtc_Word32 WebRtcSpl_AddSatW32(WebRtc_Word32 l_var1,
                                                  WebRtc_Word32 l_var2) {
    WebRtc_Word32 l_sum;

    __asm__ volatile (
        "addq_s.w   %[l_sum],       %[l_var1],      %[l_var2]    \n\t"
        : [l_sum] "=r" (l_sum)
        : [l_var1] "r" (l_var1), [l_var2] "r" (l_var2)
    );

    return l_sum;
}

static __inline WebRtc_Word16 WebRtcSpl_SubSatW16(WebRtc_Word16 var1,
                                                  WebRtc_Word16 var2) {
    WebRtc_Word32 value32;

    __asm__ volatile (
        "subq_s.ph  %[value32], %[var1],    %[var2]     \n\t"
        : [value32] "=r" (value32)
        : [var1] "r" (var1), [var2] "r" (var2)
    );

    return (WebRtc_Word16) value32;
}

static __inline WebRtc_Word32 WebRtcSpl_SubSatW32(WebRtc_Word32 l_var1,
                                                  WebRtc_Word32 l_var2) {
    WebRtc_Word32 l_diff;

    __asm__ volatile (
        "subq_s.w   %[l_diff],      %[l_var1],      %[l_var2]    \n\t"
        : [l_diff] "=r" (l_diff)
        : [l_var1] "r" (l_var1), [l_var2] "r" (l_var2)
    );

    return l_diff;
}

static __inline WebRtc_Word16 WebRtcSpl_GetSizeInBits(WebRtc_UWord32 n) {
    int bits;
    int i32 = 32;

    __asm__ volatile (
        "clz    %[bits],    %[n]                    \n\t"
        "subu   %[bits],    %[i32],     %[bits]     \n\t"
        : [bits] "=&r" (bits)
        : [n] "r" (n), [i32] "r" (i32)
    );

    return bits;
}

static __inline int WebRtcSpl_NormW32(WebRtc_Word32 a) {
    int zeros;

    __asm__ volatile (
        ".set push                                      \n\t"
        ".set noreorder                                 \n\t"
        "bnez       %[a],       1f                      \n\t"
        " sra       %[zeros],   %[a],       31          \n\t"
        "b          2f                                  \n\t"
        " move      %[zeros],   $zero                   \n\t"
        "1:                                             \n\t"
        "xor        %[zeros],   %[a],       %[zeros]    \n\t"
        "clz        %[zeros],   %[zeros]                \n\t"
        "addiu      %[zeros],   %[zeros],   -1          \n\t"
        "2:                                             \n\t"
        ".set pop                                       \n\t"
        : [zeros]"=&r"(zeros)
        : [a] "r" (a)
    );

    return zeros;
}

static __inline int WebRtcSpl_NormU32(WebRtc_UWord32 a) {
    int zeros;

    __asm__ volatile (
        "clz    %[zeros],   %[a]    \n\t"
        : [zeros] "=r" (zeros)
        : [a] "r" (a)
    );

    return (zeros & 0x1f);
}

static __inline int WebRtcSpl_NormW16(WebRtc_Word16 a) {
    int zeros;
    int a0 = a << 16;

    __asm__ volatile (
        ".set push                                      \n\t"
        ".set noreorder                                 \n\t"
        "bnez       %[a0],      1f                      \n\t"
        " sra       %[zeros],   %[a0],      31          \n\t"
        "b          2f                                  \n\t"
        " move      %[zeros],   $zero                   \n\t"
        "1:                                             \n\t"
        "xor        %[zeros],   %[a0],      %[zeros]    \n\t"
        "clz        %[zeros],   %[zeros]                \n\t"
        "addiu      %[zeros],   %[zeros],   -1          \n\t"
        "2:                                             \n\t"
        ".set pop                                       \n\t"
        : [zeros]"=&r"(zeros)
        : [a0] "r" (a0)
    );

    return zeros;
}

static __inline int32_t WebRtc_MulAccumW16(int16_t a,
                                           int16_t b,
                                           int32_t c) {
    int32_t res, c1;
    __asm__ __volatile__ (
        "mul    %[res],     %[a],   %[b]    \n\t"
        "addu   %[c1],      %[c],   %[res]  \n\t"
        : [c1] "=r" (c1), [res] "=&r" (res)
        : [a] "r" (a), [b] "r" (b), [c] "r" (c)
        : "hi", "lo"
    );
    return (c1);
}
#endif  // WEBRTC_SPL_SPL_INL_MIPS_H_
