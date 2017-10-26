# Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
#
# Use of this source code is governed by a BSD-style license
# that can be found in the LICENSE file in the root of the source
# tree. An additional intellectual property rights grant can be found
# in the file PATENTS.  All contributing project authors may
# be found in the AUTHORS file in the root of the source tree.

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

include $(LOCAL_PATH)/../../../android-webrtc.mk

LOCAL_ARM_MODE := arm
LOCAL_MODULE := libwebrtc_apm
LOCAL_VENDOR_MODULE := true
LOCAL_MODULE_TAGS := optional
LOCAL_CPP_EXTENSION := .cc
LOCAL_SRC_FILES := \
    $(call all-proto-files-under, .) \
    audio_buffer.cc \
    audio_processing_impl.cc \
    echo_cancellation_impl.cc \
    echo_control_mobile_impl.cc \
    gain_control_impl.cc \
    high_pass_filter_impl.cc \
    level_estimator_impl.cc \
    noise_suppression_impl.cc \
    rms_level.cc \
    splitting_filter.cc \
    three_band_filter_bank.cc \
    processing_component.cc \
    voice_detection_impl.cc

# Flags passed to both C and C++ files.
LOCAL_CFLAGS := \
    $(MY_WEBRTC_COMMON_DEFS) \
    '-DWEBRTC_NS_FIXED' \
    '-DWEBRTC_ANDROID_PLATFORM_BUILD' \
    '-DWEBRTC_AUDIOPROC_DEBUG_DUMP'
#   floating point
#   -DWEBRTC_NS_FLOAT'

LOCAL_CFLAGS_arm := $(MY_WEBRTC_COMMON_DEFS_arm)
LOCAL_CFLAGS_x86 := $(MY_WEBRTC_COMMON_DEFS_x86)
LOCAL_CFLAGS_mips := $(MY_WEBRTC_COMMON_DEFS_mips)
LOCAL_CFLAGS_arm64 := $(MY_WEBRTC_COMMON_DEFS_arm64)
LOCAL_CFLAGS_x86_64 := $(MY_WEBRTC_COMMON_DEFS_x86_64)
LOCAL_CFLAGS_mips64 := $(MY_WEBRTC_COMMON_DEFS_mips64)

LOCAL_CFLAGS += $(MY_WEBRTC_COMMON_CFLAGS_WARNINGS)
LOCAL_TIDY_CHECKS := $(MY_WEBRTC_COMMON_TIDY_CHECKS)

LOCAL_C_INCLUDES := \
    $(LOCAL_PATH)/include \
    $(LOCAL_PATH)/aec/include \
    $(LOCAL_PATH)/aecm/include \
    $(LOCAL_PATH)/agc/include \
    $(LOCAL_PATH)/ns/include \
    $(LOCAL_PATH)/../interface \
    $(LOCAL_PATH)/../../.. \
    $(LOCAL_PATH)/../../common_audio/signal_processing/include \
    $(LOCAL_PATH)/../../common_audio/vad/include \
    $(LOCAL_PATH)/../../system_wrappers/include \
    external/protobuf/src

include $(BUILD_STATIC_LIBRARY)

# apm process test app

include $(CLEAR_VARS)

LOCAL_MODULE_TAGS := tests
LOCAL_CPP_EXTENSION := .cc
LOCAL_SRC_FILES:= \
    $(call all-proto-files-under, .) \
    test/process_test.cc \
    test/test_utils.cc \
    test/protobuf_utils.cc \
    ../../common_audio/wav_file.cc \
    ../../common_audio/wav_header.cc \
    ../../system_wrappers/source/tick_util.cc \
    ../../test/testsupport/fileutils.cc \
    ../../test/testsupport/perf_test.cc \

# Flags passed to both C and C++ files.
LOCAL_CFLAGS := \
    $(MY_WEBRTC_COMMON_DEFS) \
    '-DWEBRTC_ANDROID_PLATFORM_BUILD' \
    '-DWEBRTC_AUDIOPROC_DEBUG_DUMP'

LOCAL_CFLAGS_arm := $(MY_WEBRTC_COMMON_DEFS_arm)
LOCAL_CFLAGS_x86 := $(MY_WEBRTC_COMMON_DEFS_x86)
LOCAL_CFLAGS_mips := $(MY_WEBRTC_COMMON_DEFS_mips)
LOCAL_CFLAGS_arm64 := $(MY_WEBRTC_COMMON_DEFS_arm64)
LOCAL_CFLAGS_x86_64 := $(MY_WEBRTC_COMMON_DEFS_x86_64)
LOCAL_CFLAGS_mips64 := $(MY_WEBRTC_COMMON_DEFS_mips64)

LOCAL_CFLAGS += $(MY_WEBRTC_COMMON_CFLAGS_WARNINGS)
LOCAL_TIDY_CHECKS := $(MY_WEBRTC_COMMON_TIDY_CHECKS)

LOCAL_C_INCLUDES := \
    $(LOCAL_PATH)/include \
    $(LOCAL_PATH)/../interface \
    $(LOCAL_PATH)/../../.. \
    $(LOCAL_PATH)/../../system_wrappers/include \

LOCAL_SHARED_LIBRARIES := \
    libutils \
    libwebrtc_audio_preprocessing \

LOCAL_VENDOR_MODULE := true
LOCAL_MODULE := webrtc_apm_process_test

LOCAL_SHARED_LIBRARIES += libprotobuf-cpp-lite

include $(BUILD_NATIVE_TEST)

# apm unit test app

include $(CLEAR_VARS)

LOCAL_MODULE_TAGS := tests
LOCAL_CPP_EXTENSION := .cc
LOCAL_SRC_FILES:= \
    $(call all-proto-files-under, .) \
    test/audio_processing_unittest.cc \
    test/protobuf_utils.cc \
    test/test_utils.cc \
    ../../common_audio/resampler/push_resampler.cc \
    ../../test/testsupport/fileutils.cc

LOCAL_C_INCLUDES := \
    $(LOCAL_PATH)/../../.. \

# Flags passed to both C and C++ files.
LOCAL_CFLAGS := \
    $(MY_WEBRTC_COMMON_DEFS) \
    '-DWEBRTC_AUDIOPROC_FIXED_PROFILE' \
    '-DWEBRTC_ANDROID_PLATFORM_BUILD' \
    '-DWEBRTC_AUDIOPROC_DEBUG_DUMP' \

LOCAL_CFLAGS_arm := $(MY_WEBRTC_COMMON_DEFS_arm)
LOCAL_CFLAGS_x86 := $(MY_WEBRTC_COMMON_DEFS_x86)
LOCAL_CFLAGS_mips := $(MY_WEBRTC_COMMON_DEFS_mips)
LOCAL_CFLAGS_arm64 := $(MY_WEBRTC_COMMON_DEFS_arm64)
LOCAL_CFLAGS_x86_64 := $(MY_WEBRTC_COMMON_DEFS_x86_64)
LOCAL_CFLAGS_mips64 := $(MY_WEBRTC_COMMON_DEFS_mips64)

LOCAL_CFLAGS += $(MY_WEBRTC_COMMON_CFLAGS_WARNINGS)
LOCAL_TIDY_CHECKS := $(MY_WEBRTC_COMMON_TIDY_CHECKS)

LOCAL_C_INCLUDES := \
    $(LOCAL_PATH)/include \
    $(LOCAL_PATH)/../interface \
    $(LOCAL_PATH)/../.. \
    $(LOCAL_PATH)/../../.. \
    $(LOCAL_PATH)/../../../test \
    $(LOCAL_PATH)/../../system_wrappers/include \
    $(LOCAL_PATH)/../../../webrtc/base \
    $(LOCAL_PATH)/../../common_audio/signal_processing/include \
    external/protobuf/src

LOCAL_SHARED_LIBRARIES := \
    libwebrtc_audio_preprocessing

LOCAL_VENDOR_MODULE := true
LOCAL_MODULE := webrtc_apm_unit_test

LOCAL_SHARED_LIBRARIES += libprotobuf-cpp-lite

include $(BUILD_NATIVE_TEST)
