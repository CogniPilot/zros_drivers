# Copyright (c) 2025 Croxel Inc.
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_ZROS_SENSE_GNSS gnss)
add_subdirectory_ifdef(CONFIG_ZROS_SENSE_RTCM3_SUB rtcm3_sub)
add_subdirectory_ifdef(CONFIG_ZROS_SENSE_UWB uwb)

add_subdirectory_ifdef(CONFIG_ZROS_SENSE_STREAM stream)
