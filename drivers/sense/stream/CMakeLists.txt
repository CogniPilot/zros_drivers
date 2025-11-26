# Copyright (c) 2025 Croxel Inc.
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_ZROS_SENSE_STREAM_IMU imu)
add_subdirectory_ifdef(CONFIG_ZROS_SENSE_STREAM_MAG mag)
