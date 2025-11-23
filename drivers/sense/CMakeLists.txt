# Copyright (c) 2025 Croxel Inc.
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_ZROS_SENSE_BARO baro)
add_subdirectory_ifdef(CONFIG_ZROS_SENSE_GNSS gnss)
add_subdirectory_ifdef(CONFIG_ZROS_SENSE_IMU imu)
add_subdirectory_ifdef(CONFIG_ZROS_SENSE_MAG mag)
