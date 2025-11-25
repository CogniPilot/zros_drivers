# Copyright (c) 2025 Croxel Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(zros_sense_stream_imu)
add_dependencies(zros_sense_stream_imu synapse_pb)
zephyr_library_sources(
	main.c
	imu_combined.c
)
