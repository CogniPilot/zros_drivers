# Copyright (c) 2023, CogniPilot Foundation
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(zros_sense_rtcm3_sub)

zephyr_include_directories()
zephyr_library_sources(
	main.c
)

add_dependencies(zros_sense_rtcm3_sub synapse_pb)
