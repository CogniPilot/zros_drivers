/*
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zros/perf_duration.h>

#include <synapse_topic_list.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imu_stream_combined, CONFIG_ZROS_SENSE_STREAM_IMU_LOG_LEVEL);

struct context {
	const char *name;
	struct zros_node node;
	struct zros_pub pub_imu;
	struct zros_topic *pub_topic;
	synapse_pb_Imu imu;
	struct zros_sub sub_imu;
};

struct context imu_stream_combined = {
	.name = "imu_stream_combined",
	.node = {},
	.pub_imu = {},
	.imu = {
		.has_stamp = true,
		.stamp = synapse_pb_Timestamp_init_default,
		.has_angular_velocity = true,
		.angular_velocity = synapse_pb_Vector3_init_default,
		.has_linear_acceleration = true,
		.linear_acceleration = synapse_pb_Vector3_init_default,
		.has_orientation = false,
	},
	.sub_imu = {},
};

static void imu_stream_combined_thread(void *arg0)
{
	struct context *ctx = (struct context *)arg0;
	int err;

	LOG_INF("init");
	zros_node_init(&ctx->node, ctx->name);
	zros_pub_init(&ctx->pub_imu, &ctx->node, &topic_imu, &ctx->imu);
	zros_sub_init(&ctx->sub_imu, &ctx->node, &topic_imu0, &ctx->imu, 1000);

	struct k_poll_event events[] = {
		*zros_sub_get_event(&ctx->sub_imu),
	};
	
	while (true) {
		err = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
		if (err != 0) {
			LOG_ERR("No IMU data coming through...");
		}

		if (zros_sub_update_available(&ctx->sub_imu)) {
			/** We're just passing-through IMU0 topic for now. We'd be able to
			 * improve this logic based on multiple IMUs publishing data.
			 */
			zros_sub_update(&ctx->sub_imu);
			zros_pub_update(&ctx->pub_imu);
		}
	}
}

K_THREAD_DEFINE(imu_stream_combined_id, 512, imu_stream_combined_thread,			   \
		&imu_stream_combined, NULL, NULL, 2, 0, 0);
