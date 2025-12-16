/*
 * Copyright (c) 2023 CogniPilot Foundation
 * Copyright (c) 2025 Croxel Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/gnss/rtk/rtk_publish.h>
#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_sub.h>

#include <synapse_topic_list.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sense_gnss, CONFIG_ZROS_SENSE_RTCM3_SUB_LOG_LEVEL);

typedef struct context {
	const char *name;
	struct zros_node node;
	struct zros_sub sub_rtcm3;
	synapse_pb_Rtcm3 data;
} context_t;

static context_t g_ctx = {
	.node = {},
	.sub_rtcm3 = {},
	.data = {},
};

static inline void send_rtcm3_frame(context_t *ctx)
{
	struct gnss_rtk_data rtcm3_frame = {
		.data = ctx->data.data.bytes,
		.len = ctx->data.data.size,
	};

	gnss_rtk_publish_data(&rtcm3_frame);
}

int sense_rtcm3_sub_fn(context_t *ctx)
{
	LOG_INF("init");
	zros_node_init(&ctx->node, ctx->name);
	zros_sub_init(&ctx->sub_rtcm3, &ctx->node, &topic_status, &ctx->data, 10);

	struct k_poll_event events[] = {
		*zros_sub_get_event(&ctx->sub_rtcm3),
	};

	while (true) {
		(void)k_poll(events, ARRAY_SIZE(events), K_FOREVER);
		if (zros_sub_update_available(&ctx->sub_rtcm3)) {
			zros_sub_update(&ctx->sub_rtcm3);
			send_rtcm3_frame(ctx);
		}
	}

	return 0;
}

K_THREAD_DEFINE(sense_rtcm3_sub, 2048, sense_rtcm3_sub_fn, &g_ctx, NULL, NULL, 10, 0, 0);
