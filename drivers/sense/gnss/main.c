/*
 * Copyright (c) 2023 CogniPilot Foundation
 * Copyright (c) 2025 Croxel Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gnss.h>
#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>

#include <synapse_topic_list.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sense_gnss, CONFIG_ZROS_SENSE_GNSS_LOG_LEVEL);

typedef struct context {
	struct zros_node node;
	struct zros_pub pub;
	synapse_pb_NavSatFix data;
	int32_t gMeasurementPeriodMs;
	bool running;
	bool isAlive;
} context_t;

#define GNSS_MODEM DEVICE_DT_GET(DT_ALIAS(gnss))

static context_t g_ctx = {
	.node = {},
	.pub = {},
	.data = {
		.has_stamp = true,
		.stamp = synapse_pb_Timestamp_init_default,
		.altitude = 0,
		.latitude = 0,
		.longitude = 0,
		.position_covariance = synapse_pb_Covariance3_init_default,
		.position_covariance_type = synapse_pb_NavSatFix_CovarianceType_UNKNOWN,
		.status = {.service = 0, .status = 0},
	},
	.running = false,
	.isAlive = false,
};

static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	/** It is possible that the GNSS driver starts publishing data but channel isn't
	 * initialized yet.
	 */
	if (!g_ctx.running) {
		return;
	}

	stamp_msg(&g_ctx.data.stamp, k_uptime_ticks());
	g_ctx.isAlive = true;

	if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
		g_ctx.data.latitude = data->nav_data.latitude / 1e9;
		g_ctx.data.longitude = data->nav_data.longitude / 1e9;
		g_ctx.data.altitude = data->nav_data.altitude / 1e3;
		zros_pub_update(&g_ctx.pub);
		LOG_DBG("lat %f long %f\n", g_ctx.data.latitude, g_ctx.data.longitude);
	} else {
		LOG_WRN("gnss update without fix. SV tracked: %d", data->info.satellites_cnt);
	}
}
GNSS_DATA_CALLBACK_DEFINE(GNSS_MODEM, gnss_data_cb);

static int gnss_node_init(void)
{
	int err;

	zros_node_init(&g_ctx.node, "sense_gnss");
	zros_pub_init(&g_ctx.pub, &g_ctx.node, &topic_nav_sat_fix, &g_ctx.data);

	if (!device_is_ready(GNSS_MODEM)) {
		return -EIO;
	}
	g_ctx.running = true;

	err = gnss_get_fix_rate(GNSS_MODEM, &g_ctx.gMeasurementPeriodMs);
	if (err != 0) {
		return -EIO;
	}

	return 0;
}

SYS_INIT(gnss_node_init, APPLICATION, 99);
