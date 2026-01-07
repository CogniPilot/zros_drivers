/*
 * Copyright (c) 2023 CogniPilot Foundation
 * Copyright (c) 2025 Croxel Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <time.h>

#include <zephyr/drivers/gnss.h>
#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>

#include <synapse_topic_list.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sense_gnss, CONFIG_ZROS_SENSE_GNSS_LOG_LEVEL);

/**
 * @brief Convert GNSS UTC time to Unix epoch seconds
 * @param utc Pointer to GNSS time structure
 * @return Unix epoch seconds, or 0 if conversion fails
 */
static int64_t gnss_time_to_epoch(const struct gnss_time *utc)
{
	struct tm tm = {
		.tm_sec = utc->millisecond / 1000,
		.tm_min = utc->minute,
		.tm_hour = utc->hour,
		.tm_mday = utc->month_day,
		.tm_mon = utc->month - 1,           /* tm_mon is 0-11 */
		.tm_year = utc->century_year + 100, /* tm_year is years since 1900 */
		.tm_isdst = 0,
	};
	time_t t = mktime(&tm);
	if (t == (time_t)-1) {
		return 0;
	}
	return (int64_t)t;
}

typedef struct context {
	struct zros_node node;
	struct zros_pub pub;
	synapse_pb_NavSatFix data;
	int32_t gMeasurementPeriodMs;
	bool running;
	bool isAlive;
	bool hadFix;
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
	.hadFix = false,
};

static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	/** It is possible that the GNSS driver starts publishing data but channel isn't
	 * initialized yet.
	 */
	if (!g_ctx.running) {
		return;
	}

	g_ctx.isAlive = true;

	if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
		if (!g_ctx.hadFix) {
			LOG_INF("GNSS fix acquired. SV tracked: %d", data->info.satellites_cnt);
			g_ctx.hadFix = true;
		}
		/* Use actual GPS UTC time for timestamp */
		g_ctx.data.stamp.seconds = gnss_time_to_epoch(&data->utc);
		g_ctx.data.stamp.nanos = (data->utc.millisecond % 1000) * 1000000;

		g_ctx.data.latitude = data->nav_data.latitude / 1e9;
		g_ctx.data.longitude = data->nav_data.longitude / 1e9;
		g_ctx.data.altitude = data->nav_data.altitude / 1e3;
		zros_pub_update(&g_ctx.pub);
		LOG_DBG("lat %f long %f\n", g_ctx.data.latitude, g_ctx.data.longitude);
	} else {
		if (g_ctx.hadFix) {
			LOG_WRN("GNSS fix lost. SV tracked: %d", data->info.satellites_cnt);
		}
		g_ctx.hadFix = false;
		LOG_INF_RATELIMIT_RATE(30000, "waiting for fix. SV tracked: %d",
				       data->info.satellites_cnt);
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
