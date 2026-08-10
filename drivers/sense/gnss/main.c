/*
 * Copyright (c) 2023 CogniPilot Foundation
 * Copyright (c) 2025 Croxel Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>

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
	synapse_topic_GnssFix_t data;
	int32_t gMeasurementPeriodMs;
	bool running;
	bool isAlive;
} context_t;

#define GNSS_MODEM DEVICE_DT_GET(DT_ALIAS(gnss))

static context_t g_ctx = {
	.node = {},
	.pub = {},
	.data = {},
	.running = false,
	.isAlive = false,
};

/* Narrow a wider unsigned quantity to uint16_t, saturating instead of wrapping. */
static uint16_t sat_u16(uint32_t v)
{
	return (v > UINT16_MAX) ? (uint16_t)UINT16_MAX : (uint16_t)v;
}

/*
 * Collapse the Zephyr fix quality onto the wire GnssFixType. The u-blox common
 * layer folds the UBX 2D and 3D solutions into GNSS_SPS, so a plain fix is
 * reported as FIX_3D; the RTK, DGNSS and dead-reckoning states carry through.
 */
static uint8_t gnss_fix_type(enum gnss_fix_quality quality)
{
	switch (quality) {
	case GNSS_FIX_QUALITY_GNSS_SPS:
	case GNSS_FIX_QUALITY_GNSS_PPS:
		return SYNAPSE_TYPES_GNSS_FIX_TYPE_FIX_3D;
	case GNSS_FIX_QUALITY_DGNSS:
		return SYNAPSE_TYPES_GNSS_FIX_TYPE_DGNSS;
	case GNSS_FIX_QUALITY_RTK:
		return SYNAPSE_TYPES_GNSS_FIX_TYPE_RTK_FIXED;
	case GNSS_FIX_QUALITY_FLOAT_RTK:
		return SYNAPSE_TYPES_GNSS_FIX_TYPE_RTK_FLOAT;
	case GNSS_FIX_QUALITY_ESTIMATED:
		return SYNAPSE_TYPES_GNSS_FIX_TYPE_DEAD_RECKONING;
	default:
		return SYNAPSE_TYPES_GNSS_FIX_TYPE_NO_FIX;
	}
}

static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	/** It is possible that the GNSS driver starts publishing data but channel isn't
	 * initialized yet.
	 */
	if (!g_ctx.running) {
		return;
	}

	g_ctx.isAlive = true;

	if (data->info.fix_status == GNSS_FIX_STATUS_NO_FIX) {
		LOG_WRN("gnss update without fix. SV tracked: %d", data->info.satellites_cnt);
		return;
	}

	synapse_topic_GnssFix_t *fix = &g_ctx.data;

	memset(fix, 0, sizeof(*fix));

	/* Boot-clock timestamp; the shared gPTP timescale is not resolved here yet. */
	fix->timestamp_ns = k_ticks_to_ns_floor64(k_uptime_ticks());
	fix->time_status = SYNAPSE_TYPES_TIME_STATUS_LOCAL_FREERUN;

	/* nav_data angles are nanodegrees; the wire uses degrees * 1e7 (nanodeg / 100). */
	fix->latitude_deg_e7 = (int32_t)(data->nav_data.latitude / 100);
	fix->longitude_deg_e7 = (int32_t)(data->nav_data.longitude / 100);

	/* altitude is millimeters above MSL; geoid separation carries it to the ellipsoid. */
	fix->altitude_msl_mm = data->nav_data.altitude;
	fix->altitude_ellipsoid_mm = data->nav_data.altitude + data->info.geoid_separation;

	/*
	 * The Zephyr GNSS API exposes no accuracy estimates, so the accuracy
	 * fields report their saturated maximum (worst case) rather than an
	 * unfounded zero that would claim a precise fix.
	 */
	fix->horizontal_accuracy_mm = UINT16_MAX;
	fix->vertical_accuracy_mm = UINT16_MAX;
	fix->velocity_accuracy_mm_s = UINT16_MAX;
	fix->yaw_accuracy_cdeg = UINT16_MAX;

	/* hdop carries the only DOP the driver reports (PDOP), in 1/1000; wire is 1/100. */
	fix->hdop_centi = sat_u16(data->info.hdop / 10);

	/* ground speed mm/s to cm/s; course over ground millidegrees to centidegrees. */
	fix->ground_speed_cm_s = sat_u16(data->nav_data.speed / 10);
	fix->course_over_ground_cdeg = sat_u16(data->nav_data.bearing / 10);

	fix->fix_type = gnss_fix_type(data->info.fix_quality);
	fix->satellites_used = (data->info.satellites_cnt > UINT8_MAX)
				       ? UINT8_MAX
				       : (uint8_t)data->info.satellites_cnt;

	/*
	 * Only course over ground is available. Vehicle yaw and vertical velocity
	 * are not surfaced by the Zephyr GNSS API, so those flags stay clear and
	 * their fields stay zero. time_unix_ns is left unset while the clock is
	 * free-running, so TIME_VALID stays clear as well.
	 */
	fix->flags = SYNAPSE_TOPIC_GNSS_FIX_FLAG_COURSE_VALID;

	/* Single GNSS instance on this node. */
	fix->id = 0U;

	zros_pub_update(&g_ctx.pub);
	LOG_DBG("lat %d lon %d msl %d mm", fix->latitude_deg_e7, fix->longitude_deg_e7,
		fix->altitude_msl_mm);
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
