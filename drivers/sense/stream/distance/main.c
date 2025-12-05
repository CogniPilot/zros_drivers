/*
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/perf_duration.h>

#include <synapse_topic_list.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

LOG_MODULE_REGISTER(sense_distance_stream, CONFIG_ZROS_SENSE_STREAM_DISTANCE_LOG_LEVEL);

/** Borrowed from zephyr/dsp/util.h as it was erroring out due to a missing
 * zdsp backend. Should sort that out.
 */
#define Z_SHIFT_Q31_TO_F32(src, m) ((float32_t)(((int64_t)src) << m) / (float32_t)(1U << 31))

#define DISTANCE_ALIAS(i) DT_ALIAS(_CONCAT(distance_stream_,i))

static struct context *get_context_from_iodev(const struct rtio_iodev *iodev);

struct context {
	const char *name;
	struct zros_node node;
	struct zros_pub pub;
	struct zros_topic *pub_topic;
	bool running;
	synapse_pb_Distance data;
	struct {
		double distance;
	} raw;
	struct {
		struct rtio *ctx;
		struct rtio_iodev *iodev;
	} stream;
};

static inline const struct sensor_decoder_api *get_decoder(struct rtio_iodev *iodev)
{
	struct sensor_read_config *read_config = (struct sensor_read_config *)iodev->data;
	const struct device *sensor = read_config->sensor;
	const struct sensor_decoder_api *decoder;

	if (sensor_get_decoder(sensor, &decoder) < 0) {
		return NULL;
	}
	return decoder;
}

static int decode_data(struct rtio_iodev *iodev, uint8_t *buf, enum sensor_channel ch,
		       struct sensor_q31_data *out)
{
	struct sensor_q31_data *dec_data = (struct sensor_q31_data *)out;
	uint32_t fit = 0;
	uint16_t frame_count;
	int ret;

	const struct sensor_decoder_api *decoder = get_decoder(iodev);

	if (!decoder) {
		return -EIO;
	}
	ret = decoder->get_frame_count(buf, (struct sensor_chan_spec) {ch, 0}, &frame_count);
	if (ret < 0) {
		return ret;
	}
	ret = decoder->decode(buf, (struct sensor_chan_spec) {ch, 0}, &fit, 1, dec_data);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static void distance_publish(struct context *ctx)
{
	stamp_msg(&ctx->data.stamp, k_uptime_ticks());
	ctx->data.distance = ctx->raw.distance;
	zros_pub_update(&ctx->pub);
}

static void process_events(int result, uint8_t *buf, uint32_t len, void *userdata)
{
	struct rtio_iodev *iodev = (struct rtio_iodev *)userdata;
	struct context *ctx = get_context_from_iodev(iodev);
	struct sensor_q31_data data;
	int ret;

	if (result < 0) {
		LOG_ERR("%s: Failed event: %d", ctx->name, result);
		ctx->running = false;
		return;
	}

	ret = decode_data(iodev, buf, SENSOR_CHAN_DISTANCE, &data);
	if (ret < 0) {
		LOG_ERR("%s: Failed to decode data: %d", ctx->name, ret);
		ctx->running = false;
		return;
	}
	ctx->raw.distance = Z_SHIFT_Q31_TO_F32(data.readings[0].value, data.shift);
	distance_publish(ctx);
}

static int setup_stream(struct context *ctx)
{
	struct rtio *rtio_ctx = ctx->stream.ctx;
	struct rtio_iodev *iodev = ctx->stream.iodev;
	int err;

	LOG_INF("setting up stream %p...", iodev);

	err = sensor_stream(iodev, rtio_ctx, (void *)iodev, NULL);
	if (err != 0) {
		LOG_ERR("Failed to start sensor-stream: %d, %p...", err, iodev);
		return err;
	}
	ctx->running = true;

	return 0;
}

static void distance_stream_thread(void *arg0)
{
	struct context *ctx = (struct context *)arg0;
	int err;

	LOG_INF("init");
	zros_node_init(&ctx->node, ctx->name);
	zros_pub_init(&ctx->pub, &ctx->node, ctx->pub_topic, &ctx->data);

	err = setup_stream(ctx);
	__ASSERT(!err, "Failed to start stream for %p", ctx->stream.iodev);

	while (true) {
		err = sensor_processing_cb_with_timeout(ctx->stream.ctx, process_events,
							K_MSEC(1000));
		if (err != 0 || !ctx->running) {
			ctx->running = false;
			LOG_ERR("Error during stream. Attempting recovery...");
			rtio_sqe_drop_all(ctx->stream.ctx);
			do {
				/* TODO: Decide when we've tried too much. */
				(void)setup_stream(ctx);
				k_sleep(K_MSEC(1000));
			} while (!ctx->running);
		}
	}
}

#define DIST_STREAM_DEFINE(i)									   \
												   \
RTIO_DEFINE_WITH_MEMPOOL(dist_stream_ctx_##i, 2, 4, 256, 8, sizeof(void *));			   \
SENSOR_DT_STREAM_IODEV(dist_stream_iodev_##i, DISTANCE_ALIAS(i),				   \
		       {SENSOR_TRIG_DATA_READY, SENSOR_STREAM_DATA_INCLUDE});			   \
												   \
static struct context dist_stream_context_##i = {						   \
	.name = STRINGIFY(sense_dist_##i),							   \
	.node = {},										   \
	.pub = {},										   \
	.pub_topic = &topic_distance,								   \
	.running = false,									   \
	.data = {										   \
		.has_stamp = true,								   \
		.stamp = synapse_pb_Timestamp_init_default,					   \
	},											   \
	.stream = {										   \
		.ctx = &dist_stream_ctx_##i,							   \
		.iodev = &dist_stream_iodev_##i,						   \
	},											   \
};												   \
												   \
K_THREAD_DEFINE(dist_stream_thread_##i##_id, 1024, distance_stream_thread,			   \
		&dist_stream_context_##i, NULL, NULL, 2, 0, 0)

#define DIST_STREAM_DEFINE_IF_EXISTS(i)								   \
	IF_ENABLED(DT_NODE_EXISTS(DISTANCE_ALIAS(i)), (DIST_STREAM_DEFINE(i)))

DIST_STREAM_DEFINE_IF_EXISTS(0);

#define DIST_STREAM_CTX_LIST									   \
		IF_ENABLED(DT_NODE_EXISTS(DISTANCE_ALIAS(0)), (&dist_stream_context_0,))

struct context * const dist_list[] = {
		DIST_STREAM_CTX_LIST
};

static struct context *get_context_from_iodev(const struct rtio_iodev *iodev)
{
	for (size_t i = 0 ; i < ARRAY_SIZE(dist_list) ; i++) {
		if (dist_list[i] != NULL && dist_list[i]->stream.iodev == iodev) {
			return dist_list[i];
		}
	}
	CODE_UNREACHABLE;
}
