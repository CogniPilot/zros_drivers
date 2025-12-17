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

LOG_MODULE_REGISTER(sense_mag_stream, CONFIG_ZROS_SENSE_STREAM_MAG_LOG_LEVEL);

/** Borrowed from zephyr/dsp/util.h as it was erroring out due to a missing
 * zdsp backend. Should sort that out.
 */
#define Z_SHIFT_Q31_TO_F32(src, m) ((float32_t)(((int64_t)src) << m) / (float32_t)(1U << 31))

#define MAG_ALIAS(i) DT_ALIAS(_CONCAT(mag_stream_,i))

static struct context *get_context_from_iodev(const struct rtio_iodev *iodev);

struct context {
	const char *name;
	struct zros_node node;
	struct zros_pub pub_mag;
	struct zros_topic *pub_topic;
	synapse_pb_MagneticField mag;
	bool running;
	struct {
		double mag[3];
	} raw;
	struct {
		struct rtio *ctx;
		struct rtio_iodev *iodev;
	} stream;
};

static inline void avg_3_axis_q31(const struct sensor_three_axis_data *data, size_t count,
				  struct sensor_three_axis_data *out)
{
	int64_t avg[3] = {0};

	for (size_t i = 0 ; i < count ; i++) {
		avg[0] += data->readings[i].x;
		avg[1] += data->readings[i].y;
		avg[2] += data->readings[i].z;
	}
	avg[0] /= count;
	avg[1] /= count;
	avg[2] /= count;

	out->header.base_timestamp_ns = data->header.base_timestamp_ns;
	out->header.reading_count = 1;
	out->shift = data->shift;
	out->readings[0].x = avg[0];
	out->readings[0].y = avg[1];
	out->readings[0].z = avg[2];
}

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

static inline const struct sensor_three_axis_ref *get_axis_ref(struct rtio_iodev *iodev)
{
	struct sensor_read_config *read_config = (struct sensor_read_config *)iodev->data;
	const struct device *sensor = read_config->sensor;
	const struct sensor_three_axis_ref *axis_ref;

	if (sensor_three_axis_ref_get(sensor, &axis_ref) < 0) {
		return NULL;
	}
	return axis_ref;
}

static int decode_data(struct rtio_iodev *iodev, uint8_t *buf, enum sensor_channel ch,
		       struct sensor_three_axis_data *out)
{
	const uint32_t dec_buf_size = sizeof(struct sensor_three_axis_data);
	uint8_t decoded_buffer[dec_buf_size];
	struct sensor_three_axis_data *dec_data = (struct sensor_three_axis_data *)decoded_buffer;
	uint32_t fit = 0;
	uint16_t frame_count;
	int ret;

	const struct sensor_decoder_api *decoder = get_decoder(iodev);
	const struct sensor_three_axis_ref *axis_ref = get_axis_ref(iodev);

	if (!decoder) {
		return -EIO;
	}
	ret = decoder->get_frame_count(buf, (struct sensor_chan_spec) {ch, 0}, &frame_count);
	if (ret < 0) {
		return ret;
	}
	ret = decoder->decode(buf, (struct sensor_chan_spec) {ch, 0}, &fit, 1, decoded_buffer);
	if (ret < 0) {
		return ret;
	}
	avg_3_axis_q31(dec_data, ret, out);
	if (axis_ref) {
		sensor_three_axis_ref_align(axis_ref, out);
	}

	return 0;
}

static void mag_publish(struct context *ctx)
{
	stamp_msg(&ctx->mag.stamp, k_uptime_ticks());
	ctx->mag.magnetic_field.x = ctx->raw.mag[0];
	ctx->mag.magnetic_field.y = ctx->raw.mag[1];
	ctx->mag.magnetic_field.z = ctx->raw.mag[2];
	zros_pub_update(&ctx->pub_mag);
}

static void process_events(int result, uint8_t *buf, uint32_t len, void *userdata)
{
	struct rtio_iodev *iodev = (struct rtio_iodev *)userdata;
	struct context *ctx = get_context_from_iodev(iodev);
	struct sensor_three_axis_data mag_avg_data;
	int ret;

	if (result < 0) {
		LOG_ERR("%s: Failed event: %d", ctx->name, result);
		ctx->running = false;
		return;
	}

	ret = decode_data(iodev, buf, SENSOR_CHAN_MAGN_XYZ, &mag_avg_data);
	if (ret < 0) {
		LOG_ERR("%s: Failed to decode mag: %d", ctx->name, ret);
		ctx->running = false;
		return;
	}

	for (size_t i = 0 ; i < 3 ; i++) {
		ctx->raw.mag[i] = Z_SHIFT_Q31_TO_F32(mag_avg_data.readings[0].values[i],
						     mag_avg_data.shift);
	}
	mag_publish(ctx);
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
		rtio_sqe_reset_all(ctx->stream.ctx);
		return err;
	}
	ctx->running = true;

	return 0;
}

static void mag_stream_thread(void *arg0)
{
	struct context *ctx = (struct context *)arg0;
	int err;

	LOG_INF("init");
	zros_node_init(&ctx->node, ctx->name);
	zros_pub_init(&ctx->pub_mag, &ctx->node, ctx->pub_topic, &ctx->mag);

	err = setup_stream(ctx);
	__ASSERT(!err, "Failed to start stream for %p", ctx->stream.iodev);

	while (true) {
		err = sensor_processing_cb_with_timeout(ctx->stream.ctx, process_events,
							K_MSEC(1000));
		if (err != 0 || !ctx->running) {
			ctx->running = false;
			LOG_ERR("Error during stream. Attempting recovery...");
			do {
				/* TODO: Decide when we've tried too much. */
				k_sleep(K_MSEC(1000));
				(void)setup_stream(ctx);
			} while (!ctx->running);
		}
	}
}

#define MAG_STREAM_DEFINE(i)									   \
												   \
RTIO_DEFINE_WITH_MEMPOOL(mag_stream_ctx_##i, 2, 4, 128, 4, sizeof(void *));			   \
SENSOR_DT_STREAM_IODEV(mag_stream_iodev_##i, MAG_ALIAS(i),					   \
		       {SENSOR_TRIG_DATA_READY, SENSOR_STREAM_DATA_INCLUDE});			   \
												   \
static struct context mag_stream_context_##i = {						   \
	.name = STRINGIFY(sense_mag_##i),							   \
	.node = {},										   \
	.pub_mag = {},										   \
	.pub_topic = &topic_magnetic_field,							   \
	.mag = {										   \
		.has_stamp = true,								   \
		.stamp = synapse_pb_Timestamp_init_default,					   \
		.frame_id = "base_link",							   \
		.magnetic_field = synapse_pb_Vector3_init_default,				   \
		.has_magnetic_field = true,							   \
		.magnetic_field_covariance = {},						   \
		.magnetic_field_covariance_count = 0,						   \
	},											   \
	.running = false,									   \
	.stream = {										   \
		.ctx = &mag_stream_ctx_##i,							   \
		.iodev = &mag_stream_iodev_##i,							   \
	},											   \
};												   \
												   \
K_THREAD_DEFINE(mag_stream_thread_##i##_id, 1024, mag_stream_thread,				   \
		&mag_stream_context_##i, NULL, NULL, 4, 0, 0)

#define MAG_STREAM_DEFINE_IF_EXISTS(i)								   \
	IF_ENABLED(DT_NODE_EXISTS(MAG_ALIAS(i)), (MAG_STREAM_DEFINE(i)))

MAG_STREAM_DEFINE_IF_EXISTS(0);

#define MAG_STREAM_CTX_LIST									   \
		IF_ENABLED(DT_NODE_EXISTS(MAG_ALIAS(0)), (&mag_stream_context_0,))

struct context * const mag_list[] = {
		MAG_STREAM_CTX_LIST
};

static struct context *get_context_from_iodev(const struct rtio_iodev *iodev)
{
	for (size_t i = 0 ; i < ARRAY_SIZE(mag_list) ; i++) {
		if (mag_list[i] != NULL && mag_list[i]->stream.iodev == iodev) {
			return mag_list[i];
		}
	}
	CODE_UNREACHABLE;
}
