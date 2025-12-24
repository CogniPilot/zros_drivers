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

LOG_MODULE_REGISTER(sense_imu_stream, CONFIG_ZROS_SENSE_STREAM_IMU_LOG_LEVEL);

/** Borrowed from zephyr/dsp/util.h as it was erroring out due to a missing
 * zdsp backend. Should sort that out.
 */
#define Z_SHIFT_Q31_TO_F32(src, m) ((float32_t)(((int64_t)src) << m) / (float32_t)(1U << 31))

#define ACCEL_G ((double)SENSOR_G / 1000000)

#define IMU_STREAM_CALIBRATION_COUNT 100

#define IMU_ALIAS(i) DT_ALIAS(_CONCAT(imu_stream_,i))

static struct context *get_context_from_iodev(const struct rtio_iodev *iodev);

enum sense_imu_stream_calibration_st {
	SENSE_IMU_STREAM_UNCALIBRATED,
	SENSE_IMU_STREAM_CALIBRATING,
	SENSE_IMU_STREAM_CALIBRATED,
};

enum sense_imu_stream_status {
	SENSE_IMU_STREAM_STATUS_ACTIVE,
	SENSE_IMU_STREAM_STATUS_FAILED,
};

struct context {
	const char *name;
	struct zros_node node;
	struct zros_pub pub_imu;
	struct zros_topic *pub_topic;
	synapse_pb_Imu imu;
	struct zros_sub sub_status;
	synapse_pb_Status status;
	synapse_pb_Status_Mode last_mode;
	bool running;
	struct {
		enum sense_imu_stream_calibration_st state;
		double accel_scale;
		struct {
			double accel[IMU_STREAM_CALIBRATION_COUNT][3];
			double gyro[IMU_STREAM_CALIBRATION_COUNT][3];
		} samples;
		struct {
			double accel[3];
			double gyro[3];
		} bias;
		size_t count;
	} calibration;
	struct {
		double accel[3];
		double gyro[3];
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

static void feed_calibration(struct context *ctx,
			     const struct sensor_three_axis_data *accel,
			     const struct sensor_three_axis_data *gyro)
{
	/** Unless it's in the process of calibrating, we'll assume calling this function
	 * implies starting another calibration session.
	 */
	if (ctx->calibration.state != SENSE_IMU_STREAM_CALIBRATING) {
		memset(&ctx->calibration.bias, 0, sizeof(ctx->calibration.bias));
		ctx->calibration.count = 0;
		ctx->calibration.state = SENSE_IMU_STREAM_CALIBRATING;
	}

	for (size_t i = 0 ; i < ARRAY_SIZE(ctx->calibration.bias.accel) ; i++) {
		ctx->calibration.samples.accel[ctx->calibration.count][i] =
			(double)Z_SHIFT_Q31_TO_F32(accel->readings[0].values[i], accel->shift);
	}
	for (size_t i = 0 ; i < ARRAY_SIZE(ctx->calibration.bias.gyro) ; i++) {
		ctx->calibration.samples.gyro[ctx->calibration.count][i] =
			(double)Z_SHIFT_Q31_TO_F32(gyro->readings[0].values[i], gyro->shift);
	}
	ctx->calibration.count++;

	/* It will require N iterations before moving on to calculate calibration statistics */
	if (ctx->calibration.count < IMU_STREAM_CALIBRATION_COUNT) {
		return;
	}

	/* We have enough samples. Proceed to calculate statistics and finish calibration */
	double accel_mean[3] = {0};
	double gyro_mean[3] = {0};
	double accel_std[3] = {0};
	double gyro_std[3] = {0};
	bool calibration_ok = true;

	for (size_t i = 0 ; i < IMU_STREAM_CALIBRATION_COUNT ; i++) {
		for (size_t j = 0 ; j < ARRAY_SIZE(accel_mean) ; j++) {
			accel_mean[j] +=
				ctx->calibration.samples.accel[i][j] / IMU_STREAM_CALIBRATION_COUNT;
		}
		for (size_t j = 0 ; j < ARRAY_SIZE(gyro_mean) ; j++) {
			gyro_mean[j] +=
				ctx->calibration.samples.gyro[i][j] / IMU_STREAM_CALIBRATION_COUNT;
		}
	}

	for (size_t i = 0 ; i < IMU_STREAM_CALIBRATION_COUNT ; i++) {
		for (size_t j = 0; j < ARRAY_SIZE(accel_std) ; j++) {
			double e = ctx->calibration.samples.accel[i][j] - accel_mean[j];

			accel_std[j] += e * e;
		}
		for (size_t j = 0; j < ARRAY_SIZE(gyro_std) ; j++) {
			double e = ctx->calibration.samples.gyro[i][j] - gyro_mean[j];

			gyro_std[j] += e * e;
		}
	}

	for (size_t i = 0 ; i < ARRAY_SIZE(accel_std) ; i++) {
		accel_std[i] = sqrt(accel_std[i] / IMU_STREAM_CALIBRATION_COUNT);
	}
	for (size_t i = 0 ; i < ARRAY_SIZE(gyro_std) ; i++) {
		gyro_std[i] = sqrt(gyro_std[i] / IMU_STREAM_CALIBRATION_COUNT);
	}
	double accel_magnitude = sqrt(accel_mean[0] * accel_mean[0] +
				      accel_mean[1] * accel_mean[1] +
				      accel_mean[2] * accel_mean[2]);

	if (accel_magnitude < 8.0 || accel_magnitude > 11.0) {
		LOG_WRN("%s: accel magnitude out of range: %10.4f (expected ~9.8)",
			ctx->name, accel_magnitude);
		calibration_ok = false;
	}

	for (size_t i = 0 ; i < ARRAY_SIZE(gyro_std) ; i++) {
		if (gyro_std[i] > 0.1) {
			LOG_WRN("%s: gyro axis %d too noisy: std=%10.4f", ctx->name,
				i, gyro_std[i]);
			calibration_ok = false;
		}
	}

	if (!calibration_ok) {
		LOG_WRN("%s: Calibration failed. Retrying...", ctx->name);
		ctx->calibration.state = SENSE_IMU_STREAM_UNCALIBRATED;
		return;
	}

	LOG_INF("%s: Calibration completed", ctx->name);
	ctx->calibration.bias.accel[0] = accel_mean[0];
	ctx->calibration.bias.accel[1] = accel_mean[1];
	ctx->calibration.bias.accel[2] = 0;
	ctx->calibration.accel_scale = accel_magnitude / ACCEL_G;
	LOG_INF("%s: accel", ctx->name);
	LOG_INF("%s: mean: %10.4f %10.4f %10.4f", ctx->name,
		accel_mean[0], accel_mean[1], accel_mean[2]);
	LOG_INF("%s: std: %10.4f %10.4f %10.4f", ctx->name,
		accel_std[0], accel_std[1], accel_std[2]);
	LOG_INF("%s: scale %10.4f", ctx->name, ctx->calibration.accel_scale);

	LOG_INF("%s gyro", ctx->name);
	LOG_INF("%s mean: %10.4f %10.4f %10.4f", ctx->name,
		gyro_mean[0], gyro_mean[1], gyro_mean[2]);
	LOG_INF("%s: std: %10.4f %10.4f %10.4f", ctx->name,
		gyro_std[0], gyro_std[1], gyro_std[2]);

	for (int i = 0; i < ARRAY_SIZE(gyro_mean); i++) {
		ctx->calibration.bias.gyro[i] = gyro_mean[i];
	}

	ctx->calibration.state = SENSE_IMU_STREAM_CALIBRATED;
	return;
}

static inline const struct sensor_decoder_api *get_imu_decoder(struct rtio_iodev *iodev)
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
	const uint32_t dec_buf_size = sizeof(struct sensor_three_axis_data) +
				      (10 - 1) * sizeof(struct sensor_three_axis_sample_data);
	uint8_t decoded_buffer[dec_buf_size];
	struct sensor_three_axis_data *dec_data = (struct sensor_three_axis_data *)decoded_buffer;
	uint32_t fit = 0;
	uint16_t frame_count;
	int ret;

	const struct sensor_decoder_api *decoder = get_imu_decoder(iodev);
	const struct sensor_three_axis_ref *axis_ref = get_axis_ref(iodev);

	if (!decoder) {
		return -EIO;
	}
	ret = decoder->get_frame_count(buf, (struct sensor_chan_spec) {ch, 0}, &frame_count);
	if (ret < 0) {
		return ret;
	}
	ret = decoder->decode(buf, (struct sensor_chan_spec) {ch, 0}, &fit, 10, decoded_buffer);
	if (ret < 0) {
		return ret;
	}
	avg_3_axis_q31(dec_data, ret, out);
	if (axis_ref) {
		sensor_three_axis_ref_align(axis_ref, out);
	}

	return 0;
}

static void imu_publish(struct context *ctx)
{
	stamp_msg(&ctx->imu.stamp, k_uptime_ticks());
	ctx->imu.linear_acceleration.x =
		(ctx->raw.accel[0] - ctx->calibration.bias.accel[0]) / ctx->calibration.accel_scale;
	ctx->imu.linear_acceleration.y =
		(ctx->raw.accel[1] - ctx->calibration.bias.accel[1]) / ctx->calibration.accel_scale;
	ctx->imu.linear_acceleration.z =
		(ctx->raw.accel[2] - ctx->calibration.bias.accel[2]) / ctx->calibration.accel_scale;
	ctx->imu.angular_velocity.x = (ctx->raw.gyro[0] - ctx->calibration.bias.gyro[0]);
	ctx->imu.angular_velocity.y = (ctx->raw.gyro[1] - ctx->calibration.bias.gyro[1]);
	ctx->imu.angular_velocity.z = (ctx->raw.gyro[2] - ctx->calibration.bias.gyro[2]);

	zros_pub_update(&ctx->pub_imu);
}

static inline bool calibration_requested(struct context *ctx)
{
	bool calibration_requested = ctx->status.mode == synapse_pb_Status_Mode_MODE_CALIBRATION &&
				     ctx->last_mode != synapse_pb_Status_Mode_MODE_CALIBRATION;

	ctx->last_mode = ctx->status.mode;

	return calibration_requested;
}

static void process_events(int result, uint8_t *buf, uint32_t len, void *userdata)
{
	struct rtio_iodev *iodev = (struct rtio_iodev *)userdata;
	struct context *ctx = get_context_from_iodev(iodev);
	struct sensor_three_axis_data accel_avg_data;
	struct sensor_three_axis_data gyro_avg_data;
	int ret;

	if (result < 0) {
		LOG_ERR("%s: Failed event: %d", ctx->name, result);
		ctx->running = false;
		return;
	}

	ret = decode_data(iodev, buf, SENSOR_CHAN_ACCEL_XYZ, &accel_avg_data);
	if (ret < 0) {
		LOG_ERR("%s: Failed to decode accel: %d", ctx->name, ret);
		ctx->running = false;
		return;
	}

	ret = decode_data(iodev, buf, SENSOR_CHAN_GYRO_XYZ, &gyro_avg_data);
	if (ret < 0) {
		LOG_ERR("%s: Failed to decode gyro: %d", ctx->name, ret);
		ctx->running = false;
		return;
	}

	if (zros_sub_update_available(&ctx->sub_status)) {
		zros_sub_update(&ctx->sub_status);
	}

	if (calibration_requested(ctx) || ctx->calibration.state != SENSE_IMU_STREAM_CALIBRATED) {
		feed_calibration(ctx, &accel_avg_data, &gyro_avg_data);
		return;
	}

	for (size_t i = 0 ; i < 3 ; i++) {
		ctx->raw.accel[i] = Z_SHIFT_Q31_TO_F32(accel_avg_data.readings[0].values[i],
						       accel_avg_data.shift);
	}
	for (size_t i = 0 ; i < 3 ; i++) {
		ctx->raw.gyro[i] = Z_SHIFT_Q31_TO_F32(gyro_avg_data.readings[0].values[i],
						      gyro_avg_data.shift);
	}
	imu_publish(ctx);
}

static int setup_stream(struct context *ctx)
{
	/* Configure IMUs to 800-Hz, if they haven't been configured in DTS */
	uint64_t period_ticks = (uint64_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC * 1250 / 1000 / 1000;
	struct sensor_value ticks_per_event = {
		.val1 = period_ticks,
	};
	struct rtio *rtio_ctx = ctx->stream.ctx;
	struct rtio_iodev *iodev = ctx->stream.iodev;
	struct sensor_read_config *read_config =
		(struct sensor_read_config *)iodev->data;
	int err;

	LOG_INF("setting up stream %p...", iodev);

	/* Don't check error as it may only be configurable through DTS */
	(void)sensor_attr_set(read_config->sensor, SENSOR_CHAN_ALL,
				SENSOR_ATTR_BATCH_DURATION, &ticks_per_event);

	err = sensor_stream(iodev, rtio_ctx, (void *)iodev, NULL);
	if (err != 0) {
		LOG_ERR("Failed to start sensor-stream: %d, %p...", err, iodev);
		rtio_sqe_reset_all(ctx->stream.ctx);
		return err;
	}
	ctx->running = true;

	return 0;
}

static void imu_stream_thread(void *arg0)
{
	struct context *ctx = (struct context *)arg0;
	int err;

	LOG_INF("init");
	zros_node_init(&ctx->node, ctx->name);
	zros_pub_init(&ctx->pub_imu, &ctx->node, ctx->pub_topic, &ctx->imu);
	zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 1);

	err = setup_stream(ctx);
	__ASSERT(!err, "Failed to start stream for %p", ctx->stream.iodev);

	while (true) {
		err = sensor_processing_cb_with_timeout(ctx->stream.ctx, process_events,
							K_MSEC(100));
		if (err != 0 || !ctx->running) {
			ctx->running = false;
			LOG_ERR("Error during stream. Attempting recovery...");
			do {
				/* TODO: Decide when we've tried too much. */
				k_sleep(K_MSEC(100));
				(void)setup_stream(ctx);
			} while (!ctx->running);
		}
	}
}

#if IS_ENABLED(CONFIG_ZROS_SENSE_STREAM_IMU_DRDY_MODE)
#define IMU_STREAM_TRIGGER SENSOR_TRIG_DATA_READY
#else
#define IMU_STREAM_TRIGGER SENSOR_TRIG_FIFO_WATERMARK
#endif

#define IMU_STREAM_DEFINE(i)									   \
												   \
RTIO_DEFINE_WITH_MEMPOOL(imu_stream_ctx_##i, 2, 4, 128, 4, sizeof(void *));			   \
SENSOR_DT_STREAM_IODEV(imu_stream_iodev_##i, IMU_ALIAS(i),					   \
		       {IMU_STREAM_TRIGGER, SENSOR_STREAM_DATA_INCLUDE});		   \
												   \
static struct context imu_stream_context_##i = {						   \
	.name = STRINGIFY(sense_imu_##i),							   \
	.node = {},										   \
	.pub_imu = {},										   \
	.pub_topic = &topic_imu##i,								   \
	.imu = {										   \
		.has_stamp = true,								   \
		.stamp = synapse_pb_Timestamp_init_default,					   \
		.has_angular_velocity = true,							   \
		.angular_velocity = synapse_pb_Vector3_init_default,				   \
		.has_linear_acceleration = true,						   \
		.linear_acceleration = synapse_pb_Vector3_init_default,				   \
		.has_orientation = false,							   \
	},											   \
	.sub_status = {},									   \
	.status = synapse_pb_Status_init_default,						   \
	.running = false,									   \
	.calibration = {									   \
		.state = SENSE_IMU_STREAM_UNCALIBRATED,						   \
	},											   \
	.stream = {										   \
		.ctx = &imu_stream_ctx_##i,							   \
		.iodev = &imu_stream_iodev_##i,							   \
	},											   \
};												   \
												   \
K_THREAD_DEFINE(imu_stream_thread_##i##_id, 1024, imu_stream_thread,				   \
		&imu_stream_context_##i, NULL, NULL, 2, 0, 0)

#define IMU_STREAM_DEFINE_IF_EXISTS(i)								   \
	IF_ENABLED(DT_NODE_EXISTS(IMU_ALIAS(i)), (IMU_STREAM_DEFINE(i)))

IMU_STREAM_DEFINE_IF_EXISTS(0);
IMU_STREAM_DEFINE_IF_EXISTS(1);
IMU_STREAM_DEFINE_IF_EXISTS(2);

#define IMU_STREAM_CTX_LIST									   \
		IF_ENABLED(DT_NODE_EXISTS(IMU_ALIAS(0)), (&imu_stream_context_0,))		   \
		IF_ENABLED(DT_NODE_EXISTS(IMU_ALIAS(1)), (&imu_stream_context_1,))		   \
		IF_ENABLED(DT_NODE_EXISTS(IMU_ALIAS(2)), (&imu_stream_context_2,))

struct context * const imu_list[] = {
		IMU_STREAM_CTX_LIST
};

static struct context *get_context_from_iodev(const struct rtio_iodev *iodev)
{
	for (size_t i = 0 ; i < ARRAY_SIZE(imu_list) ; i++) {
		if (imu_list[i] != NULL && imu_list[i]->stream.iodev == iodev) {
			return imu_list[i];
		}
	}
	CODE_UNREACHABLE;
}
