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

#include <../drivers/sensor/broadcom/afbr_s50/afbr_s50_decoder.h>

LOG_MODULE_REGISTER(sense_argus_stream, CONFIG_ZROS_SENSE_STREAM_ARGUS_LOG_LEVEL);

#define Z_SHIFT_Q31_TO_F32(src, m) ((float32_t)(((int64_t)src) << (m)) / (float32_t)(1U << 31))
#define Z_SHIFT_UQ32_TO_F32(src, m) ((float32_t)(((uint64_t)src) << (m)) / (float32_t)(1U << 31) / \
									   (float32_t)(1U << 1))
#define Z_SHIFT_Q15_TO_F32(src, m) ((float32_t)(((int64_t)src) << (m)) / (float32_t)(1U << 15))
#define Z_SHIFT_UQ16_TO_F32(src, m) ((float32_t)(((uint64_t)src) << (m)) / (float32_t)(1U << 16))

#define ARGUS_ALIAS(i) DT_ALIAS(_CONCAT(argus_stream_,i))

static struct context *get_context_from_iodev(const struct rtio_iodev *iodev);

struct context {
	const char *name;
	struct zros_node node;
	struct zros_pub pub;
	struct zros_topic *pub_topic;
	bool running;
	synapse_pb_ArgusResults data;
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

static inline void fill_bin_data(struct context *ctx, argus_results_t *payload)
{
	ctx->data.has_bin = true;
	ctx->data.bin.range = Z_SHIFT_Q31_TO_F32(payload->Bin.Range, 9);
	ctx->data.bin.amplitude = Z_SHIFT_UQ16_TO_F32(payload->Bin.Amplitude, 12);
	ctx->data.bin.signal_quality = payload->Bin.SignalQuality;
}

static inline void fill_aux_data(struct context *ctx, argus_results_t *payload)
{
	ctx->data.has_auxiliary = true;
	ctx->data.auxiliary.vdd = Z_SHIFT_UQ16_TO_F32(payload->Auxiliary.VDD, 12);
	ctx->data.auxiliary.temp = Z_SHIFT_Q15_TO_F32(payload->Auxiliary.TEMP, 11);
	ctx->data.auxiliary.vsub = Z_SHIFT_UQ16_TO_F32(payload->Auxiliary.VSUB, 12);
	ctx->data.auxiliary.vddl = Z_SHIFT_UQ16_TO_F32(payload->Auxiliary.VDDL, 12);
	ctx->data.auxiliary.iapd = Z_SHIFT_UQ16_TO_F32(payload->Auxiliary.IAPD, 12);
	ctx->data.auxiliary.bgl = Z_SHIFT_UQ16_TO_F32(payload->Auxiliary.BGL, 12);
	ctx->data.auxiliary.sna = Z_SHIFT_UQ16_TO_F32(payload->Auxiliary.SNA, 12);
}

static inline void fill_frame_data(struct context *ctx, argus_results_t *payload)
{
	ctx->data.has_frame = true;
	ctx->data.frame.integration_time = payload->Frame.IntegrationTime;
	ctx->data.frame.px_en_mask = payload->Frame.PxEnMask;
	ctx->data.frame.ch_en_mask = payload->Frame.ChEnMask;
	ctx->data.frame.state = payload->Frame.State;
	ctx->data.frame.analog_integration_depth =
		Z_SHIFT_UQ16_TO_F32(payload->Frame.AnalogIntegrationDepth, 10);
	ctx->data.frame.digital_integration_depth = payload->Frame.DigitalIntegrationDepth;
	ctx->data.frame.output_power = Z_SHIFT_UQ16_TO_F32(payload->Frame.OutputPower, 12);
	ctx->data.frame.bias_current = payload->Frame.BiasCurrent;
	ctx->data.frame.pixel_gain = payload->Frame.PixelGain;
	ctx->data.frame.pll_offset = payload->Frame.PllOffset;
	ctx->data.frame.pll_ctrl_cur = payload->Frame.PllCtrlCur;
}

static inline void fill_pixel_data(struct context *ctx, argus_results_t *payload)
{
	ctx->data.pixel_count = 32;
	for (size_t i = 0; i < 32; i++) {
		ctx->data.pixel[i].range = Z_SHIFT_Q31_TO_F32(payload->Pixels[i].Range, 9);
		ctx->data.pixel[i].phase = Z_SHIFT_UQ16_TO_F32(payload->Pixels[i].Phase, 1);
		ctx->data.pixel[i].amplitude =
			Z_SHIFT_UQ16_TO_F32(payload->Pixels[i].Amplitude, 12);
		ctx->data.pixel[i].status = payload->Pixels[i].Status;
		ctx->data.pixel[i].range_window = payload->Pixels[i].RangeWindow;
		ctx->data.pixel[i].amplitude_raw =
			Z_SHIFT_UQ16_TO_F32(payload->Pixels[i].AmplitudeRaw, 12);
		ctx->data.pixel[i].uncorrelated_noise = 
			Z_SHIFT_UQ16_TO_F32(payload->Pixels[i].UncorrelatedNoise, 12);
		ctx->data.pixel[i].snr = Z_SHIFT_UQ16_TO_F32(payload->Pixels[i].SNR, 12);
	}

	ctx->data.has_pixel_ref = true;
	ctx->data.pixel_ref.range = Z_SHIFT_Q31_TO_F32(payload->Pixels[32].Range, 9);
	ctx->data.pixel_ref.phase = Z_SHIFT_UQ16_TO_F32(payload->Pixels[32].Phase, 1);
	ctx->data.pixel_ref.amplitude =
		Z_SHIFT_UQ16_TO_F32(payload->Pixels[32].Amplitude, 12);
	ctx->data.pixel_ref.status = payload->Pixels[32].Status;
	ctx->data.pixel_ref.range_window = payload->Pixels[32].RangeWindow;
	ctx->data.pixel_ref.amplitude_raw =
		Z_SHIFT_UQ16_TO_F32(payload->Pixels[32].AmplitudeRaw, 12);
	ctx->data.pixel_ref.uncorrelated_noise = 
		Z_SHIFT_UQ16_TO_F32(payload->Pixels[32].UncorrelatedNoise, 12);
	ctx->data.pixel_ref.snr = Z_SHIFT_UQ16_TO_F32(payload->Pixels[32].SNR, 12);
}

static void argus_publish(struct context *ctx, argus_results_t *payload)
{
	stamp_msg(&ctx->data.stamp, k_uptime_ticks());

	ctx->data.status = payload->Status;
	fill_bin_data(ctx, payload);

	if (IS_ENABLED(CONFIG_ZROS_SENSE_STREAM_ARGUS_POINTCLOUD)) {
		fill_pixel_data(ctx, payload);
		ctx->data.has_auxiliary = false;
		ctx->data.has_frame = false;
	} else if (IS_ENABLED(CONFIG_ZROS_SENSE_STREAM_ARGUS_VERBOSE)) {
		fill_pixel_data(ctx, payload);
		fill_aux_data(ctx, payload);
		fill_frame_data(ctx, payload);
	} else {
		ctx->data.pixel_count = 0;
		ctx->data.has_pixel_ref = false;
		ctx->data.has_auxiliary = false;
		ctx->data.has_frame = false;
	}
	zros_pub_update(&ctx->pub);
}

static void process_events(int result, uint8_t *buf, uint32_t len, void *userdata)
{
	struct rtio_iodev *iodev = (struct rtio_iodev *)userdata;
	struct context *ctx = get_context_from_iodev(iodev);
	struct afbr_s50_edata *edata = (struct afbr_s50_edata *)buf;

	if (result < 0) {
		LOG_ERR("%s: Failed event: %d", ctx->name, result);
		ctx->running = false;
		return;
	}
	argus_publish(ctx, &edata->payload);
}

static int setup_stream(struct context *ctx)
{
	struct rtio *rtio_ctx = ctx->stream.ctx;
	struct rtio_iodev *iodev = ctx->stream.iodev;
	int err;

	LOG_INF("setting up stream %p...", iodev);

	err = sensor_stream(iodev, rtio_ctx, (void *)iodev, NULL);
	if (err != 0) {
		rtio_sqe_reset_all(ctx->stream.ctx);
		LOG_ERR("Failed to start sensor-stream: %d, %p...", err, iodev);
		return err;
	}
	LOG_INF("Stream reestablished");
	ctx->running = true;

	return 0;
}

static void argus_stream_thread(void *arg0)
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
			do {
				/* TODO: Decide when we've tried too much. */
				k_sleep(K_MSEC(1000));
				(void)setup_stream(ctx);
			} while (!ctx->running);
		}
	}
}

#define DIST_STREAM_DEFINE(i)									   \
												   \
RTIO_DEFINE_WITH_MEMPOOL(dist_stream_ctx_##i, 2, 4, 256, 8, sizeof(void *));			   \
SENSOR_DT_STREAM_IODEV(dist_stream_iodev_##i, ARGUS_ALIAS(i),					   \
		       {SENSOR_TRIG_DATA_READY, SENSOR_STREAM_DATA_INCLUDE});			   \
												   \
static struct context dist_stream_context_##i = {						   \
	.name = STRINGIFY(sense_dist_##i),							   \
	.node = {},										   \
	.pub = {},										   \
	.pub_topic = &topic_argus,								   \
	.running = false,									   \
	.data = {										   \
		.has_stamp = true,								   \
		.stamp = synapse_pb_Timestamp_init_default,					   \
		.has_bin = true,								   \
	},											   \
	.stream = {										   \
		.ctx = &dist_stream_ctx_##i,							   \
		.iodev = &dist_stream_iodev_##i,						   \
	},											   \
};												   \
												   \
K_THREAD_DEFINE(dist_stream_thread_##i##_id, 1024, argus_stream_thread,				   \
		&dist_stream_context_##i, NULL, NULL, 2, 0, 0)

#define DIST_STREAM_DEFINE_IF_EXISTS(i)								   \
	IF_ENABLED(DT_NODE_EXISTS(ARGUS_ALIAS(i)), (DIST_STREAM_DEFINE(i)))

DIST_STREAM_DEFINE_IF_EXISTS(0);

#define DIST_STREAM_CTX_LIST									   \
		IF_ENABLED(DT_NODE_EXISTS(ARGUS_ALIAS(0)), (&dist_stream_context_0,))

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
