/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ZROS UWB ranging producer.
 *
 * Owns one FiRa DS-TWR unicast ranging session on the Zephyr UWB subsystem
 * (zephyr,uwb-uci, e.g. NXP SR250 on the MR-MCXN-T1-UWB shield) and publishes
 * every ranging measurement carried by SESSION_INFO_NTF on the ZROS topic
 * "uwb" (topic_uwb, synapse_topic_UwbRanging_t). The topic and its
 * fixed-layout payload struct come from the application's
 * synapse topic list (<synapse_topic_list.h>, synapse_uwb_topic.h).
 *
 * Session bring-up follows samples/uwb/ranging_controller|controlee. The
 * session is re-created automatically when the UWBS reports it IDLE/DEINIT
 * (e.g. max ranging-round retries, in-band termination) or when a step of the
 * bring-up fails.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/uwb/uwb.h>
#include <zephyr/uwb/uwb_core.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>

#include <synapse_time_status.h>
#include <synapse_topic_list.h>

LOG_MODULE_REGISTER(sense_uwb, CONFIG_ZROS_SENSE_UWB_LOG_LEVEL);

/* ------------------------------------------------------------------ */
/* FiRa UCI SESSION_INFO_NTF layout (GID 0x02, OID 0x00), TWR results  */
/* ------------------------------------------------------------------ */

#define UWB_MT_NTF               0x03
#define UWB_GID_SESSION_CONFIG   0x01
#define UWB_GID_SESSION_CONTROL  0x02
#define UWB_OID_SESSION_STATUS   0x02
#define UWB_OID_SESSION_INFO     0x00

/* Fixed part: seq(4) handle(4) rcr(1) interval(4) type(1) rfu(1)
 * mac_mode(1) primary_handle(4) rfu(4) n_meas(1)
 */
#define INFO_OFF_SEQ             0
#define INFO_OFF_HANDLE          4
#define INFO_OFF_INTERVAL        9
#define INFO_OFF_MEAS_TYPE       13
#define INFO_OFF_MAC_MODE        15
#define INFO_OFF_N_MEAS          24
#define INFO_HDR_LEN             25

#define MEAS_TYPE_TWR            0x01
#define MAC_MODE_SHORT           0x00

/* One TWR measurement with a 2-byte MAC address. */
#define TWR_MEAS_LEN_SHORT       31

/* SESSION_STATUS_NTF: handle(4) state(1) reason(1) */
#define STATUS_NTF_LEN           6

/* FiRa AoA values are signed Q9.7 degrees. */
#define Q9_7_TO_DEG(v)           ((float)(int16_t)(v) / 128.0f)

/* FiRa ranging status values treated as a valid measurement. */
#define UWB_RANGING_STATUS_OK               0x00
#define UWB_RANGING_STATUS_OK_NEGATIVE_DIST 0x1B

#define UWB_SESSION_STATE_DEINIT 0x01
#define UWB_SESSION_STATE_IDLE   0x03

#if defined(CONFIG_ZROS_SENSE_UWB_ROLE_CONTROLLER)
#define UWB_ROLE_IS_CONTROLLER 1
#else
#define UWB_ROLE_IS_CONTROLLER 0
#endif

struct context {
	struct zros_node node;
	struct zros_pub pub;
	synapse_topic_UwbRanging_t data;
	struct k_sem session_lost;
	bool time_ever_synced;
	uint32_t session_handle;
	bool session_active;
	uint32_t n_published;
	uint32_t n_failed;
	uint32_t n_restarts;
};

static struct context g_ctx;

static uint8_t g_device_mac[2];
static uint8_t g_dest_mac[2];

static void publish_measurement(const uint8_t *hdr, const uint8_t *m, int64_t now_ticks)
{
	synapse_topic_UwbRanging_t *d = &g_ctx.data;
	const uint16_t peer_mac = sys_get_le16(&m[0]);
	const uint16_t own_mac = sys_get_le16(g_device_mac);
	int64_t time_offset_ns = 0;

	memset(d, 0, sizeof(*d));

	/* Boot-clock stamp of the notification, carried onto the gPTP timescale
	 * when this node is disciplined.
	 */
	d->time_status = synapse_time_status_resolve(&g_ctx.time_ever_synced, &time_offset_ns);
	d->timestamp_ns = (uint64_t)((int64_t)k_ticks_to_ns_floor64(now_ticks) + time_offset_ns);
	d->session_id = CONFIG_ZROS_SENSE_UWB_SESSION_ID;
	d->sequence = sys_get_le32(&hdr[INFO_OFF_SEQ]);
	d->ranging_interval_ms = sys_get_le32(&hdr[INFO_OFF_INTERVAL]);
	d->role = UWB_ROLE_IS_CONTROLLER ? SYNAPSE_TOPIC_UWB_ROLE_CONTROLLER
					 : SYNAPSE_TOPIC_UWB_ROLE_CONTROLEE;
	d->initiator_mac = UWB_ROLE_IS_CONTROLLER ? own_mac : peer_mac;
	d->responder_mac = UWB_ROLE_IS_CONTROLLER ? peer_mac : own_mac;

	d->status = m[2];
	d->nlos = m[3];
	d->distance_m = (float)sys_get_le16(&m[4]) / 100.0f; /* cm -> m */
	d->aoa_azimuth_deg = Q9_7_TO_DEG(sys_get_le16(&m[6]));
	d->aoa_azimuth_fom = m[8];
	d->aoa_elevation_deg = Q9_7_TO_DEG(sys_get_le16(&m[9]));
	d->aoa_elevation_fom = m[11];
	d->aoa_dest_azimuth_deg = Q9_7_TO_DEG(sys_get_le16(&m[12]));
	d->aoa_dest_azimuth_fom = m[14];
	d->aoa_dest_elevation_deg = Q9_7_TO_DEG(sys_get_le16(&m[15]));
	d->aoa_dest_elevation_fom = m[17];
	d->slot_index = m[18];
	/* RSSI: unsigned Q7.1, value is -dBm; 0 means not reported. */
	if (m[19] != 0U) {
		d->rssi_dbm = -((float)m[19] / 2.0f);
		d->flags |= SYNAPSE_TOPIC_UWB_FLAG_RSSI_VALID;
	}
	if (IS_ENABLED(CONFIG_ZROS_SENSE_UWB_AOA)) {
		d->flags |= SYNAPSE_TOPIC_UWB_FLAG_AOA_VALID;
	}

	const bool valid = (d->status == UWB_RANGING_STATUS_OK ||
			    d->status == UWB_RANGING_STATUS_OK_NEGATIVE_DIST);
	if (valid) {
		d->flags |= SYNAPSE_TOPIC_UWB_FLAG_DISTANCE_VALID;
		g_ctx.n_published++;
	} else {
		g_ctx.n_failed++;
	}

	zros_pub_update(&g_ctx.pub);
	LOG_DBG("seq %u peer 0x%04x status 0x%02x dist %d cm", d->sequence, peer_mac,
		d->status, (int)(d->distance_m * 100.0f));
}

static void handle_session_info(const uint8_t *p, size_t len)
{
	const int64_t now_ticks = k_uptime_ticks();

	if (len < INFO_HDR_LEN) {
		LOG_WRN("short SESSION_INFO_NTF (%u)", (unsigned int)len);
		return;
	}
	if (sys_get_le32(&p[INFO_OFF_HANDLE]) != g_ctx.session_handle) {
		return;
	}
	if (p[INFO_OFF_MEAS_TYPE] != MEAS_TYPE_TWR || p[INFO_OFF_MAC_MODE] != MAC_MODE_SHORT) {
		LOG_WRN("unsupported measurement type %u / mac mode %u", p[INFO_OFF_MEAS_TYPE],
			p[INFO_OFF_MAC_MODE]);
		return;
	}

	const uint8_t n = p[INFO_OFF_N_MEAS];

	for (uint8_t i = 0; i < n; i++) {
		size_t off = INFO_HDR_LEN + (size_t)i * TWR_MEAS_LEN_SHORT;

		if (off + TWR_MEAS_LEN_SHORT > len) {
			LOG_WRN("truncated measurement %u/%u", i, n);
			break;
		}
		publish_measurement(p, &p[off], now_ticks);
	}
}

static void handle_session_status(const uint8_t *p, size_t len)
{
	if (len < STATUS_NTF_LEN || sys_get_le32(&p[0]) != g_ctx.session_handle) {
		return;
	}
	const uint8_t state = p[4];
	const uint8_t reason = p[5];

	LOG_DBG("session state %u reason 0x%02x", state, reason);
	if (g_ctx.session_active &&
	    (state == UWB_SESSION_STATE_IDLE || state == UWB_SESSION_STATE_DEINIT)) {
		LOG_WRN("session left ACTIVE (state %u, reason 0x%02x)", state, reason);
		g_ctx.session_active = false;
		k_sem_give(&g_ctx.session_lost);
	}
}

/*
 * Runs in the UCI reader thread for every notification / data packet. Only
 * ranging results and session status are consumed here; everything else goes
 * to the subsystem's default notification handler (which copies the packet).
 */
static void uwb_ntf_cb(const uint8_t *const packet, const uint32_t len)
{
	if (packet == NULL || len <= UCI_HEADER_SIZE) {
		return;
	}

	const uint8_t mt = (packet[0] & UCI_MT_MASK) >> UCI_MT_SHIFT;
	const uint8_t gid = packet[0] & UCI_GID_MASK;
	const uint8_t oid = packet[1] & UCI_OID_MASK;
	const uint8_t *payload = &packet[UCI_HEADER_SIZE];
	const size_t plen = len - UCI_HEADER_SIZE;

	if (mt == UWB_MT_NTF && gid == UWB_GID_SESSION_CONTROL && oid == UWB_OID_SESSION_INFO) {
		handle_session_info(payload, plen);
		return;
	}
	if (mt == UWB_MT_NTF && gid == UWB_GID_SESSION_CONFIG && oid == UWB_OID_SESSION_STATUS) {
		handle_session_status(payload, plen);
		return;
	}
	uwb_ntf_callback_handler(packet, len);
}

#if defined(CONFIG_ZROS_SENSE_UWB_AOA)
#define NXP_UCI_GID_VENDOR             0x0F
#define NXP_UCI_OID_SET_VENDOR_APP_CFG 0x00
#define NXP_VAPP_ANTENNAE_CONFIG_RX    0x03
#define NXP_VAPP_ENABLE_FOM            0x91
#define NXP_ANT_RX_MODE_AOA            0x01
static int session_configure_vendor_aoa(uint32_t handle)
{
	uint8_t p[16];
	uint8_t rsp[UCI_HEADER_SIZE + 8];
	uint32_t rsp_len = sizeof(rsp);
	size_t i = 0;
	sys_put_le32(handle, &p[i]);
	i += 4;
	p[i++] = 2; /* number of vendor params */
	p[i++] = NXP_VAPP_ANTENNAE_CONFIG_RX;
#if defined(CONFIG_ZROS_SENSE_UWB_AOA_3D)
	p[i++] = 4;
	p[i++] = NXP_ANT_RX_MODE_AOA;
	p[i++] = 2;
	p[i++] = CONFIG_ZROS_SENSE_UWB_AOA_RX_PAIR_H;
	p[i++] = CONFIG_ZROS_SENSE_UWB_AOA_RX_PAIR_V;
#else
	p[i++] = 3;
	p[i++] = NXP_ANT_RX_MODE_AOA;
	p[i++] = 1;
	p[i++] = CONFIG_ZROS_SENSE_UWB_AOA_RX_PAIR_H;
#endif
	p[i++] = NXP_VAPP_ENABLE_FOM;
	p[i++] = 1;
	p[i++] = 1;
	if (uwb_uci_transceive_control_packet(NXP_UCI_GID_VENDOR, NXP_UCI_OID_SET_VENDOR_APP_CFG,
					      p, i, rsp, &rsp_len) != 0) {
		LOG_ERR("SET_VENDOR_APP_CONFIG: no response");
		return -EIO;
	}
	if (rsp_len > UCI_HEADER_SIZE && rsp[UCI_HEADER_SIZE] != 0U) {
		LOG_ERR("SET_VENDOR_APP_CONFIG: status 0x%02x", rsp[UCI_HEADER_SIZE]);
		return -EIO;
	}
	return 0;
}
#endif /* CONFIG_ZROS_SENSE_UWB_AOA */
static uwb_status_code_t session_configure(uint32_t handle)
{
	uwb_status_code_t st;
	const uint32_t interval = CONFIG_ZROS_SENSE_UWB_RANGING_INTERVAL_MS;
	uint8_t interval_le[4];

	sys_put_le32(interval, interval_le);

	uwb_config_t role_cfg[] = {
		{UWB_APP_CONFIG_NUMBER_OF_CONTROLEES, 1, ARR(1), 0},
		{UWB_APP_CONFIG_DST_MAC_ADDRESS, sizeof(g_dest_mac), g_dest_mac, 0},
		{UWB_APP_CONFIG_DEVICE_ROLE, 1,
		 ARR(UWB_ROLE_IS_CONTROLLER ? UWB_DEVICE_ROLE_INITIATOR
					    : UWB_DEVICE_ROLE_RESPONDER), 0},
		{UWB_APP_CONFIG_MULTI_NODE_MODE, 1, ARR(UWB_MULTI_NODE_MODE_UNICAST), 0},
		{UWB_APP_CONFIG_MAC_ADDRESS_MODE, 1, ARR(UWB_MAC_ADDR_MODE_2BYTES), 0},
		{UWB_APP_CONFIG_SCHEDULE_MODE, 1, ARR(UWB_SCHEDULE_MODE_TIME_SCHEDULED), 0},
		{UWB_APP_CONFIG_DEVICE_MAC_ADDRESS, sizeof(g_device_mac), g_device_mac, 0},
		{UWB_APP_CONFIG_RANGING_ROUND_USAGE, 1, ARR(UWB_RANGING_ROUND_USAGE_DS_TWR), 0},
		{UWB_APP_CONFIG_DEVICE_TYPE, 1,
		 ARR(UWB_ROLE_IS_CONTROLLER ? UWB_DEVICE_TYPE_CONTROLLER
					    : UWB_DEVICE_TYPE_CONTROLEE), 0},
	};

	st = uwb_api_set_app_configs(handle, role_cfg, ARRAY_SIZE(role_cfg));
	if (st != UWB_STATUS_CODE_SUCCESS) {
		return st;
	}

	/* PHY settings, identical to samples/uwb/ranging_controller demo_ranging_config.h */
	uwb_config_t phy_cfg[] = {
		{UWB_APP_CONFIG_AOA_RESULT_REQ, 1,
		 ARR(IS_ENABLED(CONFIG_ZROS_SENSE_UWB_AOA) ? 1 : 0), 0},
		{UWB_APP_CONFIG_PREAMBLE_CODE_INDEX, 1,
		 ARR(CONFIG_ZROS_SENSE_UWB_PREAMBLE_CODE_INDEX), 0},
		{UWB_APP_CONFIG_PREAMBLE_DURATION, 1, ARR(1), 0},
		{UWB_APP_CONFIG_PSDU_DATA_RATE, 1, ARR(0), 0},
		{UWB_APP_CONFIG_PRF_MODE, 1, ARR(0), 0},
		{UWB_APP_CONFIG_BPRF_PHR_DATA_RATE, 1, ARR(0), 0},
		{UWB_APP_CONFIG_STS_LENGTH, 1, ARR(1), 0},
		{UWB_APP_CONFIG_RESULT_REPORT_CONFIG, 1,
		 ARR(IS_ENABLED(CONFIG_ZROS_SENSE_UWB_AOA) ? 0x0F : 0x01), 0},
		{UWB_APP_CONFIG_RANGING_DURATION, sizeof(interval_le), interval_le, 0},
	};
	size_t n_phy = ARRAY_SIZE(phy_cfg);

	if (interval == 0U) {
		n_phy--; /* RANGING_DURATION is last: keep the UWBS default */
	}

	st = uwb_api_set_app_configs(handle, phy_cfg, n_phy);
	if (st != UWB_STATUS_CODE_SUCCESS) {
		return st;
	}
#if defined(CONFIG_ZROS_SENSE_UWB_AOA)
	if (session_configure_vendor_aoa(handle) != 0) {
		return UWB_STATUS_CODE_FAILED;
	}
#endif
	return UWB_STATUS_CODE_SUCCESS;
}

static void session_teardown(void)
{
	if (g_ctx.session_handle != 0U) {
		(void)uwb_api_session_stop(g_ctx.session_handle);
		(void)uwb_api_session_deinit(g_ctx.session_handle);
		g_ctx.session_handle = 0U;
	}
}

static int session_bringup(void)
{
	uwb_status_code_t st;
	uint32_t handle = 0U;

	st = uwb_api_session_init(CONFIG_ZROS_SENSE_UWB_SESSION_ID,
				  UWB_SESSION_TYPE_RANGING, &handle);
	if (st != UWB_STATUS_CODE_SUCCESS) {
		LOG_ERR("session_init failed: %d", st);
		return -EIO;
	}
	g_ctx.session_handle = handle;

	st = session_configure(handle);
	if (st != UWB_STATUS_CODE_SUCCESS) {
		LOG_ERR("set_app_configs failed: %d", st);
		return -EIO;
	}

	k_sem_reset(&g_ctx.session_lost);
	g_ctx.session_active = true;
	st = uwb_api_session_start(handle);
	if (st != UWB_STATUS_CODE_SUCCESS) {
		g_ctx.session_active = false;
		LOG_ERR("session_start failed: %d", st);
		return -EIO;
	}

	LOG_INF("%s ranging: session 0x%08x, own 0x%04x, peer 0x%04x",
		UWB_ROLE_IS_CONTROLLER ? "controller" : "controlee",
		CONFIG_ZROS_SENSE_UWB_SESSION_ID, sys_get_le16(g_device_mac),
		sys_get_le16(g_dest_mac));
	return 0;
}

static void uwb_session_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	sys_put_le16(CONFIG_ZROS_SENSE_UWB_DEVICE_MAC, g_device_mac);
	sys_put_le16(CONFIG_ZROS_SENSE_UWB_DEST_MAC, g_dest_mac);
	k_sem_init(&g_ctx.session_lost, 0, 1);

	zros_node_init(&g_ctx.node, "sense_uwb");
	zros_pub_init(&g_ctx.pub, &g_ctx.node, &topic_uwb, &g_ctx.data);

	if (uwb_uci_register_callback(uwb_ntf_cb) != 0) {
		LOG_ERR("could not register UCI callback");
		return;
	}

	while (true) {
		if (session_bringup() == 0) {
			(void)k_sem_take(&g_ctx.session_lost, K_FOREVER);
		}
		g_ctx.session_active = false;
		session_teardown();
		g_ctx.n_restarts++;
		k_sleep(K_MSEC(CONFIG_ZROS_SENSE_UWB_RESTART_DELAY_MS));
	}
}

K_THREAD_DEFINE(sense_uwb_thread, CONFIG_ZROS_SENSE_UWB_THREAD_STACK_SIZE,
		uwb_session_thread, NULL, NULL, NULL, CONFIG_ZROS_SENSE_UWB_THREAD_PRIORITY,
		0, 0);

#if defined(CONFIG_SHELL)
static int cmd_uwb_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	const synapse_topic_UwbRanging_t *d = &g_ctx.data;

	shell_print(sh, "role %s, session 0x%08x (%s), handle 0x%08x",
		    UWB_ROLE_IS_CONTROLLER ? "controller" : "controlee",
		    CONFIG_ZROS_SENSE_UWB_SESSION_ID,
		    g_ctx.session_active ? "active" : "inactive", g_ctx.session_handle);
	shell_print(sh, "ok %u, failed %u, restarts %u", g_ctx.n_published, g_ctx.n_failed,
		    g_ctx.n_restarts);
	shell_print(sh, "last: seq %u init 0x%04x resp 0x%04x status 0x%02x nlos %u "
		    "dist %.3f m az %.2f el %.2f deg",
		    d->sequence, d->initiator_mac, d->responder_mac, d->status, d->nlos,
		    (double)d->distance_m, (double)d->aoa_azimuth_deg,
		    (double)d->aoa_elevation_deg);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_uwb, SHELL_CMD(status, NULL, "UWB ranging status",
						  cmd_uwb_status),
			       SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(uwb, &sub_uwb, "UWB ranging commands", NULL);
#endif
