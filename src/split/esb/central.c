/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/types.h>
#include <zephyr/init.h>

#include <zephyr/settings/settings.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);

#include <zmk/stdlib.h>
#include <zmk/behavior.h>
#include <zmk/sensors.h>
#include <zmk/split/transport/central.h>
#include <zmk/split/transport/types.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/pointing/input_split.h>
#include <zmk/hid_indicators_types.h>
#include <zmk/physical_layouts.h>

#include "app_esb.h"
#include "common.h"

#define RX_BUFFER_SIZE                                                                             \
    ((sizeof(struct esb_event_envelope) + sizeof(struct esb_msg_postfix)) *                        \
     CONFIG_ZMK_SPLIT_ESB_EVENT_BUFFER_ITEMS)
#define TX_BUFFER_SIZE                                                                             \
    ((sizeof(struct esb_command_envelope) + sizeof(struct esb_msg_postfix)) *                      \
     CONFIG_ZMK_SPLIT_ESB_CMD_BUFFER_ITEMS)

RING_BUF_DECLARE(rx_buf, RX_BUFFER_SIZE);
RING_BUF_DECLARE(tx_buf, TX_BUFFER_SIZE);

static K_SEM_DEFINE(esb_send_cmd_sem, 1, 1);

static void publish_events_work(struct k_work *work);

K_WORK_DEFINE(publish_events, publish_events_work);

uint8_t async_rx_buf[RX_BUFFER_SIZE / 2][2];

static struct zmk_split_esb_async_state async_state = {
    .process_tx_work = &publish_events,
    .rx_bufs = {async_rx_buf[0], async_rx_buf[1]},
    .rx_bufs_len = RX_BUFFER_SIZE / 2,
    .rx_size_process_trigger = ESB_MSG_EXTRA_SIZE + 1,
    .rx_buf = &rx_buf,
    .tx_buf = &tx_buf,
};

static void begin_tx(void) {
    zmk_split_esb_async_tx(&async_state);
}

static ssize_t get_payload_data_size(const struct zmk_split_transport_central_command *cmd) {
    switch (cmd->type) {
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS:
        return 0;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR:
        return sizeof(cmd->data.invoke_behavior);
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
        return sizeof(cmd->data.set_physical_layout);
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
        return sizeof(cmd->data.set_hid_indicators);
    default:
        return -ENOTSUP;
    }
}

static int split_central_esb_send_command(uint8_t source,
                                          struct zmk_split_transport_central_command cmd) {

    ssize_t data_size = get_payload_data_size(&cmd);
    if (data_size < 0) {
        LOG_WRN("Failed to determine payload data size %d", data_size);
        return data_size;
    }

    // lock it for a safe result from ring_buf_space_get()
    int ret = k_sem_take(&esb_send_cmd_sem, K_FOREVER);
    if (ret) {
        LOG_WRN("Shouldn't be called FOREVER");
        return 0;
    }

    // Data + type + source
    size_t payload_size =
        data_size + sizeof(source) + sizeof(enum zmk_split_transport_central_command_type);

    if (ring_buf_space_get(&tx_buf) < ESB_MSG_EXTRA_SIZE + payload_size) {
        LOG_WRN("No room to send command to the peripheral %d (have %d but only space for %d/%d)", 
                source, ESB_MSG_EXTRA_SIZE + payload_size, ring_buf_space_get(&tx_buf),
                ring_buf_capacity_get(&tx_buf));
        k_sem_give(&esb_send_cmd_sem);
        return -ENOSPC;
    }

    struct esb_command_envelope env = {.prefix = {
                                            .magic_prefix = ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX,
                                            .payload_size = payload_size,
                                        },
                                        .payload = {
                                            .source = source,
                                            .cmd = cmd,
                                        }};

    size_t pfx_len = sizeof(env.prefix) + payload_size;
    // LOG_HEXDUMP_DBG(&env, pfx_len, "Payload");

    size_t put = ring_buf_put(&tx_buf, (uint8_t *)&env, pfx_len);
    if (put != pfx_len) {
        LOG_WRN("Failed to put the whole message (%d vs %d)", put, pfx_len);
    }

    struct esb_msg_postfix postfix = {.crc = crc32_ieee((void *)&env, pfx_len)};

    put = ring_buf_put(&tx_buf, (uint8_t *)&postfix, sizeof(postfix));
    if (put != sizeof(postfix)) {
        LOG_WRN("Failed to put the postfix (%d vs %d)", put, sizeof(postfix));
    }

    begin_tx();

    k_sem_give(&esb_send_cmd_sem);
    return 0;
}

void zmk_split_esb_on_prx_esb_callback(app_esb_event_t *event) {
    zmk_split_esb_cb(event, &async_state);
}

static int split_central_esb_get_available_source_ids(uint8_t *sources) {
    sources[0] = 0;

    return 1;
}

static zmk_split_transport_central_status_changed_cb_t transport_status_cb;
static bool is_enabled;

static int split_central_esb_set_enabled(bool enabled) {
    is_enabled = enabled;
    return zmk_split_esb_set_enable(enabled);
}

static int
split_central_esb_set_status_callback(zmk_split_transport_central_status_changed_cb_t cb) {
    transport_status_cb = cb;
    return 0;
}

static struct zmk_split_transport_status split_central_esb_get_status() {
    return (struct zmk_split_transport_status){
        .available = true,
        .enabled = is_enabled,
        .connections = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED,
    };
}

static const struct zmk_split_transport_central_api central_api = {
    .send_command = split_central_esb_send_command,
    .get_available_source_ids = split_central_esb_get_available_source_ids,
    .set_enabled = split_central_esb_set_enabled,
    .set_status_callback = split_central_esb_set_status_callback,
    .get_status = split_central_esb_get_status,
};

ZMK_SPLIT_TRANSPORT_CENTRAL_REGISTER(esb_central, &central_api, CONFIG_ZMK_SPLIT_ESB_PRIORITY);

static void notify_transport_status(void) {
    if (transport_status_cb) {
        transport_status_cb(&esb_central, split_central_esb_get_status());
    }
}

static void notify_status_work_cb(struct k_work *_work) { notify_transport_status(); }

static K_WORK_DEFINE(notify_status_work, notify_status_work_cb);

static int zmk_split_esb_central_init(void) {
    int ret = zmk_split_esb_init(APP_ESB_MODE_PRX, zmk_split_esb_on_prx_esb_callback);
    if (ret) {
        LOG_ERR("zmk_split_esb_init failed (err %d)", ret);
        return ret;
    }
    k_work_submit(&notify_status_work);
    return 0;
}

SYS_INIT(zmk_split_esb_central_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static void publish_events_work(struct k_work *work) {
    while (ring_buf_size_get(&rx_buf) > ESB_MSG_EXTRA_SIZE) {
        struct esb_event_envelope env;
        int item_err =
            zmk_split_esb_get_item(&rx_buf, (uint8_t *)&env, sizeof(struct esb_event_envelope));
        switch (item_err) {
        case 0:
            zmk_split_transport_central_peripheral_event_handler(&esb_central, env.payload.source,
                                                                 env.payload.event);
            break;
        case -EAGAIN:
            return;
        default:
            LOG_WRN("Issue fetching an item from the RX buffer: %d", item_err);
            return;
        }
    }
}
