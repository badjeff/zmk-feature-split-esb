/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/sys/ring_buffer.h>
#include <zephyr/device.h>

#include <zmk/split/transport/types.h>
#include "app_esb.h"

#define ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX "ZmKe"

/* Message type discriminant carried in esb_msg_prefix.msg_type.
 *
 * KEY_STATE replaces per-event KEY_POSITION_EVENT packets with a 16-byte
 * bitmap of all currently pressed positions (position N → byte N/8 bit N%8).
 * Sending the full state on every change makes key data idempotent over ESB:
 * a duplicate or late packet XORs to zero diff, emitting no phantom events.
 * No sequence numbers or deduplication are needed for key position data.
 */
#define ESB_MSG_TYPE_EVENT     0  /* generic peripheral event (input, sensor, battery) */
#define ESB_MSG_TYPE_KEY_STATE 1  /* full key-position bitmap */
#define ESB_MSG_TYPE_COMMAND   2  /* central command */

struct esb_msg_prefix {
    uint8_t magic_prefix[sizeof(ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX) - 1];
    uint8_t msg_type;
    uint8_t payload_size;
} __packed;

struct esb_command_payload {
    uint8_t source;
    struct zmk_split_transport_central_command cmd;
} __packed;

struct esb_command_envelope {
    struct esb_msg_prefix prefix;
    struct esb_command_payload payload;
} __packed;

struct esb_event_payload {
    uint8_t source;
    struct zmk_split_transport_peripheral_event event;
} __packed;

struct esb_event_envelope {
    struct esb_msg_prefix prefix;
    struct esb_event_payload payload;
} __packed;

/* 128 key positions encoded as a bitmask. */
#define ESB_KEY_STATE_LEN 16

struct esb_key_state_payload {
    uint8_t source;
    uint8_t state[ESB_KEY_STATE_LEN];
} __packed;

struct esb_key_state_envelope {
    struct esb_msg_prefix prefix;
    struct esb_key_state_payload payload;
} __packed;

struct esb_msg_postfix {
    uint32_t crc;
} __packed;

#define ESB_MSG_EXTRA_SIZE (sizeof(struct esb_msg_prefix) + sizeof(struct esb_msg_postfix))

typedef void (*zmk_split_esb_process_tx_callback_t)(void);

struct zmk_split_esb_async_state {
    atomic_t state;

    uint8_t *rx_bufs[2];
    size_t rx_bufs_len;
    size_t rx_size_process_trigger;

    struct ring_buf *tx_buf;
    struct ring_buf *rx_buf;

    zmk_split_esb_process_tx_callback_t process_tx_callback;

    const struct device *uart;

    struct k_work_delayable restart_rx_work;
    struct k_work *process_tx_work;
    const struct gpio_dt_spec *dir_gpio;
};

void zmk_split_esb_async_tx(struct zmk_split_esb_async_state *state);

void zmk_split_esb_cb(app_esb_event_t *event, struct zmk_split_esb_async_state *state);

int zmk_split_esb_get_item(struct ring_buf *rx_buf, uint8_t *env, size_t env_size);
