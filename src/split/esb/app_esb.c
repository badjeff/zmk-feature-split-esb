/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_esb.h"
#include "timeslot.h"
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <esb.h>

#include <zmk/events/activity_state_changed.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_esb, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);


#define DT_DRV_COMPAT zmk_esb_split
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define HAS_BASE_ADDR_0 (DT_INST_NODE_HAS_PROP(0, base_addr_0))
#define HAS_BASE_ADDR_1 (DT_INST_NODE_HAS_PROP(0, base_addr_1))
#define HAS_ADDR_PREFIX (DT_INST_NODE_HAS_PROP(0, addr_prefix))

#define BASE_ADDR_0_LEN (DT_INST_PROP_LEN(0, base_addr_0))
#define BASE_ADDR_1_LEN (DT_INST_PROP_LEN(0, base_addr_1))
#define ADDR_PREFIX_LEN (DT_INST_PROP_LEN(0, addr_prefix))

#if (!HAS_BASE_ADDR_0 || BASE_ADDR_0_LEN != 4)
#error "zmk,esb-split :: base-addr-0 must include 4 bytes"
#endif

#if (!HAS_BASE_ADDR_1 || BASE_ADDR_1_LEN != 4)
#error "zmk,esb-split :: base-addr-1 must include 4 bytes"
#endif

#if (!HAS_ADDR_PREFIX || ADDR_PREFIX_LEN != 8)
#error "zmk,esb-split :: base-addr-0 must include 8 bytes"
#endif

uint8_t esb_base_addr_0[4] = DT_INST_PROP(0, base_addr_0);
uint8_t esb_base_addr_1[4] = DT_INST_PROP(0, base_addr_1);
uint8_t esb_addr_prefix[4] = DT_INST_PROP(0, addr_prefix);

#else
#error "Need to create a node with compatible of 'zmk,esb-split` with `all `address` property set."
#endif

static app_esb_callback_t m_callback;

// Define a buffer of payloads to store TX payloads in between timeslots
K_MSGQ_DEFINE(m_msgq_tx_payloads, sizeof(struct esb_payload), 
              CONFIG_ZMK_SPLIT_ESB_PROTO_MSGQ_ITEMS, 4);

static app_esb_mode_t m_mode;
static bool m_active = false;
static bool m_enabled = false;

static int pull_packet_from_tx_msgq(void);

static void on_timeslot_start_stop(zmk_split_esb_timeslot_callback_type_t type);

static void event_handler(struct esb_evt const *event) {
    app_esb_event_t m_event;
    switch (event->evt_id) {
        case ESB_EVENT_TX_SUCCESS:
            // LOG_DBG("TX SUCCESS, tx_attempts: %d", event->tx_attempts);
            // LOG_DBG("give d1");
            // Forward an event to the application
            m_event.evt_type = APP_ESB_EVT_TX_SUCCESS;
            m_callback(&m_event);
            pull_packet_from_tx_msgq();
            break;
        case ESB_EVENT_TX_FAILED:
            LOG_WRN("TX FAILED, tx_attempts: %d", event->tx_attempts);
            // esb_flush_tx(); // DOUH, had fixed @ 3.1.0-rc1, not ready yet.
            // Forward an event to the application
            m_event.evt_type = APP_ESB_EVT_TX_FAIL;
            m_callback(&m_event);
            pull_packet_from_tx_msgq();
            break;
        case ESB_EVENT_RX_RECEIVED:
            // LOG_DBG("RX SUCCESS");
            struct esb_payload rx_payload;
            uint8_t buf[CONFIG_ESB_MAX_PAYLOAD_LENGTH];
            if (esb_read_rx_payload(&rx_payload) == 0) {
                // LOG_DBG("Chunk %d, len: %d", rx_payload.pid, rx_payload.length);
                memcpy(buf, rx_payload.data, rx_payload.length);
                // LOG_DBG("Packet len: %d", rx_payload.length);
                m_event.evt_type = APP_ESB_EVT_RX;
                m_event.buf = buf;
                m_event.data_length = rx_payload.length;
                m_callback(&m_event);
            }
            break;
    }
}

static int clocks_start(void) {
    int err;
    int res;
    struct onoff_manager *clk_mgr;
    struct onoff_client clk_cli;

    clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (!clk_mgr) {
        LOG_ERR("Unable to get the Clock manager");
        return -ENXIO;
    }

    sys_notify_init_spinwait(&clk_cli.notify);

    err = onoff_request(clk_mgr, &clk_cli);
    if (err < 0) {
        LOG_ERR("Clock request failed: %d", err);
        return err;
    }

    do {
        err = sys_notify_fetch_result(&clk_cli.notify, &res);
        if (!err && res) {
            LOG_ERR("Clock could not be started: %d", res);
            return res;
        }
    } while (err);

    LOG_DBG("HF clock started");
    return 0;
}

static int esb_initialize(app_esb_mode_t mode) {
    int err;
    struct esb_config config = ESB_DEFAULT_CONFIG;

    config.protocol = ESB_PROTOCOL_ESB_DPL;
    config.retransmit_delay = CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_DELAY;
    config.retransmit_count = CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_COUNT;
    config.bitrate = ESB_BITRATE_2MBPS;
    config.event_handler = event_handler;
    config.mode = (mode == APP_ESB_MODE_PTX) ? ESB_MODE_PTX : ESB_MODE_PRX;
    config.tx_mode = ESB_TXMODE_MANUAL_START;
    config.selective_auto_ack = true;

    err = esb_init(&config);

    if (err) {
        return err;
    }

    err = esb_set_base_address_0(esb_base_addr_0);
    if (err) {
        return err;
    }

    err = esb_set_base_address_1(esb_base_addr_1);
    if (err) {
        return err;
    }

    err = esb_set_prefixes(esb_addr_prefix, ARRAY_SIZE(esb_addr_prefix));
    if (err) {
        return err;
    }

    NVIC_SetPriority(RADIO_IRQn, 0);

    if (mode == APP_ESB_MODE_PRX) {
        esb_start_rx();
    }

    return 0;
}

#define ESB_TX_FIFO_REQUE_MAX (CONFIG_ZMK_SPLIT_ESB_PROTO_MSGQ_ITEMS \
                               * CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_COUNT)

static int pull_packet_from_tx_msgq(void) {
    int ret = 0;
    struct esb_payload tx_payload;
    static uint8_t que_was_fulled = 0;

    if (k_msgq_peek(&m_msgq_tx_payloads, &tx_payload) == 0) {
        ret = esb_write_payload(&tx_payload);

        if (ret == -ENOMEM) {
            LOG_WRN("esb_tx_fifo: queue full %d", que_was_fulled);

            // *** deprecated pre-emptive queuing logic ***
            // LOG_DBG("esb_tx_fifo: queue full, popping first message and queueing again");
            // ret = esb_pop_tx();
            // if (ret) {
            //     LOG_ERR("esb_tx_fifo: popping first message and queueing failed (%d)", ret);
            // }
            // ret = esb_write_payload(&tx_payload);
            // if (ret) {
            //     LOG_ERR("esb_write_payload failed (%d)", ret);
            // }

            // force dequeue, guarding for phantom PRX.
            que_was_fulled++;
            if (que_was_fulled >= ESB_TX_FIFO_REQUE_MAX) {
                esb_flush_tx();
                // dequeue FIFO msg
                k_msgq_get(&m_msgq_tx_payloads, &tx_payload, K_NO_WAIT);
            }

        } else if (ret == -EMSGSIZE) {
            LOG_WRN("esb_tx_fifo: tx_payload size too large (%d) > CONFIG_ESB_MAX_PAYLOAD_LENGTH (%d)",
                    tx_payload.length, CONFIG_ESB_MAX_PAYLOAD_LENGTH);
            // dequeue FIFO msg
            k_msgq_get(&m_msgq_tx_payloads, &tx_payload, K_NO_WAIT);

        } else if (ret) {
            LOG_WRN("esb_write_payload failed (%d)", ret);

        } else {
            // LOG_DBG("Payload len: %d", tx_payload.length);
            esb_start_tx();
            // dequeue FIFO msg
            k_msgq_get(&m_msgq_tx_payloads, &tx_payload, K_NO_WAIT);
            que_was_fulled = 0;
        }
    }

    esb_start_tx();
    return ret;
}

int zmk_split_esb_init(app_esb_mode_t mode, app_esb_callback_t callback) {
    int ret;
    m_callback = callback;
    m_mode = mode;
    ret = clocks_start();
    if (ret < 0) {
        return ret;
    }
    LOG_INF("Timeslothandler init");
    zmk_split_esb_timeslot_init(on_timeslot_start_stop);
    return 0;
}

int zmk_split_esb_set_enable(bool enabled) {
    m_enabled = enabled;
    if (enabled) {
        zmk_split_esb_timeslot_open_session();
        return 0;
    } else {
        zmk_split_esb_timeslot_close_session();
        return 0;
    }
}

int zmk_split_esb_send(app_esb_data_t *tx_packet) {
    int ret = 0;
    struct esb_payload tx_payload;
    tx_payload.pipe = 0;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_PROTO_TX_ACK)
    tx_payload.noack = false;
#else
    tx_payload.noack = true;
#endif
    memcpy(tx_payload.data, tx_packet->data, tx_packet->len);
    tx_payload.length = tx_packet->len;
    if (!tx_payload.length) {
        LOG_WRN("bypass queuing null payload");
        return 0;
    }
    ret = k_msgq_put(&m_msgq_tx_payloads, &tx_payload, K_NO_WAIT);

    // *** deprecated pre-emptive queuing logic ***
    // if (ret == -EAGAIN || ret == -ENOMSG) {
    //     LOG_WRN("esb tx_payload_q full, popping first message and queueing again");
    //     struct esb_payload dicarded_payload;
    //     k_msgq_get(&m_msgq_tx_payloads, &dicarded_payload, K_NO_WAIT);
    //     ret = k_msgq_put(&m_msgq_tx_payloads, &tx_payload, K_NO_WAIT);
    // }

    if (ret != 0) {
        LOG_WRN("Failed to queue esb tx_payload_q (%d)", ret);
    }
    if (m_active) {
        pull_packet_from_tx_msgq();
    }
    return ret;
}

static int app_esb_suspend(void) {
    m_active = false;
    if(m_mode == APP_ESB_MODE_PTX) {
        uint32_t irq_key = irq_lock();

        irq_disable(RADIO_IRQn);
        NVIC_DisableIRQ(RADIO_IRQn);

        NRF_RADIO->SHORTS = 0;

        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_DISABLE = 1;
        while(NRF_RADIO->EVENTS_DISABLED == 0);

        NRF_TIMER2->TASKS_STOP = 1;
        NRF_RADIO->INTENCLR = 0xFFFFFFFF;
        
        esb_disable();

        NVIC_ClearPendingIRQ(RADIO_IRQn);

        irq_unlock(irq_key);
    }
    else {
        esb_stop_rx();
    }

    // Todo: Figure out how to use the esb_suspend() function 
    // rather than having to disable at the end of every timeslot
    //esb_suspend();
    return 0;
}

static int app_esb_resume(void) {
    if(m_mode == APP_ESB_MODE_PTX) {
        int err = esb_initialize(m_mode);
        m_active = true;
        pull_packet_from_tx_msgq();
        return err;
    }
    else {
        int err = esb_initialize(m_mode);
        m_active = true;
        pull_packet_from_tx_msgq();
        return err;
    }
}

/* Callback function signalling that a timeslot is started or stopped */
static void on_timeslot_start_stop(zmk_split_esb_timeslot_callback_type_t type) {
    switch (type) {
        case APP_TS_STARTED:
            app_esb_resume();
            break;
        case APP_TS_STOPPED:
            app_esb_suspend();
            break;
    }
}

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);
    if (!state_ev) {
        return 0;
    }

    if (m_mode == APP_ESB_MODE_PTX) {
        if (state_ev->state != ZMK_ACTIVITY_ACTIVE && m_enabled) {
            zmk_split_esb_set_enable(false);
        }
        else if (state_ev->state == ZMK_ACTIVITY_ACTIVE && !m_enabled) {
            zmk_split_esb_set_enable(true);
        }
    }

    return 0;
}

ZMK_LISTENER(zmk_split_esb_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_split_esb_idle_sleeper, zmk_activity_state_changed);
