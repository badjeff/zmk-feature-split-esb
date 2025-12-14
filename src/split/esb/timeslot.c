/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include "timeslot.h"
#include <zephyr/irq.h>
#include <zephyr/sys/ring_buffer.h>
#include <hal/nrf_timer.h>

#include <mpsl_timeslot.h>
#include <mpsl.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(timeslot, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);


#define TIMESLOT_REQUEST_TIMEOUT_US  1000000
#define TIMESLOT_LENGTH_US           10000
#define TIMESLOT_EXT_MARGIN_MARGIN	 1000
#define TIMESLOT_REQ_EARLIEST_MARGIN 100
#define TIMER_EXPIRY_US_EARLY 		 (TIMESLOT_LENGTH_US - MPSL_TIMESLOT_EXTENSION_MARGIN_MIN_US - TIMESLOT_EXT_MARGIN_MARGIN)
#define TIMER_EXPIRY_REQ			 (TIMESLOT_LENGTH_US - MPSL_TIMESLOT_EXTENSION_MARGIN_MIN_US - TIMESLOT_REQ_EARLIEST_MARGIN)

#define MPSL_THREAD_PRIO             CONFIG_MPSL_THREAD_COOP_PRIO
#define STACKSIZE                    CONFIG_MAIN_STACK_SIZE

static zmk_split_esb_timeslot_callback_t m_callback;
static volatile bool m_sess_open = false;
static volatile bool m_in_timeslot = false;

// Declare the RADIO IRQ handler to supress warning
extern void *__ptr__radio_dynamic_irq_handler;
typedef void (*radio_irq_handler_t)(const void *args);

// Requests and callbacks to be run serialized from an SWI interrupt
enum mpsl_timeslot_call {
    REQ_OPEN_SESSION,
    REQ_MAKE_REQUEST,
    REQ_CLOSE_SESSION
};

// Timeslot request
static mpsl_timeslot_request_t timeslot_request_earliest = {
    .request_type = MPSL_TIMESLOT_REQ_TYPE_EARLIEST,
    .params.earliest.hfclk = MPSL_TIMESLOT_HFCLK_CFG_NO_GUARANTEE,
    .params.earliest.priority = MPSL_TIMESLOT_PRIORITY_NORMAL,
    .params.earliest.length_us = TIMESLOT_LENGTH_US,
    .params.earliest.timeout_us = TIMESLOT_REQUEST_TIMEOUT_US
};

static mpsl_timeslot_signal_return_param_t signal_callback_return_param;

// Message queue for requesting MPSL API calls to non-preemptible thread
K_MSGQ_DEFINE(mpsl_api_msgq, sizeof(enum mpsl_timeslot_call), 10, 4);

static void schedule_request(enum mpsl_timeslot_call call) {
    int err;
    if (call == REQ_OPEN_SESSION) {
        m_sess_open = true;
    }
    else if (call == REQ_CLOSE_SESSION) {
        m_sess_open = false;
    }
    enum mpsl_timeslot_call api_call = call;
    err = k_msgq_put(&mpsl_api_msgq, &api_call, K_NO_WAIT);
    if (err) {
        LOG_ERR("Message sent error: %d", err);
        k_oops();
    }
}

static void set_timeslot_active_status(bool active) {
    if (active) {
        if (!m_in_timeslot) {
            m_in_timeslot = true;
            m_callback(APP_TS_STARTED);
        }
    } else {
        if (m_in_timeslot) {
            m_in_timeslot = false;
            m_callback(APP_TS_STOPPED);
        }
    }
}

static mpsl_timeslot_signal_return_param_t *mpsl_timeslot_callback(mpsl_timeslot_session_id_t session_id, 
                                                                  uint32_t signal_type) {
    (void) session_id; // unused parameter
    static bool timeslot_extension_failed;
    // NRF_P0->OUTSET = BIT(28);
    mpsl_timeslot_signal_return_param_t *p_ret_val = NULL;
    switch (signal_type) {
        case MPSL_TIMESLOT_SIGNAL_START:
            LOG_DBG("TS start");
            signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
            p_ret_val = &signal_callback_return_param;

            timeslot_extension_failed = false;

            // Reset the radio to make sure no configuration remains from BLE
            NVIC_ClearPendingIRQ(RADIO_IRQn);
            NRF_RADIO->POWER = RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos;
            NRF_RADIO->POWER = RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos;
            NVIC_ClearPendingIRQ(RADIO_IRQn);

            nrf_timer_bit_width_set(NRF_TIMER0, NRF_TIMER_BIT_WIDTH_32);
            
            nrf_timer_cc_set(NRF_TIMER0, NRF_TIMER_CC_CHANNEL0, TIMER_EXPIRY_US_EARLY);
            nrf_timer_int_enable(NRF_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);

            nrf_timer_cc_set(NRF_TIMER0, NRF_TIMER_CC_CHANNEL1, TIMER_EXPIRY_REQ);
            nrf_timer_int_enable(NRF_TIMER0, NRF_TIMER_INT_COMPARE1_MASK);

            set_timeslot_active_status(true);
            break;

        case MPSL_TIMESLOT_SIGNAL_TIMER0:
            // LOG_DBG("Signal TIMER0");
            if (!m_sess_open) {
                signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
                p_ret_val = &signal_callback_return_param;
                break;
            }

            if(nrf_timer_event_check(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0)) {
                nrf_timer_int_disable(NRF_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);
                nrf_timer_event_clear(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0);

                signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_EXTEND;
                signal_callback_return_param.params.extend.length_us = TIMESLOT_LENGTH_US;	
            }
            else if(nrf_timer_event_check(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE1)) {
                nrf_timer_int_disable(NRF_TIMER0, NRF_TIMER_INT_COMPARE1_MASK);
                nrf_timer_event_clear(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE1);

                if(timeslot_extension_failed) {
                    signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST;
                    signal_callback_return_param.params.request.p_next = &timeslot_request_earliest;
                } else {
                    signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
                }
            }
            p_ret_val = &signal_callback_return_param;
            break;

        case MPSL_TIMESLOT_SIGNAL_EXTEND_SUCCEEDED:
            // LOG_DBG("Extend Succeeded");
            if (!m_sess_open) {
                signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
                p_ret_val = &signal_callback_return_param;
                break;
            }

            signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;

            // Set next trigger time to be the current + Timer expiry early
            uint32_t current_cc = nrf_timer_cc_get(NRF_TIMER0, NRF_TIMER_CC_CHANNEL0);
            nrf_timer_bit_width_set(NRF_TIMER0, NRF_TIMER_BIT_WIDTH_32);
            nrf_timer_cc_set(NRF_TIMER0, NRF_TIMER_CC_CHANNEL0, current_cc + TIMESLOT_LENGTH_US);
            nrf_timer_int_enable(NRF_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);

            current_cc = nrf_timer_cc_get(NRF_TIMER0, NRF_TIMER_CC_CHANNEL1);
            nrf_timer_bit_width_set(NRF_TIMER0, NRF_TIMER_BIT_WIDTH_32);
            nrf_timer_cc_set(NRF_TIMER0, NRF_TIMER_CC_CHANNEL1, current_cc + TIMESLOT_LENGTH_US);
            nrf_timer_int_enable(NRF_TIMER0, NRF_TIMER_INT_COMPARE1_MASK);

            p_ret_val = &signal_callback_return_param;
            break;

        case MPSL_TIMESLOT_SIGNAL_EXTEND_FAILED:
            LOG_DBG("Extend failed");	
            signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
            timeslot_extension_failed = true;
            p_ret_val = &signal_callback_return_param;
            set_timeslot_active_status(false);
            break;

        case MPSL_TIMESLOT_SIGNAL_RADIO:
            // LOG_DBG("radio");
            signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
            p_ret_val = &signal_callback_return_param;

            // We have to manually call the RADIO IRQ handler when the RADIO signal occurs
            if(m_in_timeslot) {
                radio_irq_handler_t radio_irq_handler = (radio_irq_handler_t)__ptr__radio_dynamic_irq_handler;
                radio_irq_handler(NULL);
            } else {
                NVIC_ClearPendingIRQ(RADIO_IRQn);
                NVIC_DisableIRQ(RADIO_IRQn);
            }
            break;

        case MPSL_TIMESLOT_SIGNAL_OVERSTAYED:
            LOG_WRN("something overstayed!");
            signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END;
            p_ret_val = &signal_callback_return_param;
            set_timeslot_active_status(false);
            break;

        case MPSL_TIMESLOT_SIGNAL_CANCELLED:
            LOG_DBG("something cancelled!");
            signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
            p_ret_val = &signal_callback_return_param;
            set_timeslot_active_status(false);
            
            // In this case returning SIGNAL_ACTION_REQUEST causes hardfault. 
            // We have to request a new timeslot instead, from thread context. 
            schedule_request(REQ_MAKE_REQUEST);
            break;

        case MPSL_TIMESLOT_SIGNAL_BLOCKED:
            LOG_DBG("something blocked!");
            signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
            p_ret_val = &signal_callback_return_param;
            set_timeslot_active_status(false);

            // Request a new timeslot in this case
            schedule_request(REQ_MAKE_REQUEST);
            break;

        case MPSL_TIMESLOT_SIGNAL_INVALID_RETURN:
            LOG_WRN("something gave invalid return");
            signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END;
            p_ret_val = &signal_callback_return_param;
            set_timeslot_active_status(false);
            break;

        case MPSL_TIMESLOT_SIGNAL_SESSION_IDLE:
            LOG_DBG("idle");

            // Request a new timeslot in this case
            schedule_request(REQ_MAKE_REQUEST);

            signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
            p_ret_val = &signal_callback_return_param;
            set_timeslot_active_status(false);
            break;

        case MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED:
            LOG_DBG("Session closed");

            signal_callback_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
            p_ret_val = &signal_callback_return_param;
            set_timeslot_active_status(false);
            break;

        default:
            LOG_ERR("unexpected signal: %u", signal_type);
            k_oops();
            break;
    }
    // NRF_P0->OUTCLR = BIT(28);
    return p_ret_val;
}

/* To ensure thread safe operation, call all MPSL APIs from a non-preemptible
 * thread.
 */
static void mpsl_nonpreemptible_thread(void) {
    int err;
    enum mpsl_timeslot_call api_call = 0;

    /* Initialize to invalid session id */
    mpsl_timeslot_session_id_t session_id = 0xFFu;

    while (1) {
        if (k_msgq_get(&mpsl_api_msgq, &api_call, K_FOREVER) == 0) {
            //NRF_P0->OUTSET = BIT(29);
            switch (api_call) {
                case REQ_OPEN_SESSION:
                    LOG_DBG("req open");
                    err = mpsl_timeslot_session_open(mpsl_timeslot_callback, &session_id);
                    if (err) {
                        LOG_ERR("Timeslot session open error: %d", err);
                        k_oops();
                    }
                    break;
                case REQ_MAKE_REQUEST:
                    LOG_DBG("req request");
                    err = mpsl_timeslot_request(session_id, &timeslot_request_earliest);
                    if (err) {
                        LOG_ERR("Timeslot request error: %d", err);
                        k_oops();
                    }
                    break;
                case REQ_CLOSE_SESSION:
                    LOG_DBG("req close");
                    err = mpsl_timeslot_session_close(session_id);
                    if (err) {
                        LOG_ERR("Timeslot session close error: %d", err);
                        k_oops();
                    }
                    break;
                default:
                    LOG_ERR("Wrong timeslot API call");
                    k_oops();
                    break;
            }
            //NRF_P0->OUTCLR = BIT(29);
        }
    }
}

void zmk_split_esb_timeslot_close_session(void) {
    schedule_request(REQ_CLOSE_SESSION);
}

void zmk_split_esb_timeslot_open_session(void) {
    schedule_request(REQ_OPEN_SESSION);
    schedule_request(REQ_MAKE_REQUEST);
}

void zmk_split_esb_timeslot_init(zmk_split_esb_timeslot_callback_t callback) {
    m_callback = callback;
}

K_THREAD_DEFINE(mpsl_nonpreemptible_thread_id, STACKSIZE,
        mpsl_nonpreemptible_thread, NULL, NULL, NULL,
        K_PRIO_COOP(MPSL_THREAD_PRIO), 0, 0);
