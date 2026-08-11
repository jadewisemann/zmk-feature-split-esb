/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/types.h>
#include <zephyr/init.h>

#include <zephyr/settings/settings.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);

#include <zmk/stdlib.h>
#include <zmk/behavior.h>
#include <zmk/sensors.h>
#include <zmk/split/transport/peripheral.h>
#include <zmk/split/transport/types.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/split_central_status_changed.h>
#include <zmk/battery.h>
#include <zmk/events/sensor_event.h>
#include <zmk/pointing/input_split.h>
#include <zmk/hid_indicators_types.h>
#include <zmk/physical_layouts.h>

// #include "timeslot.h"
#include "app_esb.h"
#include "common.h"

static struct peripheral_slot_context slot;
static int
split_peripheral_esb_report_event(const struct zmk_split_transport_peripheral_event *event);
static int peripheral_handler_rb(int source, uint8_t *data, uint8_t size);

static void battery_report_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(battery_report_work, battery_report_work_handler);
static void battery_report_work_handler(struct k_work *work) {
    struct zmk_split_transport_peripheral_event event = {
        .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT,
        .data.battery_event.level = zmk_battery_state_of_charge(),
    };

    if (split_peripheral_esb_report_event(&event) == -ENOMEM) {
        k_work_reschedule(&battery_report_work, K_SECONDS(1));
    }
}

static void notify_transport_status(void);
static void notify_status_work_cb(struct k_work *_work) { notify_transport_status(); }
static K_WORK_DEFINE(notify_status_work, notify_status_work_cb);

bool is_central_connected = false;

#if CONFIG_ZMK_SPLIT_ESB_SEND_WHOLE_KEY
/* Recovery path for lost whole-key state.
 *
 * The ARQ layer only protects a frame while the link is up. Two things step
 * around it entirely: esb_tdma_stop() calls reset_tx_all(), which discards
 * whatever is still queued, and send_data() fails with -ENOMEM once the TX
 * ring fills while desynced - a failure the core listener ignores. In both
 * cases slot.position_state was already updated before the send was attempted,
 * so the event exists nowhere else: the central keeps stale key state until
 * this half happens to send another key event. If what was lost carried a
 * release, that reads as a key stuck down until the next keypress on this
 * half.
 *
 * A whole-key frame carries the complete bitmap, so resending it is
 * idempotent. Recovery is therefore just "send the current state again":
 * armed on every failed send, and on every (re)connect to cover frames wiped
 * by reset_tx_all() or a reboot of either side (after a reboot the zeroed
 * bitmap is exactly what releases keys the central still holds).
 *
 * No locking is needed around slot.position_state: the core raises peripheral
 * events on the system workqueue and this work runs there too.
 */
#define STATE_RESYNC_RETRY_MS 20

static void state_resync_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(state_resync_work, state_resync_work_handler);
static void state_resync_work_handler(struct k_work *work) {
    if (!is_central_connected) {
        /* central_connected() re-arms this; retrying while the link is down
         * would only refill the TX ring that reset_tx_all() just cleared. */
        return;
    }

    if (send_data(PERIPHERAL_ID, ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT,
                  slot.position_state) < 0) {
        k_work_reschedule(&state_resync_work, K_MSEC(STATE_RESYNC_RETRY_MS));
    }
}
#endif

void central_connected(int pipe) {
    if (is_central_connected) {
        return;
    }

    is_central_connected = true;
    k_work_submit(&notify_status_work);
    k_work_reschedule(&battery_report_work, K_NO_WAIT);
#if CONFIG_ZMK_SPLIT_ESB_SEND_WHOLE_KEY
    k_work_reschedule(&state_resync_work, K_NO_WAIT);
#endif
}

void central_disconnected(int pipe) {
    if (!is_central_connected) {
        return;
    }

    is_central_connected = false;
    k_work_submit(&notify_status_work);
    if ((zmk_activity_get_state() != ZMK_ACTIVITY_ACTIVE)) {
        esb_tdma_stop(false);
    }
}

static struct esb_conn_cb conn_cb = {
    .connected = central_connected,
    .disconnected = central_disconnected,
};

static struct esb_context ctx = {.rx_handler = peripheral_handler_rb,
                                 .rx_size = get_payload_data_size_cmd,
                                 .tx_size = get_payload_data_size_evt};

static bool is_enabled = false;

static int
split_peripheral_esb_report_event(const struct zmk_split_transport_peripheral_event *event) {
    int ret = 0;

    switch (event->type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
        if (!change_position_state(event->data.key_position_event.position,
                                   event->data.key_position_event.pressed, slot.position_state)) {
            break;
        }
#if CONFIG_ZMK_SPLIT_ESB_SEND_WHOLE_KEY
        ret = send_data(PERIPHERAL_ID, event->type, slot.position_state);
        if (ret < 0) {
            /* slot.position_state was already flipped above, so this event no
             * longer exists anywhere else - keep retrying the bitmap until it
             * gets through or the reconnect path takes over. */
            k_work_reschedule(&state_resync_work, K_MSEC(STATE_RESYNC_RETRY_MS));
        }
        break;
#endif
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT:
        ret = send_data(PERIPHERAL_ID, event->type, &event->data);
        break;

    default:
        break;
    }

    return ret;
}

static int split_peripheral_esb_set_enabled(bool enabled) {
    is_enabled = enabled;

    return 0;
}

static zmk_split_transport_peripheral_status_changed_cb_t transport_status_cb;
static int
split_peripheral_esb_set_status_callback(zmk_split_transport_peripheral_status_changed_cb_t cb) {
    transport_status_cb = cb;
    return 0;
}

static struct zmk_split_transport_status split_peripheral_esb_get_status(void) {
    // return (struct zmk_split_transport_status){
    //     .available = esb_is_initialized(),
    //     .enabled = is_enabled,
    //     .connections = is_central_connected ?
    //     ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED
    //                                         :
    //                                         ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_DISCONNECTED,
    // };

    return (struct zmk_split_transport_status){
        .available = true,
        .enabled = is_enabled,
        .connections = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED,
    };
}

static const struct zmk_split_transport_peripheral_api peripheral_api = {
    .report_event = split_peripheral_esb_report_event,
    .set_enabled = split_peripheral_esb_set_enabled,
    .set_status_callback = split_peripheral_esb_set_status_callback,
    .get_status = split_peripheral_esb_get_status,
};

ZMK_SPLIT_TRANSPORT_PERIPHERAL_REGISTER(esb_peripheral, &peripheral_api,
                                        CONFIG_ZMK_SPLIT_ESB_PRIORITY);

static void notify_transport_status(void) {
    if (transport_status_cb) {
        transport_status_cb(&esb_peripheral, split_peripheral_esb_get_status());
    }
}

static int zmk_split_esb_peripheral_init(void) {
    esb_ctx = &ctx;
    print_reset_reason();

    init_slot(&slot);
    esb_conn_cb_register(&conn_cb);

    int ret = zmk_split_esb_init();
    if (ret < 0) {
        LOG_ERR("zmk_split_esb_init failed (ret %d)", ret);
        return ret;
    }

    k_work_submit(&notify_status_work);
    return 0;
}

SYS_INIT(zmk_split_esb_peripheral_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static int peripheral_handler_rb(int source, uint8_t *data, uint8_t size) {
    if (size < 1) {
        return 0;
    }

    uint8_t type = data[0];
    ssize_t data_size = get_payload_data_size_cmd(type);

    if (data_size < 0) {
        return 1;
    }

    if (size < data_size + 1) {
        return 0;
    }

    struct zmk_split_transport_central_command cmd = {.type = type};
    memcpy(&cmd.data, &data[1], data_size);
    zmk_split_transport_peripheral_command_handler(&esb_peripheral, cmd);

    return data_size + 1;
}
