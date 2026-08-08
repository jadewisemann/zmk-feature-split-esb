/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_esb.h"
#include "common.h"
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <esb.h>
#include <zmk/events/endpoint_changed.h>

// for backoff logic
#include <zephyr/kernel.h>

#include <zmk/events/activity_state_changed.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_esb, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);
#define DT_DRV_COMPAT zmk_esb_split
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define IS_CENTRAL IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#define IS_PERIPHERAL !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

#if IS_CENTRAL
#define TX_POWER_INIT (4)
#define ENABLED_PIPES GENMASK(PERIPHERAL_COUNT - 1, 0)
#else
#define TX_POWER_INIT (-4)
#define ENABLED_PIPES BIT(PERIPHERAL_ID)
#endif

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_BITRATE_2MBPS)
#define BITRATE ESB_BITRATE_2MBPS
#else
#define BITRATE ESB_BITRATE_1MBPS
#endif

#if CONFIG_LOG
const char *ACTIVE_STATE_CHAR[] = {"ACTIVE", "IDLE", "SLEEP"};
const char *TX_POWER_CHAR[] = {"OK", "UP", "DOWN"};
#endif

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
#error "zmk,esb-split :: addr-prefix must include 8 bytes"
#endif

uint8_t esb_base_addr_0[4] = DT_INST_PROP(0, base_addr_0);
uint8_t esb_base_addr_1[4] = DT_INST_PROP(0, base_addr_1);
uint8_t esb_addr_prefix[8] = DT_INST_PROP(0, addr_prefix);

#else
#error "Need to create a node with compatible of 'zmk,esb-split` with `all `address` property set."
#endif

struct esb_context *esb_ctx;

void rx_work_handler(struct k_work *work);
K_WORK_DEFINE(rx_work, rx_work_handler);
struct k_work_q rx_workqueue;

void submit_rx_work(void) {
#if CONFIG_ZMK_SPLIT_ESB_USE_WORK_QUEUE_DEDICATED
    k_work_submit_to_queue(&rx_workqueue, &rx_work);
#else
    k_work_submit(&rx_work);
#endif
}

static void event_handler(struct esb_evt const *event);
static struct esb_config config = {
    .protocol = ESB_PROTOCOL_ESB_DPL,
    .mode = ESB_MODE_PTX,
    .bitrate = BITRATE,
    .crc = ESB_CRC_16BIT,
    .tx_mode = ESB_TXMODE_AUTO,
    .payload_length = CONFIG_ESB_MAX_PAYLOAD_LENGTH,
    .use_fast_ramp_up = true,
    .event_handler = event_handler,
    .tx_output_power = TX_POWER_INIT,
#if IS_PERIPHERAL
    .pipe = SOURCE_TO_PIPE(CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_ID),
#endif
};

static void event_handler(struct esb_evt const *event) {
    switch (event->evt_id) {
    case ESB_EVENT_TX_SUCCESS:
        LOG_DBG("TX SUCCESS: TRY %u", event->tx.tx_attempts);
        if (esb_ctx->tx_op)
            esb_ctx->tx_op();
        break;
    case ESB_EVENT_TX_FAILED:
        LOG_WRN("ESB_EVENT_TX_FAILED");
        break;
    case ESB_EVENT_RX_RECEIVED:
        LOG_DBG("RX SUCCESS");
        submit_rx_work();
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

static void init_workqueue(struct k_work_q *workqueue) {
    static K_THREAD_STACK_DEFINE(rx_thread_stack, CONFIG_ZMK_SPLIT_ESB_RX_THREAD_STACK_SIZE);

    k_work_queue_start(workqueue, rx_thread_stack, K_THREAD_STACK_SIZEOF(rx_thread_stack),
                       CONFIG_ZMK_SPLIT_ESB_RX_THREAD_PRIORITY, NULL);
}

static int esb_initialize(void) {
    LOG_DBG("esb init");

    int err = esb_init(&config);
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

    err = esb_enable_pipes(ENABLED_PIPES);
    if (err) {
        return err;
    }

    // NVIC_SetPriority(RADIO_IRQn, 0);

    LOG_DBG("esb init done");

    return 0;
}

int zmk_split_esb_init(void) {
    config.mode = IS_PERIPHERAL ? ESB_MODE_PTX : ESB_MODE_PRX;
    // ret = clocks_start();
    // if (ret < 0) {
    //     return ret;
    // }

    int ret = esb_initialize();
    if (ret) {
        /* Starting TDMA on top of a radio that never came up leaves the
         * transport permanently half-dead instead of visibly failing. */
        LOG_ERR("esb_initialize failed (err %d)", ret);
        return ret;
    }

#if CONFIG_ZMK_SPLIT_ESB_USE_WORK_QUEUE_DEDICATED
    init_workqueue(&rx_workqueue);
#endif

    ret = esb_tdma_start();
    if (ret) {
        LOG_ERR("esb_tdma_start failed (err %d)", ret);
        return ret;
    }

    return 0;
}

static bool is_valid_source(const uint8_t source) {
    return BIT(SOURCE_TO_PIPE(source)) & ENABLED_PIPES;
}

int send_data_pl(uint8_t pipe, uint8_t type, void *data) {
    ssize_t data_size = esb_ctx->tx_size(type);
    if (data_size < 0) {
        LOG_WRN("invalid type(%d)", type);

        return -EINVAL;
    }

    struct esb_payload payload = {.pipe = pipe};

    int idx = 0;
    payload.length = data_size + 1;

    payload.data[idx++] = type;
    memcpy(&payload.data[idx], data, data_size);

    int err = esb_write_payload(&payload);
    if (err) {
        LOG_DBG("esb_write_payload error(%d).", err);

        return err;
    }

    return 0;
}

int send_data_rb(uint8_t pipe, uint8_t type, void *data) {
    ssize_t data_size = esb_ctx->tx_size(type);
    if (data_size < 0) {
        LOG_WRN("invalid type(%d)", type);

        return -EINVAL;
    }

    if (esb_tx_start(pipe, data_size + 2) == -ENOMEM) {
        return -ENOMEM;
    }

    uint8_t header = HEADER;
    esb_tx_put(&header, sizeof(header));
    esb_tx_put(&type, sizeof(type));
    esb_tx_put(data, data_size);

    esb_tx_finish();

    return 0;
}

int send_data(uint8_t source, uint8_t type, void *data) {
    if (!is_valid_source(source)) {
        LOG_WRN("invalid source(%d)", source);

        return -EINVAL;
    }

#if CONFIG_ESB_TX_RINGBUF
    return send_data_rb(SOURCE_TO_PIPE(source), type, data);
#else
    return send_data_pl(SOURCE_TO_PIPE(source), type, data);
#endif
}

#define RXBUF_SIZE (CONFIG_ESB_MAX_PAYLOAD_LENGTH * 3)

/* `length` below is a uint8_t, and the reassembly loop hands `len - used - 1`
 * to the rx handler as a uint8_t. Both break silently once the buffer can hold
 * more than 255 bytes. */
BUILD_ASSERT(RXBUF_SIZE <= UINT8_MAX,
             "CONFIG_ESB_MAX_PAYLOAD_LENGTH is too large: RXBUF_SIZE must fit in a uint8_t");

static struct rx_buffer {
    uint8_t length;
    uint8_t data[RXBUF_SIZE];
} rxbuf[PERIPHERAL_COUNT];

static struct rx_buffer *get_buffer(int source) {
#if IS_CENTRAL
    return &rxbuf[SOURCE_TO_PIPE(source)];
#else
    return &rxbuf[0];
#endif
}

static int handle_data_dr(void) {
    struct esb_payload payload;
    int ret = esb_read_rx_payload(&payload);
    if (ret != 0) {
        LOG_DBG("esb_read_rx_payload returned err(%d)", ret);

        return -ENODATA;
    }

    int source = PIPE_TO_SOURCE(payload.pipe);
    if (!is_valid_source(source)) {
        LOG_WRN("invalid source(%d)", source);

        return -EINVAL;
    }

    LOG_HEXDUMP_DBG(payload.data, payload.length, "rx ");

    esb_ctx->rx_handler(source, payload.data, payload.length);

    return 0;
}

static int handle_data_rb(void) {
    struct esb_payload payload;
    int ret = esb_read_rx_payload(&payload);
    if (ret != 0) {
        LOG_DBG("esb_read_rx_payload returned err(%d)", ret);

        return -ENODATA;
    }

    LOG_DBG("pipe %d len %d", payload.pipe, payload.length);

    int source = PIPE_TO_SOURCE(payload.pipe);
    if (!is_valid_source(source)) {
        LOG_WRN("invalid source(%d)", source);

        return -EINVAL;
    }

    struct rx_buffer *buffer = get_buffer(source);

    size_t len = buffer->length;
    uint8_t *buf = buffer->data;

    if (payload.length > RXBUF_SIZE - len) {
        /* Whatever is buffered can never be completed now, and returning
         * -ENOMEM would wedge this source forever: nothing else drains the
         * reassembly buffer. Drop it and resynchronise on the next header. */
        LOG_WRN("rx buffer overflow on source %d, dropping %u buffered byte(s)", source,
                (unsigned)len);
        buffer->length = 0;
        len = 0;

        if (payload.length > RXBUF_SIZE) {
            return 0;
        }
    }

    memcpy(&buf[len], payload.data, payload.length);
    len += payload.length;

    size_t used = 0;
    while (used < len) {
        if (buf[used] != HEADER) {
            used++;
            continue;
        }

        /* The handler dereferences data[0] unconditionally, so never call it
         * with an empty frame body - a packet that ends right after a header
         * byte is normal, the rest arrives in the next one. */
        size_t avail = len - used - 1;
        if (avail == 0) {
            break;
        }

        int consumed = esb_ctx->rx_handler(source, &buf[used + 1], (uint8_t)avail);
        if (consumed == 0) {
            /* Incomplete frame, wait for the remainder. */
            break;
        }

        if (consumed < 0 || (size_t)consumed > avail) {
            /* Corrupt frame: skip this header byte and resynchronise rather
             * than letting `used` run past `len`. */
            LOG_WRN("rx handler rejected frame on source %d (consumed %d, avail %u)", source,
                    consumed, (unsigned)avail);
            used++;
            continue;
        }

        used += (size_t)consumed + 1;
    }

    size_t left = len - used;
    if (left > 0 && used > 0) {
        memmove(buf, &buf[used], left);
    }
    buffer->length = (uint8_t)left;

    return 0;
}

int handle_data(void) {
#if CONFIG_ESB_TX_RINGBUF
    return handle_data_rb();
#else
    return handle_data_dr();
#endif
}

void rx_work_handler(struct k_work *work) {
    while (handle_data() != -ENODATA) {
    }
}

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);
    if (!state_ev) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    LOG_DBG("activity state changed: %s", ACTIVE_STATE_CHAR[state_ev->state]);

    switch (state_ev->state) {
    case ZMK_ACTIVITY_ACTIVE:
        /* -EBUSY here just means TDMA was never stopped, which is the normal
         * case on the central. */
        esb_tdma_start();
        break;
    case ZMK_ACTIVITY_IDLE:
#if IS_PERIPHERAL
        esb_tdma_stop(false);
#endif
        break;
    case ZMK_ACTIVITY_SLEEP:
#if IS_PERIPHERAL
        /* Only a peripheral shuts the radio down before deep sleep - it is
         * woken by its own matrix, not by the radio. The central must keep
         * listening: stopping it here takes the whole keyboard down and
         * nothing brings it back until the dongle is re-plugged. */
        esb_tdma_stop(true);
#endif
        break;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zmk_split_esb_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_split_esb_idle_sleeper, zmk_activity_state_changed);
