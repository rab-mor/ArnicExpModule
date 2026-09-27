/**
 * rs485_slave.c - F103 expansion module
 *
 * LinkTask: this module's side of the RS485 bus (rs485_proto.h). It runs in
 * CubeMX's defaultTask (freertos.c calls Rs485Slave_Run()).
 *
 *   - Address: the four DIP switches, read once at boot. DIP_1 is the least
 *     significant bit; a switch that is ON pulls its line to 0 V and counts
 *     as 1. Changing the switches takes effect at the next reset.
 *   - Receive: one circular DMA on USART1, started once and left running.
 *     The receive-event callback (half, full, line idle) only wakes the task.
 *     Every node hears every frame, including its own replies (/RE is tied
 *     low): replies are dropped, and requests for other addresses too.
 *   - Transmit: UART_EN (PA8) high, DMA out, and UART_EN low again from the
 *     transmit-complete interrupt, i.e. as soon as the last stop bit has
 *     left. A reply is one frame, well under the ~80 ms the DE anti-jabber
 *     network allows.
 *   - Events (VERIFIED / FAULT from RelayTask) are kept in a small buffer,
 *     each with an id, and sent in every reply until the H7 acknowledges
 *     them in a later request. A reply lost to noise loses nothing.
 *   - A relay command whose cmd_seq matches the last one accepted for that
 *     relay is a retry of a command whose reply was lost: it is answered
 *     DUPLICATE and not queued again.
 */

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "main.h"
#include "usart.h"
#include "board_config.h"
#include "app_shared.h"
#include "rs485_proto.h"
#include "exp_tasks.h"

#define RX_DMA_BYTES        512u
#define EVT_RING            8u          /* > RS485_EVENTS_MAX: room while unacked */
#define DE_SETTLE_LOOPS     200u        /* ~15 us: transceiver driver on before the start bit */

link_dbg_t        g_link_dbg;
volatile uint32_t g_hb_link;

static uint8_t         s_rx_dma[RX_DMA_BYTES];
static uint8_t         s_tx[RS485_MAX_FRAME];
static uint8_t         s_payload[RS485_MAX_PAYLOAD];
static rs485_parser_t  s_parser;
static uint16_t        s_rd;

static uint8_t         s_addr;
static uint16_t        s_nonce;
static bool            s_have_nonce;

/* Events, oldest first. Written by RelayTask (link_post_event), read and
   trimmed by LinkTask: both under a critical section. */
static rs485_event_t   s_evt[EVT_RING];
static uint8_t         s_evt_head;
static uint8_t         s_evt_count;
static uint16_t        s_evt_next_id = 1u;
static uint16_t        s_evt_dropped;

static uint16_t        s_last_seq[RELAY_COUNT];
static bool            s_have_seq[RELAY_COUNT];

/* ------------------------------- Events ---------------------------------- */

void link_post_event(const link_event_t *evt)
{
    taskENTER_CRITICAL();
    if (s_evt_count == EVT_RING) {
        s_evt_head = (uint8_t)((s_evt_head + 1u) % EVT_RING);   /* drop the oldest */
        s_evt_count--;
        s_evt_dropped++;
    }
    rs485_event_t *e = &s_evt[(s_evt_head + s_evt_count) % EVT_RING];
    e->id          = s_evt_next_id;
    e->type        = evt->type;
    e->relay_idx   = evt->relay_idx;
    e->cmd_seq     = evt->seq;
    e->measured_ma = evt->measured_ma;
    s_evt_count++;
    if (++s_evt_next_id == 0u) {
        s_evt_next_id = 1u;             /* 0 means "nothing acked yet" */
    }
    taskEXIT_CRITICAL();
}

static void events_ack(const rs485_req_hdr_t *h)
{
    if (!s_have_nonce || h->nonce != s_nonce) {
        return;                         /* acks from before this boot */
    }
    taskENTER_CRITICAL();
    while (s_evt_count > 0u && (int16_t)(h->ack_evt - s_evt[s_evt_head].id) >= 0) {
        s_evt_head = (uint8_t)((s_evt_head + 1u) % EVT_RING);
        s_evt_count--;
    }
    taskEXIT_CRITICAL();
}

/* ------------------------------- Identity -------------------------------- */

static uint8_t read_dip(void)
{
    uint8_t a = 0u;
    if (HAL_GPIO_ReadPin(DIP_1_GPIO_Port, DIP_1_Pin) == GPIO_PIN_RESET) { a |= 1u; }
    if (HAL_GPIO_ReadPin(DIP_2_GPIO_Port, DIP_2_Pin) == GPIO_PIN_RESET) { a |= 2u; }
    if (HAL_GPIO_ReadPin(DIP_3_GPIO_Port, DIP_3_Pin) == GPIO_PIN_RESET) { a |= 4u; }
    if (HAL_GPIO_ReadPin(DIP_4_GPIO_Port, DIP_4_Pin) == GPIO_PIN_RESET) { a |= 8u; }
    return a;
}

static uint32_t uid_hash(void)
{
    /* F103 96-bit unique ID at 0x1FFFF7E8. FNV-1a over its 12 bytes. */
    const uint8_t *uid = (const uint8_t *)UID_BASE;
    uint32_t h = 2166136261u;
    for (uint32_t i = 0u; i < 12u; i++) {
        h ^= uid[i];
        h *= 16777619u;
    }
    return h;
}

/* The nonce is fixed when the first request arrives: that moment, relative
   to our own boot, depends on when the H7 happened to poll, so it differs
   from boot to boot even when everything powers up together. */
static void make_nonce(void)
{
    if (s_have_nonce) {
        return;
    }
    uint32_t x = uid_hash() ^ (HAL_GetTick() << 7) ^ SysTick->VAL ^ (TIM3->CNT << 16);
    x ^= x >> 16;
    s_nonce = (uint16_t)x;
    if (s_nonce == 0u) {
        s_nonce = 1u;
    }
    s_have_nonce = true;
    g_link_dbg.nonce = s_nonce;
}

/* -------------------------------- UART ----------------------------------- */

static void rx_start(void)
{
    (void)HAL_UART_AbortReceive(&huart1);
    s_rd = 0u;
    rs485_parser_reset(&s_parser);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx_dma, RX_DMA_BYTES);
}

static uint16_t rx_write_pos(void)
{
    const uint32_t left = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    return (uint16_t)((RX_DMA_BYTES - left) % RX_DMA_BYTES);
}

static void send(uint16_t n)
{
    (void)osThreadFlagsClear(LINK_FLAG_TX_DONE);

    HAL_GPIO_WritePin(UART_EN_GPIO_Port, UART_EN_Pin, GPIO_PIN_SET);
    for (volatile uint32_t i = 0u; i < DE_SETTLE_LOOPS; i++) { }

    if (HAL_UART_Transmit_DMA(&huart1, s_tx, n) != HAL_OK) {
        HAL_GPIO_WritePin(UART_EN_GPIO_Port, UART_EN_Pin, GPIO_PIN_RESET);
        g_link_dbg.tx_timeouts++;
        return;
    }

    const uint32_t fl = osThreadFlagsWait(LINK_FLAG_TX_DONE, osFlagsWaitAny, RS485_TX_TIMEOUT_MS);
    if ((fl & osFlagsError) != 0u) {
        (void)HAL_UART_AbortTransmit(&huart1);
        HAL_GPIO_WritePin(UART_EN_GPIO_Port, UART_EN_Pin, GPIO_PIN_RESET);
        g_link_dbg.tx_timeouts++;
        return;
    }
    g_link_dbg.replies++;
}

/* ------------------------------- Replies --------------------------------- */

static uint8_t build_status(uint8_t cmd_result)
{
    const app_snapshot_t *snap = &g_snap[g_snap_active];
    rs485_status_t st;
    memset(&st, 0, sizeof(st));

    st.nonce     = s_nonce;
    st.window_id = snap->window_id;
    for (uint8_t k = 0u; k < RELAY_COUNT && k < RS485_RELAYS_PER_MODULE; k++) {
        int32_t ma = snap->relay_ma[k];
        if (ma >  32767) { ma =  32767; }
        if (ma < -32768) { ma = -32768; }
        st.relay_ma[k] = (int16_t)ma;
    }
    st.rail_24v_mv    = snap->rail_24V_mv;
    st.rail_5v_mv     = snap->rail_5V_mv;
    st.relay_state    = snap->relay_state;
    st.relay_known    = snap->relay_known;
    st.relay_evidence = snap->relay_evidence;
    st.fault_flags    = snap->fault_flags;
    st.store_flags    = snap->store_flags;
    st.cmd_result     = cmd_result;

    taskENTER_CRITICAL();
    st.dropped_events = s_evt_dropped;
    uint8_t n = s_evt_count;
    if (n > RS485_EVENTS_MAX) { n = RS485_EVENTS_MAX; }
    for (uint8_t e = 0u; e < n; e++) {
        st.events[e] = s_evt[(s_evt_head + e) % EVT_RING];
    }
    st.event_count = n;
    taskEXIT_CRITICAL();

    g_link_dbg.events_dropped = st.dropped_events;
    return rs485_encode_status(s_payload, &st);
}

static uint8_t build_info(void)
{
    rs485_info_t info;
    info.proto_version = RS485_PROTO_VERSION;
    info.addr          = s_addr;
    info.relay_count   = (uint8_t)RELAY_COUNT;
    info.board_type    = (uint8_t)BOARD_TYPE_EXPANSION;
    info.fw_version    = (uint16_t)((FW_VERSION_MAJOR << 8) | FW_VERSION_MINOR);
    info.nonce         = s_nonce;
    info.uptime_s      = HAL_GetTick() / 1000u;
    info.uid_hash      = uid_hash();
    return rs485_encode_info(s_payload, &info);
}

static uint8_t handle_relay(const rs485_relay_req_t *r)
{
    if (r->relay_idx >= RELAY_COUNT || r->target > 1u) {
        return RS485_CMD_REJECTED;
    }
    if (s_have_seq[r->relay_idx] && s_last_seq[r->relay_idx] == r->cmd_seq) {
        g_link_dbg.cmds_duplicate++;
        return RS485_CMD_DUPLICATE;     /* our last reply was lost; already queued */
    }
    const relay_cmd_t cmd = { r->relay_idx, r->target, r->cmd_seq };
    if (osMessageQueuePut(relay_cmd_qHandle, &cmd, 0u, 0u) != osOK) {
        g_link_dbg.cmds_busy++;
        return RS485_CMD_BUSY;
    }
    s_last_seq[r->relay_idx] = r->cmd_seq;
    s_have_seq[r->relay_idx] = true;
    g_link_dbg.cmds_queued++;
    return RS485_CMD_ACCEPTED;
}

static void handle_frame(const rs485_frame_t *f)
{
    if ((f->type & RS485_REPLY_FLAG) != 0u) {
        return;                         /* a module's reply, maybe our own */
    }
    if (f->addr != s_addr) {
        g_link_dbg.other_addr++;
        return;
    }

    g_link_dbg.requests++;
    g_link_dbg.last_request_ms = HAL_GetTick();
    make_nonce();
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);   /* activity */

    uint8_t len = 0u;
    switch (f->type) {
        case RS485_MSG_PING: {
            rs485_req_hdr_t h;
            if (rs485_decode_req_hdr(f->payload, f->len, &h)) {
                events_ack(&h);
            }
            len = build_info();
            break;
        }
        case RS485_MSG_STATUS: {
            rs485_req_hdr_t h;
            if (!rs485_decode_req_hdr(f->payload, f->len, &h)) {
                return;
            }
            events_ack(&h);
            len = build_status(RS485_CMD_NONE);
            break;
        }
        case RS485_MSG_RELAY: {
            rs485_relay_req_t r;
            if (!rs485_decode_relay_req(f->payload, f->len, &r)) {
                return;
            }
            events_ack(&r.hdr);
            len = build_status(handle_relay(&r));
            break;
        }
        default:
            return;                     /* unknown: no reply, the master times out */
    }

    const uint16_t n = rs485_build(s_tx, s_addr, (uint8_t)(f->type | RS485_REPLY_FLAG),
                                   f->seq, s_payload, len);
    if (n > 0u) {
        send(n);
    }
}

static void drain(void)
{
    const uint16_t wr = rx_write_pos();
    while (s_rd != wr) {
        const uint8_t b = s_rx_dma[s_rd];
        s_rd = (uint16_t)((s_rd + 1u) % RX_DMA_BYTES);

        rs485_frame_t f;
        if (rs485_parser_feed(&s_parser, b, &f)) {
            handle_frame(&f);
        }
    }
    g_link_dbg.crc_errors = s_parser.crc_errors;
}

/* -------------------------------- Task ----------------------------------- */

void Rs485Slave_Run(void *argument)
{
    (void)argument;

    HAL_GPIO_WritePin(UART_EN_GPIO_Port, UART_EN_Pin, GPIO_PIN_RESET);
    osDelay(10u);                       /* DIP lines settle on their pull-ups */
    s_addr = read_dip();
    g_link_dbg.addr = s_addr;

    rx_start();

    for (;;) {
        const uint32_t fl = osThreadFlagsWait(LINK_FLAG_RX | LINK_FLAG_UART_ERROR,
                                              osFlagsWaitAny, RS485_IDLE_MS);
        g_hb_link++;

        if (((fl & osFlagsError) == 0u) && ((fl & LINK_FLAG_UART_ERROR) != 0u)) {
            g_link_dbg.uart_errors++;
        }
        if (huart1.RxState != HAL_UART_STATE_BUSY_RX) {
            rx_start();                 /* an overrun stopped the receive DMA */
        }

        drain();

        /* Red LED: any fault, or no request from the H7 for 3 s. */
        const app_snapshot_t *snap = &g_snap[g_snap_active];
        const bool quiet = (uint32_t)(HAL_GetTick() - g_link_dbg.last_request_ms) > 3000u;
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin,
                          (snap->fault_flags != 0u || quiet) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/* ---------------------------- HAL callbacks ------------------------------ */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    (void)Size;
    if (huart->Instance == USART1 && LinkTaskHandle != NULL) {
        (void)osThreadFlagsSet(LinkTaskHandle, LINK_FLAG_RX);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* Last stop bit is out: release the bus at once. */
        HAL_GPIO_WritePin(UART_EN_GPIO_Port, UART_EN_Pin, GPIO_PIN_RESET);
        if (LinkTaskHandle != NULL) {
            (void)osThreadFlagsSet(LinkTaskHandle, LINK_FLAG_TX_DONE);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1 && LinkTaskHandle != NULL) {
        (void)osThreadFlagsSet(LinkTaskHandle, LINK_FLAG_UART_ERROR);
    }
}
