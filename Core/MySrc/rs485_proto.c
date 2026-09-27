/**
 * @file  rs485_proto.c
 * @brief See rs485_proto.h. Shared verbatim by the H7 and the expansion
 *        module. Byte-by-byte little-endian packing, so struct layout and
 *        alignment never reach the wire.
 */

#include "rs485_proto.h"
#include <string.h>

static inline void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline uint16_t get16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static inline uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint16_t rs485_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0u; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0u; b < 8u; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ------------------------------ Parser ---------------------------------- */

void rs485_parser_reset(rs485_parser_t *p)
{
    p->have = 0u;
    p->need = 0u;
}

bool rs485_parser_feed(rs485_parser_t *p, uint8_t byte, rs485_frame_t *out)
{
    if (p->have == 0u) {
        if (byte != RS485_SYNC0) { p->sync_drops++; return false; }
        p->buf[p->have++] = byte;
        return false;
    }
    if (p->have == 1u) {
        if (byte != RS485_SYNC1) {
            p->sync_drops++;
            /* This byte may itself start the next frame. */
            p->have = (byte == RS485_SYNC0) ? 1u : 0u;
            return false;
        }
        p->buf[p->have++] = byte;
        return false;
    }

    p->buf[p->have++] = byte;

    if (p->have == RS485_HDR_BYTES) {
        const uint8_t len = p->buf[5];
        if (len > RS485_MAX_PAYLOAD) {
            p->sync_drops += p->have;
            p->have = 0u;
            return false;
        }
        p->need = (uint16_t)(RS485_HDR_BYTES + len + RS485_CRC_BYTES);
        return false;
    }

    if (p->have < RS485_HDR_BYTES || p->have < p->need) {
        return false;
    }

    /* Whole frame in. */
    const uint8_t  len  = p->buf[5];
    const uint16_t crc  = get16(&p->buf[RS485_HDR_BYTES + len]);
    const uint16_t calc = rs485_crc16(&p->buf[2], (size_t)(RS485_HDR_BYTES - 2u + len));
    p->have = 0u;
    p->need = 0u;

    if (crc != calc) {
        p->crc_errors++;
        return false;
    }

    out->addr    = p->buf[2];
    out->type    = p->buf[3];
    out->seq     = p->buf[4];
    out->len     = len;
    out->payload = &p->buf[RS485_HDR_BYTES];
    return true;
}

/* ------------------------------ Encoding -------------------------------- */

uint16_t rs485_build(uint8_t *out, uint8_t addr, uint8_t type, uint8_t seq,
                     const uint8_t *payload, uint8_t len)
{
    if (len > RS485_MAX_PAYLOAD) {
        return 0u;
    }
    out[0] = RS485_SYNC0;
    out[1] = RS485_SYNC1;
    out[2] = addr;
    out[3] = type;
    out[4] = seq;
    out[5] = len;
    if (len > 0u && payload != NULL) {
        memmove(&out[RS485_HDR_BYTES], payload, len);
    }
    put16(&out[RS485_HDR_BYTES + len], rs485_crc16(&out[2], (size_t)(RS485_HDR_BYTES - 2u + len)));
    return (uint16_t)(RS485_HDR_BYTES + len + RS485_CRC_BYTES);
}

uint8_t rs485_encode_req_hdr(uint8_t *p, const rs485_req_hdr_t *h)
{
    put16(&p[0], h->nonce);
    put16(&p[2], h->ack_evt);
    return RS485_REQ_HDR_BYTES;
}

uint8_t rs485_encode_relay_req(uint8_t *p, const rs485_relay_req_t *r)
{
    (void)rs485_encode_req_hdr(p, &r->hdr);
    p[4] = r->relay_idx;
    p[5] = r->target;
    put16(&p[6], r->cmd_seq);
    return RS485_RELAY_REQ_BYTES;
}

uint8_t rs485_encode_info(uint8_t *p, const rs485_info_t *i)
{
    p[0] = i->proto_version;
    p[1] = i->addr;
    p[2] = i->relay_count;
    p[3] = i->board_type;
    put16(&p[4], i->fw_version);
    put16(&p[6], i->nonce);
    put32(&p[8], i->uptime_s);
    put32(&p[12], i->uid_hash);
    return RS485_INFO_BYTES;
}

uint8_t rs485_encode_status(uint8_t *p, const rs485_status_t *s)
{
    memset(p, 0, RS485_STATUS_BYTES);
    put16(&p[0], s->nonce);
    put32(&p[2], s->window_id);
    for (uint8_t k = 0u; k < RS485_RELAYS_PER_MODULE; k++) {
        put16(&p[6u + 2u * k], (uint16_t)s->relay_ma[k]);
    }
    put16(&p[30], s->rail_24v_mv);
    put16(&p[32], s->rail_5v_mv);
    put16(&p[34], s->relay_state);
    put16(&p[36], s->relay_known);
    put16(&p[38], s->relay_evidence);
    p[40] = s->fault_flags;
    p[41] = s->store_flags;
    put16(&p[42], s->dropped_events);
    p[44] = s->cmd_result;

    uint8_t n = s->event_count;
    if (n > RS485_EVENTS_MAX) { n = RS485_EVENTS_MAX; }
    p[45] = n;
    for (uint8_t e = 0u; e < n; e++) {
        uint8_t *q = &p[46u + RS485_EVENT_BYTES * e];
        put16(&q[0], s->events[e].id);
        q[2] = s->events[e].type;
        q[3] = s->events[e].relay_idx;
        put16(&q[4], s->events[e].cmd_seq);
        put32(&q[6], (uint32_t)s->events[e].measured_ma);
    }
    return (uint8_t)RS485_STATUS_BYTES;
}

bool rs485_decode_req_hdr(const uint8_t *p, uint8_t len, rs485_req_hdr_t *h)
{
    if (len < RS485_REQ_HDR_BYTES) { return false; }
    h->nonce   = get16(&p[0]);
    h->ack_evt = get16(&p[2]);
    return true;
}

bool rs485_decode_relay_req(const uint8_t *p, uint8_t len, rs485_relay_req_t *r)
{
    if (len < RS485_RELAY_REQ_BYTES) { return false; }
    (void)rs485_decode_req_hdr(p, len, &r->hdr);
    r->relay_idx = p[4];
    r->target    = p[5];
    r->cmd_seq   = get16(&p[6]);
    return true;
}

bool rs485_decode_info(const uint8_t *p, uint8_t len, rs485_info_t *i)
{
    if (len < RS485_INFO_BYTES) { return false; }
    i->proto_version = p[0];
    i->addr          = p[1];
    i->relay_count   = p[2];
    i->board_type    = p[3];
    i->fw_version    = get16(&p[4]);
    i->nonce         = get16(&p[6]);
    i->uptime_s      = get32(&p[8]);
    i->uid_hash      = get32(&p[12]);
    return (i->addr < RS485_MAX_MODULES) && (i->relay_count <= RS485_RELAYS_PER_MODULE);
}

bool rs485_decode_status(const uint8_t *p, uint8_t len, rs485_status_t *s)
{
    if (len < 46u) { return false; }
    s->nonce     = get16(&p[0]);
    s->window_id = get32(&p[2]);
    for (uint8_t k = 0u; k < RS485_RELAYS_PER_MODULE; k++) {
        s->relay_ma[k] = (int16_t)get16(&p[6u + 2u * k]);
    }
    s->rail_24v_mv    = get16(&p[30]);
    s->rail_5v_mv     = get16(&p[32]);
    s->relay_state    = get16(&p[34]);
    s->relay_known    = get16(&p[36]);
    s->relay_evidence = get16(&p[38]);
    s->fault_flags    = p[40];
    s->store_flags    = p[41];
    s->dropped_events = get16(&p[42]);
    s->cmd_result     = p[44];
    s->event_count    = p[45];

    if (s->event_count > RS485_EVENTS_MAX ||
        len < (uint8_t)(46u + RS485_EVENT_BYTES * s->event_count)) {
        s->event_count = 0u;
        return false;
    }
    for (uint8_t e = 0u; e < s->event_count; e++) {
        const uint8_t *q = &p[46u + RS485_EVENT_BYTES * e];
        s->events[e].id          = get16(&q[0]);
        s->events[e].type        = q[2];
        s->events[e].relay_idx   = q[3];
        s->events[e].cmd_seq     = get16(&q[4]);
        s->events[e].measured_ma = (int32_t)get32(&q[6]);
    }
    return true;
}
