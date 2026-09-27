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
    put32(&p[4], h->win_ack);
    put16(&p[8], h->f0_centi_hz);
    return RS485_REQ_HDR_BYTES;
}

uint8_t rs485_encode_relay_req(uint8_t *p, const rs485_relay_req_t *r)
{
    (void)rs485_encode_req_hdr(p, &r->hdr);
    p[10] = r->relay_idx;
    p[11] = r->target;
    put16(&p[12], r->cmd_seq);
    return RS485_RELAY_REQ_BYTES;
}

uint8_t rs485_encode_harm_req(uint8_t *p, const rs485_harm_req_t *r)
{
    (void)rs485_encode_req_hdr(p, &r->hdr);
    p[10] = r->part;
    return RS485_HARM_REQ_BYTES;
}

uint8_t rs485_encode_ota_req(uint8_t *p, const rs485_ota_req_t *r)
{
    uint8_t n = r->len;
    if (n > RS485_OTA_DATA_MAX) { n = RS485_OTA_DATA_MAX; }
    (void)rs485_encode_req_hdr(p, &r->hdr);
    p[10] = r->op;
    p[11] = r->op_seq;
    p[12] = n;
    p[13] = 0u;
    put32(&p[14], r->offset);
    if (n > 0u && r->data != NULL) {
        memcpy(&p[RS485_OTA_REQ_HDR_BYTES], r->data, n);
    }
    return (uint8_t)(RS485_OTA_REQ_HDR_BYTES + n);
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
    uint8_t ne = s->event_count;
    if (ne > RS485_EVENTS_MAX) { ne = RS485_EVENTS_MAX; }
    uint8_t nw = s->win_count;
    if (nw > rs485_win_room(ne)) { nw = rs485_win_room(ne); }

    put16(&p[0], s->nonce);
    put32(&p[2], s->window_id);
    put16(&p[6], s->rail_24v_mv);
    put16(&p[8], s->rail_5v_mv);
    put16(&p[10], s->relay_state);
    put16(&p[12], s->relay_known);
    put16(&p[14], s->relay_evidence);
    p[16] = s->fault_flags;
    p[17] = s->store_flags;
    put16(&p[18], s->dropped_events);
    p[20] = s->cmd_result;
    p[21] = s->harm_set_id;
    p[22] = ne;
    p[23] = nw;
    put32(&p[24], s->win_first_id);

    uint8_t *q = &p[RS485_STATUS_HDR_BYTES];
    for (uint8_t e = 0u; e < ne; e++, q += RS485_EVENT_BYTES) {
        put16(&q[0], s->events[e].id);
        q[2] = s->events[e].type;
        q[3] = s->events[e].relay_idx;
        put16(&q[4], s->events[e].cmd_seq);
        put32(&q[6], (uint32_t)s->events[e].measured_ma);
    }
    for (uint8_t w = 0u; w < nw; w++) {
        for (uint8_t k = 0u; k < RS485_RELAYS_PER_MODULE; k++, q += 2) {
            put16(q, (uint16_t)s->win_ma[w][k]);
        }
    }
    return (uint8_t)(q - p);
}

static void put_harm_rec(uint8_t *q, const rs485_harm_rec_t *r)
{
    q[0] = r->relay;
    put16(&q[1], r->h1_ma);
    for (uint8_t k = 0u; k < RS485_HARM_RATIOS; k++) {
        put16(&q[3u + 2u * k], r->ratio[k]);
        q[19u + k] = (uint8_t)r->phase[k];
    }
}

uint8_t rs485_encode_harm_part(uint8_t *p, const rs485_harm_part_t *h)
{
    uint8_t n = h->count;
    if (n > RS485_HARM_PER_PART) { n = RS485_HARM_PER_PART; }
    p[0] = h->set_id;
    p[1] = h->part;
    p[2] = h->total;
    p[3] = n;
    put16(&p[4], h->f0_centi_hz);
    p[6] = 0u;
    p[7] = 0u;
    for (uint8_t i = 0u; i < n; i++) {
        put_harm_rec(&p[RS485_HARM_HDR_BYTES + RS485_HARM_REC_BYTES * i], &h->rec[i]);
    }
    return (uint8_t)(RS485_HARM_HDR_BYTES + RS485_HARM_REC_BYTES * n);
}

uint8_t rs485_encode_ota_status(uint8_t *p, const rs485_ota_status_t *s)
{
    p[0] = s->op_seq;
    p[1] = s->result;
    p[2] = s->state;
    p[3] = s->board_type;
    put32(&p[4], s->next_offset);
    put16(&p[8], s->fw_version);
    return RS485_OTA_STATUS_BYTES;
}

/* ------------------------------ Decoding -------------------------------- */

bool rs485_decode_req_hdr(const uint8_t *p, uint8_t len, rs485_req_hdr_t *h)
{
    if (len < RS485_REQ_HDR_BYTES) { return false; }
    h->nonce       = get16(&p[0]);
    h->ack_evt     = get16(&p[2]);
    h->win_ack     = get32(&p[4]);
    h->f0_centi_hz = get16(&p[8]);
    return true;
}

bool rs485_decode_relay_req(const uint8_t *p, uint8_t len, rs485_relay_req_t *r)
{
    if (len < RS485_RELAY_REQ_BYTES) { return false; }
    (void)rs485_decode_req_hdr(p, len, &r->hdr);
    r->relay_idx = p[10];
    r->target    = p[11];
    r->cmd_seq   = get16(&p[12]);
    return true;
}

bool rs485_decode_harm_req(const uint8_t *p, uint8_t len, rs485_harm_req_t *r)
{
    if (len < RS485_HARM_REQ_BYTES) { return false; }
    (void)rs485_decode_req_hdr(p, len, &r->hdr);
    r->part = p[10];
    return true;
}

bool rs485_decode_ota_req(const uint8_t *p, uint8_t len, rs485_ota_req_t *r)
{
    if (len < RS485_OTA_REQ_HDR_BYTES) { return false; }
    (void)rs485_decode_req_hdr(p, len, &r->hdr);
    r->op     = p[10];
    r->op_seq = p[11];
    r->len    = p[12];
    r->offset = get32(&p[14]);
    r->data   = &p[RS485_OTA_REQ_HDR_BYTES];
    return (r->len <= RS485_OTA_DATA_MAX) && (len >= (uint8_t)(RS485_OTA_REQ_HDR_BYTES + r->len));
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
    if (len < RS485_STATUS_HDR_BYTES) { return false; }
    s->nonce          = get16(&p[0]);
    s->window_id      = get32(&p[2]);
    s->rail_24v_mv    = get16(&p[6]);
    s->rail_5v_mv     = get16(&p[8]);
    s->relay_state    = get16(&p[10]);
    s->relay_known    = get16(&p[12]);
    s->relay_evidence = get16(&p[14]);
    s->fault_flags    = p[16];
    s->store_flags    = p[17];
    s->dropped_events = get16(&p[18]);
    s->cmd_result     = p[20];
    s->harm_set_id    = p[21];
    s->event_count    = p[22];
    s->win_count      = p[23];
    s->win_first_id   = get32(&p[24]);

    if (s->event_count > RS485_EVENTS_MAX || s->win_count > rs485_win_room(s->event_count) ||
        len < (uint32_t)RS485_STATUS_HDR_BYTES + RS485_EVENT_BYTES * s->event_count +
              RS485_WINDOW_BYTES * s->win_count) {
        s->event_count = 0u;
        s->win_count   = 0u;
        return false;
    }
    const uint8_t *q = &p[RS485_STATUS_HDR_BYTES];
    for (uint8_t e = 0u; e < s->event_count; e++, q += RS485_EVENT_BYTES) {
        s->events[e].id          = get16(&q[0]);
        s->events[e].type        = q[2];
        s->events[e].relay_idx   = q[3];
        s->events[e].cmd_seq     = get16(&q[4]);
        s->events[e].measured_ma = (int32_t)get32(&q[6]);
    }
    for (uint8_t w = 0u; w < s->win_count; w++) {
        for (uint8_t k = 0u; k < RS485_RELAYS_PER_MODULE; k++, q += 2) {
            s->win_ma[w][k] = (int16_t)get16(q);
        }
    }
    return true;
}

bool rs485_decode_harm_part(const uint8_t *p, uint8_t len, rs485_harm_part_t *h)
{
    if (len < RS485_HARM_HDR_BYTES) { return false; }
    h->set_id      = p[0];
    h->part        = p[1];
    h->total       = p[2];
    h->count       = p[3];
    h->f0_centi_hz = get16(&p[4]);
    if (h->count > RS485_HARM_PER_PART || h->total > RS485_RELAYS_PER_MODULE ||
        len < (uint32_t)RS485_HARM_HDR_BYTES + RS485_HARM_REC_BYTES * h->count) {
        h->count = 0u;
        return false;
    }
    for (uint8_t i = 0u; i < h->count; i++) {
        const uint8_t *q = &p[RS485_HARM_HDR_BYTES + RS485_HARM_REC_BYTES * i];
        rs485_harm_rec_t *r = &h->rec[i];
        r->relay = q[0];
        r->h1_ma = get16(&q[1]);
        for (uint8_t k = 0u; k < RS485_HARM_RATIOS; k++) {
            r->ratio[k] = get16(&q[3u + 2u * k]);
            r->phase[k] = (int8_t)q[19u + k];
        }
        if (r->relay >= RS485_RELAYS_PER_MODULE) {
            h->count = 0u;
            return false;
        }
    }
    return true;
}

bool rs485_decode_ota_status(const uint8_t *p, uint8_t len, rs485_ota_status_t *s)
{
    if (len < RS485_OTA_STATUS_BYTES) { return false; }
    s->op_seq      = p[0];
    s->result      = p[1];
    s->state       = p[2];
    s->board_type  = p[3];
    s->next_offset = get32(&p[4]);
    s->fw_version  = get16(&p[8]);
    return true;
}
