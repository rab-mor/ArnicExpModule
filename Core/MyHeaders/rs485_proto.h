#ifndef RS485_PROTO_H
#define RS485_PROTO_H
/**
 * @file  rs485_proto.h
 * @brief Wire format of the RS485 expansion bus (H7 UART7 <-> expansion
 *        modules). Shared verbatim by the H7 project (Common/) and the
 *        expansion module project. No HAL, no RTOS, no project includes.
 *
 * Bus rules (the transceivers' anti-jabber circuit and /RE tied low):
 *   - RS485_BAUD 8N1, half duplex, one master (the H7). A module only
 *     transmits to answer a request addressed to it, so at most one driver is
 *     ever on. 187500 baud: exact on the F103 (72 MHz / 16 / 24), 0.06 % off
 *     on the H7 (100 MHz / 533), inside the SN65HVD3082E's 200 kbps rating.
 *   - Every node hears its own transmission. Requests and replies are told
 *     apart by RS485_REPLY_FLAG in `type`, so each side simply drops frames
 *     going its own way: no byte counting.
 *   - A frame is at most RS485_MAX_FRAME (248) bytes, 13 ms on the wire: the
 *     DE hardware cuts a driver off after ~80 ms of continuous transmit.
 *
 * Frame:
 *   off  size  field
 *    0    1    sync0   0xA5
 *    1    1    sync1   0x3C
 *    2    1    addr    module address 0..15 (its DIP switches)
 *    3    1    type    RS485_MSG_*; | RS485_REPLY_FLAG on replies
 *    4    1    seq     chosen by the master, echoed in the reply
 *    5    1    len     payload bytes, 0..RS485_MAX_PAYLOAD
 *    6    len  payload (little-endian fields)
 *   6+len 2    crc16   CCITT-FALSE over bytes 2 .. 5+len, little-endian
 *
 * Every request payload starts with rs485_req_hdr_t (10 bytes):
 *   0  u16 nonce      the module's boot nonce as the master last saw it
 *   2  u16 ack_evt    highest event id the master has consumed
 *   4  u32 win_ack    highest measurement window the master has stored
 *   8  u16 f0         line frequency from the ADE9000, 0.01 Hz (0 = unknown)
 * Events and windows stay in the module until the master acknowledges them,
 * so a reply lost to noise loses nothing. After a module reboot the nonce no
 * longer matches and the module ignores the acks, so nothing from the new
 * boot is discarded.
 *
 * Messages and replies:
 *   PING      hdr                 -> rs485_info_t
 *   STATUS    hdr                 -> rs485_status_t
 *   RELAY     hdr + relay command -> rs485_status_t, cmd_result set
 *   HARMONICS hdr + part          -> rs485_harm_part_t
 *   OTA       hdr + rs485_ota_req_t -> rs485_ota_status_t
 *
 * Version 2 (this file): 187500 baud; win_ack and f0 in the request header;
 * the status carries up to RS485_WIN_MAX unacknowledged 50 ms windows instead
 * of only the latest one; HARMONICS and OTA added. Not compatible with v1:
 * the H7 and every module are flashed together.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define RS485_SYNC0             0xA5u
#define RS485_SYNC1             0x3Cu
#define RS485_PROTO_VERSION     2u
#define RS485_BAUD              187500u
#define RS485_HDR_BYTES         6u
#define RS485_CRC_BYTES         2u
#define RS485_MAX_PAYLOAD       240u
#define RS485_MAX_FRAME         (RS485_HDR_BYTES + RS485_MAX_PAYLOAD + RS485_CRC_BYTES)

#define RS485_MAX_MODULES       16u     /* DIP addresses 0..15               */
#define RS485_RELAYS_PER_MODULE 12u
#define RS485_EVENTS_MAX        4u      /* events carried by one status reply */
#define RS485_WIN_MAX           8u      /* windows carried by one status reply */

#define RS485_REPLY_FLAG        0x80u

typedef enum {
    RS485_MSG_PING      = 0x01u,
    RS485_MSG_STATUS    = 0x02u,
    RS485_MSG_RELAY     = 0x03u,
    RS485_MSG_HARMONICS = 0x04u,
    RS485_MSG_OTA       = 0x05u,
} rs485_msg_t;

/* rs485_status_t.cmd_result */
#define RS485_CMD_NONE          0xFFu   /* the request was not a RELAY command */
#define RS485_CMD_ACCEPTED      0u      /* queued; VERIFIED / FAULT follow     */
#define RS485_CMD_REJECTED      1u      /* bad relay index or target           */
#define RS485_CMD_BUSY          2u      /* command queue full, try again       */
#define RS485_CMD_DUPLICATE     3u      /* same cmd_seq as the last one: already queued */

/* Event types: the same numbers as the F103 link (link_event_type_t). */
#define RS485_EVT_ACCEPTED      0u
#define RS485_EVT_VERIFIED      1u
#define RS485_EVT_REJECTED      2u
#define RS485_EVT_FAULT         3u

/* ------------------------------ Requests -------------------------------- */

typedef struct {
    uint16_t nonce;             /* module boot nonce the acks refer to  */
    uint16_t ack_evt;           /* highest event id the master consumed */
    uint32_t win_ack;           /* highest window_id the master stored  */
    uint16_t f0_centi_hz;       /* 0.01 Hz, 0 = unknown                 */
} rs485_req_hdr_t;

typedef struct {
    rs485_req_hdr_t hdr;
    uint8_t  relay_idx;         /* 0..RS485_RELAYS_PER_MODULE-1         */
    uint8_t  target;            /* 1 = ON, 0 = OFF                      */
    uint16_t cmd_seq;           /* echoed in the command's events        */
} rs485_relay_req_t;

typedef struct {
    rs485_req_hdr_t hdr;
    uint8_t  part;              /* 0 = records 0..7 of the set, 1 = 8.. */
} rs485_harm_req_t;

/* Firmware update. The ops and results are f1_ota.h's (the same numbers as
   the main board's SPI link: LINK_OTA_* / F1OTA_*). */
#define RS485_OTA_QUERY         0u
#define RS485_OTA_BEGIN         1u      /* offset = image size; data = u32 crc32, u32 fw_version, u8 board */
#define RS485_OTA_DATA          2u
#define RS485_OTA_END           3u
#define RS485_OTA_ABORT         4u
#define RS485_OTA_DATA_MAX      192u

typedef struct {
    rs485_req_hdr_t hdr;
    uint8_t  op;
    uint8_t  op_seq;
    uint8_t  len;               /* data bytes                           */
    uint32_t offset;
    const uint8_t *data;        /* decode: points into the frame        */
} rs485_ota_req_t;

#define RS485_REQ_HDR_BYTES     10u
#define RS485_RELAY_REQ_BYTES   14u
#define RS485_HARM_REQ_BYTES    11u
#define RS485_OTA_REQ_HDR_BYTES 18u     /* then len data bytes */

_Static_assert(RS485_OTA_REQ_HDR_BYTES + RS485_OTA_DATA_MAX <= RS485_MAX_PAYLOAD, "OTA request must fit a frame");

/* ------------------------------- Replies -------------------------------- */

typedef struct {
    uint8_t  proto_version;     /* RS485_PROTO_VERSION                  */
    uint8_t  addr;              /* DIP address the module read          */
    uint8_t  relay_count;       /* 12                                   */
    uint8_t  board_type;        /* 1 = expansion module                 */
    uint16_t fw_version;        /* major << 8 | minor                   */
    uint16_t nonce;             /* changes on every boot                */
    uint32_t uptime_s;
    uint32_t uid_hash;          /* from the MCU unique ID: two modules on one
                                   address show up as a changing uid_hash */
} rs485_info_t;

#define RS485_INFO_BYTES        16u

typedef struct {
    uint16_t id;                /* 1, 2, 3... per boot                  */
    uint8_t  type;              /* RS485_EVT_*                          */
    uint8_t  relay_idx;
    uint16_t cmd_seq;
    int32_t  measured_ma;
} rs485_event_t;

/*
 * Status reply:
 *   off  size
 *    0    2   nonce
 *    2    4   window_id       newest window measured
 *    6    2   rail_24v_mv
 *    8    2   rail_5v_mv
 *   10    2   relay_state     bit n = relay n on
 *   12    2   relay_known     bit n = position established
 *   14    2   relay_evidence  bit n = corrected by current
 *   16    1   fault_flags     module FAULT_*
 *   17    1   store_flags     STORE_F_*
 *   18    2   dropped_events  events lost to a full buffer
 *   20    1   cmd_result      RS485_CMD_*
 *   21    1   harm_set_id     newest harmonics set (0 = none yet)
 *   22    1   event_count     0..RS485_EVENTS_MAX
 *   23    1   win_count       0..RS485_WIN_MAX
 *   24    4   win_first_id    window_id of the first window below
 *   28   10 x event_count     rs485_event_t
 *    .   24 x win_count       relay_ma[12] i16 each, consecutive window ids
 *
 * Windows: every 50 ms window after the request's win_ack, oldest first, as
 * many as fit. A module keeps RS485_WIN_RING of them, so a module polled late
 * still loses none. With win_ack 0 (a master that just started) or a win_ack
 * the module never produced (the module just rebooted), only the newest.
 */
typedef struct {
    uint16_t nonce;
    uint32_t window_id;
    uint16_t rail_24v_mv;
    uint16_t rail_5v_mv;
    uint16_t relay_state;
    uint16_t relay_known;
    uint16_t relay_evidence;
    uint8_t  fault_flags;
    uint8_t  store_flags;
    uint16_t dropped_events;
    uint8_t  cmd_result;
    uint8_t  harm_set_id;
    uint8_t  event_count;
    uint8_t  win_count;
    uint32_t win_first_id;
    rs485_event_t events[RS485_EVENTS_MAX];
    int16_t  win_ma[RS485_WIN_MAX][RS485_RELAYS_PER_MODULE];
} rs485_status_t;

#define RS485_STATUS_HDR_BYTES  28u
#define RS485_EVENT_BYTES       10u
#define RS485_WINDOW_BYTES      (2u * RS485_RELAYS_PER_MODULE)   /* 24 */

/* Windows that fit next to `events` events. */
static inline uint8_t rs485_win_room(uint8_t events)
{
    const uint32_t room = (RS485_MAX_PAYLOAD - RS485_STATUS_HDR_BYTES - RS485_EVENT_BYTES * events) / RS485_WINDOW_BYTES;
    return (uint8_t)((room < RS485_WIN_MAX) ? room : RS485_WIN_MAX);
}

/*
 * Harmonics reply: the relays of the newest set whose fundamental is at
 * least HARM_MIN_H1_MA (harmonics.h); an idle relay costs nothing.
 *   off  size
 *    0    1   set_id
 *    1    1   part            as requested
 *    2    1   total           relays in the set with current
 *    3    1   count           records in this reply
 *    4    2   f0_centi_hz     frequency the detectors sat on
 *    6    2   -               0
 *    8   27 x count           u8 relay, then the 26-byte record of link_proto.h
 *                             (h1_ma, ratio[8], phase[8])
 * part 0 carries records 0..RS485_HARM_PER_PART-1 of the set, part 1 the rest.
 */
#define RS485_HARM_PER_PART     8u
#define RS485_HARM_REC_BYTES    27u
#define RS485_HARM_HDR_BYTES    8u
#define RS485_HARM_RATIOS       8u

typedef struct {
    uint8_t  relay;
    uint16_t h1_ma;
    uint16_t ratio[RS485_HARM_RATIOS];
    int8_t   phase[RS485_HARM_RATIOS];
} rs485_harm_rec_t;

typedef struct {
    uint8_t  set_id;
    uint8_t  part;
    uint8_t  total;
    uint8_t  count;
    uint16_t f0_centi_hz;
    rs485_harm_rec_t rec[RS485_HARM_PER_PART];
} rs485_harm_part_t;

_Static_assert(RS485_HARM_HDR_BYTES + RS485_HARM_PER_PART * RS485_HARM_REC_BYTES <= RS485_MAX_PAYLOAD,
               "a harmonics part must fit a frame");

/* OTA reply: 0 op_seq, 1 result (F1OTA_*), 2 state (F1OTA_ST_*), 3 board_type,
   4 next_offset u32, 8 fw_version u16 (running firmware). */
typedef struct {
    uint8_t  op_seq;
    uint8_t  result;
    uint8_t  state;
    uint8_t  board_type;
    uint32_t next_offset;
    uint16_t fw_version;
} rs485_ota_status_t;

#define RS485_OTA_STATUS_BYTES  10u

/* --------------------------- Stream parser ------------------------------ */

typedef struct {
    uint8_t  buf[RS485_MAX_FRAME];
    uint16_t have;              /* bytes collected so far               */
    uint16_t need;              /* total frame bytes once len is known  */
    uint32_t crc_errors;
    uint32_t sync_drops;        /* bytes skipped looking for sync       */
} rs485_parser_t;

typedef struct {
    uint8_t        addr;
    uint8_t        type;        /* RS485_REPLY_FLAG included            */
    uint8_t        seq;
    uint8_t        len;
    const uint8_t *payload;     /* points into the parser's buffer      */
} rs485_frame_t;

void rs485_parser_reset(rs485_parser_t *p);

/* Feed one byte. Returns true when it completes a frame with a good CRC;
   *out then describes it until the next call. */
bool rs485_parser_feed(rs485_parser_t *p, uint8_t byte, rs485_frame_t *out);

/* ------------------------------ Encoding -------------------------------- */

uint16_t rs485_crc16(const uint8_t *data, size_t len);

/* Build a frame into out (at least RS485_MAX_FRAME bytes). Returns its
   length, or 0 if len is too large. */
uint16_t rs485_build(uint8_t *out, uint8_t addr, uint8_t type, uint8_t seq,
                     const uint8_t *payload, uint8_t len);

uint8_t rs485_encode_req_hdr(uint8_t *p, const rs485_req_hdr_t *h);
uint8_t rs485_encode_relay_req(uint8_t *p, const rs485_relay_req_t *r);
uint8_t rs485_encode_harm_req(uint8_t *p, const rs485_harm_req_t *r);
uint8_t rs485_encode_ota_req(uint8_t *p, const rs485_ota_req_t *r);   /* copies r->data */
uint8_t rs485_encode_info(uint8_t *p, const rs485_info_t *i);
uint8_t rs485_encode_status(uint8_t *p, const rs485_status_t *s);
uint8_t rs485_encode_harm_part(uint8_t *p, const rs485_harm_part_t *h);
uint8_t rs485_encode_ota_status(uint8_t *p, const rs485_ota_status_t *s);

/* Decoders return false if len is too short or a field is out of range. */
bool rs485_decode_req_hdr(const uint8_t *p, uint8_t len, rs485_req_hdr_t *h);
bool rs485_decode_relay_req(const uint8_t *p, uint8_t len, rs485_relay_req_t *r);
bool rs485_decode_harm_req(const uint8_t *p, uint8_t len, rs485_harm_req_t *r);
bool rs485_decode_ota_req(const uint8_t *p, uint8_t len, rs485_ota_req_t *r);
bool rs485_decode_info(const uint8_t *p, uint8_t len, rs485_info_t *i);
bool rs485_decode_status(const uint8_t *p, uint8_t len, rs485_status_t *s);
bool rs485_decode_harm_part(const uint8_t *p, uint8_t len, rs485_harm_part_t *h);
bool rs485_decode_ota_status(const uint8_t *p, uint8_t len, rs485_ota_status_t *s);

#endif /* RS485_PROTO_H */
