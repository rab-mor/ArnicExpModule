#ifndef RS485_PROTO_H
#define RS485_PROTO_H
/**
 * @file  rs485_proto.h
 * @brief Wire format of the RS485 expansion bus (H7 UART7 <-> expansion
 *        modules). Shared verbatim by the H7 project (Common/) and the
 *        expansion module project. No HAL, no RTOS, no project includes.
 *
 * Bus rules (the transceivers' anti-jabber circuit and /RE tied low):
 *   - 115200 8N1, half duplex, one master (the H7). A module only transmits
 *     to answer a request addressed to it, so at most one driver is ever on.
 *   - Every node hears its own transmission. Requests and replies are told
 *     apart by RS485_REPLY_FLAG in `type`, so each side simply drops frames
 *     going its own way: no byte counting.
 *   - A frame is at most RS485_MAX_FRAME (248) bytes, 21.5 ms on the wire:
 *     the DE hardware cuts a driver off after ~80 ms of continuous transmit.
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
 * Every request payload starts with rs485_req_hdr_t. It acknowledges the
 * module's events: the module keeps each event until the master acks it, so
 * a reply lost to noise loses nothing. `nonce` is the module's boot nonce as
 * the master last saw it; after a module reboot it no longer matches and the
 * module ignores the ack, so events from the new boot are never discarded.
 *
 * Replies:
 *   PING   -> rs485_info_t      (discovery: who is at this address)
 *   STATUS -> rs485_status_t    (measurements, relay positions, events)
 *   RELAY  -> rs485_status_t    with cmd_result set for the command
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define RS485_SYNC0             0xA5u
#define RS485_SYNC1             0x3Cu
#define RS485_PROTO_VERSION     1u
#define RS485_BAUD              115200u
#define RS485_HDR_BYTES         6u
#define RS485_CRC_BYTES         2u
#define RS485_MAX_PAYLOAD       240u
#define RS485_MAX_FRAME         (RS485_HDR_BYTES + RS485_MAX_PAYLOAD + RS485_CRC_BYTES)

#define RS485_MAX_MODULES       16u     /* DIP addresses 0..15               */
#define RS485_RELAYS_PER_MODULE 12u
#define RS485_EVENTS_MAX        4u      /* events carried by one status reply */

#define RS485_REPLY_FLAG        0x80u

typedef enum {
    RS485_MSG_PING   = 0x01u,
    RS485_MSG_STATUS = 0x02u,
    RS485_MSG_RELAY  = 0x03u,
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
    uint16_t nonce;             /* module boot nonce the ack refers to  */
    uint16_t ack_evt;           /* highest event id the master consumed */
} rs485_req_hdr_t;

typedef struct {
    rs485_req_hdr_t hdr;
    uint8_t  relay_idx;         /* 0..RS485_RELAYS_PER_MODULE-1         */
    uint8_t  target;            /* 1 = ON, 0 = OFF                      */
    uint16_t cmd_seq;           /* echoed in the command's events        */
} rs485_relay_req_t;

#define RS485_REQ_HDR_BYTES     4u
#define RS485_RELAY_REQ_BYTES   8u

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

typedef struct {
    uint16_t nonce;
    uint32_t window_id;         /* +1 per 50 ms measurement window      */
    int16_t  relay_ma[RS485_RELAYS_PER_MODULE];
    uint16_t rail_24v_mv;
    uint16_t rail_5v_mv;
    uint16_t relay_state;       /* bit n = relay n on                   */
    uint16_t relay_known;       /* bit n = position established         */
    uint16_t relay_evidence;    /* bit n = corrected by current         */
    uint8_t  fault_flags;       /* module FAULT_*                       */
    uint8_t  store_flags;       /* STORE_F_*                            */
    uint16_t dropped_events;    /* events lost to a full queue          */
    uint8_t  cmd_result;        /* RS485_CMD_*                          */
    uint8_t  event_count;       /* 0..RS485_EVENTS_MAX                  */
    rs485_event_t events[RS485_EVENTS_MAX];
} rs485_status_t;

#define RS485_EVENT_BYTES       10u
#define RS485_STATUS_BYTES      (46u + RS485_EVENTS_MAX * RS485_EVENT_BYTES)   /* 86 */

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
uint8_t rs485_encode_info(uint8_t *p, const rs485_info_t *i);
uint8_t rs485_encode_status(uint8_t *p, const rs485_status_t *s);

/* Decoders return false if len is too short or a field is out of range. */
bool rs485_decode_req_hdr(const uint8_t *p, uint8_t len, rs485_req_hdr_t *h);
bool rs485_decode_relay_req(const uint8_t *p, uint8_t len, rs485_relay_req_t *r);
bool rs485_decode_info(const uint8_t *p, uint8_t len, rs485_info_t *i);
bool rs485_decode_status(const uint8_t *p, uint8_t len, rs485_status_t *s);

#endif /* RS485_PROTO_H */
