#ifndef APP_SHARED_H
#define APP_SHARED_H
/**
 * app_shared.h - F103 expansion module header file
 *
 * What the tasks share: the relay command queue, relay events, the
 * measurement snapshot and the fault / store flags. Same shape as the main
 * board's app_shared.h, widened to 12 relays.
 */

#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os2.h"
#include "board_config.h"

/* LinkTask (rs485_slave.c) thread flags */
#define LINK_FLAG_RX            (1u << 0)   /* UART receive event (half, full, idle) */
#define LINK_FLAG_TX_DONE       (1u << 1)   /* last stop bit left, UART_EN dropped   */
#define LINK_FLAG_UART_ERROR    (1u << 2)

/* MeasureTask thread flag */
#define MEAS_FLAG_WINDOW_DONE   (1u << 0)

/* fault_flags: the same bit meanings as the main board's F103 */
#define FAULT_UNCALIBRATED  (1u << 0)
#define FAULT_ADC_START     (1u << 1)
#define FAULT_ADC_STALL     (1u << 2)
#define FAULT_ADC_OVERRUN   (1u << 3)
#define FAULT_EEPROM        (1u << 4)

#define STORE_F_PRESENT     (1u << 0)
#define STORE_F_LOADED      (1u << 1)
#define STORE_F_SAVE_FAILED (1u << 2)
#define STORE_F_PENDING     (1u << 3)

/* Event types: the same numbers as the main board and rs485_proto.h */
typedef enum {
    LINK_EVENT_ACCEPTED = 0u,
    LINK_EVENT_VERIFIED = 1u,
    LINK_EVENT_REJECTED = 2u,
    LINK_EVENT_FAULT    = 3u,
} link_event_type_t;

extern osThreadId_t LinkTaskHandle;         /* freertos.c: defaultTask runs LinkTask */
extern osThreadId_t MeasureTaskHandle;
extern osThreadId_t RelayTaskHandle;
extern osThreadId_t SupervisorTaskHandle;
extern osMessageQueueId_t relay_cmd_qHandle;

/* ---------- relay_cmd_q: LinkTask -> RelayTask ---------- */
typedef struct {
    uint8_t  relay_idx;     /* 0..RELAY_COUNT-1 */
    uint8_t  target;        /* 1 = on, 0 = off  */
    uint16_t seq;           /* the H7's cmd_seq */
} relay_cmd_t;
_Static_assert(sizeof(relay_cmd_t) == 4, "relay_cmd_t size changed");

/* ---------- events: RelayTask -> LinkTask (rs485_slave.c keeps them) ---------- */
typedef struct {
    uint8_t  type;          /* link_event_type_t */
    uint8_t  relay_idx;
    uint16_t seq;
    int32_t  measured_ma;
} link_event_t;
_Static_assert(sizeof(link_event_t) == 8, "link_event_t size changed");

/* Post an event for the H7. Never blocks; a full buffer drops the oldest. */
void link_post_event(const link_event_t *evt);

/* ---------- Snapshot: MeasureTask -> LinkTask, double-buffered ---------- */
typedef struct {
    uint32_t window_id;
    int32_t  relay_ma[RELAY_COUNT];
    uint16_t rail_24V_mv;
    uint16_t rail_5V_mv;
    uint16_t relay_state;       /* bit n: relay n on                      */
    uint16_t relay_known;       /* bit n: position established            */
    uint16_t relay_evidence;    /* bit n: corrected by current evidence   */
    uint8_t  fault_flags;
    uint8_t  store_flags;
} app_snapshot_t;

extern app_snapshot_t    g_snap[2];
extern volatile uint8_t  g_snap_active;

/* ---------- Relay truth: RelayTask -> MeasureTask ---------- */
typedef struct {
    volatile uint16_t evidence_mask;
    volatile uint8_t  store_flags;
    uint8_t           _pad;
} relay_truth_t;

extern relay_truth_t g_relay_truth;

#endif /* APP_SHARED_H */
