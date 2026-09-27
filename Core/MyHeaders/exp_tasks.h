/**
 * exp_tasks.h - F103 expansion module header file
 *
 * Task entry points and live stats. Tasks are created in freertos.c
 * (USER CODE BEGIN RTOS_THREADS). CubeMX's defaultTask only exits: its
 * 512-byte stack is too small for LinkTask, which has its own now.
 *
 * Ownership:
 *   LinkTask       - USART1 / RS485: answers the H7, queues relay commands,
 *                    keeps events until the H7 acknowledges them
 *   RelayTask      - relays_request() / relays_tick(); the EEPROM record;
 *                    VERIFIED / FAULT events; current evidence
 *   MeasureTask    - fills g_snap[] once per 50 ms measurement window
 *   SupervisorTask - kicks the MAX706 (WDOG, PE3) while the others run
 */

#ifndef EXP_TASKS_H
#define EXP_TASKS_H

#include <stdint.h>

void Rs485Slave_Run(void *argument);
void RelayTask_Run(void *argument);
void MeasureTask_Run(void *argument);
void SupervisorTask_Run(void *argument);

extern volatile uint32_t g_hb_relay;
extern volatile uint32_t g_hb_measure;
extern volatile uint32_t g_hb_link;

typedef struct {
    uint32_t cmds;
    uint32_t verified;
    uint32_t faults;
    uint32_t superseded;
    uint32_t rejected;
    uint32_t stuck_on;
    uint32_t evidence_on;
    uint32_t saves;
    uint32_t save_errors;
    uint32_t store_result;   /* boot: 0 no EEPROM, 1 no record, 2 restored */
    uint32_t restored_state;
    uint32_t restored_known;
    uint32_t state_mask;
    uint32_t known_mask;
} relay_task_dbg_t;

extern relay_task_dbg_t g_relay_dbg;

typedef struct {
    int32_t  start_rc;
    uint32_t windows;
    uint32_t calibrated;
    uint32_t overruns;
    uint32_t stalls;
    int32_t  relay_ma[12];
    uint32_t rail_24v_mv;
    uint32_t rail_5v_mv;
} measure_dbg_t;

extern measure_dbg_t g_measure_dbg;

typedef struct {
    uint32_t kicks;
    uint32_t withheld;
    uint32_t stalled_mask;   /* bit 0 RelayTask, 1 MeasureTask, 2 LinkTask; sticky */
    uint32_t has_pin;
} supervisor_dbg_t;

extern supervisor_dbg_t g_sup_dbg;

typedef struct {
    uint8_t  addr;           /* DIP address read at boot                    */
    uint16_t nonce;          /* this boot's nonce                           */
    uint32_t requests;       /* frames addressed to us                      */
    uint32_t replies;
    uint32_t other_addr;     /* frames for other modules (normal)           */
    uint32_t crc_errors;
    uint32_t uart_errors;
    uint32_t tx_timeouts;
    uint32_t cmds_queued;
    uint32_t cmds_duplicate;
    uint32_t cmds_busy;
    uint32_t events_dropped; /* event buffer full: oldest dropped           */
    uint32_t last_request_ms;
    uint32_t windows_sent;   /* measurement windows carried in replies      */
    uint32_t ota_requests;
    uint32_t baud_fixed;     /* 1 = usart.c had another baud rate           */
} link_dbg_t;

extern link_dbg_t g_link_dbg;

#endif /* EXP_TASKS_H */
