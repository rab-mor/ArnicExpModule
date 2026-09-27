/**
 * win_ring.c - F103 expansion module
 *
 * See win_ring.h. MeasureTask writes, LinkTask reads; both under a critical
 * section, which is a few hundred cycles at most.
 */

#include "win_ring.h"
#include "FreeRTOS.h"
#include "task.h"

typedef struct {
    uint32_t id;
    int16_t  ma[RELAY_COUNT];
} win_t;

static win_t   s_ring[WIN_RING_LEN];
static uint8_t s_head;                  /* next slot to write */
static uint8_t s_count;

void win_push(uint32_t window_id, const int32_t *relay_ma)
{
    win_t w;
    w.id = window_id;
    for (uint32_t k = 0u; k < RELAY_COUNT; k++) {
        int32_t ma = relay_ma[k];
        if (ma >  32767) { ma =  32767; }
        if (ma < -32768) { ma = -32768; }
        w.ma[k] = (int16_t)ma;
    }

    taskENTER_CRITICAL();
    s_ring[s_head] = w;
    s_head = (uint8_t)((s_head + 1u) % WIN_RING_LEN);
    if (s_count < WIN_RING_LEN) {
        s_count++;
    }
    taskEXIT_CRITICAL();
}

uint8_t win_collect(uint32_t win_ack, uint8_t max, uint32_t *first_id,
                    int16_t (*out)[RELAY_COUNT])
{
    uint8_t n = 0u;

    taskENTER_CRITICAL();
    if (s_count > 0u && max > 0u) {
        const uint32_t newest = s_ring[(s_head + WIN_RING_LEN - 1u) % WIN_RING_LEN].id;
        const uint32_t oldest = newest - (s_count - 1u);

        uint32_t start;
        if (win_ack == 0u || (int32_t)(win_ack - newest) > 0) {
            start = newest;                         /* fresh master, or we rebooted */
        } else if ((int32_t)(win_ack - oldest) < 0) {
            start = oldest;                         /* older ones are gone: the H7 counts the gap */
        } else {
            start = win_ack + 1u;                   /* may be newest + 1: nothing new */
        }

        const uint32_t avail = ((int32_t)(newest - start) >= 0) ? (newest - start + 1u) : 0u;
        n = (uint8_t)((avail < max) ? avail : max);

        uint32_t idx = (s_head + WIN_RING_LEN - 1u - (newest - start)) % WIN_RING_LEN;
        for (uint8_t i = 0u; i < n; i++) {
            for (uint32_t k = 0u; k < RELAY_COUNT; k++) {
                out[i][k] = s_ring[idx].ma[k];
            }
            idx = (idx + 1u) % WIN_RING_LEN;
        }
        *first_id = start;
    }
    taskEXIT_CRITICAL();
    return n;
}
