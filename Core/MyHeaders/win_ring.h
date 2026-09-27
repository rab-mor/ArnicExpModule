#ifndef WIN_RING_H
#define WIN_RING_H
/**
 * win_ring.h - F103 expansion module header file
 *
 * The last WIN_RING_LEN measurement windows (2 s of 50 ms windows), kept
 * until the H7 has them. MeasureTask pushes one per window; LinkTask copies
 * out the ones after the H7's win_ack into each status reply, so a poll that
 * comes late loses nothing. Window ids are consecutive (MeasureTask numbers
 * every window it publishes, faulted ones included).
 */

#include <stdint.h>
#include "board_config.h"

#define WIN_RING_LEN    40u

/* MeasureTask, once per published window. */
void win_push(uint32_t window_id, const int32_t *relay_ma);

/* LinkTask. The windows after win_ack, oldest first, at most max of them:
   copied to out, the first one's id in *first_id. With win_ack 0, or a
   win_ack newer than anything here (this module rebooted), only the newest.
   Returns how many. */
uint8_t win_collect(uint32_t win_ack, uint8_t max, uint32_t *first_id,
                    int16_t (*out)[RELAY_COUNT]);

#endif /* WIN_RING_H */
