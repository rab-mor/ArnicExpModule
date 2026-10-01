/**
 * relays.c - F103 expansion module
 *
 * Non-blocking scheduler for twelve dual-coil latching relays.  The same
 * scheduler as the main board's relays.c; only the coil map differs.
 *
 * Design notes, and why this differs from the H7 implementation it
 * replaces:
 *
 *  - The old relay_pulse_start() indexed active_pulses[] by relay but
 *    stored the coil pin in that entry.  A second command for the same
 *    relay inside the 80 ms window overwrote the entry with the other
 *    coil's pin, leaving the first coil energised with nothing tracking
 *    it.  It never went low.  Here the pulse record holds the relay's
 *    own state machine and a coil selector, so a mid-pulse request is
 *    recorded as a target and acted on only after the coil drops.
 *
 *  - Concurrency is capped at RELAY_MAX_CONCURRENT.  The old code would
 *    energise all eight coils at once given an eight-bit update mask.
 *
 *  - Shadow state is tracked, so redundant commands do not re-pulse and
 *    the link can report real positions.
 *
 *  - Nothing is ever pulsed at boot.  A latching relay keeps its position
 *    through a reset or a power cut, so relays_init() assumes OFF without
 *    touching a coil, and RelayTask then restores the saved positions with
 *    relays_restore() (relay_store.c).  The only thing that moves a relay is
 *    a command.
 *
 *  - A command always pulses, even when the relay is believed to be there
 *    already.  That is the only way to be certain of a latching relay's
 *    position, and it costs one 80 ms coil pulse.
 *
 *  - "established" means the position is backed by something: a pulse that
 *    completed, the saved record, or current flowing through the contacts
 *    (relays_adopt()).  A position assumed at first boot is not established.
 *
 *  - Only one coil per relay can ever be energised, structurally: each
 *    relay owns a single coil selector, not two independent pins.
 *
 *  - RELAY_DRIVE_COILS (board_config.h) = 0 on the bench: every pulse is
 *    scheduled, timed and tracked exactly as on the real board, but the
 *    coil pins are never written.  coil_write() is the only place a coil
 *    is driven.
 */

#include "relays.h"
#include "board_config.h"

#ifndef RELAY_DRIVE_COILS
#error "Add RELAY_DRIVE_COILS to board_config.h (0 = bench, 1 = real relays)"
#endif

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} gpio_pin_t;

/* Coil selector values. */
#define COIL_A                  0U      /* connector pin 1, "Reset" */
#define COIL_B                  1U      /* connector pin 2, "Set"   */

/* ======================================================================
 * Coil GPIO map - expansion module
 *
 * From the wiring as given (MCU net -> relay coil).  A is the reset coil
 * (connector pin 1, OFF), B is the set coil (pin 2, ON):
 *
 *   relays 1-4 straight:   RLYnA -> nA, RLYnB -> nB
 *   RLY5B -> 8B   RLY5A -> 8A   RLY6B -> 7B   RLY6A -> 7A     (reversed)
 *   RLY7B -> 6A   RLY7A -> 6B   RLY8B -> 5A   RLY8A -> 5B     (reversed, A/B swapped)
 *   RLY9B -> 12B  RLY9A -> 12A  RLY10B -> 11B RLY10A -> 11A   (reversed)
 *   RLY11B -> 10B RLY11A -> 10A RLY12B -> 9B  RLY12A -> 9A    (reversed)
 *
 * The tables are indexed by PHYSICAL RELAY (K1..K12 => 0..11) and hold the
 * MCU pin that drives that relay's coil, with the net it came from.
 * ==================================================================== */

/* Coil A - connector pin 1 - "Reset" (OFF). */
static const gpio_pin_t k_coil_a[RELAY_COUNT] = {
    { RLY1_A_GPIO_Port,  RLY1_A_Pin  },  /* K1  <- RLY1_A  (PE7)           */
    { RLY2_A_GPIO_Port,  RLY2_A_Pin  },  /* K2  <- RLY2_A  (PE9)           */
    { RLY3_A_GPIO_Port,  RLY3_A_Pin  },  /* K3  <- RLY3_A  (PE11)          */
    { RLY4_A_GPIO_Port,  RLY4_A_Pin  },  /* K4  <- RLY4_A  (PE13)          */
    { RLY8_B_GPIO_Port,  RLY8_B_Pin  },  /* K5  <- RLY8_B  (PE15)  SWAP    */
    { RLY7_B_GPIO_Port,  RLY7_B_Pin  },  /* K6  <- RLY7_B  (PB11)  SWAP    */
    { RLY6_A_GPIO_Port,  RLY6_A_Pin  },  /* K7  <- RLY6_A  (PB12)          */
    { RLY5_A_GPIO_Port,  RLY5_A_Pin  },  /* K8  <- RLY5_A  (PB14)          */
    { RLY12_A_GPIO_Port, RLY12_A_Pin },  /* K9  <- RLY12_A (PD8)           */
    { RLY11_A_GPIO_Port, RLY11_A_Pin },  /* K10 <- RLY11_A (PD10)          */
    { RLY10_A_GPIO_Port, RLY10_A_Pin },  /* K11 <- RLY10_A (PD12)          */
    { RLY9_A_GPIO_Port,  RLY9_A_Pin  },  /* K12 <- RLY9_A  (PD14)          */
};

/* Coil B - connector pin 2 - "Set" (ON). */
static const gpio_pin_t k_coil_b[RELAY_COUNT] = {
    { RLY1_B_GPIO_Port,  RLY1_B_Pin  },  /* K1  <- RLY1_B  (PB2)           */
    { RLY2_B_GPIO_Port,  RLY2_B_Pin  },  /* K2  <- RLY2_B  (PE8)           */
    { RLY3_B_GPIO_Port,  RLY3_B_Pin  },  /* K3  <- RLY3_B  (PE10)          */
    { RLY4_B_GPIO_Port,  RLY4_B_Pin  },  /* K4  <- RLY4_B  (PE12)          */
    { RLY8_A_GPIO_Port,  RLY8_A_Pin  },  /* K5  <- RLY8_A  (PE14)  SWAP    */
    { RLY7_A_GPIO_Port,  RLY7_A_Pin  },  /* K6  <- RLY7_A  (PB10)  SWAP    */
    { RLY6_B_GPIO_Port,  RLY6_B_Pin  },  /* K7  <- RLY6_B  (PB13)          */
    { RLY5_B_GPIO_Port,  RLY5_B_Pin  },  /* K8  <- RLY5_B  (PB15)          */
    { RLY12_B_GPIO_Port, RLY12_B_Pin },  /* K9  <- RLY12_B (PD9)           */
    { RLY11_B_GPIO_Port, RLY11_B_Pin },  /* K10 <- RLY11_B (PD11)          */
    { RLY10_B_GPIO_Port, RLY10_B_Pin },  /* K11 <- RLY10_B (PD13)          */
    { RLY9_B_GPIO_Port,  RLY9_B_Pin  },  /* K12 <- RLY9_B  (PD15)          */
};

/*
 * PB3/PB4 are JTAG pins but carry D2/D1 (keypad inputs), not coils, and the
 * CubeMX NOJTAG remap frees them anyway.  PB2 (RLY1_B) is BOOT1: it is only
 * sampled at reset, and MX_GPIO_Init() drives it low.
 */
static volatile uint32_t s_activity = 0U;

/* ====================================================================== */

typedef struct {
    uint8_t  target;        /* requested state, 1 = ON                   */
    uint8_t  known;         /* position we believe the relay is in       */
    uint8_t  established;   /* known is backed by a pulse, record or current */
    uint8_t  force;         /* a command asked for a pulse               */
    uint8_t  pulsing;       /* a coil is energised right now             */
    uint8_t  coil;          /* which coil, COIL_A or COIL_B              */
    uint32_t t_start;       /* tick at which the coil went high          */
    uint32_t t_ready;       /* earliest tick a new pulse may start       */
} relay_ctx_t;

static relay_ctx_t s_ctx[RELAY_COUNT];

/* ---------------------------------------------------------------------- */

static const gpio_pin_t *coil_pin(uint8_t idx, uint8_t coil)
{
    return (coil == COIL_B) ? &k_coil_b[idx] : &k_coil_a[idx];
}

/**
 * Which coil drives the requested user-visible state.
 *
 * Normally ON is the Set coil (B) and OFF is the Reset coil (A).  For a
 * relay flagged in RELAY_REVERSED_MASK the roles swap.  This is the only
 * place the inversion lives - do not add a second one at the call site,
 * which is how the H7 code ended up with two cancelling errors.
 */
static uint8_t coil_for_state(uint8_t idx, uint8_t state)
{
    uint8_t on_coil = ((RELAY_REVERSED_MASK >> idx) & 1U) ? COIL_A : COIL_B;

    if (state != 0U) {
        return on_coil;
    }
    return (on_coil == COIL_B) ? COIL_A : COIL_B;
}

/** The position a relay latches into after a pulse on this coil. */
static uint8_t state_for_coil(uint8_t idx, uint8_t coil)
{
    return (coil == coil_for_state(idx, 1U)) ? 1U : 0U;
}

static void coil_write(uint8_t idx, uint8_t coil, GPIO_PinState level)
{
    const gpio_pin_t *p = coil_pin(idx, coil);
#if RELAY_DRIVE_COILS
    HAL_GPIO_WritePin(p->port, p->pin, level);
#else
    /* Bench build: the pulse is scheduled and timed, but no pin is driven. */
    (void)p;
    (void)level;
#endif
}

/* ---------------------------------------------------------------------- */

void relays_init(void)
{
    uint32_t now = HAL_GetTick();

    /* Every coil low before anything else.  MX_GPIO_Init() should already
     * have left them low, but a reset that does not power-cycle the board
     * can leave a coil energised from before. */
    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        coil_write(i, COIL_A, GPIO_PIN_RESET);
        coil_write(i, COIL_B, GPIO_PIN_RESET);

        /* Assume OFF, but do not pulse: target == known means the scheduler
         * leaves it alone.  relays_restore() replaces this with the saved
         * positions. */
        s_ctx[i].target      = 0U;
        s_ctx[i].known       = 0U;
        s_ctx[i].established = 0U;
        s_ctx[i].force       = 0U;
        s_ctx[i].pulsing     = 0U;
        s_ctx[i].coil        = COIL_A;
        s_ctx[i].t_start     = now;
        s_ctx[i].t_ready     = now;
    }
}

void relays_restore(uint32_t state_mask, uint32_t established_mask)
{
    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        if (s_ctx[i].pulsing != 0U || s_ctx[i].force != 0U) {
            continue;       /* a command got in first: it wins */
        }
        const uint8_t on = ((state_mask >> i) & 1UL) ? 1U : 0U;
        s_ctx[i].known       = on;
        s_ctx[i].target      = on;
        s_ctx[i].established = ((established_mask >> i) & 1UL) ? 1U : 0U;
    }
}

uint8_t relays_adopt(uint8_t idx, uint8_t state)
{
    if (idx >= RELAY_COUNT || state > 1U) {
        return 0U;
    }
    relay_ctx_t *c = &s_ctx[idx];
    if (c->pulsing != 0U || c->force != 0U || c->known != c->target) {
        return 0U;          /* moving: the pulse will settle it */
    }
    if (c->known == state && c->established != 0U) {
        return 0U;          /* nothing new */
    }
    c->known       = state;
    c->target      = state;
    c->established = 1U;
    s_activity++;
    return 1U;
}

void relays_tick(void)
{
    uint32_t now    = HAL_GetTick();
    uint8_t  active = 0U;

    /* Phase 1: drop coils whose pulse has expired. */
    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        if (s_ctx[i].pulsing == 0U) {
            continue;
        }

        if ((uint32_t)(now - s_ctx[i].t_start) >= RELAY_PULSE_MS) {
            coil_write(i, s_ctx[i].coil, GPIO_PIN_RESET);
            s_activity++;
            s_ctx[i].pulsing     = 0U;
            /* The relay latched where THIS coil sends it.  Not the target:
             * a request that arrived mid-pulse may have changed that, and
             * it still needs a pulse of its own. */
            s_ctx[i].known       = state_for_coil(i, s_ctx[i].coil);
            s_ctx[i].established = 1U;
            s_ctx[i].t_ready     = now + RELAY_INTERPULSE_MS;
        }
    }

    /* Phase 2: count what is still energised. */
    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        if (s_ctx[i].pulsing != 0U) {
            active++;
        }
    }

    /* Phase 3: start pulses for relays that need one, up to the cap. */
    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        if (active >= RELAY_MAX_CONCURRENT) {
            break;
        }
        if (s_ctx[i].pulsing != 0U) {
            continue;
        }
        /* Where it should be, and no command asked for a pulse. */
        if (s_ctx[i].force == 0U && s_ctx[i].known == s_ctx[i].target) {
            continue;
        }
        /* Signed compare so the tick wrap at 49.7 days is handled. */
        if ((int32_t)(now - s_ctx[i].t_ready) < 0) {
            continue;
        }

        s_ctx[i].coil    = coil_for_state(i, s_ctx[i].target);
        s_ctx[i].t_start = now;
        s_ctx[i].pulsing = 1U;
        s_ctx[i].force   = 0U;
        coil_write(i, s_ctx[i].coil, GPIO_PIN_SET);
        s_activity++;
        active++;
    }
}



uint32_t relays_activity(void) {
	return s_activity;
}


/* ---------------------------------------------------------------------- */

int relays_request(uint8_t idx, uint8_t state)
{
    if (idx >= RELAY_COUNT) {
        return RELAYS_ERR_INDEX;
    }
    if (state > 1U) {
        return RELAYS_ERR_STATE;
    }

    /* Record only.  If a pulse is in flight it finishes first; the
     * scheduler picks this up on the pass after that.  A pulse already
     * heading for this state satisfies the command; anything else gets a
     * pulse of its own, even if we believe the relay is there already. */
    relay_ctx_t *c = &s_ctx[idx];
    c->target = state;
    c->force  = (c->pulsing != 0U && state_for_coil(idx, c->coil) == state) ? 0U : 1U;
    return RELAYS_OK;
}

uint32_t relays_apply_mask(uint32_t update_mask, uint32_t state_mask)
{
    uint32_t errors = 0U;

    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        if ((update_mask & (1UL << i)) == 0UL) {
            continue;
        }
        uint8_t state = ((state_mask & (1UL << i)) != 0UL) ? 1U : 0U;
        if (relays_request(i, state) != RELAYS_OK) {
            errors++;
        }
    }
    return errors;
}


uint8_t relays_get_state(uint8_t idx)
{
    return (idx < RELAY_COUNT) ? s_ctx[idx].known : 0U;
}


uint8_t relays_state_known(uint8_t idx)
{
    return (idx < RELAY_COUNT) ? s_ctx[idx].established : 0U;
}


uint8_t relays_settled(uint8_t idx)
{
    if (idx >= RELAY_COUNT) {
        return 0U;
    }
    const relay_ctx_t *c = &s_ctx[idx];
    return (c->pulsing == 0U && c->force == 0U && c->known == c->target) ? 1U : 0U;
}


uint32_t relays_get_state_mask(void)
{
    uint32_t m = 0U;
    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        if (s_ctx[i].known != 0U) {
            m |= (1UL << i);
        }
    }
    return m;
}


uint32_t relays_known_mask(void)
{
    uint32_t m = 0U;
    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        if (s_ctx[i].established != 0U) {
            m |= (1UL << i);
        }
    }
    return m;
}

uint8_t relays_busy(void)
{
    for (uint8_t i = 0U; i < RELAY_COUNT; i++) {
        if (s_ctx[i].pulsing != 0U) {
            return 1U;
        }
        if (s_ctx[i].force != 0U || s_ctx[i].known != s_ctx[i].target) {
            return 1U;
        }
    }
    return 0U;
}
