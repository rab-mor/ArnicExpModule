/**
 * board_config.h - F103 expansion module header file
 *
 * Board constants for the RS485 expansion module: STM32F103VCT6, LQFP100,
 * 72 MHz from an 8 MHz crystal, 12 latching relays, 12 current sensors.
 *
 * Only values fixed by the hardware or the CubeMX configuration live here.
 * The relay coil tables are in relays.c, next to the code that drives them.
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include <stdint.h>

/* ======================================================================
 * Identity
 * ==================================================================== */

#define BOARD_TYPE_EXPANSION    1U
#define F1_THIS_BOARD           BOARD_TYPE_EXPANSION    /* F1_BOARD_EXPANSION (f1_image.h) */
#define FW_VERSION_MAJOR        1U
#define FW_VERSION_MINOR        0U

/* ======================================================================
 * Clock tree (CubeMX): HSE 8 MHz -> PLL x9 -> 72 MHz, APB1 36 MHz,
 * APB2 72 MHz, ADC 72 / 6 = 12 MHz.  TIM3 on APB1 runs at 72 MHz (the APB1
 * prescaler is not 1, so its timer clock is doubled).
 * ==================================================================== */

#define BOARD_APB1_TIMER_HZ     72000000UL
#define BOARD_ADCCLK_HZ         12000000UL

_Static_assert(BOARD_ADCCLK_HZ <= 14000000UL, "F103 ADC clock must not exceed 14 MHz");

/* ======================================================================
 * ADC sampling: TIM3 TRGO at 8 kHz triggers one dual regular-simultaneous
 * scan of 7 ranks.  6 x (55.5 + 12.5) + (239.5 + 12.5) = 660 ADC clocks
 * = 55 us per trigger, inside the 125 us period.
 * ==================================================================== */

#define ADC_SAMPLE_RATE_HZ      8000UL
#define TIM3_PRESCALER          71U
#define TIM3_PERIOD             124U

_Static_assert((BOARD_APB1_TIMER_HZ / (TIM3_PRESCALER + 1UL))
                   / (TIM3_PERIOD + 1UL) == ADC_SAMPLE_RATE_HZ, "TIM3 PSC/ARR do not yield ADC_SAMPLE_RATE_HZ");

#define ADC_RANKS_PER_SCAN      7U
#define ADC_TRIGGERS_PER_HALF   100U
#define ADC_HALF_WORDS          (ADC_RANKS_PER_SCAN * ADC_TRIGGERS_PER_HALF)
#define ADC_DMA_WORDS           (ADC_HALF_WORDS * 2U)   /* 1400 words = 5.6 KB */

/*
 * Rank layout, as configured in CubeMX (ADC1 = low half of each DMA word,
 * ADC2 = high half).  Values are current-sensor numbers from the net labels
 * ADCx_CurrentSensorN, 1-based:
 *
 *   rank | ADC1 (low)          | ADC2 (high)
 *   -----+---------------------+---------------------
 *     1  | IN2  PA2  CS4       | IN0  PA0  CS6
 *     2  | IN3  PA3  CS3       | IN1  PA1  CS5
 *     3  | IN4  PA4  CS2       | IN8  PB0  CS7
 *     4  | IN5  PA5  CS1       | IN9  PB1  CS8
 *     5  | IN6  PA6  CS9       | IN14 PC4  CS11
 *     6  | IN7  PA7  CS10      | IN15 PC5  CS12
 *     7  | IN12 PC2  5 V rail  | IN13 PC3  24 V rail
 *
 * Note rank 7 is the other way round from the main board: 5 V is on ADC1.
 */
#define ADC_RANK_CS_LO          { 4U, 3U, 2U, 1U,  9U, 10U }
#define ADC_RANK_CS_HI          { 6U, 5U, 7U, 8U, 11U, 12U }
#define ADC_CURRENT_RANKS       6U
#define ADC_RAIL_RANK           6U      /* 0-based: rank 7 */

/*
 * Relay k (0-based) is measured by current sensor CS(k+1).
 * ASSUMPTION, to confirm on the schematic: sensor N sits on relay N, as on
 * the main board.  If not, change only this table.
 */
#define RELAY_CURRENT_SENSOR_MAP \
    { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U }

/* ======================================================================
 * RMS window: 400 samples at 8 kHz = 50 ms = 3 cycles at 60 Hz, 2.5 at
 * 50 Hz; a 50 ms boxcar has nulls at every 20 Hz, so the 2f ripple of the
 * squared signal cancels at both mains frequencies.
 * ==================================================================== */

#define RMS_WINDOW_SAMPLES      400U

_Static_assert(RMS_WINDOW_SAMPLES % ADC_TRIGGERS_PER_HALF == 0U, "RMS window must be a whole number of DMA half-buffers");

/* ======================================================================
 * Scaling: 12-bit ADC on 3.3 V; ACS725LLCTR-20AB-T sensors (+/-20 A,
 * 66 mV/A, ratiometric), assumed the same parts as the main board.
 * ==================================================================== */

#define ADC_VREF_MV             3300.0f
#define ADC_FULL_SCALE          4096.0f
#define ADC_MV_PER_LSB          (ADC_VREF_MV / ADC_FULL_SCALE)

#define SENSE_LSB_PER_AMP       81.92f
#define SENSE_AMPS_PER_LSB      (1.0f / SENSE_LSB_PER_AMP)
#define SENSE_ZERO_COUNTS       2048
#define SENSE_NOISE_FLOOR_A     0.05f
#define SENSE_GATE_MAX_A        0.25f

/* Rail dividers (top + bottom) / bottom, same parts as the main board. */
#define RAIL_24V_DIVIDER        11.0f
#define RAIL_5V_DIVIDER         5.8214f

/* ======================================================================
 * Relays (ADJH23012 latching, via ULN2803s).  Timing as the main board.
 * ==================================================================== */

#define RELAY_COUNT             12U
#define RELAY_PULSE_MS          80U
#define RELAY_INTERPULSE_MS     20U
#define RELAY_MAX_CONCURRENT    2U
#define RELAY_REVERSED_MASK     0x0000U
#define RELAY_DRIVE_COILS       1       /* 0 = pulses timed and tracked, pins never driven */

/* Current evidence and the relay record: see the main board. */
#define EVIDENCE_ON_MA          300
#define EVIDENCE_WINDOWS        10U
#define RSTORE_SETTLE_MS        250U
#define RSTORE_RETRY_MS         5000U

/* ======================================================================
 * External watchdog: MAX706S on WDOG (PE3), WDO wired to MR.
 * Timeout 1.0 s minimum.
 * ==================================================================== */

#define WDOG_KICK_MS            50U
#define WDOG_STALL_MS           1000U

/* ======================================================================
 * RS485 (USART1, PA9 TX / PA10 RX, UART_EN PA8 drives the transceiver's
 * DE through the anti-jabber network).
 * ==================================================================== */

#define RS485_IDLE_MS           100U    /* LinkTask wakes at least this often  */
#define RS485_TX_TIMEOUT_MS     40U     /* 248 bytes take 21.5 ms              */

#endif /* BOARD_CONFIG_H */
