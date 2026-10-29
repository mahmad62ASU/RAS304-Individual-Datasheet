/*
 * Team 103 / RAS 304 — temperature supervisor reference, Rev 01
 *
 * PROPOSED DESIGN EXAMPLE — not PIC production firmware or a safety release.
 * The platform must provide ADC acquisition, timing, GPIO, watchdog behavior,
 * debounced manual rearm edges, and the team's actual fault-stop protocol.
 * No hardware testing, ADC calibration, or trip validation is claimed.
 *
 * Circuit: +5V -> 10k fixed resistor -> ADC node -> 10k NTC -> GND.
 * MCP6002 unity buffer -> 1k/100nF -> ADC. Three channels in this order:
 * pitch driver, roll driver, local 7805 regulator.
 *
 * FAULT_CMD HIGH drives a P2N2222A open collector, pulling BUS 5 FAULT_N LOW.
 * FALSE releases the collector; a team-specified external pull-up is required.
 * This circuit does not guarantee a fault on MCU or module power loss.
 *
 * Example only: 50 C warning / fan on, 60 C latched trip,
 * manual rearm only when ALL valid probes are <=45 C.
 * Confirm limits against the actual hardware before use.
 */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define THERMAL_CHANNELS  3u
#define ADC_COUNTS_MAX    4095u /* Example 12-bit ADC; confirm setup. */
#define ADC_RAIL_GUARD    64u   /* Example open/short window; calibrate. */
#define NTC_R25_OHM       10000.0f
#define DIVIDER_R_OHM     10000.0f
#define NTC_BETA_K        3977.0f
#define NTC_T25_K         298.15f
#define WARN_C            50.0f
#define TRIP_C            60.0f
#define REARM_C           45.0f

typedef enum {
    PROBE_PITCH = 0,
    PROBE_ROLL = 1,
    PROBE_REGULATOR = 2
} thermal_channel_t;

typedef struct {
    float temperature_c[THERMAL_CHANNELS];
    bool probe_valid[THERMAL_CHANNELS];
    bool fault_latched;
    bool warning;
    bool fan_run;
    bool fault_cmd_high; /* Drive MCU output HIGH to assert shared fault. */
} thermal_state_t;

/* This beta model is an approximation. Account for tolerances, mounting,
 * ADC acquisition, reference choice, buffer headroom, and calibration. */
static bool ntc_from_adc(uint16_t raw, float *temperature_c)
{
    if (raw <= ADC_RAIL_GUARD ||
        raw >= (ADC_COUNTS_MAX - ADC_RAIL_GUARD)) {
        *temperature_c = NAN;
        return false;
    }

    const float resistance = DIVIDER_R_OHM * (float)raw /
                             (float)(ADC_COUNTS_MAX - raw);
    const float inverse_kelvin = (1.0f / NTC_T25_K) +
                                logf(resistance / NTC_R25_OHM) / NTC_BETA_K;
    const float value_c = (1.0f / inverse_kelvin) - 273.15f;
    if (!isfinite(value_c)) {
        *temperature_c = NAN;
        return false;
    }
    *temperature_c = value_c;
    return true;
}

void thermal_init(thermal_state_t *state)
{
    for (unsigned i = 0; i < THERMAL_CHANNELS; ++i) {
        state->temperature_c[i] = NAN;
        state->probe_valid[i] = false;
    }
    /* Conservative startup policy once firmware is running: a valid cool
     * scan and deliberate manual rearm are required before release. */
    state->fault_latched = true;
    state->warning = true;
    state->fan_run = true;
    state->fault_cmd_high = true;
}

/* Call after each complete ADC scan at a fixed rate selected during validation.
 * rearm_edge must be a debounced rising-edge event, not a held button level.
 * The platform must also detect stale ADC scans and reset/watchdog failures.
 */
void thermal_update(thermal_state_t *state,
                    const uint16_t raw[THERMAL_CHANNELS],
                    bool rearm_edge)
{
    bool all_valid = true;
    bool all_cool = true;
    bool any_hot = false;
    bool any_trip = false;

    for (unsigned i = 0; i < THERMAL_CHANNELS; ++i) {
        state->probe_valid[i] = ntc_from_adc(raw[i], &state->temperature_c[i]);
        if (!state->probe_valid[i]) {
            all_valid = false;
            all_cool = false;
            continue;
        }
        const float t = state->temperature_c[i];
        all_cool = all_cool && (t <= REARM_C);
        any_hot = any_hot || (t >= WARN_C);
        any_trip = any_trip || (t >= TRIP_C);
    }

    /* Invalid probes and overtemperature always win over rearm requests. */
    if (!all_valid || any_trip) {
        state->fault_latched = true;
    } else if (rearm_edge && all_cool) {
        state->fault_latched = false;
    }

    /* Optional fan: 50/45 C hysteresis. A latched fault requests cooling.
     * Fan presence/rotation and its effectiveness are not monitored here. */
    if (state->fault_latched || any_hot) {
        state->fan_run = true;
    } else if (all_cool) {
        state->fan_run = false;
    }
    state->warning = any_hot || !all_valid || state->fault_latched;
    state->fault_cmd_high = state->fault_latched;
}

/* Platform integration outline:
 *
 * thermal_state_t thermal;
 * thermal_init(&thermal);
 * configure_fault_output_high();  // establish intended reset/startup behavior
 * for (;;) {
 *     uint16_t raw[3];
 *     acquire_complete_adc_scan(raw);  // actual PIC pins/timing remain TBD
 *     thermal_update(&thermal, raw, take_debounced_rearm_edge());
 *     write_fault_cmd(thermal.fault_cmd_high);
 *     write_optional_fan(thermal.fan_run);
 *     report_temperatures_and_status(&thermal); // team protocol remains TBD
 * }
 *
 * Required validation: divider voltage at known temperatures; sensor unplug/
 * short tests; ADC settling and filter response; MCU reset and loss-of-power
 * behavior; hysteresis and manual rearm; end-to-end bus trip response.
 */
