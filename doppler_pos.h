/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef __DOPPLER_POS_H__
#define __DOPPLER_POS_H__
#include <stdint.h>
#include "ira_data.h"

/* This engine accepts IRA (ring alert) observations only. */
#define DOPPLER_IRA_HZ (1626000000.0 + 6.5 * (1000000.0 / 24.0))
typedef enum {
    DOPPLER_COLLECTING, DOPPLER_VALID, DOPPLER_STALE,
    DOPPLER_GEOMETRY, DOPPLER_REJECTED
} doppler_status_t;
typedef struct {
    double lat, lon, alt;
    /* Deprecated compatibility field. Always NAN; Doppler geometry is not
     * dimensionless GNSS HDOP. Use conditional horizontal_sigma_m instead. */
    double hdop;
    double residual_rms_mps, horizontal_sigma_m, clock_drift_mps;
    uint64_t epoch_ns;
    int n_measurements, n_satellites, n_rejected, converged;
    doppler_status_t status;
} doppler_solution_t;

/* Reset buffers, previous solution, and height aiding. Thread safe. */
void doppler_pos_init(void);
void doppler_pos_set_height(double height_m); /* WGS84 ellipsoidal, NAN disables */
void doppler_pos_add_measurement(const ira_data_t *, double frequency, uint64_t timestamp);
/* Precision-aware form used by replay/live adapters. 0 means rejected input.
 * sigma_hz covers carrier estimation only; orbit uncertainty is added inside. */
int doppler_pos_add_observation(const ira_data_t *, double frequency,
                               uint64_t timestamp, double sigma_hz);
/* Live: ages latest observation using a monotonic clock, not UTC assumptions. */
int doppler_pos_solve(doppler_solution_t *out);
/* Replay: now_ns must use the observation time scale. No wall-clock dependency. */
int doppler_pos_solve_at(uint64_t now_ns, doppler_solution_t *out);
const char *doppler_status_name(doppler_status_t status);
#endif
