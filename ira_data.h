/* SPDX-License-Identifier: GPL-3.0-or-later
 * IRA decoded fields shared by the decoder and standalone positioning tools. */
#ifndef IRIDIUM_IRA_DATA_H
#define IRIDIUM_IRA_DATA_H
#include <stdint.h>
typedef struct {
    int sat_id;
    int beam_id;
    double lat, lon;
    int alt;                /* km */
    int pos_xyz[3];         /* raw 12-bit signed XYZ (Doppler positioning) */
    int n_pages;
    struct {
        uint32_t tmsi;
        int msc_id;
    } pages[12];            /* max 12 paging blocks per IRA */
} ira_data_t;
#endif
