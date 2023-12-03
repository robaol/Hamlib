/*
 *  Hamlib ICOM M710 backend - main header
 *  Copyright (c) 2014-2015 by Stephane Fillod
 *
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef _ICM710ITU_H
#define _ICM710ITU_H 1

#include "hamlib/rig.h"
#include "cal.h"
#include "tones.h"

#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#endif

#include "icmarine.h"

#if 0
struct icm710itu_priv_caps {
    unsigned char default_remote_id;  /* the remote default equipment's ID */
};
#endif

/* The M710 does support queries. See block comment in icm710itu.c */
/* But we do need a mechanism to allow TUNING to happen and not
 * timeout, causing an error. So, we have flags and data to control and
 * remember tuned frequency and a tuning timeout that we temporarily
 * install when about to tune.
 */
struct icm710itu_priv_data {
    unsigned char remote_id;  /* the remote equipment's ID */
    split_t split; /* current split mode */
    char *mode_str[NUM_MODE_STR]; /* number of modes defined */
    int flagTuneOnNewTxfreq; /* Control whether to issue TUNER:ON commands */
    freq_t lastTunedTxfreq; /* Allow detection of new txfreq */
    int tuneTimeout; /* in ms. Tuning takes a long time, so need a longer timeout */

};

extern const struct confparams icm710itu_cfg_params[];

int icm710itu_init(RIG *rig);
int icm710itu_get_freq(RIG *rig, vfo_t vfo, freq_t *freq);
int icm710itu_set_freq(RIG *rig, vfo_t vfo, freq_t freq);
int icm710itu_set_tx_freq(RIG *rig, vfo_t vfo, freq_t freq);
int icm710itu_get_split_vfo(RIG *rig, vfo_t rx_vfo, split_t *split, vfo_t *tx_vfo);
int icm710itu_set_split_vfo(RIG *rig, vfo_t rx_vfo, split_t split, vfo_t tx_vfo);
int icm710itu_vfo_op(RIG *rig, vfo_t vfo, vfo_op_t op);
int icm710itu_set_func(RIG *rig, vfo_t vfo, setting_t func, int status);

#endif /* _ICM710ITU_H */
