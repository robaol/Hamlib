/*
 *  Hamlib ICOM Marine backend - description of IC-M710 caps
 *  Copyright (c) 2015 by Stephane Fillod
 *  Copyright (c) 2017 by Michael Black W9MDB
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include <hamlib/rig.h>
#include <serial.h>
#include <misc.h>
#include <cal.h>
#include <token.h>
#include <register.h>

#include "idx_builtin.h"
#include "bandplan.h"

#include "icm710itu.h"

#define ICM710_MODES (RIG_MODE_SSB | RIG_MODE_CW | RIG_MODE_RTTY)
#define ICM710_RX_MODES (ICM710_MODES | RIG_MODE_AM)

#define ICM710_FUNC_ALL (RIG_FUNC_NB)

#define ICM710_LEVEL_ALL (RIG_LEVEL_RFPOWER | RIG_LEVEL_AF | RIG_LEVEL_RF | RIG_LEVEL_AGC | RIG_LEVEL_RAWSTR)

#define ICM710_VFO_ALL (RIG_VFO_A)

#define ICM710_VFO_OPS (RIG_OP_TUNE)

#define ICM710_SCAN_OPS (RIG_SCAN_NONE)

/*
 * TODO calibrate the real values
 */
#define ICM710_STR_CAL          \
    {                           \
        2,                      \
        {                       \
            {0, -60}, { 8, 60 } \
        }                       \
    }

static const struct icm710itu_priv_caps icm710itu_priv_caps =
    {
        .default_remote_id = 0x01, /* default address */
};

const struct rig_caps icm710_caps =
    {
        RIG_MODEL(RIG_MODEL_IC_M710ITU),
        .model_name = "IC-M710ITU",
        .mfg_name = "Icom",
        .version = BACKEND_VER ".0",
        .copyright = "LGPL",
        .status = RIG_STATUS_ALPHA,
        .rig_type = RIG_TYPE_TRANSCEIVER,
        .ptt_type = RIG_PTT_RIG,
        .dcd_type = RIG_DCD_RIG,
        .port_type = RIG_PORT_SERIAL,
        .serial_rate_min = 4800,
        .serial_rate_max = 4800,
        .serial_data_bits = 8,
        .serial_stop_bits = 1,
        .serial_parity = RIG_PARITY_NONE,
        .serial_handshake = RIG_HANDSHAKE_NONE,
        .write_delay = 0,
        .post_write_delay = 0,
        .timeout = 100,
        .retry = 0,
        .has_get_func = ICM710_FUNC_ALL,
        .has_set_func = ICM710_FUNC_ALL,
        .has_get_level = ICM710_LEVEL_ALL,
        .has_set_level = RIG_LEVEL_SET(ICM710_LEVEL_ALL),
        .has_get_parm = RIG_PARM_NONE,
        .has_set_parm = RIG_PARM_NONE,
        .level_gran = {
            // cppcheck-suppress *
            [LVL_RAWSTR] = {.min = {.i = 0}, .max = {.i = 8}},
        },
        .parm_gran = {},
        .str_cal = ICM710_STR_CAL,
        .ctcss_list = NULL,
        .dcs_list = NULL,
        .preamp = {
            RIG_DBLST_END,
        },
        .attenuator = {
            RIG_DBLST_END,
        },
        .max_rit = Hz(0),
        .max_xit = Hz(0),
        .max_ifshift = Hz(0),
        .targetable_vfo = 0,
        .vfo_ops = ICM710_VFO_OPS,
        //.scan_ops =  ICM710_SCAN_OPS,
        .transceive = RIG_TRN_OFF,
        .bank_qty = 0,
        .chan_desc_sz = 0,

        .chan_list = {
            RIG_CHAN_END,
        },

        .rx_range_list1 = {
            {kHz(500), MHz(30) - 100, ICM710_RX_MODES, -1, -1, ICM710_VFO_ALL},
            RIG_FRNG_END,
        },
        .tx_range_list1 = {
            {kHz(1600), MHz(3) - 100, ICM710_MODES, W(60), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(4), MHz(5) - 100, ICM710_MODES, W(60), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(6), MHz(7) - 100, ICM710_MODES, W(60), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(8), MHz(9) - 100, ICM710_MODES, W(60), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(12), MHz(14) - 100, ICM710_MODES, W(60), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(16), MHz(18) - 100, ICM710_MODES, W(60), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(18), MHz(20) - 100, ICM710_MODES, W(60), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(22), MHz(23) - 100, ICM710_MODES, W(60), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(25), MHz(27.500), ICM710_MODES, W(60), W(60), ICM710_VFO_ALL, RIG_ANT_1},
            RIG_FRNG_END,
        },

        .rx_range_list2 = {
            {kHz(500), MHz(30) - 100, ICM710_RX_MODES, -1, -1, ICM710_VFO_ALL},
            RIG_FRNG_END,
        },
        .tx_range_list2 = {
            {kHz(1600), MHz(3) - 100, ICM710_MODES, W(20), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(4), MHz(5) - 100, ICM710_MODES, W(20), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(6), MHz(7) - 100, ICM710_MODES, W(20), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(8), MHz(9) - 100, ICM710_MODES, W(20), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(12), MHz(14) - 100, ICM710_MODES, W(20), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(16), MHz(18) - 100, ICM710_MODES, W(20), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(18), MHz(20) - 100, ICM710_MODES, W(20), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(22), MHz(23) - 100, ICM710_MODES, W(20), W(150), ICM710_VFO_ALL, RIG_ANT_1},
            {MHz(25), MHz(27.500), ICM710_MODES, W(20), W(60), ICM710_VFO_ALL, RIG_ANT_1},
            RIG_FRNG_END,
        },

        .tuning_steps = {
            {ICM710_RX_MODES, Hz(1)},
            RIG_TS_END,
        },
        /* mode/filter list, remember: order matters! */
        .filters = {
            {RIG_MODE_SSB | RIG_MODE_CW | RIG_MODE_RTTY, kHz(2.3)},
            {RIG_MODE_AM, kHz(14)},
            RIG_FLT_END,
        },

        .cfgparams = icm710_cfg_params,
        .set_conf = icmarine_set_conf,
        .get_conf = icmarine_get_conf,

        .priv = (void *)&icm710itu_priv_caps,
        .rig_init = icm710itu_init,
        .rig_cleanup = icmarine_cleanup,
        .rig_open = NULL,
        .rig_close = NULL,

        .set_freq = icm710itu_set_freq,
        .get_freq = icmarine_get_freq,
        .set_split_freq = icm710itu_set_tx_freq,
        .get_split_freq = icmarine_get_tx_freq,
        .set_split_vfo = icm710itu_set_split_vfo,
        .get_split_vfo = icmarine_get_split_vfo,
        .set_mode = icmarine_set_mode,
        .get_mode = icmarine_get_mode,

        .set_ptt = icmarine_set_ptt,
        .get_ptt = icmarine_get_ptt,
        .vfo_op = icm710itu_vfo_op,

        .set_level = icmarine_set_level,
        .get_level = icmarine_get_level,
        .set_func = icmarine_set_func,
        .get_func = icmarine_get_func,
        .set_parm = icmarine_set_parm,
        .get_parm = icmarine_get_parm,

};

/*
 * NMEA 0183 protocol is all handled by icmarine_transaction, defined in icmarine.c
 *
 */

#define BUFSZ 96

/* Tokens */
#define TOK_REMOTEID TOKEN_BACKEND(1)

const struct confparams icm710_cfg_params[] =
    {
        {TOK_REMOTEID, "remoteid", "Remote ID", "Transceiver's remote ID", "1", RIG_CONF_NUMERIC, {.n = {1, 99, 1}}},
        {
            RIG_CONF_END,
            NULL,
        }};

/*
 * Basically, set up *priv
 */
int icm710itu_init(RIG *rig)
{
    struct icm710itu_priv_data *priv;
    const struct icm710itu_priv_caps *priv_caps;
    const struct rig_caps *caps;

    if (!rig || !rig->caps)
    {
        return -RIG_EINVAL;
    }

    caps = rig->caps;

    if (!caps->priv)
    {
        return -RIG_ECONF;
    }

    priv_caps = (const struct icm710itu_priv_caps *)caps->priv;

    rig->state.priv = (struct icm710itu_priv_data *)calloc(1,
                                                           sizeof(struct icm710itu_priv_data));

    if (!rig->state.priv)
    {
        /* whoops! memory shortage! */
        return -RIG_ENOMEM;
    }

    priv = rig->state.priv;

    priv->remote_id = priv_caps->default_remote_id;
    priv->split = RIG_SPLIT_OFF;
    memset(priv->mode_str, 0, NUM_MODE_STR * sizeof(char *));
    MD_CW = strdup("A1A");
    MD_USB = strdup("J3E");
    MD_LSB = strdup("LSB");
    MD_AM = strdup("H3E");
    MD_FSK = strdup("FSK");
    MD_AFSK = strdup("J2B");
    priv->flagTuneOnNewTxfreq = 1;
    priv->lastTunedTxfreq = 0;
    priv->tuneTimeout = 10000;

    return RIG_OK;
}

int icm710itu_set_freq(RIG *rig, vfo_t vfo, freq_t freq)
{
    char freqbuf[BUFSZ];
    struct icm710itu_priv_data *priv;
    int retval;

    priv = (struct icm710_priv_data *)rig->state.priv;

    sprintf(freqbuf, "%.6f", freq / MHz(1));

    if (RIG_SPLIT_OFF == priv->split)
    {
        retval = icm710itu_set_tx_freq(rig, vfo, freq);

        if (retval != RIG_OK)
        {
            return retval;
        }
    }

    return icmarine_transaction(rig, CMD_RXFREQ, freqbuf, NULL);
}

int icm710itu_set_tx_freq(RIG *rig, vfo_t vfo, freq_t freq)
{
    char freqbuf[BUFSZ];
    int retval;
    struct icm710itu_priv_data *priv;

    sprintf(freqbuf, "%.6f", freq / MHz(1));

    retval = icmarine_transaction(rig, CMD_TXFREQ, freqbuf, NULL);
    if (retval != RIG_OK)
    {
        return retval;
    }

    priv = (struct icm710itu_priv_data *)rig->state.priv;
    if (priv->flagTuneOnNewTxfreq == 0)
    {
        return retval;
    }

    if (priv->lastTunedTxfreq != freq)
    {
        retval = icm710itu_vfo_op(rig, vfo, RIG_OP_TUNE);
        if (retval == RIG_OK)
        {
            priv->lastTunedTxfreq = freq;
        }
    }
    return retval;
}

int icm710itu_set_split_vfo(RIG *rig, vfo_t rx_vfo, split_t split, vfo_t tx_vfo)
{
    struct icm710itu_priv_data *priv;
    int retval;

    priv = (struct icm710itu_priv_data *)rig->state.priv;

    /* when disabling split mode */
    if (RIG_SPLIT_ON == priv->split &&
        RIG_SPLIT_OFF == split)
    {
        freq_t freq;
        retval = icmarine_get_freq(rig, rx_vfo, &freq);
        if (RIG_OK == retval)
        {
            retval = icm710itu_set_tx_freq(rig, rx_vfo, freq);
        }
    }

    priv->split = split;

    return retval;
}

int icm710itu_vfo_op(RIG *rig, vfo_t vfo, vfo_op_t op)
{
    if (RIG_OP_TUNE != op && RIG_OP_NONE != op)
    {
        return -RIG_EINVAL;
    }

    struct icm710itu_priv_data *priv;
    int retval;
    int oldTimeout = rig->state.rigport.timeout;
    int tuneStatus = -1;

    priv = (struct icm710itu_priv_data *)rig->state.priv;

    rig->state.rigport.timeout = priv->tuneTimeout;

    retval = icmarine_transaction(rig, CMD_TUNER, "TUNE", NULL);

    rig->state.rigport.timeout = oldTimeout;

    if (retval != RIG_OK)
    {
        return retval;
    }

    retval = icmarine_get_func(rig, vfo, RIG_FUNC_TUNER, &tuneStatus);

    if (retval != RIG_OK)
    {
        return retval;
    }

    return tuneStatus;
}
