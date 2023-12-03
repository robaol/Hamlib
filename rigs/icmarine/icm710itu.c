/*
 *  Hamlib ICOM Marine backend - alternative IC-M710 caps and functions
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

/*  
 *  An alternative IC-M710 backend - M710ITU
 * 
 *  by Rob, M5RAO
 * 
 *  The initial reason for this backend was that my M710GMDSS radio did not
 *  respond to the MODE commands of the existing M710 backend; it required the
 *  use of the ITU emission codes (e.g. J3E instead of USB). So, my initial 
 *  plan was to find a simple means of changing the strings passed in the MODE
 *  command.
 *  I don't know whether:
 *  - some models ship expecting the more commonplace MODE names,
 *  - the radios that use the M710 backend have been customised to change the
 *    mode labels through the cloning interface,
 *  - changing the mode label would change MODE command keywords for each
 *    emission type.
 * 
 *  I did some experiments with the radio and learned more about it.
 *  I was using an M710GMDSS with an AT-120. I found that using "set mode" to
 *  select the AT-120 didn't work at all (neither auto nor manual tune) and I
 *  needed to select AT-130 to get the radio to perform a tuning cycle.
 * 
 *  The following table summarises the interface port names and signals
 *  involved in CAT (via NMEA), AF out and Modulation in, taken from the instruction
 *  manual (Section 7 Connector information):
 * 
 *      Signals |             M710 Model
 *      on port | GMDSS     | Marine    | General
 *      =========================================
 *      CWK,SEND|           |           |
 *      MOD,AF  |           | ACC1      | ACC1
 *      SCAN,ALC|           |           |
 *      14V,GND |           |           |
 *      -----------------------------------------
 *      8V,SEND |           |           |
 *      ALC,RLC | ACC       | ACC2      | ACC2
 *      14V,GND |           |           |
 *      -----------------------------------------
 *      MOD,AF  |           |           |
 *      NMEA I/O| DSC       |           |
 *      GND     |           |           |
 *      -----------------------------------------
 *      MOD,AF  |           |           |
 *      SEND,GND| MOD/AF    |           |
 *      -----------------------------------------
 *      MOD,AF  |           |           |
 *      NMEA I/O|           | REMOTE    | REMOTE
 *      GND     |           |           |
 *      -----------------------------------------
 *      Notes:
 *      1. The signal order on the ports is not correct, to highlight signal
 *         similarity
 *      2. Some of the MOD and AF signals in some ports are differential,
 *         omitted for clarity
 *      3. The signal listed as SEND for the MOD/AF port is actually called
 *         NSEN. I changed some signal names to highlight their common function
 *         where they have the same function but a different, port-specific
 *         name.
 *      4. I rounded 13.8V to 14V to fit in the table.
 * 
 *  I found that:
 *    - when I used the MOD/AF:SEND line to put the radio in to TX, the
 *      AutoTune function operated. 
 *    - when I used the DSC NMEA Interface (command TRX,TX) to put the radio
 *      in to TX,
 *      -   the MOD/AF port's MOD signal was not transmitted,
 *      -   the AutoTune function did not operate
 *      -   acknowledgements to the commands were received from the radio,
 *          exactly as described in the NMEA instruction manual.
 *    - using the Clone pin as the control interface worked to some extent but 
 *      resulted in all commands sent being echoed back into the controlling PC
 *      serial port as it is a 1-wire interface. TODO CHECK I also found that there were
 *      no command acknowledgements from the radio on this interface /CHECK.
 *    - the radio does not respond to a TUNER,TUNE command until tuning is
 *      complete. This can take several seconds. This is also stated in the
 *      NMEA instruction doc.
 * 
 *  Later, I found that the TRX,TX method of transmitting selects the DSC Mod
 *  signal for transmission. I also found the NMEA manual says that the TRX
 *  command "uses modulation port on NMEA port". The acknowledgement
 *  description for the TX argument says "Transmit mode including tuning
 *  antenna tuner". As above, I found that the AutoTune did not work with the
 *  TRX,TX command.
 *  
 *  I reviewed the existing icm710 backend and found that:
 *  - it is designed througout to cope without any responses from the radio,
 *  - it relies on the AutoTune function operating correctly, as it uses only
 *    TUNER,ON and TUNER,OFF, which enable and disable AutoTune, provided
 *    AutoTune is enabled through "set mode".
 *  - its software is completely decoupled from the icmarine backend and
 *    implements all the functions of icmarine, with the exception of the
 *    icmarine_transaction function.
 * 
 *  The reliance on Autotune suggests that this backend model is designed to
 *  work with a SEND electrical signal.
 * 
 *  I did not wish to generate an analogue SEND signal or to connect to the
 *  MOD/AF port as well as the NMEA port, which is required for CAT commands.
 *  The NMEA port also provides complete responses, as described by the
 *  instruction manual and so a new interface could make use of them.
 * 
 *  I decided to write new versions of a small number of functions that relate
 *  to transmit frequency so that I could automate the sending of TUNER,TUNE
 *  commands whenever a new tx frequency is sent to the radio. I keep a record
 *  of the last frequency tuned so that un-neccessary TUNEs can be avoided.
 *  During the TUNE, I temporarily install a 10s timeout.
 * 
 *  As the existing icm710 backend model was already almost completely
 *  decoupled from the icmarine code, I removed the icmarine.h inclusion and
 *  declared the single needed function in its place. This code is now
 *  completely separate, except for initialisation by icmarine.
 * 
 *  I also added the full set of NMEA commands in the instruction manual to the
 *  icmarine backend. The existing consumers of this backend should be
 *  un-affected by the new capabilities/functions as their function flags have
 *  not been changed.
 *      
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

#define ICM710ITU_MODES (RIG_MODE_SSB | RIG_MODE_CW | RIG_MODE_RTTY | RIG_MODE_PKTUSB)
#define ICM710ITU_RX_MODES (ICM710ITU_MODES | RIG_MODE_AM)

#define ICM710ITU_FUNC_ALL (RIG_FUNC_NB | RIG_FUNC_SQL | RIG_FUNC_TUNER | RIG_FUNC_MUTE)

#define ICM710ITU_LEVEL_ALL (RIG_LEVEL_RAWSTR |RIG_LEVEL_AF | RIG_LEVEL_RF | \
                             RIG_LEVEL_RFPOWER | RIG_LEVEL_RFPOWER_METER | RIG_LEVEL_AGC)

#define ICM710ITU_VFO_ALL (RIG_VFO_A | RIG_VFO_B)

#define ICM710ITU_TARGETABLE_VFO (RIG_TARGETABLE_FREQ)

#define ICMARINE_TUNER_TIMEOUTMS    (10000)
#define ICM710ITU_VFO_OPS (RIG_OP_TUNE)

#define ICM710ITU_SCAN_OPS (RIG_SCAN_NONE)

#define ICM710ITU_PARM_ALL (RIG_PARM_BACKLIGHT)

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

static const struct icmarine_priv_caps icm710itu_priv_caps =
    {
        .default_remote_id = 0x01, /* default address */
};

const struct rig_caps icm710itu_caps =
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
        .has_get_func = ICM710ITU_FUNC_ALL,
        .has_set_func = ICM710ITU_FUNC_ALL,
        .has_get_level = ICM710ITU_LEVEL_ALL,
        .has_set_level = RIG_LEVEL_SET(ICM710ITU_LEVEL_ALL),
        .has_get_parm = ICM710ITU_PARM_ALL,
        .has_set_parm = ICM710ITU_PARM_ALL,
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
        .targetable_vfo = ICM710ITU_TARGETABLE_VFO,
        .vfo_ops = ICM710ITU_VFO_OPS,
        //.scan_ops =  ICM710ITU_SCAN_OPS,
        .transceive = RIG_TRN_OFF,
        .bank_qty = 0,
        .chan_desc_sz = 0,

        .chan_list = {
            RIG_CHAN_END,
        },

        .rx_range_list1 = {
            {kHz(500), MHz(30) - 100, ICM710ITU_RX_MODES, -1, -1, ICM710ITU_VFO_ALL},
            RIG_FRNG_END,
        },
        .tx_range_list1 = {
            {kHz(1600), MHz(3) - 100, ICM710ITU_MODES, W(60), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(4), MHz(5) - 100, ICM710ITU_MODES, W(60), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(6), MHz(7) - 100, ICM710ITU_MODES, W(60), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(8), MHz(9) - 100, ICM710ITU_MODES, W(60), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(12), MHz(14) - 100, ICM710ITU_MODES, W(60), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(16), MHz(18) - 100, ICM710ITU_MODES, W(60), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(18), MHz(20) - 100, ICM710ITU_MODES, W(60), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(22), MHz(23) - 100, ICM710ITU_MODES, W(60), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(25), MHz(27.500), ICM710ITU_MODES, W(60), W(60), ICM710ITU_VFO_ALL, RIG_ANT_1},
            RIG_FRNG_END,
        },

        .rx_range_list2 = {
            {kHz(500), MHz(30) - 100, ICM710ITU_RX_MODES, -1, -1, ICM710ITU_VFO_ALL},
            RIG_FRNG_END,
        },
        .tx_range_list2 = {
            {kHz(1600), MHz(3) - 100, ICM710ITU_MODES, W(20), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(4), MHz(5) - 100, ICM710ITU_MODES, W(20), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(6), MHz(7) - 100, ICM710ITU_MODES, W(20), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(8), MHz(9) - 100, ICM710ITU_MODES, W(20), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(12), MHz(14) - 100, ICM710ITU_MODES, W(20), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(16), MHz(18) - 100, ICM710ITU_MODES, W(20), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(18), MHz(20) - 100, ICM710ITU_MODES, W(20), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(22), MHz(23) - 100, ICM710ITU_MODES, W(20), W(150), ICM710ITU_VFO_ALL, RIG_ANT_1},
            {MHz(25), MHz(27.500), ICM710ITU_MODES, W(20), W(60), ICM710ITU_VFO_ALL, RIG_ANT_1},
            RIG_FRNG_END,
        },

        .tuning_steps = {
            {ICM710ITU_RX_MODES, Hz(1)},
            RIG_TS_END,
        },
        /* mode/filter list, remember: order matters! */
        .filters = {
            {RIG_MODE_SSB | RIG_MODE_CW | RIG_MODE_RTTY | RIG_MODE_PKTUSB, kHz(2.3)},
            {RIG_MODE_AM, kHz(14)},
            RIG_FLT_END,
        },

        .cfgparams = icm710itu_cfg_params,
        .set_conf = icmarine_set_conf,
        .get_conf = icmarine_get_conf,

        .priv = (void *)&icm710itu_priv_caps,
        .rig_init = icm710itu_init,
        .rig_cleanup = icmarine_cleanup,
        .rig_open = NULL,
        .rig_close = NULL,

        .set_freq = icm710itu_set_freq,
        .get_freq = icm710itu_get_freq,
        .set_split_freq = icm710itu_set_tx_freq,
        .get_split_freq = icmarine_get_tx_freq,
        .set_split_vfo = icm710itu_set_split_vfo,
        .get_split_vfo = icm710itu_get_split_vfo,
        .set_mode = icmarine_set_mode,
        .get_mode = icmarine_get_mode,

        .set_ptt = icmarine_set_ptt,
        .get_ptt = icmarine_get_ptt,
        .get_dcd = icmarine_get_dcd,
        .vfo_op = icm710itu_vfo_op,

        .set_level = icmarine_set_level,
        .get_level = icmarine_get_level,
        .set_func = icm710itu_set_func,
        .get_func = icmarine_get_func,
        .set_parm = icmarine_set_parm,
        .get_parm = icmarine_get_parm,

};

/*
 * NMEA 0183 protocol is handled by icmarine_transaction, defined in icmarine.c
 *
 */

#define BUFSZ 96

/* Tokens */
#define TOK_REMOTEID TOKEN_BACKEND(1)

const struct confparams icm710itu_cfg_params[] =
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
    const struct icmarine_priv_caps *priv_caps;
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

    priv_caps = (const struct icmarine_priv_caps *)caps->priv;

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
    priv->tuneTimeout = ICMARINE_TUNER_TIMEOUTMS;

    return RIG_OK;
}

/*
 * icm710itu_get_freq
 * Assumes rig!=NULL, freq!=NULL
 */
int icm710itu_get_freq(RIG *rig, vfo_t vfo, freq_t *freq)
{
    int retval;
    char freqbuf[BUFSZ] = "";
    double d;
    int getTXF = 0;

    rig_debug(RIG_DEBUG_TRACE, "%s:\n", __func__);

    if (vfo == RIG_VFO_B || vfo == RIG_VFO_TX)
    {
        getTXF = 1;
    }

    retval = icmarine_transaction(rig, getTXF? CMD_TXFREQ : CMD_RXFREQ, NULL, freqbuf);

    if (retval != RIG_OK)
    {
        return retval;
    }

    if (freqbuf[0] == '\0')
    {
        *freq = 0;
    }
    else
    {
        if (sscanf(freqbuf, "%lf", &d) != 1)
        {
            rig_debug(RIG_DEBUG_ERR, "%s: sscanf('%s') failed\n", __func__, freqbuf);
            return -RIG_EPROTO;
        }

        *freq = (freq_t)(d * MHz(1));
    }

    return RIG_OK;
}

int icm710itu_set_freq(RIG *rig, vfo_t vfo, freq_t freq)
{
    char freqbuf[BUFSZ];
    struct icm710itu_priv_data *priv;
    int retval;

    priv = (struct icm710itu_priv_data *)rig->state.priv;

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

int icm710itu_get_split_vfo(RIG *rig, vfo_t rx_vfo, split_t *split,
                           vfo_t *tx_vfo)
{
    struct icm710itu_priv_data *priv;

    rig_debug(RIG_DEBUG_TRACE, "%s:\n", __func__);

    priv = (struct icm710itu_priv_data *)rig->state.priv;

    *split = priv->split;
    if (*split == RIG_SPLIT_OFF)
    {
        *tx_vfo = rx_vfo;        
    }
    else
    {
        if (rx_vfo == RIG_VFO_B || rx_vfo == RIG_VFO_SUB)
        {
            rig_debug(RIG_DEBUG_VERBOSE, "%s ********************** called vfo=%s\n",
                      __func__, rig_strvfo(rx_vfo));
        }

        *tx_vfo = RIG_VFO_B;
    }

    return RIG_OK;
}

int icm710itu_set_split_vfo(RIG *rig, vfo_t rx_vfo, split_t split, vfo_t tx_vfo)
{
    struct icm710itu_priv_data *priv;
    int retval = RIG_OK;

    priv = (struct icm710itu_priv_data *)rig->state.priv;

    /* when disabling split mode */
    if (RIG_SPLIT_ON == priv->split &&
        RIG_SPLIT_OFF == split)
    {
        freq_t freq;
        retval = icm710itu_get_freq(rig, rx_vfo, &freq);
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

    priv = (struct icm710itu_priv_data *)rig->state.priv;

    rig->state.rigport.timeout = priv->tuneTimeout;

    /*
     * The most natural command is Tuner, Tune but the response is 
     * Tuner, On or Tuner, Off and so you never get a response that
     * matches the command, which icmarine_transaction expects. But 
     * Tuner, On as a command, has the same effect (causes tuning) and so is a 
     * pragmatic solution...
     *    retval = icmarine_transaction(rig, CMD_TUNER, "TUNE", NULL);
     * 
     * If tuning fails, the response is Tuner, Off which 
     * icmarine_transaction classifies as RIG_ERJCTED,
     * which is what I would do if I got Tuner, Off from a get status 
     * RIG_FUNC_TUNER, so there is no need to test the tuner status separately.
     * 
     */
    retval = icmarine_transaction(rig, CMD_TUNER, "ON", NULL);

    rig->state.rigport.timeout = oldTimeout;

    return retval;
}

int icm710itu_set_func(RIG *rig, vfo_t vfo, setting_t func, int status)
{
    int retval;

    rig_debug(RIG_DEBUG_TRACE, "%s:\n", __func__);

    switch (func)
    {
    case RIG_FUNC_NB:
        retval = icmarine_transaction(rig, CMD_NB, status ? "ON" : "OFF", NULL);
        break;

    case RIG_FUNC_SQL:
        retval = icmarine_transaction(rig, CMD_SQLC, status ? "ON" : "OFF", NULL);
        break;

    case RIG_FUNC_TUNER:
    {
        struct icm710itu_priv_data *priv;
        int oldTimeout = rig->state.rigport.timeout;

        priv = (struct icm710itu_priv_data *)rig->state.priv;

        if (status)
        {
            rig->state.rigport.timeout = priv->tuneTimeout;
        }

        retval = icmarine_transaction(rig, CMD_TUNER, status ? "ON" : "OFF", NULL);

        if (status)
        {
            rig->state.rigport.timeout = oldTimeout;
        }
    }
    break;

    case RIG_FUNC_MUTE:
        retval = icmarine_transaction(rig, CMD_SPKR, status ? "OFF" : "ON", NULL);
        break;

    default:
        return -RIG_EINVAL;
    }

    return retval;
}
