/* 
 *  Icom Marine radio NMEA Command strings
 *
 */
#ifndef _ICMARINE_NMEA_H
#define _ICMARINE_NMEA_H 1

#define CMD_RXFREQ  "RXF"   /* Receive frequency */
#define CMD_TXFREQ  "TXF"   /* Transmit frequency */
#define CMD_MODE    "MODE"  /* Modulation mode */
#define CMD_RFGAIN  "RFG"   /* 0..n, typically n = 9 */
#define CMD_RFPWR   "TXP"   /* Tx Power. 1..n, typically n = 3, model dependent */
#define CMD_AGC     "AGC"   /* AGC ON | OFF */
#define CMD_NB      "NB"    /* Noise Blanking ON | OFF */
#define CMD_SQLC    "SQLC"  /* Squelch Control ON | OFF */
#define CMD_AFGAIN  "AFG"   /* Speaker Vol 0..255 */
#define CMD_TUNER   "TUNER" /* ON | TUNE | OFF */
#define CMD_PTT     "TRX"   /* PTT: TX | RX. TX selects modulation from NMEA port. Auto tuning controlled by Set Mode */
#define CMD_SQLS    "SQLS"  /* Squelch status OPEN | CLOSE - Output only */
#define CMD_SMETER  "SIGM"  /* S-meter read 0..8 - Output only */
#define CMD_POMETER "POM"   /* Power meter 0..8 - Output only */
#define CMD_ANTC    "ANTM"  /* Antenna Current meter 0..7 - Output only */
#define CMD_SPKR    "SP"    /* Speaker ON | OFF */
#define CMD_DISPDIM "DIM"   /* Dim display ON | OFF */
#define CMD_REMOTE  "REMOTE"    /* Remote ON | DSC | OFF. DSC not used */

#endif /* _ICMARINE_NMEA_H */