/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
	LoRa lincentrator Hardware Abstraction Layer

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Sylvain Miermont


Modified by linjinzhi <jz.lin@siat.ac.cn>
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>		/* C99 types */
#include <stdbool.h>	/* bool type */
#include <stdio.h>		/* printf fprintf */
#include <string.h>		/* memcpy */

#include <stdint.h>		/* C99 types */
#include <stdbool.h>	/* bool type */
#include <stdio.h>		/* printf, fprintf, snprintf, fopen, fputs */
#include <string.h>		/* memset */
#include <signal.h>		/* sigaction */
#include <time.h>		/* time, clock_gettime, strftime, gmtime */
#include <sys/time.h>	/* timeval */
#include <unistd.h>		/* getopt, access */
#include <stdlib.h>		/* atoi, exit */

#include <sys/time.h>	/* timeval */
#include <wiringPi.h>

#include "loragw_reg.h"
#include "loragw_hal.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_HAL == 1
	#define DEBUG_MSG(str)				fprintf(stderr, str)
	#define DEBUG_PRINTF(fmt, args...)	fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
	#define DEBUG_ARRAY(a,b,c)			for(a=0;a<b;++a) fprintf(stderr,"%x.",c[a]);fprintf(stderr,"end\n")
	#define CHECK_NULL(a)				if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_HAL_ERROR;}
#else
	#define DEBUG_MSG(str)
	#define DEBUG_PRINTF(fmt, args...)
	#define DEBUG_ARRAY(a,b,c)			for(a=0;a!=0;){}
	#define CHECK_NULL(a)				if(a==NULL){return LGW_HAL_ERROR;}
#endif

#define IF_HZ_TO_REG(f)		(f << 5)/15625
#define	SET_PPM_ON(bw,dr)	(((bw == BW_125KHZ) && ((dr == DR_LORA_SF11) || (dr == DR_LORA_SF12))) || ((bw == BW_250KHZ) && (dr == DR_LORA_SF12)))
#define TRACE()				fprintf(stderr, "@ %s %d\n", __FUNCTION__, __LINE__);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS & TYPES -------------------------------------------- */

#define		MCU_ARB		0
#define		MCU_AGC		1
#define		MCU_ARB_FW_BYTE		8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */
#define		MCU_AGC_FW_BYTE		8192 /* size of the firmware IN BYTES (= twice the number of 14b words) */
#define		FW_VERSION_ADDR		0x20 /* Address of firmware version in data memory */
#define		FW_VERSION_CAL		2 /* Expected version of calibration firmware */
#define		FW_VERSION_AGC		4 /* Expected version of AGC firmware */
#define		FW_VERSION_ARB		1 /* Expected version of arbiter firmware */

#define		TX_METADATA_NB		16
#define		RX_METADATA_NB		16

#define		AGC_CMD_WAIT		16
#define		AGC_CMD_ABORT		17

#define		MIN_LORA_PREAMBLE		4
#define		STD_LORA_PREAMBLE		6
#define		MIN_FSK_PREAMBLE		3
#define		STD_FSK_PREAMBLE		5
#define		PLL_LOCK_MAX_ATTEMPTS	5

#define		TX_START_DELAY		1500

/*
SX1257 frequency setting :
F_register(24bit) = F_rf (Hz) / F_step(Hz)
                  = F_rf (Hz) * 2^19 / F_xtal(Hz)
                  = F_rf (Hz) * 2^19 / 32e6
                  = F_rf (Hz) * 256/15625

SX1255 frequency setting :
F_register(24bit) = F_rf (Hz) / F_step(Hz)
                  = F_rf (Hz) * 2^20 / F_xtal(Hz)
                  = F_rf (Hz) * 2^20 / 32e6
                  = F_rf (Hz) * 512/15625
*/
#define 	SX125x_32MHz_FRAC	15625	/* irreductible fraction for PLL register caculation */

#define		SX125x_TX_DAC_CLK_SEL	1	/* 0:int, 1:ext */
#define		SX125x_TX_DAC_GAIN		2	/* 3:0, 2:-3, 1:-6, 0:-9 dBFS (default 2) */
#define		SX125x_TX_MIX_GAIN		14	/* -38 + 2*TxMixGain dB (default 14) */
#define		SX125x_TX_PLL_BW		3	/* 0:75, 1:150, 2:225, 3:300 kHz (default 3) */
#define		SX125x_TX_ANA_BW		0	/* 17.5 / 2*(41-TxAnaBw) MHz (default 0) */
#define		SX125x_TX_DAC_BW		5	/* 24 + 8*TxDacBw Nb FIR taps (default 2) */
#define		SX125x_RX_LNA_GAIN		1	/* 1 to 6, 1 highest gain */
#define		SX125x_RX_BB_GAIN		12	/* 0 to 15 , 15 highest gain */
#define 	SX125x_LNA_ZIN			1	/* 0:50, 1:200 Ohms (default 1) */
#define		SX125x_RX_ADC_BW		7	/* 0 to 7, 2:100<BW<200, 5:200<BW<400,7:400<BW kHz SSB (default 7) */
#define		SX125x_RX_ADC_TRIM		6	/* 0 to 7, 6 for 32MHz ref, 5 for 36MHz ref */
#define 	SX125x_RX_BB_BW			0	/* 0:750, 1:500, 2:375; 3:250 kHz SSB (default 1, max 3) */
#define 	SX125x_RX_PLL_BW		0	/* 0:75, 1:150, 2:225, 3:300 kHz (default 3, max 3) */
#define 	SX125x_ADC_TEMP			0	/* ADC temperature measurement mode (default 0) */
#define 	SX125x_XOSC_GM_STARTUP	13	/* (default 13) */
#define 	SX125x_XOSC_DISABLE		2	/* Disable of Xtal Oscillator blocks bit0:regulator, bit1:core(gm), bit2:amplifier */

#define		RSSI_MULTI_BIAS			-35		/* difference between "multi" modem RSSI offset and "stand-alone" modem RSSI offset */
#define		RSSI_FSK_BIAS			-37.0	/* difference between FSK modem RSSI offset and "stand-alone" modem RSSI offset */
#define		RSSI_FSK_REF			-70.0	/* linearize FSK RSSI curve around -70 dBm */
#define		RSSI_FSK_SLOPE			0.8

/* Version string, used to identify the library version/options once compiled */
const char lgw_version_string[] = "Version: " LIBLORAGW_VERSION ";";


/*
The following static variables are the configuration set that the user can
modify using rxrf_setconf, rxif_setconf and txgain_setconf functions.
The functions _start and _send then use that set to configure the hardware.

Parameters validity and coherency is verified by the _setconf functions and
the _start and _send functions assume they are valid.
*/

static bool lgw_is_started;

static bool rf_enable[LGW_CHAN_NB_MAX];
static uint32_t rf_freq[LGW_CHAN_NB_MAX]; /* absolute, in Hz */
static float rf_rssi_offset[LGW_CHAN_NB_MAX];
static bool rf_tx_enable[LGW_CHAN_NB_MAX];
static enum lgw_radio_type_e rf_radio_type[LGW_CHAN_NB_MAX];
static enum lgw_mode_type_e rf_mode_type[LGW_CHAN_NB_MAX];
static uint8_t rf_bandwidth[LGW_CHAN_NB_MAX];
static uint8_t rf_sf[LGW_CHAN_NB_MAX];
static uint32_t	rf_datarate[LGW_CHAN_NB_MAX];
static uint8_t	rf_sync_word_size[LGW_CHAN_NB_MAX];
static uint64_t	rf_sync_word[LGW_CHAN_NB_MAX];
static uint8_t rf_status[LGW_CHAN_NB_MAX];

// TODO: config the right pins
struct lgw_conf_board_s board_conf = {
	.lorawan_public = false,
	.pins_DIO = {5,6,0,0,0,0},
	.pin_RST = 0,
	.pin_CS = {1,2,3,4,0,0,0,0,0,0}
};

static struct lgw_tx_gain_lut_s txgain_lut = {
	.size = 2,
	.lut[0] = {
		.dig_gain = 0,
		.pa_gain = 2,
		.dac_gain = 3,
		.mix_gain = 10,
		.rf_power = 14
	},
	.lut[1] = {
		.dig_gain = 0,
		.pa_gain = 3,
		.dac_gain = 3,
		.mix_gain = 14,
		.rf_power = 27
	}};

#define TX_TIMEOUT_SEC 5
#define STORE_RX_NB 32
struct lgw_pkt_rx_s storedRXPkts[STORE_RX_NB];
static uint8_t ptrRXPkts;

double difftimespec(struct timespec end, struct timespec beginning) {
    double x;

    x = 1E-9 * (double)(end.tv_nsec - beginning.tv_nsec);
    x += (double)(end.tv_sec - beginning.tv_sec);

    return x;
}
/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

void sx127x_write(uint8_t channel, uint8_t addr, uint8_t data) {
	/* checking input parameters */
	if (channel >= LGW_CHAN_NB_MAX) {
		DEBUG_MSG("ERROR: INVALID RF_CHAIN\n");
		return;
	}
	if (addr >= 0x7F) {
		DEBUG_MSG("ERROR: ADDRESS OUT OF RANGE\n");
		return;
	}
	if(rf_enable[channel] == false) {
		DEBUG_MSG("ERROR: RF_CHAIN IS NOT ENABLE\n");
		return;
	}
	lgw_reg_w( channel, addr, data);
	return;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint8_t sx127x_read(uint8_t channel, uint8_t addr) {
	uint8_t read_value;
	/* checking input parameters */
	if (channel >= LGW_CHAN_NB_MAX) {
		DEBUG_MSG("ERROR: INVALID RF_CHAIN\n");
		return 0;
	}
	if (addr >= 0x7F) {
		DEBUG_MSG("ERROR: ADDRESS OUT OF RANGE\n");
		return 0;
	}
	if(rf_enable[channel] == false) {
		DEBUG_MSG("ERROR: RF_CHAIN IS NOT ENABLE\n");
		return 0;
	}
	lgw_reg_r(channel, addr, &read_value);
	return read_value;

}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
inline void opmode(uint8_t channel, uint8_t mode){
	sx127x_write(channel,RegOpMode,(sx127x_read(channel,RegOpMode)&~OPMODE_MASK)|mode);
}

inline void opmodeLora(uint8_t channel){
	sx127x_write(channel,RegOpMode,OPMODE_LORA);
}
inline void opmodeFSK(uint8_t channel){
	sx127x_write(channel,RegOpMode,0x00);
}
inline void setFreq(uint8_t channel, uint32_t freq){ // unit of freq is Hz. eg, setFreq(0,434100000);
	uint64_t frf = ((uint64_t)freq<< 19) / 32000000;
	sx127x_write(channel, RegFrfMsb, (uint8_t)(frf>>16));
	sx127x_write(channel, RegFrfMid, (uint8_t)(frf>> 8));
	sx127x_write(channel, RegFrfLsb, (uint8_t)(frf>> 0));

	uint8_t f1 = sx127x_read(channel, RegFrfMsb);
	uint8_t f2 = sx127x_read(channel, RegFrfMid);
	uint8_t f3 = sx127x_read(channel, RegFrfLsb);
	DEBUG_PRINTF("freq = %u, RegFrfMsb = 0x%02x, RegFrfMid = 0x%02x, RegFrfLsb = 0x%02x\n", freq, f1, f2, f3);
}
enum { RXMODE_SINGLE, RXMODE_SCAN, RXMODE_RSSI };

static const uint8_t rxlorairqmask[] = {
    [RXMODE_SINGLE] = IRQ_LORA_RXDONE_MASK|IRQ_LORA_RXTOUT_MASK,
    [RXMODE_SCAN]   = IRQ_LORA_RXDONE_MASK,
    [RXMODE_RSSI]   = 0x00,
};

int rxlora (uint8_t channel, uint8_t rxmode) {
    // select LoRa modem (from sleep mode)
	opmode(channel,OPMODE_SLEEP);
    opmodeLora(channel);

    if((sx127x_read(channel, RegOpMode) & OPMODE_LORA) == 0){
		DEBUG_PRINTF("ERROR: RADIO CHIP #%d CANNOT SET TO LORA MODE\n", channel);
		return LGW_HAL_ERROR;
    }
    // don't use MAC settings at startup
    if(rxmode == RXMODE_RSSI) { // use fixed settings for rssi scan
    	sx127x_write(channel, LORARegModemConfig1, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1);
    	sx127x_write(channel, LORARegModemConfig2, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2);

        // disable LoRa modem interrupts
    	sx127x_write(channel, LORARegIrqFlagsMask,IRQ_LORA_RXTOUT_MASK|IRQ_LORA_RXDONE_MASK|IRQ_LORA_CRCERR_MASK|IRQ_LORA_HEADER_MASK|IRQ_LORA_TXDONE_MASK|IRQ_LORA_CDDONE_MASK|IRQ_LORA_FHSSCH_MASK|IRQ_LORA_CDDETD_MASK);
    } else { // single or continuous rx mode
        // configure LoRa modem
    	uint8_t mc1 = 0,mc2 = 0,mc3 = 0;
		uint8_t bw = rf_bandwidth[channel];
		switch(bw) {
			case BW_125KHZ: mc1 |= SX1276_MC1_BW_125;  break;
			case BW_250KHZ: mc1 |= SX1276_MC1_BW_250;  break;
			case BW_500KHZ: mc1 |= SX1276_MC1_BW_500;  break;
			case BW_62K5HZ: mc1 |= SX1276_MC1_BW_62K5; break;
			case BW_31K2HZ: mc1 |= SX1276_MC1_BW_31K2; break;
			case BW_15K6HZ: mc1 |= SX1276_MC1_BW_15K6; break;
			case BW_7K8HZ:  mc1 |= SX1276_MC1_BW_7K8; break;
			default: {
				DEBUG_PRINTF("ERROR: UNSUPPORTED BANDWIDTH IN LORA MODE FOR CHANNEL %d\n", channel);
				return LGW_HAL_ERROR;
			}
		}

    	mc1 |= SX1276_MC1_CR_4_5;
    	//mc1 |= SX1276_MC1_IMPLICIT_HEADER_MODE_ON;

		uint8_t sf = rf_sf[channel];
		switch(sf) {
			case DR_LORA_SF7:  mc2 |= SX1272_MC2_SF7;  break;
			case DR_LORA_SF8:  mc2 |= SX1272_MC2_SF8;  break;
			case DR_LORA_SF9:  mc2 |= SX1272_MC2_SF9;  break;
			case DR_LORA_SF10: mc2 |= SX1272_MC2_SF10; break;
			case DR_LORA_SF11: mc2 |= SX1272_MC2_SF11; break;
			case DR_LORA_SF12: mc2 |= SX1272_MC2_SF12; break;
			default: {
				DEBUG_PRINTF("ERROR: UNSUPPORTED SPREADINGFACTOR IN LORA MODE FOR CHANNEL %d\n", channel);
				return LGW_HAL_ERROR;
			}
		}

    	mc2 |= SX1276_MC2_RX_PAYLOAD_CRCON;

    	mc3 |= SX1276_MC3_AGCAUTO;
    	mc3 |= SX1276_MC3_LOW_DATA_RATE_OPTIMIZE;

    	sx127x_write(channel,LORARegModemConfig1,mc1);
    	sx127x_write(channel,LORARegModemConfig2,mc2);
    	sx127x_write(channel,LORARegModemConfig3,mc3);

    	mc1 = sx127x_read(channel, LORARegModemConfig1);
    	mc2 = sx127x_read(channel, LORARegModemConfig2);
    	mc3 = sx127x_read(channel, LORARegModemConfig3);
    	DEBUG_PRINTF("mc1 = 0x%02x, mc2 = 0x%02x, mc3 = 0x%02x\n", mc1, mc2, mc3);

        // configure frequency
    	if(rf_freq[channel] < 169000000 && (bw == BW_250KHZ || bw == BW_500KHZ)){
			DEBUG_PRINTF("ERROR: In the lower band(169MHz), bandwidths BW_250KHZ and BW_500KHZ are not supported IN LORA MODE FOR CHANNEL %d\n", channel);
			return LGW_HAL_ERROR;
    	}
    	setFreq(channel, rf_freq[channel]);
    }

    sx127x_write(channel, RegLna, (0x20 | 0x01));
    sx127x_write(channel, LORARegPayloadMaxLength, 0x40);
    sx127x_write(channel, LORARegSymbTimeoutLsb, 0x8); // set symbol timeout (for single rx)
    sx127x_write(channel, LORARegSyncWord, LORA_MAC_PREAMBLE);
    sx127x_write(channel, RegDioMapping1, MAP_DIO0_LORA_RXDONE|MAP_DIO1_LORA_RXTOUT|MAP_DIO2_LORA_NOP); // configure DIO mapping DIO0=RxDone DIO1=RxTout DIO2=NOP
    sx127x_write(channel, LORARegIrqFlags, 0xFF); // clear all radio IRQ flags
    sx127x_write(channel, LORARegIrqFlagsMask, ~rxlorairqmask[rxmode]); // enable required radio IRQs

    sx127x_write(channel, LORARegFifoRxBaseAddr, 0);
    sx127x_write(channel, LORARegFifoAddrPtr, 0);


    // now instruct the radio to receive
    if (rxmode == RXMODE_SINGLE) { // single rx
        wait_ms(3000); // TODO: optimization, busy wait until exact rx time
        opmode(channel, OPMODE_RX_SINGLE);
        DEBUG_PRINTF("INFO: RADIO CHIP #%d IS SET TO SINGLE RX MODE\n", channel);
    } else { // continous rx (scan or rssi)
        opmode(channel, OPMODE_RX);
        DEBUG_PRINTF("INFO: RADIO CHIP #%d IS SET TO CONTINOUS RX MODE\n", channel);
        wait_ms(5);
    }

    return LGW_HAL_SUCCESS;
}

int rxfsk (uint8_t channel, uint8_t rxmode) {
	// TODO: not tested yet!
    // only single rx (no continuous scanning, no noise sampling)
    if( rxmode != RXMODE_SINGLE ){
		DEBUG_MSG("ERROR: ONLY SINGLE RX MODE CAN BE SET IN FSK MODE\n");
		return LGW_HAL_ERROR;
    }
    // select FSK modem (from sleep mode)
	opmode(channel,OPMODE_SLEEP);
    opmodeFSK(channel);
    if((sx127x_read(channel, RegOpMode) & OPMODE_LORA) != 0){
		DEBUG_PRINTF("ERROR: RADIO CHIP #%d CANNOT SET TO FSK MODE\n", channel);
		return LGW_HAL_ERROR;
    }

    opmode(channel, OPMODE_STANDBY);

    // configure frequency
    setFreq(channel, 434100000);

    // set LNA gain
    //sx127x_write(channel, RegLna, 0x20|0x03); // max gain, boost enable
    sx127x_write(channel, RegLna, 0x20 | 0x01);
    // configure receiver
    sx127x_write(channel, FSKRegRxConfig, 0x1E); // AFC auto, AGC, trigger on preamble?!?
    // set receiver bandwidth
    sx127x_write(channel, FSKRegRxBw, 0x0B); // 50kHz SSb
    // set AFC bandwidth
    sx127x_write(channel, FSKRegAfcBw, 0x12); // 83.3kHz SSB
    // set preamble detection
    sx127x_write(channel, FSKRegPreambleDetect, 0xAA); // enable, 2 bytes, 10 chip errors
    // set sync config
    sx127x_write(channel, FSKRegSyncConfig, 0x12); // no auto restart, preamble 0xAA, enable, fill FIFO, 3 bytes sync
    // set packet config
    sx127x_write(channel, FSKRegPacketConfig1, 0xD8); // var-length, whitening, crc, no auto-clear, no adr filter
    sx127x_write(channel, FSKRegPacketConfig2, 0x40); // packet mode
    // set sync value
    sx127x_write(channel, FSKRegSyncValue1, 0xC1);
    sx127x_write(channel, FSKRegSyncValue2, 0x94);
    sx127x_write(channel, FSKRegSyncValue3, 0xC1);
    // set preamble timeout
    sx127x_write(channel, FSKRegRxTimeout2, 0xFF);//(LMIC.rxsyms+1)/2);
    // set bitrate
    sx127x_write(channel, FSKRegBitrateMsb, 0x02); // 50kbps
    sx127x_write(channel, FSKRegBitrateLsb, 0x80);
    // set frequency deviation
    sx127x_write(channel, FSKRegFdevMsb, 0x01); // +/- 25kHz
    sx127x_write(channel, FSKRegFdevLsb, 0x99);

    // configure DIO mapping DIO0=PayloadReady DIO1=NOP DIO2=TimeOut
    sx127x_write(channel, RegDioMapping1, MAP_DIO0_FSK_READY|MAP_DIO1_FSK_NOP|MAP_DIO2_FSK_TIMEOUT);

    // enable antenna switch for RX
    //hal_pin_rxtx(0);

    // now instruct the radio to receive
    //hal_waitUntil(LMIC.rxtime); // busy wait until exact rx time
    wait_ms(500);
    opmode(channel, OPMODE_RX); // no single rx mode available in FSK
    DEBUG_PRINTF("INFO: RADIO CHIP #%d IS SET TO FSK RX MODE\n", channel);
    return LGW_HAL_SUCCESS;
}

int txlora (uint8_t channel, uint8_t *data, uint16_t size) {
	opmode(channel,OPMODE_SLEEP);
    opmodeLora(channel);
    if((sx127x_read(channel, RegOpMode) & OPMODE_LORA) == 0){
		DEBUG_PRINTF("ERROR: RADIO CHIP #%d CANNOT SET TO LORA MODE\n", channel);
		return LGW_HAL_ERROR;
    }

    // enter standby mode (required for FIFO loading))
    opmode(channel, OPMODE_STANDBY);

    // configure LoRa modem
	uint8_t mc1 = 0,mc2 = 0,mc3 = 0;

	uint8_t bw = rf_bandwidth[channel];
	switch(bw) {
		case BW_125KHZ: mc1 |= SX1276_MC1_BW_125;  break;
		case BW_250KHZ: mc1 |= SX1276_MC1_BW_250;  break;
		case BW_500KHZ: mc1 |= SX1276_MC1_BW_500;  break;
		case BW_62K5HZ: mc1 |= SX1276_MC1_BW_62K5; break;
		case BW_31K2HZ: mc1 |= SX1276_MC1_BW_31K2; break;
		case BW_15K6HZ: mc1 |= SX1276_MC1_BW_15K6; break;
		case BW_7K8HZ:  mc1 |= SX1276_MC1_BW_7K8; break;
		default: {
			DEBUG_PRINTF("ERROR: UNSUPPORTED BANDWIDTH IN LORA MODE FOR CHANNEL %d\n", channel);
			return LGW_HAL_ERROR;
		}
	}

	mc1 |= SX1276_MC1_CR_4_5;
	//mc1 |= SX1276_MC1_IMPLICIT_HEADER_MODE_ON;

	uint8_t sf = rf_sf[channel];
	switch(sf) {
		case DR_LORA_SF7:  mc2 |= SX1272_MC2_SF7;  break;
		case DR_LORA_SF8:  mc2 |= SX1272_MC2_SF8;  break;
		case DR_LORA_SF9:  mc2 |= SX1272_MC2_SF9;  break;
		case DR_LORA_SF10: mc2 |= SX1272_MC2_SF10; break;
		case DR_LORA_SF11: mc2 |= SX1272_MC2_SF11; break;
		case DR_LORA_SF12: mc2 |= SX1272_MC2_SF12; break;
		default: {
			DEBUG_PRINTF("ERROR: UNSUPPORTED SPREADINGFACTOR IN LORA MODE FOR CHANNEL %d\n", channel);
			return LGW_HAL_ERROR;
		}
	}

	mc2 |= SX1276_MC2_RX_PAYLOAD_CRCON;
	mc3 |= SX1276_MC3_AGCAUTO;
	mc3 |= SX1276_MC3_LOW_DATA_RATE_OPTIMIZE;

	sx127x_write(channel,LORARegModemConfig1,mc1);
	sx127x_write(channel,LORARegModemConfig2,mc2);
	sx127x_write(channel,LORARegModemConfig3,mc3);

	mc1 = sx127x_read(channel, LORARegModemConfig1);
	mc2 = sx127x_read(channel, LORARegModemConfig2);
	mc3 = sx127x_read(channel, LORARegModemConfig3);
	DEBUG_PRINTF("mc1 = 0x%02x, mc2 = 0x%02x, mc3 = 0x%02x\n", mc1, mc2, mc3);

    // configure frequency
	if(rf_freq[channel] < 169000000 && (bw == BW_250KHZ || bw == BW_500KHZ)){
		DEBUG_PRINTF("ERROR: In the lower band(169MHz), bandwidths BW_250KHZ and BW_500KHZ are not supported IN LORA MODE FOR CHANNEL %d\n", channel);
		return LGW_HAL_ERROR;
	}
	setFreq(channel, rf_freq[channel]);

    // configure output power
    sx127x_write(channel, RegPaRamp, (sx127x_read(channel, RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec
    int pw = 15;
    sx127x_write(channel, RegPaConfig, (uint8_t)(0x80|(pw&0xf)));
    sx127x_write(channel, RegPaDac, sx127x_read(channel, RegPaDac)|0x4);

    // set sync word
    sx127x_write(channel, LORARegSyncWord, LORA_MAC_PREAMBLE);
    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    sx127x_write(channel, RegDioMapping1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
    // clear all radio IRQ flags
    sx127x_write(channel, LORARegIrqFlags, 0xFF);
    // mask all IRQs but TxDone
    sx127x_write(channel, LORARegIrqFlagsMask, ~IRQ_LORA_TXDONE_MASK);
    // initialize the payload size and address pointers
    sx127x_write(channel, LORARegFifoTxBaseAddr, 0x00);
    sx127x_write(channel, LORARegFifoAddrPtr, 0x00);
    sx127x_write(channel, LORARegPayloadLength, size);
    // download buffer to the radio FIFO
    lgw_reg_wb( channel, RegFifo, data, size);

    // enable antenna switch for TX
    //hal_pin_rxtx(1);

    // now we actually start the transmission
    opmode(channel, OPMODE_TX);
    DEBUG_PRINTF("INFO: RADIO CHIP #%d IS SET TO TX MODE\n", channel);
    return LGW_HAL_SUCCESS;
}

int txfsk (uint8_t channel, uint8_t *data, uint16_t size)  {
	// TODO: not verified yet!
    // select FSK modem (from sleep mode)
	opmode(channel,OPMODE_SLEEP);
    sx127x_write(channel, RegOpMode, 0x10); // FSK, BT=0.5
    if(sx127x_read(channel, RegOpMode) != 0x10){
		DEBUG_PRINTF("ERROR: RADIO CHIP #%d CANNOT SET TO FSK MODE\n", channel);
		return LGW_HAL_ERROR;
    }
    // enter standby mode (required for FIFO loading))
    opmode(channel, OPMODE_STANDBY);
    // set bitrate
    sx127x_write(channel, FSKRegBitrateMsb, 0x02); // 50kbps
    sx127x_write(channel, FSKRegBitrateLsb, 0x80);
    // set frequency deviation
    sx127x_write(channel, FSKRegFdevMsb, 0x01); // +/- 25kHz
    sx127x_write(channel, FSKRegFdevLsb, 0x99);

    // frame and packet handler settings
    sx127x_write(channel, FSKRegPreambleMsb, 0x00);
    sx127x_write(channel, FSKRegPreambleLsb, 0x05);
    sx127x_write(channel, FSKRegSyncConfig, 0x12);
    sx127x_write(channel, FSKRegPacketConfig1, 0xD0);
    sx127x_write(channel, FSKRegPacketConfig2, 0x40);
    sx127x_write(channel, FSKRegSyncValue1, 0xC1);
    sx127x_write(channel, FSKRegSyncValue2, 0x94);
    sx127x_write(channel, FSKRegSyncValue3, 0xC1);

    // configure frequency
    setFreq(channel, 434100000);

    // configure output power
    int pw = 12;
    sx127x_write(channel, RegPaConfig, (uint8_t)(0x80|(pw&0xf)));
    sx127x_write(channel, RegPaDac, sx127x_read(channel, RegPaDac)|0x4);

    // set the IRQ mapping DIO0=PacketSent DIO1=NOP DIO2=NOP
    sx127x_write(channel, RegDioMapping1, MAP_DIO0_FSK_READY|MAP_DIO1_FSK_NOP|MAP_DIO2_FSK_TXNOP);

    // initialize the payload size and address pointers
    sx127x_write(channel, FSKRegPayloadLength, size+1); // (insert length byte into payload))

    // download length byte and buffer to the radio FIFO
    sx127x_write(channel, RegFifo, size);
    lgw_reg_wb( channel, RegFifo, data, size);

    // enable antenna switch for TX
    //hal_pin_rxtx(1);

    // now we actually start the transmission
    opmode(channel, OPMODE_TX);
    DEBUG_PRINTF("INFO: RADIO CHIP #%d IS SET TO FSK TX MODE\n", channel);
    return LGW_HAL_SUCCESS;
}

int switch_into_RX(uint8_t channel) {
	if(rf_status[channel] == LGW_STATUS_RXING) {
		return LGW_HAL_SUCCESS;
	}

	if(rf_status[channel] == LGW_STATUS_TXING) {
		DEBUG_PRINTF("ERROR: [switch_into_RX] CHANNEL #%d IS SENDING DATA, CANNOT SWITCH INTO RX MODE\n", channel);
		return LGW_HAL_ERROR;
	}

	if(rf_status[channel] == LGW_STATUS_FREE) {
		if(rf_mode_type[channel] == LGW_MODE_TYPE_LORA){
			if(rxlora(channel, RXMODE_SCAN) == LGW_HAL_ERROR) {
				DEBUG_PRINTF("ERROR: [switch_into_RX] RF_CHAIN #%d SETUP TO LORA_RX MODE FAILED\n", channel);
				return LGW_HAL_ERROR;
			}
		} else if(rf_mode_type[channel] == LGW_MODE_TYPE_FSK){
			if(rxfsk(channel, RXMODE_SINGLE) == LGW_HAL_ERROR) {
				DEBUG_PRINTF("ERROR: [switch_into_RX] RF_CHAIN #%d SETUP TO FSK_RX MODE FAILED\n", channel);
				return LGW_HAL_ERROR;
			}
		} else {
			DEBUG_PRINTF("ERROR: [switch_into_RX] CHANNEL #%d ONLY LGW_MODE_TYPE_LORA, LGW_MODE_TYPE_FSK ARE SUPPORTED\n", channel);
			return LGW_HAL_ERROR;
		}
	}

	rf_status[channel] = LGW_STATUS_RXING;
	return LGW_HAL_SUCCESS;
}

int setup_sx127x(uint8_t channel) {
	DEBUG_PRINTF("Setting up SX127x #%d...\n",channel);
	if (channel >= LGW_CHAN_NB_MAX) {
		DEBUG_PRINTF("ERROR: INVALID RF_CHAIN %d\n", channel);
		return LGW_HAL_ERROR;
	}

	if(rf_enable[channel]){
		if(rf_radio_type[channel] != LGW_RADIO_TYPE_SX1278 && rf_radio_type[channel] != LGW_RADIO_TYPE_SX1276){ // currently we only support 1278 and 1276 chip types
			DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d FOR RADIO TYPE OF CHANNEL %d\n", rf_radio_type[channel], channel);
			return LGW_HAL_ERROR;
		}

		// RxChainCalibration();
/*
		DEBUG_PRINTF("INFO: RADIO CHIP #%d IS DOING Rx Chain Calibration.\n", channel);
		sx127x_write(channel, FSKRegImageCal, (sx127x_read(channel,FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
		while((sx127x_read(channel,FSKRegImageCal) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING){
		}
*/
		sx127x_write(channel, RegOpMode, OPMODE_SLEEP);

		uint8_t ver = sx127x_read(channel, RegVersion);
		DEBUG_PRINTF("Note: SX127x #%d version register returned 0x%02x\n", channel, ver);
		if(ver != 0x12) {
			DEBUG_PRINTF("ERROR: WRONG CHIP VERSION FOR RF_CHAIN #%d\n", channel);
			rf_enable[channel] = false;
			return LGW_HAL_ERROR;
		}

		return switch_into_RX(channel);

	} else {
		DEBUG_PRINTF("Note: CHANNEL %d IS NOT ENABLED\n", channel);
	}

	return LGW_HAL_SUCCESS;
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lgw_board_setconf(struct lgw_conf_board_s conf) {

	/* check if the lincentrator is running */
	if (lgw_is_started == true) {
		DEBUG_MSG("ERROR: LINCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
		return LGW_HAL_ERROR;
	}

	/* set internal config according to parameters */
	board_conf.lorawan_public = conf.lorawan_public;
	for(int i = 0; i < 6; i++) {
		board_conf.pins_DIO[i] = conf.pins_DIO[i];
	}
	board_conf.pin_RST = conf.pin_RST;

	for(int i = 0; i < 10; i++) {
		board_conf.pin_CS[i] = conf.pin_CS[i];
	}

	DEBUG_PRINTF("Note: board configuration; lorawan_public:%d, DIO = {%d,%d,%d,%d,%d,%d},CS = "
			"{%d,%d,%d,%d,%d,%d,%d,%d,%d,%d}, RST = %d\n",
			board_conf.lorawan_public,board_conf.pins_DIO[0],board_conf.pins_DIO[1],board_conf.pins_DIO[2],board_conf.pins_DIO[3],board_conf.pins_DIO[4],board_conf.pins_DIO[5],
			board_conf.pin_CS[0],board_conf.pin_CS[1],board_conf.pin_CS[2],board_conf.pin_CS[3],board_conf.pin_CS[4],board_conf.pin_CS[5],board_conf.pin_CS[6],board_conf.pin_CS[7],
			board_conf.pin_CS[8],board_conf.pin_CS[9],board_conf.pin_RST);

	return LGW_HAL_SUCCESS;
}

int lgw_rxrf_setconf(uint8_t rf_chain, struct lgw_conf_rxrf_s conf) {

	/* check if the lincentrator is running */
	if (lgw_is_started == true) {
		DEBUG_MSG("ERROR: LINCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
		return LGW_HAL_ERROR;
	}

	/* check input range (segfault prevention) */
	if (rf_chain >= LGW_CHAN_NB_MAX) {
		DEBUG_MSG("ERROR: NOT A VALID RF_CHAIN NUMBER\n");
		return LGW_HAL_ERROR;
	}

	/* check if radio type is supported */
	if ((conf.type != LGW_RADIO_TYPE_SX1276) && (conf.type != LGW_RADIO_TYPE_SX1278)) {
		DEBUG_PRINTF("ERROR: NOT A VALID RADIO TYPE IN CHANNEL #%d\n", rf_chain);
		return LGW_HAL_ERROR;
	}
	if(conf.mode == LGW_MODE_TYPE_LORA) {
		rf_sf[rf_chain] = conf.sf;
	} else if(conf.mode == LGW_MODE_TYPE_FSK){
		rf_datarate[rf_chain] = conf.datarate;
		if (conf.sync_word > 0) {
			rf_sync_word_size[rf_chain] = conf.sync_word_size;
			rf_sync_word[rf_chain] = conf.sync_word;
		}
	} else {
		DEBUG_PRINTF("ERROR: NOT A VALID MODE TYPE IN CHANNEL #%d\n", rf_chain);
		return LGW_HAL_ERROR;
	}
	/* set internal config according to parameters */
	rf_enable[rf_chain] = conf.enable;
	rf_freq[rf_chain] = conf.freq_hz;
	rf_rssi_offset[rf_chain] = conf.rssi_offset;
	rf_radio_type[rf_chain] = conf.type;
	rf_mode_type[rf_chain] = conf.mode;
	rf_tx_enable[rf_chain] = conf.tx_enable;
	// TODO: verify if this is right
	if ((((int32_t)conf.freq_hz + conf.bandwidth/2)) > LGW_FRQ_H) {
		DEBUG_PRINTF("ERROR: IF FREQUENCY %d TOO HIGH\n", conf.freq_hz);
		return LGW_HAL_ERROR;
	} else if (((int32_t)(conf.freq_hz - conf.bandwidth/2)) < LGW_FRQ_L) {
		DEBUG_PRINTF("ERROR: IF FREQUENCY %d TOO LOW\n", conf.freq_hz);
		return LGW_HAL_ERROR;
	}
	rf_bandwidth[rf_chain] = conf.bandwidth;
	rf_status[rf_chain] = LGW_STATUS_FREE;

	DEBUG_PRINTF("Note: rf_chain %d configuration; en:%d freq:%d rssi_offset:%f radio_type:%d tx_enable:%d\n", rf_chain, rf_enable[rf_chain], rf_freq[rf_chain], rf_rssi_offset[rf_chain], rf_radio_type[rf_chain], rf_tx_enable[rf_chain]);

	return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_txgain_setconf(struct lgw_tx_gain_lut_s *conf) {
	int i;

	/* Check LUT size */
	if ((conf->size < 1) || (conf->size > TX_GAIN_LUT_SIZE_MAX)) {
		DEBUG_PRINTF("ERROR: TX gain LUT must have at least one entry and  maximum %d entries\n", TX_GAIN_LUT_SIZE_MAX);
		return LGW_HAL_ERROR;
	}

	txgain_lut.size = conf->size;

	for (i = 0; i < txgain_lut.size; i++) {
		/* Check gain range */
		if (conf->lut[i].dig_gain > 3) {
			DEBUG_MSG("ERROR: TX gain LUT: SX1301 digital gain must be between 0 and 3\n");
			return LGW_HAL_ERROR;
		}
		if (conf->lut[i].dac_gain != 3) {
			DEBUG_MSG("ERROR: TX gain LUT: SX1257 DAC gains != 3 are not supported\n");
			return LGW_HAL_ERROR;
		}
		if (conf->lut[i].mix_gain > 15) {
			DEBUG_MSG("ERROR: TX gain LUT: SX1257 mixer gain must not exceed 15\n");
			return LGW_HAL_ERROR;
		} else if (conf->lut[i].mix_gain < 8) {
			DEBUG_MSG("ERROR: TX gain LUT: SX1257 mixer gains < 8 are not supported\n");
			return LGW_HAL_ERROR;
		}
		if (conf->lut[i].pa_gain > 3) {
			DEBUG_MSG("ERROR: TX gain LUT: External PA gain must not exceed 3\n");
			return LGW_HAL_ERROR;
		}

		/* Set internal LUT */
		txgain_lut.lut[i].dig_gain = conf->lut[i].dig_gain;
		txgain_lut.lut[i].dac_gain = conf->lut[i].dac_gain;
		txgain_lut.lut[i].mix_gain = conf->lut[i].mix_gain;
		txgain_lut.lut[i].pa_gain  = conf->lut[i].pa_gain;
		txgain_lut.lut[i].rf_power = conf->lut[i].rf_power;
	}

	return LGW_HAL_SUCCESS;
}

void stop_irq(int pin) {
	int gpio_pin= wpiPinToGpio(pin);
	char command [64] ;
	sprintf(command, "/usr/local/bin/gpio edge %d none", gpio_pin);
	system(command);
}

void start_irq(int pin) {
	int gpio_pin= wpiPinToGpio(pin);
	char command [64] ;
	sprintf(command, "/usr/local/bin/gpio edge %d rising", gpio_pin);
	system(command);
}

void radio_irq_handler(){
	stop_irq(board_conf.pins_DIO[0]);

	for (uint8_t channel = 0; channel < LGW_CHAN_NB_MAX; channel++) {
		if(rf_enable[channel] == false) continue;
		if( rf_mode_type[channel] == LGW_MODE_TYPE_LORA){ // LORA modem
			uint8_t flags = sx127x_read(channel, LORARegIrqFlags);
			//DEBUG_PRINTF("IRQ Flags in channel #%d is 0x%02x\n", channel,flags);
			if(flags & IRQ_LORA_TXDONE_MASK) {
				uint32_t count_us;
				lgw_get_trigcnt(&count_us);
				DEBUG_PRINTF("TX DONE in channel #%d, now(count_us) = %u\n", channel, count_us);
				// switch to continuous RX
				if(rf_status[channel] != LGW_STATUS_TXING) {
					DEBUG_PRINTF("[Warning] Radio status of channel #%d is not in TXING, but have received a TXDONE IRQ\n", channel);
				}
				rf_status[channel] = LGW_STATUS_FREE;
				// clear the TXDONE flag
				sx127x_write(channel, LORARegIrqFlags, IRQ_LORA_TXDONE_MASK);
			}
			if(flags & IRQ_LORA_RXDONE_MASK) {
				struct lgw_pkt_rx_s *p = &storedRXPkts[ptrRXPkts++];
				memset(p,0,sizeof(struct lgw_pkt_rx_s));

				lgw_get_trigcnt(&(p->count_us));
				DEBUG_PRINTF("RX DONE in channel #%d, count_us = %u\n", channel,p->count_us);

				p->size = (sx127x_read(channel, LORARegModemConfig1) & SX1276_MC1_IMPLICIT_HEADER_MODE_ON) ?sx127x_read(channel, LORARegPayloadLength) : sx127x_read(channel, LORARegRxNbBytes);

				// set FIFO read address pointer
				sx127x_write(channel, LORARegFifoAddrPtr, sx127x_read(channel, LORARegFifoRxCurrentAddr));

				// now read the FIFO
				lgw_reg_rb( channel, RegFifo, p->payload, p->size);

				p->if_chain = p->rf_chain = channel;
				p->freq_hz = (uint32_t)((int32_t)rf_freq[p->rf_chain]);
				p->modulation = MOD_LORA;

				DEBUG_PRINTF("Receiving a packet(size %d) in channel #%d in radio_irq_handler().\n",p->size,channel);

				// TODO: verify whether the following settings right or wrong
				p->snr = p->snr_min = p->snr_max = (float)sx127x_read(channel, LORARegPktSnrValue); // SNR [dB] * 4
				p->rssi = (float)sx127x_read(channel, LORARegPktRssiValue) - 125 + 64; // RSSI [dBm] (-196...+63)
				if((flags & 0x20) == 0x20) {
					p->status = STAT_CRC_BAD;
				} else {
					p->status = STAT_CRC_OK;
				}

				p->bandwidth = rf_bandwidth[channel];

				uint8_t mc1 = sx127x_read(channel,LORARegModemConfig1);
				uint8_t mc2 = sx127x_read(channel,LORARegModemConfig2);
				//uint8_t mc3 = sx127x_read(channel,LORARegModemConfig3);
				uint8_t sf = (mc2 >> 4) & 0x0F;
				switch (sf) {
					case 6: p->datarate = DR_LORA_SF6; break;
					case 7: p->datarate = DR_LORA_SF7; break;
					case 8: p->datarate = DR_LORA_SF8; break;
					case 9: p->datarate = DR_LORA_SF9; break;
					case 10: p->datarate = DR_LORA_SF10; break;
					case 11: p->datarate = DR_LORA_SF11; break;
					case 12: p->datarate = DR_LORA_SF12; break;
					default: p->datarate = DR_UNDEFINED;
				}
				uint8_t cr = (mc1 >> 1) & 0x07;
				switch (cr) {
					case 1: p->coderate = CR_LORA_4_5; break;
					case 2: p->coderate = CR_LORA_4_6; break;
					case 3: p->coderate = CR_LORA_4_7; break;
					case 4: p->coderate = CR_LORA_4_8; break;
					default: p->coderate = CR_UNDEFINED;
				}
				// clear the RXDNOE and PayloadCRCError flag
				sx127x_write(channel, LORARegIrqFlags, IRQ_LORA_RXDONE_MASK|IRQ_LORA_CRCERR_MASK|IRQ_LORA_HEADER_MASK|IRQ_LORA_CDDONE_MASK|IRQ_LORA_CDDETD_MASK);
			}

		} else { // FSK modem
			// TODO:
		}
	}

	start_irq(board_conf.pins_DIO[0]);
}
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_start(void) {
	int reg_stat;

	if (lgw_is_started == true) {
		DEBUG_MSG("Note: LoRa LINCENTRATOR already started, restarting it now\n");
	}

	if(wiringPiSetup() < 0){
		DEBUG_MSG("ERROR: Fail to open wiringPi\n");
		return LGW_HAL_ERROR;
	}

	for(int i = 0; i < 10; i++) {
		if(board_conf.pin_CS[i] > 0){
			pinMode(board_conf.pin_CS[i], OUTPUT);
			digitalWrite(board_conf.pin_CS[i],1);
		}
	}

	reg_stat = lgw_connect();

	if (reg_stat == LGW_REG_ERROR) {
		DEBUG_MSG("ERROR: FAIL TO CONNECT BOARD\n");
		return LGW_HAL_ERROR;
	}

	// reset all the chips
	if(board_conf.pin_RST > 0)
		lgw_manually_reset();

	/* setup the radios */
	for(int channel = 0; channel < LGW_CHAN_NB_MAX; channel++) {
		if(setup_sx127x(channel) == LGW_HAL_ERROR){
			DEBUG_PRINTF("ERROR: FAIL TO SETUP CHIP #%d\n",channel);
		}
		for(int i = 0; i < STORE_RX_NB; i++) {
			memset(&storedRXPkts[i],0,sizeof(struct lgw_pkt_rx_s));
		}
		ptrRXPkts = 0;
	}

	for(int i = 0; i < 6; i++) {
		if(board_conf.pins_DIO[i] > 0) {
			pinMode(board_conf.pins_DIO[i], INPUT);
			// register as interrupt
			// TODO: this can cause raspberry pi 3 to collapse
			/*
			pullUpDnControl(board_conf.pins_DIO[i], PUD_UP);
			if(wiringPiISR(board_conf.pins_DIO[i], INT_EDGE_RISING, &radio_irq_handler) < 0) {
				DEBUG_MSG("ERROR: Fail to setup ISR\n");
				return LGW_HAL_ERROR;
			}
			*/
		}
	}

	// TODO: to be tested, only enable DIO0, hopefully not cause Pi 3 to collapse
	pullUpDnControl(board_conf.pins_DIO[0], PUD_UP);
	if(wiringPiISR(board_conf.pins_DIO[0], INT_EDGE_RISING, &radio_irq_handler) < 0) {
		DEBUG_MSG("ERROR: Fail to setup ISR\n");
		return LGW_HAL_ERROR;
	}

	lgw_is_started = true;
	return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_stop(void) {
	// TODO: reset the chips
	lgw_manually_reset();
	if(lgw_disconnect() == LGW_HAL_ERROR){
		return LGW_HAL_ERROR;
	}
	lgw_is_started = false;
	return LGW_HAL_SUCCESS;
}

void lgw_manually_reset(){
	pinMode(board_conf.pin_RST, OUTPUT);
	digitalWrite(board_conf.pin_RST, 0);
	wait_ms(1);
	pinMode(board_conf.pin_RST, INPUT);
	wait_ms(6);
}
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
// the way to receive packets when not using RX IRQ
//int lgw_receive(uint8_t max_pkt, struct lgw_pkt_rx_s *pkt_data) {
//	int nb_pkt_fetch; /* loop variable and return value */
//	struct lgw_pkt_rx_s *p; /* pointer to the current structure in the struct array */
//
//	/* check if the lincentrator is running */
//	if (lgw_is_started == false) {
//		DEBUG_MSG("ERROR: LINCENTRATOR IS NOT RUNNING, START IT BEFORE RECEIVING\n");
//		return LGW_HAL_ERROR;
//	}
//
//	/* check input variables */
//	if (max_pkt <= 0) {
//		DEBUG_PRINTF("ERROR: %d = INVALID MAX NUMBER OF PACKETS TO FETCH\n", max_pkt);
//		return LGW_HAL_ERROR;
//	}
//
//	CHECK_NULL(pkt_data);
//
//	nb_pkt_fetch = 0;
//	for (uint8_t channel = 0; channel < LGW_CHAN_NB_MAX; channel++) {
//		if(rf_enable[channel] == false) continue;
//
//		/* point to the proper struct in the struct array */
//		p = &pkt_data[nb_pkt_fetch];
//		memset(p,0,sizeof(struct lgw_pkt_rx_s));
//
//		if( rf_mode_type[channel] == LGW_MODE_TYPE_LORA){ // LORA modem
//			uint8_t flags = sx127x_read(channel, LORARegIrqFlags);
//			if(flags & IRQ_LORA_RXDONE_MASK) {
//				p->size = (sx127x_read(channel, LORARegModemConfig1) & SX1276_MC1_IMPLICIT_HEADER_MODE_ON) ?
//						sx127x_read(channel, LORARegPayloadLength) : sx127x_read(channel, LORARegRxNbBytes);
//
//				// set FIFO read address pointer
//				sx127x_write(channel, LORARegFifoAddrPtr, sx127x_read(channel, LORARegFifoRxCurrentAddr));
//
//				// now read the FIFO
//				lgw_reg_rb( channel, RegFifo, p->payload, p->size);
//
//	            p->if_chain = p->rf_chain = channel;
//	    		p->freq_hz = (uint32_t)((int32_t)rf_freq[p->rf_chain]);
//	            p->modulation = MOD_LORA;
//
//	            DEBUG_PRINTF("Receiving packet(size %d) in channel #%d, nb_pkt_fetch = %d, max_pkt = %d\n",p->size,channel,nb_pkt_fetch,max_pkt);
//
//	            // TODO: verify whether the following settings right or wrong
//	            p->snr = p->snr_min = p->snr_max = (float)sx127x_read(channel, LORARegPktSnrValue); // SNR [dB] * 4
//	            p->rssi = (float)sx127x_read(channel, LORARegPktRssiValue) - 125 + 64; // RSSI [dBm] (-196...+63)
//	            if((flags & 0x20) == 0x20) {
//	            	p->status = STAT_CRC_BAD;
//	            } else {
//	            	p->status = STAT_CRC_OK;
//	            }
//
//	            p->bandwidth = rf_bandwidth[channel];
//
//	        	uint8_t mc1 = sx127x_read(channel,LORARegModemConfig1);
//	        	uint8_t mc2 = sx127x_read(channel,LORARegModemConfig2);
//	        	//uint8_t mc3 = sx127x_read(channel,LORARegModemConfig3);
//				uint8_t sf = (mc2 >> 4) & 0x0F;
//				switch (sf) {
//					case 7: p->datarate = DR_LORA_SF7; break;
//					case 8: p->datarate = DR_LORA_SF8; break;
//					case 9: p->datarate = DR_LORA_SF9; break;
//					case 10: p->datarate = DR_LORA_SF10; break;
//					case 11: p->datarate = DR_LORA_SF11; break;
//					case 12: p->datarate = DR_LORA_SF12; break;
//					default: p->datarate = DR_UNDEFINED;
//				}
//				uint8_t cr = (mc1 >> 1) & 0x07;
//				switch (cr) {
//					case 1: p->coderate = CR_LORA_4_5; break;
//					case 2: p->coderate = CR_LORA_4_6; break;
//					case 3: p->coderate = CR_LORA_4_7; break;
//					case 4: p->coderate = CR_LORA_4_8; break;
//					default: p->coderate = CR_UNDEFINED;
//				}
//
//	            // clear the RXDNOE and PayloadCRCError flag
//	            sx127x_write(channel, LORARegIrqFlags, IRQ_LORA_RXDONE_MASK|IRQ_LORA_CRCERR_MASK|IRQ_LORA_HEADER_MASK|IRQ_LORA_CDDONE_MASK|IRQ_LORA_CDDETD_MASK);
//			}
//
//		} else if(rf_mode_type[channel] == LGW_MODE_TYPE_FSK) { // FSK modem
//	        // uint8_t flags1 = sx127x_read(channel, FSKRegIrqFlags1);
//	        uint8_t flags2 = sx127x_read(channel, FSKRegIrqFlags2);
//	        if(flags2 & IRQ_FSK2_PAYLOADREADY_MASK){
//	        	p->size = sx127x_read(channel, FSKRegPayloadLength);
//				// now read the FIFO
//				lgw_reg_rb( channel, RegFifo, p->payload, p->size);
//
//				// TODO: verify whether the following settings right or wrong
//				p->snr = p->snr_min = p->snr_max = -128.0;
//				p->rssi = 0;
//				p->modulation = MOD_FSK;
//				p->bandwidth = rf_bandwidth[channel];
//				p->datarate = rf_datarate[channel];
//				p->coderate = CR_UNDEFINED;
//
//				if(flags2 & 0x02){
//					p->crc = STAT_CRC_OK;
//				} else {
//					p->crc = STAT_CRC_BAD;
//				}
//				// FSK modem no need to clear IRQ flags?
//	        }
//		}
//
//		if(p->size > 0 ) {
//			if(++nb_pkt_fetch >= max_pkt) break;
//		}
//	}
//
//	return nb_pkt_fetch;
//}
//

int lgw_receive(uint8_t max_pkt, struct lgw_pkt_rx_s *pkt_data) {
	int nb_pkt_fetch; /* loop variable and return value */
	struct lgw_pkt_rx_s *p; /* pointer to the current structure in the struct array */

	/* check if the lincentrator is running */
	if (lgw_is_started == false) {
		DEBUG_MSG("ERROR: LINCENTRATOR IS NOT RUNNING, START IT BEFORE RECEIVING\n");
		return LGW_HAL_ERROR;
	}

	// check input variables, needed to fetch all packets once in a time
	if (max_pkt < STORE_RX_NB) {
		DEBUG_PRINTF("ERROR: %d = INVALID MAX NUMBER OF PACKETS TO FETCH, NEEDED TO BE GREAT THAN %d\n", max_pkt, STORE_RX_NB);
		return LGW_HAL_ERROR;
	}

	CHECK_NULL(pkt_data);

	nb_pkt_fetch = 0;
	stop_irq(board_conf.pins_DIO[0]);

	for (uint8_t i = 0; i < ptrRXPkts; i++) {
		p = &pkt_data[nb_pkt_fetch];
		memcpy(p,&storedRXPkts[i],sizeof(struct lgw_pkt_rx_s));
		if(p->size > 0 ) {
			++nb_pkt_fetch;
		}
	}
	ptrRXPkts = 0;

	start_irq(board_conf.pins_DIO[0]);

	return nb_pkt_fetch;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_send(struct lgw_pkt_tx_s pkt_data) {
	/* check if the lincentrator is running */
	if (lgw_is_started == false) {
		DEBUG_MSG("ERROR: LINCENTRATOR IS NOT RUNNING, START IT BEFORE SENDING\n");
		return LGW_HAL_ERROR;
	}

	/* check input range (segfault prevention) */
	if (pkt_data.rf_chain >= LGW_CHAN_NB_MAX) {
		DEBUG_MSG("ERROR: INVALID RF_CHAIN TO SEND PACKETS\n");
		return LGW_HAL_ERROR;
	}

	/* check input variables */
	if (rf_tx_enable[pkt_data.rf_chain] == false) {
		DEBUG_MSG("ERROR: SELECTED RF_CHAIN IS DISABLED FOR TX ON SELECTED BOARD\n");
		return LGW_HAL_ERROR;
	}
	if (rf_enable[pkt_data.rf_chain] == false) {
		DEBUG_MSG("ERROR: SELECTED RF_CHAIN IS DISABLED\n");
		return LGW_HAL_ERROR;
	}
	if (!IS_TX_MODE(pkt_data.tx_mode)) {
		DEBUG_MSG("ERROR: TX_MODE NOT SUPPORTED\n");
		return LGW_HAL_ERROR;
	}
	if (pkt_data.modulation == MOD_LORA) {
		if (!IS_LORA_BW(pkt_data.bandwidth)) {
			DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY LORA TX\n");
			return LGW_HAL_ERROR;
		}
		if (!IS_LORA_STD_DR(pkt_data.datarate)) {
			DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY LORA TX\n");
			return LGW_HAL_ERROR;
		}
		if (!IS_LORA_CR(pkt_data.coderate)) {
			DEBUG_MSG("ERROR: CODERATE NOT SUPPORTED BY LORA TX\n");
			return LGW_HAL_ERROR;
		}
		if (pkt_data.size > 255) {
			DEBUG_MSG("ERROR: PAYLOAD LENGTH TOO BIG FOR LORA TX\n");
			return LGW_HAL_ERROR;
		}
	} else if (pkt_data.modulation == MOD_FSK) {
		if((pkt_data.f_dev < 1) || (pkt_data.f_dev > 200)) {
			DEBUG_MSG("ERROR: TX FREQUENCY DEVIATION OUT OF ACCEPTABLE RANGE\n");
			return LGW_HAL_ERROR;
		}
		if(!IS_FSK_DR(pkt_data.datarate)) {
			DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY FSK IF CHAIN\n");
			return LGW_HAL_ERROR;
		}
		if (pkt_data.size > 255) {
			DEBUG_MSG("ERROR: PAYLOAD LENGTH TOO BIG FOR FSK TX\n");
			return LGW_HAL_ERROR;
		}
	} else {
		DEBUG_MSG("ERROR: INVALID TX MODULATION\n");
		return LGW_HAL_ERROR;
	}

	/* interpretation of TX power */
	uint8_t pow_index = 0;
	for (pow_index = txgain_lut.size-1; pow_index > 0; pow_index--) {
		if (txgain_lut.lut[pow_index].rf_power <= pkt_data.rf_power) {
			break;
		}
	}

	uint8_t channel = pkt_data.rf_chain;

	if (pkt_data.modulation == MOD_LORA) {
		lgw_abort_tx(channel);
		opmodeLora(channel);
	    if((sx127x_read(channel, RegOpMode) & OPMODE_LORA) == 0){
			DEBUG_PRINTF("ERROR: RADIO CHIP #%d CANNOT SET TO LORA MODE\n", channel);
			return LGW_HAL_ERROR;
	    }
		opmode(channel, OPMODE_STANDBY);

		/* CRC, LoRa CR  SF ... */
		uint8_t mc1 = 0, mc2 = 0,mc3 = 0;
		switch (pkt_data.datarate) {
			case DR_LORA_SF7: mc2 |= SX1272_MC2_SF7; break;
			case DR_LORA_SF8: mc2 |= SX1272_MC2_SF8; break;
			case DR_LORA_SF9: mc2 |= SX1272_MC2_SF9; break;
			case DR_LORA_SF10: mc2 |= SX1272_MC2_SF10; break;
			case DR_LORA_SF11: mc2 |= SX1272_MC2_SF11; break;
			case DR_LORA_SF12: mc2 |= SX1272_MC2_SF12; break;
			default: DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d FOR DATA RATE\n", pkt_data.datarate);
		}
		switch (pkt_data.coderate) {
			case CR_LORA_4_5: mc1 |= SX1276_MC1_CR_4_5; break;
			case CR_LORA_4_6: mc1 |= SX1276_MC1_CR_4_6; break;
			case CR_LORA_4_7: mc1 |= SX1276_MC1_CR_4_7; break;
			case CR_LORA_4_8: mc1 |= SX1276_MC1_CR_4_8; break;
			default: DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d FOR CODE RATE\n", pkt_data.coderate);
		}
		switch (pkt_data.bandwidth) {
			case BW_125KHZ: mc1 |= SX1276_MC1_BW_125; break;
			case BW_250KHZ: mc1 |= SX1276_MC1_BW_250; break;
			case BW_500KHZ: mc1 |= SX1276_MC1_BW_500; break;
			default: DEBUG_PRINTF("ERROR: UNEXPECTED VALUE %d FOR BAND WIDTH\n", pkt_data.bandwidth);
		}
		if (pkt_data.no_crc == false) {
			mc2 |= SX1276_MC2_RX_PAYLOAD_CRCON; /* set 'CRC enable' bit */
		} else {
			DEBUG_MSG("Info: packet will be sent without CRC\n");
		}
		if (pkt_data.no_header == true) {
			mc1 |= SX1276_MC1_IMPLICIT_HEADER_MODE_ON; /* set 'implicit header' bit */
		}
		/* TODO: figure out how to set PPM and TX polarity
		if (SET_PPM_ON(pkt_data.bandwidth,pkt_data.datarate)) {
			buff[11] |= 0x08; // set 'PPM offset' bit at 1
		}

		if (pkt_data.invert_pol == true) {
			buff[11] |= 0x10; // set 'TX polarity' bit at 1
		}
		*/

		mc3 |= SX1276_MC3_AGCAUTO;
        if ((pkt_data.datarate == DR_LORA_SF11 || pkt_data.datarate == DR_LORA_SF12) && pkt_data.bandwidth == BW_125KHZ) {
            mc3 |= SX1276_MC3_LOW_DATA_RATE_OPTIMIZE;
        }

		sx127x_write(channel,LORARegModemConfig1,mc1);
		sx127x_write(channel,LORARegModemConfig2,mc2);
		sx127x_write(channel,LORARegModemConfig3,mc3);
    	mc1 = sx127x_read(channel, LORARegModemConfig1);
    	mc2 = sx127x_read(channel, LORARegModemConfig2);
    	mc3 = sx127x_read(channel, LORARegModemConfig3);
		DEBUG_PRINTF("txlora(#%d): mc1 = 0x%02x, mc2 = 0x%02x, mc3 = 0x%02x\n",channel, mc1, mc2, mc3);

		// configure frequency
		uint32_t freq = pkt_data.freq_hz;
		setFreq(channel, freq);

	    // configure output power
	    sx127x_write(channel, RegPaRamp, (sx127x_read(channel, RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec
	    sx127x_write(channel, RegPaConfig, (uint8_t)(0x80|(pow_index&0xf)));
	    sx127x_write(channel, RegPaDac, sx127x_read(channel, RegPaDac)|0x4);

	    // set sync word
	    sx127x_write(channel, LORARegSyncWord, LORA_MAC_PREAMBLE);
	    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
	    sx127x_write(channel, RegDioMapping1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
	    // clear all radio IRQ flags
	    sx127x_write(channel, LORARegIrqFlags, 0xFF);
	    // mask all IRQs but TxDone
	    sx127x_write(channel, LORARegIrqFlagsMask, ~IRQ_LORA_TXDONE_MASK);
	    // initialize the payload size and address pointers
	    sx127x_write(channel, LORARegFifoTxBaseAddr, 0x00);
	    sx127x_write(channel, LORARegFifoAddrPtr, 0x00);
	    sx127x_write(channel, LORARegPayloadLength, pkt_data.size);

		/* LoRa preamble size */
		if (pkt_data.preamble == 0) { /* if not explicit, use recommended LoRa preamble size */
			pkt_data.preamble = STD_LORA_PREAMBLE;
		} else if (pkt_data.preamble < MIN_LORA_PREAMBLE) { /* enforce minimum preamble size */
			pkt_data.preamble = MIN_LORA_PREAMBLE;
			DEBUG_MSG("Note: preamble length adjusted to respect minimum LoRa preamble size\n");
		}
		sx127x_write(channel, LORARegPreambleMsb, 0xFF & (pkt_data.preamble >> 8));
		sx127x_write(channel, LORARegPreambleLsb, 0xFF & pkt_data.preamble);

	    // download buffer to the radio FIFO
	    lgw_reg_wb( channel, RegFifo, pkt_data.payload, pkt_data.size);
	    // wait until the exactly right time
	    DEBUG_PRINTF("!!! OPMODE_TX, pkt_data.count_us = %u\n", pkt_data.count_us); // TODO: todel
	    uint32_t count_us;
	    do{
	    	lgw_get_trigcnt(&count_us);
	    }while((pkt_data.count_us - count_us) > 100); // error 0.1ms
	    // now we actually start the transmission
	    opmode(channel, OPMODE_TX);

	} else if (pkt_data.modulation == MOD_FSK) { // TODO: verify this FSK
		lgw_abort_tx(channel);
		opmode(channel, OPMODE_SLEEP);
		opmodeFSK(channel);
	    if(sx127x_read(channel, RegOpMode) != 0x10){
			DEBUG_PRINTF("ERROR: RADIO CHIP #%d CANNOT SET TO FSK MODE\n", channel);
			return -1;
	    }
		opmode(channel, OPMODE_STANDBY);

	    // set bitrate
		uint16_t fsk_dr_div = (uint16_t)((uint32_t)LGW_XTAL_FREQU / pkt_data.datarate); /* Ok for datarate between 500bps and 250kbps */
		sx127x_write(channel, FSKRegBitrateMsb, 0xFF & fsk_dr_div);
		sx127x_write(channel, FSKRegBitrateLsb, 0xFF & (fsk_dr_div >> 8));
		// set frequency deviation
		sx127x_write(channel, FSKRegFdevMsb, pkt_data.f_dev);
		sx127x_write(channel, FSKRegFdevLsb, 0x99);

		// frame and packet handler settings
		if (pkt_data.preamble == 0) { /* if not explicit, use LoRa MAC preamble size */
			pkt_data.preamble = STD_FSK_PREAMBLE;
		} else if (pkt_data.preamble < MIN_FSK_PREAMBLE) { /* enforce minimum preamble size */
			pkt_data.preamble = MIN_FSK_PREAMBLE;
			DEBUG_MSG("Note: preamble length adjusted to respect minimum FSK preamble size\n");
		}
		sx127x_write(channel, FSKRegPreambleMsb, pkt_data.preamble);
		sx127x_write(channel, FSKRegPreambleLsb, 0xFF & (pkt_data.preamble >> 8));
		sx127x_write(channel, FSKRegSyncConfig, 0x12);
		/* always in variable length packet mode, whitening, and CCITT CRC if CRC is not disabled  */
		uint8_t pc1 = ((0x40 | (pkt_data.no_crc?0:0x01)) << 4 ) | (pkt_data.no_crc?0:0x01) | (0x02 << 2);
		sx127x_write(channel, FSKRegPacketConfig1, pc1);
		sx127x_write(channel, FSKRegPacketConfig2, 0x40);
		sx127x_write(channel, FSKRegSyncValue1, 0xC1);
		sx127x_write(channel, FSKRegSyncValue2, 0x94);
		sx127x_write(channel, FSKRegSyncValue3, 0xC1);

		// configure frequency
		setFreq(channel, 434100000);

		// configure output power
		sx127x_write(channel, RegPaConfig, (uint8_t)(0x80|(pow_index&0xf)));
		sx127x_write(channel, RegPaDac, sx127x_read(channel, RegPaDac)|0x4);

		// set the IRQ mapping DIO0=PacketSent DIO1=NOP DIO2=NOP
		sx127x_write(channel, RegDioMapping1, MAP_DIO0_FSK_READY|MAP_DIO1_FSK_NOP|MAP_DIO2_FSK_TXNOP);

		uint8_t size = (uint8_t)pkt_data.size; /* TODO: how to handle 255 bytes packets ?!? */
		// initialize the payload size and address pointers
		sx127x_write(channel, FSKRegPayloadLength, size+1); // (insert length byte into payload))

		// download length byte and buffer to the radio FIFO
		sx127x_write(channel, RegFifo, size);
		lgw_reg_wb( channel, RegFifo, pkt_data.payload, size);

		// enable antenna switch for TX
		//hal_pin_rxtx(1);

		// now we actually start the transmission
		opmode(channel, OPMODE_TX);

	} else {
		DEBUG_MSG("ERROR: INVALID TX MODULATION..\n");
		return LGW_HAL_ERROR;
	}

	rf_status[channel] = LGW_STATUS_TXING;


	// wait until TXDONE or timeout
	time_t tx_start_time = time(NULL);
	do{
		if(rf_status[channel] == LGW_STATUS_FREE) // TXDONE has been processed in radio_irq_handle()
			break;
		// when not using TX interrupt, then uncomment the following statements
//		flags = sx127x_read(channel,LORARegIrqFlags);
//		if( flags & IRQ_LORA_TXDONE_MASK ) {
//			rf_status[channel] = LGW_STATUS_FREE;
//			// clear the TXDNOE flag
//			sx127x_write(channel, LORARegIrqFlags, IRQ_LORA_TXDONE_MASK);
//			break;
//		}
		if(difftime(time(NULL), tx_start_time) > TX_TIMEOUT_SEC){
			DEBUG_MSG("WARNING: TX TIMEOUT..\n");
			rf_status[channel] = LGW_STATUS_FREE;
			break;
		}
		wait_ms(100);
	} while(true);

	DEBUG_MSG("INFO: Switch into RX mode from lgw_send()\n");
	return switch_into_RX(channel);

}
// END of lgw_send()

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_status(uint8_t channel, uint8_t *code) {
	// TODO: fix up this function properly
	CHECK_NULL(code);
	if(channel >= LGW_CHAN_NB_MAX) {
		DEBUG_MSG("ERROR: INVALID RF/IF_CHAIN, CANNOT GET ITS STATUS\n");
		return LGW_HAL_ERROR;
	}

	if(rf_enable[channel] == false || rf_tx_enable[channel] == false) {
		*code = TX_OFF;
		return LGW_HAL_SUCCESS;
	} else {
		if(rf_status[channel] == LGW_STATUS_TXING) {
			*code = TX_EMITTING;
		} else {
			*code = TX_FREE;
		}
		return LGW_HAL_SUCCESS;
	}

	return LGW_HAL_ERROR;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_abort_tx(int channel) {
	// we don't have abort_tx function
	opmode(channel, OPMODE_SLEEP);
	return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_trigcnt(uint32_t* trig_cnt_us) {
	// TODO: linjinzhi: we are not using sx1301, thus we can't implemented this. We just use the system time as concentrator count_us for now
	struct timeval now;
	gettimeofday(&now, NULL);
	(*trig_cnt_us) = (uint32_t)(((uint64_t)now.tv_sec)*1000000UL + now.tv_usec);
	return LGW_HAL_SUCCESS;

}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

const char* lgw_version_info() {
	return lgw_version_string;
}


/* --- EOF ------------------------------------------------------------------ */
