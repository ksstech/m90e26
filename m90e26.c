/*
 * Copyright 2018-20 AM Maree/KSS Technologies (Pty) Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/*
 * m90e26.c
 */

#include	"hal_config.h"

#if		(halHAS_M90E26 > 0)

#include	"FreeRTOS_Support.h"
#include	"m90e26.h"
#include	"m90e26_cmds.h"
#if		(halHAS_SSD1306 > 0)
	#include	"ssd1306.h"
#endif
#include	"endpoint_id.h"

#include	"rules_engine.h"
#include	"x_errors_events.h"
#include	"systiming.h"					// timing debugging
#include	"syslog.h"
#include	"printfx.h"
#include	"x_values_convert.h"
#include	"x_string_to_values.h"

#include	"hal_spi.h"
#include	"hal_storage.h"
#include	"hal_gpio.h"
#include	"hal_variables.h"

#include	<string.h>

#define	debugFLAG					0xC400

#define	debugREAD					(debugFLAG & 0x0001)
#define	debugWRITE					(debugFLAG & 0x0002)
#define	debugRMW					(debugFLAG & 0x0004)
#define	debugINIT					(debugFLAG & 0x0008)

#define	debugMODE					(debugFLAG & 0x0010)
#define	debugCURRENT				(debugFLAG & 0x0020)
#define	debugENERGY					(debugFLAG & 0x0040)
#define	debugFACTOR					(debugFLAG & 0x0080)

#define	debugANGLE					(debugFLAG & 0x0100)
#define	debugPOWER					(debugFLAG & 0x0200)
#define	debugOFFSET					(debugFLAG & 0x0400)
#define	debugCONTRAST				(debugFLAG & 0x0800)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ###################################### Private variables #######################################

spi_device_interface_config_t	m90e26_config[halHAS_M90E26] = {
#if		(halHAS_M90E26 > 0)
	[0] = {
		.command_bits		= 0,
		.address_bits		= 0,
		.dummy_bits			= 0,
		.mode				= 3,						// only SPI mode 3 supported
		.duty_cycle_pos		= 128,						// same as 0 = 50/50% duty cycle
		.cs_ena_pretrans 	= 0,
		.cs_ena_posttrans	= 0,
		.clock_speed_hz		= 160000,
		.input_delay_ns		= 0,
		.spics_io_num		= m90e26VSPI_CS1,
		.flags				= 0,
		.queue_size			= 16,
		.pre_cb				= 0,						// no callback handler
		.post_cb			= 0,
	},
#endif
#if		(halHAS_M90E26 > 1)
	[1] = {
		.command_bits		= 0,
		.address_bits		= 0,
		.dummy_bits			= 0,
		.mode				= 3,
		.duty_cycle_pos		= 128,						// same as 0 = 50/50% duty cycle
		.cs_ena_pretrans 	= 0,
		.cs_ena_posttrans	= 0,
		.clock_speed_hz		= 160000,
		.input_delay_ns		= 0,
		.spics_io_num		= m90e26VSPI_CS2,
		.flags				= 0,
		.queue_size			= 16,
		.pre_cb				= 0,						// no callback handler
		.post_cb			= 0,
	},
#endif
} ;

spi_device_handle_t				m90e26_handle[halHAS_M90E26] = { 0 } ;
SemaphoreHandle_t				m90e26mutex[halHAS_M90E26] = { 0 } ;

struct {
	uint8_t	tBlank ;									// # seconds to blank in between
	uint8_t MinContrast ;
	uint8_t	MaxContrast ;
	uint8_t	NowContrast ;
	struct {
		uint8_t	E_Scale	: 1 ;							// 0 = WHr,	1 = KwHr
		uint8_t	P_Scale	: 1 ;							// 0 = W,	1 = Kw
		uint8_t	I_Scale	: 1 ;							// 0 = A,	1 = KwHr
		uint8_t	Display	: 2 ;							// 0 = Off, 1 = Normal,	2 = Current, 3 = Button
		uint8_t	Spare	: 3 ;
		uint8_t	N_Gain	: 3 ;							// 1 -> 4
		uint8_t	L_Gain	: 5 ;							// 1 -> 24
	} Chan[halHAS_M90E26] ;
} m90e26Config = { 0 } ;

const uint8_t	m90e26RegAddr[] = {
	E_ACT_FWD,	E_ACT_REV,	E_ACT_ABS,					// Active energy
	E_REACT_FWD,E_REACT_REV,E_REACT_ABS,				// ReActive energy
	I_RMS_L,	V_RMS,		P_ACT_L,	P_REACT_L,		// Voltage, Freq & LIVE Power info
	FREQ,		P_FACTOR_L,	P_ANGLE_L,	P_APP_L,
#if		(m90e26NEUTRAL == 1)
	I_RMS_N,	P_ACT_N,	P_REACT_N,					// NEUTRAL Power info
	P_FACTOR_N,	P_ANGLE_N,	P_APP_N,
#endif
} ;

/*		FACTORY default (power on or soft reset)
 * 		########################################
 *	###	PLconsH	PLconsL	L_GAIN	L_PHI	N_GAIN	N_PHI	P_SupTH	P_NolTH	Q_SupTH	Q_NolTH	MetMODE
 *		0x0015, 0xD174, 0x0000, 0x0000, 0x0000, 0x0000, 0x08BD, 0x0000, 0x0AEC, 0x0000, 0x9422
 *	###	V_Gain	IgainL	IgainN	Voffset	IofstL	IofstN	PofstL	QofstL	PofstN	QofstN
 *		0x6720, 0x7A13, 0x7530, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
 *	###	FUNC_EN	SAG_TH	PWRMODE
 *		0x000C, 0x1D64, 0x0000
 *		METMODE = 9422	Lgain=1, Ngain=1, LNSel=1(L), DisHPF=EnableHPF0+1
 *						Amod=Fwd+Rev Energy Pulse, Rmod=Fwd+Rev Energy Pulse
 *						 Zxcon=All(pos+neg), Pthresh=3.125%
 * 	Tishan:
 *		0x00B9, 0xC1F3, 0xD139, 0x0000, 0x0000, 0x0000, 0x08BD, 0x0000, 0x0AEC, 0x0000, 0x9422 },
 *		0xD464, 0x6E49, 0x7530, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
 */
nvs_m90e26_t	nvsM90E26default[halHAS_M90E26] = {
#if		(halHAS_M90E26 > 0)
	{ {	0x00B9, 0xC1F3, 0x0000, 0x0000, 0x0000, 0x0000, 0x08BD, 0x8100, 0x0AEC, 0x8100, 0x9422 },
	  {	0x6C50, 0x7C2A, 0x7530, 0x0000, 0xF711, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
	  {	0x0030, 0x1F2F, 0x0000	}, },
#endif
#if		(halHAS_M90E26 > 1)
	{ {	0x00B9, 0xC1F3, 0x0000, 0x0000, 0x0000, 0x0000, 0x08BD, 0x8100, 0x0AEC, 0x8100, 0x9422 },
	  {	0x6C50, 0x7C2A, 0x7530, 0x0000, 0xF800, 0xF400, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF },
  	  {	0x0030, 0x1F2F, 0x0000	}, },
#endif
} ;

// ############################### common support routines #########################################

void	m90e26WriteU16(uint8_t eChan, uint8_t address, uint16_t val) {
	IF_myASSERT(debugPARAM, eChan < halHAS_M90E26 && address < 0x70 && m90e26_handle[eChan]) ;
	xRtosSemaphoreTake(&m90e26mutex[eChan], portMAX_DELAY) ;
	spi_transaction_t m90e26_buf ;
	memset(&m90e26_buf, 0, sizeof(m90e26_buf));
	m90e26_buf.length		= 8 * 3;
	m90e26_buf.flags 		= SPI_TRANS_USE_TXDATA ;
	m90e26_buf.tx_data[0]	= address ;
	m90e26_buf.tx_data[1]	= val >> 8 ;
	m90e26_buf.tx_data[2]	= val & 0xFF ;
	ESP_ERROR_CHECK(spi_device_transmit(m90e26_handle[eChan], &m90e26_buf)) ;
	xRtosSemaphoreGive(&m90e26mutex[eChan]) ;
	IF_PRINT(debugWRITE, "TX: addr=%02x d0=%02x d1=%02x\n", m90e26_buf.tx_data[0], m90e26_buf.tx_data[1], m90e26_buf.tx_data[2]) ;
}

uint16_t m90e26ReadU16(uint8_t eChan, uint8_t address) {
	IF_myASSERT(debugPARAM, eChan < halHAS_M90E26 && address < 0x70 && m90e26_handle[eChan]) ;
	xRtosSemaphoreTake(&m90e26mutex[eChan], portMAX_DELAY) ;
	spi_transaction_t m90e26_buf ;
	memset(&m90e26_buf, 0, sizeof(m90e26_buf)) ;
	m90e26_buf.length		= 8 * 3 ;
	m90e26_buf.flags 		= SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA ;
	m90e26_buf.tx_data[0]	= address | 0x80 ;
	ESP_ERROR_CHECK(spi_device_transmit(m90e26_handle[eChan], &m90e26_buf)) ;
	xRtosSemaphoreGive(&m90e26mutex[eChan]) ;
	IF_PRINT(debugREAD, "RX: addr=%02x  d0=%02x  d1=%02x  dx=%04x\n", m90e26_buf.tx_data[0], m90e26_buf.rx_data[1], m90e26_buf.rx_data[2], (m90e26_buf.rx_data[1] << 8) | m90e26_buf.rx_data[2]) ;
	return (m90e26_buf.rx_data[1] << 8) | m90e26_buf.rx_data[2] ;
}

void	m90e26WriteRegister(uint8_t eChan, uint8_t Reg, uint16_t Val) {
	IF_myASSERT(debugPARAM, eChan < halHAS_M90E26) ;
	if (INRANGE(PLconstH, Reg, MET_MODE, uint8_t)) {
		if (m90e26ReadU16(eChan, CALSTART) != CODE_START) {
			SL_INFO("CALSTART (x20) in wrong state, must be x5678") ;
		} else {
			m90e26WriteU16(eChan, Reg, Val) ;				// write new value & update CRC
			m90e26WriteU16(eChan, CRC_1, m90e26ReadU16(eChan, CRC_1)) ;
			IF_PRINT(debugTRACK, "Before: #%d %-'h\n", Reg-PLconstH, SIZEOF_MEMBER(nvs_m90e26_t, calreg), &nvsM90E26default[eChan].calreg) ;
			nvsM90E26default[eChan].calreg[Reg-PLconstH] = Val ;
			IF_PRINT(debugTRACK, "After : #%d %-'h\n", Reg-PLconstH, SIZEOF_MEMBER(nvs_m90e26_t, calreg), &nvsM90E26default[eChan].calreg) ;
		}
	} else if (INRANGE(U_GAIN, Reg, Q_OFST_N, uint8_t)) {
		if (m90e26ReadU16(eChan, ADJSTART) != CODE_START) {
			SL_INFO("ADJSTART (x30) in wrong state, must be x5678") ;
		} else {
			m90e26WriteU16(eChan, Reg, Val) ;				// write new value & update CRC
			m90e26WriteU16(eChan, CRC_2, m90e26ReadU16(eChan, CRC_2)) ;
			IF_PRINT(debugTRACK, "Before: #%d %-'h\n", Reg-U_GAIN, SIZEOF_MEMBER(nvs_m90e26_t, adjreg), &nvsM90E26default[eChan].adjreg) ;
			nvsM90E26default[eChan].adjreg[Reg-U_GAIN] = Val ;
			IF_PRINT(debugTRACK, "After : #%d %-'h\n", Reg-U_GAIN, SIZEOF_MEMBER(nvs_m90e26_t, adjreg), &nvsM90E26default[eChan].adjreg) ;
		}
	} else if (Reg == SOFTRESET || INRANGE(FUNC_ENAB, Reg, POWER_MODE, uint8_t) || Reg == CALSTART || Reg == ADJSTART) {
		m90e26WriteU16(eChan, Reg, Val) ;				// write new value
		if (INRANGE(FUNC_ENAB, Reg, POWER_MODE, uint8_t)) {
			IF_PRINT(debugTRACK, "Before: #%d %-'h\n", Reg-FUNC_ENAB, SIZEOF_MEMBER(nvs_m90e26_t, cfgreg), &nvsM90E26default[eChan].cfgreg) ;
			nvsM90E26default[eChan].cfgreg[Reg-FUNC_ENAB] = Val ;
			IF_PRINT(debugTRACK, "After : #%d %-'h\n", Reg-FUNC_ENAB, SIZEOF_MEMBER(nvs_m90e26_t, cfgreg), &nvsM90E26default[eChan].cfgreg) ;
		}
	} else {
		SL_ERR("Invalid register=0x%02X", Reg) ;
	}
}

uint16_t m90e26ReadModifyWrite(uint8_t eChan, uint8_t Addr, uint16_t Value, uint16_t Mask) {
	IF_PRINT(debugRMW, "  C=%d  R=%d  &=x%04X  |=x%04X", eChan, Addr, Value, Mask) ;
	uint16_t CurValue = m90e26ReadU16(eChan, Addr) ;
	IF_PRINT(debugRMW, "  V=x%04X", CurValue) ;
	CurValue &= ~Mask ;
	IF_PRINT(debugRMW, " -> x%04X", CurValue) ;
	CurValue |= Value ;
	IF_PRINT(debugRMW, " -> x%04X\n", CurValue) ;
	m90e26WriteRegister(eChan, Addr, CurValue) ;
	return CurValue ;
}

int16_t m90e26ReadI16S(uint8_t eChan, uint8_t Reg) {
	uint16_t RawVal = m90e26ReadU16(eChan, Reg) ;
	bool	Sign	= RawVal & 0x8000 ? true : false ;
	int16_t	i16Val	= RawVal & 0x7FFF ;
	int16_t	ConVal	= Sign == true ? -1 * i16Val : i16Val ;
	return ConVal ;
}

int16_t m90e26ReadI16TC(uint8_t eChan, uint8_t Reg) { return ~m90e26ReadU16(eChan, Reg) + 1 ; }

uint32_t m90e26ReadU32(uint8_t eChan, uint8_t Reg) {
#if		(m90e26READ_32BIT == 1)
	return (m90e26ReadU16(eChan, Reg) << 16) + m90e26ReadU16(eChan, LSB) ;
#else
	return (m90e26ReadU16(eChan, Reg) << 16) ;
#endif
}

int32_t m90e26ReadI32TC(uint8_t eChan, uint8_t Reg) { return ~m90e26ReadU32(eChan, Reg) + 1 ; }

int32_t	m90e26LoadNVSConfig(uint8_t eChan, uint8_t Idx) {
	IF_myASSERT(debugPARAM, eChan < halHAS_M90E26 && Idx < CALIB_NUM) ;
	size_t	SizeBlob = CALIB_NUM * sizeof(nvs_m90e26_t) ;
	nvs_m90e26_t * psCalib = malloc(SizeBlob) ;
	int32_t iRV = halSTORAGE_ReadBlob(halSTORAGE_STORE, halSTORAGE_KEY_M90E26, psCalib, &SizeBlob) ;
	if (iRV == erSUCCESS) {
		psCalib += Idx ;								// write the FuncEnab, Vsag Threshold and PowerMode registers
		for (int32_t i = 0; i < NUM_OF_MEM_ELEM(nvs_m90e26_t, cfgreg); m90e26WriteU16(eChan, i+FUNC_ENAB, psCalib->cfgreg[i]), ++i) ;

		m90e26WriteU16(eChan, CALSTART, CODE_START) ;		// write the configuration registers with METER calibration data
		for (int32_t i = 0; i < NUM_OF_MEM_ELEM(nvs_m90e26_t, calreg); m90e26WriteRegister(eChan, i+PLconstH, psCalib->calreg[i]), ++i) ;
		IF_EXEC_3(configPRODUCTION == 1, m90e26WriteU16, eChan, CALSTART, CODE_CHECK) ;

		m90e26WriteU16(eChan, ADJSTART, CODE_START) ;		// write the configuration registers with MEASURE calibration data
		for (int32_t i = 0; i < NUM_OF_MEM_ELEM(nvs_m90e26_t, adjreg); m90e26WriteRegister(eChan, i+U_GAIN, psCalib->adjreg[i]), ++i) ;
		IF_EXEC_3(configPRODUCTION == 1, m90e26WriteU16, eChan, ADJSTART, CODE_CHECK) ;
	} else {
		SL_ERR("Failed Ch %d config %d '%s' (%x)", eChan, Idx, esp_err_to_name(iRV), iRV) ;
	}
	free (psCalib) ;
	return iRV ;
}

void	CmndM90_WriteChannels(uint8_t eChan, uint8_t Reg, uint16_t Value) {
	uint8_t	Cnow = eChan < halHAS_M90E26 ? eChan : 0 ;
	do { m90e26WriteRegister(eChan, Reg, Value); } while (++Cnow < eChan) ;
}

uint8_t	m90e26CalcInfo(ep_work_t * psEpWork) {
	xEpWorkToUri(psEpWork) ;
	psEpWork->idx = psEpWork->uri - URI_M90E26_E_ACT_FWD_0 ;
	psEpWork->eChan = 0 ;
	// In case of multiple channels, adjust till in range....
	while (psEpWork->idx >= M90E26_NUMURI_0) {
		psEpWork->idx -= M90E26_NUMURI_0 ;
		psEpWork->eChan++ ;
	}
	IF_myASSERT(debugRESULT, (psEpWork->idx < M90E26_NUMURI_0) && (psEpWork->eChan < halHAS_M90E26)) ;
	return psEpWork->idx ;
}

/* ######################################## Calibration ############################################
 * Overall process documented in the following Application Note:
 * http://ww1.microchip.com/downloads/en/AppNotes/Atmel-46102-SE-M90E26-ApplicationNote.pdf */

/**
 * m90e26SetPowerOffset() -
 * @param	eChan
 * @param	RegPower
 * @param	RegOFST
 * [Re]Active, LINE & NEUTRAL Power Offset calibration
 * Not sure if the power register should be read whilst in normal or
 * adjustment mode. Currently no real value is being read
 */
void	m90e26PowerOffsetCalcSet(uint8_t eChan, uint8_t RegPOWER, uint8_t RegOFST) {
	m90e26WriteU16(eChan, POWER_MODE, CODE_POWER) ;		// set into low power mode for calibration
#if 	(m90e26CALIB_32BIT == 1)
	uint64_t SumOffset = 0 ;
	for (int32_t i = 0; i < m90e26CALIB_ITER; ++i) {
		uint32_t CurVal = m90e26ReadU32(eChan, RegPOWER) ;
		SumOffset += CurVal ;
		IF_PRINT(debugOFFSET, "#%d=0x%04X  ", i, CurVal) ;
	}
	uint32_t NewAVG	= SumOffset / m90e26CALIB_ITER ;
	uint32_t NewOffset = (~NewAVG + 1) >> 16 ;
	IF_PRINT(debugOFFSET, "\nCh %d:  S=0x%010llX  A=0x%08X  O=0x%08X\n", eChan, SumOffset, NewAVG, NewOffset) ;

#elif 0
	uint32_t SumOffset = 0 ;
	for (int32_t i = 0; i < m90e26CALIB_ITER; ++i) {
		uint32_t CurVal = m90e26ReadU16(eChan, RegPOWER) ;
		SumOffset += CurVal ;
		IF_PRINT(debugOFFSET, "#%d=0x%04X  ", i, CurVal) ;
	}
	uint32_t NewAVG	= SumOffset / m90e26CALIB_ITER ;
	uint32_t NewOffset = ~NewAVG + 1;
	IF_PRINT(debugOFFSET, "\nCh %d:  S=0x%08X  A=0x%08X  O=0x%08X\n", eChan, SumOffset, NewAVG, NewOffset) ;

#elif 0
	int32_t SumOffset = 0 ;
	for (int32_t i = 0; i < m90e26CALIB_ITER; ++i) {
		int32_t CurVal = m90e26ReadI16S(eChan, RegPOWER) ;
		SumOffset += CurVal ;
		IF_PRINT(debugOFFSET, "#%d=%d  ", i, CurVal) ;
	}
	int16_t NewAVG	= SumOffset / m90e26CALIB_ITER ;
	int16_t NewOffset = ~NewAVG + 1;
	IF_PRINT(debugOFFSET, "\nCh %d: S=%d  A=%d  O=0x%08X\n", eChan, SumOffset, NewAVG, NewOffset) ;

#endif
	m90e26WriteRegister(eChan, RegOFST, NewOffset) ;
	m90e26WriteU16(eChan, POWER_MODE, CODE_RESET) ;		// reset to normal power mode
}

void	m90e26CurrentOffsetCalcSet(uint8_t eChan, uint8_t RegRMS, uint8_t RegGAIN, uint8_t RegOFST) {
	uint32_t CurAmps = m90e26ReadU16(eChan, RegRMS) ;
	uint32_t CurGain = m90e26ReadU16(eChan, RegGAIN) ;
	uint32_t Factor1 = CurAmps * CurGain >> 8 ;
	uint32_t Factor2 = ~Factor1 + 1 ;
	m90e26WriteRegister(eChan, RegOFST, Factor2) ;
	IF_PRINT(debugOFFSET, "Ch %d %s: Icur=0x%04X  Gcur=0x%04X  F1=0x%08X  F2=0x%08X\n",
			eChan, RegRMS == I_RMS_L ? "Live" : "Neutral", CurAmps, CurGain, Factor1, Factor2) ;
}

/**
 * CmndM90C() Write value to M90E26 configuration (CALibration or ADJustment) register
 * @brief	M90C {eChan = 0->2} {regnum = 0->0x3B} {value}
 * @return
 */
int32_t CmndM90C(cli_t * psCLI) {
	uint8_t		Chan, Reg ;
	uint16_t	Value ;
	char * pTmp = pcStringParseValueRange(psCLI->pcParse, (p32_t) &Chan, vfUXX, vs08B, sepSPACE, (x32_t) 0, (x32_t) halHAS_M90E26) ;
	EQ_GOTO(pTmp, pcFAILURE, exit) ;

	// Allow all config registers
	pTmp = pcStringParseValueRange(psCLI->pcParse = pTmp, (p32_t) &Reg, vfUXX, vs08B, sepSPACE, (x32_t) SOFTRESET, (x32_t) CRC_2) ;
	EQ_GOTO(pTmp, pcFAILURE, exit) ;

	// value to be written
	pTmp = pcStringParseValueRange(psCLI->pcParse = pTmp, (p32_t) &Value, vfUXX, vs16B, sepSPACE, (x32_t) 0x0, (x32_t) 0xFFFF) ;
	EQ_GOTO(pTmp, pcFAILURE, exit) ;

	CmndM90_WriteChannels(Chan, Reg, Value) ;
	psCLI->pcParse = pTmp ;
	return erSUCCESS ;
exit:
	return erFAILURE ;
}

int32_t CmndM90D(cli_t * psCLI) { halSTORAGE_DeleteKeyValue(halSTORAGE_STORE, halSTORAGE_KEY_M90E26) ; return erSUCCESS ; }

/**
 * CmndM90L() - set Live gain
 * @param	pCmdBuf
 * @return	NULL in successful
 * 			pointer to location in buffer where parsing error occurred.
 */
int32_t CmndM90L(cli_t * psCLI) {
	uint8_t	eChan, Value ;
	char * pTmp = pcStringParseValueRange(psCLI->pcParse, (p32_t) &eChan, vfUXX, vs08B, sepSPACE, (x32_t) 0, (x32_t) halHAS_M90E26) ;
	EQ_GOTO(pTmp, pcFAILURE, exit) ;

	pTmp = pcStringParseValueRange(psCLI->pcParse = pTmp, (p32_t) &Value, vfUXX, vs08B, sepSPACE, (x32_t) 1, (x32_t) 24) ;
	EQ_GOTO(pTmp, pcFAILURE, exit) ;

	uint8_t	Cnow = eChan < halHAS_M90E26 ? eChan : 0 ;
	do { m90e26SetLiveGain(eChan, Value) ; } while (++Cnow < eChan) ;
	psCLI->pcParse = pTmp ;
	return erSUCCESS ;
exit:
	return erFAILURE ;
}

/**
 * CmndM90N() - set Neutral gain
 * @param	pCmdBuf
 * @return	NULL in successful
 * 			pointer to location in buffer where parsing error occurred.
 */
int32_t CmndM90N(cli_t * psCLI) {
	uint8_t	eChan, Value ;
	char * pTmp = pcStringParseValueRange(psCLI->pcParse, (p32_t) &eChan, vfUXX, vs08B, sepSPACE, (x32_t) 0, (x32_t) halHAS_M90E26) ;
	EQ_GOTO(pTmp, pcFAILURE, exit) ;

	pTmp = pcStringParseValueRange(psCLI->pcParse = pTmp, (p32_t) &Value, vfUXX, vs08B, sepSPACE, (x32_t) 1, (x32_t) 4) ;
	EQ_GOTO(pTmp, pcFAILURE, exit) ;

	uint8_t	Cnow = eChan < halHAS_M90E26 ? eChan : 0 ;
	do { m90e26SetNeutralGain(eChan, Value) ; } while (++Cnow < eChan) ;
	psCLI->pcParse = pTmp ;
	return erSUCCESS ;
exit:
	return erFAILURE ;
}

int32_t CmndM90O(cli_t * psCLI) {
	uint8_t	eChan ;
	char * pTmp = pcStringParseValueRange(psCLI->pcParse, (p32_t) &eChan, vfUXX, vs08B, sepSPACE, (x32_t) 0, (x32_t) halHAS_M90E26) ;
	EQ_GOTO(pTmp, pcFAILURE, exit) ;

	uint8_t	Cnow = eChan < halHAS_M90E26 ? eChan : 0 ;
	do {
		m90e26CurrentOffsetCalcSet(Cnow, I_RMS_L, I_GAIN_L, I_OFST_L) ;
#if		(m90e26NEUTRAL == 1)
		m90e26CurrentOffsetCalcSet(Cnow, I_RMS_N, I_GAIN_N, I_OFST_N) ;
#endif
	} while (++Cnow < eChan) ;
	psCLI->pcParse = pTmp ;
	return erSUCCESS ;
exit:
	return erFAILURE ;
}

int32_t CmndM90P(cli_t * psCLI) {
	uint8_t	eChan ;
	char * pTmp = pcStringParseValueRange(psCLI->pcParse, (p32_t) &eChan, vfUXX, vs08B, sepSPACE, (x32_t) 0, (x32_t) halHAS_M90E26) ;
	EQ_GOTO(pTmp, pcFAILURE, exit) ;

	uint8_t Cnow = eChan < halHAS_M90E26 ? eChan : 0 ;
	do {
		m90e26PowerOffsetCalcSet(Cnow, P_ACT_L, P_OFST_L) ;
		m90e26PowerOffsetCalcSet(Cnow, P_REACT_L, Q_OFST_L) ;
#if		(m90e26NEUTRAL == 1)
		m90e26PowerOffsetCalcSet(Cnow, P_ACT_N, P_OFST_N) ;
		m90e26PowerOffsetCalcSet(Cnow, P_REACT_N, Q_OFST_N) ;
#endif
	} while (++Cnow < eChan) ;
	psCLI->pcParse = pTmp ;
	return erSUCCESS ;
exit:
	return erFAILURE ;
}

/**
 * CmndM90S() - save current configuration (CAL & ADJ) registers to specific blob array position.
 * @param	pCmdBuf {Chan = 0->1} {Index = 0->X}
 * @return	NULL in successful
 * 			pointer to location in buffer where parsing error occurred.
 */
int32_t CmndM90S(cli_t * psCLI) {
	uint8_t	Chan, Value ;
	char * pTmp = pcStringParseValueRange(psCLI->pcParse, (p32_t) &Chan, vfUXX, vs08B, sepSPACE, (x32_t) 0, (x32_t) (halHAS_M90E26 - 1)) ;
	EQ_GOTO(pTmp, pcFAILURE, exit) ;

	pTmp = pcStringParseValueRange(psCLI->pcParse = pTmp, (p32_t) &Value, vfUXX, vs08B, sepSPACE, (x32_t) 0, (x32_t) (CALIB_NUM - 1)) ;
	EQ_GOTO(pTmp, pcFAILURE, exit) ;

	size_t	SizeBlob = CALIB_NUM * sizeof(nvs_m90e26_t) ;
	nvs_m90e26_t * psCalib = malloc(SizeBlob) ;
	memset(psCalib, 0, SizeBlob) ;
	int32_t iRV = halSTORAGE_ReadBlob(halSTORAGE_STORE, halSTORAGE_KEY_M90E26, psCalib, &SizeBlob) ;
	IF_SL_NOT(debugRESULT && iRV != erSUCCESS, "Error reading M90E26blob, starting with blank") ;

	nvs_m90e26_t * psTemp = psCalib + Value ;
	for (int32_t i = 0; i < NUM_OF_MEM_ELEM(nvs_m90e26_t, calreg); psTemp->calreg[i] = m90e26ReadU16(Chan, i+PLconstH), ++i) ;
	for (int32_t i = 0; i < NUM_OF_MEM_ELEM(nvs_m90e26_t, adjreg); psTemp->adjreg[i] = m90e26ReadU16(Chan, i+U_GAIN), ++i) ;
	for (int32_t i = 0; i < NUM_OF_MEM_ELEM(nvs_m90e26_t, cfgreg); psTemp->cfgreg[i] = m90e26ReadU16(Chan, i+FUNC_ENAB), ++i) ;

	iRV = halSTORAGE_WriteBlob(halSTORAGE_STORE, halSTORAGE_KEY_M90E26, psCalib, CALIB_NUM * sizeof(nvs_m90e26_t)) ;
	IF_myASSERT(debugRESULT, iRV == erSUCCESS) ;
	free(psCalib) ;
	psCLI->pcParse = pTmp ;
	return erSUCCESS ;
exit:
	return erFAILURE ;
}

int32_t CmndM90Z(cli_t * psCLI) {
	uint8_t	Chan ;
	char * pTmp = pcStringParseValueRange(psCLI->pcParse, (p32_t) &Chan, vfUXX, vs08B, sepSPACE, (x32_t) 0, (x32_t) halHAS_M90E26) ;
	EQ_GOTO(pTmp, pcFAILURE, exit) ;
	CmndM90_WriteChannels(Chan, SOFTRESET, CODE_RESET) ;
	psCLI->pcParse = pTmp ;
	return erSUCCESS ;
exit:
	return erFAILURE ;
}

// ############################## identification & initialization ##################################

int32_t	m90e26Identify(uint8_t eChan) {
	IF_myASSERT(debugPARAM, eChan < halHAS_M90E26) ;
	ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &m90e26_config[eChan], &m90e26_handle[eChan])) ;
	m90e26mutex[eChan]	= xSemaphoreCreateMutex() ;
	IF_myASSERT(debugRESULT, m90e26mutex[eChan]) ;
	return erSUCCESS ;
}

/**
 * m90e26Init() -
 */
int32_t	m90e26Init(uint8_t eChan) {
	IF_myASSERT(debugPARAM, eChan < halHAS_M90E26) ;
	/* Check that blob with CALibration and ADJustment values exists
	 * If not existing, create with factory defaults as first record */
	size_t	SizeBlob = CALIB_NUM * sizeof(nvs_m90e26_t) ;
	nvs_m90e26_t * psCalib = malloc(SizeBlob) ;
	int32_t iRV = halSTORAGE_ReadBlob(halSTORAGE_STORE, halSTORAGE_KEY_M90E26, psCalib, &SizeBlob) ;
	if (iRV != erSUCCESS || SizeBlob != CALIB_NUM * sizeof(nvs_m90e26_t)) {
		memset(psCalib, 0, SizeBlob = CALIB_NUM * sizeof(nvs_m90e26_t)) ;
		memcpy(psCalib, &nvsM90E26default, sizeof(nvsM90E26default)) ;
		iRV = halSTORAGE_WriteBlob(halSTORAGE_STORE, halSTORAGE_KEY_M90E26, psCalib, SizeBlob) ;
		SL_WARN("NVS defaults create %s", iRV == erSUCCESS ? "Success" : "Failed") ;
		IF_myASSERT(debugRESULT, iRV == erSUCCESS) ;
	}
	free(psCalib) ;

	m90e26WriteU16(eChan, SOFTRESET, CODE_RESET) ;		// start with default values, not running, no valid values
	m90e26LoadNVSConfig(eChan, 0) ;						// load config #0 from NVS blob as default
	m90e26Config.Chan[eChan].L_Gain		= 1 ;			// set default state
#if	(m90e26NEUTRAL == 1)
	m90e26Config.Chan[eChan].N_Gain		= 1 ;
#endif
	m90e26Config.Chan[eChan].E_Scale	= 0 ;			// Wh not kWh
	m90e26Config.Chan[eChan].P_Scale	= 0 ;			// W not kW
	m90e26Config.Chan[eChan].I_Scale	= 0 ;			// A not mA
	m90e26Config.MaxContrast			= 255 ;
	return (m90e26GetSysStatus(eChan) & 0xF000) ? erFAILURE : erSUCCESS ;
}

// ########################### 32 bit value endpoint support functions #############################

int32_t	m90e26ReadCurrent(ep_work_t * psEpWork) {
	uint8_t	eIdx = m90e26CalcInfo(psEpWork) ;
	float	f32Val	= (float) m90e26ReadU32(psEpWork->eChan, m90e26RegAddr[eIdx]) ;
	IF_PRINT(debugCURRENT, "Irms: URI=%d  Idx=%d  Reg=%02X  Ch=%d", psEpWork->uri, eIdx, m90e26RegAddr[eIdx], psEpWork->eChan) ;
	if (m90e26Config.Chan[psEpWork->eChan].I_Scale == 0) {
		f32Val	/= 65536000.0 ;							// convert to Amp
		IF_PRINT(debugCURRENT, "  Val=%4.5fA", f32Val) ;
	} else {
		f32Val	/= 65536.0 ;							// convert to mA
		IF_PRINT(debugCURRENT, "  Val=%4.5fmA", f32Val) ;
	}

#if		(m90e26NEUTRAL == 1)
	IF_myASSERT(debugRESULT, eIdx == eI_RMS_L || eIdx == eI_RMS_N) ;
	if (eIdx == eI_RMS_L) {
		f32Val /= m90e26Config.Chan[psEpWork->eChan].L_Gain ;
		IF_PRINT(debugCURRENT, "  Lgain=%d", m90e26Config.Chan[psEpWork->eChan].L_Gain) ;
	} else {
		f32Val /= m90e26Config.Chan[psEpWork->eChan].N_Gain ;
		IF_PRINT(debugCURRENT, "  Ngain=%d", m90e26Config.Chan[psEpWork->eChan].N_Gain) ;
	}

#else
	IF_myASSERT(debugRESULT, eIdx == eI_RMS_L) ;
	f32Val /= m90e26Config.eChan[psEpWork->eChan].L_Gain ;
	IF_PRINT(debugCURRENT, "  Lgain=%d", m90e26Config.eChan[psEpWork->eChan].L_Gain) ;

#endif
	IF_PRINT(debugCURRENT, "  Act=%4.5f\n", f32Val) ;
	xEpSetValue(psEpWork, (x32_t) f32Val) ;
	return erSUCCESS ;
}

int32_t	m90e26ReadVoltage(ep_work_t * psEpWork) {		// OK
	m90e26CalcInfo(psEpWork) ;
	float	f32Val	= (float) m90e26ReadU32(psEpWork->eChan, m90e26RegAddr[psEpWork->idx]) ;
	f32Val			/= 6553600.0 ;						// Change mV to V
	xEpSetValue(psEpWork, (x32_t) f32Val) ;
//	IF_PRINT(debugVOLTS, "Vrms: Ch=%d  Val=%9.3f\n", psEpWork->eChan, f32Val) ;
	return erSUCCESS ;
}

int32_t	m90e26ReadPower(ep_work_t * psEpWork) {
	m90e26CalcInfo(psEpWork) ;
	float	f32Val	= (float) m90e26ReadI32TC(psEpWork->eChan, m90e26RegAddr[psEpWork->idx]) ;
	if (m90e26Config.Chan[psEpWork->eChan].P_Scale == 1) {
		f32Val	/= 65536000.0 ;							// make KWh (alt range)
	} else {
		f32Val	/= 65536.0 ;							// make Wh (default)
	}
	xEpSetValue(psEpWork, (x32_t) f32Val) ;
	IF_PRINT(debugPOWER, "Power: Ch=%d  Reg=0x%02X  Val=%9.3f\n", psEpWork->eChan, m90e26RegAddr[psEpWork->idx], f32Val) ;
	return erSUCCESS ;
}

// ########################### 16 bit value endpoint support functions #############################

int32_t	m90e26ReadEnergy(ep_work_t * psEpWork) {
	if (psEpWork->Var.varDef.cv.sumX) {					// if just a normal update cycle
		m90e26CalcInfo(psEpWork) ;
		float f32Val	= (float) m90e26ReadU16(psEpWork->eChan, m90e26RegAddr[psEpWork->idx]) ;
		f32Val	/= m90e26Config.Chan[psEpWork->eChan].E_Scale ? 10000.0 : 10.0 ;
		xEpSetValue(psEpWork, (x32_t) f32Val) ;
		// Update running total in NVS memory
		sRTCvars.aRTCsum[psEpWork->eChan][psEpWork->idx] += f32Val ;
	} else {											// else it is a value reset call
		vCompVarResetValue(&psEpWork->Var) ;
		IF_PRINT(debugENERGY, "Energy: Sum RESET\n") ;
	}
	return erSUCCESS ;
}

int32_t	m90e26ReadFrequency(ep_work_t * psEpWork) {
	m90e26CalcInfo(psEpWork) ;
	float f32Val	= (float) m90e26ReadU16(psEpWork->eChan, m90e26RegAddr[psEpWork->idx]) / 100.0 ;
	xEpSetValue(psEpWork, (x32_t) f32Val) ;
	return erSUCCESS ;
}

int32_t	m90e26ReadPowerFactor(ep_work_t * psEpWork) {
	m90e26CalcInfo(psEpWork) ;
	float f32Val	= (float)  m90e26ReadI16S(psEpWork->eChan, m90e26RegAddr[psEpWork->idx]) / 1000.0 ;
	xEpSetValue(psEpWork, (x32_t) f32Val) ;
	return erSUCCESS ;
}

int32_t	m90e26ReadPowerAngle(ep_work_t * psEpWork) {
	m90e26CalcInfo(psEpWork) ;
	float f32Val	= (float) m90e26ReadI16S(psEpWork->eChan, m90e26RegAddr[psEpWork->idx]) / 10.0 ;
	xEpSetValue(psEpWork, (x32_t) f32Val) ;
	return erSUCCESS ;
}

inline uint16_t m90e26GetSysStatus(uint8_t eChan)	{ return m90e26ReadU16(eChan, SYS_STATUS) ; }
inline uint16_t m90e26GetMeterStatus(uint8_t eChan)	{ return m90e26ReadU16(eChan, MET_STATUS) ; }

int32_t	m90e26SetLiveGain(uint8_t eChan, uint8_t Gain) {
	uint16_t	NewValue ;
	switch (Gain) {
	case 1:		NewValue	= 0x8000 ;		m90e26Config.Chan[eChan].L_Gain	= 1 ;	break ;
	case 4:		NewValue	= 0x0000 ;		m90e26Config.Chan[eChan].L_Gain	= 4 ;	break ;
	case 8:		NewValue	= 0x2000 ;		m90e26Config.Chan[eChan].L_Gain	= 8 ;	break ;
	case 16:	NewValue	= 0x4000 ;		m90e26Config.Chan[eChan].L_Gain	= 16 ;	break ;
	case 24:	NewValue	= 0x6000 ;		m90e26Config.Chan[eChan].L_Gain	= 24 ;	break ;
	default:	IF_SL_ERR(debugPARAM, "Invalid Live Gain =%d", Gain) ;			return erSCRIPT_INV_PARA ;
	}
	NewValue = m90e26ReadModifyWrite(eChan, MET_MODE, NewValue, 0xE000) ;
	return erSUCCESS ;
}

int32_t	m90e26SetNeutralGain(uint8_t eChan, uint8_t Gain) {
	uint16_t	NewValue ;
	switch (Gain) {
	case 1:		NewValue	= 0x1000 ;		m90e26Config.Chan[eChan].N_Gain = 1 ;	break ;
	case 2:		NewValue	= 0x0000 ;		m90e26Config.Chan[eChan].N_Gain = 2 ;	break ;
	case 4:		NewValue	= 0x0800 ;		m90e26Config.Chan[eChan].N_Gain = 4 ;	break ;
	default:	IF_SL_ERR(debugPARAM, "Invalid Neutral Gain =%d", Gain) ;		return erSCRIPT_INV_PARA ;
	}
	NewValue = m90e26ReadModifyWrite(eChan, MET_MODE, NewValue, 0x1800) ;
	return erSUCCESS ;
}

int32_t m90e26SoftReset(uint8_t eChan) {
	IF_myASSERT(debugPARAM, eChan < halHAS_M90E26) ;
	m90e26WriteU16(eChan, SOFTRESET, CODE_RESET) ;
	return erSUCCESS ;
}

int32_t m90e26Recalibrate(uint8_t eChan) {
	m90e26SoftReset(eChan) ;
	return m90e26Init(eChan) ;
}

// ############################### dynamic configuration support ###################################

int32_t	m90e26DisplayContrast(uint8_t Contrast) { ssd1306SetContrast(Contrast) ;	return erSUCCESS ; }

int32_t	m90e26DisplayState(uint8_t State) { ssd1306SetDisplayState(State) ; return erSUCCESS ; }

/**
 * m90e26ConfigMode() --  configure device functionality
 *
 * mode	/m90e26	option [para1 [para2 [para3]]]
 * 				L_GAIN	chan gain
 * 				N_GAIN	chan gain
 * 				BRIGHT	brightness
 * 				DISPLAY
 **/
int32_t	m90e26ConfigMode(rule_t * psRule) {
	IF_PRINT(debugMODE, "m90e26 Mode  p0=%d  p1=%d  p2=%d\n", psRule->para.u32[0][0], psRule->para.u32[0][1], psRule->para.u32[0][2]) ;
	int32_t iRV = erSUCCESS ;
	switch (psRule->para.u32[0][0]) {
	case eL_GAIN:		iRV = m90e26SetLiveGain(psRule->para.u32[0][1], psRule->para.u32[0][2]) ;		break ;

#if		(m90e26NEUTRAL == 1)		// NEUTRAL Line wrapper functions
	case eN_GAIN:		iRV = m90e26SetNeutralGain(psRule->para.u32[0][1], psRule->para.u32[0][2]) ;	break ;
#endif

	case eSOFTRESET:	iRV = m90e26SoftReset(psRule->para.u32[0][1]) ;									break ;

	case eRECALIB:		iRV = m90e26Recalibrate(psRule->para.u32[0][1]) ;								break ;

#if		(halHAS_SSD1306 > 0)
	case eBRIGHT:
		if ((psRule->para.u32[0][2] <= 255) && (psRule->para.u32[0][1] <= psRule->para.u32[0][2])) {
			m90e26Config.MinContrast = psRule->para.u32[0][1] ;
			m90e26Config.MaxContrast = psRule->para.u32[0][2] ;
		} else {
			iRV = erSCRIPT_INV_PARA ;
		}
		break ;

	case eDISPLAY:
	#if	(halHAS_M90E26 > 0)
		if (psRule->para.u32[0][1] < eDM_MAXIMUM) {
			m90e26Config.Chan[0].Display = psRule->para.u32[0][1] ;
		} else {
			iRV = erSCRIPT_INV_PARA ;
		}
	#endif
	#if	(halHAS_M90E26 > 1)
		if (psRule->para.u32[0][2] < eDM_MAXIMUM) {
			m90e26Config.Chan[1].Display = psRule->para.u32[0][2] ;
		} else {
			iRV = erSCRIPT_INV_PARA ;
		}
	#endif
		break ;

	case eBLANKING:		m90e26Config.tBlank = psRule->para.u32[0][1] ;									break ;
#endif
	default:			iRV = erSCRIPT_INV_MODE ;
	}
	return iRV ;
}

// ############################### device reporting functions ######################################

#define	HDR_CALIB		"Ch CALSTRT PLconsH PLconsL   Lgain    Lphi   Ngain    Nphi PStrtTh  PNolTh QStrtTh  QNolTh   MMode   CRC_1"
#define	HDR_MMODE		"  LgainN   LNSel 0-HPF-1 AmodCF1 RmodCF2   Zxmod Pthres%"
#define	HDR_ADJUST		"Ch ADJSTRT   Ugain  IgainL  IgainN   Vofst  IofstL  IofstN  PofstL  QofstL  PofstN  QofstN   CRC_2"
#define	HDR_DATA_LIVE	"Ch  ActFwd  ActRev  ActAbs  ReaFwd  ReaRev  ReaAbs   IrmsL    Vrms   PactL PreactL Freq Hz  PfactL PangleL   PappL"
#if		(m90e26NEUTRAL == 1)
	#define	HDR_DATA_NEUT	  "   IrmsN   PactN PreactN  PfactN PangleN   PappN"
#else
	#define	HDR_DATA_NEUT	  ""
#endif
#define	HDR_STATUS		"Ch  System    CRC1    CRC2  L/N Ch RevQchg RevPchg SagWarn   Meter Qnoload Pnoload    RevQ    RevP  Tamper  L-Mode"
#define	BLANK8			"        "

static const uint8_t m90e26DataReg[] = {
	E_ACT_FWD, E_ACT_REV, E_ACT_ABS, E_REACT_FWD, E_REACT_REV, E_REACT_ABS,
	I_RMS_L, V_RMS, P_ACT_L, P_REACT_L, FREQ, P_FACTOR_L, P_ANGLE_L, P_APP_L,
	I_RMS_N, P_ACT_N, P_REACT_N, P_FACTOR_N, P_ANGLE_N, P_APP_N,
} ;

void	m90e26ReportCalib(void) {
	PRINT("%C%s%C\n", xpfSGR(attrRESET, colourFG_CYAN,0,0), HDR_CALIB HDR_MMODE, xpfSGR(attrRESET, 0, 0, 0)) ;
	for (int32_t eChan = 0; eChan < halHAS_M90E26; ++eChan) {
		PRINT("%2d", eChan) ;
		for (int32_t i = CALSTART; i <= CRC_1; PRINT("  0x%04X", m90e26ReadU16(eChan, i++))) ;
		m90e26meter_mode_t	MeterMode = (m90e26meter_mode_t) m90e26ReadU16(eChan, MET_MODE) ;
		uint8_t	Pthres[16] = { 200, 100, 50, 25, 16, 32, 48, 64,80, 96, 112, 128, 144, 160, 176, 192 } ;
		PRINT("  %-2s  %2s %7s %7s %7s %7s  %6s %2.4f\n",
			MeterMode.Lgain == 3 ? "24" : MeterMode.Lgain == 2 ? "16" : MeterMode.Lgain == 1 ? "8" : MeterMode.Lgain == 0 ? "4" : "1",
			MeterMode.Ngain == 0 ? "2" : MeterMode.Ngain == 1 ? "4" : "1",
			MeterMode.LNSel ? "Live" : "Neutral",
			MeterMode.DisHPF == 0 ? "Ena Ena" : MeterMode.DisHPF == 1 ? "Ena Dis" : MeterMode.DisHPF == 2 ? "Dis Ena" : "Dis Dis",
			MeterMode.Amod	? "Abs" : "Act",
			MeterMode.Rmod	? "Abs" : "ReAct",
			MeterMode.Zxcon	== 0 ? "AllPos" : MeterMode.Zxcon	== 1 ? "AllNeg" : MeterMode.Zxcon	== 2 ? "All+/-" : "None",
			(float) Pthres[MeterMode.Pthresh] / 16) ;
	}
}

void	m90e26ReportAdjust(void) {
	PRINT("%C%s%C\n", xpfSGR(attrRESET, colourFG_CYAN,0,0), HDR_ADJUST, xpfSGR(attrRESET, 0, 0, 0)) ;
	for (int32_t eChan = 0; eChan < halHAS_M90E26; ++eChan) {
		PRINT("%2d", eChan) ;
		for (int32_t i = ADJSTART; i <= CRC_2; PRINT("  0x%04X", m90e26ReadU16(eChan, i++))) ;
		PRINT("\n") ;
	}
}

void	m90e26ReportData(void) {
	PRINT("%C%s%C\n", xpfSGR(attrRESET, colourFG_CYAN,0,0), HDR_DATA_LIVE HDR_DATA_NEUT, xpfSGR(attrRESET, 0, 0, 0)) ;
	for (int32_t eChan = 0; eChan < halHAS_M90E26; ++eChan) {
		PRINT("%2d", eChan) ;
		for (int32_t i = 0; i < eNUM_DATA_REG; ++i) {
			if (i < 6) {								// For energy registers reading it will reset the value...
				PRINT(" %7g", sRTCvars.aRTCsum[eChan][i]) ;
			} else {
				PRINT("  0x%04X", m90e26ReadU16(eChan, m90e26DataReg[i])) ;
			}
		}
		PRINT("\n") ;
	}
}

void	m90e26ReportStatus(void) {
	PRINT("%C%s%C\n", xpfSGR(attrRESET, colourFG_CYAN,0,0), HDR_STATUS, xpfSGR(attrRESET, 0, 0, 0)) ;
	for (int32_t eChan = 0; eChan < halHAS_M90E26; ++eChan) {
		m90e36system_stat_t SysStatus = (m90e36system_stat_t) m90e26GetSysStatus(eChan) ;
		PRINT("%2d  0x%04X", eChan, SysStatus.val) ;
		PRINT(SysStatus.CalErr		? "  Error "	: BLANK8) ;
		PRINT(SysStatus.AdjErr		? "  Error "	: BLANK8) ;
		PRINT(SysStatus.LnChge		? " L/N chg"	: BLANK8) ;
		PRINT(SysStatus.RevQchg		? " DIR chg"	: BLANK8) ;
		PRINT(SysStatus.RevPchg		? " DIR chg"	: BLANK8) ;
		PRINT(SysStatus.SagWarn		? "  V sag "	: BLANK8) ;
		m90e26meter_stat_t MeterStatus = (m90e26meter_stat_t) m90e26GetMeterStatus(eChan) ;
		PRINT("  0x%04X", MeterStatus.val) ;
		PRINT(MeterStatus.Qnoload	? " NO Load"	: BLANK8) ;
		PRINT(MeterStatus.Pnoload	? " NO Load"	: BLANK8) ;
		PRINT(MeterStatus.RevQ		? " Reverse"	: BLANK8) ;
		PRINT(MeterStatus.RevP		? " Reverse"	: BLANK8) ;
		PRINT(MeterStatus.Line 		? "    Live"	: " Neutral") ;
		PRINT(MeterStatus.LNMode==3 ? "    Flex"	:
			  MeterStatus.LNMode==2 ? "    Both"	:
			  MeterStatus.LNMode==1 ? "    Live"	: "  Tamper") ;
		PRINT("\n") ;
	}
}

void	m90e26Report(void) {
	m90e26ReportCalib() ;
	m90e26ReportAdjust() ;
	m90e26ReportData() ;
	m90e26ReportStatus() ;
}

#if	(halHAS_SSD1306 > 0)
#define	m90e26STEP_CONTRAST		0x04
#define	m90e26STAT_INTVL		pdMS_TO_TICKS(2 * MILLIS_IN_SECOND)

static	uint8_t eChan = 0,
				Index = 0 ;
static	TickType_t	NextTick = 0 ;
static	ep_work_t * psEpWork;

void	m90e26DisplayInfo(void) {
	ssd1306SetDisplayState(1) ;
	ssd1306SetTextCursor(0, 0) ;
	if ((Index % 2) == 0) {
		devprintfx(ssd1306PutChar, "Vo%8.3f" "Fr%8.3f" "Ir%8.3f" "Pa%8.3f" "An%8.3f" "Fa%8.3f",
		xCompVarGetValue(&psEpWork[eVOLTS].Var, NULL),
		xCompVarGetValue(&psEpWork[eFREQ].Var, NULL),
		xCompVarGetValue(&psEpWork[eI_RMS_L].Var, NULL),
		xCompVarGetValue(&psEpWork[eP_ACT_L].Var, NULL),
		xCompVarGetValue(&psEpWork[eP_ANGLE_L].Var, NULL),
		xCompVarGetValue(&psEpWork[eP_FACTOR_L].Var, NULL)) ;
	} else {
		devprintfx(ssd1306PutChar, "Af%8.3f" "Ar%8.3f" "Aa%8.3f" "Rf%8.3f" "Rr%8.3f" "Ra%8.3f",
		xCompVarGetValue(&psEpWork[eE_ACT_FWD].Var, NULL),
		xCompVarGetValue(&psEpWork[eE_ACT_REV].Var, NULL),
		xCompVarGetValue(&psEpWork[eE_ACT_ABS].Var, NULL),
		xCompVarGetValue(&psEpWork[eE_REACT_FWD].Var, NULL),
		xCompVarGetValue(&psEpWork[eE_REACT_REV].Var, NULL),
		xCompVarGetValue(&psEpWork[eE_REACT_ABS].Var, NULL)) ;
	}
}

void	m90e26Display(void) {
	if (m90e26_handle[0] == 0) {
		return ;
	}
	TickType_t CurTick = xTaskGetTickCount() ;
	if (NextTick == 0) {
		NextTick = CurTick ;
	} else if (NextTick > CurTick) {
		return ;
	}
#if 0
	if ((Index == 0) && m90e26Config.tBlank) {
		NextTick = CurTick + pdMS_TO_TICKS(m90e26Config.tBlank * MILLIS_IN_SECOND) ;
		ssd1306SetDisplayState(0) ;
		return true ;
	}
#endif
	//
	NextTick = CurTick + m90e26STAT_INTVL ;
	eChan = Index / halHAS_M90E26 ;
#if		(halHAS_M90E26 == 1)
	psEpWork = &table_work[URI_M90E26_E_ACT_FWD_0] ;
#elif	(halHAS_M90E26 == 2)
	psEpWork = &table_work[eChan == 0 ? URI_M90E26_E_ACT_FWD_0 : URI_M90E26_E_ACT_FWD_1] ;
#endif
	double dValue ;
	xCompVarGetValue(&psEpWork[eI_RMS_L].Var, &dValue) ;
	if ((m90e26Config.NowContrast > 0) &&
		(m90e26Config.Chan[eChan].Display == eDM_NORMAL ||
		(m90e26Config.Chan[eChan].Display == eDM_CURRENT && dValue != 0.0) ) ) {
		m90e26DisplayInfo() ;
	} else {
		ssd1306SetDisplayState(0) ;
	}
	//
	++Index ;
	Index %= (halHAS_M90E26 * 2) ;
	if (Index == 0) {
		m90e26Config.NowContrast += m90e26STEP_CONTRAST ;
		if (m90e26Config.NowContrast > m90e26Config.MaxContrast) {
			m90e26Config.NowContrast = m90e26Config.MinContrast ;
		}
		ssd1306SetContrast(m90e26Config.NowContrast) ;
		IF_PRINT(debugCONTRAST, "Contrast = %d\n", m90e26Config.NowContrast) ;
	}
	return ;
}
#endif

/* ################################### OLD CODE #####################################
uint16_t m90e26CalcCRC(uint8_t eChan, uint8_t Addr0, int8_t Count) {
	uint8_t Lcrc = 0, Hcrc = 0 ;
	uint16_t RegData[Count] ;
	for (int32_t i = 0; i < Count; ++i) {				// read the range of registers
		RegData[i] = m90e26Read(eChan, Addr0 + i) ;
	}
	for (int32_t i = 0; i < Count; ++i) {				// HI bytes: MOD256 sum & XOR
		Lcrc += RegData[i] >> 8 ;
		Hcrc ^= RegData[i] >> 8 ;
	}
	for (int32_t i = 0; i < Count; ++i) {				// LO bytes: MOD256 sum & XOR
		Lcrc += RegData[i] & 0xFF ;
		Hcrc ^= RegData[i] & 0xFF ;
	}
	IF_PRINT(debugCRC, "CRC=%04x from %-'h\n", (Hcrc << 8) | Lcrc, Count * 2, RegData) ;
	return (Hcrc << 8) | Lcrc ;
}

void	m90e26HandleCRC(uint8_t eChan, uint8_t RegAddr1, uint8_t RegAddr2) {
	IF_myASSERT(debugPARAM, eChan<halHAS_M90E26 && ((RegAddr1==CALSTART && RegAddr2==CRC_1) || (RegAddr1==ADJSTART && RegAddr2==CRC_2))) ;
	m90e26Write(eChan, RegAddr2, m90e26CalcCRC(eChan, RegAddr1 + 1, RegAddr2 - RegAddr1 - 1)) ;
	m90e26Write(eChan, RegAddr1, CODE_CHECK) ;
}

 */
#endif
