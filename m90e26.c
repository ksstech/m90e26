/*
 * m90e26.c
 * Copyright (c) 2018-22 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "m90e26.h"
#include "options.h"
#include "FreeRTOS_Support.h"
#include "ssd1306.h"
#include "printfx.h"
#include "syslog.h"
#include "systiming.h"					// timing debugging
#include "x_errors_events.h"
#include "hal_spi.h"
#include "hal_storage.h"

#include "nvs.h"

#define	debugFLAG					0xF000

#define	debugREAD					(debugFLAG & 0x0001)
#define	debugWRITE					(debugFLAG & 0x0002)
#define	debugRMW					(debugFLAG & 0x0004)
//#define	debugINIT					(debugFLAG & 0x0008)

#define	debugCURRENT				(debugFLAG & 0x0020)
#define	debugPOWER					(debugFLAG & 0x0200)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ###################################### BUILD Macros #############################################

#if		(m90e26NEUTRAL > 0)
	#define	M90E26_NUMURI_0		(URI_M90E26_P_APP_N_0 - URI_M90E26_E_ACT_FWD_0 + 1)
	#define	M90E26_NUMURI_1		(URI_M90E26_P_APP_N_1 - URI_M90E26_E_ACT_FWD_1 + 1)
#else
	#define	M90E26_NUMURI_0		(URI_M90E26_P_APP_L_0 - URI_M90E26_E_ACT_FWD_0 + 1)
	#define	M90E26_NUMURI_1		(URI_M90E26_P_APP_L_1 - URI_M90E26_E_ACT_FWD_1 + 1)
#endif
#if		(M90E26_NUMURI_0 != M90E26_NUMURI_1)
	#error	"error in m90e26 URI definitions!!!"
#endif

#define	M90E26_T_SNS				1000

// ###################################### Private variables #######################################

spi_device_interface_config_t	m90e26_config[halHAS_M90E26] = {
#if	(halHAS_M90E26 > 0)
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
#if	(halHAS_M90E26 > 1)
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

spi_device_handle_t	m90e26_handle[halHAS_M90E26];
SemaphoreHandle_t	m90e26mutex[halHAS_M90E26];
u8_t	NumM90E26 = halHAS_M90E26;

struct __attribute__((packed)) {
	u8_t	NowContrast ;
	struct __attribute__((packed)) {
		u8_t	E_Scale	: 1 ;							// 0 = WHr,	1 = KwHr
		u8_t	P_Scale	: 1 ;							// 0 = W,	1 = Kw
		u8_t	I_Scale	: 1 ;							// 0 = A,	1 = KwHr
		u8_t	Display	: 2 ;							// 0 = Off, 1 = Normal,	2 = Current, 3 = Button
		u8_t	Spare	: 3 ;
		u8_t	N_Gain	: 3 ;							// 1 -> 4
		u8_t	L_Gain	: 5 ;							// 1 -> 24
	} Chan[halHAS_M90E26] ;
} m90e26Config = { 0 } ;

const u8_t	m90e26RegAddr[] = {
	E_ACT_FWD,	E_ACT_REV,	E_ACT_ABS,					// Active energy
	E_REACT_FWD,E_REACT_REV,E_REACT_ABS,				// ReActive energy
	I_RMS_L,	V_RMS,		P_ACT_L,	P_REACT_L,		// Voltage, Freq & LIVE Power info
	FREQ,		P_FACTOR_L,	P_ANGLE_L,	P_APP_L,
	#if	(m90e26NEUTRAL > 0)
	I_RMS_N,	P_ACT_N,	P_REACT_N,					// NEUTRAL Power info
	P_FACTOR_N,	P_ANGLE_N,	P_APP_N,
	#endif
};

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
	#if	(halHAS_M90E26 > 0)
	{ {	0x00B9, 0xC1F3, 0x0000, 0x0000, 0x0000, 0x0000, 0x08BD, 0x8100, 0x0AEC, 0x8100, 0x9422 },
	  {	0x6C50, 0x7C2A, 0x7530, 0x0000, 0xF711, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
	  {	0x0030, 0x1F2F, 0x0000	}, },
	#endif
	#if	(halHAS_M90E26 > 1)
	{ {	0x00B9, 0xC1F3, 0x0000, 0x0000, 0x0000, 0x0000, 0x08BD, 0x8100, 0x0AEC, 0x8100, 0x9422 },
	  {	0x6C50, 0x7C2A, 0x7530, 0x0000, 0xF800, 0xF400, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF },
  	  {	0x0030, 0x1F2F, 0x0000	}, },
	#endif
};

// ############################### common support routines #########################################

void m90e26WriteU16(u8_t eChan, u8_t address, u16_t val) {
	IF_myASSERT(debugPARAM, address < 0x70 && m90e26_handle[eChan]) ;
	xRtosSemaphoreTake(&m90e26mutex[eChan], portMAX_DELAY) ;
	spi_transaction_t m90e26_buf ;
	memset(&m90e26_buf, 0, sizeof(m90e26_buf));
	m90e26_buf.length		= 8 * 3;
	m90e26_buf.flags 		= SPI_TRANS_USE_TXDATA ;
	m90e26_buf.tx_data[0]	= address ;
	m90e26_buf.tx_data[1]	= val >> 8 ;
	m90e26_buf.tx_data[2]	= val & 0xFF ;
	IF_EXEC_1(debugTIMING, xSysTimerStart, stM90EX6W) ;
	ESP_ERROR_CHECK(spi_device_transmit(m90e26_handle[eChan], &m90e26_buf)) ;
	IF_EXEC_1(debugTIMING, xSysTimerStop, stM90EX6W) ;
	xRtosSemaphoreGive(&m90e26mutex[eChan]);
	IF_P(debugWRITE, "TX: addr=%02x d0=%02x d1=%02x\r\n", m90e26_buf.tx_data[0], m90e26_buf.tx_data[1], m90e26_buf.tx_data[2]) ;
}

u16_t m90e26ReadU16(u8_t eChan, u8_t address) {
	IF_myASSERT(debugPARAM, address < 0x70 && m90e26_handle[eChan]) ;
	xRtosSemaphoreTake(&m90e26mutex[eChan], portMAX_DELAY) ;
	spi_transaction_t m90e26_buf ;
	memset(&m90e26_buf, 0, sizeof(m90e26_buf)) ;
	m90e26_buf.length		= 8 * 3 ;
	m90e26_buf.flags 		= SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA ;
	m90e26_buf.tx_data[0]	= address | 0x80 ;
	IF_EXEC_1(debugTIMING, xSysTimerStart, stM90EX6R) ;
	ESP_ERROR_CHECK(spi_device_transmit(m90e26_handle[eChan], &m90e26_buf)) ;
	IF_EXEC_1(debugTIMING, xSysTimerStop, stM90EX6R) ;
	xRtosSemaphoreGive(&m90e26mutex[eChan]) ;
	IF_P(debugREAD, "RX: addr=%02x  d0=%02x  d1=%02x  dx=%04x\r\n", m90e26_buf.tx_data[0], m90e26_buf.rx_data[1], m90e26_buf.rx_data[2], (m90e26_buf.rx_data[1] << 8) | m90e26_buf.rx_data[2]) ;
	return (m90e26_buf.rx_data[1] << 8) | m90e26_buf.rx_data[2] ;
}

void m90e26WriteRegister(u8_t eChan, u8_t Reg, u16_t Val) {
	if (INRANGE(PLconstH, Reg, MET_MODE)) {
		if (m90e26ReadU16(eChan, CALSTART) != CODE_START) {
			SL_NOT("CALSTART (x20) in wrong state, must be x5678") ;
		} else {
			m90e26WriteU16(eChan, Reg, Val) ;				// write new value & update CRC
			m90e26WriteU16(eChan, CRC_1, m90e26ReadU16(eChan, CRC_1)) ;
			IF_P(debugTRACK && ioB1GET(ioM90write), "Before: #%d %-`H\r\n", Reg-PLconstH, SO_MEM(nvs_m90e26_t, calreg), &nvsM90E26default[eChan].calreg) ;
			nvsM90E26default[eChan].calreg[Reg-PLconstH] = Val ;
			IF_P(debugTRACK && ioB1GET(ioM90write), "After : #%d %-`H\r\n", Reg-PLconstH, SO_MEM(nvs_m90e26_t, calreg), &nvsM90E26default[eChan].calreg) ;
		}
	} else if (INRANGE(U_GAIN, Reg, Q_OFST_N)) {
		if (m90e26ReadU16(eChan, ADJSTART) != CODE_START) {
			SL_NOT("ADJSTART (x30) in wrong state, must be x5678") ;
		} else {
			m90e26WriteU16(eChan, Reg, Val) ;				// write new value & update CRC
			m90e26WriteU16(eChan, CRC_2, m90e26ReadU16(eChan, CRC_2)) ;
			IF_P(debugTRACK && ioB1GET(ioM90write), "Before: #%d %-`H\r\n", Reg-U_GAIN, SO_MEM(nvs_m90e26_t, adjreg), &nvsM90E26default[eChan].adjreg) ;
			nvsM90E26default[eChan].adjreg[Reg-U_GAIN] = Val ;
			IF_P(debugTRACK && ioB1GET(ioM90write), "After : #%d %-`H\r\n", Reg-U_GAIN, SO_MEM(nvs_m90e26_t, adjreg), &nvsM90E26default[eChan].adjreg) ;
		}
	} else if (Reg == SOFTRESET || INRANGE(FUNC_ENAB, Reg, POWER_MODE) || Reg == CALSTART || Reg == ADJSTART) {
		m90e26WriteU16(eChan, Reg, Val) ;				// write new value
		if (INRANGE(FUNC_ENAB, Reg, POWER_MODE)) {
			IF_P(debugTRACK && ioB1GET(ioM90write), "Before: #%d %-`H\r\n", Reg-FUNC_ENAB, SO_MEM(nvs_m90e26_t, cfgreg), &nvsM90E26default[eChan].cfgreg) ;
			nvsM90E26default[eChan].cfgreg[Reg-FUNC_ENAB] = Val ;
			IF_P(debugTRACK && ioB1GET(ioM90write), "After : #%d %-`H\r\n", Reg-FUNC_ENAB, SO_MEM(nvs_m90e26_t, cfgreg), &nvsM90E26default[eChan].cfgreg) ;
		}
	} else {
		SL_ERR("Invalid register=0x%02X", Reg) ;
	}
}

u16_t m90e26ReadModifyWrite(u8_t eChan, u8_t Addr, u16_t Value, u16_t Mask) {
	IF_P(debugRMW, "  C=%d  R=%d  &=x%04X  |=x%04X", eChan, Addr, Value, Mask) ;
	u16_t CurValue = m90e26ReadU16(eChan, Addr) ;
	IF_P(debugRMW, "  V=x%04X", CurValue) ;
	CurValue &= ~Mask ;
	IF_P(debugRMW, " -> x%04X", CurValue) ;
	CurValue |= Value ;
	IF_P(debugRMW, " -> x%04X\r\n", CurValue) ;
	m90e26WriteRegister(eChan, Addr, CurValue) ;
	return CurValue ;
}

s16_t m90e26ReadI16S(u8_t eChan, u8_t Reg) {
	u16_t RawVal = m90e26ReadU16(eChan, Reg) ;
	bool	Sign	= RawVal & 0x8000 ? true : false ;
	s16_t	i16Val	= RawVal & 0x7FFF ;
	s16_t	ConVal	= Sign == true ? -1 * i16Val : i16Val ;
	return ConVal ;
}

s16_t m90e26ReadI16TC(u8_t eChan, u8_t Reg) { return ~m90e26ReadU16(eChan, Reg) + 1 ; }

u32_t m90e26ReadU32(u8_t eChan, u8_t Reg) {
	#if	(m90e26READ_32BIT == 1)
	return (m90e26ReadU16(eChan, Reg) << 16) + m90e26ReadU16(eChan, LSB) ;
	#else
	return (m90e26ReadU16(eChan, Reg) << 16) ;
	#endif
}

s32_t m90e26ReadI32TC(u8_t eChan, u8_t Reg) { return ~m90e26ReadU32(eChan, Reg) + 1 ; }

int	m90e26LoadNVSConfig(u8_t eChan, u8_t Idx) {
	IF_myASSERT(debugPARAM, Idx < CALIB_NUM) ;
	size_t	SizeBlob = CALIB_NUM * sizeof(nvs_m90e26_t) ;
	nvs_m90e26_t * psCalib = pvRtosMalloc(SizeBlob) ;
	int iRV = halSTORAGE_ReadBlob(halSTORAGE_STORE, halSTORAGE_KEY_M90E26, psCalib, &SizeBlob, ESP_OK) ;
	if (iRV == erSUCCESS) {
		psCalib += Idx ;								// write the FuncEnab, Vsag Threshold and PowerMode registers
		for (int i = 0; i < NO_ELEM(nvs_m90e26_t, cfgreg); m90e26WriteU16(eChan, i+FUNC_ENAB, psCalib->cfgreg[i]), ++i) ;

		m90e26WriteU16(eChan, CALSTART, CODE_START) ;		// write the configuration registers with METER calibration data
		for (int i = 0; i < NO_ELEM(nvs_m90e26_t, calreg); m90e26WriteRegister(eChan, i+PLconstH, psCalib->calreg[i]), ++i) ;
		IF_EXEC_3(configPRODUCTION == 1, m90e26WriteU16, eChan, CALSTART, CODE_CHECK) ;

		m90e26WriteU16(eChan, ADJSTART, CODE_START) ;		// write the configuration registers with MEASURE calibration data
		for (int i = 0; i < NO_ELEM(nvs_m90e26_t, adjreg); m90e26WriteRegister(eChan, i+U_GAIN, psCalib->adjreg[i]), ++i) ;
		IF_EXEC_3(configPRODUCTION == 1, m90e26WriteU16, eChan, ADJSTART, CODE_CHECK) ;
	} else {
		SL_ERR("Failed Ch=%d config=%d", eChan, Idx);
	}
	vRtosFree (psCalib);
	return iRV ;
}

int	CmndM90_WriteChannels(u8_t eChan, u8_t Reg, u16_t Value) {
	u8_t	Cnow = eChan < NumM90E26 ? eChan : 0 ;
	do {
		m90e26WriteRegister(eChan, Reg, Value) ;
	} while (++Cnow < eChan) ;
	return erSUCCESS ;
}

u8_t m90e26CalcInfo(epw_t * psEW) {
	xEpWorkToUri(psEW) ;
	psEW->idx = psEW->uri - URI_M90E26_E_ACT_FWD_0 ;
	psEW->eChan = 0 ;
	// In case of multiple channels, adjust till in range....
	while (psEW->idx >= M90E26_NUMURI_0) {
		psEW->idx -= M90E26_NUMURI_0 ;
		psEW->eChan++ ;
	}
	IF_myASSERT(debugRESULT, (psEW->idx < M90E26_NUMURI_0) && (psEW->eChan < halHAS_M90E26)) ;
	return psEW->idx ;
}

/* ######################################## Calibration ############################################
 * Overall process documented in the following Application Note:
 * http://ww1.microchip.com/downloads/en/AppNotes/Atmel-46102-SE-M90E26-ApplicationNote.pdf */

// select inclusion of LSB value for RMS and power measurements
#define	m90e26USE_LSB				1					// add LSB to RMS & Pwr values read
#define	m90e26CALIB_32BIT			1
#define	m90e26CALIB_ITER			10					// number of READ iterations to determine mean value

/**
 * Re/Active, LINE & NEUTRAL, Power Offset calibration
 * @param	eChan
 * @param	RegPower
 * @param	RegOFST
 *
 * Not sure if the power register should be read whilst in normal or
 * adjustment mode. Currently no real value is being read
 */
void m90e26PowerOffsetCalcSet(u8_t eChan, u8_t RegPOWER, u8_t RegOFST) {
	m90e26WriteU16(eChan, POWER_MODE, CODE_POWER) ;		// set into low power mode for calibration
	#if (m90e26CALIB_32BIT == 1)
	uint64_t SumOffset = 0 ;
	for (int i = 0; i < m90e26CALIB_ITER; ++i) {
		u32_t CurVal = m90e26ReadU32(eChan, RegPOWER) ;
		SumOffset += CurVal ;
		IF_P(debugTRACK && ioB1GET(ioM90offset), "#%d=0x%04X  ", i, CurVal) ;
	}
	u32_t NewAVG	= SumOffset / m90e26CALIB_ITER ;
	u32_t NewOffset = (~NewAVG + 1) >> 16 ;
	IF_P(debugTRACK && ioB1GET(ioM90offset), "\r\nCh %d:  S=0x%010llX  A=0x%08X  O=0x%08X\r\n", eChan, SumOffset, NewAVG, NewOffset) ;

	#elif 0
	u32_t SumOffset = 0 ;
	for (int i = 0; i < m90e26CALIB_ITER; ++i) {
		u32_t CurVal = m90e26ReadU16(eChan, RegPOWER) ;
		SumOffset += CurVal ;
		IF_P(debugTRACK && ioB1GET(ioM90offset), "#%d=0x%04X  ", i, CurVal) ;
	}
	u32_t NewAVG	= SumOffset / m90e26CALIB_ITER ;
	u32_t NewOffset = ~NewAVG + 1;
	IF_P(debugTRACK && ioB1GET(ioM90offset), "\r\nCh %d:  S=0x%08X  A=0x%08X  O=0x%08X\r\n", eChan, SumOffset, NewAVG, NewOffset) ;

	#elif 0
	s32_t SumOffset = 0 ;
	for (int i = 0; i < m90e26CALIB_ITER; ++i) {
		s32_t CurVal = m90e26ReadI16S(eChan, RegPOWER) ;
		SumOffset += CurVal ;
		IF_P(debugTRACK && ioB1GET(ioM90offset), "#%d=%d  ", i, CurVal) ;
	}
	s16_t NewAVG	= SumOffset / m90e26CALIB_ITER ;
	s16_t NewOffset = ~NewAVG + 1;
	IF_P(debugTRACK && ioB1GET(ioM90offset), "\r\nCh %d: S=%d  A=%d  O=0x%08X\r\n", eChan, SumOffset, NewAVG, NewOffset) ;
	#endif
	m90e26WriteRegister(eChan, RegOFST, NewOffset) ;
	m90e26WriteU16(eChan, POWER_MODE, CODE_RESET) ;		// reset to normal power mode
}

void m90e26CurrentOffsetCalcSet(u8_t eChan, u8_t RegRMS, u8_t RegGAIN, u8_t RegOFST) {
	u32_t CurAmps = m90e26ReadU16(eChan, RegRMS) ;
	u32_t CurGain = m90e26ReadU16(eChan, RegGAIN) ;
	u32_t Factor1 = CurAmps * CurGain >> 8 ;
	u32_t Factor2 = ~Factor1 + 1 ;
	m90e26WriteRegister(eChan, RegOFST, Factor2) ;
	IF_P(debugTRACK && ioB1GET(ioM90offset), "Ch %d %s: Icur=0x%04X  Gcur=0x%04X  F1=0x%08X  F2=0x%08X\r\n",
			eChan, RegRMS == I_RMS_L ? "Live" : "Neutral", CurAmps, CurGain, Factor1, Factor2) ;
}

// ############################## identification & initialization ##################################

int	m90e26Identify(u8_t eChan) {
	IF_myASSERT(debugPARAM, eChan < NumM90E26) ;
	ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &m90e26_config[eChan], &m90e26_handle[eChan])) ;
	m90e26mutex[eChan]	= xSemaphoreCreateMutex() ;
	IF_myASSERT(debugRESULT, m90e26mutex[eChan]) ;

	int	Uri0 = (eChan == 0) ? URI_M90E26_E_ACT_FWD_0	: URI_M90E26_E_ACT_FWD_1 ;
	#if	(m90e26NEUTRAL > 0)
	int UriX = (eChan == 0) ? URI_M90E26_P_APP_N_0		: URI_M90E26_P_APP_N_1 ;
	#else
	int UriX = (eChan == 0) ? URI_M90E26_P_APP_L_0		: URI_M90E26_P_APP_L_1 ;
	#endif
	for (int i = Uri0 ; i <= UriX; ++i) {
		epw_t * psEW = &table_work[i] ;
		if (i < (Uri0 + 6)) {
			psEW->var.def.cv.sumX	= 1 ;
		}
		psEW->var.def.cv.vf	= vfFXX ;
		psEW->var.def.cv.vt	= vtVALUE ;
		psEW->var.def.cv.vs	= vs32B ;
		psEW->var.def.cv.vc	= 1 ;
		psEW->Tsns = psEW->Rsns = M90E26_T_SNS;		// start sensing
	}
	return erSUCCESS ;
}

/**
 * m90e26Init() -
 */
int	m90e26Init(u8_t eChan) {
	IF_myASSERT(debugPARAM, eChan < NumM90E26) ;
	IF_SYSTIMER_INIT(debugTIMING, stM90EX6R, stMICROS, "M90E26RD", 1500, 15000) ;
	IF_SYSTIMER_INIT(debugTIMING, stM90EX6W, stMICROS, "M90E26WR", 1500, 15000) ;
	/* Check that blob with CALibration and ADJustment values exists
	 * If not existing, create with factory defaults as first record */
	size_t	SizeBlob = CALIB_NUM * sizeof(nvs_m90e26_t) ;
	nvs_m90e26_t * psCalib = pvRtosMalloc(SizeBlob) ;
	int iRV = halSTORAGE_ReadBlob(halSTORAGE_STORE, halSTORAGE_KEY_M90E26, psCalib, &SizeBlob, ESP_ERR_NVS_NOT_FOUND);
	if ((iRV != erSUCCESS) || (SizeBlob != (CALIB_NUM * sizeof(nvs_m90e26_t)))) {
		memset(psCalib, 0, SizeBlob = CALIB_NUM * sizeof(nvs_m90e26_t)) ;
		memcpy(psCalib, &nvsM90E26default, sizeof(nvsM90E26default)) ;
		iRV = halSTORAGE_WriteBlob(halSTORAGE_STORE, halSTORAGE_KEY_M90E26, psCalib, SizeBlob);
		IF_myASSERT(debugRESULT, iRV == erSUCCESS);
	}
	vRtosFree(psCalib) ;

	m90e26WriteU16(eChan, SOFTRESET, CODE_RESET) ;		// start with default values, not running, no valid values
	m90e26LoadNVSConfig(eChan, 0) ;						// load config #0 from NVS blob as default
	m90e26Config.Chan[eChan].L_Gain		= 1 ;			// set default state
	#if	(m90e26NEUTRAL > 0)
	m90e26Config.Chan[eChan].N_Gain		= 1 ;
	#endif
	m90e26Config.Chan[eChan].E_Scale	= 0 ;			// Wh not kWh
	m90e26Config.Chan[eChan].P_Scale	= 0 ;			// W not kW
	m90e26Config.Chan[eChan].I_Scale	= 0 ;			// A not mA
	return (m90e26GetSysStatus(eChan) & 0xF000) ? erFAILURE : erSUCCESS ;
}

// ########################### 32 bit value endpoint support functions #############################

int	m90e26ReadCurrent(epw_t * psEW) {
	u8_t eIdx = m90e26CalcInfo(psEW) ;
	float f32Val = (float) m90e26ReadU32(psEW->eChan, m90e26RegAddr[eIdx]) ;
	IF_P(debugCURRENT, "Irms: URI=%d  Idx=%d  Reg=%02X  Ch=%d", psEW->uri, eIdx, m90e26RegAddr[eIdx], psEW->eChan) ;
	if (m90e26Config.Chan[psEW->eChan].I_Scale == 0) {
		f32Val /= 65536000.0 ;							// convert to Amp
		IF_P(debugCURRENT, "  Val=%4.5fA", f32Val) ;
	} else {
		f32Val /= 65536.0 ;								// convert to mA
		IF_P(debugCURRENT, "  Val=%4.5fmA", f32Val) ;
	}

	#if	(m90e26NEUTRAL > 0)
	IF_myASSERT(debugRESULT, eIdx == eI_RMS_L || eIdx == eI_RMS_N) ;
	if (eIdx == eI_RMS_L) {
		f32Val /= m90e26Config.Chan[psEW->eChan].L_Gain ;
		IF_P(debugCURRENT, "  Lgain=%d", m90e26Config.Chan[psEW->eChan].L_Gain) ;
	} else {
		f32Val /= m90e26Config.Chan[psEW->eChan].N_Gain ;
		IF_P(debugCURRENT, "  Ngain=%d", m90e26Config.Chan[psEW->eChan].N_Gain) ;
	}
	#else
	IF_myASSERT(debugRESULT, eIdx == eI_RMS_L) ;
	f32Val /= m90e26Config.Chan[psEW->eChan].L_Gain ;
	IF_P(debugCURRENT, "  Lgain=%d", m90e26Config.Chan[psEW->eChan].L_Gain) ;
	#endif
	IF_P(debugCURRENT, "  Act=%4.5f\r\n", f32Val) ;
	xEpSetValue(psEW, (x32_t) f32Val) ;
	return erSUCCESS ;
}

int	m90e26ReadVoltage(epw_t * psEW) {		// OK
	m90e26CalcInfo(psEW) ;
	float	f32Val	= (float) m90e26ReadU32(psEW->eChan, m90e26RegAddr[psEW->idx]) ;
	f32Val			/= 6553600.0 ;						// Change mV to V
	xEpSetValue(psEW, (x32_t) f32Val) ;
//	IF_P(debugVOLTS, "Vrms: Ch=%d  Val=%9.3f\r\n", psEW->eChan, f32Val) ;
	return erSUCCESS ;
}

int	m90e26ReadPower(epw_t * psEW) {
	m90e26CalcInfo(psEW) ;
	float	f32Val	= (float) m90e26ReadI32TC(psEW->eChan, m90e26RegAddr[psEW->idx]) ;
	f32Val /= (m90e26Config.Chan[psEW->eChan].P_Scale == 1) ? 65536000.0 : 65536.0 ;
	xEpSetValue(psEW, (x32_t) f32Val) ;
	IF_P(debugPOWER, "Power: Ch=%d  Reg=0x%02X  Val=%9.3f\r\n", psEW->eChan, m90e26RegAddr[psEW->idx], f32Val) ;
	return erSUCCESS ;
}

// ########################### 16 bit value endpoint support functions #############################

int	m90e26ReadEnergy(epw_t * psEW) {
	if (psEW->var.def.cv.sumX) {	// if just a normal update cycle
		m90e26CalcInfo(psEW) ;
		float f32Val	= (float) m90e26ReadU16(psEW->eChan, m90e26RegAddr[psEW->idx]) ;
		f32Val	/= m90e26Config.Chan[psEW->eChan].E_Scale ? 10000.0 : 10.0 ;
		xEpSetValue(psEW, (x32_t) f32Val) ;
		// Update running total in NVS memory
		sRTCvars.aRTCsum[psEW->eChan][psEW->idx] += f32Val ;
	} else {						// else it is a value reset call
		vCV_ResetValue(&psEW->var);
	}
	return erSUCCESS ;
}

int	m90e26ReadFrequency(epw_t * psEW) {
	m90e26CalcInfo(psEW) ;
	float f32Val = (float) m90e26ReadU16(psEW->eChan, m90e26RegAddr[psEW->idx]) / 100.0 ;
	xEpSetValue(psEW, (x32_t) f32Val) ;
	return erSUCCESS ;
}

int	m90e26ReadPowerFactor(epw_t * psEW) {
	m90e26CalcInfo(psEW) ;
	float f32Val = (float)  m90e26ReadI16S(psEW->eChan, m90e26RegAddr[psEW->idx]) / 1000.0 ;
	xEpSetValue(psEW, (x32_t) f32Val) ;
	return erSUCCESS ;
}

int	m90e26ReadPowerAngle(epw_t * psEW) {
	m90e26CalcInfo(psEW) ;
	float f32Val = (float) m90e26ReadI16S(psEW->eChan, m90e26RegAddr[psEW->idx]) / 10.0 ;
	xEpSetValue(psEW, (x32_t) f32Val) ;
	return erSUCCESS ;
}

inline u16_t m90e26GetSysStatus(u8_t eChan) { return m90e26ReadU16(eChan, SYS_STATUS) ; }

inline u16_t m90e26GetMeterStatus(u8_t eChan)	{ return m90e26ReadU16(eChan, MET_STATUS) ; }

int	m90e26SetLiveGain(u8_t eChan, u8_t Gain) {
	u16_t	NewValue ;
	switch (Gain) {
	case 1:		NewValue	= 0x8000 ;		m90e26Config.Chan[eChan].L_Gain	= 1 ;	break ;
	case 4:		NewValue	= 0x0000 ;		m90e26Config.Chan[eChan].L_Gain	= 4 ;	break ;
	case 8:		NewValue	= 0x2000 ;		m90e26Config.Chan[eChan].L_Gain	= 8 ;	break ;
	case 16:	NewValue	= 0x4000 ;		m90e26Config.Chan[eChan].L_Gain	= 16 ;	break ;
	case 24:	NewValue	= 0x6000 ;		m90e26Config.Chan[eChan].L_Gain	= 24 ;	break ;
	default:
		IF_SL_ERR(debugPARAM, "Invalid Live Gain =%d", Gain) ;
		return erINVALID_PARA ;
	}
	NewValue = m90e26ReadModifyWrite(eChan, MET_MODE, NewValue, 0xE000) ;
	return erSUCCESS ;
}

int	m90e26SetNeutralGain(u8_t eChan, u8_t Gain) {
	u16_t	NewValue ;
	switch (Gain) {
	case 1:		NewValue	= 0x1000 ;		m90e26Config.Chan[eChan].N_Gain = 1 ;	break ;
	case 2:		NewValue	= 0x0000 ;		m90e26Config.Chan[eChan].N_Gain = 2 ;	break ;
	case 4:		NewValue	= 0x0800 ;		m90e26Config.Chan[eChan].N_Gain = 4 ;	break ;
	default:
		IF_SL_ERR(debugPARAM, "Invalid Neutral Gain =%d", Gain) ;
		return erINVALID_PARA ;
	}
	NewValue = m90e26ReadModifyWrite(eChan, MET_MODE, NewValue, 0x1800) ;
	return erSUCCESS ;
}

// ############################### dynamic configuration support ###################################

int	m90e26DisplayContrast(u8_t Contrast) { ssd1306SetContrast(Contrast) ; return erSUCCESS ; }

int	m90e26DisplayState(u8_t State) { ssd1306SetDisplayState(State) ; return erSUCCESS ; }

/**
 * Configure device functionality
 * @brief	mode /m90e26 ch# option [para1 [para2 [para3]]]
 * 					Option 1=Lgain	2=Ngain
 **/
int	m90e26ConfigMode(rule_t * psRule, int Xnow, int Xmax) {
	u8_t AI = psRule->ActIdx;
	u32_t P0 = psRule->para.x32[AI][0].u32;
	u32_t P1 = psRule->para.x32[AI][1].u32;
	s32_t P2 = psRule->para.x32[AI][2].i32;
	s32_t P3 = psRule->para.x32[AI][3].i32;
	IF_P(debugTRACK && ioB1GET(ioMode), "m90e26 Idx=%d  Mode=%d  p2=%d p3=%d\r\n", P0, P1, P2, P3);
	int iRV = erSUCCESS ;
	do {
		switch (P1) {
		case eL_GAIN:
			iRV = m90e26SetLiveGain(Xnow, P2) ;
			break ;

		#if	(m90e26NEUTRAL > 0)		// NEUTRAL Line wrapper functions
		case eN_GAIN:
			iRV = m90e26SetNeutralGain(Xnow, P2) ;
			break ;
		#endif

		case eSOFTRESET:
		case eRECALIB:
			CmndM90_WriteChannels(Xnow, SOFTRESET, CODE_RESET) ;
			if (P1 == eRECALIB)
				iRV = m90e26Init(Xnow);
			break ;

		case m90e26CALC_CUR_OFST:
			m90e26CurrentOffsetCalcSet(Xnow, I_RMS_L, I_GAIN_L, I_OFST_L) ;
			#if	(m90e26NEUTRAL > 0)
			m90e26CurrentOffsetCalcSet(Xnow, I_RMS_N, I_GAIN_N, I_OFST_N) ;
			#endif
			break ;

		case m90e26CALC_PWR_OFST:
			m90e26PowerOffsetCalcSet(Xnow, P_ACT_L, P_OFST_L) ;
			m90e26PowerOffsetCalcSet(Xnow, P_REACT_L, Q_OFST_L) ;
			#if	(m90e26NEUTRAL > 0)
			m90e26PowerOffsetCalcSet(Xnow, P_ACT_N, P_OFST_N) ;
			m90e26PowerOffsetCalcSet(Xnow, P_REACT_N, Q_OFST_N) ;
			#endif
			break ;

		case m90e26CALIB_SAVE:
			if (OUTSIDE(0, P2, CALIB_NUM-1))
				return erINVALID_PARA;

			size_t	SizeBlob = CALIB_NUM * sizeof(nvs_m90e26_t) ;
			nvs_m90e26_t * psCalib = pvRtosMalloc(SizeBlob) ;
			memset(psCalib, 0, SizeBlob) ;
			int iRV = halSTORAGE_ReadBlob(halSTORAGE_STORE, halSTORAGE_KEY_M90E26, psCalib, &SizeBlob, ESP_OK);
			IF_SL_INFO(iRV != erSUCCESS, "Error reading M90E26blob, starting with blank");
			nvs_m90e26_t * psTemp = psCalib + P2 ;
			for (int i = 0; i < NO_ELEM(nvs_m90e26_t, calreg); ++i)
				psTemp->calreg[i] = m90e26ReadU16(Xnow, i + PLconstH);
			for (int i = 0; i < NO_ELEM(nvs_m90e26_t, adjreg); ++i)
				psTemp->adjreg[i] = m90e26ReadU16(Xnow, i + U_GAIN);
			for (int i = 0; i < NO_ELEM(nvs_m90e26_t, cfgreg); ++i)
				psTemp->cfgreg[i] = m90e26ReadU16(Xnow, i + FUNC_ENAB);

			iRV = halSTORAGE_WriteBlob(halSTORAGE_STORE, halSTORAGE_KEY_M90E26, psCalib, CALIB_NUM * sizeof(nvs_m90e26_t)) ;
			IF_myASSERT(debugRESULT, iRV == erSUCCESS) ;
			vRtosFree(psCalib) ;
			break ;

		case m90e26CALIB_DELETE:
			iRV = halSTORAGE_DeleteKeyValue(halSTORAGE_STORE, halSTORAGE_KEY_M90E26) ;
			Xnow = Xmax ;
			break ;

		case m90e26WRITE_REG:
			if (OUTSIDE(SOFTRESET, P2, CRC_2) || OUTSIDE(0, P3, 0xFFFF))
				return erINVALID_PARA;
			CmndM90_WriteChannels(Xnow, P2, P3) ;
			break ;

		default:
			iRV = erINVALID_MODE ;
		}
	} while (iRV >= erSUCCESS && ++Xnow < Xmax) ;
	return iRV ;
}

// ############################### device reporting functions ######################################

#define	HDR_CALIB		"Ch CALSTRT PLconsH PLconsL   Lgain    Lphi   Ngain    Nphi PStrtTh  PNolTh QStrtTh  QNolTh   MMode   CRC_1"
#define	HDR_MMODE		"  LgainN   LNSel 0-HPF-1 AmodCF1 RmodCF2   Zxmod Pthres%"
#define	HDR_ADJUST		"Ch ADJSTRT   Ugain  IgainL  IgainN   Vofst  IofstL  IofstN  PofstL  QofstL  PofstN  QofstN   CRC_2"
#define	HDR_DATA_LIVE	"Ch  ActFwd  ActRev  ActAbs  ReaFwd  ReaRev  ReaAbs   IrmsL    Vrms   PactL PreactL Freq Hz  PfactL PangleL   PappL"
#if	(m90e26NEUTRAL > 0)
#define	HDR_DATA_NEUT	"   IrmsN   PactN PreactN  PfactN PangleN   PappN"
#else
#define	HDR_DATA_NEUT	""
#endif
#define	HDR_STATUS		"Ch  System    CRC1    CRC2  L/N Ch RevQchg RevPchg SagWarn   Meter Qnoload Pnoload    RevQ    RevP  Tamper  L-Mode"
#define	BLANK8			"        "

void m90e26ReportCalib(void) {
	printfx("%C%s%C\r\n", colourFG_CYAN, HDR_CALIB HDR_MMODE, attrRESET) ;
	for (int eChan = 0; eChan < NumM90E26; ++eChan) {
		printfx("%2d", eChan) ;
		for (int i = CALSTART; i <= CRC_1; printfx("  0x%04X", m90e26ReadU16(eChan, i++)));
		m90e26meter_mode_t	MeterMode = (m90e26meter_mode_t) m90e26ReadU16(eChan, MET_MODE) ;
		u8_t	Pthres[16] = { 200, 100, 50, 25, 16, 32, 48, 64,80, 96, 112, 128, 144, 160, 176, 192 } ;
		printfx("  %-2s  %2s %7s %7s %7s %7s  %6s %2.4f\r\n",
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

void m90e26ReportAdjust(void) {
	printfx("%C%s%C\r\n", colourFG_CYAN, HDR_ADJUST, attrRESET);
	for (int eChan = 0; eChan < NumM90E26; ++eChan) {
		printfx("%2d", eChan) ;
		for (int i = ADJSTART; i <= CRC_2; printfx("  0x%04X", m90e26ReadU16(eChan, i++))) ;
		printfx("\r\n") ;
	}
}

void m90e26ReportData(void) {
	const u8_t m90e26DataReg[] = {
		E_ACT_FWD, E_ACT_REV, E_ACT_ABS, E_REACT_FWD, E_REACT_REV, E_REACT_ABS,
		I_RMS_L, V_RMS, P_ACT_L, P_REACT_L, FREQ, P_FACTOR_L, P_ANGLE_L, P_APP_L,
		I_RMS_N, P_ACT_N, P_REACT_N, P_FACTOR_N, P_ANGLE_N, P_APP_N,
	} ;
	printfx("%C" HDR_DATA_LIVE HDR_DATA_NEUT "%C\r\n", colourFG_CYAN, attrRESET) ;
	for (int eChan = 0; eChan < NumM90E26; ++eChan) {
		printfx("%2d", eChan) ;
		for (int i = 0; i < eNUM_DATA_REG; ++i) {
			if (i < 6) {								// For energy registers reading it will reset the value...
				printfx(" %7.2g", sRTCvars.aRTCsum[eChan][i]) ;
			} else {
				printfx("  0x%04X", m90e26ReadU16(eChan, m90e26DataReg[i])) ;
			}
		}
		printfx("\r\n") ;
	}
}

void m90e26ReportStatus(void) {
	printfx("%C%s%C\r\n", colourFG_CYAN, HDR_STATUS, attrRESET) ;
	for (int eChan = 0; eChan < NumM90E26; ++eChan) {
		m90e26system_stat_t SysStatus = (m90e26system_stat_t) m90e26GetSysStatus(eChan) ;
		printfx("%2d  0x%04X", eChan, SysStatus.val) ;
		printfx(SysStatus.CalErr		? "  Error "	: BLANK8) ;
		printfx(SysStatus.AdjErr		? "  Error "	: BLANK8) ;
		printfx(SysStatus.LnChge		? " L/N chg"	: BLANK8) ;
		printfx(SysStatus.RevQchg		? " DIR chg"	: BLANK8) ;
		printfx(SysStatus.RevPchg		? " DIR chg"	: BLANK8) ;
		printfx(SysStatus.SagWarn		? "  V sag "	: BLANK8) ;
		m90e26meter_stat_t MeterStatus = (m90e26meter_stat_t) m90e26GetMeterStatus(eChan) ;
		printfx("  0x%04X", MeterStatus.val) ;
		printfx(MeterStatus.Qnoload	? " NO Load"	: BLANK8) ;
		printfx(MeterStatus.Pnoload	? " NO Load"	: BLANK8) ;
		printfx(MeterStatus.RevQ		? " Reverse"	: BLANK8) ;
		printfx(MeterStatus.RevP		? " Reverse"	: BLANK8) ;
		printfx(MeterStatus.Line 		? "    Live"	: " Neutral") ;
		printfx(MeterStatus.LNMode==3 ? "    Flex"	:
			  MeterStatus.LNMode==2 ? "    Both"	:
			  MeterStatus.LNMode==1 ? "    Live"	: "  Tamper") ;
		printfx("\r\n") ;
	}
}

void m90e26Report(void) {
	m90e26ReportCalib() ;
	m90e26ReportAdjust() ;
	m90e26ReportData() ;
	m90e26ReportStatus() ;
}

#define	m90e26STEP_CONTRAST		0x04
#define	m90e26STAT_INTVL		pdMS_TO_TICKS(2 * MILLIS_IN_SECOND)

void m90e26DisplayInfo(u8_t Index) {
	static	epw_t * psEW ;
	#if	(halHAS_M90E26 == 2)
	u8_t eChan = Index / NumM90E26 ;
	psEW = &table_work[(eChan == 0) ? URI_M90E26_E_ACT_FWD_0 : URI_M90E26_E_ACT_FWD_1] ;
	#else
	psEW = &table_work[URI_M90E26_E_ACT_FWD_0] ;
	#endif
	ssd1306SetDisplayState(1) ;
	ssd1306SetTextCursor(0, 0) ;
	if ((Index % 2) == 0) {
		devprintfx(ssd1306PutChar, "Vo%8.3f" "Fr%8.3f" "Ir%8.3f" "Pa%8.3f" "An%8.3f" "Fa%8.3f",
		xCV_GetValue(&psEW[eVOLTS].var, NULL),
		xCV_GetValue(&psEW[eFREQ].var, NULL),
		xCV_GetValue(&psEW[eI_RMS_L].var, NULL),
		xCV_GetValue(&psEW[eP_ACT_L].var, NULL),
		xCV_GetValue(&psEW[eP_ANGLE_L].var, NULL),
		xCV_GetValue(&psEW[eP_FACTOR_L].var, NULL)) ;
	} else {
		devprintfx(ssd1306PutChar, "Af%8.3f" "Ar%8.3f" "Aa%8.3f" "Rf%8.3f" "Rr%8.3f" "Ra%8.3f",
		xCV_GetValue(&psEW[eE_ACT_FWD].var, NULL),
		xCV_GetValue(&psEW[eE_ACT_REV].var, NULL),
		xCV_GetValue(&psEW[eE_ACT_ABS].var, NULL),
		xCV_GetValue(&psEW[eE_REACT_FWD].var, NULL),
		xCV_GetValue(&psEW[eE_REACT_REV].var, NULL),
		xCV_GetValue(&psEW[eE_REACT_ABS].var, NULL)) ;
	}
}

static TickType_t NextTick = 0;
static u8_t Index = 0;

void m90e26Display(void) {
	if (m90e26_handle[0] == 0)
		return;
	TickType_t CurTick = xTaskGetTickCount() ;
	if (NextTick == 0)
		NextTick = CurTick ;
	else if (NextTick > CurTick)
		return;
	#if 0
	if ((Index == 0) && m90e26Config.tBlank) {
		NextTick = CurTick + pdMS_TO_TICKS(m90e26Config.tBlank * MILLIS_IN_SECOND) ;
		ssd1306SetDisplayState(0) ;
		return;
	}
	#endif
	//
	NextTick = CurTick + m90e26STAT_INTVL ;
	if (m90e26Config.NowContrast > 0)
		m90e26DisplayInfo(Index) ;
	else
		ssd1306SetDisplayState(0);
	//
	++Index ;
	Index %= (NumM90E26 * 2) ;
	if (Index == 0)
		m90e26Config.NowContrast = ssd1306SetContrast(m90e26Config.NowContrast + m90e26STEP_CONTRAST);
}

/* ################################### OLD CODE #####################################

u16_t m90e26CalcCRC(u8_t eChan, u8_t Addr0, int8_t Count) {
	u8_t Lcrc = 0, Hcrc = 0 ;
	u16_t RegData[Count] ;
	for (int i = 0; i < Count; ++i) {				// read the range of registers
		RegData[i] = m90e26Read(eChan, Addr0 + i) ;
	}
	for (int i = 0; i < Count; ++i) {				// HI bytes: MOD256 sum & XOR
		Lcrc += RegData[i] >> 8 ;
		Hcrc ^= RegData[i] >> 8 ;
	}
	for (int i = 0; i < Count; ++i) {				// LO bytes: MOD256 sum & XOR
		Lcrc += RegData[i] & 0xFF ;
		Hcrc ^= RegData[i] & 0xFF ;
	}
	IF_P(debugCRC, "CRC=%04x from %-`h\r\n", (Hcrc << 8) | Lcrc, Count * 2, RegData) ;
	return (Hcrc << 8) | Lcrc ;
}

void	m90e26HandleCRC(u8_t eChan, u8_t RegAddr1, u8_t RegAddr2) {
	IF_myASSERT(debugPARAM, eChan<NumM90E26 && ((RegAddr1==CALSTART && RegAddr2==CRC_1) || (RegAddr1==ADJSTART && RegAddr2==CRC_2))) ;
	m90e26Write(eChan, RegAddr2, m90e26CalcCRC(eChan, RegAddr1 + 1, RegAddr2 - RegAddr1 - 1)) ;
	m90e26Write(eChan, RegAddr1, CODE_CHECK) ;
}

 */
