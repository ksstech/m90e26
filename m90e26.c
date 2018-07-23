/*
 * Copyright 2018 AM Maree/KSS Technologies (Pty) Ltd.
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

#include	"x_config.h"

#if		(halHAS_M90E26 > 0)
#include	"hal_config.h"
#include	"hal_spi.h"

#include	"x_debug.h"
#include	"x_errors_events.h"
#include	"x_ticktimer.h"					// timing debugging
#include	"x_syslog.h"
#include	"x_values_convert.h"

#include	"m90e26.h"
#include	"endpoints.h"

#include	<stdint.h>
#include	<string.h>

// ######################################### DEBUG MACROS ##########################################

#define	m90e26DEBUG					0x0000

#define	m90e26DEBUG_PARAM			(m90e26DEBUG & 0x0001)
#define	m90e26DEBUG_RESULT			(m90e26DEBUG & 0x0002)
#define	m90e26DEBUG_READ			(m90e26DEBUG & 0x0004)
#define	m90e26DEBUG_WRITE			(m90e26DEBUG & 0x0008)

#define	m90e26DEBUG_TIMING			(m90e26DEBUG & 0x0010)
#define	m90e26DEBUG_CONVERT			(m90e26DEBUG & 0x0020)

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
		.spics_io_num		= GPIO_NUM_5,				// D8 = CS pin
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
		.spics_io_num		= GPIO_NUM_17,				// D3 = CS pin
		.flags				= 0,
		.queue_size			= 16,
		.pre_cb				= 0,						// no callback handler
		.post_cb			= 0,
	},
#endif
} ;

spi_device_handle_t				m90e26_handle[halHAS_M90E26] ;

const conf_reg_t CalibMeter[] = {
#if		(M90E26_CALIB_TABLE == 0)					// AMM values
	{ SOFTRESET,	0,	RSTCOD },
	{ CALSTART,		0,	STDCOD },
	{ PLconstH,		0,	0x00B9 },
	{ PLconstL, 	0,	0xC1F3 },
	{ P_NOL_TH,		0,	0x8100 },					// Guess
	{ Q_NOL_TH,		0,	0x8100 },					// Guess

#elif	(M90E26_CALIB_TABLE == 1)					// Tisham values
	{ SOFTRESET,	0,	RSTCOD },
	{ FUNC_ENAB,	0,	0x0030 },
	{ V_SAG_THR,	0,	0x1F2F },
	{ CALSTART,		0,	STDCOD },
	{ PLconstH,		0,	0x00B9 },
	{ PLconstL, 	0,	0xC1F3 },
	{ L_GAIN,		0,	0x1D39 },
	{ P_SUP_TH,		0,	0x08BD },
	{ Q_SUP_TH,		0,	0x0AEC },

#elif	(M90E26_CALIB_TABLE == 2)					// defaults
	{ SOFTRESET,	0,	RSTCOD },
	{ FUNC_ENAB,	0,	0x000C },	// 0x003C = enable Sag, SagWarn, RevAct & RevReAct
	{ V_SAG_THR,	0,	0x1D64 },
	{ POWER_MODE,	0,	0x0000 },
	{ CALSTART,		0,	STDCOD },
	{ PLconstH,		0,	0x0015 },
	{ PLconstL, 	0,	0xD174 },
	{ L_GAIN,		0,	0x0000 },
	{ L_PHI,		0,	0x0000 },
	{ N_GAIN,		0,	0x0000 },
	{ N_PHI,		0,	0x0000 },
	{ P_SUP_TH,		0,	0x08BD },
	{ P_NOL_TH,		0,	0x0000 },
	{ Q_SUP_TH,		0,	0x0AEC },
	{ Q_NOL_TH,		0,	0x0000 },
	{ MET_MODE,		0,	0x9422 },	// 9422	Lgain=1, Ngain=1, LNSel=1(L), DisHPF=EnableHPF0+1
									// Amod=Fwd+Rev Energy Pulse, Rmod=Fwd+Rev Energy Pulse
									// Zxcon=All(pos+neg), Pthresh=3.125%
#else
	#error "Invalid calibration table value specified !!!"
#endif
} ;

const conf_reg_t CalibMeasure[] = {
#if		(M90E26_CALIB_TABLE == 0)					// AMM values
	{ ADJSTART,		0,	STDCOD },
	{ V_GAIN,		0,	0x6C50 },	// OK
	{ I_GAIN_L,		0,	0x7C2A },	// OK
	#if	(M90E26_CALIB_SOFT == 0)
	{ I_OFST_L,		0,	0xF800 },	// Trial & Error
	{ I_OFST_N,		0,	0xF400 },	// Trial & Error
	{ P_OFST_L,		0,	0xFFFF },	// CALCULATED
	{ Q_OFST_L,		0,	0xFFFF },	// CALCULATED
	{ P_OFST_N,		0,	0xFFFF },	// CALCULATED
	{ Q_OFST_N,		0,	0xFFFF },	// CALCULATED
	#endif

#elif	(M90E26_CALIB_TABLE == 1)					// Tisham values
	{ ADJSTART,		0,	STDCOD },
	{ V_GAIN,		0,	0xD464 },
	{ I_GAIN_L,		0,	0x6E49 },

#elif	(M90E26_CALIB_TABLE == 2)					// defaults
	{ ADJSTART,		0,	STDCOD },
	{ V_GAIN,		0,	0x6720 },
	{ I_GAIN_L,		0,	0x7A13 },
	{ I_GAIN_N,		0,	0x7530 },
	{ V_OFFSET,		0,	0x0000 },
	{ I_OFST_L,		0,	0x0000 },
	{ I_OFST_N,		0,	0x0000 },
	{ P_OFST_L,		0,	0x0000 },
	{ Q_OFST_L,		0,	0x0000 },
	{ P_OFST_N,		0,	0x0000 },
	{ Q_OFST_N,		0,	0x0000 },
#else
	#error "Invalid calibration table value specified !!!"
#endif
} ;

// ###################################### Private functions ########################################

void	m90e26Write(uint8_t eChan, uint8_t address, uint16_t val) {
	spi_transaction_t m90e26_buf ;
	memset(&m90e26_buf, 0, sizeof(m90e26_buf));
	m90e26_buf.length		= 8 * 3;
	m90e26_buf.flags 		= SPI_TRANS_USE_TXDATA ;
	m90e26_buf.tx_data[0]	= address ;
	m90e26_buf.tx_data[1]	= val >> 8 ;
	m90e26_buf.tx_data[2]	= val & 0xFF ;
	ESP_ERROR_CHECK(spi_device_transmit(m90e26_handle[eChan], &m90e26_buf)) ;
	IF_PRINT(m90e26DEBUG_WRITE, "TX: addr=%02x d0=%02x d1=%02x\n", m90e26_buf.tx_data[0], m90e26_buf.tx_data[1], m90e26_buf.tx_data[2]) ;
}

uint16_t m90e26Read(uint8_t eChan, uint8_t address) {
	spi_transaction_t m90e26_buf ;
	memset(&m90e26_buf, 0, sizeof(m90e26_buf)) ;
	m90e26_buf.length		= 8 * 3 ;
	m90e26_buf.flags 		= SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA ;
	m90e26_buf.tx_data[0]	= address | 0x80 ;
	ESP_ERROR_CHECK(spi_device_transmit(m90e26_handle[eChan], &m90e26_buf)) ;
	IF_PRINT(m90e26DEBUG_READ, "RX: addr=%02x d0=%02x d1=%02x\n", m90e26_buf.tx_data[0], m90e26_buf.rx_data[1], m90e26_buf.rx_data[2]) ;
	return (m90e26_buf.rx_data[1] << 8) | m90e26_buf.rx_data[2] ;
}

uint16_t m90e26ReadModifyWrite(uint8_t eChan, uint8_t Addr, uint16_t Value, uint16_t Mask) {
uint16_t CurValue = m90e26Read(eChan, Addr) ;
	CurValue &= ~Mask ;
	CurValue |= Value ;
	m90e26Write(eChan, Addr, CurValue) ;
	return CurValue ;
}

// ############################## initialization and (re)configuration #############################

void	m90e26SetCurrentOffset(uint8_t eChan, uint8_t RegRMS, uint8_t RegGAIN, uint8_t RegOFST) {
	uint16_t CurAmps, CurGain ;
	uint32_t Factor1, Factor2 ;

	CurAmps = m90e26Read(eChan, RegRMS) ;
	CurGain = m90e26Read(eChan, RegGAIN) ;
	Factor1 = (CurAmps * CurGain) / 2^8 ;
//	Factor2 = ~Factor1 >> 16 ;
	Factor2 = ~Factor1 & 0x0000FFFF ;
	m90e26Write(eChan, RegOFST, Factor2) ;
	PRINT("Ch %d: Il=0x%04x  Gl=0x%04x  F1=0x%08x  F2=0x%04x\n", eChan, CurAmps, CurGain, Factor1, Factor2) ;
}

void	m90e26SetPowerOffset(uint8_t eChan, uint8_t RegPOWER, uint8_t RegOFST) {
	uint32_t SumOffset = 0 ;
	for (int32_t i = 0; i < M90E26_CALIB_ITER; i++)		SumOffset += m90e26Read(eChan, RegPOWER) ;
	uint16_t NewOffset = ~(SumOffset / M90E26_CALIB_ITER) ;
	m90e26Write(eChan, RegOFST, NewOffset) ;
	PRINT("Ch %d: Rr=%02x  Rw=%02x  %dx  Sum=0x%08x  Ofst=0x%04x\n", eChan, RegPOWER, RegOFST, M90E26_CALIB_ITER, SumOffset, NewOffset) ;
}

/**
 * m90e26CalcCRC() - calculate the CRC for a range of registers
 */
uint16_t m90e26CalcCRC(uint8_t eChan, uint8_t Addr0, int8_t Count) {
	uint8_t Lcrc = 0, Hcrc = 0 ;
	uint16_t RegData[Count] ;
	for (int32_t i = 0; i < Count; i++) {				// read the range of registers
		RegData[i] = m90e26Read(eChan, Addr0 + i) ;
	}
	for (int32_t i = 0; i < Count; i++) {				// HI bytes: MOD256 sum & XOR
		Lcrc += RegData[i] >> 8 ;
		Hcrc ^= RegData[i] >> 8 ;
	}
	for (int32_t i = 0; i < Count; i++) {				// LO bytes: MOD256 sum & XOR
		Lcrc += RegData[i] & 0xFF ;
		Hcrc ^= RegData[i] & 0xFF ;
	}
	IF_PRINT(m90e26DEBUG_RESULT, "CRC=%04x from %-'h", (Hcrc << 8) | Lcrc, Count * 2, RegData) ;
	return (Hcrc << 8) | Lcrc ;
}

/**
 * m90e26HandleCRC() - Calculate the CRC from the range of registers & write the lock register/pattern
 */
void	m90e26HandleCRC(uint8_t eChan, uint8_t RegAddr1, uint8_t RegAddr2) {
	IF_myASSERT(m90e26DEBUG_PARAM, (eChan < halHAS_M90E26) && ((RegAddr1==CALSTART && RegAddr2==CRC_1) || (RegAddr1==ADJSTART && RegAddr2==CRC_2))) ;
	m90e26Write(eChan, RegAddr2, m90e26CalcCRC(eChan, RegAddr1 + 1, RegAddr2 - RegAddr1 - 1)) ;
	m90e26Write(eChan, RegAddr1, CFGCOD) ;
}

int32_t	m90e26Init(uint8_t eChan) {
	IF_myASSERT(m90e26DEBUG_PARAM, (eChan < halHAS_M90E26)) ;
	ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &m90e26_config[eChan], &m90e26_handle[eChan])) ;

	// write the configuration registers with METER calibration data
	for (int32_t i = 0; i < NUM_OF_MEMBERS(CalibMeter); i++)	m90e26Write(eChan, CalibMeter[i].addr, CalibMeter[i].raw_val) ;
	m90e26HandleCRC(eChan, CALSTART, CRC_1) ;

#if		(M90E26_CALIB_TABLE == 0) && (M90E26_CALIB_SOFT == 1)
	// write the configuration registers with MEASURE calibration data
	for (int32_t i = 0; i < NUM_OF_MEMBERS(CalibMeasure); i++)	m90e26Write(eChan, CalibMeasure[i].addr, CalibMeasure[i].raw_val) ;
	/* Preference is to fix this functionality, then hard code the values into the
	 * initialization table and do it that way. Alternative would be to run this
	 * ONLY if calibration values cannot be found in NVS, same as WIFI credentials */
	PRINT("Ch %d: Offset Compensation start, DISCONNECT CT's\n", eChan) ;

	/* LIVE & NEUTRAL Current Offset calibration not working properly
	 * The formula as described in Atmel-46102-SE-M90E26-ApplicationNote.pdf
	 * is NOT clear on the calculation and does not yield proper values  */
	m90e26SetCurrentOffset(eChan, I_RMS_L, I_GAIN_L, I_OFST_L) ;
	#if	(halUSE_M90E26_NEUTRAL == 1)
	m90e26SetCurrentOffset(eChan, I_RMS_N, I_GAIN_N, I_OFST_N) ;
	#endif

	/* [Re]Active, LINE & NEUTRAL Power Offset calibration
	 * Not sure if the power register should be read whilst in normal or
	 * adjustment mode. Currently no real value is being read */
	m90e26Write(eChan, POWER_MODE, PWRCOD) ;			// set into low power mode for calibration
	m90e26SetPowerOffset(eChan, P_ACT_L, P_OFST_L) ;	// L Line Active Power Offset
	m90e26SetPowerOffset(eChan, P_REACT_L, Q_OFST_L) ;	// L Line ReActive Power Offset
	#if	(halUSE_M90E26_NEUTRAL == 1)
	m90e26SetPowerOffset(eChan, P_ACT_N, P_OFST_N) ;	// N Line Active Power Offset
	m90e26SetPowerOffset(eChan, P_REACT_N, Q_OFST_N) ;	// N Line ReActive Power Offset
	#endif
	m90e26Write(eChan, POWER_MODE, RSTCOD) ;			// reset to normal power mode

	PRINT("Ch %d: Offset Compensation done, RECONNECT CT's\n", eChan) ;
#else
	// write the configuration registers with MEASURE calibration data
	for (int32_t i = 0; i < NUM_OF_MEMBERS(CalibMeasure); i++)	m90e26Write(eChan, CalibMeasure[i].addr, CalibMeasure[i].raw_val) ;
#endif
	m90e26HandleCRC(eChan, ADJSTART, CRC_2) ;

	return (m90e26GetSysStatus(eChan) & 0xF000) ? erFAILURE : erSUCCESS ;
}

// ############################# endpoint support functions ########################################

// Energy returns kWh if PL constant set to 1000imp/kWh, cleared after reading
#ifndef	M90E26_ENERGY_SCALE
//	#define	M90E26_ENERGY_SCALE		10000.0			// default to KWHr
	#define	M90E26_ENERGY_SCALE		10.0			// default to WHr
#endif

// Common support routines
int32_t	m90e26ReadEnergy(ep_work_t * pEpWork, uint8_t RegAddr, uint8_t eUri) {
	if (pEpWork->Var.varDef.fl_sum) {					// if just a normal update cycle
		uint8_t	eChan	= (pEpWork == &table_work[eUri]) ? 0 : 1 ;
		uint16_t RawVal	= m90e26Read(eChan, RegAddr) ;
		float f32Val	= (float) RawVal / M90E26_ENERGY_SCALE ;
		xEpSetValue(pEpWork, (x32_t) f32Val) ;
	} else {											// else it is a reset/set call
		pEpWork->Var.varDef.fl_sum		= 1 ;			// and set the flag
	}
	return erSUCCESS ;
}

int32_t	m90e26ReadCurrent(ep_work_t * pEpWork, uint8_t RegAddr, uint8_t eUri) {
	uint8_t	eChan	= (pEpWork == &table_work[eUri]) ? 0 : 1 ;
	uint16_t RawVal	= m90e26Read(eChan, RegAddr) ;
	float f32Val	= (float) RawVal ;
#if		(M90E26_RESOLUTION == 1)
	RawVal	= m90e26Read(eChan, eLSB) ;
	f32Val	+= (float) RawVal / 65536 ;
#endif
	f32Val	/= 1000 ;
	xEpSetValue(pEpWork, (x32_t) f32Val) ;
	return erSUCCESS ;
}

int32_t	m90e26ReadVolts(ep_work_t * pEpWork, uint8_t RegAddr, uint8_t eUri) {
	uint8_t	eChan	= (pEpWork == &table_work[eUri]) ? 0 : 1 ;
	uint16_t RawVal	= m90e26Read(eChan, RegAddr) ;
	float f32Val	= (float) RawVal ;
#if		(M90E26_RESOLUTION == 1)
	RawVal	= m90e26Read(eChan, eLSB) ;
	f32Val	+= (float) RawVal / 65536 ;
#endif
	f32Val	/= 100.0 ;
	xEpSetValue(pEpWork, (x32_t) f32Val) ;
	return erSUCCESS ;
}

int32_t	m90e26ReadPower(ep_work_t * pEpWork, uint8_t RegAddr, uint8_t eUri) {
	uint8_t	eChan	= (pEpWork == &table_work[eUri]) ? 0 : 1 ;
	uint16_t RawVal	= m90e26Read(eChan, RegAddr) ;
	float f32Val	= (float) xConvert2sComp(RawVal, 16) ;
#if		(M90E26_RESOLUTION == 1)
	RawVal	= m90e26Read(eChan, eLSB) ;
	f32Val	+= (float) RawVal / 65536 ;
#endif
	f32Val	/= 1000.0 ;
	xEpSetValue(pEpWork, (x32_t) f32Val) ;
	return erSUCCESS ;
}

int32_t	m90e26ReadFrequency(ep_work_t * pEpWork, uint8_t RegAddr, uint8_t eUri) {
	uint8_t	eChan	= (pEpWork == &table_work[eUri]) ? 0 : 1 ;
	uint16_t RawVal	= m90e26Read(eChan, RegAddr) ;
	float f32Val	= (float) RawVal / 100.0 ;
	xEpSetValue(pEpWork, (x32_t) f32Val) ;
	IF_PRINT(m90e26DEBUG_CONVERT, "Ch %d: eReg:%02X  Raw:0x%04X  Val:%9.3f\r", eChan, RegAddr, RawVal, f32Val) ;
	return erSUCCESS ;
}

int32_t	m90e26ReadPowerFactor(ep_work_t * pEpWork, uint8_t RegAddr, uint8_t eUri) {
	uint8_t	eChan	= (pEpWork == &table_work[eUri]) ? 0 : 1 ;
	uint16_t RawVal	= m90e26Read(eChan, RegAddr) ;
	int32_t ConvVal = RawVal & 0x8000 ? -1 * (RawVal & 0x7FFF) : RawVal ;
	float f32Val	= (float) ConvVal / 1000.0 ;
	xEpSetValue(pEpWork, (x32_t) f32Val) ;
	return erSUCCESS ;
}

int32_t	m90e26ReadPowerAngle(ep_work_t * pEpWork, uint8_t RegAddr, uint8_t eUri) {
	uint8_t	eChan	= (pEpWork == &table_work[eUri]) ? 0 : 1 ;
	uint16_t RawVal	= m90e26Read(eChan, RegAddr) ;
	int32_t ConvVal = RawVal & 0x8000 ? -1 * (RawVal & 0x7FFF) : RawVal ;
	float f32Val	= (float) ConvVal / 10.0 ;
	xEpSetValue(pEpWork, (x32_t) f32Val) ;
	return erSUCCESS ;
}

// Energy wrapper functions
int32_t	m90e26Read_E_ACT_FWD(ep_work_t * pEpWork)	{ return m90e26ReadEnergy(pEpWork, E_ACT_FWD, URI_M90E26_E_ACT_FWD_0) ; }
int32_t	m90e26Read_E_ACT_REV(ep_work_t * pEpWork)	{ return m90e26ReadEnergy(pEpWork, E_ACT_REV, URI_M90E26_E_ACT_REV_0) ; }
int32_t	m90e26Read_E_ACT_ABS(ep_work_t * pEpWork)	{ return m90e26ReadEnergy(pEpWork, E_ACT_ABS, URI_M90E26_E_ACT_ABS_0) ; }
int32_t	m90e26Read_E_REACT_FWD(ep_work_t * pEpWork)	{ return m90e26ReadEnergy(pEpWork, E_REACT_FWD, URI_M90E26_E_REACT_FWD_0) ; }
int32_t	m90e26Read_E_REACT_REV(ep_work_t * pEpWork)	{ return m90e26ReadEnergy(pEpWork, E_REACT_REV, URI_M90E26_E_REACT_REV_0) ; }
int32_t	m90e26Read_E_REACT_ABS(ep_work_t * pEpWork)	{ return m90e26ReadEnergy(pEpWork, E_REACT_REV, URI_M90E26_E_REACT_ABS_0) ; }
// LIVE Line wrapper functions
int32_t	m90e26Read_I_RMS_L(ep_work_t * pEpWork)		{ return m90e26ReadCurrent(pEpWork, I_RMS_L, URI_M90E26_I_RMS_L_0) ; }
int32_t	m90e26Read_V_RMS(ep_work_t * pEpWork)		{ return m90e26ReadVolts(pEpWork, V_RMS, URI_M90E26_V_RMS_0) ; }
int32_t	m90e26Read_P_ACT_L(ep_work_t * pEpWork)		{ return m90e26ReadPower(pEpWork, P_ACT_L, URI_M90E26_P_ACT_L_0) ; }
int32_t	m90e26Read_P_REACT_L(ep_work_t * pEpWork)	{ return m90e26ReadPower(pEpWork, P_REACT_L, URI_M90E26_P_REACT_L_0) ; }
int32_t	m90e26Read_FREQ(ep_work_t * pEpWork)		{ return m90e26ReadFrequency(pEpWork, FREQ, URI_M90E26_FREQ_0) ; }
int32_t	m90e26Read_P_FACTOR_L(ep_work_t * pEpWork)	{ return m90e26ReadPowerFactor(pEpWork, P_FACTOR_L, URI_M90E26_P_FACTOR_L_0) ; }
int32_t	m90e26Read_P_ANGLE_L(ep_work_t * pEpWork)	{ return m90e26ReadPowerAngle(pEpWork, P_ANGLE_L, URI_M90E26_P_ANGLE_L_0) ; }
int32_t	m90e26Read_P_APP_L(ep_work_t * pEpWork)		{ return m90e26ReadPower(pEpWork, P_APP_L, URI_M90E26_P_APP_L_0) ; }

#if		(halUSE_M90E26_NEUTRAL == 1)		// NEUTRAL Line wrapper functions
int32_t	m90e26Read_I_RMS_N(ep_work_t * pEpWork)		{ return m90e26ReadCurrent(pEpWork, I_RMS_N, URI_M90E26_I_RMS_N_0) ; }
int32_t	m90e26Read_P_ACT_N(ep_work_t * pEpWork)		{ return m90e26ReadPower(pEpWork, P_ACT_N, URI_M90E26_P_ACT_N_0) ; }
int32_t	m90e26Read_P_REACT_N(ep_work_t * pEpWork)	{ return m90e26ReadPower(pEpWork, P_REACT_N, URI_M90E26_P_REACT_N_0) ; }
int32_t	m90e26Read_P_FACTOR_N(ep_work_t * pEpWork)	{ return m90e26ReadPowerFactor(pEpWork, P_FACTOR_N, URI_M90E26_P_FACTOR_N_0) ; }
int32_t	m90e26Read_P_ANGLE_N(ep_work_t * pEpWork)	{ return m90e26ReadPowerAngle(pEpWork, P_ANGLE_N, URI_M90E26_P_ANGLE_N_0) ; }
int32_t	m90e26Read_P_APP_N(ep_work_t * pEpWork)		{ return m90e26ReadPower(pEpWork, P_APP_N, URI_M90E26_P_APP_N_0) ; }
#endif

int32_t	m90e26SetLiveGain(uint8_t eChan, uint8_t Gain) {
	uint16_t	NewValue ;
	switch (Gain) {
	case 1:		NewValue	= 0x8000 ;	break ;
	case 4:		NewValue	= 0x0000 ;	break ;
	case 8:		NewValue	= 0x2000 ;	break ;
	case 16:	NewValue	= 0x4000 ;	break ;
	case 24:	NewValue	= 0x6000 ;	break ;
	default:	IF_SL_ERR(m90e26DEBUG_PARAM, "Invalid Live Gain =%d", Gain) ; return erFAILURE ;
	}
	NewValue = m90e26ReadModifyWrite(eChan, MET_MODE, NewValue, 0xE000) ;
	m90e26HandleCRC(eChan, CALSTART, CRC_1) ;
	return erSUCCESS ;
}

#if		(halUSE_M90E26_NEUTRAL == 1)		// Neutral Line
int32_t	m90e26SetNeutralGain(uint8_t eChan, uint8_t Gain) {
	uint16_t	NewValue ;
	switch (Gain) {
	case 1:		NewValue	= 0x1000 ;	break ;
	case 2:		NewValue	= 0x0000 ;	break ;
	case 4:		NewValue	= 0x0800 ;	break ;
	default:	 IF_SL_ERR(m90e26DEBUG_PARAM, "Invalid Neutral Gain =%d", Gain) ; return erFAILURE ;
	}
	NewValue = m90e26ReadModifyWrite(eChan, MET_MODE, NewValue, 0x1800) ;
	m90e26HandleCRC(eChan, CALSTART, CRC_1) ;
	return erSUCCESS ;
}
#endif

inline uint16_t m90e26GetSysStatus(uint8_t eChan)	{ return m90e26Read(eChan, SYS_STATUS) ; }
inline uint16_t m90e26GetMeterStatus(uint8_t eChan)	{ return m90e26Read(eChan, MET_STATUS) ; }
inline uint16_t m90e26GetLastData(uint8_t eChan)	{ return m90e26Read(eChan, LASTDATA) ; }

// ############################### device reporting functions ######################################

void	m90e26ReportStatus(uint8_t eChan) {
	sysstatus_t SysStatus = (sysstatus_t) m90e26GetSysStatus(eChan) ;
	PRINT("Ch %d :  SystemStatus %04X", eChan, SysStatus.val) ;
	if (SysStatus.CalErr)		PRINT("\tCRC_1 Error!!") ;
	if (SysStatus.AdjErr)		PRINT("\tCRC_2 Error!!") ;
	if (SysStatus.LnChge)		PRINT("\tMetering line L<>N change!!") ;
	if (SysStatus.RevQchg)		PRINT("\tReactive Energy DIR change!!") ;
	if (SysStatus.RevPchg)		PRINT("\tActive Energy DIR change!!") ;
	if (SysStatus.SagWarn)		PRINT("\tVoltage SAG") ;
	PRINT("\n") ;

	enstatus_t MeterStatus = (enstatus_t) m90e26GetMeterStatus(eChan) ;
	PRINT("Ch %d :  MeterStatus %04X\n", eChan, MeterStatus.val) ;
	if (MeterStatus.Qnoload)	PRINT("\tReActive NO Load  ") ;
	if (MeterStatus.Pnoload)	PRINT("\tActive NO Load  ") ;
	if (MeterStatus.RevQ)		PRINT("\tReActive Reverse  ") ;
	if (MeterStatus.RevP)		PRINT("\tActive Reverse  ") ;
	PRINT("Tamper %s  ", MeterStatus.Line ? "Live" : "Neutral") ;
	PRINT("LineMode (%d) %s\n", MeterStatus.LNMode,
								(MeterStatus.LNMode) == 0x3 ? "Flexible" :
								(MeterStatus.LNMode) == 0x2 ? "L+N" :
								(MeterStatus.LNMode) == 0x1 ? "L only" : "AntiTamper") ;
}

void	m90e26ReportCalib(uint8_t eChan) {
	PRINT("Ch %d:  CALSTRT  PLconsH  PLconsL    Lgain     Lphi    Ngain     Nphi  PStrtTh   PNolTh  QStrtTh   QNolTh    MMode    CRC_1\n     ", eChan) ;
	for(int32_t i = CALSTART; i <= CRC_1; i++)	PRINT("   0x%04X", m90e26Read(eChan, i)) ;
	PRINT("\n") ;
}

void	m90e26ReportAdjust(uint8_t eChan) {
	PRINT("Ch %d:  ADJSTRT    Vgain   IgainL   IgainN    Vofst   IofstL   IofstN   PofstL   QofstL   PofstN   QofstN    CRC_2\n     ", eChan) ;
	for(int32_t i = ADJSTART; i <= CRC_2; i++)	PRINT("   0x%04X", m90e26Read(eChan, i)) ;
	PRINT("\n") ;
}

#define	M90E26_DATA_BASE_HEADING			"Ch %d:   ActFwd   ActRev   ActAbs   ReaFwd   ReaRev   ReaAbs    IrmsL     Vrms    PactL  PreactL     Freq   PfactL  PangleL    PappL"

#if		(halUSE_M90E26_NEUTRAL == 1)					// Neutral Line
	#define	M90E26_DATA_HEADING_NEUTRAL		"    IrmsN    PactN  PreactN   PfactN  PangleN    PappN"
#else
	#define	M90E26_DATA_HEADING_NEUTRAL		""
#endif

#if		(M90E26_RESOLUTION == 1)						// LSB
	#define	M90E26_DATA_HEADING_LSB			"      LSB"
#else
	#define	M90E26_DATA_HEADING_LSB			""
#endif

#if		(M90E26_LAST_DATA == 1)							// LAST_DATA
	#define	M90E26_DATA_HEADING_LASTDATA	" LastData"
#else
	#define	M90E26_DATA_HEADING_LASTDATA	""
#endif

static const uint8_t m90e26DataReg[] = {
	E_ACT_FWD, E_ACT_REV, E_ACT_ABS, E_REACT_FWD, E_REACT_REV, E_REACT_ABS, I_RMS_L, V_RMS, P_ACT_L, P_REACT_L, FREQ, P_FACTOR_L, P_ANGLE_L, P_APP_L,
#if		(halUSE_M90E26_NEUTRAL == 1)
	I_RMS_N, P_ACT_N, P_REACT_N, P_FACTOR_N, P_ANGLE_N, P_APP_N,
#endif
#if		(M90E26_RESOLUTION == 1)
 	LSB,
#endif
#if		(M90E26_LAST_DATA == 1)
	LASTDATA,
#endif
} ;

void	m90e26ReportData(uint8_t eChan) {
	PRINT(M90E26_DATA_BASE_HEADING M90E26_DATA_HEADING_NEUTRAL M90E26_DATA_HEADING_LSB M90E26_DATA_HEADING_LASTDATA "\n     ", eChan) ;
	for (int32_t i = 0; i < eNUM_DATA_REG; i++)		PRINT("   0x%04X", m90e26Read(eChan, m90e26DataReg[i])) ;	PRINT("\n") ;
}

void	m90e26Report(void) {
	for (int32_t eChan = 0; eChan < halHAS_M90E26; eChan++) {
		m90e26ReportCalib(eChan) ;
		m90e26ReportAdjust(eChan) ;
		m90e26ReportData(eChan) ;
		m90e26ReportStatus(eChan) ;
	}
}

#if		(halHAS_SSD1306 > 0)
#include	"ssd1306/ssd1306.h"
static	uint8_t Index = 0 ;
static	TickType_t PrevTick = 0 ;
void	m90e26Display(void) {
	if (m90e26_handle[0] == 0) {
		return ;
	}
	TickType_t CurTicks = xTaskGetTickCount() ;
	if ((CurTicks - PrevTick) < M90E26_STAT_INTVL) {	// enough time elapsed ?
		return ;										// nope, return
	}
	PrevTick = CurTicks ;								// yes, save current timestamp
	uint8_t eChan = Index / halHAS_M90E26 ;
	ssd1306SetTextCursor(&sSSD1306, 0, 0) ;
	ep_work_t * pEpWork = &table_work[eChan == 0 ? URI_M90E26_E_ACT_FWD_0 : URI_M90E26_E_ACT_FWD_1] ;
	if ((Index % 2) == 0) {
		devprintf(ssd1306PutC, "V:%8.3fF:%8.3fI:%8.3fAP%8.3fPa%8.3fPf%8.3f",
			xCompVarGetValue(&pEpWork[eVOLTS].Var, NULL), xCompVarGetValue(&pEpWork[eFREQ].Var, NULL),
			xCompVarGetValue(&pEpWork[eI_RMS_L].Var, NULL), xCompVarGetValue(&pEpWork[eP_ACT_L].Var, NULL),
			xCompVarGetValue(&pEpWork[eP_ANGLE_L].Var, NULL), xCompVarGetValue(&pEpWork[eP_FACTOR_L].Var, NULL)) ;
	} else {
		devprintf(ssd1306PutC, "Af%8.3fAr%8.3fAa%8.3fRf%8.3fRr%8.3fRa%8.3f",
			xCompVarGetValue(&pEpWork[eE_ACT_FWD].Var, NULL), xCompVarGetValue(&pEpWork[eE_ACT_REV].Var, NULL),
			xCompVarGetValue(&pEpWork[eE_ACT_ABS].Var, NULL), xCompVarGetValue(&pEpWork[eE_REACT_FWD].Var, NULL),
			xCompVarGetValue(&pEpWork[eE_REACT_REV].Var, NULL), xCompVarGetValue(&pEpWork[eE_REACT_ABS].Var, NULL)) ;
	}
	Index++ ;
	Index %= (halHAS_M90E26 * 2) ;
}
#endif

// ################################## Diagnostics functions ########################################

#endif
