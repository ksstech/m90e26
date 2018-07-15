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

#if		(ESP32_PLATFORM == 1)
	#include	"esp_freertos_hooks.h"
#endif

#if		(halHAS_SSD1306 > 0)
	#include	"ssd1306/ssd1306.h"
#endif

#include	<stdint.h>
#include	<string.h>

// ######################################### DEBUG MACROS ##########################################

#define	m90e26DEBUG					0x0000

#define	m90e26DEBUG_PARAM			(m90e26DEBUG & 0x0001)
#define	m90e26DEBUG_RESULT			(m90e26DEBUG & 0x0002)
#define	m90e26DEBUG_READ			(m90e26DEBUG & 0x0004)
#define	m90e26DEBUG_WRITE			(m90e26DEBUG & 0x0008)

#define	m90e26DEBUG_TIMING			(m90e26DEBUG & 0x0010)

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

const conf_reg_t RegConfig[] = {
#if 1
	{ SOFTRESET,	0,	RSTCOD },
//	{ FUNC_ENAB,	0,	0x003C },	// enable Sag, SagWarn, RevAct & RevReAct
//	{ V_SAG_THR,	0,	0x1F2F },
//	{ POWER_MODE,	0,	0x0000 },
//	metering calibration
	{ PLconstH,		0,	0x00B9 },
	{ PLconstL, 	0,	0xC1F3 },
	{ Lgain,		0,	0x1D39 },
//	{ Lphi			0,	0x0000 },
	{ PStartTh,		0,	0x08BD },
//	{ PNolTh,		0,	0x0000 },
	{ QStartTh,		0,	0x0AEC },
//	{ QNolTh,		0,	0x0000 },
//	{ MMode,		0,	0x9422 },
//	measurement calibration
	{ V_GAIN,		0,	0x6C50 },	// OK
	{ I_GAIN_L,		0,	0x7A45 },	// OK
#elif 0					// Tisham values
	{ SOFTRESET,	0,	RSTCOD },
	{ FUNC_ENAB,	0,	0x0030 },
	{ V_SAG_THR,	0,	0x1F2F },
//	{ POWER_MODE,	0,	0x0000 },
//	metering calibration
	{ PLconstH,		0,	0x00B9 },
	{ PLconstL, 	0,	0xC1F3 },
	{ Lgain,		0,	0x1D39 },
	{ Lphi,			0,	0x0000 },
	{ PStartTh,		0,	0x08BD },
	{ PNolTh,		0,	0x0000 },
	{ QStartTh,		0,	0x0AEC },
	{ QNolTh,		0,	0x0000 },
	{ MMode,		0,	0x9422 },
//	measurement calibration
	{ V_GAIN,		0,	0xD464 },
	{ I_GAIN_L,		0,	0x6E49 },
	{ V_OFFSET,		0,	0x0000 },
	{ I_OFST_L,		0,	0x0000 },
	{ P_OFST_L,		0,	0x0000 },
	{ Q_OFST_L,		0,	0x0000 },
#else					// defaults
	{ SOFTRESET,	0,	RSTCOD },
//	{ FUNC_ENAB,	0,	0x000C },
//	{ V_SAG_THR,	0,	0x1D64 },
//	{ POWER_MODE,	0,	0x0000 },
//	metering calibration
//	{ PLconstH,		0,	0x0015 },
//	{ PLconstL, 	0,	0xD174 },
//	{ Lgain,		0,	0x0000 },
//	{ Lphi			0,	0x0000 },
//	{ Ngain,		0,	0x0000 },
//	{ Nphi			0,	0x0000 },
//	{ PStartTh,		0,	0x08BD },
//	{ PNolTh,		0,	0x0000 },
//	{ QStartTh,		0,	0x0AEC },
//	{ QNolTh,		0,	0x0000 },
//	{ MMode,		0,	0x9422 },	// 9422	Lgain=1, Ngain=1, LNSel=1(L), DisHPF=EnableHPF0+1
									// Amod=Fwd+Rev Energy Pulse, Rmod=Fwd+Rev Energy Pulse
									// Zxcon=All(pos+neg), Pthresh=3.125%
//	measurement calibration
//	{ V_GAIN,		0,	0x6720 },
//	{ I_GAIN_L,		0,	0x7A13 },
//	{ I_GAIN_N,		0,	0x7530 },
//	{ V_OFFSET,		0,	0x0000 },
//	{ I_OFST_L,		0,	0x0000 },
//	{ I_OFST_N,		0,	0x0000 },
//	{ P_OFST_L,		0,	0x0000 },
//	{ Q_OFST_L,		0,	0x0000 },
//	{ P_OFST_N,		0,	0x0000 },
//	{ Q_OFST_N,		0,	0x0000 },
#endif
} ;

data_reg_t	m90e26Data[] = {
	[eE_ACT_FWD]	=	MAKE_DATA_REG(E_ACT_FWD)
	[eE_ACT_REV]	=	MAKE_DATA_REG(E_ACT_REV)
	[eE_ACT_ABS]	=	MAKE_DATA_REG(E_ACT_ABS)
	[eE_REACT_FWD]	=	MAKE_DATA_REG(E_REACT_FWD)
	[eE_REACT_REV]	=	MAKE_DATA_REG(E_REACT_REV)
	[eE_REACT_ABS]	=	MAKE_DATA_REG(E_REACT_ABS)

	[eI_RMS_L]		=	MAKE_DATA_REG(I_RMS_L)
	[eVOLTS]		=	MAKE_DATA_REG(V_RMS)
	[eP_ACT_L]		=	MAKE_DATA_REG(P_ACT_L)
	[eP_REACT_L]	=	MAKE_DATA_REG(P_REACT_L)
	[eFREQ]			=	MAKE_DATA_REG(FREQ)
	[eP_FACTOR_L]	=	MAKE_DATA_REG(P_FACTOR_L)
	[eP_ANGLE_L]	=	MAKE_DATA_REG(P_ANGLE_L)
	[eP_APP_L]		=	MAKE_DATA_REG(P_APP_L)

	[eI_RMS_N]		=	MAKE_DATA_REG(I_RMS_N)
	[eP_ACT_N]		=	MAKE_DATA_REG(P_ACT_N)
	[eP_REACT_N]	=	MAKE_DATA_REG(P_REACT_N)
	[eP_FACTOR_N]	=	MAKE_DATA_REG(P_FACTOR_N)
	[eP_ANGLE_N]	=	MAKE_DATA_REG(P_ANGLE_N)
	[eP_APP_N]		=	MAKE_DATA_REG(P_APP_N)
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
// ####################################### Global functions ########################################

// Energy returns kWh if PL constant set to 1000imp/kWh, cleared after reading
#ifndef	M90E26_ENERGY_SCALE
//	#define	M90E26_ENERGY_SCALE		10000.0			// default to KWHr
	#define	M90E26_ENERGY_SCALE		10.0			// default to WHr
#endif

void	m90e26Convert_ENERGY(uint8_t eChan, uint8_t eRegAddr) {
	m90e26Data[eRegAddr].value[eChan]	= (float) m90e26Data[eRegAddr].raw_val[eChan] / M90E26_ENERGY_SCALE ;
}

void	m90e26Convert_CURRENT(uint8_t eChan, uint8_t eRegAddr) {
	m90e26Data[eRegAddr].value[eChan]		= (float) m90e26Data[eRegAddr].raw_val[eChan] / 1000 ;
}

void	m90e26Convert_VOLTS(uint8_t eChan, uint8_t eRegAddr) {
	m90e26Data[eRegAddr].value[eChan]		= (float) m90e26Data[eRegAddr].raw_val[eChan] / 100 ;
}

void	m90e26Convert_POWER(uint8_t eChan, uint8_t eRegAddr) {
	m90e26Data[eRegAddr].value[eChan]	= (float) xConvert2sComp(m90e26Data[eRegAddr].raw_val[eChan], 16) / 1000 ;
}

void	m90e26Convert_FREQ(uint8_t eChan, uint8_t eRegAddr) {
	m90e26Data[eRegAddr].value[eChan]		= (float) m90e26Data[eRegAddr].raw_val[eChan] / 100 ;
}

void	m90e26Convert_FACTOR(uint8_t eChan, uint8_t eRegAddr) {
	uint16_t r0 = m90e26Data[eRegAddr].raw_val[eChan] ;
	int32_t c0 = r0 & 0x8000 ? (r0 & 0x7FFF) * -1 : r0 ;
	m90e26Data[eRegAddr].value[eChan]	= (float) c0 / 1000 ;
}

void	m90e26Convert_ANGLE(uint8_t eChan, uint8_t eRegAddr) {
	uint16_t r0 = m90e26Data[eRegAddr].raw_val[eChan] ;
	int32_t c0 = r0 & 0x8000 ? (r0 & 0x7FFF) * -1 : r0 ;
	m90e26Data[eRegAddr].value[eChan]	= (float) c0 / 10 ;
}

void	m90e26Read_Convert(uint8_t eReg, void (*handler)(uint8_t, uint8_t)) {
#if 1			// AVG=466uSec
	IF_CLOCKTIMER_START(m90e26DEBUG_TIMING, clockTIMER_M90EX6) ;
	spi_transaction_t m90e26_buf[halHAS_M90E26] ;
	spi_transaction_t * pResult ;
	memset(m90e26_buf, 0, sizeof(m90e26_buf)) ;
	// send read register request(s)
	for(int32_t eChan = 0; eChan < halHAS_M90E26; eChan++) {
		m90e26_buf[eChan].length			= 8 * 3 ;
		m90e26_buf[eChan].flags 			= SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA ;
		m90e26_buf[eChan].tx_data[0]		= m90e26Data[eReg].addr | 0x80 ;
		ESP_ERROR_CHECK(spi_device_queue_trans(m90e26_handle[eChan], &m90e26_buf[eChan], portMAX_DELAY)) ;
	}
	// Get the read result(s), save & convert
	for(int32_t eChan = 0; eChan < halHAS_M90E26; eChan++) {
		ESP_ERROR_CHECK(spi_device_get_trans_result(m90e26_handle[eChan], &pResult, portMAX_DELAY)) ;
		m90e26Data[eReg].raw_val[eChan]	= pResult->rx_data[1] << 8 | pResult->rx_data[2] ;
		handler(eChan, eReg) ;
	}
	IF_CLOCKTIMER_STOP(m90e26DEBUG_TIMING, clockTIMER_M90EX6) ;
#else			// AVG=496uSec
	IF_myASSERT(m90e26DEBUG_PARAM, (eReg < eNUM_DATA_REG) && INRANGE_MEM(handler)) ;
	IF_CLOCKTIMER_START(m90e26DEBUG_TIMING, clockTIMER_M90EX6) ;
	spi_transaction_t m90e26_buf ;
	memset(&m90e26_buf, 0, sizeof(m90e26_buf)) ;
	// send read register request(s)
	for(int32_t eChan = 0; eChan < halHAS_M90E26; eChan++) {
		m90e26_buf.length				= 8 * 3 ;
		m90e26_buf.flags 				= SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA ;
		m90e26_buf.tx_data[0]			= m90e26Data[eReg].addr | 0x80 ;
		ESP_ERROR_CHECK(spi_device_transmit(m90e26_handle[eChan], &m90e26_buf)) ;
		m90e26Data[eReg].raw_val[eChan]	= m90e26_buf.rx_data[1] << 8 | m90e26_buf.rx_data[2] ;
		handler(eChan, eReg) ;							// convert raw value
	}
	IF_CLOCKTIMER_STOP(m90e26DEBUG_TIMING, clockTIMER_M90EX6) ;
#endif
}

//Energy
int32_t	m90e26Read_E_ACT_FWD(ep_work_t * pEpWork)	{ m90e26Read_Convert(eE_ACT_FWD, m90e26Convert_ENERGY) ; return erSUCCESS ; }
int32_t	m90e26Read_E_ACT_REV(ep_work_t * pEpWork)	{ m90e26Read_Convert(eE_ACT_REV, m90e26Convert_ENERGY) ; return erSUCCESS ; }
int32_t	m90e26Read_E_ACT_ABS(ep_work_t * pEpWork)	{ m90e26Read_Convert(eE_ACT_ABS, m90e26Convert_ENERGY) ; return erSUCCESS ; }
int32_t	m90e26Read_E_REACT_FWD(ep_work_t * pEpWork)	{ m90e26Read_Convert(eE_REACT_FWD, m90e26Convert_ENERGY) ; return erSUCCESS ; }
int32_t	m90e26Read_E_REACT_REV(ep_work_t * pEpWork)	{ m90e26Read_Convert(eE_REACT_REV, m90e26Convert_ENERGY) ; return erSUCCESS ; }
int32_t	m90e26Read_E_REACT_ABS(ep_work_t * pEpWork)	{ m90e26Read_Convert(eE_REACT_ABS, m90e26Convert_ENERGY) ; return erSUCCESS ; }
// Current
int32_t	m90e26Read_I_RMS_L(ep_work_t * pEpWork)		{ m90e26Read_Convert(eI_RMS_L, m90e26Convert_CURRENT) ; return erSUCCESS ; }
int32_t	m90e26Read_I_RMS_N(ep_work_t * pEpWork)		{ m90e26Read_Convert(eI_RMS_N, m90e26Convert_CURRENT) ; return erSUCCESS ; }
// Volts
int32_t	m90e26Read_V_RMS(ep_work_t * pEpWork)		{ m90e26Read_Convert(eVOLTS, m90e26Convert_VOLTS) ; return erSUCCESS ; }
// Frequency
int32_t	m90e26Read_FREQ(ep_work_t * pEpWork)		{ m90e26Read_Convert(eFREQ, m90e26Convert_FREQ) ; return erSUCCESS ; }
// Power Mean Active/Reactive/Apparent
int32_t	m90e26Read_P_ACT_L(ep_work_t * pEpWork)		{ m90e26Read_Convert(eP_ACT_L, m90e26Convert_POWER) ; return erSUCCESS ; }
int32_t	m90e26Read_P_REACT_L(ep_work_t * pEpWork)	{ m90e26Read_Convert(eP_REACT_L, m90e26Convert_POWER) ; return erSUCCESS ; }
int32_t	m90e26Read_P_APP_L(ep_work_t * pEpWork)		{ m90e26Read_Convert(eP_APP_L, m90e26Convert_POWER) ; return erSUCCESS ; }

int32_t	m90e26Read_P_ACT_N(ep_work_t * pEpWork)		{ m90e26Read_Convert(eP_ACT_N, m90e26Convert_POWER) ; return erSUCCESS ; }
int32_t	m90e26Read_P_REACT_N(ep_work_t * pEpWork)	{ m90e26Read_Convert(eP_REACT_N, m90e26Convert_POWER) ; return erSUCCESS ; }
int32_t	m90e26Read_P_APP_N(ep_work_t * pEpWork)		{ m90e26Read_Convert(eP_APP_N, m90e26Convert_POWER) ; return erSUCCESS ; }
// Power Factor
int32_t	m90e26Read_P_FACTOR_L(ep_work_t * pEpWork)	{ m90e26Read_Convert(eP_FACTOR_L, m90e26Convert_FACTOR) ; return erSUCCESS ; }
int32_t	m90e26Read_P_FACTOR_N(ep_work_t * pEpWork)	{ m90e26Read_Convert(eP_FACTOR_N, m90e26Convert_FACTOR) ; return erSUCCESS ; }
// Power Angle
int32_t	m90e26Read_P_ANGLE_L(ep_work_t * pEpWork)	{ m90e26Read_Convert(eP_ANGLE_L, m90e26Convert_ANGLE) ; return erSUCCESS ; }
int32_t	m90e26Read_P_ANGLE_N(ep_work_t * pEpWork)	{ m90e26Read_Convert(eP_ANGLE_N, m90e26Convert_ANGLE) ; return erSUCCESS ; }

// ############################## initialization and (re)configuration #############################

/**
 * m90e26CalcCRC() - calculate the CRC for a range of registers
 */
uint16_t m90e26CalcCRC(uint8_t eChan, uint8_t Addr0, int8_t Count) {
	uint8_t Lcrc = 0, Hcrc = 0 ;
	uint16_t RegData[Count] ;
	// read the range of registers
	for (int32_t i = 0; i < Count; i++) {
		RegData[i] = m90e26Read(eChan, Addr0 + i) ;
	}
	for (int32_t i = 0; i < Count; i++) {
		Lcrc += RegData[i] >> 8 ;		// MOD256 sum the HI bytes
		Hcrc ^= RegData[i] >> 8 ;		// XOR the HI bytes
	}
	for (int32_t i = 0; i < Count; i++) {
		Lcrc += RegData[i] & 0xFF ;		// MOD256 sum the LO bytes
		Hcrc ^= RegData[i] & 0xFF ;		// XOR the LO bytes
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
	// write the configuration registers with calibration data
	for (int32_t i = 0; i < NUM_OF_MEMBERS(RegConfig); i++) {
			m90e26Write(eChan, RegConfig[i].addr, RegConfig[i].raw_val) ;
	}
	m90e26HandleCRC(eChan, CALSTART, CRC_1) ;
	m90e26HandleCRC(eChan, ADJSTART, CRC_2) ;
	return (m90e26GetSysStatus(eChan) & 0xF000) ? erFAILURE : erSUCCESS ;
}

int32_t	m90e26SetLiveGain(uint8_t eChan, uint8_t Gain) {
uint16_t	NewValue ;
	switch (Gain) {
	case 1:
		NewValue	= 0x8000 ;
		break ;
	case 4:
		NewValue	= 0x0000 ;
		break ;
	case 8:
		NewValue	= 0x2000 ;
		break ;
	case 16:
		NewValue	= 0x4000 ;
		break ;
	case 24:
		NewValue	= 0x6000 ;
		break ;
	default:
		IF_SL_ERR(m90e26DEBUG_PARAM, "Invalid Live Gain =%d", Gain) ;
		return erFAILURE ;
	}
	NewValue = m90e26ReadModifyWrite(eChan, MMode, NewValue, 0xE000) ;
	m90e26HandleCRC(eChan, CALSTART, CRC_1) ;
	return erSUCCESS ;
}

int32_t	m90e26SetNeutralGain(uint8_t eChan, uint8_t Gain) {
uint16_t	NewValue ;
	switch (Gain) {
	case 1:
		NewValue	= 0x1000 ;
		break ;
	case 2:
		NewValue	= 0x0000 ;
		break ;
	case 4:
		NewValue	= 0x0800 ;
		break ;
	default:
		IF_SL_ERR(m90e26DEBUG_PARAM, "Invalid Neutral Gain =%d", Gain) ;
		return erFAILURE ;
	}
	NewValue = m90e26ReadModifyWrite(eChan, MMode, NewValue, 0x1800) ;
	m90e26HandleCRC(eChan, CALSTART, CRC_1) ;
	return erSUCCESS ;
}

#define	NUM_OF_VALUES	20

void	m90e26SetPowerOffsetCompensation(void) {
	IF_PRINT(m90e26DEBUG_RESULT, "ensure ALL CT's for ALL channels are disconnected\n") ;
	for (int32_t eChan = 0; eChan < halHAS_M90E26; eChan++) {
		m90e26Write(eChan, POWER_MODE, PWRCOD) ;
		uint16_t CurOffset ;
		uint32_t SumOffset = 0 ;

	// L Line Active Power Offset
		for (int32_t i = 0, SumOffset = 0; i < NUM_OF_VALUES; i++) {
			CurOffset = m90e26Read(eChan, P_OFST_L) ;
			SumOffset += CurOffset ;
		}
		m90e26Write(eChan, P_OFST_L, ~(SumOffset / NUM_OF_VALUES)) ;

	// L Line Reactive Power Offset
		for (int32_t i = 0, SumOffset = 0; i < NUM_OF_VALUES; i++) {
			CurOffset = m90e26Read(eChan, Q_OFST_L) ;
			SumOffset += CurOffset ;
		}
		m90e26Write(eChan, Q_OFST_L, ~(SumOffset / NUM_OF_VALUES)) ;

	// N Line Active Power Offset
		for (int32_t i = 0, SumOffset = 0; i < NUM_OF_VALUES; i++) {
			CurOffset = m90e26Read(eChan, P_OFST_N) ;
			SumOffset += CurOffset ;
		}
		m90e26Write(eChan, P_OFST_N, ~(SumOffset / NUM_OF_VALUES)) ;

	// N Line Reactive Power Offset
		for (int32_t i = 0, SumOffset = 0; i < NUM_OF_VALUES; i++) {
			CurOffset = m90e26Read(eChan, Q_OFST_N) ;
			SumOffset += CurOffset ;
		}
		m90e26Write(eChan, Q_OFST_N, ~(SumOffset / NUM_OF_VALUES)) ;

	// reset to normal power mode & update the CRC...
		m90e26Write(eChan, POWER_MODE, RSTCOD) ;
		m90e26HandleCRC(eChan, ADJSTART, CRC_2) ;
	}
	IF_PRINT(m90e26DEBUG_RESULT, "L & N Line Re/Active Power Offset calibration done\n") ;
}

void	m90e26SetCurrentOffsetCompensation(void) {
	IF_PRINT(m90e26DEBUG_RESULT, "Ensure ALL CT's for ALL channels are disconnected\n") ;
	uint16_t CurOffset ;
	for (int32_t eChan = 0; eChan < halHAS_M90E26; eChan++) {
		m90e26Write(eChan, ADJSTART, STDCOD) ;				// enable adjustment mode
	// L Line Current Offset
		CurOffset = m90e26Read(eChan, I_RMS_L) ;
		// ?????
		m90e26Write(eChan, I_OFST_L, ~CurOffset) ;

	// L Line Current Offset
		CurOffset = m90e26Read(eChan, I_RMS_N) ;
		// ?????
		m90e26Write(eChan, I_OFST_N, ~CurOffset) ;

		m90e26HandleCRC(eChan, ADJSTART, CRC_2) ;
	}
	IF_PRINT(m90e26DEBUG_RESULT, "L & N Line Current Offset calibration done\n") ;
}

inline uint16_t m90e26GetSysStatus(uint8_t eChan)	{ return m90e26Read(eChan, SYS_STATUS) ; }
inline uint16_t m90e26GetMeterStatus(uint8_t eChan)	{ return m90e26Read(eChan, MET_STATUS) ; }
inline uint16_t m90e26GetLastData(uint8_t eChan)	{ return m90e26Read(eChan, LASTDATA) ; }

// ############################### device reporting functions ######################################

void	m90e26ReportStatus(uint8_t eChan) {
	uint16_t SysStatus = m90e26GetSysStatus(eChan) ;
	PRINT("Ch %d :  SystemStatus %04x", eChan, SysStatus) ;
	if (SysStatus & 0xC000)		PRINT("\tCRC_1 Error!!") ;
	if (SysStatus & 0x3000)		PRINT("\tCRC_2 Error!!") ;
	if (SysStatus & 0x0080)		PRINT("\tMetering line L<>N change!!") ;
	if (SysStatus & 0x0040)		PRINT("\tReactive Energy DIR change!!") ;
	if (SysStatus & 0x0020)		PRINT("\tActive Energy DIR change!!") ;
	if (SysStatus & 0x0002)		PRINT("\tVoltage SAG") ;
	PRINT("\n") ;

	uint16_t MeterStatus = m90e26GetMeterStatus(eChan) ;
	PRINT("Ch %d :  MeterStatus %04x\n", eChan, MeterStatus) ;
	if (MeterStatus & 0x8000)	PRINT("\tReActive NO Load  ") ;
	if (MeterStatus & 0x4000)	PRINT("\tActive NO Load  ") ;
	if (MeterStatus & 0x2000)	PRINT("\tReActive Reverse  ") ;
	if (MeterStatus & 0x1000)	PRINT("\tActive Reverse  ") ;
	PRINT("Tamper %s  ", MeterStatus & 0x0800 ? "Live" : "Neutral") ;
	PRINT("LineMode (%d) %s\n", MeterStatus & 0x3,
								(MeterStatus & 0x3) == 0x3 ? "Flexible" :
								(MeterStatus & 0x3) == 0x2 ? "L+N" :
								(MeterStatus & 0x3) == 0x1 ? "L only" : "AntiTamper") ;
}

void	m90e26ReportCalib(uint8_t eChan) {
const uint8_t m90e26RegCalib[]	= {	CALSTART, PLconstH, PLconstL, Lgain, Lphi, Ngain, Nphi, PStartTh, PNolTh, QStartTh, QNolTh, MMode, CRC_1 } ;
	PRINT("Ch %d : CALSTRT PLconsH PLconsL   Lgain    Lphi   Ngain    Nphi PStrtTh  PNolTh QStrtTh  QNolTh   MMode   CRC_1\n       ", eChan) ;
	for(int32_t i = 0; i < sizeof(m90e26RegCalib); i++)		PRINT(" 0x%04x ", m90e26Read(eChan, m90e26RegCalib[i])) ;
	PRINT("\n") ;
}

void	m90e26ReportAdjust(uint8_t eChan) {
const uint8_t m90e26RegAdjust[]	= {	ADJSTART, V_GAIN, I_GAIN_L, I_GAIN_N, V_OFFSET, I_OFST_L, I_OFST_N, P_OFST_L, Q_OFST_L, P_OFST_N, Q_OFST_N, CRC_2 } ;
	PRINT("Ch %d : ADJSTRT   Vgain  IgainL  IgainN Voffset  IofstL  IofstN  PofstL  QofstL  PofstN  QofstN   CRC_2\n       ", eChan) ;
	for(int32_t i = 0; i < sizeof(m90e26RegAdjust); i++)	PRINT(" 0x%04x ", m90e26Read(eChan, m90e26RegAdjust[i])) ;
	PRINT("\n") ;
}

void	m90e26ReportData(uint8_t eChan) {
	data_reg_t * pData = &m90e26Data[0] ;
	PRINT("Ch %d :  ActFwd  ActRev  ActAbs  ReaFwd  ReaRev  ReaAbs   IrmsL    Vrms   PactL PreactL    Freq  PfactL PangleL   PappL   IrmsN   PactN PreactN  PfactN PangleN   PappN\n      ", eChan) ;
	for (int32_t i = 0; i < eNUM_DATA_REG; i++)		PRINT("%8x", pData[i].raw_val[eChan]) ;	PRINT("\n      ") ;
	for (int32_t i = 0; i < eNUM_DATA_REG; i++)		PRINT("%8.1f", pData[i].value[eChan]) ;	PRINT("\n") ;
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
static	uint8_t Index = 0 ;
static	uint16_t Count = 0 ;
void	m90e26Display(void) {
uint8_t	Chan ;
	if (m90e26_handle[0] == 0) {
		return ;
	}
	if (++Count % 15) {
		return ;
	} else {
		Count = 0 ;
	}
	Chan = Index / halHAS_M90E26 ;
	ssd1306SetTextCursor(&sSSD1306, 0, 0) ;
	if ((Index % 2) == 0) {
		devprintf(ssd1306PutC, "V:%8.2fF:%8.2fI:%8.2fAP%8.2gPa%8.2fPf%8.2f",
			m90e26Data[eVOLTS].value[Chan],m90e26Data[eFREQ].value[Chan],m90e26Data[eI_RMS_L].value[Chan],
			m90e26Data[eP_ACT_L].value[Chan],m90e26Data[eP_ANGLE_L].value[Chan],m90e26Data[eP_FACTOR_L].value[Chan]) ;
	} else {
		devprintf(ssd1306PutC, "Af%8.2fAr%8.2fAa%8.2fRf%8.2fRr%8.2fRa%8.2f",
			m90e26Data[eE_ACT_FWD].value[Chan],m90e26Data[eE_ACT_REV].value[Chan],m90e26Data[eE_ACT_ABS].value[Chan],
			m90e26Data[eE_REACT_FWD].value[Chan],m90e26Data[eE_REACT_REV].value[Chan],m90e26Data[eE_REACT_ABS].value[Chan]) ;
	}
	Index++ ;
	Index %= (halHAS_M90E26 * 2) ;
}
#endif

void	m90e26DisplayInfo(void) {
#if 0
	// Angle & factor
	uint16_t r0 = m90e26Data[eP_ANGLE_L].raw_val[0] ;
	int32_t c0 = r0 & 0x8000 ? (r0 & 0x7FFF) * -1 : r0 ;
	float f0 = (float) c0 / 10 ;
	PRINT("Ch:0  Pangle=%04x => %d => %f\n", r0, c0, f0) ;
#elif 0
	// power mean
	uint16_t r0 = m90e26Data[eP_ACT_L].raw_val[0] ;
	int32_t c0 = xConvert2sComp(r0, 16) ;
	float f0 = (float) c0 / 1000 ;
	PRINT("Ch:0  Pwr=%04x => %d => %f\n", r0, c0, f0) ;
#elif 1
	// Energy
	uint16_t r0 = m90e26Data[eE_ACT_FWD].raw_val[1] ;
	int32_t c0 = xConvert2sComp(r0, 16) ;
	float f0 = (float) c0 / 1000 ;
	PRINT("Ch:1  Pwr=%04x => %d => %f\n", r0, c0, f0) ;
#endif
}

// ################################## Diagnostics functions ########################################
#endif
