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
 * TODO:
 * 1.	Store all energy readings into NVS to improve accuracy
 * 2.	On boot check if sensor config value are same as default
 *		if so, assume hard reboot and initialise chip with our values
 *		if not, assume chip pre-initialised and handle existing energy values in registers to improve accuracy
 *
 */

#include	"hal_config.h"

#if		(halHAS_M90E26 > 0)

#include	"endpoints.h"
#include	"rules_engine.h"
#include	"x_errors_events.h"
#include	"x_systiming.h"					// timing debugging
#include	"x_syslog.h"
#include	"x_values_convert.h"

#include	"hal_debug.h"
#include	"hal_spi.h"
#include	"hal_storage.h"
#include	"m90e26.h"

#if		(halHAS_SSD1306 > 0)
	#include	"ssd1306/ssd1306.h"
#endif

#include	<stdint.h>
#include	<string.h>

#define	debugFLAG					0xC800

#define	debugREAD					(debugFLAG & 0x0001)
#define	debugWRITE					(debugFLAG & 0x0002)
#define	debugCURRENT				(debugFLAG & 0x0004)
#define	debugENERGY					(debugFLAG & 0x0008)

#define	debugFREQ					(debugFLAG & 0x0010)
#define	debugFACTOR					(debugFLAG & 0x0020)
#define	debugANGLE					(debugFLAG & 0x0040)
#define	debugPOWER					(debugFLAG & 0x0080)

#define	debugINIT					(debugFLAG & 0x0100)
#define	debugMODE					(debugFLAG & 0x0200)
#define	debugCRC					(debugFLAG & 0x0400)
#define	debugOFFSET					(debugFLAG & 0x0800)

#define	debugCONTRAST				(debugFLAG & 0x1000)
#define	debugVOLTS					(debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG & 0x8000)

// ###################################### Private variables #######################################

spi_device_interface_config_t	m90e26_config[M90E26_NUM] = {
#if		(halHAS_M90E26 > 0)
	[M90E26_0] = {
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
	[M90E26_1] = {
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

spi_device_handle_t				m90e26_handle[M90E26_NUM] = { 0 } ;
SemaphoreHandle_t				m90e26mutex[M90E26_NUM] = { 0 } ;

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
	} Chan[M90E26_NUM] ;
} m90e26Config = { 0 } ;

const uint8_t	m90e26RegAddr[] = {
	E_ACT_FWD,	E_ACT_REV,	E_ACT_ABS,					// Active energy
	E_REACT_FWD,E_REACT_REV,E_REACT_ABS,				// ReActive energy
	I_RMS_L,	V_RMS,		P_ACT_L,	P_REACT_L,		// Voltage, Freq & LIVE Power info
	FREQ,		P_FACTOR_L,	P_ANGLE_L,	P_APP_L,
#if		(M90E26_NEUTRAL == 1)
	I_RMS_N,	P_ACT_N,	P_REACT_N,					// NEUTRAL Power info
	P_FACTOR_N,	P_ANGLE_N,	P_APP_N,
#endif
} ;

const nvs_m90e26_t	nvsM90E26default = {
	{	0x8765,	0x0015, 0xD174, 0x0000, 0x0000, 0x0000, 0x0000, 0x08BD, 0x0000, 0x0AEC, 0x0000, 0x9422 },
	{	0x8765, 0x6720, 0x7A13, 0x7530, 0x0000, 0x9BD1, 0x1F6F, 0xFFF5, 0x001B, 0x0009, 0x000A },
	{	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
} ;

nvs_m90e26_t	nvsM90E26[CALIB_NUM] ;

// ###################################### Private functions ########################################

void	m90e26Write(uint8_t eChan, uint8_t address, uint16_t val) {
	IF_myASSERT(debugPARAM, eChan < M90E26_NUM && address < 0x70 && m90e26_handle[eChan]) ;
	xSemaphoreTake(m90e26mutex[eChan], portMAX_DELAY) ;
	spi_transaction_t m90e26_buf ;
	memset(&m90e26_buf, 0, sizeof(m90e26_buf));
	m90e26_buf.length		= 8 * 3;
	m90e26_buf.flags 		= SPI_TRANS_USE_TXDATA ;
	m90e26_buf.tx_data[0]	= address ;
	m90e26_buf.tx_data[1]	= val >> 8 ;
	m90e26_buf.tx_data[2]	= val & 0xFF ;
	ESP_ERROR_CHECK(spi_device_transmit(m90e26_handle[eChan], &m90e26_buf)) ;
	xSemaphoreGive(m90e26mutex[eChan]) ;
	IF_PRINT(debugWRITE, "TX: addr=%02x d0=%02x d1=%02x\n", m90e26_buf.tx_data[0], m90e26_buf.tx_data[1], m90e26_buf.tx_data[2]) ;
}

uint16_t m90e26Read(uint8_t eChan, uint8_t address) {
	IF_myASSERT(debugPARAM, eChan < M90E26_NUM && address < 0x70 && m90e26_handle[eChan]) ;
	xSemaphoreTake(m90e26mutex[eChan], portMAX_DELAY) ;
	spi_transaction_t m90e26_buf ;
	memset(&m90e26_buf, 0, sizeof(m90e26_buf)) ;
	m90e26_buf.length		= 8 * 3 ;
	m90e26_buf.flags 		= SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA ;
	m90e26_buf.tx_data[0]	= address | 0x80 ;
	ESP_ERROR_CHECK(spi_device_transmit(m90e26_handle[eChan], &m90e26_buf)) ;
	xSemaphoreGive(m90e26mutex[eChan]) ;
	IF_PRINT(debugREAD, "RX: addr=%02x  d0=%02x  d1=%02x  dx=%04x\n", m90e26_buf.tx_data[0], m90e26_buf.rx_data[1], m90e26_buf.rx_data[2], (m90e26_buf.rx_data[1] << 8) | m90e26_buf.rx_data[2]) ;
	return (m90e26_buf.rx_data[1] << 8) | m90e26_buf.rx_data[2] ;
}

uint16_t m90e26ReadModifyWrite(uint8_t eChan, uint8_t Addr, uint16_t Value, uint16_t Mask) {
	uint16_t CurValue = m90e26Read(eChan, Addr) ;
	CurValue &= ~Mask ;
	CurValue |= Value ;
	m90e26Write(eChan, Addr, CurValue) ;
	return CurValue ;
}

// ################################## (re)configuration & CRCs #####################################

void	m90e26SetCurrentOffset(uint8_t eChan, uint8_t RegRMS, uint8_t RegGAIN, uint8_t RegOFST) {
	uint16_t CurAmps = m90e26Read(eChan, RegRMS) ;
	uint16_t CurGain = m90e26Read(eChan, RegGAIN) ;
	uint32_t Factor1 = (CurAmps * CurGain) / 2^8 ;
	uint32_t Factor2 = ~Factor1 & 0x0000FFFF ;
	m90e26Write(eChan, RegOFST, Factor2) ;
	IF_PRINT(debugOFFSET, "Ch %d: Rrms=%d Rgain=%d Rofst=%d  Icur=0x%04x  Gcur=0x%04x  F1=0x%08x  F2=0x%04x\n",
			eChan, RegRMS, RegGAIN, RegOFST, CurAmps, CurGain, Factor1, Factor2) ;
}

void	m90e26SetPowerOffset(uint8_t eChan, uint8_t RegPOWER, uint8_t RegOFST) {
	uint32_t SumOffset = 0 ;
	for (int32_t i = 0; i < M90E26_CALIB_ITER; i++) {
		SumOffset += m90e26Read(eChan, RegPOWER) ;
	}
	uint16_t NewOffset = ~(SumOffset / M90E26_CALIB_ITER) ;
	m90e26Write(eChan, RegOFST, NewOffset) ;
	IF_PRINT(debugOFFSET, "Ch %d: Regs=%d->%d  %dx  Sum=0x%08x  Ofst=0x%04x\n",
			eChan, RegPOWER, RegOFST, M90E26_CALIB_ITER, SumOffset, NewOffset) ;
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
	IF_PRINT(debugCRC, "CRC=%04x from %-'h\n", (Hcrc << 8) | Lcrc, Count * 2, RegData) ;
	return (Hcrc << 8) | Lcrc ;
}

/**
 * m90e26HandleCRC() - Calculate the CRC from the range of registers & write the lock register/pattern
 */
void	m90e26HandleCRC(uint8_t eChan, uint8_t RegAddr1, uint8_t RegAddr2) {
	IF_myASSERT(debugPARAM, (eChan < halHAS_M90E26) &&
							((RegAddr1==CALSTART && RegAddr2==CRC_1) ||
							 (RegAddr1==ADJSTART && RegAddr2==CRC_2) ) ) ;
	m90e26Write(eChan, RegAddr2, m90e26CalcCRC(eChan, RegAddr1 + 1, RegAddr2 - RegAddr1 - 1)) ;
	m90e26Write(eChan, RegAddr1, CFGCOD) ;
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
	// check if maybe already configured and running
	if ((m90e26Read(eChan, V_GAIN)		== nvsM90E26default.adjreg[1]) &&
		(m90e26Read(eChan, I_GAIN_L)	== nvsM90E26default.adjreg[2]) &&
		(m90e26Read(eChan, V_OFFSET)	== nvsM90E26default.adjreg[4]) &&
		(m90e26Read(eChan, I_OFST_L)	== nvsM90E26default.adjreg[5])) {
		SL_WARN("m90e26 #%d ALREADY configured & running", eChan) ;
	} else {
		/* Check that blob with CALibration and ADJustment values exists
		 * If not existing, create with factory defaults as first record
		 */
		size_t	SizeBlob = CALIB_NUM * sizeof(nvs_m90e26_t) ;
		nvs_m90e26_t * psCalib = malloc(SizeBlob);
		int32_t iRV = halSTORAGE_ReadBlob(halSTORAGE_STORE, halSTORAGE_KEY_M90E26, psCalib, &SizeBlob) ;
		if (iRV != erSUCCESS) {
			bzero(psCalib, SizeBlob = CALIB_NUM * sizeof(nvs_m90e26_t)) ;
			memcpy(psCalib, &nvsM90E26default, sizeof(nvs_m90e26_t)) ;
			iRV = halSTORAGE_WriteBlob(halSTORAGE_STORE, halSTORAGE_KEY_M90E26, psCalib, SizeBlob) ;
		}
		// write the configuration registers with METER calibration data
		for (int32_t i = 0; i < SIZEOF_MEMBER(nvs_m90e26_t, calreg); i++) {
			m90e26Write(eChan, CALSTART+i, nvsM90E26default.calreg[i]) ;
		}
		m90e26HandleCRC(eChan, CALSTART, CRC_1) ;

		// write the configuration registers with MEASURE calibration data
		for (int32_t i = 0; i < SIZEOF_MEMBER(nvs_m90e26_t, adjreg); i++) {
			m90e26Write(eChan, ADJSTART+i, nvsM90E26default.adjreg[i]) ;
		}
		m90e26HandleCRC(eChan, ADJSTART, CRC_2) ;		// calculate & write CRC
	}

	// set default state
	m90e26Config.Chan[eChan].L_Gain		= 1 ;
	#if	(M90E26_NEUTRAL == 1)
	m90e26Config.Chan[eChan].N_Gain		= 1 ;
	#endif
	m90e26Config.Chan[eChan].E_Scale	= 0 ;			// Wh not kWh
	m90e26Config.Chan[eChan].P_Scale	= 0 ;			// W not kW
	m90e26Config.Chan[eChan].I_Scale	= 0 ;			// A not mA
	m90e26Config.MaxContrast			= 255 ;
	return (m90e26GetSysStatus(eChan) & 0xF000) ? erFAILURE : erSUCCESS ;
}

/* Overall process documented in the following Application Note:
 * http://ww1.microchip.com/downloads/en/AppNotes/Atmel-46102-SE-M90E26-ApplicationNote.pdf
 */
void	m90e26Calibrate(uint8_t eChan) {
	/* Preference is to fix this functionality, then hard code the values into the
	 * initialization table and do it that way. Alternative would be to run this
	 * ONLY if calibration values cannot be found in NVS, same as WIFI credentials */
	IF_PRINT(debugINIT, "Ch %d: Offset Compensation start, DISCONNECT CT's\n", eChan) ;
	m90e26Write(eChan, ADJSTART, STDCOD) ;
	/* LIVE & NEUTRAL Current Offset calibration not working properly The formula as described
	 * in the appnote is NOT clear on the calculation and does not yield proper values  */
	m90e26SetCurrentOffset(eChan, I_RMS_L, I_GAIN_L, I_OFST_L) ;
#if		(M90E26_NEUTRAL == 1)
	m90e26SetCurrentOffset(eChan, I_RMS_N, I_GAIN_N, I_OFST_N) ;
#endif
	/* [Re]Active, LINE & NEUTRAL Power Offset calibration
	 * Not sure if the power register should be read whilst in normal or
	 * adjustment mode. Currently no real value is being read */
	m90e26Write(eChan, POWER_MODE, PWRCOD) ;			// set into low power mode for calibration
	m90e26SetPowerOffset(eChan, P_ACT_L, P_OFST_L) ;	// L Line Active Power Offset
	m90e26SetPowerOffset(eChan, P_REACT_L, Q_OFST_L) ;	// L Line ReActive Power Offset
#if		(M90E26_NEUTRAL == 1)
	m90e26SetPowerOffset(eChan, P_ACT_N, P_OFST_N) ;	// N Line Active Power Offset
	m90e26SetPowerOffset(eChan, P_REACT_N, Q_OFST_N) ;	// N Line ReActive Power Offset
#endif
	m90e26Write(eChan, POWER_MODE, RSTCOD) ;			// reset to normal power mode
	m90e26HandleCRC(eChan, ADJSTART, CRC_2) ;			// calculate & write CRC
	IF_PRINT(debugINIT, "Ch %d: Offset Compensation done, RECONNECT CT's\n", eChan) ;
}

// ############################### common support routines #########################################

uint8_t	m90e26CalcInfo(ep_work_t * psEpWork) {
	xEpWorkToUri(psEpWork) ;
	psEpWork->idx = psEpWork->uri - URI_M90E26_E_ACT_FWD_0 ;
	psEpWork->eChan = 0 ;
#if		(halHAS_M90E26 > 1)
	if (psEpWork->idx >= M90E26_NUMURI_0) {
		psEpWork->idx -= M90E26_NUMURI_0 ;
		++(psEpWork->eChan) ;
	}
#endif
	IF_myASSERT(debugRESULT, (psEpWork->idx < M90E26_NUMURI_0) && (psEpWork->eChan < M90E26_NUM)) ;
	return psEpWork->idx ;
}

// ############################# endpoint support functions ########################################

int32_t	m90e26ReadEnergy(ep_work_t * psEpWork) {
	if (psEpWork->Var.varDef.cv.sumX) {					// if just a normal update cycle
		int8_t eUri = m90e26CalcInfo(psEpWork) ;
		uint16_t RawVal	= m90e26Read(psEpWork->eChan, m90e26RegAddr[psEpWork->idx]) ;
		float f32Val	= (float) RawVal / (m90e26Config.Chan[psEpWork->eChan].E_Scale ? 10000.0 : 10.0) ;
		xEpSetValue(psEpWork, (x32_t) f32Val) ;
		IF_PRINT(debugENERGY, "Energy: URI=%d  Ch=%u  Reg=0x%02X  Raw=0x%04X  Val=%9.3f\n",
				eUri, psEpWork->eChan, m90e26RegAddr[psEpWork->idx], RawVal, f32Val) ;
	} else {											// else it is a value reset call
		vCompVarResetValue(&psEpWork->Var) ;
		IF_PRINT(debugENERGY, "Energy: Sum RESET\n") ;
	}
	return erSUCCESS ;
}

int32_t	m90e26ReadCurrent(ep_work_t * psEpWork) {
	uint8_t eUri = m90e26CalcInfo(psEpWork) ;
	uint32_t HiVal	= m90e26Read(psEpWork->eChan, m90e26RegAddr[psEpWork->idx]) ;
	uint32_t LoVal	= m90e26Read(psEpWork->eChan, LSB) ;
	uint32_t x32Val = (HiVal << 16) + LoVal ;
	float	f32Val	= (float) x32Val ;
	IF_PRINT(debugCURRENT, "Irms: eUri=%d  Reg=%02X  Ch=%d  Hi=0x%04X  Lo=0x%04X  x32=0x%08X",
			eUri, m90e26RegAddr[psEpWork->idx], psEpWork->eChan, HiVal, LoVal, x32Val) ;
	if (m90e26Config.Chan[psEpWork->eChan].I_Scale == 0) {
		f32Val	/= 65536000.0 ;								// convert to Amp (not mA)
		IF_PRINT(debugCURRENT, "  Val=%4.5fA", f32Val) ;
	} else {
		f32Val	/= 65536.0 ;								// convert to mA)
		IF_PRINT(debugCURRENT, "  Val=%4.5fmA", f32Val) ;
	}

#if		(M90E26_NEUTRAL == 1)
	IF_myASSERT(debugRESULT, psEpWork->idx == eI_RMS_L || psEpWork->idx == eI_RMS_N) ;
	if (psEpWork->idx == eI_RMS_L) {
		f32Val /= m90e26Config.Chan[psEpWork->eChan].L_Gain ;
		IF_PRINT(debugCURRENT, "  Lgain=%d", m90e26Config.Chan[psEpWork->eChan].L_Gain) ;
	} else {
		f32Val /= m90e26Config.Chan[psEpWork->eChan].N_Gain ;
		IF_PRINT(debugCURRENT, "  Ngain=%d", m90e26Config.Chan[psEpWork->eChan].N_Gain) ;
	}
#else
	IF_myASSERT(debugRESULT, psEpWork->idx == eI_RMS_L) ;
	f32Val /= m90e26Config.Chan[psEpWork->eChan].L_Gain ;
	IF_PRINT(debugCURRENT, "  Lgain=%d", m90e26Config.Chan[psEpWork->eChan].L_Gain) ;
#endif
	IF_PRINT(debugCURRENT, "  Act=%4.5f\n", f32Val) ;
	xEpSetValue(psEpWork, (x32_t) f32Val) ;
	return erSUCCESS ;
}

int32_t	m90e26ReadVoltage(ep_work_t * psEpWork) {		// OK
	m90e26CalcInfo(psEpWork) ;
	uint32_t HiVal	= m90e26Read(psEpWork->eChan, m90e26RegAddr[psEpWork->idx]) ;
	uint32_t LoVal	= m90e26Read(psEpWork->eChan, LSB) ;
	uint32_t x32Val = (HiVal << 16) + LoVal ;
	float f32Val	= (float) x32Val ;
	f32Val			/= 6553600.0 ;
	xEpSetValue(psEpWork, (x32_t) f32Val) ;
	IF_PRINT(debugVOLTS, "Vrms: Ch=%d  Hi=0x%04X  Lo=0x%04X  x32=0x%08X  Val=%9.3f\n",
			psEpWork->eChan, HiVal, LoVal, x32Val, f32Val) ;
	return erSUCCESS ;
}

int32_t	m90e26ReadPower(ep_work_t * psEpWork) {
	m90e26CalcInfo(psEpWork) ;
	uint32_t HiVal	= m90e26Read(psEpWork->eChan, m90e26RegAddr[psEpWork->idx]) ;
	uint32_t LoVal	= m90e26Read(psEpWork->eChan, LSB) ;
	uint32_t x32Val = (HiVal << 16) + LoVal ;
	float f32Val	= (float) xConvert2sComp(x32Val, 32) ;
	if (m90e26Config.Chan[psEpWork->eChan].P_Scale == 1) {
		f32Val	/= 65536000.0 ;							// make KWh (alt range)
	} else {
		f32Val	/= 65536.0 ;							// make Wh (default)
	}
	xEpSetValue(psEpWork, (x32_t) f32Val) ;
	IF_PRINT(debugPOWER, "Power: Ch=%d  Reg=%02X  x16=0x%04X  x8=0x%04x  x32=0x%08X  Val=%9.3f\n",
			psEpWork->eChan, m90e26RegAddr[psEpWork->idx], HiVal, LoVal, x32Val, f32Val) ;
	return erSUCCESS ;
}

int32_t	m90e26ReadFrequency(ep_work_t * psEpWork) {		// OK
	m90e26CalcInfo(psEpWork) ;
	uint16_t RawVal	= m90e26Read(psEpWork->eChan, m90e26RegAddr[psEpWork->idx]) ;
	float f32Val	= (float) RawVal / 100.0 ;
	xEpSetValue(psEpWork, (x32_t) f32Val) ;
	IF_PRINT(debugFREQ, "Freq: Ch=%d  Raw=0x%04X  Val=%9.3f\n", psEpWork->eChan, RawVal, f32Val) ;
	return erSUCCESS ;
}

int32_t	m90e26ReadPowerFactor(ep_work_t * psEpWork) {
	m90e26CalcInfo(psEpWork) ;
	uint16_t RawVal	= m90e26Read(psEpWork->eChan, m90e26RegAddr[psEpWork->idx]) ;
	int32_t ConVal	= (RawVal & 0x8000) ? (-1 * (RawVal & 0x7FFF)) : RawVal ;
	float f32Val	= (float) ConVal / 1000.0 ;
	xEpSetValue(psEpWork, (x32_t) f32Val) ;
	IF_PRINT(debugFACTOR, "PF: Ch=%d  Raw=0x%04X  Con=%d  Val=%9.3f\n", psEpWork->eChan, RawVal, ConVal, f32Val) ;
	return erSUCCESS ;
}

int32_t	m90e26ReadPowerAngle(ep_work_t * psEpWork) {
	m90e26CalcInfo(psEpWork) ;
	uint16_t RawVal	= m90e26Read(psEpWork->eChan, m90e26RegAddr[psEpWork->idx]) ;
	int32_t ConVal = (RawVal & 0x8000) ? (-1 * (RawVal & 0x7FFF)) : RawVal ;
	float f32Val	= (float) ConVal / 10.0 ;
	xEpSetValue(psEpWork, (x32_t) f32Val) ;
	IF_PRINT(debugANGLE, "Angle: Ch=%d  Raw=0x%04X  Con=%d  Val=%9.3f\n", psEpWork->eChan, RawVal, ConVal, f32Val) ;
	return erSUCCESS ;
}

inline uint16_t m90e26GetSysStatus(uint8_t eChan)	{ return m90e26Read(eChan, SYS_STATUS) ; }
inline uint16_t m90e26GetMeterStatus(uint8_t eChan)	{ return m90e26Read(eChan, MET_STATUS) ; }
inline uint16_t m90e26GetLastData(uint8_t eChan)	{ return m90e26Read(eChan, LASTDATA) ; }

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
	m90e26HandleCRC(eChan, CALSTART, CRC_1) ;
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
	m90e26HandleCRC(eChan, CALSTART, CRC_1) ;
	return erSUCCESS ;
}

int32_t m90e26SoftReset(uint8_t eChan) {
	IF_myASSERT(debugPARAM, eChan < halHAS_M90E26) ;
	m90e26Write(eChan, SOFTRESET, RSTCOD) ;
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
	case eL_GAIN:	iRV = m90e26SetLiveGain(psRule->para.u32[0][1], psRule->para.u32[0][2]) ;		break ;
#if		(M90E26_NEUTRAL == 1)		// NEUTRAL Line wrapper functions
	case eN_GAIN:	iRV = m90e26SetNeutralGain(psRule->para.u32[0][1], psRule->para.u32[0][2]) ;	break ;
#endif
	case eSOFTRESET:iRV = m90e26SoftReset(psRule->para.u32[0][1]) ;									break ;
	case eRECALIB:	iRV = m90e26Recalibrate(psRule->para.u32[0][1]) ;								break ;

#if		(halHAS_SSD1306 > 0)
	case eBRIGHT:
		if ((psRule->para.u32[0][2] <= 255) &&
			(psRule->para.u32[0][1] <= psRule->para.u32[0][2])) {
			m90e26Config.MinContrast = psRule->para.u32[0][1] ;
			m90e26Config.MaxContrast = psRule->para.u32[0][2] ;
		} else {
			iRV = erSCRIPT_INV_PARA ;
		}
		break ;

	case eDISPLAY:
	#if	(halHAS_M90E26 > 0)
		if (psRule->para.u32[0][1] < eDM_MAXIMUM) {
			m90e26Config.Chan[M90E26_0].Display = psRule->para.u32[0][1] ;
		} else {
			iRV = erSCRIPT_INV_PARA ;
		}
	#endif
	#if	(halHAS_M90E26 > 1)
		if (psRule->para.u32[0][2] < eDM_MAXIMUM) {
			m90e26Config.Chan[M90E26_1].Display = psRule->para.u32[0][2] ;
		} else {
			iRV = erSCRIPT_INV_PARA ;
		}
	#endif
		break ;

	case eBLANKING:	m90e26Config.tBlank = psRule->para.u32[0][1] ;									break ;
#endif
	default:		iRV = erSCRIPT_INV_MODE ;
	}
	return iRV ;
}

// ############################### device reporting functions ######################################

#define	HDR_CALIB		"Ch CALSTRT PLconsH PLconsL   Lgain    Lphi   Ngain    Nphi PStrtTh  PNolTh QStrtTh  QNolTh   MMode   CRC_1"
#define	HDR_ADJUST		"Ch ADJSTRT   Vgain  IgainL  IgainN   Vofst  IofstL  IofstN  PofstL  QofstL  PofstN  QofstN   CRC_2"
#define	HDR_DATA_LIVE	"Ch  ActFwd  ActRev  ActAbs  ReaFwd  ReaRev  ReaAbs   IrmsL    Vrms   PactL PreactL Freq Hz  PfactL PangleL   PappL"
#define	HDR_DATA_NEUT	  "   IrmsN   PactN PreactN  PfactN PangleN   PappN"
#define	HDR_STATUS		"Ch  System    CRC1    CRC2  L/N Ch RevQchg RevPchg SagWarn   Meter Qnoload Pnoload    RevQ    RevP  Tamper  L-Mode"
#define	BLANK8			"        "

static const uint8_t m90e26DataReg[] = {
	E_ACT_FWD, E_ACT_REV, E_ACT_ABS, E_REACT_FWD, E_REACT_REV, E_REACT_ABS, I_RMS_L, V_RMS, P_ACT_L, P_REACT_L, FREQ, P_FACTOR_L, P_ANGLE_L, P_APP_L,
	I_RMS_N, P_ACT_N, P_REACT_N, P_FACTOR_N, P_ANGLE_N, P_APP_N, LSB, LASTDATA,
} ;

void	m90e26ReportCalib(void) {
	PRINT("%C%s%C\n", xpfSGR(attrRESET, colourFG_CYAN,0,0), HDR_CALIB, attrRESET) ;
	for (int32_t eChan = 0; eChan < halHAS_M90E26; ++eChan) {
		PRINT("%2d", eChan) ;
		for (int32_t i = CALSTART; i <= CRC_1; i++) {
			PRINT("  0x%04X", m90e26Read(eChan, i)) ;
		}
		PRINT("\n") ;
	}
}

void	m90e26ReportAdjust(void) {
	PRINT("%C%s%C\n", xpfSGR(attrRESET, colourFG_CYAN,0,0), HDR_ADJUST, attrRESET) ;
	for (int32_t eChan = 0; eChan < halHAS_M90E26; ++eChan) {
		PRINT("%2d", eChan) ;
		for (int32_t i = ADJSTART; i <= CRC_2; i++) {
			PRINT("  0x%04X", m90e26Read(eChan, i)) ;
		}
		PRINT("\n") ;
	}
}

void	m90e26ReportData(void) {
	PRINT("%C%s%C\n", xpfSGR(attrRESET, colourFG_CYAN,0,0), HDR_DATA_LIVE HDR_DATA_NEUT, attrRESET) ;
	for (int32_t eChan = 0; eChan < halHAS_M90E26; ++eChan) {
		PRINT("%2d", eChan) ;
		for (int32_t i = 0; i < eNUM_DATA_REG; i++) {
			PRINT("  0x%04X", m90e26Read(eChan, m90e26DataReg[i])) ;
		}
		PRINT("\n") ;
	}
}

void	m90e26ReportStatus(void) {
	PRINT("%C%s%C\n", xpfSGR(attrRESET, colourFG_CYAN,0,0), HDR_STATUS, attrRESET) ;
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
	psEpWork = &table_work[eChan == M90E26_0 ? URI_M90E26_E_ACT_FWD_0 : URI_M90E26_E_ACT_FWD_1] ;
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

#endif
