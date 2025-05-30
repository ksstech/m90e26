// m90e26.c - Copyright (c) 2018-25 Andre M. Maree / KSS Technologies (Pty) Ltd.

#include "hal_platform.h"

#if (HAL_M90E26 > 0)
#include "endpoints.h"
#include "m90e26.h"
#include "hal_rtc.h"
#include "hal_flash.h"
#include "FreeRTOS_Support.h"
#include "report.h"
#include "syslog.h"
#include "systiming.h"					// timing debugging
#include "errors_events.h"

#include "driver/spi_master.h"
#include "nvs.h"
#include <string.h>

// ######################################## Build macros ###########################################

#define	debugFLAG					0xF000

#define	debugREAD					(debugFLAG & 0x0001)
#define	debugWRITE					(debugFLAG & 0x0002)
#define	debugRMW					(debugFLAG & 0x0004)
#define	debugCURRENT				(debugFLAG & 0x0008)
#define	debugPOWER					(debugFLAG & 0x0010)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ########################################### Macros ##############################################

#if (m90e26NEUTRAL > 0)
	#define	M90E26_NUMURI_0			(URI_M90E26_P_APP_N_0 - URI_M90E26_E_ACT_FWD_0 + 1)
	#define	M90E26_NUMURI_1			(URI_M90E26_P_APP_N_1 - URI_M90E26_E_ACT_FWD_1 + 1)
#else
	#define	M90E26_NUMURI_0			(URI_M90E26_P_APP_L_0 - URI_M90E26_E_ACT_FWD_0 + 1)
	#define	M90E26_NUMURI_1			(URI_M90E26_P_APP_L_1 - URI_M90E26_E_ACT_FWD_1 + 1)
#endif

#if (M90E26_NUMURI_0 != M90E26_NUMURI_1)
	#error	"error in m90e26 URI definitions!!!"
#endif

#define	M90E26_T_SNS				1000

// ###################################### Private variables #######################################

spi_device_interface_config_t	m90e26_config[HAL_M90E26] = {
	#if	(HAL_M90E26 > 0)
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
	#if	(HAL_M90E26 > 1)
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
};

spi_device_handle_t	m90e26_handle[HAL_M90E26];
SemaphoreHandle_t m90e26mutex[HAL_M90E26];
u8_t NumM90E26 = HAL_M90E26;

struct m90e26cfg_s m90e26Cfg = { 0 };

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
nvs_m90e26_t nvsM90E26default[HAL_M90E26] = {
	#if	(HAL_M90E26 > 0)
	{ {	0x00B9, 0xC1F3, 0x0000, 0x0000, 0x0000, 0x0000, 0x08BD, 0x8100, 0x0AEC, 0x8100, 0x9422 },
	  {	0x6C50, 0x7C2A, 0x7530, 0x0000, 0xF711, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 },
	  {	0x0030, 0x1F2F, 0x0000	}, },
	#endif
	#if	(HAL_M90E26 > 1)
	{ {	0x00B9, 0xC1F3, 0x0000, 0x0000, 0x0000, 0x0000, 0x08BD, 0x8100, 0x0AEC, 0x8100, 0x9422 },
	  {	0x6C50, 0x7C2A, 0x7530, 0x0000, 0xF800, 0xF400, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF },
  	  {	0x0030, 0x1F2F, 0x0000	}, },
	#endif
};

#define	m90e26STEP_CONTRAST		0x04
#define	m90e26STAT_INTVL		pdMS_TO_TICKS(2 * MILLIS_IN_SECOND)

// ############################### common support routines #########################################

void m90e26WriteU16(u8_t eCh, u8_t address, u16_t val) {
	IF_myASSERT(debugPARAM, address < 0x70 && m90e26_handle[eCh]);
	xRtosSemaphoreTake(&m90e26mutex[eCh], portMAX_DELAY);
	spi_transaction_t m90e26_buf;
	memset(&m90e26_buf, 0, sizeof(m90e26_buf));
	m90e26_buf.length		= 8 * 3;
	m90e26_buf.flags 		= SPI_TRANS_USE_TXDATA;
	m90e26_buf.tx_data[0]	= address;
	m90e26_buf.tx_data[1]	= val >> 8;
	m90e26_buf.tx_data[2]	= val & 0xFF;
	IF_EXEC_1(debugTIMING, xSysTimerStart, stM90EX6W);
	ESP_ERROR_CHECK(spi_device_transmit(m90e26_handle[eCh], &m90e26_buf));
	IF_EXEC_1(debugTIMING, xSysTimerStop, stM90EX6W);
	xRtosSemaphoreGive(&m90e26mutex[eCh]);
	IF_PX(debugWRITE, "TX: addr=%02x d0=%02x d1=%02x\r\n", m90e26_buf.tx_data[0], m90e26_buf.tx_data[1], m90e26_buf.tx_data[2]);
}

u16_t m90e26ReadU16(u8_t eCh, u8_t address) {
	IF_myASSERT(debugPARAM, address < 0x70 && m90e26_handle[eCh]);
	xRtosSemaphoreTake(&m90e26mutex[eCh], portMAX_DELAY);
	spi_transaction_t m90e26_buf;
	memset(&m90e26_buf, 0, sizeof(m90e26_buf));
	m90e26_buf.length		= 8 * 3;
	m90e26_buf.flags 		= SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
	m90e26_buf.tx_data[0]	= address | 0x80;
	IF_EXEC_1(debugTIMING, xSysTimerStart, stM90EX6R);
	ESP_ERROR_CHECK(spi_device_transmit(m90e26_handle[eCh], &m90e26_buf));
	IF_EXEC_1(debugTIMING, xSysTimerStop, stM90EX6R);
	xRtosSemaphoreGive(&m90e26mutex[eCh]);
	IF_PX(debugREAD, "RX: addr=%02x  d0=%02x  d1=%02x  dx=%04x\r\n", m90e26_buf.tx_data[0], m90e26_buf.rx_data[1], m90e26_buf.rx_data[2], (m90e26_buf.rx_data[1] << 8) | m90e26_buf.rx_data[2]);
	return (m90e26_buf.rx_data[1] << 8) | m90e26_buf.rx_data[2];
}

void m90e26WriteRegister(u8_t eCh, u8_t Reg, u16_t Val) {
	if (INRANGE(PLconstH, Reg, MET_MODE)) {
		if (m90e26ReadU16(eCh, CALSTART) != CODE_START) {
			SL_NOT("CALSTART (x20) in wrong state, must be x5678");
		} else {
			m90e26WriteU16(eCh, Reg, Val);				// write new value & update CRC
			m90e26WriteU16(eCh, CRC_1, m90e26ReadU16(eCh, CRC_1));
			IF_PX(debugTRACK && xOptionGet(ioM90write), "Before: #%d %'-hY\r\n", Reg-PLconstH, SO_MEM(nvs_m90e26_t, calreg), &nvsM90E26default[eCh].calreg);
			nvsM90E26default[eCh].calreg[Reg-PLconstH] = Val;
			IF_PX(debugTRACK && xOptionGet(ioM90write), "After : #%d %'-hY\r\n", Reg-PLconstH, SO_MEM(nvs_m90e26_t, calreg), &nvsM90E26default[eCh].calreg);
		}
	} else if (INRANGE(U_GAIN, Reg, Q_OFST_N)) {
		if (m90e26ReadU16(eCh, ADJSTART) != CODE_START) {
			SL_NOT("ADJSTART (x30) in wrong state, must be x5678");
		} else {
			m90e26WriteU16(eCh, Reg, Val);				// write new value & update CRC
			m90e26WriteU16(eCh, CRC_2, m90e26ReadU16(eCh, CRC_2));
			IF_PX(debugTRACK && xOptionGet(ioM90write), "Before: #%d %'-hY\r\n", Reg-U_GAIN, SO_MEM(nvs_m90e26_t, adjreg), &nvsM90E26default[eCh].adjreg);
			nvsM90E26default[eCh].adjreg[Reg-U_GAIN] = Val;
			IF_PX(debugTRACK && xOptionGet(ioM90write), "After : #%d %'-hY\r\n", Reg-U_GAIN, SO_MEM(nvs_m90e26_t, adjreg), &nvsM90E26default[eCh].adjreg);
		}
	} else if (Reg == SOFTRESET || INRANGE(FUNC_ENAB, Reg, POWER_MODE) || Reg == CALSTART || Reg == ADJSTART) {
		m90e26WriteU16(eCh, Reg, Val);				// write new value
		if (INRANGE(FUNC_ENAB, Reg, POWER_MODE)) {
			IF_PX(debugTRACK && xOptionGet(ioM90write), "Before: #%d %'-hY\r\n", Reg-FUNC_ENAB, SO_MEM(nvs_m90e26_t, cfgreg), &nvsM90E26default[eCh].cfgreg);
			nvsM90E26default[eCh].cfgreg[Reg-FUNC_ENAB] = Val;
			IF_PX(debugTRACK && xOptionGet(ioM90write), "After : #%d %'-hY\r\n", Reg-FUNC_ENAB, SO_MEM(nvs_m90e26_t, cfgreg), &nvsM90E26default[eCh].cfgreg);
		}
	} else {
		SL_ERR("Invalid register=0x%02X", Reg);
	}
}

u16_t m90e26ReadModifyWrite(u8_t eCh, u8_t Addr, u16_t Value, u16_t Mask) {
	IF_PX(debugRMW, "  C=%d  R=%d  &=x%04X  |=x%04X", eCh, Addr, Value, Mask);
	u16_t CurValue = m90e26ReadU16(eCh, Addr);
	IF_PX(debugRMW, "  V=x%04X", CurValue);
	CurValue &= ~Mask;
	IF_PX(debugRMW, " -> x%04X", CurValue);
	CurValue |= Value;
	IF_PX(debugRMW, " -> x%04X\r\n", CurValue);
	m90e26WriteRegister(eCh, Addr, CurValue);
	return CurValue;
}

i16_t m90e26ReadI16S(u8_t eCh, u8_t Reg) {
	u16_t RawVal = m90e26ReadU16(eCh, Reg);
	bool Sign = RawVal & 0x8000 ? true : false;
	i16_t i16Val = RawVal & 0x7FFF;
	i16_t ConVal = Sign == true ? -1 * i16Val : i16Val;
	return ConVal;
}

i16_t m90e26ReadI16TC(u8_t eCh, u8_t Reg) { return ~m90e26ReadU16(eCh, Reg) + 1; }

u32_t m90e26ReadU32(u8_t eCh, u8_t Reg) {
	#if	(m90e26READ_32BIT == 1)
	return (m90e26ReadU16(eCh, Reg) << 16) + m90e26ReadU16(eCh, LSB);
	#else
	return (m90e26ReadU16(eCh, Reg) << 16);
	#endif
}

i32_t m90e26ReadI32TC(u8_t eCh, u8_t Reg) { return ~m90e26ReadU32(eCh, Reg) + 1; }

int	m90e26LoadNVSConfig(u8_t eCh, u8_t Idx) {
	IF_myASSERT(debugPARAM, Idx < m90e26CALIB_NUM);
	size_t	SizeBlob = m90e26CALIB_NUM * sizeof(nvs_m90e26_t);
	nvs_m90e26_t * psCalib = malloc(SizeBlob);
	int iRV = halFlashReadBlob(halFLASH_STORE, m90e26STORAGE_KEY, psCalib, &SizeBlob);
	if (iRV == ESP_OK && SizeBlob == (m90e26CALIB_NUM * sizeof(nvs_m90e26_t))) {
		psCalib += Idx;									// write the FuncEnab, Vsag Threshold and PowerMode registers
		for (int i = 0; i < NO_ELEM(nvs_m90e26_t, cfgreg); m90e26WriteU16(eCh, i+FUNC_ENAB, psCalib->cfgreg[i]), ++i);

		m90e26WriteU16(eCh, CALSTART, CODE_START);		// write the configuration registers with METER calibration data
		for (int i = 0; i < NO_ELEM(nvs_m90e26_t, calreg); m90e26WriteRegister(eCh, i+PLconstH, psCalib->calreg[i]), ++i);
		IF_EXEC_3(m90e26CALIB_CHECK > 0, m90e26WriteU16, eCh, CALSTART, CODE_CHECK);

		m90e26WriteU16(eCh, ADJSTART, CODE_START);		// write the configuration registers with MEASURE calibration data
		for (int i = 0; i < NO_ELEM(nvs_m90e26_t, adjreg); m90e26WriteRegister(eCh, i+U_GAIN, psCalib->adjreg[i]), ++i);
		IF_EXEC_3(m90e26CALIB_CHECK > 0, m90e26WriteU16, eCh, ADJSTART, CODE_CHECK);
	} else {
		SL_ERR("Failed Ch=%d Cfg=%d iRV=%d", eCh, Idx, iRV);
	}
	free (psCalib);
	return iRV;
}

u8_t m90e26CalcInfo(epw_t * psEW) {
	xEpWorkToUri(psEW);
	psEW->idx = psEW->uri - URI_M90E26_E_ACT_FWD_0;
	psEW->eChan = 0;
	// In case of multiple channels, adjust till in range....
	while (psEW->idx >= M90E26_NUMURI_0) {
		psEW->idx -= M90E26_NUMURI_0;
		psEW->eChan++;
	}
	IF_myASSERT(debugRESULT, (psEW->idx < M90E26_NUMURI_0) && (psEW->eChan < HAL_M90E26));
	return psEW->idx;
}

// ############################## identification & initialization ##################################

int	m90e26Identify(u8_t eCh) {
	IF_myASSERT(debugPARAM, eCh < NumM90E26);
	ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &m90e26_config[eCh], &m90e26_handle[eCh]));
	m90e26mutex[eCh]	= xSemaphoreCreateMutex();
	IF_myASSERT(debugRESULT, m90e26mutex[eCh]);

	int	Uri0 = (eCh == 0) ? URI_M90E26_E_ACT_FWD_0	: URI_M90E26_E_ACT_FWD_1;
	#if	(m90e26NEUTRAL > 0)
	int UriX = (eCh == 0) ? URI_M90E26_P_APP_N_0		: URI_M90E26_P_APP_N_1;
	#else
	int UriX = (eCh == 0) ? URI_M90E26_P_APP_L_0		: URI_M90E26_P_APP_L_1;
	#endif
	for (int i = Uri0; i <= UriX; ++i) {
		epw_t * psEW = &table_work[i];
		if (i < (Uri0 + 6)) psEW->var.def.cv.sumX = 1;
		psEW->var.def = SETDEF_CVAR(0,0,vtVALUE,cvF32,1,0,0);
		psEW->Tsns = psEW->Rsns = M90E26_T_SNS;		// start sensing
	}
	return erSUCCESS;
}

int m90e26Config(void) {
	const spi_bus_config_t buscfg = {
		.mosi_io_num	= m90e26VSPI_MOSI,
		.miso_io_num	= m90e26VSPI_MISO,
		.sclk_io_num	= m90e26VSPI_SCLK,
		.quadwp_io_num	= -1,
		.quadhd_io_num	= -1,
		.max_transfer_sz = 128,
		.flags = 0
	};
	ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH1));

	for (s8_t eCh = 0; eCh < HAL_M90E26; ++eCh) {
		if ((m90e26Identify(eCh) != erSUCCESS) || (m90e26Init(eCh) != erSUCCESS)) {
			SL_ERR("Failed to identify/init #%d", eCh);
			return erFAILURE;
		}
	}
	return erSUCCESS;
}

/**
 * m90e26Init() -
 */
int	m90e26Init(u8_t eCh) {
	IF_myASSERT(debugPARAM, eCh < NumM90E26);
	IF_SYSTIMER_INIT(debugTIMING, stM90EX6R, stMICROS, "M90E26RD", 1500, 15000);
	IF_SYSTIMER_INIT(debugTIMING, stM90EX6W, stMICROS, "M90E26WR", 1500, 15000);
	/* Check that blob with CALibration and ADJustment values exists
	 * If not existing, create with factory defaults as first record */
	size_t	SizeBlob = m90e26CALIB_NUM * sizeof(nvs_m90e26_t);
	nvs_m90e26_t * psCalib = malloc(SizeBlob);
	int iRV = halFlashReadBlob(halFLASH_STORE, m90e26STORAGE_KEY, psCalib, &SizeBlob);
	if ((iRV != ESP_OK) || (SizeBlob != (m90e26CALIB_NUM * sizeof(nvs_m90e26_t)))) {
		memset(psCalib, 0, SizeBlob = m90e26CALIB_NUM * sizeof(nvs_m90e26_t));
		memcpy(psCalib, &nvsM90E26default, sizeof(nvsM90E26default));
		iRV = halFlashWriteBlob(halFLASH_STORE, m90e26STORAGE_KEY, psCalib, m90e26CALIB_NUM * sizeof(nvs_m90e26_t));
		IF_myASSERT(debugRESULT, iRV == ESP_OK);
	}
	free(psCalib);

	m90e26WriteU16(eCh, SOFTRESET, CODE_RESET);			// start with default values, not running, no valid values
	m90e26LoadNVSConfig(eCh, 0);						// load config #0 from NVS blob as default
	m90e26Cfg.Chan[eCh].L_Gain = 1;						// set default state
	#if	(m90e26NEUTRAL > 0)
	m90e26Cfg.Chan[eCh].N_Gain = 1;
	#endif
	m90e26Cfg.Chan[eCh].E_Scale = 0;					// Wh not kWh
	m90e26Cfg.Chan[eCh].P_Scale = 0;					// W not kW
	m90e26Cfg.Chan[eCh].I_Scale = 0;					// A not mA
	iRV = (m90e26GetSysStatus(eCh) & 0xF000) ? erFAILURE : erSUCCESS;
	if (iRV > erFAILURE) halEventUpdateDevice(devMASK_M90E26, 1);
	return iRV;
}

// ########################### 32 bit value endpoint support functions #############################

int	m90e26SenseCurrent(epw_t * psEW) {
	u8_t eIdx = m90e26CalcInfo(psEW);
	float f32Val = (float) m90e26ReadU32(psEW->eChan, m90e26RegAddr[eIdx]);
	IF_PX(debugCURRENT, "Irms: URI=%d  Idx=%d  Reg=%02X  Ch=%d", psEW->uri, eIdx, m90e26RegAddr[eIdx], psEW->eChan);
	if (m90e26Cfg.Chan[psEW->eChan].I_Scale == 0) {
		f32Val /= 65536000.0;							// convert to Amp
		IF_PX(debugCURRENT, "  Val=%4.5fA", f32Val);
	} else {
		f32Val /= 65536.0;								// convert to mA
		IF_PX(debugCURRENT, "  Val=%4.5fmA", f32Val);
	}

	#if	(m90e26NEUTRAL > 0)
	IF_myASSERT(debugRESULT, eIdx == eI_RMS_L || eIdx == eI_RMS_N);
	if (eIdx == eI_RMS_L) {
		f32Val /= m90e26Cfg.Chan[psEW->eChan].L_Gain;
		IF_PX(debugCURRENT, "  Lgain=%d", m90e26Cfg.Chan[psEW->eChan].L_Gain);
	} else {
		f32Val /= m90e26Cfg.Chan[psEW->eChan].N_Gain;
		IF_PX(debugCURRENT, "  Ngain=%d", m90e26Cfg.Chan[psEW->eChan].N_Gain);
	}
	#else
	IF_myASSERT(debugRESULT, eIdx == eI_RMS_L);
	f32Val /= m90e26Cfg.Chan[psEW->eChan].L_Gain;
	IF_PX(debugCURRENT, "  Lgain=%d", m90e26Cfg.Chan[psEW->eChan].L_Gain);
	#endif
	IF_PX(debugCURRENT, "  Act=%4.5f\r\n", f32Val);
	xEpSetValue(psEW, (x32_t) f32Val);
	return erSUCCESS;
}

int	m90e26SenseVoltage(epw_t * psEW) {		// OK
	m90e26CalcInfo(psEW);
	float	f32Val	= (float) m90e26ReadU32(psEW->eChan, m90e26RegAddr[psEW->idx]);
	f32Val			/= 6553600.0;						// Change mV to V
	xEpSetValue(psEW, (x32_t) f32Val);
//	IF_PX(debugVOLTS, "Vrms: Ch=%d  Val=%9.3f\r\n", psEW->eChan, f32Val);
	return erSUCCESS;
}

int	m90e26SensePower(epw_t * psEW) {
	m90e26CalcInfo(psEW);
	float	f32Val	= (float) m90e26ReadI32TC(psEW->eChan, m90e26RegAddr[psEW->idx]);
	f32Val /= (m90e26Cfg.Chan[psEW->eChan].P_Scale == 1) ? 65536000.0 : 65536.0;
	xEpSetValue(psEW, (x32_t) f32Val);
	IF_PX(debugPOWER, "Power: Ch=%d  Reg=0x%02X  Val=%9.3f\r\n", psEW->eChan, m90e26RegAddr[psEW->idx], f32Val);
	return erSUCCESS;
}

// ########################### 16 bit value endpoint support functions #############################

int	m90e26SenseEnergy(epw_t * psEW) {
	if (psEW->var.def.cv.sumX) {	// if just a normal update cycle
		m90e26CalcInfo(psEW);
		float f32Val	= (float) m90e26ReadU16(psEW->eChan, m90e26RegAddr[psEW->idx]);
		f32Val	/= m90e26Cfg.Chan[psEW->eChan].E_Scale ? 10000.0 : 10.0;
		xEpSetValue(psEW, (x32_t) f32Val);
		// Update running total in NVS memory
		sRTCvars.aRTCsum[psEW->eChan][psEW->idx] += f32Val;
	} else {						// else it is a value reset call
		vCV_ResetValue(&psEW->var);
	}
	return erSUCCESS;
}

int	m90e26SenseFrequency(epw_t * psEW) {
	m90e26CalcInfo(psEW);
	float f32Val = (float) m90e26ReadU16(psEW->eChan, m90e26RegAddr[psEW->idx]) / 100.0;
	xEpSetValue(psEW, (x32_t) f32Val);
	return erSUCCESS;
}

int	m90e26SensePowerFactor(epw_t * psEW) {
	m90e26CalcInfo(psEW);
	float f32Val = (float)  m90e26ReadI16S(psEW->eChan, m90e26RegAddr[psEW->idx]) / 1000.0;
	xEpSetValue(psEW, (x32_t) f32Val);
	return erSUCCESS;
}

int	m90e26SensePowerAngle(epw_t * psEW) {
	m90e26CalcInfo(psEW);
	float f32Val = (float) m90e26ReadI16S(psEW->eChan, m90e26RegAddr[psEW->idx]) / 10.0;
	xEpSetValue(psEW, (x32_t) f32Val);
	return erSUCCESS;
}

inline u16_t m90e26GetSysStatus(u8_t eCh) { return m90e26ReadU16(eCh, SYS_STATUS); }

inline u16_t m90e26GetMeterStatus(u8_t eCh)	{ return m90e26ReadU16(eCh, MET_STATUS); }

// ############################################ GUI Support ########################################

#if (HAL_SSD1306 > 0 && appGUI > 0)

#include "ssd1306.h"

static TickType_t NextTick = 0;
static u8_t Index = 0;
static TimerHandle_t m90e26TH = { 0 };
static StaticTimer_t m90e26TS = { 0 };

extern char DispBuf[];
extern char * pReqBuf;

static void m90e26GuiUpdateInfo(u8_t Index) {
	epw_t * psEW;
	#if	(HAL_M90E26 == 2)
	u8_t eCh = Index / NumM90E26;
	psEW = &table_work[(eCh == 0) ? URI_M90E26_E_ACT_FWD_0 : URI_M90E26_E_ACT_FWD_1];
	#else
	psEW = &table_work[URI_M90E26_E_ACT_FWD_0];
	#endif
	if ((Index % 2) == 0) {
		snprintfx(DispBuf, halLCD_MAX_CHAR,
		"Vo%8.3f" "Fr%8.3f" "Ir%8.3f" "Pa%8.3f" "An%8.3f" "Fa%8.3f",
		xCV_GetValueScaled(&psEW[eVOLTS].var, NULL),
		xCV_GetValueScaled(&psEW[eFREQ].var, NULL),
		xCV_GetValueScaled(&psEW[eI_RMS_L].var, NULL),
		xCV_GetValueScaled(&psEW[eP_ACT_L].var, NULL),
		xCV_GetValueScaled(&psEW[eP_ANGLE_L].var, NULL),
		xCV_GetValueScaled(&psEW[eP_FACTOR_L].var, NULL));
	} else {
		snprintfx(DispBuf, halLCD_MAX_CHAR,
		"Af%8.3f" "Ar%8.3f" "Aa%8.3f" "Rf%8.3f" "Rr%8.3f" "Ra%8.3f",
		xCV_GetValueScaled(&psEW[eE_ACT_FWD].var, NULL),
		xCV_GetValueScaled(&psEW[eE_ACT_REV].var, NULL),
		xCV_GetValueScaled(&psEW[eE_ACT_ABS].var, NULL),
		xCV_GetValueScaled(&psEW[eE_REACT_FWD].var, NULL),
		xCV_GetValueScaled(&psEW[eE_REACT_REV].var, NULL),
		xCV_GetValueScaled(&psEW[eE_REACT_ABS].var, NULL));
	}
	pReqBuf = DispBuf;				// enable next text display
}

void m90e26GuiTimerInit(void) {
	m90e26TH = xTimerCreateStatic("m90e26", pdMS_TO_TICKS(2000), pdTRUE, &m90e26TH, m90e26GuiTimerHandler, &m90e26TS);
	IF_myASSERT(debugRESULT, m90e26TH != 0);
	int iRV = xTimerStart(m90e26TH, 0);
	IF_myASSERT(debugRESULT, iRV != pdFAIL);
}

void m90e26GuiTimerDeInit(void) {
	int iRV = xTimerStop(m90e26TH, 0);
	IF_myASSERT(debugRESULT, iRV != pdFAIL);
	iRV = xTimerDelete(m90e26TH, 0);
	IF_myASSERT(debugRESULT, iRV != pdFAIL);
}

void m90e26GuiTimerHandler(TimerHandle_t xTimer) {
	// GUI not (yet) running or set to delete or M90E26/SSD1306 not yet initialized
	if (halEventCheckTasksOK(taskGUI_MASK) == 0 || halEventCheckDevice(devMASK_SSD1306|devMASK_M90E26) == 0)
		return;
	TickType_t CurTick = xTaskGetTickCount();
	if (NextTick == 0) NextTick = CurTick;
	else if (NextTick > CurTick) return;
	NextTick = CurTick + m90e26STAT_INTVL;
	if (ssd1306GetDisplayState()) m90e26GuiUpdateInfo(Index);
	++Index;
	Index %= (NumM90E26 * 2);
	if (Index == 0) ssd1306StepContrast(m90e26STEP_CONTRAST);
}

#endif	// HAL_SSD1306 && appGUI

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

int m90e26ReportCalib(report_t * psR) {
	int iRV = xReport(psR, "%C%s%C\r\n", colourFG_CYAN, HDR_CALIB HDR_MMODE, attrRESET);
	for (int eCh = 0; eCh < NumM90E26; ++eCh) {
		iRV += xReport(psR, "%2d", eCh);
		for (int i = CALSTART; i <= CRC_1; iRV += xReport(psR, "  0x%04X", m90e26ReadU16(eCh, i++)));
		m90e26meter_mode_t	MeterMode = (m90e26meter_mode_t) m90e26ReadU16(eCh, MET_MODE);
		u8_t	Pthres[16] = { 200, 100, 50, 25, 16, 32, 48, 64,80, 96, 112, 128, 144, 160, 176, 192 };
		iRV += xReport(psR, "  %-2s  %2s %7s %7s %7s %7s  %6s %2.4f\r\n",
			MeterMode.Lgain == 3 ? "24" : MeterMode.Lgain == 2 ? "16" : MeterMode.Lgain == 1 ? "8" : MeterMode.Lgain == 0 ? "4" : "1",
			MeterMode.Ngain == 0 ? "2" : MeterMode.Ngain == 1 ? "4" : "1",
			MeterMode.LNSel ? "Live" : "Neutral",
			MeterMode.DisHPF == 0 ? "Ena Ena" : MeterMode.DisHPF == 1 ? "Ena Dis" : MeterMode.DisHPF == 2 ? "Dis Ena" : "Dis Dis",
			MeterMode.Amod	? "Abs" : "Act",
			MeterMode.Rmod	? "Abs" : "ReAct",
			MeterMode.Zxcon	== 0 ? "AllPos" : MeterMode.Zxcon	== 1 ? "AllNeg" : MeterMode.Zxcon	== 2 ? "All+/-" : "None",
			(float) Pthres[MeterMode.Pthresh] / 16);
	}
	return iRV;
}

int m90e26ReportAdjust(report_t * psR) {
	int iRV = xReport(psR, "%C%s%C\r\n", colourFG_CYAN, HDR_ADJUST, attrRESET);
	for (int eCh = 0; eCh < NumM90E26; ++eCh) {
		iRV += xReport(psR, "%2d", eCh);
		for (int i = ADJSTART; i <= CRC_2; iRV += xReport(psR, "  0x%04X", m90e26ReadU16(eCh, i++)));
		iRV += xReport(psR, strNL);
	}
	return iRV;
}

int m90e26ReportData(report_t * psR) {
	const u8_t m90e26DataReg[] = {
		E_ACT_FWD, E_ACT_REV, E_ACT_ABS, E_REACT_FWD, E_REACT_REV, E_REACT_ABS,
		I_RMS_L, V_RMS, P_ACT_L, P_REACT_L, FREQ, P_FACTOR_L, P_ANGLE_L, P_APP_L,
		I_RMS_N, P_ACT_N, P_REACT_N, P_FACTOR_N, P_ANGLE_N, P_APP_N,
	};
	int iRV = xReport(psR, "%C" HDR_DATA_LIVE HDR_DATA_NEUT "%C\r\n", colourFG_CYAN, attrRESET);
	for (int eCh = 0; eCh < NumM90E26; ++eCh) {
		iRV += xReport(psR, "%2d", eCh);
		for (int i = 0; i < eNUM_DATA_REG; ++i) {
			if (i < 6) {								// For energy registers reading it will reset the value...
				iRV += xReport(psR, " %7.2g", sRTCvars.aRTCsum[eCh][i]);
			} else {
				iRV += xReport(psR, "  0x%04X", m90e26ReadU16(eCh, m90e26DataReg[i]));
			}
		}
		iRV += xReport(psR, strNL);
	}
	return iRV;
}

int m90e26ReportStatus(report_t * psR) {
	int iRV = xReport(psR, "%C%s%C\r\n", colourFG_CYAN, HDR_STATUS, attrRESET);
	for (int eCh = 0; eCh < NumM90E26; ++eCh) {
		m90e26system_stat_t SysStatus = (m90e26system_stat_t) m90e26GetSysStatus(eCh);
		iRV += xReport(psR, "%2d  0x%04X", eCh, SysStatus.val);
		iRV += xReport(psR, SysStatus.CalErr		? "  Error "	: BLANK8);
		iRV += xReport(psR, SysStatus.AdjErr		? "  Error "	: BLANK8);
		iRV += xReport(psR, SysStatus.LnChge		? " L/N chg"	: BLANK8);
		iRV += xReport(psR, SysStatus.RevQchg		? " DIR chg"	: BLANK8);
		iRV += xReport(psR, SysStatus.RevPchg		? " DIR chg"	: BLANK8);
		iRV += xReport(psR, SysStatus.SagWarn		? "  V sag "	: BLANK8);
		m90e26meter_stat_t MeterStatus = (m90e26meter_stat_t) m90e26GetMeterStatus(eCh);
		iRV += xReport(psR, "  0x%04X", MeterStatus.val);
		iRV += xReport(psR, MeterStatus.Qnoload	? " NO Load"	: BLANK8);
		iRV += xReport(psR, MeterStatus.Pnoload	? " NO Load"	: BLANK8);
		iRV += xReport(psR, MeterStatus.RevQ		? " Reverse"	: BLANK8);
		iRV += xReport(psR, MeterStatus.RevP		? " Reverse"	: BLANK8);
		iRV += xReport(psR, MeterStatus.Line 		? "    Live"	: " Neutral");
		iRV += xReport(psR, MeterStatus.LNMode==3 ? "    Flex"	:
			  MeterStatus.LNMode==2 ? "    Both"	:
			  MeterStatus.LNMode==1 ? "    Live"	: "  Tamper");
		iRV += xReport(psR, strNL);
	}
	return iRV;
}

int m90e26Report(report_t * psR) {
	int iRV = m90e26ReportCalib(psR);
	iRV += m90e26ReportAdjust(psR);
	iRV += m90e26ReportData(psR);
	iRV += m90e26ReportStatus(psR);
#if (HAL_SSD1306 > 0 && halGUI > 0)
	iRV += xRtosReportTimer(psR, m90e26TH);
#endif
	return iRV;
}

#endif	// HAL_M90E26


/* ################################### OLD CODE #####################################

u16_t m90e26CalcCRC(u8_t eCh, u8_t Addr0, int8_t Count) {
	u8_t Lcrc = 0, Hcrc = 0;
	u16_t RegData[Count];
	for (int i = 0; i < Count; ++i) {				// read the range of registers
		RegData[i] = m90e26Read(eCh, Addr0 + i);
	}
	for (int i = 0; i < Count; ++i) {				// HI bytes: MOD256 sum & XOR
		Lcrc += RegData[i] >> 8;
		Hcrc ^= RegData[i] >> 8;
	}
	for (int i = 0; i < Count; ++i) {				// LO bytes: MOD256 sum & XOR
		Lcrc += RegData[i] & 0xFF;
		Hcrc ^= RegData[i] & 0xFF;
	}
	IF_PX(debugCRC, "CRC=%04x from %-'h\r\n", (Hcrc << 8) | Lcrc, Count * 2, RegData);
	return (Hcrc << 8) | Lcrc;
}

void	m90e26HandleCRC(u8_t eCh, u8_t RegAddr1, u8_t RegAddr2) {
	IF_myASSERT(debugPARAM, eCh<NumM90E26 && ((RegAddr1==CALSTART && RegAddr2==CRC_1) || (RegAddr1==ADJSTART && RegAddr2==CRC_2)));
	m90e26Write(eCh, RegAddr2, m90e26CalcCRC(eCh, RegAddr1 + 1, RegAddr2 - RegAddr1 - 1));
	m90e26Write(eCh, RegAddr1, CODE_CHECK);
}

 */
