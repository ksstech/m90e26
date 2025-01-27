/*
 * m90e26.h
 */

#pragma once

#include "definitions.h"
#include "hal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Notes & references:
 * mode option to select WHr or KWHr, set default, variable used as divisor
 *
 * SPI is Full duplex, synchronous, MSB first
 *
 * https://www.hackster.io/whatnick/atm90e26-and-esp8266-energy-monitoring-cdac92
 * https://imgur.com/a/BsEUM
 * https://www.reddit.com/r/diyelectronics/
 */

// ##################################### BUILD definitions #########################################

#define	m90e26NEUTRAL				0

#define	m90e26VSPI_CS1				GPIO_NUM_5			// D8
#define	m90e26VSPI_CS2				GPIO_NUM_17			// D3
#define	m90e26VSPI_SCLK				GPIO_NUM_18
#define	m90e26VSPI_MISO				GPIO_NUM_19
#define	m90e26VSPI_MOSI				GPIO_NUM_23

// ############################################# Macros ############################################

#define SOFTRESET			0x00	// Software Reset
#define SYS_STATUS			0x01	// System Status
#define FUNC_ENAB			0x02	// Function Enable
#define V_SAG_THR			0x03 	// Voltage Sag Threshold
#define POWER_MODE			0x04 	// Small-Power Mode
#define LASTDATA			0x06	// Last Read/Write SPI/UART Value
#define LSB					0x08 	// RMS/Power 16-bit LSB

#define CALSTART			0x20 	// Calibration Start Command
#define PLconstH			0x21 	// High Word of PL_Constant
#define PLconstL			0x22 	// Low Word of PL_Constant
#define L_GAIN				0x23 	// L Line Calibration Gain
#define L_PHI				0x24 	// L Line Calibration Angle
#define N_GAIN				0x25 	// N Line Calibration Gain
#define N_PHI				0x26 	// N Line Calibration Angle
#define P_SUP_TH			0x27 	// Active Startup Power Threshold
#define P_NOL_TH			0x28 	// Active No-Load Power Threshold
#define Q_SUP_TH			0x29 	// Reactive Startup Power Threshold
#define Q_NOL_TH			0x2A 	// Reactive No-Load Power Threshold
#define MET_MODE			0x2B 	// Metering Mode Configuration
#define CRC_1				0x2C 	// Checksum 1

#define ADJSTART			0x30 	// Measurement Calibration Start Command
#define U_GAIN				0x31 	// Voltage rms Gain
#define I_GAIN_L			0x32 	// L Line Current rms Gain
#define I_GAIN_N			0x33 	// N Line Current rms Gain
#define V_OFFSET			0x34 	// Voltage Offset
#define I_OFST_L			0x35 	// L Line Current Offset
#define I_OFST_N			0x36 	// N Line Current Offset
#define P_OFST_L			0x37 	// L Line Active Power Offset
#define Q_OFST_L 			0x38 	// L Line Reactive Power Offset
#define P_OFST_N 			0x39 	// N Line Active Power Offset
#define Q_OFST_N 			0x3A 	// N Line Reactive Power Offset
#define CRC_2 				0x3B 	// Checksum 2

// Energy registers
#define E_ACT_FWD 			0x40 	// Forward Active Energy
#define E_ACT_REV 			0x41 	// Reverse Active Energy
#define E_ACT_ABS 			0x42 	// Absolute Active Energy
#define E_REACT_FWD			0x43 	// Forward (Inductive) Reactive Energy
#define E_REACT_REV 		0x44 	// Reverse (Capacitive) Reactive Energy
#define E_REACT_ABS 		0x45 	// Absolute Reactive Energy

#define MET_STATUS 			0x46 	// Metering Status

#define I_RMS_L				0x48 	// L Line Current rms
#define V_RMS				0x49 	// Voltage rms
#define P_ACT_L				0x4A 	// L Line Mean Active Power
#define P_REACT_L			0x4B 	// L Line Mean Reactive Power
#define FREQ				0x4C 	// Voltage Frequency
#define P_FACTOR_L			0x4D 	// L Line Power Factor
#define P_ANGLE_L			0x4E 	// L Line Phase Angle between Voltage and Current
#define P_APP_L 			0x4F 	// L Line Mean Apparent Power

#define I_RMS_N 			0x68 	// N Line Current rms
#define P_ACT_N 			0x6A 	// N Line Mean Active Power
#define P_REACT_N 			0x6B 	// N Line Mean Reactive Power
#define P_FACTOR_N 			0x6D 	// N Line Power Factor
#define P_ANGLE_N 			0x6E 	// N Line Phase Angle between Voltage and Current
#define P_APP_N 			0x6F 	// N Line Mean Apparent Power

#define	CODE_DFALT			0x6886	// indicates default Power On status, not measuring
#define	CODE_START			0x5678	// Reset to defaults, start metering, allow calibration/adjustment
#define	CODE_CHECK			0x8765	// check calibration/adjustment, continue measurement if all OK
#define	CODE_RESET			0x789A	// trigger software reset
#define	CODE_POWER			0xA987	// Set small power mode

// Number of sets of calibration parameters stored in NVS
#define m90e26CALIB_NUM				4
#define m90e26CALIB_CHECK			1

// ######################################## Enumerations ###########################################

enum {								// configuration registers
	eCALSTART,
	ePLconstH,
	ePLconstL,
	eLgain,
	eLphi,
	eNgain,
	eNphi,
	ePStartTh,
	ePNolTh,
	eQStartTh,
	eQNolTh,
	eMMode,
	eCRC_1,
};

enum {								// sensor data registers
// Energy
 	eE_ACT_FWD,
 	eE_ACT_REV,
 	eE_ACT_ABS,
 	eE_REACT_FWD,
 	eE_REACT_REV,
 	eE_REACT_ABS,
// Live line
 	eI_RMS_L,
 	eVOLTS,
 	eP_ACT_L,
 	eP_REACT_L,
 	eFREQ,
 	eP_FACTOR_L,
 	eP_ANGLE_L,
 	eP_APP_L,

#if		(m90e26NEUTRAL > 0)			// Neutral Line
 	eI_RMS_N,
 	eP_ACT_N,
 	eP_REACT_N,
 	eP_FACTOR_N,
 	eP_ANGLE_N,
 	eP_APP_N,
#endif
	eNUM_DATA_REG
 };

enum display_mode {
	eDM_DISABLED,
	eDM_NORMAL,						// always on, cycle through, step contrast
	eDM_CURRENT,					// if current != 0, then include
	eDM_BUTTON,
	eDM_MAXIMUM,
};


// ######################################### Structures ############################################

typedef struct {
 	u8_t	addr;
 	u8_t	flag;
 	u16_t	raw_val;
} conf_reg_t;
DUMB_STATIC_ASSERT(sizeof(conf_reg_t) == 4);

typedef union {
	struct {
		u8_t	r3 		: 1;			// LSB
		u8_t	SagWarn	: 1;
		u8_t	r2		: 3;
		u8_t	RevPchg	: 1;
		u8_t	RevQchg	: 1;
		u8_t	LnChge	: 1;
		u8_t	r1		: 4;
		u8_t	AdjErr	: 2;
		u8_t	CalErr	: 2;			// MSB
	};
	u16_t	val;
} m90e26system_stat_t;
DUMB_STATIC_ASSERT(sizeof(m90e26system_stat_t) == 2);

typedef union {
	struct {
		u8_t	LNMode	: 2;			// LCB
		u16_t	r1		: 9;
		u8_t	Line	: 1;
		u8_t	RevP	: 1;
		u8_t	RevQ	: 1;
		u8_t	Pnoload	: 1;
		u8_t	Qnoload	: 1;			// MSB
	};
	u16_t	val;
} m90e26meter_stat_t;
DUMB_STATIC_ASSERT(sizeof(m90e26meter_stat_t) == 2);

typedef union {
	struct {
		u8_t	Pthresh	: 4;			// LSB
		u8_t	Zxcon	: 2;
		u8_t	Rmod	: 1;
		u8_t	Amod	: 1;
		u8_t	DisHPF	: 2;
		u8_t	LNSel	: 1;
		u8_t	Ngain	: 2;
		u8_t	Lgain	: 3;			// MSB
	};
	u16_t	val;
} m90e26meter_mode_t;
DUMB_STATIC_ASSERT(sizeof(m90e26meter_mode_t) == 2);

typedef struct __attribute__((packed)) nvs_m90e26_t {
	u16_t	calreg[11];
	u16_t	adjreg[10];
	u16_t	cfgreg[3];
} nvs_m90e26_t;
DUMB_STATIC_ASSERT(sizeof(nvs_m90e26_t) == 48);

struct __attribute__((packed)) m90e26cfg_s{
	struct __attribute__((packed)) {
		u8_t	L_Gain	:5;								// 1 -> 24
		u8_t	N_Gain	:3;								// 1 -> 4
		u8_t	Spare	:2;
		u8_t	Display	:2;								// 0 = Off, 1 = Normal,	2 = Current, 3 = Button
		u8_t	E_Scale	:1;								// 0 = WHr,	1 = KwHr
		u8_t	P_Scale	:1;								// 0 = W,	1 = Kw
		u8_t	I_Scale	:1;								// 0 = A,	1 = KwHr
		bool	state;
	} Chan[HAL_M90E26];
};

// ####################################### Global variables ########################################

extern struct m90e26cfg_s m90e26Cfg;
extern u8_t NumM90E26;

// ####################################### Global functions ########################################

void m90e26WriteU16(u8_t eChan, u8_t address, u16_t val);
u16_t m90e26ReadU16(u8_t eChan, u8_t address);
u16_t m90e26ReadModifyWrite(u8_t eChan, u8_t Addr, u16_t Value, u16_t Mask);
i16_t m90e26ReadI16S(u8_t eChan, u8_t Reg);
i16_t m90e26ReadI16TC(u8_t eChan, u8_t Reg);
u32_t m90e26ReadU32(u8_t eChan, u8_t Reg);
i32_t m90e26ReadI32TC(u8_t eChan, u8_t Reg);

int	m90e26LoadNVSConfig(u8_t eChan, u8_t Idx);
void m90e26WriteRegister(u8_t eChan, u8_t Reg, u16_t Val);

int m90e26Config(void);
int	m90e26Identify(u8_t eDev);
int	m90e26Init(u8_t eDev);
void m90e26Calibrate(u8_t eChan);

void m90e26DataReadAll(u8_t eChan);
void m90e26DataConvertAll(u8_t eChan);

u16_t m90e26GetSysStatus(u8_t eChan);
u16_t m90e26GetMeterStatus(u8_t eChan);

struct epw_t;
int	m90e26SenseCurrent(struct epw_t *);
int	m90e26SenseVoltage(struct epw_t *);
int	m90e26SensePower(struct epw_t *);
int	m90e26SenseEnergy(struct epw_t *);
int	m90e26SenseFrequency(struct epw_t *);
int	m90e26SensePowerFactor(struct epw_t *);
int	m90e26SensePowerAngle(struct epw_t *);

struct rule_t;
int	m90e26ConfigMode(struct rule_t * psRule, int xCur, int Xmax);

struct report_t;
int m90e26Report(struct report_t * psR);

void m90e26GuiTimerInit(void);
void m90e26GuiTimerDeInit(void);
void m90e26GuiTimerHandler(TimerHandle_t xTimer);

#ifdef __cplusplus
}
#endif
