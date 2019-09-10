/*
 * Copyright 2018-19 AM Maree/KSS Technologies (Pty) Ltd.
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
 * m90e26.h
 */

#pragma		once

#include	<stdint.h>

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

#define M90E26_CALIB_TABLE		2						// 0=AMM, 1=Tisham, 2=DEFAULT
// Only effective if (M90E26_CALIB_TABLE == 0) above !!!
#define	M90E26_CALIB_SOFT		0						// enable software based calibration
#define	M90E26_CALIB_ITER		100						// number of READ iterations to determine mean value

#define	M90E26_LAST_DATA		1						// enable support for LASTDATA verification
#define	M90E26_NEUTRAL			1

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
#define V_GAIN				0x31 	// Voltage rms Gain
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

#define	PWR_ON				0x6886	// indicates default Power On status, not measuring
#define	STDCOD				0x5678	// trigger calibration/adjustment
#define	CFGCOD				0x8765	// check calibration/adjustment, start measurement if all OK
#define	RSTCOD				0x789A	// trigger software reset
#define	PWRCOD				0xA987	// Set small power mode

// ######################################## Enumerations ###########################################

enum {													// eChan available
#if		(halHAS_M90E26 > 0)
	M90E26_0,
#endif
#if		(halHAS_M90E26 > 1)
	M90E26_1,
#endif
	M90E26_NUM,
} ;

enum {													// configuration registers
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
} ;

enum {													// sensor data registers
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

#if		(M90E26_NEUTRAL == 1)							// Neutral Line
 	eI_RMS_N,
 	eP_ACT_N,
 	eP_REACT_N,
 	eP_FACTOR_N,
 	eP_ANGLE_N,
 	eP_APP_N,
#endif
	eNUM_DATA_REG
 } ;

enum {													// supported mode options
	eINVALID,
	eL_GAIN,											// 1, 4, 8, 16, 24
	eN_GAIN,											// 1, 2, 4
	eSOFTRESET,											// reset only
	eRECALIB,											// reset & recalibrate
	eBRIGHT,											//
	eDISPLAY,											// sensor values display on/off
	eBLANKING,											// set period for blanking between channels
} ;

enum display_mode {
	eDM_DISABLED,
	eDM_NORMAL,											// always on, cycle through, step contrast
	eDM_CURRENT,										// if current != 0, then include
	eDM_BUTTON,
	eDM_MAXIMUM,
} ;

enum { FACTORY = 0, CALIB1, CALIB2, CALIB3, CALIB_NUM} ;

// ######################################### Structures ############################################

typedef struct conf_reg_s {
 	uint8_t		addr ;
 	uint8_t		flag ;
 	uint16_t	raw_val ;
} conf_reg_t ;

typedef union {
	struct {
		uint8_t		r3 		: 1 ;			// LSB
		uint8_t		SagWarn	: 1 ;
		uint8_t		r2		: 3 ;
		uint8_t		RevPchg	: 1 ;
		uint8_t		RevQchg	: 1 ;
		uint8_t		LnChge	: 1 ;
		uint8_t		r1		: 4 ;
		uint8_t		AdjErr	: 2 ;
		uint8_t		CalErr	: 2 ;			// MSB
	} ;
	uint16_t	val ;
} m90e36system_stat_t ;

typedef union {
	struct {
		uint8_t		LNMode	: 2 ;			// LCB
		uint16_t	r1		: 9 ;
		uint8_t		Line	: 1 ;
		uint8_t		RevP	: 1 ;
		uint8_t		RevQ	: 1 ;
		uint8_t		Pnoload	: 1 ;
		uint8_t		Qnoload	: 1 ;			// MSB
};
	uint16_t	val ;
} m90e26meter_stat_t ;

struct m90e26_cal_s {
	uint16_t	calreg[12] ;
	uint16_t	adjreg[11] ;
	uint16_t	spare[9] ;
} ;
DUMB_STATIC_ASSERT(sizeof(struct m90e26_cal_s) == 64) ;

typedef	struct m90e26_cal_s nvs_m90e26_t ;

// See http://www.catb.org/esr/structure-packing/
// Also http://c0x.coding-guidelines.com/6.7.2.1.html

// ####################################### Global variables ########################################


// ####################################### Global functions ########################################

int32_t	m90e26Identify(uint8_t eDev) ;
int32_t	m90e26Init(uint8_t eDev) ;
void	m90e26Calibrate(uint8_t eChan) ;

void	m90e26DataReadAll(uint8_t eChan) ;
void	m90e26DataConvertAll(uint8_t eChan) ;

int32_t	m90e26SetLiveGain(uint8_t eChan, uint8_t Gain) ;
int32_t	m90e26SetNeutralGain(uint8_t eChan, uint8_t Gain) ;

void	m90e26SetOffsetCompensation(uint8_t eChan) ;

uint16_t m90e26GetSysStatus(uint8_t eChan) ;
uint16_t m90e26GetMeterStatus(uint8_t eChan) ;

struct	ep_work_s ;
int32_t	m90e26ReadEnergy(struct ep_work_s * pEpWork) ;
int32_t	m90e26ReadCurrent(struct ep_work_s * pEpWork) ;
int32_t	m90e26ReadVoltage(struct ep_work_s * pEpWork) ;
int32_t	m90e26ReadPower(struct ep_work_s * pEpWork) ;
int32_t	m90e26ReadFrequency(struct ep_work_s * pEpWork) ;
int32_t	m90e26ReadPowerFactor(struct ep_work_s * pEpWork) ;
int32_t	m90e26ReadPowerAngle(struct ep_work_s * pEpWork) ;

struct rule_s ;
int32_t	m90e26ConfigMode(struct rule_s * psRule) ;
void	m90e26Report(void) ;
void	m90e26Display(void) ;
int32_t	m90e26DisplayContrast(uint8_t Contrast) ;
int32_t	m90e26DisplayState(uint8_t State) ;
