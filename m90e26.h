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
 * m90e26.h
 */

/**************************************************************************************************
 *
 *
 *************************************************************************************************/
#pragma		once

#include	<stdint.h>

/* Notes & references:
 * mode option to select WHr or KWHr, set default, variable used as divisor
 *
 * Full duplex, synchronous
 * MSB first
 *
 * https://www.hackster.io/whatnick/atm90e26-and-esp8266-energy-monitoring-cdac92
 * https://imgur.com/a/BsEUM
 * https://www.reddit.com/r/diyelectronics/
 */
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
#define Lgain				0x23 	// L Line Calibration Gain
#define Lphi				0x24 	// L Line Calibration Angle
#define Ngain				0x25 	// N Line Calibration Gain
#define Nphi				0x26 	// N Line Calibration Angle
#define PStartTh			0x27 	// Active Startup Power Threshold
#define PNolTh				0x28 	// Active No-Load Power Threshold
#define QStartTh			0x29 	// Reactive Startup Power Threshold
#define QNolTh				0x2A 	// Reactive No-Load Power Threshold
#define MMode				0x2B 	// Metering Mode Configuration
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
#define P_ANGLE_L			0x4E 	// Phase Angle between Voltage and L Line Current
#define P_APP_L 			0x4F 	// L Line Mean Apparent Power

#define I_RMS_N 			0x68 	// N Line Current rms
#define P_ACT_N 			0x6A 	// N Line Mean Active Power
#define P_REACT_N 			0x6B 	// N Line Mean Reactive Power
#define P_FACTOR_N 			0x6D 	// N Line Power Factor
#define P_ANGLE_N 			0x6E 	// Phase Angle between Voltage and N Line Current
#define P_APP_N 			0x6F 	// N Line Mean Apparent Power

#define	RSTCOD				0x789A
#define	STDCOD				0x5678
#define	CFGCOD				0x8765
#define	PWRCOD				0xA987

// ######################################## Enumerations ###########################################

enum {
#if		(halHAS_M90E26 > 0)
	M90E26_0,
#endif
#if		(halHAS_M90E26 > 1)
	M90E26_1,
#endif
	M90E26_NUM,
} ;

enum {
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
	eNUM_CALIB
} ;

enum {
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
// Neutral Line
 	eI_RMS_N,
 	eP_ACT_N,
 	eP_REACT_N,
 	eP_FACTOR_N,
 	eP_ANGLE_N,
 	eP_APP_N,
	eNUM_DATA_REG
 } ;

// ######################################### Structures ############################################

typedef struct conf_reg_s {
 	uint8_t		addr ;
 	uint8_t		flag ;
 	uint16_t	raw_val ;
 } conf_reg_t ;

#define	MAKE_DATA_REG(x)		{ .addr = x },

typedef struct data_reg_s {
 	float		value[halHAS_M90E26] ;
 	uint16_t	raw_val[halHAS_M90E26] ;
 	uint8_t		addr ;
 } data_reg_t ;

typedef struct m90e26_s {
	float	current[halHAS_M90E26] ;
	float	voltage[halHAS_M90E26] ;
	float	power[halHAS_M90E26] ;
	float	energy[halHAS_M90E26] ;
} m90e26_t ;

// See http://www.catb.org/esr/structure-packing/
// Also http://c0x.coding-guidelines.com/6.7.2.1.html

// ####################################### Global variables ########################################

extern	data_reg_t	m90e26Data[] ;

// ####################################### Global functions ########################################

struct	ep_work_s ;
int32_t	m90e26Init(uint8_t eChan) ;
void	m90e26DataReadAll(uint8_t eChan) ;
void	m90e26DataConvertAll(uint8_t eChan) ;

int32_t	m90e26SetLiveGain(uint8_t eChan, uint8_t Gain) ;
int32_t	m90e26SetNeutralGain(uint8_t eChan, uint8_t Gain) ;
void	m90e26SetPowerOffsetCompensation(void) ;

uint16_t m90e26GetSysStatus(uint8_t eChan) ;
uint16_t m90e26GetMeterStatus(uint8_t eChan) ;

int32_t	m90e26Read_E_ACT_FWD(struct ep_work_s * pEpWork) ;
int32_t	m90e26Read_E_ACT_REV(struct ep_work_s * pEpWork) ;
int32_t	m90e26Read_E_ACT_ABS(struct ep_work_s * pEpWork) ;
int32_t	m90e26Read_E_REACT_FWD(struct ep_work_s * pEpWork) ;
int32_t	m90e26Read_E_REACT_REV(struct ep_work_s * pEpWork)	;
int32_t	m90e26Read_E_REACT_ABS(struct ep_work_s * pEpWork)	;

int32_t	m90e26Read_I_RMS_L(struct ep_work_s * pEpWork) ;
int32_t	m90e26Read_I_RMS_N(struct ep_work_s * pEpWork)	;
int32_t	m90e26Read_V_RMS(struct ep_work_s * pEpWork) ;
int32_t	m90e26Read_FREQ(struct ep_work_s * pEpWork) ;
int32_t	m90e26Read_P_ACT_L(struct ep_work_s * pEpWork) ;
int32_t	m90e26Read_P_REACT_L(struct ep_work_s * pEpWork) ;
int32_t	m90e26Read_P_APP_L(struct ep_work_s * pEpWork)	;
int32_t	m90e26Read_P_ACT_N(struct ep_work_s * pEpWork) ;
int32_t	m90e26Read_P_REACT_N(struct ep_work_s * pEpWork) ;
int32_t	m90e26Read_P_APP_N(struct ep_work_s * pEpWork) ;
int32_t	m90e26Read_P_FACTOR_L(struct ep_work_s * pEpWork) ;
int32_t	m90e26Read_P_FACTOR_N(struct ep_work_s * pEpWork) ;
int32_t	m90e26Read_P_ANGLE_L(struct ep_work_s * pEpWork) ;
int32_t	m90e26Read_P_ANGLE_N(struct ep_work_s * pEpWork) ;

void	m90e26ReportStatus(uint8_t eChan) ;
void	m90e26ReportData(uint8_t eChan) ;
void	m90e26ReportCalib(uint8_t eChan) ;
void	m90e26ReportAdjust(uint8_t eChan) ;
void	m90e26Report(void) ;
void	m90e26Display(void) ;
void	m90e26DisplayInfo(void) ;