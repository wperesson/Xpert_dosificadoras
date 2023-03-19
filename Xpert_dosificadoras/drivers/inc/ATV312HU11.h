/*
 * ATV312HU11.h
 *
 *  Created on: 26 mar. 2021
 *      Author: german
 */
#include "board.h"
#if ATV312
#ifndef DRIVERS_INC_ATV312HU11_H_
#define DRIVERS_INC_ATV312HU11_H_
#define CMD		0x2135		//2:Control word
#define CMI 	0x2138		//3:Extended control word
#define RPR 	0x0c30		//4:Reset counters command
#define LFRD 	0x219a		//5:Speed setpoint
#define LFR 	0x2136		//6:Frequency setpoint
#define PISP 	0x2137		//7:PID regulator setpoint
#define AIV1 	0x14A1		//8:Analog input virtual
#define ETA 	0x219B		//9:Status word
#define HMIS 	0x0CA8		//10:Product status
#define ETI 	0x0C86		//11:Extended status word
#define CRC 	0x20F9		//12:Active reference channel
#define CCC 	0x20FA		//13:Active command channel
#define RFRD 	0x219C		//14:Output velocity
#define RFR 	0x0C82		//15:Estimated motor frequency (signed value)
#define LCR 	0x0C84		//16:Estimated motor current
#define OPR 	0x0C8B		//17:Output power monitoring (100% = nominal motor power)
#define FRH 	0x0C83		//18:Frequency reference before ramp
#define RPC 	0x2ECE		//19:PID reference after ramp
#define RPF 	0x2ECD		//20:PID regulator feedback reference
#define RPE 	0x2ECC		//21:PID regulator discrepancy
#define ULN 	0x0C87		//22:Main voltage (from DC bus)
#define THD 	0x0C89		//23:Drive thermal state
#define THR 	0x259E		//24:Motor thermal state
#define PTH 	0x0CA1		//25:Total drive operating time
#define TAC 	0x0CA3		//26:IGBT alarm time
#define TAC2 	0x0CAA		//27:Time at the minimum frequency
#define FTO 	0x3857		//28:Overload fault duration
#define NPL 	0x1003		//29:Logic input type
#define IL1I 	0x1451		//30:Logic inputs state
#define OL1R 	0x145C		//31:Logic outputs state
#define AI1I 	0x1466		//32:Analogic input state
#define AI1C 	0x147A		//33:Analog input 1 physical value
#define AI1R 	0x1470		//34:Analog input 1 standardized value
#define AO1C 	0x1497		//35:Analog output 1 physical value
#define AO1R 	0x148D		//36:Analog output 1 standardized value
#define LFT 	0x1BD1		//37:Altivar fault code
#define CIC 	0x1BDA		//38:Incorrect configuration
#define DP0 	0x1C20		//39:Fault code on last fault
#define EP0 	0x1C2A		//40:Status word on last fault
#define DP1 	0x1C21		//41:Fault code on fault n-1
#define ULP1 	0x1C67		//42:Supply voltage on fault n-1
#define LCP1 	0x1C49		//43:Motor current on fault n-1
#define RFP1 	0x1C53		//44:Output frequency on fault n-1
#define EP1 	0x1C2B		//45:Status word on fault n-1
#define RTP1 	0x1C5D		//46:Motor operating time on fault n-1
#define OTP1 	0x1CA3		//47:Estimated Motor torque value at fault 1
#define TDP1 	0x1CAD		//48:Measured drive thermal state at fault 1
#define TJP1 	0x1CB7		//49:Estimated power component temperature (Tj) at fault 1
#define SFP1 	0x1CC1		//50:Actual motor switching frequency at fault 1
#define DP2 	0x1C22		//51:Fault code on fault n-2
#define ULP2 	0x1C68		//52:Supply voltage on fault n-2
#define LCP2 	0x1C4A		//53:Motor current on fault n-2
#define RFP2 	0x1C54		//54:Output frequency on fault n-2
#define EP2 	0x1C2C		//55:Status word on fault n-2
#define RTP2 	0x1C5E		//56:Motor operating time on fault n-2
#define OTP2 	0x1CA4		//57:Estimated Motor torque value at fault 2
#define TDP2 	0x1CAE		//58:Measured drive thermal state at fault 2
#define TJP2 	0x1CB8		//59:Estimated power component temperature (Tj) at fault 2
#define SFP2 	0x1CC2		//60:Actual motor switching frequency at fault 2
#define DP3 	0x1C23		//61:Fault code on fault n-3
#define ULP3 	0x1C69		//62:Supply voltage on fault n-3
#define LCP3 	0x1C4B		//63:Motor current on fault n-3
#define RFP3 	0x1C55		//64:Output frequency on fault n-3
#define EP3 	0x1C2D		//65:Status word on fault n-3
#define RTP3 	0x1C5F		//66:Motor operating time on fault n-3
#define OTP3 	0x1CA5		//67:Estimated Motor torque value at fault 3
#define TDP3 	0x1CAF		//68:Measured drive thermal state at fault 3
#define TJP3 	0x1CB9		//69:Estimated power component temperature (Tj) at fault 3
#define SFP3 	0x1CC3		//70:Actual motor switching frequency at fault 3
#define DP4 	0x1C24		//71:Fault code on fault n-4
#define ULP4 	0x1C6A		//72:Supply voltage on fault n-4
#define LCP4 	0x1C4C		//73:Motor current on fault n-4
#define RFP4 	0x1C56		//74:Output frequency on fault n-4
#define EP4 	0x1C2E		//75:Status word on fault n-4
#define RTP4 	0x1C60		//76:Motor operating time on fault n-4
#define OTP4 	0x1CA6		//77:Estimated Motor torque value at fault 4
#define TDP4 	0x1CB0		//78:Measured drive thermal state at fault 4
#define TJP4 	0x1CBA		//79:Estimated power component temperature (Tj) at fault 4
#define SFP4 	0x1CC4		//80:Actual motor switching frequency at fault 4
#define NCV 	0x0BC3		//81:Drive nominal rating
#define VCAL 	0x0BC4		//82:Drive line voltage
#define INV 	0x0BC9		//83:Nominal drive current
#define VDP 	0x0CE6		//84:Drive software version
#define SPN 	0x0CE5		//85:Specific product number
#define NC1 	0x31D9		//86:Communication scanner, value of write word 1
#define NC2 	0x31DA		//87:Communication scanner, value of write word 2
#define NC3 	0x31DB		//88:Communication scanner, value of write word 3
#define NC4 	0x31DC		//89:Communication scanner, value of write word 4
#define NM1 	0x31C5		//90:Communication scanner, value of read word 1
#define NM2 	0x31C6		//91:Communication scanner, value of read word 2
#define NM3 	0x31C7		//92:Communication scanner, value of read word 3
#define NM4 	0x31C8		//93:Communication scanner, value of read word 4
#define SCS 	0x1F41		//94:Save configuration
#define FCS 	0x1F42		//95:Restore configuration
#define CFG 	0x0BEC		//96:Macro configuration
#define LSP 	0x0C21		//97:Low speed
#define HSP 	0x0C20		//98:High speed
#define ITH 	0x2596		//99:Motor thermal current
#define SFC 	0x2391		//100:Speed filter coefficient ( 0(IP) to 1(PI) )
#define CTD 	0x2AF9		//101:Motor current threshold
#define FTD 	0x2AFB		//102:Motor frequency threshold
#define BFR 	0x0BC7		//103:Basic frequency
#define NPR 	0x258D		//104:Nominal Motor Power
#define UNS 	0x2581		//105:Nominal motor voltage
#define NCR 	0x2583		//106:Nominal motor current
#define FRS 	0x2582		//107:Nominal motor frequency
#define NSP 	0x2584		//108:Nominal motor speed
#define COS 	0x2586		//109:Rated motor cos Phi
#define MTM 	0x2590		//110:Motor thermal state memo
#define STUN 	0x2591		//111:Autotune selection store
#define FLG 	0x2594		//112:Frequency loop gain
#define STA 	0x2595		//113:Frequency loop stability
#define TFR 	0x0C1F		//114:Top frequency
#define TUN 	0x2588		//115:Auto-tuning
#define CTT 	0x2587		//116:Motor control type assignment
#define UFR 	0x2597		//117:IR compensation
#define PFL 	0x2598		//118:Flux profile
#define SLP 	0x2599		//119:Slip compensation
#define SPGU 	0x259D		//120:Inertia gain for the derivative term of the UF laws
#define RSM 	0x25A8		//121:Calculated (cold state) or measured stator resistance
#define RSMI 	0x25A9		//122:Measured stator resistance
#define SFT 	0x0C1D		//123:Switching frequency type
#define SFR 	0x0C1E		//124:Switching frequency range
#define CLI 	0x23F1		//125:Current limitation
#define NRD 	0x0C23		//126:Motor noise reduction
#define TCC 	0x2B5D		//127:Type of control
#define TCT	 	0x2B5E		//128:2 wire type control
#define RRS 	0x2B61		//129:Reverse direction
#define AI1T 	0x1132		//130:AI1 type
#define CRL1 	0x1150		//131:AI1 current scaling parameter of 0%
#define CRH 	0x115A		//132:AI1 current scaling parameter of 100%
#define R1 		0x1389		//133:R1 assignment
#define R1S	 	0x1069		//134:R1 status (output active level)
#define LO1 	0x1391		//135:LO1 assignment
#define LO1S 	0x1071		//136:LO1 status (output active level)
#define AO1 	0x139D		//137:AO1 assignment
#define AO1S 	0x11F9		//138:AO1 type
#define FR1 	0x20DD		//139:Reference source 1
#define RIN 	0x0C24		//140:Reverse inhibition
#define PST 	0xFA02		//141:Stop key priority
#define CHCF 	0x20D1		//142:Channel configuration
#define CD1 	0x20E7		//143:Channel 1 command source
#define RPT 	0x232C		//144:Shape ramp assignment
#define ACC 	0x2329		//145:Acceleration time (between 0 and FRS)
#define DEC 	0x232A		//146:Deceleration time (between FRS and 0)
#define RPS 	0x2332		//147:Ramp switch assignment
#define AC2 	0x2334		//148:Acceleration time 2 (between 0 and FRS)
#define DE2 	0x2335 		//149:Deceleration time 2 (between FRS and 0)
#define BRA 	0x232B		//150:Braking function assignment
#define STT 	0x2BC1		//151:Stop mode
#define NST 	0x2BC2 		//152:Free wheel stop assignment
#define FST 	0x2BC4		//153:Fast stop assignment
#define DCF 	0x2BDE		//154:Deceleration ramp time reduction
#define ADC 	0x28A1		//155:Automatic DC injection
#define SDC1 	0x28A3		//156:Current level of automatic DC injection
#define TDC1 	0x28A2		//157:IDC injection time
#define JOG 	0x2B66		//158:Jog assignment
#define JGF 	0x2B67		//159:Jog frequency
#define PS2 	0x2C89		//160:2 preset speeds
#define PS4 	0x2C8A		//161:4 preset speeds
#define PS8 	0x2C8B		//162:8 preset speeds
#define SP2 	0x2C92		//163:Preset speed 2
#define SP3 	0x2C93		//164:Preset speed 3
#define SP4 	0x2C94		//165:Preset speed 4
#define SP5 	0x2C95		//166:Preset speed 5
#define SP6 	0x2C96		//167:Preset speed 6
#define SP7 	0x2C97		//168:Preset speed 7
#define SP8	 	0x2C98		//169:Preset speed 8
#define JPF 	0x2C25		//170:Skip frequency
#define PIF 	0x2E7D		//171:PID : PI function feedback assignment
#define TLS 	0x2DB5		//172:Time limited speed (LSP)
#define FBS 	0x2E7F		//173:PID Feedback scale factor
#define PII 	0x2E84		//174:PID : PI internal reference selection
#define RPI 	0x2E90		//175:PID : Internal reference PI
#define RPL 	0x2E87		//176:PID minimum value reference
#define RPH 	0x2E88		//177:PID max value reference
#define RPG 	0x2EA5		//178:PID : PI regulator proportional gain
#define RIG 	0x2EA6		//179:PID : PI regulator integral gain
#define RDG 	0x2EA7		//180:PID : PI regulator derivative gain
#define PRP 	0x2ED0		//181:PID : ACC/DEC setpoint ramp parameter
#define PIC 	0x2EA4		//182:PID : PI regulator Reversal direction correction
#define PAU 	0x2EC2		//183:PID : Auto-manu
#define PIM 	0x2EB2		//184:PID : Reference input in manual mode
#define SFS 	0x2EB3		//185:PID predictive speed
#define RSL 	0x2EB8		//186:PID : Wake up threshold on PI error
#define UPP 	0x2EBC		//187:Wake up threshold
#define LFF 	0x1BA8		//188:Withdrawal frequency
#define LPI 	0x2EBD		//189:Reaction threshold for PI monitoring function
#define MPI 	0x2EBF		//190:Behaviour of PI monitoring function
#define NFD 	0x2ED6		//191:Period for PI state point checking
#define PR2 	0x2E85		//192:PID : Assignment for 2 presets PI
#define PR4 	0x2E86		//193:PID : Assignment for 4 presets PI
#define RP2 	0x2E91		//194:PID : Preset PI number 2
#define RP3 	0x2E92 		//195:PID : Preset PI number 3
#define RP4 	0x2E93		//196:PID : Preset PI number 4
#define APO 	0x-			//197:???
#define FFD 	0x2ED7		//198:Max speed for PI state point checking
#define FOF 	0x3B63		//199:Stopping frequency of the auxiliary pump
#define FON 	0x3B62		//200:Starting frequency of the auxiliary pump
#define FTU 	0x384D		//201:Underload fault duration
#define LFD 	0x2ED8		//202:Speed reference for PI state point checking
#define MDE 	0x3B61		//203:Selecting the operating mode
#define ROF 	0x3B67		//204:Ramp for stopping the auxiliary pump
#define RON 	0x3B66		//205:Ramp for reaching the nominal speed of the auxiliary pump
#define SLE 	0x2DB6		//206:Stop on LSP hysteresis
#define TOF 	0x3B65		//207:Time delay before the auxiliary pump stop command
#define TON 	0x3B64		//208:Time delay before starting the auxiliary pump
#define TPI_ATV 	0x2EBE 		//209:Reaction time for PI monitoring function
#define INH 	0x1BD5		//210:Fault inhibition assignment
#define SLL 	0x1B62		//211:Drive behaviour when ETIx.SLFEvent is detected on Modbus channel
#define LFL1 	0x1B69		//212:4-20 mA loos behaviour
#define NCA1 	0x31B1		//213:Communication scanner, address of write word 1
#define NCA2 	0x31B2		//214:Communication scanner, address of write word 2
#define NCA3 	0x31B3		//215:Communication scanner, address of write word 3
#define NCA4 	0x31B4		//216:Communication scanner, address of write word 4
#define NMA1 	0x319D		//217:Communication scanner, address of read word 1
#define NMA2 	0x319E		//218:Communication scanner, address of read word 2
#define NMA3 	0x319F		//219:Communication scanner, address of read word 3
#define NMA4 	0x31A0		//220:Communication scanner, address of read word 4
#define ADD 	0x1771		//221:Terminal modbus : Drive address
#define TBR 	0x1773		//222:Terminal modbus : Baud-rate
#define TFO 	0x1774		//223:Terminal modbus : Frame format
#define TTO 	0x1775		//224:Terminal modbus : Time-out
#define FLO 	0x20EF		//225:Forced local assignment
#define FLOC 	0x20F0		//226:Forced local reference source assignment
#define RTHI	0x0CA0		//227:Run elapsed time display
#define PET 	0x0CA4		//228:Process elapsed time
#define FTH 	0x0CA7		//229:Fan time display
#define ULT 	0x384B		//230:Application underload time delay
#define LUL 	0x384F		//231:Application underload threshold
#define TOL 	0x3855		//232:Application Overload time delay
#define LOC 	0x3859		//233:Application overload threshold
#define SH2 	0x3AFD		//234:2 HSP assignment
#define SH4	 	0x3AFE		//235:4 HSP assignment
#define HSU 	0x3B05		//236:Display of High speed value
#define HSP2 	0x3B06		//237:High speed 2
#define HSP3 	0x3B07		//238:High speed 3
#define HSP4 	0x3B08		//239:High speed 4
#define ETF 	0x1BDB		//240:External fault assignment
#define EPL 	0x1B5E		//241:Drive behaviour on external fault detection

typedef enum {
	nOF=0,
	InF,
	CFF=3,
	CFI,
	SLF1,
	EPF1=8,
	OCF,
	CrF1,
	OHF=16,
	OLF,
	ObF,
	OSF,
	OPF1,
	PHF,
	USF,
	SCF1,
	SOF,
	tnF,
	InF1,
	InF2,
	InF3,
	InF4,
	SCF3=32,
	OPF2,
	SLF2=42,
	SLF3=45,
	InF9=51,
	InFb=53,
	tJF,
	SCF4,
	SCF5,
	InFE=69,
	CFI2=77,
	ULF=100,
	OLC,
	SPIF,
	LFF1=106,
	XXXX=253,
}TyATHfault;

typedef struct{
	uint16_t RPMset;//indica el setpoit RMP
	uint16_t RPMOut;//indica las RPM de salida del variador
	uint8_t SLID;// indica el esclavo
	uint16_t LastAddTX; //Guarda la ultima direccion enviada
	bool Rx_OK;//Indicador de que la utima encuesta fue exitosa
	uint8_t LastOper;
	TyATHfault FaultCode;
	uint8_t Status;

}Typ_VSP;
//extern Typ_VSP VSD;
#endif
#define FAULT_MASK 0x8
#endif /* DRIVERS_INC_ATV312HU11_H_ */
