#ifndef _IHART_DATAACQUISITION_H
#define _IHART_DATAACQUISITION_H

#include "IHart_glb_defs.h"
#include "IHart_cfg.h"
#include "uart.h"

/***********************************************
* Tables 2. Unit Codes
************************************************/
/* Temperature */
#define DC							32		// degrees celsius
#define DF							33		// degrees fahrenheit
#define DR							34		// degrees rankine
#define DK							35		// degrees kelvin

/* Pressure */
#define InH2O						1		// inches of water 68 degrees  
#define InHg						2		// inches of mercury 0 degrees C
#define FtH2O						3		// feet of water 68 degrees F
#define mmH2O						4		// millimeters of water 68 degrees F
#define mmHg						5		// millimeters of mercury 0 degrees C
#define psi							6		// pounds per square inch
#define bar							7		// bars
#define mbar						8		// millibars
#define g_SqCm						9		// grams per square centimeter
#define kg_SqCm						10		// kilograms per square centimeter
#define PA							11		// pascals
#define kPA							12		// kilopascals
#define torr						13		// torr
#define ATM							14		// atmospheres
#define MPa							237		// megapascals
#define in_H2O_4_degrees_C			238		// inches of water 4 degrees C
#define mm_H2O_4_degrees_C			239		// millimeters of water 4 degrees C

/* Volumetric Flow */                                        
#define CUBIC_FEET_PER_MINUTE       15      //cubic feet per minute                	                                                  
#define GALLONS_PER_MINUTE          16      //gallons per minute                   	                                                  
#define LITERS_PER_MINUTE           17 		//liters per minute                    	                                                  
#define I_GALLONS_PER_MINUTE        18 		//imperial gallons per minute          	                                                  
#define CUBIC_METER_PER_HOUR        19 		//cubic meter per hour                 	                                                  
#define GALLONS_PER_SECOND          22 		//gallons per second                   	                                                  
#define MILLION_GALLONS_PER_DAY     23 		//million gallons per day              	                                                  
#define LITERS_PER_SECOND           24 		//liters per second                    	                                                  
#define MILLION_LITERS_PER_DAY      25 		//million liters per day               	                                                  
#define CUBIC_FEET_PER_SECOND       26 		//cubic feet per second                	                                                  
#define CUBIC_FEET_PER_DAY          27 		//cubic feet per day                   	                                                  
#define CUBIC_METERS_PER_SECOND     28 		//cubic meters per second              	                                                  
#define CUBIC_METERS_PER_DAY        29 		//cubic meters per day                 	                                                  
#define I_GALLONS_PER_HOUR          30 		//imperial gallons per hour            	                                                  
#define I_GALLONS_PER_DAY           31 		//imperial gallons per day             	                                                  
#define N_CUBIC_METER_PER_HOUR      121 	//normal cubic meter per hour          	 MKS System                                       
#define N_LITER_PER_HOUR            122 	//normal liter per hour                	 MKS System                                       
#define S_CUBIC_FEET_PER_MINUTE     123 	//standard cubic feet per minute       	 U.S. System                                      
#define CUBIC_FEET_PER_HOUR         130 	//cubic feet per hour                  	                                                  
#define CUBIC_METERS_PER_MINUTE     131 	//cubic meters per minute              	                                                  
#define BARRELS_PER_SECOND          132 	//barrels per second                   	 1 barrel equals 42 U.S. gallons                  
#define BARRELS_PER_MINUTE          133 	//barrels per minute                   	 1 barrel equals 42 U.S. gallons                  
#define BARRELS_PER_HOUR            134 	//barrels per hour                     	 1 barrel equals 42 U.S. gallons                  
#define BARRELS_PER_DAY             135 	//barrels per day                      	 1 barrel equals 42 U.S. gallons                  
#define GALLONS_PER_HOUR            136 	//gallons per hour                     	                                                  
#define I_GALLONS_PER_SECOND        137 	//imperial gallons per second          	                                                  
#define LITERS_PER_HOUR             138 	//liters per hour                      	                                                  
#define BEER_BARRELS_PER_SECOND     170 	//beer barrels per second              	 1 beer barrel equals 31 U.S. gallons             
#define BEER_BARRELS_PER_MINUTE     171 	//beer barrels per minute              	 1 beer barrel equals 31 U.S. gallons             
#define BEER_BARRELS_PER_HOUR       172 	//beer barrels per hour                	 1 beer barrel equals 31 U.S. gallons             
#define BEER_BARRELS_PER_DAY        173 	//beer barrels per day                 	 1 beer barrel equals 31 U.S. gallons             
#define N_LITER_PER_DAY             174 	//normal liter per day                 	 at 273.15 degrees Kelvin, 101.325 kilo pascals   
#define N_LITER_PER_MINUTE          175 	//normal liter per minute              	 at 273.15 degrees Kelvin, 101.325 kilo pascals   
#define N_LITER_PER_SECOND          176 	//normal liter per second              	 at 273.15 degrees Kelvin, 101.325 kilo pascals   
#define S_LITER_PER_DAY             177 	//standard liter per day               	 at 20 degrees Celsius, 1 Atmosphere              
#define S_LITER_PER_HOUR            178 	//standard liter per hour              	 at 20 degrees Celsius, 1 Atmosphere              
#define S_LITER_PER_MINUTE          179 	//standard liter per minute            	 at 20 degrees Celsius, 1 Atmosphere              
#define S_LITER_PER_SECOND          180 	//standard liter per second            	 at 20 degrees Celsius, 1 Atmosphere              
#define N_CUBIC_METER_PER_DAY       181 	//normal cubic meter per day           	 at 273.15 degrees Kelvin, 101.325 kilopascals    
#define N_CUBIC_METER_PER_MINUTE    182 	//normal cubic meter per minute        	 at 273.15 degrees Kelvin, 101.325 kilopascals    
#define N_CUBIC_METER_PER_SECOND    183 	//normal cubic meter per second        	 at 273.15 degrees Kelvin, 101.325 kilopascals    
#define S_CUBIC_FEET_PER_DAY        184 	//standard cubic feet per day          	 at 32 degrees Fahrenheit, 1 Atmosphere           
#define S_CUBIC_FEET_PER_HOUR       185 	//standard cubic feet per hour         	 at 32 degrees Fahrenheit, 1 Atmosphere           
#define S_CUBIC_FEET_PER_SECOND     186 	//standard cubic feet per second       	 at 32 degrees Fahrenheit, 1 Atmosphere           
#define S_CUBIC_METER_PER_DAY       187 	//standard cubic meter per day         	 at 20 degrees Celsius, 1 Atmosphere              
#define S_CUBIC_METER_PER_HOUR      188 	//standard cubic meter per hour        	 at 20 degrees Celsius, 1 Atmosphere              
#define S_CUBIC_METER_PER_MINUTE    189 	//standard cubic meter per minute      	 at 20 degrees Celsius, 1 Atmosphere              
#define S_CUBIC_METER_PER_SECOND    190 	//standard cubic meter per second      	 at 20 degrees Celsius, 1 Atmosphere              
#define GALLONS_PER_DAY             235		//gallons per day                      	                                                  

/* Velocity */                                        
#define FTPS                        20          //feet per second    
#define MPS							21          //meters per second  
#define INPS						114         //inches per second  
#define INPM						115         //inches per minute  
#define FTPM						116         //feet per minute    
#define MPH                         120         //meters per hour  

/* Volume */                                        
#define GALLONS                     40      //gallons                                                  
#define LITERS                      41      //liters                                                   
#define IMPERIAL_GALLONS            42      //imperial gallons                                         
#define CUBIC_METERS                43      //cubic meters                                             
#define BARRELS                     46      //barrels 1 barrel equals 42 U.S. gallons                  
#define BUSHELS                     110     //bushels                                                 
#define CUBIC_YARDS                 111     //cubic yards                                             
#define CUBIC_FEET                  112     //cubic feet                                              
#define CUBIC_INCHES                113     //cubic inches                                            
#define BBL_LIQ                     124     //bbl liq 1 liquid barrel equals 31.5 U.S. gallons        
#define N_CUBIC_METER               166     //normal cubic meter MKS System                           
#define N_LITER                     167     //normal liter MKS System                                 
#define S_CUBIC_FEET                168     //standard cubic feet U.S. System                         
#define BEER_BARREL                 170     //beer barrel            31 US gallons                                   
#define S_LITER                     171     //standard liter         at 20 degrees Celsius, 1 Atmosphere             
#define S_CUBIC_METER               172     //standard cubic meter   at 20 degrees Celsius, 1 Atmosphere             
#define HECTOLITERS                 236     //hectoliters                                             

/* Length */
#define Feet						44		// Feet
#define Meters						45		// meters
#define Inches						47		// Inches
#define Centmeters					48		// Centmeters
#define Millimeters					49		// millimeters

/* Mass */                                        
#define GRAMS                       60      //grams        
#define KILOGRAMS                   61      //kilograms    
#define M_TONS                      62      //metric tons  
#define POUNDS                      63      //pounds       
#define S_TONS                      64      //short tons   
#define L_TONS                      65      //long tons    
#define OUNCE                       125     //ounce       

/* Mass Flow */                                        
#define GRPS						70		//grams per second        
#define GRPM						71 		//grams per minute        
#define GRPH						72 		//grams per hour          
#define KIPS						73 		//kilograms per second    
#define KIPM						74 		//kilograms per minute    
#define KIPH						75 		//kilograms per hour      
#define KIPD						76 		//kilograms per day       
#define MTPM						77 		//metric tons per minute  
#define MTPH						78 		//metric tons per hour    
#define MTPD						79 		//metric tons per day     
#define POPS						80 		//pounds per second       
#define POPM						81 		//pounds per minute       
#define POPH						82 		//pounds per hour         
#define POPD						83 		//pounds per day          
#define STPM						84 		//short tons per minute   
#define STPH						85 		//short tons per hour     
#define STPD						86 		//short tons per day      
#define LTPH						87 		//long tons per hour      
#define LTPD						88 		//long tons per day       

/* Mass per Volume */
#define SGU                         90      //specific gravity units                 
#define GPCC                        91      //grams per cubic centimeter         
#define KPCM                        92      //kilograms per cubic meter          
#define PPG                         93      //pounds per gallon                      
#define PPCF                        94      //pounds per cubic foot              
#define GPM                         95      //grams per milliliter                   
#define KPL                         96      //kilograms per liter                    
#define GPL                         97      //grams per liter                        
#define PPCI                        98      //pounds per cubic inch             
#define STPCY                       99      //short tons per cubic yard          
#define DT                          100     //degrees twaddell                    
#define DBH                         102     //degrees baume heavy                    
#define DBL                         103     //degrees baume light                   
#define DA                          104     //degrees API                     
#define MPL                         146     //micrograms per liter                   
#define MPCM                        147     //micrograms per cubic meter         

/* Time */
#define Minutes                     50      //minutes 
#define Seconds                     51      //seconds
#define Hours                       52      //hours  
#define Days                        53      //days      

/* Viscosity */
#define Centistokes                 54      //centistokes  
#define Centipoise                  55      //centipoise  

/* Angular Velocity */
#define Degrees_Per_Second          117     //degrees per second       
#define Revolutions_Per_Second      118     //revolutions per second  
#define Revolutions_Per_Minute      119     //revolutions per minute  

/* Energy (Work) */
#define Newton_Meter                69     //newton meter                                   
#define Deka_Therm                  89     //deka therm                                    
#define Foot_Pound_Force            126    //foot pound force                              
#define Kilo_Watt_Hour              128    //kilo watt hour                                 
#define Mega_Calorie                162    //mega calorie 1 calorie = 4.184 Joules         
#define Mega_Joule                  164    //mega joule                                    
#define British_Thermal_Unit        165    //british thermal unit 1Btu=0.2519958kcal Energy

/* Force */
#define Newton                      68     //newton                                   

/* Power */
#define Kilo_Watt                       127    //kilo watt                                                        
#define Horsepower                      129    //horsepower                                                      
#define Mega_Calorie_Per_Hour           140    //mega calorie per hour 1 calorie = 4.184 Joules                  
#define Mega_Joule_Per_Hour             141    //mega joule per hour                                              
#define British_Thermal_Unit_Per_Hour   142    //british thermal unit per hour 1Btu=0.2519958kcal Energy         

/* Force */
#define Hertz                       38     //hertz                                   

/* Analytical */
#define Percent                         57     //percent                              
#define PH                              59     //pH                                  
#define Percent_Steam_Quality           150    //percent steam quality               
#define Percent_Plato                   160    //percent plato                        
#define Percent_Lower_Explosion_Level   161    //percent lower explosion level       

/* Capacitance */
#define Picofarads                  153     //picofarads                                   

/* EMF */
#define Millivolts                  36      //millivolts                                   
#define Volts                       58      //volts                                        

/* Current */
#define Milliamperes                39      //milliamperes                                   

/* Resistance */
#define Ohms						37      //ohms
#define Kohms						163     //kohms

/* Angle */
#define Degrees                     143     //degrees
#define Radian                      144     //radian 

/* Conductance */
#define Microsiemens                    56      //microsiemens                
#define Milli_Siemens_Per_Centimeter    66      //milli siemens per centimeter
#define Micro_Siemens_Per_Centimeter    67      //micro siemens per centimeter

/* Volume per Volume */
#define Volume_Percent              149     //volume percent       
#define Mililiters_Per_Liter        154     //mililiters per liter 
#define Microliters_Per_Liter       155     //microliters per liter

/* Volume per Mass */
#define Degrees_Balling             107     //degrees balling      
#define Cubic_Feet_Per_Pound        152     //cubic feet per pound 

/* Concentration */
#define Degrees_Brix                101     //degrees brix             
#define Percent_Solids_Per_Weight   105     //percent solids per weight
#define Percent_Solids_Per_Volume   106     //percent solids per volume
#define Proof_Per_Volume            108     //proof per volume         
#define Proof_Per_Mass              109     //proof per mass           
#define Parts_Per_Million           139     //parts per million        
#define Parts_Per_Billion           169     //parts per billion     

/* Miscellaneous */
#define Percent                     57      //percent


/**************************************************
Table 51. Wireless Operation Mode
***************************************************/
#define Wireless_Operate_Idle           0   //Idle
#define Wireless_Operate_Active_Search  1   //Active Search
#define Wireless_Operate_Negotiating    2   //Negotiating
#define Wireless_Operate_Quarantined    3   //Quarantined
#define Wireless_Operate_Operational    4   //Operational
#define Wireless_Operate_Suspended      5   //Suspended
#define Wireless_Operate_Paccive_Search 6   //Passive Search

/**************************************************
Table 3. Transfer Function Codes
***************************************************/
#define FUCTION_LINEAR                  0    // Equation y=mx+b
#define FUCTION_SQRT                    1    // Equation y=sqrt(x)
#define FUCTION_SQRT3                   2    // Equation y=sqrt(x^3)
#define FUCTION_SQRT5                   3    // Equation y=sqrt(x^5)
#define FUCTION_SPECIAL_CURVE           4    // special curve
#define FUCTION_SQUARE                  5    // Equation y=x^2

/**************************************
* Table 6. Alarm Selection Codes
***************************************/
#define ALARM_HIGH                      0           // high
#define ALARM_LOW                       1           // low
#define ALARM_HOLD_LAST_VALUE           239         // hold_last_output_value
#define ALARM_NOT_USED                  250         // not_used
#define ALARM_UNKNOWN                   252         // unknown
#define ALARM_NONE                      251         // none
#define ALARM_SPECIAL                   253         // special

/****************************************
* Tables 7. Write Protect Codes
*****************************************/
#define WRITE_PROTECT_NO                0           // not_write_protected
#define WRITE_PROTECT_YES               1           // write_protected
#define WRITE_PROTECT_NOT_USED          250         // not_used
#define WRITE_PROTECT_NONE              251         // none
#define WRITE_PROTECT_UNKNOWN           252         // unknown
#define WRITE_PROTECT_SPECIAL           253         // special

/***********************************************
* Tables 9. Burst Mode Control Codes
************************************************/
#define BURST_MODE_OFF                  0           // off
#define BURST_MODE_ON                   1           // on
#define BURST_MODE_NOT_USED             250         // not_used
#define BURST_MODE_NONE                 251         // none
#define BURST_MODE_UNKNOWN              252         // unknown
#define BURST_MODE_SPECIAL              253         // special

/***********************************************
* Tables 34. Device Variable Code
************************************************/
#define BATTERY_LIFE                    243        // Battery life (Float in Days) 
#define PERCENT_RANGE                   244        // Percent Range 
#define LOOP_CURRENT                    245        // Loop Current
#define PRIMARY_VARIABLE                246        // Primary Variable
#define SECONDARY_VARIABLE              247        // Secondary Variable
#define TERTIARY_VARIABLE               248        // Tertiary Variable
#define QUATERNARY_VARIABLE             249        // Quaternary Variable

/***********************************************
* The mapping betwen Dynamic and Device variables
************************************************/
enum DYNAMICS {PV, SV, TV, QV};
#define PVVar gsDeviceVars[PV].aucDynamicVar
#define SVVar gsDeviceVars[SV].aucDynamicVar
#define TVVar gsDeviceVars[TV].aucDynamicVar
#define QVVar gsDeviceVars[QV].aucDynamicVar


/***********************************************
* Device Variables information
************************************************/
typedef struct
{
    union MyFloat   unValue;            // Value 
    USIGN8          ucProperty;         // Device Variable Properties    (Spec183-Table65)
    USIGN8          ucForcedUnit;       // Forced Unit
    USIGN8          ucPvRangeUnit;      // PV Range Unit
    USIGN8          ucFunct;            // Transfer Function Code
    USIGN8          ucAlarm;            // Alarm Selection Code
    USIGN8          aucSensorSN[3];     // Sensor SN 
    FLOAT           fDamp;              // Damping Value
    union MyFloat   unUpper;            // Upper Range Value  
    union MyFloat   unLower;            // Lower Range Value
    union MyFloat   unUpperLimit;       // Upper Sensor Limit 
    union MyFloat   unLowerLimit;       // Lower Sensor Limit
    FLOAT           fZero;              // Zero Difference
    FLOAT           fMinSpan;           // Minimum Span
    USIGN8          ucClassification;   // Device Variable Classification(Spec183-Table21)
    USIGN8          ucFamily;           // Device Variable Family        (Spec183-Table20)
    USIGN8          ucStatus;           // Device Variable Status        (Spec183-A.3.Device Variable Status)
    USIGN8          aucDynamicVar;      // The mapping between Device Variables and Dynamic Variables
}DeviceVariable;

typedef struct
{
	USIGN8 aucHpsTag[6];			// Tag 
	USIGN8 aucHpsDes[12];			// Descriptor 
	USIGN8 ucHpsDay;				// Day
	USIGN8 ucHpsMonth;				// Month
	USIGN8 ucHpsYear;				// Year
}TagDesDate;

/***********************************************
* Table44 Device Power Source
************************************************/
typedef enum
{
  LINE_POWER = 0,
  BATTER_POWER = 1,
  RECHARGEABLE_BATTER_POWER = 2
}DEVICE_POWERSOURCE_ENUM;

/***********************************************
* Table58 Device Power Status
************************************************/
typedef enum
{
   Power_Status_Nominal         = 0,
   Power_Status_Low             = 1,
   Power_Status_Critically_Low  = 2,
   Power_Status_Recharging_Low  = 3,
   Power_Status_Recharging_High = 4
}DEVICE_POWER_STATUS_ENUM;


/***********************************************
* Battery Information
************************************************/
typedef struct 
{
    DEVICE_POWERSOURCE_ENUM  PowerSource;  // Power Source(Spec-183 Table44)
    DEVICE_POWER_STATUS_ENUM PowerStatus;  // Power Status(Spec-183 Table58)
    USIGN32                  DurationTime; // Duration at peak packet load before power drain (1/32ms)
    USIGN32                  RecoverTime;  // Time to recover from power drain                (1/32ms)
    USIGN16                  BatteryLife;  // Battery life remaining in days
    FLOAT                    fBatV;        // Battery Value
    USIGN8                   ucForcedUnit;
}SlavePowerInfo;

/***********************************************
* Diagnostic Information
************************************************/
typedef struct 
{
  USIGN8 DiagnosticID;   //Diagnostic id (default:5)
  SIGN8  AvgRSL;         //Connection quality
}SlaveDiagnosticInfo;

/***********************************************
* Slave Device information
************************************************/

typedef struct
{
	USIGN8			    ucPollingAddr;					// Polling Address
	USIGN8			    aucHartMessage[24];				// Message 
	USIGN8			    aucAssemblyNumber[3];			// Final Assembly Number
	TagDesDate	        sHartTagDesDate;				// Tag, Descriptor, Date 
	USIGN8				aucLongTag[32];                 // Long Tag
    USIGN8              ucSenComm38;                    // (defualt)
    USIGN8              ucLock_Divice;                  // (defualt)
	
    USIGN8			    ucNumOfRequestPreambles;		// Num Of Request Preambles 
	USIGN8			    ucDevFlag;						// (defualt) (Spec183-Table11)
	USIGN8			    ucNeedMoreStatus;				// Need More Status Flag
    
    USIGN8              ucPrimaryMaster;             // (defualt)
    USIGN8              ucMasterChange;              // (defualt)
    USIGN8              ucMasterType;                // (defualt)
    USIGN8              ucFirst;                     // (defualt)
    USIGN8              ucFirstPrimaryMaster;        // (defualt)
    
    USIGN8              ucLoopMode;                     // Loop Mode (Spec183-Table16)  
    FLOAT               fPercent;                       // PV Percent
    FLOAT               fCurrent;                       // PV Current
    SlavePowerInfo      gsHartPowerInfo;                // battery Information
    FLOAT               fPV;                            // PV
    FLOAT               fSV;                            // SV
    FLOAT               fTV;                            // TV
    FLOAT               fQV;                            // QV
    USIGN32             ulTime;                         // (defualt)
 
    USIGN8              gucDeviceStatus[25];            // Additional Device Status
    USIGN8              ucWriteProtect;                 // Write Protect Code(1:Enable  0:Disable)
    USIGN8              ucDeviceProfile;                // (default)
    USIGN8              ucAnalogChannelFlags;           // (defualt)
    USIGN8              gucWirelessOperationMode;       // Wireless Operation Mode(Spec-183 Table51)
    SlaveDiagnosticInfo gsDiagnosticInfo;               // (defualt)
    USIGN8              gsUpdateDynamicFlag;            // ���¶�̬������־λ
    USIGN8              ucNoticeResetFlag;              // ֪ͨģ�鸴λ��־λ 0-û�и�λ����  1-֪ͨģ�鸴λ  2-�յ�ģ��Ļظ�
    USIGN8              gsAD_Execute_Flag;              // AD_Execute
}SlaveInfo;

/***********************************************
* Parity information
************************************************/
typedef struct 
{
    USIGN8 ErrState;        // Error state 
    USIGN8 ErrCounter;      // Record the wrong location
}ParityInfo;

/************************************************
* ColdStart-CC��Ϣ�ṹ��
***********************************************/
typedef union
{
    struct
    {
        USIGN8 Reset_ColdStart:      1;
        USIGN8 Reset_ConfigueChange: 1;
    }ColdStart_CC;
    USIGN8 Data;
}ColdStart_CC_UNION;

/************************************************
*  Synchronization information
************************************************/
typedef struct _T_STATUS_DATA
{
    USIGN8  ucLocalDeviceStatusCommon;    //Local�豸״̬�������øı����������
    USIGN8  ucLocalExtendedStatus;        //��չ״̬���ο�Common Table17
    USIGN8  ucLocalStandardizedStatus3;   //(default:��ģ��ά��)
    USIGN8  ucLocalDeviceStatus;          //Local���øı��������״̬
    USIGN16 ucLocalConfigChangedCounter;   //�ı�Counterֵ

    USIGN8  ucRemoteDeviceStatusCommon;   //Remoteģ��״̬
    USIGN8  ucRemoteExtendedStatus;
    USIGN8  ucRemoteStandardizedStatus3;
    USIGN16 ucRemoteConfigChangedCounter;  //Remoteģ��CC_Counter
    ColdStart_CC_UNION ColdStart_CC_Flag;  //CC/coldstart ��־
    USIGN8  ucRemoteStandardizedStatus1;
    USIGN8  ucRemoteStandardizedStatus2;
}T_STATUS_DATA;

/*************************************************
 �ӿڹ��� :�豸��Ϣ��ʼ��
****************************************************/
void IHartUpdateDynamic(void);
void IHartDeviceInit(void);
void IsBatteryAlarm(void);
void IsDevVarsOutofLimit(void);
float GetPerRange(USIGN8 ucIndex);
float GetDevValue(USIGN8 ucIndex);

USIGN8 NewUnitRight(USIGN8 ucClassification, USIGN8 ucUnit);
FLOAT  DevUnitChange(USIGN8 ucClassification, USIGN8 ucSrcUnit, FLOAT fValue, USIGN8 ucDestUnit);

extern void IHartStackInit(void);
extern void IHartUserTime(void);
extern void WriteBuffer(USIGN8 *pucDest, const char *pcStr, USIGN8 ucLength);
extern void SetData(void * pvSrc, USIGN8 ucSize, USIGN8* pucDst, USIGN8 ucIndex);



extern DeviceVariable      gsDeviceVars[VARIABLE_COUNT];    //�豸��������
extern SlaveInfo           gsHartSlaveInfo;                  //�����豸��Ϣ
extern ParityInfo          gsParityInfo;
extern T_STATUS_DATA       gsStatusData; 

#ifdef FUN_MONITOR_RTC
extern USIGN16  MyNewRtc;
#endif

#endif
