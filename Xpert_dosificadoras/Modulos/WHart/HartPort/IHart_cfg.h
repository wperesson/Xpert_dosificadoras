#ifndef  _IHART_CFG_H
#define  _IHART_CFG_H

#include "IHart_glb_defs.h"


/***********************************************
* Tables 20. Device Variable Family Codes
************************************************/
//0-3  Reserved. Must Not be Used  
#define FAMILY_TEMPERATURE    4   // Temperature 
#define FAMILY_PRESSURE       5   // Pressure 
#define FAMILY_ACTUATOR       6   // Valve / Actuator 
#define FAMILY_S_PID_CTRL     7   // Simple PID Control 
#define FAMILY_PH             8   // pH 
#define FAMILY_CONDUCTIVITY   9   // Conductivity 
#define FAMILY_TOTALIZER      10  // Totalizer 
#define FAMILY_LEVEL          11  // Level 
#define FAMILY_VORTEX_FLOW    12  // Vortex Flow 
#define FAMILY_MAG_FLOW       13  // Mag Flow 
#define FAMILY_CORIOLIS_FLOW  14  // Coriolis Flow 
//132-249 Reserved. Must Not be Used  
#define FAMILY_NOT_USED       250 // Not Used 


/***********************************************
* Tables 21. Device Variable Classification Codes
************************************************/
#define CLASSIFICATION_NOT                  0  //Device Variable Not Classified
#define CLASSIFICATION_RESERVED             1  //1-63 //Reserved
#define CLASSIFICATION_TEMPERATURE          64 //Temperature
#define CLASSIFICATION_PRESSURE             65 //Pressure
#define CLASSIFICATION_VOLUMETRIC_FLOW      66 //Volumetric Flow
#define CLASSIFICATION_VELOCITY             67 //Velocity
#define CLASSIFICATION_VOLUME               68 //Volume
#define CLASSIFICATION_LENGTH               69 //Length
#define CLASSIFICATION_TIME                 70 //Time
#define CLASSIFICATION_MASS                 71 //Mass
#define CLASSIFICATION_MASS_FLOW            72 //Mass Flow
#define CLASSIFICATION_MASS_PER_VOLUME      73 //Mass Per Volume
#define CLASSIFICATION_VISCOSITY            74 //Viscosity
#define CLASSIFICATION_ANGULAR_VELOCITY     75 //Angular Velocity
#define CLASSIFICATION_AREA                 76 //Area
#define CLASSIFICATION_ENERGY               77 //Energy (Work)
#define CLASSIFICATION_FORCE                78 //Force
#define CLASSIFICATION_POWER                79 //Power
#define CLASSIFICATION_FREQUENCY            80 //Frequency
#define CLASSIFICATION_ANALYTICAL           81 //Analytical
#define CLASSIFICATION_CAPACITANCE          82 //Capacitance
#define CLASSIFICATION_EMF                  83 //Emf
#define CLASSIFICATION_CURRENT              84 //Current
#define CLASSIFICATION_RESISTANCE           85 //Resistance
#define CLASSIFICATION_ANGLE                86 //Angle
#define CLASSIFICATION_CONDUCTANCE          87 //Conductance
#define CLASSIFICATION_VOLUME_PER_VOLUME    88 //Volume Per Volume
#define CLASSIFICATION_VOLUME_PER_MASS      89 //Volume Per Mass
#define CLASSIFICATION_CONCENTRATION        90 //Concentration
#define CLASSIFICATION_VALVE_ACTUATOR       91 //Valve Actuator
#define CLASSIFICATION_LEVEL                92 //Level
#define CLASSIFICATION_VORTEX_FLOW          93 //Vortex Flow
#define CLASSIFICATION_MAG_FLOW             94 //Mag Flow
#define CLASSIFICATION_CORIOLIS             95 //Coriolis Flow
#define CLASSIFICATION_NONE                 250 //NONE



#define HART_MOD_TIMEOUT    20 /*1200   2���ַ�ʱ�� < 20ms*/
#define M1100_TIMEOUT       8//4  /*19200  2���ַ�ʱ�� < 2ms */


//�û��Զ�������ṹ��
typedef struct
{
    USIGN16 commandNum;
    USIGN16 (*Function)(USIGN8 *pReqData, USIGN8 nReqLength, USIGN8 *pRspData, USIGN8 *nRspLenth);
}UserDefineCommand;



extern const USIGN8 IUserRevision[4];
//extern const USIGN8 IUserLongAddr[7];

extern USIGN8 IUserLongAddr[7];

//extern const char IUserHpsTag[];
extern char IUserHpsTag[];

extern const char IUserHpsDes[];
extern const char IUserMessage[];
extern const USIGN8 IUserAssemblyNumber[3];
extern const USIGN8 IUserDate[3];

extern const USIGN8 IUserDynamicsCount;
extern const USIGN8 IUserVariableCount;

extern const USIGN8 IUserSTOTime;
extern const USIGN8 IUserRTSLowTimer;
extern const USIGN8 IUserVariableUpdateRate;
extern const USIGN8 IUserFactorySet;
extern const USIGN16 IUserHartDataSaveStartAddr;

extern const USIGN8 gucDefaultUnit[VARIABLE_COUNT];
extern const USIGN8 gucDefaultClassification[VARIABLE_COUNT];
extern const USIGN8 IUserDefaultFamily[VARIABLE_COUNT];
extern const FLOAT IUserDefaultUpperLimit[VARIABLE_COUNT];
extern const FLOAT IUserDefaultLowerLimit[VARIABLE_COUNT];


extern const UserDefineCommand IHartDefineCommands[];
extern const UserDefineCommand IUserDefineCommands[];
extern const USIGN16 IHartCommandCount;
extern const USIGN16 IUserCommandCount;


#endif
