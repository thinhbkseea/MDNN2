
/*
 * global.h
 *
 *  Created on: Nov 30, 2022
 *      Author: thinh
 */


#ifndef global_H_
#define global_H_

//#include "SPI_API.h"
#include "Sensorless_Trap_Parameters_Setup.h"
#include <Init.h>
//#include "mdbu_global.h"
#include "dataTypeDefinition.h"
#include "msp430.h"
//#include "mdbuserial.h"
//#include "mdbuserial_protocol.h"
//#include "mdbuserial.h"
//#include "usb.h"
unsigned int close_loop_cycle;

#define DRV832X_REGISTER_WIDTH (2) // Defines the No of bytes used in 1 SPI trasaction , because we use 16 bit SPI register width is 2
/*
 * The system parameters are listed below
 */

#define PWM_PERIOD                              1000    // PWM Period time = 24Mhz/[this number] {1000 = 41.67us}
#define COUNTER_1_MSECOND                       0x18    //25*40.96us = 1ms
#define COUNTER_10_MSECONDS                     240     //250 * 40.96us = 10ms
#define COUNTER_100_MSECONDS                    2400    //2500 * 40.96us = 100ms
#define COUNTER_200_MSECONDS                    4800    //5000 * 40.96us = 200ms
#define COUNTER_1_SECOND                        24000   //24414 * 40.96us = 2s
#define COUNTER_2_SECONDS                       48000   //50000 * 40.96us = 2s
#define ACCEL_30_DEGREES                        10417
#define ACCEL_60_DEGREES                        20834

void EnableGateDrivers(void);
void DisableGateDrivers(void);
void sensorlessTrapController_Init(void);
void SetPWMDutyCycle(uint16_t PWMDutyCycle);
void PWM_SetCommutation(uint8_t commState);
void IPD_SetState(uint8_t commState);
void UpdateNextCommutation(void);
void SetMotorSpeed(void);
void UpdateBEMFADC(void);
void FastReadBEMF(void);
void ReadVCC(void);
void ReadCurrentShunt(void);
void ISCReadPhaseVoltage(void);
void HostControllerInit(void);
void ReadSPDFDBK(void);
void BrakeMotor(void);
/* Host Controller Specifics*/

//// GPIO Port 1 Definitions
//#define EN_DRV      BIT6    // P1.6
//// GPIO Port 2 Definitions
//#define nFAULT      BIT7    // P2.7
void drv83xx_regRestoreFromCache();
void drv832x_setGPIO(unsigned char gpioPort, unsigned char gpioNum, unsigned char gpioVal);
unsigned char drv832x_getGPIO(unsigned char gpioPort, unsigned char gpioNum);
void drv832x_setMtrParam(unsigned char num, unsigned long value);
unsigned long drv832x_getMtrParam(unsigned char num);
void drv83xx_StartMotor();
void drv83xx_StopMotor();
unsigned int drv832x_registerRead(unsigned char address);
void drv832x_registerWrite(unsigned char address, unsigned int value);
void drv832x_setCtrlTypeParam(unsigned char value);
unsigned char drv832x_getCtrlTypeParam(void);
typedef enum
{
    SYSTEM_INIT = 0,
    SYSTEM_IDLE =1,
    MOTOR_IDLE = 2,
    MOTOR_ISC = 3,
    MOTOR_ALIGN = 4,
    MOTOR_IPD = 5,
    MOTOR_START = 6,
    MOTOR_RUN = 7,
    MOTOR_STOP = 8,
    FAULT = 9,
    MOTOR_DIRECTION = 10
}STATE_MACHINE;

typedef enum
{
    HOST_EN = 0,
    HOST_IDLE = 1,
    HOST_ACTIVE = 2
} HOSTCONTROL_STATUS;

typedef enum
{
    START = 0,
    TIMER_INTERRUPT = 1,
    ADC_READ = 2,
    BRAKE = 3,
    DONE = 4
}IPD_STATE;

typedef enum
{
    READ_BEMF = 0,
    BRAKE_MOTOR = 1,
    RUN_MOTOR = 2,
    RUN_IPD = 3
}ISC_STATE;

typedef enum
{
    NOFAULT = 0,
    VOLTAGE = 1,
    OVERCURRENT = 2,
    OVERTEMPERATURE = 3,
    MOTOR_STALL = 4,
    GATE_DRIVER = 5,
    UNKNOWN = 6

}FAULTS;

typedef struct APPLICATION_STATUS
{
    STATE_MACHINE currentstate;
    STATE_MACHINE previousstate;
    FAULTS fault;
} APPLICATION_STATUS;

// Host Controller
typedef struct HOST_CONTROLLER_Obj
{
    uint8_t EnabledGateDrivers;
    uint8_t Start_Stop_Motor;        //If Motor is in Stop_Mode = 1, If Motor is in Start_mode = 0

} HOST_CONTROLLER_Obj;

typedef struct SENSORLESS_TRAP_Obj
{
    //Initial Speed Control Variables
    uint16_t ComparatorNext;
    uint16_t ComparatorState;
    uint8_t ISCbrake;
    uint16_t ISCcount;
    uint8_t ISCdone;
    uint16_t ISCfirst;
    ISC_STATE ISCStatus;

    //IPD Variables
    uint16_t IPDCoastTime;
    uint16_t IPDCount;
    uint16_t IPDCurrent;
    BOOL IPDDone;
    uint16_t IPDMaxCRV;
    SINT32 IPDCurrentRiseValue[7];
    BOOL IPDStart;
    uint16_t IPDState;
    IPD_STATE IPDStatus;

    //Align Variables
    BOOL AlignComplete;
    uint16_t AlignWaitCounter;
    BOOL StartAlign;

    //Open Loop Acceleration Variables
    uint16_t AccelCounter;
    uint16_t AccelDistance;
    BOOL AccelDone;
    uint16_t AccelVelocityInit;    /* This variable holds the initial value of open loop acceleration */
    uint16_t Counter_1M_Second;    /* This variable holds the counter value for 1 milli second based on PWM frequency */
    uint16_t Counter_1_Second;    /* This variable holds the counter value for 1  second based on PWM frequency */



    //Closed Loop Variables
    BOOL ADCchange;
    uint16_t ADCcnt;
    uint16_t ADCdelay;
    BOOL ADCready;
    BOOL ADCswitch;
    uint16_t BEMFtrigger;
    uint8_t CommStateDetect;
    uint16_t CTvoltage;
    uint16_t GetBEMF;
    BOOL SpeedChange;
    uint16_t SpeedDivider;
    uint16_t SumBEMF;
    uint16_t PhaseA_BEMF;
    uint16_t PhaseB_BEMF;
    uint16_t PhaseC_BEMF;

    //System Variables
    uint8_t CurrentCommState;
    uint16_t CurrentDutyCycle;
    BOOL Direction;
    BOOL Direction_flag;
    uint16_t faultreg;
    uint16_t OClimit;
    uint16_t SystemCount;
    uint16_t TargetDutyCycle;
    uint16_t VCCvoltage;
    uint16_t MotorPhaseCurrent;
    uint16_t RestartDelay;
    uint16_t RestartDelayCounter;
    uint16_t RotationCount;
    uint16_t StallDetectCounter;
    uint16_t StallDetectDelay;
    BOOL TimerOverflowFlag;  /* Interrupt counter to ensure the speed measurement with 18bit resolution*/

    //MDBU Serial Variables
    uint16_t SPDFdbk;          /* This variable holds the Motor parameter (0) Motor Spin frequency in timer counts with 18bit resolution */
    uint16_t DeviceID;         /* This variable holds the Motor parameter (2) Device ID to set the appropriate motor control page for a connected device */
    uint16_t ISCMinBEMF;       /* This variable holds the Motor parameter (3) ISC_MIN_BEMF to set the min BEMF value above which ISC routine is executed */
    uint16_t ISCBrakeTime;     /* This variable holds the Motor parameter (4) ISC brake time to set the amount of time to brake the motor during ISC routine */
    uint16_t IPDBrakeTime;     /* This variable holds the Motor parameter (5) IPD brake time to set the amount of time to brake before applying another pulse during IPD */
    uint16_t IPDPulseTime;     /* This variable holds the Motor parameter (6) IPD Pulse time to set the amount of time current pulse is given during IPD */
    uint16_t IPDDecayConstant; /* This variable holds the Motor parameter (7) IPD Decay Constant time to set the amount of time current pulse is allowed to decay during IPD */
    uint16_t AlignSector;      /* This variable holds the Motor parameter (8) Align secor to set the commutation state to be aligned at the motor start up */
    uint16_t AlignWaitTime;    /* This variable holds the Motor parameter (9) Align wait time to set the amount of time for which voltage pulses are given during Aligning the rotor at start up */
    uint16_t AccelRate;        /* This variable holds the Motor parameter (10) Accel rate defines the Open loop Blind  acceleration rate */
    uint16_t AccelStop;        /* This variable holds the Motor parameter (11) Accel stop defines the open loop to closed loop hand off velocity */
    uint16_t AccelVelocity;    /* This variable holds the Motor parameter (11) Accel stop defines the open loop initial velocity */
    uint16_t BEMFThreshold;    /* This variable holds the Motor parameter (13) BEMF_threshold to set the BEMF integration threshold value */
    uint16_t RampRateDelay;    /* This variable holds the Motor parameter (14) Ramp rate delay to set the acceleration/ desceleration of the motor */
    uint16_t SetMotorSpeed;    /* This Variable holds the Motor parameter (31) speed input from the GUI through MDBU serial */
    uint16_t SetSpeed;           //speed input from button

    uint16_t CommutationBlankTime; /* This Variable holds the Motor parameter (16) to set the number of PWM cycles to before which BEMF is sampled */
    uint16_t PWMBlankCounts;   /* This Variable holds the Motor parameter (17) to set the number of PWM cycles to after which BEMF is sampled after a commutation is taken place */
    uint16_t MaxDutyCycle;     /* This Variable holds the Motor parameter (18) to set the PWM Maximum duty cycle */
    uint16_t MinOffDutyCycle;  /* This Variable holds the Motor parameter (19) to set the PWM Minimum off duty cycle to spin the motor */
    uint16_t MinOnDutyCycle;   /* This Variable holds the Motor parameter (20) to set the PWM Minimum on duty cycle to start the motor */
    uint16_t StartupDutyCycle; /* This Variable holds the Motor parameter (21) to set the PWM duty cycle at strtup*/
    uint16_t PWMPeriod;          /* This Variable holds the Motor parameter (22) to set the PWM switching frequency*/
    uint16_t UnderVolLim;      /* This Variable holds the Motor parameter (23) to set the supply undervoltage limit */
    uint16_t OverVolLim;       /* This Variable holds the Motor parameter  (24) to set the supply overvoltage limit */
    uint16_t StallDetectRev;    /* This Variable holds the Motor parameter  (25) to set the minimum revolutions to detect for a stall fault */
    uint16_t StallDetecttime;    /* This Variable holds the Motor parameter  (26) to set the time after which stall fault is triggered */
    uint16_t MotorPhaseCurrentLimit; /* This Variable holds the Motor parameter (27) to set the motor Phase current limit */
    uint16_t AutoFaultRecoveryTime; /* This Variable holds the Motor parameter (28) to set the time limit after which faults are automatically recovered */
    uint16_t Align_IPD;           /* This Variable holds the Motor parameter (29) to set the rotor or to find the current position of rotor */
    uint16_t PWM_Mode;            /* This Variable holds the Ctrl type : 0 for 6PWM mode , 1 for 1 PWM mode*/
} SENSORLESS_TRAP_Obj;

typedef struct FLT_STAT_REG0_Obj
{
    uint8_t REG0_FAULT;      // bit 10
    uint8_t REG0_VDS_OCP;    // bit 9
    uint8_t REG0_GDF;        // bit 8
    uint8_t REG0_UVLO;       // bit 7
    uint8_t REG0_OTSD;       // bit 6
    uint8_t REG0_VDS_HA;     // bit 5
    uint8_t REG0_VDS_LA;     // bit 4
    uint8_t REG0_VDS_HB;     // bit 3
    uint8_t REG0_VDS_LB;     // bit 2
    uint8_t REG0_VDS_HC;     // bit 1
    uint8_t REG0_VDS_LC;     // bit 0
} FLT_STAT_REG0_Obj;

typedef struct VGS_STAT_REG1_Obj
{
    uint8_t REG1_SA_OC;      // bit 10
    uint8_t REG1_SB_OC;      // bit 9
    uint8_t REG1_SC_OC;      // bit 8
    uint8_t REG1_OTW;        // bit 7
    uint8_t REG1_CPUV;       // bit 6
    uint8_t REG1_VGS_HA;     // bit 5
    uint8_t REG1_VGS_LA;     // bit 4
    uint8_t REG1_VGS_HB;     // bit 3
    uint8_t REG1_VGS_LB;     // bit 2
    uint8_t REG1_VGS_HC;     // bit 1
    uint8_t REG1_VGS_LC;     // bit 0
} VGS_STAT_REG1_Obj;

typedef struct DRV_CTRL_REG2_Obj
{
    uint8_t REG2_OCP_ACT;      // bit 10
    uint8_t REG2_DIS_CPUV;  // bit 9
    uint8_t REG2_DIS_GDF;   // bit 8
    uint8_t REG2_OTW_REP;   // bit 7
    uint8_t REG2_PWM_MODE;  // bit 6:5
    uint8_t REG2_PWM_COM;  // bit 4
    uint8_t REG2_PWM_DIR;  // bit 3
    uint8_t REG2_COAST;     // bit 2
    uint8_t REG2_BRAKE;     // bit 1
    uint8_t REG2_CLR_FLT;   // bit 0
} DRV_CTRL_REG2_Obj;

typedef struct GATE_DRV_HS_REG3_Obj
{
    uint8_t REG3_LOCK;          // bit 10:8
    uint8_t REG3_IDRIVEP_HS;    // bit 7:4
    uint8_t REG3_IDRIVEN_HS;    // bit 3:0
} GATE_DRV_HS_REG3_Obj;

typedef struct GATE_DRV_LS_REG4_Obj
{
    uint8_t REG4_CBC;           // bit 10
    uint8_t REG4_TDRIVE;        // bit 9:8
    uint8_t REG4_IDRIVEP_LS;    // bit 7:4
    uint8_t REG4_IDRIVEN_LS;    // bit 3:0
} GATE_DRV_LS_REG4_Obj;

typedef struct OCP_CTRL_REG5_Obj
{
    uint8_t REG5_TRETRY;        // bit 10
    uint8_t REG5_DEAD_TIME;     // bit 9:8
    uint8_t REG5_OCP_MODE;      // bit 7:6
    uint8_t REG5_OCP_DEG;       // bit 5:4
    uint8_t REG5_VDS_LVL;     // bit 3:0
} OCP_CTRL_REG5_Obj;

typedef struct CSA_CTRL_REG6_Obj
{
    uint8_t REG6_CSA_FET;       // bit 10
    uint8_t REG6_VREF_DIV;      // bit 9
    uint8_t REG6_LS_REF;        // bit 8
    uint8_t REG6_CSA_GAIN;      // bit 7:6
    uint8_t REG6_DIS_SEN;       // bit 5
    uint8_t REG6_CSA_CAL_A;     // bit 4
    uint8_t REG6_CSA_CAL_B;     // bit 3
    uint8_t REG6_CSA_CAL_C;     // bit 2
    uint8_t REG6_SEN_LVL;       // bit 1:0
} CSA_CTRL_REG6_Obj;

typedef struct REG_MAP_Obj
{
    uint16_t Fault_Status_Reg0;
    uint16_t VGS_Status_Reg1;
    uint16_t Driver_Control_Reg2;
    uint16_t Gate_Drive_HS_Reg3;
    uint16_t Gate_Drive_LS_Reg4;
    uint16_t OCP_Control_Reg5;
    uint16_t CSA_Control_Reg6;
} REG_MAP_Obj;

#endif
