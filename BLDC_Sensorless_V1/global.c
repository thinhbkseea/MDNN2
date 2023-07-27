/*
 * global.c
 *
 *  Created on: Nov 30, 2022
 *      Author: thinh
 */


#include "global.h"
// Motor Parameters
#define MTR_PARAM_ELEC_SPEED 0
#define MTR_PARAM_FAULT_STATE 1
#define MTR_PARAM_DEVICE_ID 2
#define MTR_PARAM_ISC_MIN_BEMF 3
#define MTR_PARAM_ISC_BRAKE_TIME 4
#define MTR_PARAM_IPD_BRAKE_TIME 5
#define MTR_PARAM_IPD_PULSE_TIME 6
#define MTR_PARAM_IPD_DECAY_CONSTANT 7
#define MTR_PARAM_ALIGN_SECTOR 8
#define MTR_PARAM_ALIGN_WAIT_TIME 9
#define MTR_PARAM_ACCEL_RATE 10
#define MTR_PARAM_ACCEL_STOP 11
#define MTR_PARAM_ACCEL_VEL_INIT 12
#define MTR_PARAM_BEMF_THRESHOLD 13
#define MTR_PARAM_RAMP_RATE_DELAY 14
#define MTR_PARAM_DIR_REV_DELAY 15
#define MTR_PARAM_COMM_BLANK_TIME 16
#define MTR_PARAM_PWM_BLANK_COUNTS 17
#define MTR_PARAM_MAX_DUTY_CYCLE 18
#define MTR_PARAM_MIN_OFF_DUTY_CYCLE 19
#define MTR_PARAM_MIN_ON_DUTY_CYCLE 20
#define MTR_PARAM_START_UP_DUTY_CYCLE 21
#define MTR_PARAM_PWM_FREQ 22
#define MTR_PARAM_UNDER_VOL_LIM 23
#define MTR_PARAM_OVER_VOL_LIM 24
#define MTR_PARAM_STALL_DETECT_REV 25
#define MTR_PARAM_STALL_DETECT_TIME 26
#define MTR_PARAM_MOTOR_PHASE_CURR_LIM 27
#define MTR_PARAM_AUTO_FAULT_RECOVERY_TIME 28
#define MTR_PARAM_ALIGN_IPD 29
#define MTR_PARAM_DIR  30
#define MTR_PARAM_SPEED 31
#define MTR_START_STOP_MOTOR 32


// Controller
extern SENSORLESS_TRAP_Obj SensorlessTrapController;
extern APPLICATION_STATUS ApplicationStatus;
extern HOSTCONTROL_STATUS HostControl_Status;
//extern mdbuSerial_RxPacket mdbuSerial_RxPkt;
extern HOST_CONTROLLER_Obj HostController;
// Registers
extern FLT_STAT_REG0_Obj Fault_Status_Reg;
extern VGS_STAT_REG1_Obj VGS_Status_Reg;
extern DRV_CTRL_REG2_Obj Driver_Control_Reg;
extern GATE_DRV_HS_REG3_Obj Gate_Drive_HS_Reg;
extern GATE_DRV_LS_REG4_Obj Gate_Drive_LS_Reg;
extern OCP_CTRL_REG5_Obj OCP_Control_Reg;
extern CSA_CTRL_REG6_Obj CSA_Control_Reg;
extern REG_MAP_Obj Reg_Map_Cache;

unsigned int count_i = 0;
unsigned int sum_current = 0;

unsigned int count_j = 0;
unsigned int sum_vol = 0;
unsigned int vol_array[8];
unsigned int vol_pre = 0;
unsigned int count_m = 0;

void EnableGateDrivers()
{
    /* Gate Drive Enable using Port 1.6 */
    P2OUT |= BIT0;          // Enable Gate Drivers
}
/* switch on the low side switches of all three phases */
void BrakeMotor()
{
    if(SensorlessTrapController.PWM_Mode == 0)  // If six PWM mode
    {
        P6SEL0 &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5);

        P6OUT &=  ~(BIT0 | BIT2 | BIT4);                                     // Turn High Side U,V,W phase Off
        P6OUT |= (BIT1 | BIT3 | BIT5);                                     // Turn Low Side U,V,W phase On
    }
    else
    {
        P6OUT &= ~BIT5  ; //  Make INLC as GPIO low to apply the brakes
    }
}
void DisableGateDrivers()
{
    if(SensorlessTrapController.PWM_Mode == 0)  // If six PWM mode
    {
        P6OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5);
        P6SEL0 &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5);
    }
    else
    {
        P6SEL0 &= ~(BIT1 | BIT2 | BIT3 | BIT4 | BIT5);

        P6OUT &= ~(BIT1 | BIT2 | BIT3);
        P6OUT |= BIT5  ; //  Make INLC as GPIO low to remove the brakes

    }
}

/*function
 * drv83xx_regRestoreFromCache()
 * Restores the device register values by rewriting them with the cached values.
 * */

//void drv83xx_regRestoreFromCache()
//{
//    if(SensorlessTrapController.DeviceID & BIT0)                  // Initialize the SPI variables and settings only if the device is "S" Variant
//    {
//        /* Write all the cached register values to the device */
//        SPI_Write(SPI_REG_DRV_CTRL, Reg_Map_Cache.Driver_Control_Reg2);
//        SPI_Write(SPI_REG_GATE_DRV_HS, Reg_Map_Cache.Gate_Drive_HS_Reg3);
//        SPI_Write(SPI_REG_GATE_DRV_LS, Reg_Map_Cache.Gate_Drive_LS_Reg4);
//        SPI_Write(SPI_REG_OCP_CTRL, Reg_Map_Cache.OCP_Control_Reg5);
//
//        /* This register exists only in DRV8323S */
//        if (SensorlessTrapController.DeviceID & BIT2)
//        {
//            SPI_Write(SPI_REG_CSA_CTRL, Reg_Map_Cache.CSA_Control_Reg6);
//        }
//    }
//}

/*function
 * drv832x_regToCache(unsigned char address)
 * Caches the device register value in the firmware
 * */
//void drv832x_regToCache(unsigned char address, unsigned int regValue)
//{
//    if(SensorlessTrapController.DeviceID & BIT0)                  // Initialize the SPI variables and settings only if the device is "S" Variant
//    {
//
//        switch (address)
//        {
//            case 0:
//                Reg_Map_Cache.Fault_Status_Reg0 = regValue;
//                break;
//
//            case 1:
//                Reg_Map_Cache.VGS_Status_Reg1 = regValue;
//                break;
//
//            case 2:
//                Reg_Map_Cache.Driver_Control_Reg2 = regValue;
//                break;
//
//            case 3:
//                Reg_Map_Cache.Gate_Drive_HS_Reg3 = regValue;
//                break;
//
//            case 4:
//                Reg_Map_Cache.Gate_Drive_LS_Reg4 = regValue;
//                break;
//
//            case 5:
//                Reg_Map_Cache.OCP_Control_Reg5 = regValue;
//                break;
//
//            case 6:
//                /* This register exists only in DRV8323S */
//                if (SensorlessTrapController.DeviceID & BIT2)
//                {
//                    Reg_Map_Cache.CSA_Control_Reg6 = regValue;
//                }
//                break;
//
//            default:
//                break;
//        }
//    }
//}

/*function
 * drv832x_registerRead(unsigned char address)
 * Device specific register read funtion
 * */
//unsigned int drv832x_registerRead(unsigned char address)
//{
//    unsigned int regValue;
//
//    /* Read the value from the device*/
//    regValue = SPI_Read(address);
//
//    /* Cache the value in the firmware */
//    if (ApplicationStatus.fault == 0)
//    {
//        drv832x_regToCache(address, regValue);
//    }
//
//    return regValue;
//}

/*function
 * drv832x_registerWrite()
 * Device specific register write funtion
 * */
//void drv832x_registerWrite(unsigned char address, unsigned int value)
//{
//     if((HostController.Start_Stop_Motor == 0) && (address == 0x02))  // Allow SPI write function to motor control modes address 0x02 only when the motor is in stop state
//     {
//        ;
//     }
//     else
//     {
//        /* Cache the value in the firmware */
//        if (ApplicationStatus.fault == 0)
//        {
//            drv832x_regToCache(address, value);
//        }
//
//        /* Write the value to the device */
//        SPI_Write(address, value);
//     }
//}

void drv83xx_StartMotor()
{
    HostController.Start_Stop_Motor = 0;
}

void drv83xx_StopMotor()
{
    HostController.Start_Stop_Motor = 1;
}
/*
void drv832x_setCtrlTypeParam(unsigned char value)
{
    SensorlessTrapController.PWM_Mode = value;
    DisableGateDrivers();
    if(SensorlessTrapController.PWM_Mode == 0) // If 6x PWM mode configure the Mode pin to pull down
    {
        P3SEL0 &= ~BIT0;  // Select the GPIO functionality of the pin
        P3DIR |= BIT0;   // Set the pin as Output
        P3OUT &= ~BIT0;  // Set the pin logic low
    }
    else // If 1x PWM mode configure mode pin to High Impedence state
    {
        P3SEL0 &= ~BIT0;  // Select the GPIO functionality of the pin
        P3DIR &= ~BIT0;  // Set the pin as Input
        P3REN &= ~BIT0;  // Diable the the Pull up/ pull down set the pin to high impedence
    }
//    if(SensorlessTrapController.DeviceID & BIT0)                  // Write the PWM mode to registers only if the device is "S" Variant
//    {
//            unsigned int regValue;
//
//            regValue = Reg_Map_Cache.Driver_Control_Reg2;
//
//            // 1XPWM
//            if (value)
//            {
//                regValue &= ~PWM_MODE_MASK;
//                regValue |= PWM_MODE_1X;
//            }
//            else
//            {
//                regValue &= ~PWM_MODE_MASK;
//            }
//
//            Reg_Map_Cache.Driver_Control_Reg2 = regValue;
//            SPI_Write(SPI_REG_DRV_CTRL, Reg_Map_Cache.Driver_Control_Reg2);
//    }
}*/
//unsigned char drv832x_getCtrlTypeParam(void)
//{
//    return(SensorlessTrapController.PWM_Mode);
//}
/*
void drv832x_setMtrParam(unsigned char num, unsigned long value)
{
    switch(num)
    {
        case MTR_PARAM_ISC_MIN_BEMF:
            SensorlessTrapController.ISCMinBEMF = *((uint16_t*)&value);
            break;
        case MTR_PARAM_ISC_BRAKE_TIME:
            SensorlessTrapController.ISCBrakeTime = *((uint16_t*)&value);
            break;
        case MTR_PARAM_IPD_BRAKE_TIME:
            SensorlessTrapController.IPDBrakeTime = *((uint16_t*)&value);
            break;
        case MTR_PARAM_IPD_PULSE_TIME:
            SensorlessTrapController.IPDPulseTime = *((uint16_t*)&value);
            break;
        case MTR_PARAM_IPD_DECAY_CONSTANT:
            SensorlessTrapController.IPDDecayConstant = *((uint16_t*)&value);
            break;
        case MTR_PARAM_ALIGN_SECTOR:
            SensorlessTrapController.AlignSector = *((uint16_t*)&value);
            break;
        case MTR_PARAM_ALIGN_WAIT_TIME:
            SensorlessTrapController.AlignWaitTime = *((uint16_t*)&value);
            break;
        case MTR_PARAM_ACCEL_RATE:
            SensorlessTrapController.AccelRate = *((uint16_t*)&value);
            break;
        case MTR_PARAM_ACCEL_STOP:
            SensorlessTrapController.AccelStop = *((uint16_t*)&value);
            break;
        case MTR_PARAM_ACCEL_VEL_INIT:
            SensorlessTrapController.AccelVelocity = *((uint16_t*)&value);
            break;
        case MTR_PARAM_BEMF_THRESHOLD:
            SensorlessTrapController.BEMFThreshold = *((uint16_t*)&value);
            break;
        case MTR_PARAM_RAMP_RATE_DELAY:
            SensorlessTrapController.RampRateDelay = *((uint16_t*)&value);
            break;
        case MTR_PARAM_COMM_BLANK_TIME:
            SensorlessTrapController.CommutationBlankTime = *((uint16_t*)&value);
            break;
        case MTR_PARAM_PWM_BLANK_COUNTS:
            SensorlessTrapController.PWMBlankCounts = *((uint16_t*)&value);
            break;
        case MTR_PARAM_MAX_DUTY_CYCLE:
            SensorlessTrapController.MaxDutyCycle = *((uint16_t*)&value);
            break;
        case MTR_PARAM_MIN_OFF_DUTY_CYCLE:
            SensorlessTrapController.MinOffDutyCycle = *((uint16_t*)&value);
            break;
        case MTR_PARAM_MIN_ON_DUTY_CYCLE:
            SensorlessTrapController.MinOnDutyCycle = *((uint16_t*)&value);
            break;
        case MTR_PARAM_START_UP_DUTY_CYCLE:
            SensorlessTrapController.StartupDutyCycle = *((uint16_t*)&value);
            break;
        case MTR_PARAM_PWM_FREQ:
            SensorlessTrapController.PWMPeriod = *((uint16_t*)&value);
            SensorlessTrapController.Counter_1M_Second = (24000 / SensorlessTrapController.PWMPeriod);
            SensorlessTrapController.Counter_1_Second = SensorlessTrapController.Counter_1M_Second * 1000;
            TimerB0_Init();             // Timer B0 initialization
            TimerB3_Init();
            break;
        case MTR_PARAM_UNDER_VOL_LIM:
            SensorlessTrapController.UnderVolLim = *((uint16_t*)&value);
            break;
        case MTR_PARAM_OVER_VOL_LIM:
            SensorlessTrapController.OverVolLim = *((uint16_t*)&value);
            break;
        case MTR_PARAM_STALL_DETECT_REV:
            SensorlessTrapController.StallDetectRev = *((uint16_t*)&value);
            break;
        case MTR_PARAM_STALL_DETECT_TIME:
            SensorlessTrapController.StallDetecttime = *((uint16_t*)&value);
            break;
        case MTR_PARAM_MOTOR_PHASE_CURR_LIM:
            SensorlessTrapController.MotorPhaseCurrentLimit = *((uint16_t*)&value);
            break;
        case MTR_PARAM_AUTO_FAULT_RECOVERY_TIME:
            SensorlessTrapController.AutoFaultRecoveryTime = *((uint16_t*)&value);
            break;
        case MTR_PARAM_ALIGN_IPD:
            SensorlessTrapController.Align_IPD = *((uint16_t*)&value);
            break;
        case MTR_PARAM_DIR:
            if((HostController.Start_Stop_Motor)&&(ApplicationStatus.currentstate != MOTOR_RUN))
            {
                SensorlessTrapController.Direction = !SensorlessTrapController.Direction;
            }
            else
            {
                SensorlessTrapController.Direction_flag =  !SensorlessTrapController.Direction_flag;
            }
            break;
        case MTR_PARAM_SPEED:
            SensorlessTrapController.SetMotorSpeed = *((uint16_t*)&value);
            break;
        default:
            break;
    }
}*/
//dtbui
/*
unsigned long drv832x_getMtrParam(unsigned char num)
{
    switch(num)
    {
        case MTR_PARAM_ELEC_SPEED:
            return(SensorlessTrapController.SPDFdbk);
        case MTR_PARAM_FAULT_STATE:
            return(ApplicationStatus.fault);
        case MTR_PARAM_DEVICE_ID:
            return(SensorlessTrapController.DeviceID);
        case MTR_PARAM_ISC_MIN_BEMF:
            return(SensorlessTrapController.ISCMinBEMF);
        case MTR_PARAM_ISC_BRAKE_TIME:
            return(SensorlessTrapController.ISCBrakeTime);
        case MTR_PARAM_IPD_BRAKE_TIME:
            return(SensorlessTrapController.IPDBrakeTime);
        case MTR_PARAM_IPD_PULSE_TIME:
            return(SensorlessTrapController.IPDPulseTime);
        case MTR_PARAM_IPD_DECAY_CONSTANT:
            return(SensorlessTrapController.IPDDecayConstant);
        case MTR_PARAM_ALIGN_SECTOR:
            return(SensorlessTrapController.AlignSector);
        case MTR_PARAM_ALIGN_WAIT_TIME:
            return(SensorlessTrapController.AlignWaitTime);
        case MTR_PARAM_ACCEL_RATE:
            return(SensorlessTrapController.AccelRate);
        case MTR_PARAM_ACCEL_STOP:
            return(SensorlessTrapController.AccelStop);
        case MTR_PARAM_ACCEL_VEL_INIT:
            return(SensorlessTrapController.AccelVelocity);
        case MTR_PARAM_BEMF_THRESHOLD:
            return(SensorlessTrapController.BEMFThreshold);
        case MTR_PARAM_RAMP_RATE_DELAY:
            return(SensorlessTrapController.RampRateDelay);
        case MTR_PARAM_COMM_BLANK_TIME:
            return(SensorlessTrapController.CommutationBlankTime);
        case MTR_PARAM_PWM_BLANK_COUNTS:
            return(SensorlessTrapController.PWMBlankCounts);
        case MTR_PARAM_MAX_DUTY_CYCLE:
            return(SensorlessTrapController.MaxDutyCycle);
        case MTR_PARAM_MIN_OFF_DUTY_CYCLE:
            return(SensorlessTrapController.MinOffDutyCycle);
        case MTR_PARAM_MIN_ON_DUTY_CYCLE:
            return(SensorlessTrapController.MinOnDutyCycle);
        case MTR_PARAM_START_UP_DUTY_CYCLE:
            return(SensorlessTrapController.StartupDutyCycle);
        case MTR_PARAM_PWM_FREQ:
            return(SensorlessTrapController.PWMPeriod);
        case MTR_PARAM_UNDER_VOL_LIM:
            return(SensorlessTrapController.UnderVolLim);
        case MTR_PARAM_OVER_VOL_LIM:
            return(SensorlessTrapController.OverVolLim);
        case MTR_PARAM_STALL_DETECT_REV:
            return(SensorlessTrapController.StallDetectRev);
        case MTR_PARAM_STALL_DETECT_TIME:
            return(SensorlessTrapController.StallDetecttime);
        case MTR_PARAM_MOTOR_PHASE_CURR_LIM:
            return(SensorlessTrapController.MotorPhaseCurrentLimit);
        case MTR_PARAM_AUTO_FAULT_RECOVERY_TIME:
            return(SensorlessTrapController.AutoFaultRecoveryTime);
        case MTR_PARAM_ALIGN_IPD:
            return(SensorlessTrapController.Align_IPD);
        case MTR_PARAM_DIR:
            return(SensorlessTrapController.Direction);
        case MTR_PARAM_SPEED:
            return(SensorlessTrapController.SetMotorSpeed);
        case MTR_START_STOP_MOTOR:
            return(HostController.Start_Stop_Motor);
        default:
            return(0);
    }
}
*/
 //dtbui
//unsigned char drv832x_getGPIO(unsigned char gpioPort, unsigned char gpioNum)
//{
//    if (gpioPort == 0x01)
//    {
//        if ((gpioNum & P1IN) == 0)
//        {
//            HostController.EnabledGateDrivers = 0;
//            return 0;
//        }
//        else
//        {
//            HostController.EnabledGateDrivers = 1;
//            return 1;
//        }
//    }
//    else
//    {
//        // do nothing
//        return 0;
//    }
//}
//void drv832x_setGPIO(unsigned char gpioPort, unsigned char gpioNum, unsigned char gpioVal)
//{
//        if (gpioPort == 0x01)
//        {
//            if(gpioVal == 0)
//            {
//                P1OUT &= ~gpioNum;
//                HostController.EnabledGateDrivers = 0;
//            }
//            else
//            {
//                P1OUT |= gpioNum;
//                HostController.EnabledGateDrivers = 1;
//            }
//        }
//        else
//        {
//            // do nothing
//        }
//
//}


/*function
 * IPD_SetState(uint8_t hallState)
 * Set IPD State
 * INPUT: commutation  state
 * */
void IPD_SetState(uint8_t commState)
{
    if(SensorlessTrapController.PWM_Mode == 0)  // If six PWM mode
    {
        P6OUT &= ~BIT0;         // Turn High Side A phase Off
        P6OUT &= ~BIT2;         // Turn High Side B phase Off
        P6OUT &= ~BIT4;         // Turn High Side C phase Off
        P6OUT &= ~BIT1;         // Turn Low Side A phase Off
        P6OUT &= ~BIT3;         // Turn Low Side B phase Off
        P6OUT &= ~BIT5;         // Turn Low Side C phase Off
        switch(commState)
        {
        case 1:        /* B-C */

            P6OUT |= BIT2;                                                   /* Set High side of B phase*/
            P6OUT |= BIT5;                                                                          /* Set Low side of C phase */

            break;

        case 2:       /* C-A */

            P6OUT |= BIT4;                                                                          /* Set High side of C phase*/
            P6OUT |= BIT1;                                                                          /* Set Low side of A phase */

            break;

        case 3:        /* A-B */

            P6OUT |= BIT0;                                                                          /* Set High side of A phase*/
            P6OUT |= BIT3;                                                                          /* Set Low side of B phase */

            break;
        case 4:            /* B-A */

            P6OUT |= BIT2;                                                   /* Set High side of B phase*/
            P6OUT |= BIT1;                                                                          /* Set Low side of A phase */
            break;

        case 5:       /* A-C */

            P6OUT |= BIT0;                                                                          /* Set High side of A phase*/
            P6OUT |= BIT5;                                                                          /* Set Low side of C phase */

            break;

        case 6:       /* C-B */

            P6OUT |= BIT4;                                                  /* Set High side of C phase*/
            P6OUT |= BIT3;                                                                          /* Set Low side of B phase */

            break;
        default:
            break;
        }
    }
    else  // If 1x PWM mode
    {
        P6SEL0 &= ~(BIT1 | BIT3 | BIT2 | BIT5 | BIT4);
        P6SEL0 |= BIT0  ;  // Generate Single PWM for modulating duty cycle and frequency
        TB3CCR1 = SensorlessTrapController.PWMPeriod; // Set the PWM to 100% duty
        P6OUT &= ~BIT1 ; // Make INLA as GPIO low to switch of phase A switches
        P6OUT &= ~BIT2 ; //  Make INHB as GPIO low to switch of phase B switches
        P6OUT &= ~BIT3 ; //  Make INLB as GPIO low to switch of phase C switches
        P6OUT |= BIT5  ; //  Make INLC as GPIO high to remove the brakes

        switch(commState)
        {
        case 1:        /* B-C */

            P6OUT |= BIT1;
            P6OUT |= BIT2;
            break;

        case 2:        /* C-A */
            P6OUT |= BIT3;
            P6OUT |= BIT2;

            break;

        case 3:        /* A-B */
            P6OUT |= BIT1;
            P6OUT |= BIT3;
            break;

        case 4:            /* B-A */
            P6OUT |= BIT2;

            break;

        case 5:       /* A-C */
            P6OUT |= BIT1;

            break;

        case 6:         /* C-B */
            P6OUT |= BIT3;
            break;
        default:
            break;
        }

    }
}

/*function
 * PWM_SetCommutation(uint8_t hallState)
 * Set PWM commutation
 * INPUT: Hall state
 * */
void PWM_SetCommutation(uint8_t commState)
{

    if(SensorlessTrapController.PWM_Mode == 0)
    {
        /* Implementing Synchronous PWM i.e. to Toggle between High side and low side of a Phase with Dead Band*/

        switch(commState)
        {
        case 1:        /* B-C */

            /* Reset switches for phase A (LOW-HIGH) */

            P6SEL0 &= ~(BIT0 | BIT1);                                                                    /* Select P2.4 , P2.5 as I/O Function for Phase A*/
            P6OUT &= ~(BIT0 | BIT1);                                    /* Reset bits P2.4 , P2.5  */
            /* Reset switches for phase C (HIGH)*/

            P6SEL0 &= ~( BIT4);                                                                    /* Select P1.2 , P1.3 as I/O Function for Phase C*/
            P6OUT &= ~( BIT4);                                    /* Reset bits P1.2 , P1.3  */

            P6SEL0 |= BIT2 | BIT3;                                   /* Select Synchronous PWM for B phase*/
            P6OUT |= BIT5;                                                                          /* Set Low side of C phase */
            SensorlessTrapController.RotationCount++;
            break;

        case 2:        /* A-C */

            /* Reset switches for phase B (LOW-HIGH)*/
            P6OUT &= ~(BIT2 | BIT3);                                    /* Reset bits P1.4 , P1.5  */
            P6SEL0 &= ~(BIT2 | BIT3);                                                                    /* Select P1.4 , P1.5 as I/O Function for Phase B*/

            /* Reset switches for phase C (HIGH)*/
            P6OUT &= ~(BIT4);                                    /* Reset bits P1.2 , P1.3  */
            P6SEL0 &= ~(BIT4);                                                                    /* Select P1.2 , P1.3 as I/O Function for Phase C*/

            P6SEL0 |= BIT0 | BIT1;                                                           /* Select Synchronous PWM for of A phase*/
            P6OUT |= BIT5;                                                                          /* Set Low side of C phase */
            break;

        case 3:        /* A-B */

            /* Reset switches for phase B (HIGH)*/
            P6OUT &= ~(BIT2);                                    /* Reset bits P1.4 , P1.5  */
            P6SEL0 &= ~(BIT2);                                                                    /* Select P1.4 , P1.5 as I/O Function for Phase B*/

            /* Reset switches for phase C (LOW-HIGH)*/
            P6OUT &= ~(BIT4 | BIT5);                                    /* Reset bits P1.2 , P1.3  */
            P6SEL0 &= ~(BIT4 | BIT5);                                                                    /* Select P1.2 , P1.3 as I/O Function for Phase C*/


            P6SEL0 |= BIT0 | BIT1;                                                           /* Select Synchronous PWM for A phase*/
            P6OUT |= BIT3;                                                                          /* Set Low side of B phase */
            break;

        case 4:            /* C-B */

            /* Reset switches for phase A (LOW-HIGH) */

            P6OUT &= ~(BIT0 | BIT1);                                    /* Reset bits P2.4 , P2.5  */
            P6SEL0 &= ~(BIT0 | BIT1);                                                                    /* Select P2.4 , P2.5 as I/O Function for Phase A*/

            /* Reset switches for phase B (HIGH)*/
            P6OUT &= ~(BIT2);                                    /* Reset bits P1.4 , P1.5  */
            P6SEL0 &= ~(BIT2);                                                                    /* Select P1.4 , P1.5 as I/O Function for Phase B*/


            P6SEL0 |= BIT4 | BIT5;                                   /* Select Synchronous PWM for C phase*/
            P6OUT |= BIT3;                                                                          /* Set Low side of B phase */
            break;

        case 5:       /* C-A */

            /* Reset switches for phase A (HIGH) */

            P6OUT &= ~(BIT0);                                    /* Reset bits P2.4 , P2.5  */
            P6SEL0 &= ~(BIT0);                                                                    /* Select P2.4 , P2.5 as I/O Function for Phase A*/

            /* Reset switches for phase B (LOW-HIGH)*/
            P6OUT &= ~(BIT2 | BIT3);                                    /* Reset bits P1.4 , P1.5  */
            P6SEL0 &= ~(BIT2 | BIT3);                                                                    /* Select P1.4 , P1.5 as I/O Function for Phase B*/

            P6SEL0 |= BIT4 | BIT5;                                                           /* Select Synchronous PWM for C phase*/
            P6OUT |= BIT1;                                                                          /* Set Low side of A phase */

            break;

        case 6:         /* B-A */

            /* Reset switches for phase A (HIGH) */

            P6OUT &= ~(BIT0);                                    /* Reset bits P2.4 , P2.5  */
            P6SEL0 &= ~(BIT0);                                                                    /* Select P2.4 , P2.5 as I/O Function for Phase A*/

            /* Reset switches for phase C (LOW-HIGH)*/
            P6OUT &= ~(BIT4 | BIT5);                                    /* Reset bits P1.2 , P1.3  */
            P6SEL0 &= ~(BIT4 | BIT5);                                                                    /* Select P1.2 , P1.3 as I/O Function for Phase C*/


            P6SEL0 |= BIT2 | BIT3;                                                           /* Select Synchronous PWM for B phase*/
            P6OUT |= BIT1;                                                                          /* Set Low side of A phase */

            break;
        default:

            DisableGateDrivers();
            break;
        }
    }
    else
    {
        switch(commState)
        {
        case 1:        /* B-C */

            P6SEL0 &= ~(BIT3 | BIT4 | BIT5);
            P6SEL0 |= BIT0  ;  // Generate Single PWM for modulating duty cycle and frequency

            P6OUT &= ~BIT3 ; //  Make INLB as GPIO low to switch of phase C switches
            P6OUT |= BIT5  ; //  Make INLC as GPIO low to remove the brakes

            P6OUT |= BIT1;
            P6OUT |= BIT2;

            SensorlessTrapController.RotationCount++;
            break;

        case 2:        /* A-C */

            P6SEL0 &= ~(BIT2 | BIT3 | BIT4 | BIT5);
            P6SEL0 |= BIT0  ;  // Generate Single PWM for modulating duty cycle and frequency

            P6OUT &= ~(BIT2 | BIT3) ;
            P6OUT |= BIT5  ; //  Make INLC as GPIO low to remove the brakes

            P6OUT |= BIT1;
            break;

        case 3:        /* A-B */

            P6SEL0 &= ~(BIT2 | BIT4 | BIT5);
            P6SEL0 |= BIT0  ;  // Generate Single PWM for modulating duty cycle and frequency

            P6OUT &= ~BIT2 ; //  Make INHB as GPIO low to switch of phase B switches
            P6OUT |= BIT5  ; //  Make INLC as GPIO low to remove the brakes

            P6OUT |= BIT1;
            P6OUT |= BIT3;
            break;

        case 4:            /* C-B */

            P6SEL0 &= ~BIT1;
            P6SEL0 &= ~(BIT2);        /* Select P1.4 , P1.5 as I/O Function for Phase B*/
            P6SEL0 &= ~(BIT4 | BIT5);        /* Select P1.2 , P1.3 as I/O Function for Phase C*/                                                                    /* Select P2.4  I/O Function for Phase A*/
            P6SEL0 |= BIT0  ;  // Generate Single PWM for modulating duty cycle and frequency

            P6OUT &= ~BIT1 ; // Make INLA as GPIO low to switch of phase A switches
            P6OUT &= ~BIT2 ; //  Make INHB as GPIO low to switch of phase B switches
            P6OUT |= BIT5  ; //  Make INLC as GPIO low to remove the brakes

            P6OUT |= BIT3;
            break;

        case 5:       /* C-A */

            P6SEL0 &= ~(BIT1 | BIT4 | BIT5);
            P6SEL0 |= BIT0  ;  // Generate Single PWM for modulating duty cycle and frequency

            P6OUT &= ~BIT1 ; // Make INLA as GPIO low to switch of phase A switches
            P6OUT |= BIT5  ; //  Make INLC as GPIO low to remove the brakes

            P6OUT |= BIT3;
            P6OUT |= BIT2;
            break;

        case 6:         /* B-A */

            P6SEL0 &= ~(BIT1 | BIT3 | BIT4 | BIT5);
            P6SEL0 |= BIT0  ;  // Generate Single PWM for modulating duty cycle and frequency

            P6OUT &= ~(BIT1 | BIT3) ;
            P6OUT |= BIT2 | BIT5  ; //  Make INLC as GPIO low to remove the brakes


            break;
        default:
            DisableGateDrivers();
            break;

        }
    }
}

/*function
 * SetPWMDutyCycle(UINT16 PWMDutyCycle)
 * Sets the PWM Duty Cycle
 * INPUT: DutyCycle Counter Value
 * */
void SetPWMDutyCycle(uint16_t PWMDutyCycle)
{ //PWM_BLANK_COUNTS is the time before the falling PWM edge for ADC sampling
    TB3CCR1 = PWMDutyCycle;
    TB3CCR2 = PWMDutyCycle;
    TB3CCR3 = PWMDutyCycle;
    TB3CCR4 = PWMDutyCycle;
    TB3CCR5 = PWMDutyCycle;
    TB3CCR6 = PWMDutyCycle;
    TB0CCR1 = ((PWMDutyCycle>>1) -  SensorlessTrapController.PWMBlankCounts);
}

/*function
 * UpdateNextCommutation()
 * Increment or decrement commutation based on direction
 * */
void UpdateNextCommutation(void)
{

    if(SensorlessTrapController.Direction == TRUE)
    {
        SensorlessTrapController.CurrentCommState++;
        if(SensorlessTrapController.CurrentCommState > 6)
        {
            SensorlessTrapController.CurrentCommState = 1;
        }
    }
    else
    {
        SensorlessTrapController.CurrentCommState--;
        if(SensorlessTrapController.CurrentCommState < 1)
        {
            SensorlessTrapController.CurrentCommState = 6;
        }
    }
    if((SensorlessTrapController.CurrentCommState ==1) && (ApplicationStatus.currentstate == MOTOR_RUN))
    {
        ReadSPDFDBK();
    }
}

/*function
 * UpdateBEMFADC
 * Selects the floating phase to be monitored based on commutation states
 * */
void UpdateBEMFADC(void)

{

    switch(SensorlessTrapController.CurrentCommState)
    {
    case 1: ADCMCTL0 = ADCINCH_1;               //   channel = A0 (Read the BEMF reading from Phase A ) ,
        break;
    case 2: ADCMCTL0 = ADCINCH_2;               //   channel = A1 (Read the BEMF reading from Phase B ) ,
        break;
    case 3: ADCMCTL0 = ADCINCH_3;               //   channel = A2 (Read the BEMF reading from Phase C ) ,
        break;
    case 4: ADCMCTL0 = ADCINCH_1;               //   channel = A0 (Read the BEMF reading from Phase A ) ,
        break;
    case 5: ADCMCTL0 = ADCINCH_2;               //   channel = A1 (Read the BEMF reading from Phase B ) ,
        break;
    case 6: ADCMCTL0 = ADCINCH_3;               //   channel = A2 (Read the BEMF reading from Phase C ) ,
        break;
    default:
        break;
    }

}

/*function
 * FastReadBEMF
 * Triggers ADC and Samples BEMF of all three Phases
 * is the speed input (potentiometer) for the applications
 * */
void FastReadBEMF(void)
{
    ADCCTL0 |= ADCENC | ADCSC;                                                      // Start sampling of channels
    while(ADCCTL1 & ADCBUSY_L)
    {
    }
    ;

    ADCCTL0 &= ~ADCENC;
    ADCCTL0 &= ~ADCSC;

    SensorlessTrapController.GetBEMF = ADCMEM0 & 0x0FFF;
//    if(SensorlessTrapController.GetBEMF>1000)
//      {
//          P4OUT |= BIT7;
//          P5OUT &= ~BIT1;
//      }
//
//      else
//      {
//          P5OUT |= BIT1;
//          P4OUT &= ~BIT7;
//      }
    /* Filter only last 12 bits */;
    SensorlessTrapController.GetBEMF >>= PWM_FACTOR;            /*12 bit ADC result has to be scaled to 10 bit value */
}

/*function
 * ReadVCC()
 * Triggers ADC and samples supply voltage and evaluates for over or under voltage fault
 * */
void ReadVCC()
{
    ADCMCTL0 = ADCINCH_0;         //   channel = A5 (Read the supply voltage)
    ADCCTL0 |= ADCENC | ADCSC;
    while(ADCCTL1 & ADCBUSY_L)
    {
    }
    ;
    ADCCTL0 = ~ADCENC;
    //ADCCTL0 = ~ADCSC;

//    SensorlessTrapController.VCCvoltage = ADCMEM0 & 0x0FFF;       /* Filter the result and read only last 12 bits because MSP430F5529 has 12bit ADC*/
//    if(ADCMEM0>1700)
//     {
//         P4OUT |= BIT7;
//         P5OUT &= ~BIT1;
//     }
//
//     else
//     {
//         P5OUT |= BIT1;
//         P4OUT &= ~BIT7;
//     }
    if (count_m < 8)
    {
        count_m ++;
        SensorlessTrapController.VCCvoltage = ADCMEM0 & 0x0FFF;
    }
    vol_pre = vol_array[count_j];
    vol_array[count_j] = ADCMEM0 & 0x0FFF;
    if(count_j < 8)
    {
        sum_vol += vol_array[count_j];
        sum_vol -= vol_pre;
        count_j ++;
        if(count_j != 0 && count_m == 8)
            SensorlessTrapController.VCCvoltage = sum_vol >> 3;

        if (count_j == 8)
            count_j = 0;
    }
    if (ApplicationStatus.currentstate == FAULT || ApplicationStatus.currentstate == MOTOR_STOP)
        count_m = 0;

    SensorlessTrapController.VCCvoltage >>= PWM_FACTOR;                         /*12 bit ADC result has to be scaled to 10 bit value */
//    if(close_loop_cycle>100)
//        SensorlessTrapController.CTvoltage = //1450;//460;
//                (SensorlessTrapController.VCCvoltage >> 1);                                         // Center tap voltage is VCC/2 , used in BEMF integration calculation
//    else
        SensorlessTrapController.CTvoltage = //1300;//460;
        (SensorlessTrapController.VCCvoltage >> 1);                                         // Center tap voltage is VCC/2 , used in BEMF integration calculation

    if(SensorlessTrapController.VCCvoltage <
            SensorlessTrapController.UnderVolLim | SensorlessTrapController.VCCvoltage >
    SensorlessTrapController.OverVolLim)                                                                                                       /* Under Voltage of 10.0V and over voltage at 20.0V*/
    {
        ApplicationStatus.previousstate = ApplicationStatus.currentstate;
        ApplicationStatus.currentstate = FAULT;
        ApplicationStatus.fault = VOLTAGE;
    }
}

/*function
 * SetMotorSpeed()
 * Reads the Input motor speed from the GUI to set the Target duty cycle
 * */
void SetMotorSpeed(void)
{
    //SensorlessTrapController.TargetDutyCycle = SensorlessTrapController.SetMotorSpeed;   // Update the Duty cycle with Motor speed set from the MDBU serial   //dtbui
    //read speed from pot //dtbui

//    ADCMCTL0 = ADCINCH_4;         //   channel = A6 (Read the pot)
//    ADCCTL0 |= ADCENC;                                 // Enable Conversions
//    ADCCTL0 |= ADCSC;                          // Start sampling of channels
//    while (ADCCTL1 & ADCBUSY_L)
//    {
//    };
//
//    ADCCTL0 &= ~ADCENC;                          // End sampling of channels
//    ADCCTL0 &= ~ADCSC;                                // Disable conversions
//
//    SensorlessTrapController.TargetDutyCycle = ADCMEM0 & 0x0FFF; // Filter the result and read only last 12 bits because MSP430F5529 has 12bit ADC
//    SensorlessTrapController.TargetDutyCycle >>= PWM_FACTOR; //2;PWM_FACTOR;                         //12 bit ADC result has to be scaled to 10 bit value




    //set start speed

    SensorlessTrapController.TargetDutyCycle = SensorlessTrapController.SetSpeed;
    //
    if(SensorlessTrapController.TargetDutyCycle >= SensorlessTrapController.MaxDutyCycle)
    {
        SensorlessTrapController.TargetDutyCycle = SensorlessTrapController.MaxDutyCycle;
    }
    else if((SensorlessTrapController.CurrentDutyCycle <= SensorlessTrapController.MinOffDutyCycle) &&
            (ApplicationStatus.currentstate != MOTOR_IDLE) &&
            (ApplicationStatus.currentstate != MOTOR_DIRECTION))
    {
        //ensure the target duty cycle stays above minimum or shut it off
        if(SensorlessTrapController.TargetDutyCycle <= SensorlessTrapController.CurrentDutyCycle)
        {
            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
            ApplicationStatus.currentstate = MOTOR_STOP;
        }
    }
    if(SensorlessTrapController.TargetDutyCycle !=
       SensorlessTrapController.CurrentDutyCycle)
    {                                                                                                                                                   //set a flag is the speed has changed
        SensorlessTrapController.SpeedChange = TRUE;
    }
}

/*function
 * ReadCurrentshunt()
 * Reads CSA value and triggers OC faults for Motor current greater than Set Limit
 * */
void ReadCurrentShunt()
{
    ADCMCTL0 = ADCINCH_8;               //   channel = A4 (Read the CSA reading from Phase A for over current protection) , End of Sequence
//    Current_array[99]=0;
//    for (i = 0; i < 16; i++)
//    {
        ADCCTL0 |= ADCENC | ADCSC;                 // Start sampling of channels
        while (ADCCTL1 & ADCBUSY_L)
        {
        };
//    SensorlessTrapController.MotorPhaseCurrent = ADCMEM0 & 0x0FFF;       /* Filter the result and read only last 12 bits because MSP430F5529 has 12bit ADC*/
//        Current_array[i] = ADCMEM0 & 0x0FFF;
//        Current_array[99] += Current_array[i];
        sum_current += ADCMEM0 & 0x0FFF;
        count_i++;
        ADCCTL0 &= ~ADCENC;                          // End sampling of channels
        ADCCTL0 &= ~ADCSC;
//    }
//    SensorlessTrapController.MotorPhaseCurrent=Current_array[99]>>4;
//    if(SensorlessTrapController.DeviceID & BIT2)    // If Current sense is in phase shunt remove the offset.
//    {
//
//        SensorlessTrapController.MotorPhaseCurrent -= 2048;     // subtracting the bias, Vref/2 is added as bias voltage to support bidirectional current sensing
//    }
    if (count_i == 16)
    {
        SensorlessTrapController.MotorPhaseCurrent = sum_current >> 4;
        sum_current = 0;
        count_i = 0;
    }

    SensorlessTrapController.MotorPhaseCurrent = abs(
    SensorlessTrapController.MotorPhaseCurrent);

//    if(SensorlessTrapController.MotorPhaseCurrent > 200)
//        P3OUT |= BIT1;
    if(SensorlessTrapController.MotorPhaseCurrent >
       SensorlessTrapController.MotorPhaseCurrentLimit)                                                   /* Motor Phase Current Limit*/
    {

        ApplicationStatus.previousstate = ApplicationStatus.currentstate;
        ApplicationStatus.currentstate = FAULT;
        ApplicationStatus.fault = OVERCURRENT;
    }
}
//void ReadCurrentShunt()
//{
//    ADCMCTL0 = ADCINCH_8;               //   channel = A4 (Read the CSA reading from Phase A for over current protection) , End of Sequence
//    ADCCTL0 |= ADCENC | ADCSC;                                                      // Start sampling of channels
//    while(ADCCTL1 & ADCBUSY_L)
//    {
//    }
//    ;
//    SensorlessTrapController.MotorPhaseCurrent = ADCMEM0 & 0x0FFF;       /* Filter the result and read only last 12 bits because MSP430F5529 has 12bit ADC*/
//
//    ADCCTL0 &= ~ADCENC;                                                     // End sampling of channels
//    ADCCTL0 &= ~ADCSC;
//
////    if(SensorlessTrapController.DeviceID & BIT2)    // If Current sense is in phase shunt remove the offset.
////    {
////
////        SensorlessTrapController.MotorPhaseCurrent -= 2048;     // subtracting the bias, Vref/2 is added as bias voltage to support bidirectional current sensing
////    }
//    SensorlessTrapController.MotorPhaseCurrent = abs(
//    SensorlessTrapController.MotorPhaseCurrent);
//    if(SensorlessTrapController.MotorPhaseCurrent >
//       SensorlessTrapController.MotorPhaseCurrentLimit)                                                   /* Motor Phase Current Limit*/
//    {
//
//        ApplicationStatus.previousstate = ApplicationStatus.currentstate;
//        ApplicationStatus.currentstate = FAULT;
//        ApplicationStatus.fault = OVERCURRENT;
//    }
//}
/*function
 * ISCReadPhaseVoltage()
 * Triggers ADC and samples 3 phase voltages and evaluates if motor is already spinning
 * */
void ISCReadPhaseVoltage()
{
    ReadVCC();                                        // Read Supply Voltage Vcc

    //phase-A
    ADCMCTL0 = ADCINCH_1;           //   channel = A0 (Read the Phase A Voltage)
    ADCCTL0 |= ADCENC | ADCSC;
    while (ADCCTL1 & ADCBUSY_L)
    {
    };
    SensorlessTrapController.PhaseA_BEMF = abs(
            SensorlessTrapController.VCCvoltage - (ADCMEM0 & 0x0FFF));
    ADCCTL0 &= ~ADCENC;                                                     // End sampling of channels
    ADCCTL0 &= ~ADCSC;
    //phase-B
    ADCMCTL0 = ADCINCH_2;           //   channel = A0 (Read the Phase B Voltage)
    ADCCTL0 |= ADCENC | ADCSC;
    while (ADCCTL1 & ADCBUSY_L)
    {
    };
    SensorlessTrapController.PhaseB_BEMF = abs(
            SensorlessTrapController.VCCvoltage - (ADCMEM0 & 0x0FFF));
    ADCCTL0 &= ~ADCENC;                                                     // End sampling of channels
    ADCCTL0 &= ~ADCSC;
    //phase-C
    ADCMCTL0 = ADCINCH_3;           //   channel = A0 (Read the Phase C Voltage)
    ADCCTL0 |= ADCENC | ADCSC;
    while (ADCCTL1 & ADCBUSY_L)
    {
    };
    SensorlessTrapController.PhaseC_BEMF = abs(
            SensorlessTrapController.VCCvoltage - (ADCMEM0 & 0x0FFF));

    ADCCTL0 &= ~ADCENC;                                                     // End sampling of channels
    ADCCTL0 &= ~ADCSC;
}
void ReadSPDFDBK()
{
    //RPM count & Speed measurement of 18 Bit resolution

    if(SensorlessTrapController.TimerOverflowFlag == 0) // A count of 1 indicates timer A1 ticked for 0x010000 clocks
    {
        SensorlessTrapController.SPDFdbk = TB2R;  // Generate the speed count = The max value 0FFFF represents the least possible speed measurement which is 625K / 0x0FFFF = 9.52hz as  timer 1 is run at 625KHz
    }
    SensorlessTrapController.TimerOverflowFlag = 0 ; //Reset the Interrupt flag
    TB2R = 0x0000;              // Reset the timer count
}



