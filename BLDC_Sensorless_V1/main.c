
/* ******************************************************************************/
#include "global.h"

// Controller
SENSORLESS_TRAP_Obj SensorlessTrapController;
APPLICATION_STATUS ApplicationStatus;
HOSTCONTROL_STATUS HostControl_Status;
HOST_CONTROLLER_Obj HostController;

// Registers
FLT_STAT_REG0_Obj Fault_Status_Reg;
VGS_STAT_REG1_Obj VGS_Status_Reg;
DRV_CTRL_REG2_Obj Driver_Control_Reg;
GATE_DRV_HS_REG3_Obj Gate_Drive_HS_Reg;
GATE_DRV_LS_REG4_Obj Gate_Drive_LS_Reg;
OCP_CTRL_REG5_Obj OCP_Control_Reg;
CSA_CTRL_REG6_Obj CSA_Control_Reg;
REG_MAP_Obj Reg_Map_Cache;
/*function
 * DRV8x_State_Machine(void)
 * Handles the state machine and transitions of the Application
 * */
void DRV8x_State_Machine(void)
{
    // process current motor state

    if(ApplicationStatus.fault != 0)    // If any fault is detected change the state to fault
    {
        ApplicationStatus.previousstate = ApplicationStatus.currentstate;
        ApplicationStatus.currentstate = FAULT;
    }
    switch(ApplicationStatus.currentstate)
    {
    case SYSTEM_INIT:
        DRV8x_Digital_Init();                                              /*Initialize MSP430F5529 peripherals */
        sensorlessTrapController_Init();                                                // Initialize Motor Variables

//        if(HostController.EnabledGateDrivers)
//        {
//            drv83xx_regRestoreFromCache();                                                     // Restore device register values from the cached ones
//        }

        ApplicationStatus.previousstate = ApplicationStatus.currentstate;
        ApplicationStatus.currentstate = SYSTEM_IDLE;                                            // Move to Motor INIT state and initialize Motor variables
        break;
    case SYSTEM_IDLE:
        if (HostController.EnabledGateDrivers)
        {
            EnableGateDrivers();
        }
        if (HostController.Start_Stop_Motor)  // If Motor is in Stop Mode i.e. 1
        {
            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
            ApplicationStatus.currentstate = SYSTEM_IDLE;
//            DisableGateDrivers();
            P2OUT &= ~BIT0;
        }
        else                                 // If Motor is in Start Mode i.e. 0
        {
            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
            ApplicationStatus.currentstate = MOTOR_IDLE;
//            drv83xx_regRestoreFromCache(); // Restore device register values from the cached ones
        }

         // Read Supply Voltage Vcc
         break;
    case MOTOR_IDLE:

        SetMotorSpeed();
        if(HostController.Start_Stop_Motor)     // If Motor is in Stop Mode i.e. 1
        {
                ApplicationStatus.previousstate = ApplicationStatus.currentstate;
                ApplicationStatus.currentstate = SYSTEM_IDLE;
        }
        if((SensorlessTrapController.TargetDutyCycle > SensorlessTrapController.MinOnDutyCycle) && (!HostController.Start_Stop_Motor) )
        {
            sensorlessTrapController_Init();                                                // Initialize Motor Variables
            if(SensorlessTrapController.Direction_flag)
            {
                SensorlessTrapController.Direction =
                            !SensorlessTrapController.Direction;
                SensorlessTrapController.Direction_flag = FALSE;
            }
                                                                                  // Read ADC input for speed input (potentiometer)
            if(SensorlessTrapController.Direction == TRUE)
            {
//                P4OUT |= BIT5;                                                         // Turn  on LED0
                P2OUT |= BIT4;                                                                                 // Turn off LED 1
            }
            else
            {
//                P4OUT &= ~BIT5;                                                         // Turn  off LED0
                P2OUT &= ~BIT4;                                                                                    // Turn on LED 1

            }

            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
            ApplicationStatus.currentstate = MOTOR_ISC;                                             // Move to Motor ISC
            DisableGateDrivers();
            SensorlessTrapController.ISCStatus = READ_BEMF;
            ReadVCC();                                                                                      // Read Supply Voltage Vcc
        }
        ReadVCC();                                                                                          // Read Supply Voltage Vcc
        break;
    case MOTOR_ISC:
        // wait until a done signal is recieved from the ISC routine
        switch(SensorlessTrapController.ISCStatus)
        {
        case READ_BEMF:                  // Read the BEMF of phase windings during motor startup,  if the BEMF is greater than minimum BEMF , allow motor to rotate till BEMF is in limits to apply the brakes
            P4OUT |= BIT5;
            ISCReadPhaseVoltage();

            if((SensorlessTrapController.PhaseA_BEMF > SensorlessTrapController.ISCMinBEMF) ||
               (SensorlessTrapController.PhaseB_BEMF > SensorlessTrapController.ISCMinBEMF) ||
               (SensorlessTrapController.PhaseC_BEMF > SensorlessTrapController.ISCMinBEMF))
            {

                DisableGateDrivers();
                SensorlessTrapController.ISCcount = 0;
                SensorlessTrapController.ISCStatus = RUN_MOTOR;                                                               // If motor is spinning Brake the motor for a specified time after coasting the motor for 2 seconds
                SensorlessTrapController.ISCbrake = 0;
            }
            else
            {
                //P5OUT |= BIT1;
                BrakeMotor();
                SensorlessTrapController.ISCbrake = SensorlessTrapController.ISCBrakeTime;
                SensorlessTrapController.ISCStatus = BRAKE_MOTOR;                                                 // If Motor is not spinning Continue with IPD
                SensorlessTrapController.SystemCount = 0;
            }
            break;

        case RUN_IPD:                                                                   // ISCdone = 1 indicates the motor is stopped
//            P4OUT &= ~BIT1;
        if(SensorlessTrapController.Align_IPD)
        {
            SensorlessTrapController.AlignComplete = 0;
            SensorlessTrapController.StartAlign = FALSE;
            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
            ApplicationStatus.currentstate = MOTOR_ALIGN;                                                       // Move to motor Align
        }
        else
        {
            SensorlessTrapController.SystemCount = 0;
            IPD_Init();
            SensorlessTrapController.IPDStart = TRUE;
            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
            ApplicationStatus.currentstate = MOTOR_IPD;                                                         // Move to motor Initial position detection
            SensorlessTrapController.IPDState = 1;
            SensorlessTrapController.IPDStatus = START;
        }
            break;
        }
        break;

    case MOTOR_ALIGN:

        if(SensorlessTrapController.AlignComplete == TRUE)                                                              // wait for the align routine to finish
        {

            SensorlessTrapController.CurrentDutyCycle = SensorlessTrapController.StartupDutyCycle;
            SensorlessTrapController.CurrentCommState = SensorlessTrapController.AlignSector;
            UpdateNextCommutation();                                                                                                            // update the commutation sequence to ~90 degrees from the rotor
            UpdateNextCommutation();
            SetPWMDutyCycle(SensorlessTrapController.CurrentDutyCycle);                                         // set the pwm duty cycle to the open loop duty cycle
            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
            ApplicationStatus.currentstate = MOTOR_START;                                                               //change to Motor IDLE state and wait for start duty cycle
            if(SensorlessTrapController.Direction_flag)
            {
                SensorlessTrapController.Direction_flag = FALSE;
                SensorlessTrapController.Direction =
                                !SensorlessTrapController.Direction;                                                              // This flag monitors for change in direction, If direction change is applied multiple times in quick succession before speed becoming zero, this flag determines the true direction by the time motor is ramped up.                      __delay_cycles(25000000);                              /* A delay of 1s is applied before changing the direction of spin*/
                ApplicationStatus.previousstate =
                        ApplicationStatus.currentstate;
                ApplicationStatus.currentstate = MOTOR_DIRECTION;
            }
            if(HostController.Start_Stop_Motor)
            {
                ApplicationStatus.previousstate =
                       ApplicationStatus.currentstate;
                ApplicationStatus.currentstate = MOTOR_STOP;
            }

        }
        break;
    case MOTOR_IPD:

        if(SensorlessTrapController.IPDStatus == DONE)
        {
            SensorlessTrapController.IPDState = 1;
            while(SensorlessTrapController.IPDState < 7)                                        // find the minimum rise time from IPD
            {
                if(SensorlessTrapController.IPDCurrentRiseValue[
                       SensorlessTrapController.IPDState] >
                   SensorlessTrapController.IPDCurrentRiseValue[
                       SensorlessTrapController.IPDMaxCRV])
                {
                    SensorlessTrapController.IPDMaxCRV =
                        SensorlessTrapController.IPDState;                                                            // set the Maximum Current Rise value if it is detected
                    SensorlessTrapController.IPDState++;                             // skip any large rise times
                }
                else
                {
                    SensorlessTrapController.IPDState++;
                }
            }
            switch(SensorlessTrapController.IPDMaxCRV)                                          // based on the Maximum Current Rise value set the commutation state
            {
            case 1:                             //B-C
                if(SensorlessTrapController.IPDCurrentRiseValue[5] >=
                   SensorlessTrapController.IPDCurrentRiseValue[4])
                {
                    SensorlessTrapController.CurrentCommState = 0x03;                                           //drive state A-B in actual Set commutation
                }
                else
                {
                    SensorlessTrapController.CurrentCommState = 0x02;                                    //drive state A-C in actual Set commutation
                }
                break;
            case 2:                            //C-A
                if(SensorlessTrapController.IPDCurrentRiseValue[4] >=
                   SensorlessTrapController.IPDCurrentRiseValue[6])
                {
                    SensorlessTrapController.CurrentCommState = 0x01;                                    //drive state B-C in actual Set commutation
                }
                else
                {
                    SensorlessTrapController.CurrentCommState = 0x06;                                    //drive state B-A in actual Set commutation
                }
                break;
            case 3:                            //A-B
                if(SensorlessTrapController.IPDCurrentRiseValue[6] >=
                   SensorlessTrapController.IPDCurrentRiseValue[5])
                {
                    SensorlessTrapController.CurrentCommState = 0x05;                                    //drive state C-A in actual Set commutation
                }
                else
                {
                    SensorlessTrapController.CurrentCommState = 0x04;                                    //drive state C-B in actual Set commutation
                }
                break;
            case 4:                             //B-A
                if(SensorlessTrapController.IPDCurrentRiseValue[1] >=
                   SensorlessTrapController.IPDCurrentRiseValue[2])
                {
                    SensorlessTrapController.CurrentCommState = 0x02;                                    //drive state A-C in actual Set commutation
                }
                else
                {
                    SensorlessTrapController.CurrentCommState = 0x01;                                    //drive state B-C in actual Set commutation
                }
                break;
            case 5:                            //A-C
                if(SensorlessTrapController.IPDCurrentRiseValue[3] >=
                   SensorlessTrapController.IPDCurrentRiseValue[1])
                {
                    SensorlessTrapController.CurrentCommState = 0x04;                                    //drive state C-B in actual Set commutation
                }
                else
                {
                    SensorlessTrapController.CurrentCommState = 0x03;                                    //drive state A-B in actual Set commutation
                }
                break;
            case 6:                            //C-B
                if(SensorlessTrapController.IPDCurrentRiseValue[2] >=
                   SensorlessTrapController.IPDCurrentRiseValue[3])
                {
                    SensorlessTrapController.CurrentCommState = 0x06;                                    //drive state B-A in actual Set commutation
                }
                else
                {
                    SensorlessTrapController.CurrentCommState = 0x05;                                    //drive state C-A in actual Set commutation
                }
                break;
            default:
                break;
            }
            if(SensorlessTrapController.Direction == FALSE)                              // for reverse direction , start 180 degrees opposite
            {
                UpdateNextCommutation();
                UpdateNextCommutation();
                UpdateNextCommutation();
            }
            DRV8x_Digital_Init();                                                          /*Initialize MSP430F5529 peripherals */
//            if (SensorlessTrapController.DeviceID & BIT2)                                  // Reset current setting after IPD
//            {
//                SPI_Write(SPI_REG_CSA_CTRL, Reg_Map_Cache.CSA_Control_Reg6);
//            }
            BrakeMotor();
            SensorlessTrapController.CurrentDutyCycle = SensorlessTrapController.StartupDutyCycle;
            SetPWMDutyCycle(SensorlessTrapController.CurrentDutyCycle);                                                 // set the pwm duty cycle to the open loop duty cycle
            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
            ApplicationStatus.currentstate = MOTOR_START;                                                               //change to Motor Start state
            if(SensorlessTrapController.Direction_flag)
            {
                SensorlessTrapController.Direction_flag = FALSE;
                SensorlessTrapController.Direction =
                                !SensorlessTrapController.Direction;                                                              // This flag monitors for change in direction, If direction change is applied multiple times in quick succession before speed becoming zero, this flag determines the true direction by the time motor is ramped up.                      __delay_cycles(25000000);                              // A delay of 1s is applied before changing the direction of spin
                ApplicationStatus.previousstate =
                        ApplicationStatus.currentstate;
                ApplicationStatus.currentstate = MOTOR_DIRECTION;
            }
            if(HostController.Start_Stop_Motor)
            {
                ApplicationStatus.previousstate =
                       ApplicationStatus.currentstate;
                ApplicationStatus.currentstate = MOTOR_STOP;
            }
        }
        break;

    case MOTOR_START:
        if(SensorlessTrapController.AccelDone == TRUE)                                                                  // wait for a flag indicating open loop speed is reached
        {
            SensorlessTrapController.ADCchange = FALSE;
                                                                           // Disable Direction Interrupt as Motor run is Time critical algorithm , Enabling interrupts disrupts the calculation of BEMF , Instead Direction PIN is read every electrical cycle for sensing change in direction
            TIMER_SPD_Init();                                                                          // Timer Initialization to read the motor electrical speed
            SensorlessTrapController.StallDetectCounter = 0;
            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
            ApplicationStatus.currentstate = MOTOR_RUN;                                                         // Move to motor run
        }
        break;

    case MOTOR_RUN:
//        P3OUT |= BIT2;
//        if ((P2IFG & BIT3)==1)
//        {
//            HostController.EnabledGateDrivers = 0x00;
//            HostController.Start_Stop_Motor = 0x01;
//        }
        if((SensorlessTrapController.Direction_flag)||(HostController.Start_Stop_Motor))
        {
            if(SensorlessTrapController.SpeedDivider > SensorlessTrapController.RampRateDelay)
            {
                SensorlessTrapController.SpeedDivider = 0;
                if(SensorlessTrapController.CurrentDutyCycle >= (SensorlessTrapController.MinOffDutyCycle))  // Adding hysteresis for going to Motor direction state
                {
                    SensorlessTrapController.CurrentDutyCycle -= RAMP_RATE;
                }
                else
                {
                    if(SensorlessTrapController.Direction_flag)
                    {
                        SensorlessTrapController.Direction_flag = FALSE;
                        SensorlessTrapController.Direction =
                                        !SensorlessTrapController.Direction;                                                              // This flag monitors for change in direction, If direction change is applied multiple times in quick succession before speed becoming zero, this flag determines the true direction by the time motor is ramped up.                      __delay_cycles(25000000);                              // A delay of 1s is applied before changing the direction of spin
                        if(SensorlessTrapController.Direction == TRUE)
                        {
//                            P4OUT |= BIT5;                                                         // Turn  on LED1
                            P2OUT |= BIT4;                                                                                 // Turn off LED 2
                        }
                        else
                        {
//                            P4OUT &= ~BIT5;                                                        // Turn  off LED1
                            P2OUT &= ~BIT4;                                                                                  // Turn on LED 2

                        }
                        ApplicationStatus.previousstate = ApplicationStatus.currentstate;
                        ApplicationStatus.currentstate = MOTOR_STOP;                                     // move to motor initialization
                    }
                    if(HostController.Start_Stop_Motor)
                    {
                        ApplicationStatus.previousstate =
                               ApplicationStatus.currentstate;
                        ApplicationStatus.currentstate = MOTOR_STOP;
                    }
                }
            }
        }
        else if(SensorlessTrapController.SpeedChange &&
                (SensorlessTrapController.SpeedDivider > SensorlessTrapController.RampRateDelay))                                                                 // if the commanded speed is changed and wait for the accel divisor
        {
            if(SensorlessTrapController.TargetDutyCycle >
               SensorlessTrapController.CurrentDutyCycle)                                                                      // If motor is accelerating , increase duty cycle
            {
                SensorlessTrapController.CurrentDutyCycle += RAMP_RATE;
            }
            else if(SensorlessTrapController.TargetDutyCycle <
                    SensorlessTrapController.CurrentDutyCycle)                                                                      // If motor is descelerating , decrease duty cycle
            {
                SensorlessTrapController.CurrentDutyCycle -= RAMP_RATE;
            }
            else if(SensorlessTrapController.TargetDutyCycle ==
                    SensorlessTrapController.CurrentDutyCycle)                                                                       // if motor speed is constant , keep duty cycle intact
            {
                SensorlessTrapController.SpeedChange = FALSE;                                                                                                                           // Speed change is triggered in Read pot speed function
            }
            SensorlessTrapController.SpeedDivider = 0;                                                                                                                              // Speed divider counts the
        }
        break;

    case MOTOR_STOP:

        DisableGateDrivers();
        ApplicationStatus.previousstate = ApplicationStatus.currentstate;
        ApplicationStatus.currentstate = SYSTEM_IDLE;                         // move to motor initialization
        break;

    default: break;

    case FAULT:
        DisableGateDrivers();                                                     /* Disable Gate Drivers when a fault is triggered*/
        /*handles the faults and sets fault LEDS*/
        switch(ApplicationStatus.fault)
        {
        case NOFAULT:
            break;

        case VOLTAGE:
            P2OUT |= BIT5;

            break;


        case MOTOR_STALL:
            P2OUT |= BIT6;

            break;
        case OVERCURRENT:
            P3OUT |= BIT1;
            break;
        case OVERTEMPERATURE:
            P3OUT |= BIT2;

            break;
        case GATE_DRIVER:
            P3OUT |= BIT3;
            break;
        case UNKNOWN:                                                                     /* If the fault is Triggered from sources other than described above read the Fault register IC values from Fault Variables */
            P3OUT |= BIT4;

            break;

        default: break;
        }
    }
}

//void HostController_StateMachine(void)
//{
///*function
// * DRV8x_State_Machine(void)
// * Handles the state machine and transitions of the Application
// * */
// switch(HostControl_Status)
// {
//    case HOST_IDLE:
//    {
//
//        uint8_t *data,length;
//        uint8_t i,cmd;
//        HostControl_Status = HOST_IDLE;
//        if(mdbuSerial_rxStateStop)
//        {
//
//            cmd =  mdbuSerial_RxPkt.pkt_cmd;
//            data =  mdbuSerial_RxPkt.pkt_data;
//            length = mdbuSerial_RxPkt.pkt_len;
//            mdbuSerial_rxStateStop = 0;
//            for(i=0;i<MDBUSERIAL_NUM_CALLBACKS && MDBUSERIAL_NUM_CALLBACKS != 0;i++)
//            {
//                if(mdbuserial_callbacktable[i].cmd == cmd)
//                {
//                    mdbuserial_callbacktable[i].callback(data, (size_t)(length));
//                    //goto packet_clean;
//                }
//            }
//            if(HostController.EnabledGateDrivers)
//            {
//                HostControl_Status = HOST_ACTIVE;
//                drv83xx_regRestoreFromCache();                      // Default values are read by GUI after enabling the pre drivers
//            }
//        }
//    }
//
//       break;
//     case HOST_ACTIVE:
//     {
//         uint8_t *data,length;
//         uint8_t i,cmd;
//         if(mdbuSerial_rxStateStop)
//         {
//            cmd =  mdbuSerial_RxPkt.pkt_cmd;
//            data =  mdbuSerial_RxPkt.pkt_data;
//            length = mdbuSerial_RxPkt.pkt_len;
//
//            mdbuSerial_rxStateStop = 0;
//            for(i=0;i<MDBUSERIAL_NUM_CALLBACKS && MDBUSERIAL_NUM_CALLBACKS != 0;i++)
//            {
//                if(mdbuserial_callbacktable[i].cmd == cmd)
//                {
//                    mdbuserial_callbacktable[i].callback(data, (size_t)(length));
//                    //goto packet_clean;
//                }
//            }
//            if(HostController.EnabledGateDrivers == 0)
//                HostControl_Status = HOST_IDLE;
//        }
//      break;
//    }
//    }
//
//
//}

/*function
 * main()
 * Initializes the Application and calls periodically the state machine function
 * */

int main()
{

    Init_Application();                 // Initialize the State Machine
    //mdbuSerial_init();                    // Initialise MDBU Serial Protocol
    HostControllerInit();               // Initialise Host Controller

    while(1)
    {
        DRV8x_State_Machine();        // call background state machine
        //HostController_StateMachine();
    }
}
