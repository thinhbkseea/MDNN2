/*
 * ISRs.c
 *
 *  Created on: Nov 30, 2022
 *      Author: thinh
 */

/******************************************************************************/
#include "global.h"

// Controller
extern SENSORLESS_TRAP_Obj SensorlessTrapController;
extern APPLICATION_STATUS ApplicationStatus;
extern HOST_CONTROLLER_Obj HostController;
_Bool press_on_off = 0;
unsigned int press_time = 0;
_Bool on_off_state=0;
unsigned int time_count = 0;
unsigned int press_count = 0;
_Bool signal_start = 0;
_Bool signal_stop = 0;
unsigned int delay_count = 0;
unsigned int stop_time_count = 0;
_Bool up_speed_mode=0;
unsigned int t_shutdown=0;
unsigned int shutdown_count=0;
_Bool start_motor = 0;
unsigned int start_stop_delay = 0;
unsigned int stop_edge = 0;
unsigned int stop_edge_count = 0;
_Bool stop_signal = 0;
unsigned int start_tmp=0;
unsigned int pulse_delay = 0;
unsigned int port_delay = 0;
unsigned int count_tmp = 0;

// Registers
extern FLT_STAT_REG0_Obj Fault_Status_Reg;
extern VGS_STAT_REG1_Obj VGS_Status_Reg;
extern DRV_CTRL_REG2_Obj Driver_Control_Reg;
extern GATE_DRV_HS_REG3_Obj Gate_Drive_HS_Reg;
extern GATE_DRV_LS_REG4_Obj Gate_Drive_LS_Reg;
extern OCP_CTRL_REG5_Obj OCP_Control_Reg;
extern CSA_CTRL_REG6_Obj CSA_Control_Reg;

// Timer1_A0 interrupt service routine
#pragma vector=TIMER2_B0_VECTOR
__interrupt void TIMER2_B0_ISR(void)
{
    if(ApplicationStatus.currentstate == MOTOR_IPD)
    {
        if(SensorlessTrapController.IPDStatus == TIMER_INTERRUPT)
        {
            BrakeMotor();
            SensorlessTrapController.IPDStatus = ADC_READ;
            //ADCCTL0 = ADCON | ADCSHT_2;                            // Turn on ADC, avoid overflow of results , select 16 clock cycles for sampling current , 4 clk cycles for sampling BEMF
            //ADCCTL1 = ADCSHP | ADCCONSEQ_0;       // Use sampling timer, Single sequence, start conversion from memory address 0 , Select clock as SMCLK
            switch(SensorlessTrapController.IPDState)
            {
                case 1: ADCMCTL0 = ADCINCH_6;
                    break;
                case 2: ADCMCTL0 = ADCINCH_4;
                    break;
                case 3: ADCMCTL0 = ADCINCH_5;
                    break;
                case 4: ADCMCTL0 = ADCINCH_4;
                    break;
                case 5: ADCMCTL0 = ADCINCH_6;
                    break;
                case 6: ADCMCTL0 = ADCINCH_5;
                    break;
                default:
                    break;
            }
            ADCIE = ADCIE0;
            ADCCTL0 |= ADCENC;                              // Enable conversions control registers shouldn't be changes after enabling conversions
            ADCCTL0 |= ADCSC;
 //           P4OUT |= BIT1;
        }
    }
    SensorlessTrapController.TimerOverflowFlag = 1;       //This count is used to calculate speed of 18bit resolution
    SensorlessTrapController.SPDFdbk = 0x0FFFF ; // Set the speed count to maximum value that represents the zero speed

}

// Timer1_A1 interrupt service routine
#pragma vector=TIMER2_B1_VECTOR
__interrupt void TIMER2_B1_ISR(void)
{
    uint16_t interruptValue = TB2IV;
}

// Timer2_A0 interrupt service routine
#pragma vector=TIMER1_B0_VECTOR
__interrupt void TIMER1_B0_ISR(void)
{
    uint16_t interruptValue = TB1IV;
    if((port_delay==1)&&(press_count<3))
    {

        pulse_delay++;
        if (pulse_delay >= 700)
        {
            if ((P2IN & BIT1) == 0)
            {
                press_count++;
                port_delay = 0;
            }
            else
            {
                pulse_delay = 0;
                port_delay = 0;
            }
        }

    }
    switch (press_count)
    {
    case 0:
    {
        if (up_speed_mode == 1) //dang trong che do tang toc, doi cho tang toc xong roi tang tiep
        {
            delay_count++;
            if (delay_count >= 24000) //10000
            {
                up_speed_mode = 0;
            }
        }
    }
        break;
    case 1:
    {
        stop_time_count = 0;

        time_count++;
        if (time_count >= 24000) //40us*40000
        {
            if ((signal_start == 1) && (up_speed_mode == 0))
            {
                //reduce speed
                if ((SensorlessTrapController.SetSpeed - 100) >= 500)
                {
                    SensorlessTrapController.SetSpeed -= 100;
                }

                if ((SensorlessTrapController.SetSpeed -100) <= 500)
                {
                    SensorlessTrapController.SetSpeed = 500;
                }
            }

            time_count = 0;
            press_count = 0;
            P3OUT |= BIT3;
            P3OUT &= ~BIT2;
//            up_speed_mode = 1;
        }
    }
        break;
    case 2:
    {
        stop_time_count = 0;

//        if (signal_stop == 0)
//        {
//            if (signal_start == 0)
//            {
//                signal_start = 1;
//                //start
//                SensorlessTrapController.SetSpeed = 500; //600
//                HostController.EnabledGateDrivers = 0x01;
//                HostController.Start_Stop_Motor = 0x00;
//            }
//            else
//            {
//                //up speed
//                if (SensorlessTrapController.SetSpeed <= (SensorlessTrapController.MaxDutyCycle - 100))
//                {
//                    SensorlessTrapController.SetSpeed += 100;
//                }
//                if ((SensorlessTrapController.SetSpeed + 100)>= SensorlessTrapController.MaxDutyCycle)
//                {
//                    SensorlessTrapController.SetSpeed =
//                            SensorlessTrapController.MaxDutyCycle;
//                }
//
//            }
//
//        }
        if (up_speed_mode == 1) //dang trong che do tang toc, doi cho tang toc xong roi tang tiep
        {
            delay_count++;
            if (delay_count >= 24000) //10000
            {
                up_speed_mode = 0;
            }
        }
        if (signal_start == 0)
        {
            signal_start = 1;
            //start
            SensorlessTrapController.SetSpeed = 700; //600 700
            HostController.EnabledGateDrivers = 0x01;
            HostController.Start_Stop_Motor = 0x00;
        }
        if ( ApplicationStatus.currentstate == MOTOR_RUN && up_speed_mode == 0)
        {
            //up speed
            if (SensorlessTrapController.SetSpeed <= 810)//(SensorlessTrapController.MaxDutyCycle - 100))
            {
                SensorlessTrapController.SetSpeed += 100;
            }
//            if (SensorlessTrapController.SetSpeed > 900)
//                SensorlessTrapController.SetSpeed = 900;
//            if ((SensorlessTrapController.SetSpeed + 100)>= SensorlessTrapController.MaxDutyCycle)
//            {
//                SensorlessTrapController.SetSpeed =
//                        SensorlessTrapController.MaxDutyCycle;
//            }

        }

        press_count = 0;
        time_count = 0;
        up_speed_mode = 1;
        delay_count = 0;
        P3OUT |= BIT2 | BIT3;
    }
        break;
    default:
    {
        press_count = 0;
        time_count = 0;
    }
        break;
    }

    if (signal_stop == 1)
    {
        t_shutdown++;
        if (t_shutdown >= COUNTER_2_SECONDS)
        {
            shutdown_count++;
            if(shutdown_count >=3)
            {
                signal_stop = 0;
            }

            t_shutdown = 0;
        }

    }
//    if (press_on_off == 1)
//    {
//        press_time++;
//        if (press_time >= 10000)
//        {
//            on_off_state ^= 1;
//            if (on_off_state == 1)
//            {
//                //P1OUT |= BIT6;
//                HostController.EnabledGateDrivers = 0x01;   //enable gate driver
//                //HostController.Start_Stop_Motor = 0x00; //start motor
//                start_motor = 0x01;
//            }
//            else
//            {
//                HostController.Start_Stop_Motor = 0x01;
//                HostController.EnabledGateDrivers = 0x00;
//
//                P2OUT &= ~BIT0;
//
//            }
//            press_time = 0;
//            press_on_off = 0;
//        }
//
//    }
//    if (start_motor == 1)
//    {
//        start_stop_delay++;
//        if (start_stop_delay >= 40000)
//        {
//            HostController.Start_Stop_Motor = 0x00; //start motor
//            start_motor = 0;
//            start_stop_delay = 0;
//        }
//    }

        if (((P2IN & BIT2) == 0)||((P2IN & BIT1)==1))//&&(ApplicationStatus.currentstate == MOTOR_RUN))
//        if (stop_signal==1)
        {
            stop_time_count++;
            if (stop_time_count >= 24000)
            {

                signal_start = 0;
    //            signal_stop = 1;
                HostController.EnabledGateDrivers = 0x00;
                HostController.Start_Stop_Motor = 0x01;
                P2OUT &= ~BIT0;
                stop_time_count = 0;
                // stop_count = 0;
                press_count = 0;
                time_count = 0;
                P3OUT |= BIT2;
                P3OUT &= ~BIT3;
            }
        }

//  if (stop_count >= 1)

//    t_shutdown++;
//    if(t_shutdown >= COUNTER_2_SECONDS)
//    {
//        shutdown_count++;
//
//        if (shutdown_count>=30)
//        {
//            HostController.EnabledGateDrivers = 0x00;
//            HostController.Start_Stop_Motor = 0x01;
//            shutdown_count=0;
//        }
//        t_shutdown=0;
//    }

//    if ((P2IN & BIT3) == 0)
//    {
//        signal_start = 0;
//        HostController.EnabledGateDrivers = 0x00;
//        HostController.Start_Stop_Motor = 0x01;
//        P2OUT &= ~BIT0;
//        press_count = 0;
//        time_count = 0;
//    }
}

/* Timer2_A1 interrupt service routine
   This is for CCR1 Compare match
   Used for sampling the BEMF ADC
 */
#pragma vector=TIMER3_B1_VECTOR
__interrupt void TIMER3_B1_ISR(void)
{


}

/* Timer0_B0 interrupt service routine
   ISR to check for stall fault and handle fault recovery timeout
 */
#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
{
    if(ApplicationStatus.currentstate == MOTOR_RUN)     // If the motor is in run state where pulses are given and still if motor is not spinning , then stall fault is detected
    {

        SensorlessTrapController.StallDetectCounter++;          // Stall detect counter increases for every PWM Interrupt
        if(SensorlessTrapController.StallDetectCounter >=
                    SensorlessTrapController.Counter_1M_Second) // Stall Delay incremented for every 1 milli second
        {
            SensorlessTrapController.StallDetectCounter = 0;
            SensorlessTrapController.StallDetectDelay++;
            if(SensorlessTrapController.StallDetectDelay >=
                SensorlessTrapController.StallDetecttime)
            {
                SensorlessTrapController.StallDetectDelay = 0;
                /* check for STall fault*/

                if(SensorlessTrapController.RotationCount <= SensorlessTrapController.StallDetectRev)         // < 1 rpm --> stall
                {

                    ApplicationStatus.previousstate = ApplicationStatus.currentstate;
                    ApplicationStatus.currentstate = FAULT;
                    ApplicationStatus.fault = MOTOR_STALL;
                }
                SensorlessTrapController.RotationCount = 0;
            }
        }
    }

    /*In fault case wait for Fault recovery time and restart software with initialization*/
    if(ApplicationStatus.currentstate == FAULT)
    {
        SensorlessTrapController.RestartDelayCounter++;                       // Every PWM Interrupt the Fault recovery delay counter increases
        SensorlessTrapController.StallDetectCounter = 0;                      // When a Fault is detected , Stall fault is ignored
        SensorlessTrapController.StallDetectDelay = 0;
        if (SensorlessTrapController.RestartDelayCounter > SensorlessTrapController.Counter_1M_Second)
        {
            SensorlessTrapController.RestartDelay++;                        // Once Delay counter counts 1 milli second increase the restart Delay
            SensorlessTrapController.RestartDelayCounter = 0;
            if(SensorlessTrapController.RestartDelay >= SensorlessTrapController.AutoFaultRecoveryTime)
            {
                SensorlessTrapController.RestartDelay = 0;
//                P4OUT &= ~BIT5;                              /* OFF LED0 */
                P3OUT &= ~BIT4;                              /* OFF LED1 */
//                if(SensorlessTrapController.DeviceID & BIT0)                  // Initialize the SPI variables and settings only if the device is "S" Variant
//                {
//                    SPI_Write(SPI_REG_DRV_CTRL,
//                          (SPI_Read(SPI_REG_DRV_CTRL) | CLR_FLT_MASK));                           //Try to Clear the faults and warnings
//                }

                if((P4IN & BIT1) != 0)                                           // Check whether If Fault still persist i.e if p2.7 is low
                {
                    ApplicationStatus.previousstate =
                        ApplicationStatus.currentstate;
                    ApplicationStatus.currentstate = SYSTEM_INIT;
                    ApplicationStatus.fault = NOFAULT;                                              /* restart need to NOFAULT */
                }
            }
        }
    }
}

/* Timer0_A0 interrupt service routine
   This is for only CCRO. This interrupt occurs every PWM period
   Used for time sensitive control in the state machine
 */
#pragma vector=TIMER3_B0_VECTOR
__interrupt void TIMER3_B0_ISR(void)
{
    SensorlessTrapController.SystemCount++;
    switch(ApplicationStatus.currentstate)
    {
    case MOTOR_ALIGN:
        if(SensorlessTrapController.StartAlign == FALSE)
        {
            if(SensorlessTrapController.CurrentDutyCycle < SensorlessTrapController.StartupDutyCycle)//SensorlessTrapController.StartupDutyCycle) //150
            {             //continue to increase the duty cycle to the specified value

                SensorlessTrapController.CurrentDutyCycle++;
                SetPWMDutyCycle(SensorlessTrapController.CurrentDutyCycle);

            }
            if(SensorlessTrapController.AlignWaitCounter == 0)
            {             //set the commutation sequence once
                PWM_SetCommutation(SensorlessTrapController.AlignSector);
            }
        }
        SensorlessTrapController.AlignWaitCounter++;
        if(SensorlessTrapController.AlignWaitCounter >=  SensorlessTrapController.AlignWaitTime)
        {         //after driving the align sector for align time stop
            SensorlessTrapController.CurrentDutyCycle = 0;

            DisableGateDrivers();
            BrakeMotor();
            SensorlessTrapController.AlignWaitCounter = 0;
            if(SensorlessTrapController.StartAlign)
            {             //after waiting another align wait time signal align complete
                SensorlessTrapController.AlignComplete = TRUE;
                SensorlessTrapController.AlignWaitCounter = 0;
            }
            SensorlessTrapController.StartAlign = TRUE;
        }
        break;
    case MOTOR_ISC:
        SensorlessTrapController.ISCcount++;
        if(SensorlessTrapController.ISCStatus == RUN_MOTOR)                        // motor is allowed to spin till BEMF is in control by coasting the motor for 2 seconds
        {
            if(SensorlessTrapController.ISCcount > COUNTER_2_SECONDS)
            {
                if(SensorlessTrapController.ISCbrake == 0)
                {
                    SensorlessTrapController.ISCStatus = BRAKE_MOTOR;                                     // go to Brake motor
                }
                else
                {
                    SensorlessTrapController.ISCbrake--;
                    SensorlessTrapController.ISCcount = 0;
                }
            }
        }
        else if(SensorlessTrapController.ISCStatus == BRAKE_MOTOR)
        {
            if(SensorlessTrapController.ISCcount > COUNTER_1_MSECOND)
            {                                                                                                                                                           //200 millisecond timeout 5000 * 41us
                if(SensorlessTrapController.ISCbrake == 0)
                {
                    SensorlessTrapController.ISCStatus = RUN_IPD;                                     //go to IPD
                }
                else
                {                                 //continues to brake for 1ms * ISCbrake value
                    SensorlessTrapController.ISCbrake--;
                    SensorlessTrapController.ISCcount = 0;
                }
            }
        }

        break;
    case MOTOR_IPD:
        if(SensorlessTrapController.IPDStatus == START)
        {
            IPD_SetState(SensorlessTrapController.IPDState);
            TB2R = 0x0000;                                                                                                      /* reset TA1 counter , which sets no of clock cycles a phase is switched on*/
            TB2CCTL0 |= CCIE;
            SensorlessTrapController.IPDStatus = TIMER_INTERRUPT;
        }
        if(SensorlessTrapController.IPDStatus == BRAKE)
        {
            SensorlessTrapController.IPDCount++;
            SensorlessTrapController.IPDCoastTime =
                SensorlessTrapController.IPDBrakeTime * SensorlessTrapController.IPDDecayConstant;
            if(SensorlessTrapController.IPDCount >=
               SensorlessTrapController.IPDBrakeTime)
            {
                DisableGateDrivers();                                  // Turn Off all switches during Coast time
                if(SensorlessTrapController.IPDCount >=
                   SensorlessTrapController.IPDCoastTime)
                {
                    SensorlessTrapController.IPDState++;
                    SensorlessTrapController.IPDCount = 0;
                    SensorlessTrapController.IPDStatus = START;
                }
            }
            if(SensorlessTrapController.IPDState >= 7)
            {
                SensorlessTrapController.IPDStatus = DONE;
                ADCCTL0 &= ~ADCENC;                                          // Disable the ADC conversions after completion of IPD
                ADCCTL0 &= ~ADCSC;
                ADCIE = 0x00;
            }
        }
        break;

    case MOTOR_START:

        SensorlessTrapController.AccelCounter++;
        if((SensorlessTrapController.AccelCounter == SensorlessTrapController.Counter_1M_Second)&&
           (SensorlessTrapController.AccelDone == FALSE))                                                                             //1ms
        {

            SensorlessTrapController.AccelCounter = 0;                                          //reset  period counter
            SensorlessTrapController.AccelVelocityInit += SensorlessTrapController.AccelRate;                     //increase velocity by the acceleration rate
            SensorlessTrapController.AccelDistance +=
                (SensorlessTrapController.AccelVelocityInit - (SensorlessTrapController.AccelRate >> 1)) >>
                3;                                                                                                                       //calculate distance

            if(SensorlessTrapController.CurrentDutyCycle < 700)//350 SensorlessTrapController.SetSpeed
            {
                start_tmp ++;
                if (start_tmp == 2)
                {
                    SensorlessTrapController.CurrentDutyCycle ++;
                    start_tmp=0;
                }

            }



            if(SensorlessTrapController.AccelDistance > ACCEL_60_DEGREES)
            {                     //if distance is 60 degrees commutate
                count_tmp ++;
//                if(SensorlessTrapController.CurrentDutyCycle > 500)
//                {
//                    //P3OUT |= BIT1;
//                    SensorlessTrapController.SetSpeed = SensorlessTrapController.CurrentDutyCycle;
//                }
                if (count_tmp > 40)
                    P3OUT |= BIT1;
                if (count_tmp > 120)
                    P3OUT &= ~BIT1;

                SensorlessTrapController.AccelDistance = 0;
                UpdateNextCommutation();
                PWM_SetCommutation(SensorlessTrapController.CurrentCommState);

                SetPWMDutyCycle(SensorlessTrapController.CurrentDutyCycle);
                SetMotorSpeed();                                                                                                                                                                                                                            //this enables the motor to stop during open loop
//                P5OUT |= BIT1;
                if(SensorlessTrapController.AccelVelocityInit > SensorlessTrapController.AccelStop)
                {                                       //if our open loop speed reaches threshold switch to closed loop

                    UpdateBEMFADC();
                    SensorlessTrapController.AccelDone = TRUE;
                    P3OUT &= ~BIT1;
                    //P3OUT |= BIT1;
                    count_tmp = 0;
                }
            }
        }
        break;

    case MOTOR_RUN:
        SensorlessTrapController.ADCcnt++;
        SensorlessTrapController.SpeedDivider++;
        break;

    default:
        break;
    }
}


#pragma vector = PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    //dtbui

//    if (P2IFG & BIT2)         //NC
//        {
//            stop_signal=1;
//            P2IFG &= ~BIT2;
//        }
    if(P2IFG & BIT1)        //NO
       {
////           stop_signal=0;
//           if(press_count==0)
//           {
//
//               if(up_speed_mode==0)
//               {
//                   press_count=1;
//               }
//               else
//               {
//                   press_count=0;
//               }
//
//
//           }
//           if(press_count==1)
//           {
//               if(time_count>=10000)
//               {
//                   press_count = 2;
//
//               }
//               else press_count=1;
//           }

           port_delay=1;
           P2IFG &= ~BIT1;
       }

    if (P2IFG & BIT3)           //start stop with POT input
    {
        HostController.EnabledGateDrivers = 0x00;
        HostController.Start_Stop_Motor = 0x01;
        //P3OUT |= BIT4;
//        if(press_on_off==0)
//        {
//           press_on_off=1;
//        }
//
////        press_on_off ^= 1;
////
////        if (press_on_off == 1)
////        {
////            //P1OUT |= BIT6;
////            HostController.EnabledGateDrivers = 0x01;   //enable gate driver
////            HostController.Start_Stop_Motor = 0x00; //start motor
////        }
////        else
////        {
////
////            HostController.EnabledGateDrivers = 0x00;
////            HostController.Start_Stop_Motor = 0x01;
////            P2OUT &= ~BIT0;
////
////        }
        P2IFG &= ~BIT3;
    }
}

// Timer0_A1 interrupt service routine
#pragma vector=TIMER1_B1_VECTOR
__interrupt void TIMER1_B1_ISR(void)
{

}

/* Timer0_B0 interrupt service routine
   This is for only CCR1.      // This interrupt is to monitor the BEMF of the floating phase voltages just before the PWM falling edge so that BEMF will get settled after voltage dynamics
 */
#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void)
{
    if(ApplicationStatus.currentstate == MOTOR_RUN)
    {     //verify the state machine is in closed loop control
        if(SensorlessTrapController.ADCchange == TRUE)                                                                  // if the commutation sequence just changed (during blanking time)
        {
            switch(SensorlessTrapController.CurrentCommState)                             // Monitor either Vcc, Phase current , or Speed Input during Commutation Blank Time
            {
            case 1: SetMotorSpeed();                                            // Reads Change in speed input
                break;
            case 2: ReadVCC();                                                          // Reads supply voltage value
                break;
            case 3: SetMotorSpeed();                                            // Reads Change in speed input
                break;
            case 4: ReadVCC();                                                          // Reads supply voltage value
                break;
            default:
                break;
            }
            SensorlessTrapController.ADCchange = FALSE;

        }
        UpdateBEMFADC();                                                                                                // set the ADC MUX for the floating phase
        if(SensorlessTrapController.ADCcnt > SensorlessTrapController.CommutationBlankTime)                                                                                   // if the blanking time is over and the ADC has a new value
        {
            FastReadBEMF();

            SensorlessTrapController.CommStateDetect =
                SensorlessTrapController.CurrentCommState & 0x01;                                                                    //detect if the commutation state is even or odd

            if(SensorlessTrapController.Direction == FALSE)
            {
                //BEMF Decreasing && BEMF>CT

                if((SensorlessTrapController.CommStateDetect) &&
                   SensorlessTrapController.GetBEMF <=
                   SensorlessTrapController.CTvoltage)
                {


                    SensorlessTrapController.SumBEMF +=
                        SensorlessTrapController.CTvoltage -
                        SensorlessTrapController.GetBEMF;
                }
                //BEMF Increasing && CT>BEMF
                else if((SensorlessTrapController.CommStateDetect == 0) &&
                        SensorlessTrapController.GetBEMF >=
                        SensorlessTrapController.CTvoltage)
                {
;
//                    t_com++;
//                    if(ADCMCTL0 == ADCINCH_2)
//                        P4OUT |= BIT5;

                    SensorlessTrapController.SumBEMF +=
                        SensorlessTrapController.GetBEMF -
                        SensorlessTrapController.CTvoltage;
                }
            }
            else
            {
                //BEMF Decreasing && BEMF>CT

                if((SensorlessTrapController.CommStateDetect == 0) &&
                   SensorlessTrapController.GetBEMF <=
                   SensorlessTrapController.CTvoltage)
                {
                    SensorlessTrapController.SumBEMF +=
                        SensorlessTrapController.CTvoltage -
                        SensorlessTrapController.GetBEMF;
                    P4OUT |= BIT7;
                }
                //BEMF Increasing && CT>BEMF
                else if((SensorlessTrapController.CommStateDetect) &&
                        SensorlessTrapController.GetBEMF >=
                        SensorlessTrapController.CTvoltage)
                {
                    SensorlessTrapController.SumBEMF +=
                        SensorlessTrapController.GetBEMF -
                        SensorlessTrapController.CTvoltage;

                }
            }
//            if(SensorlessTrapController.SumBEMF >500)
//                P4OUT |= BIT7;
//            if(SensorlessTrapController.SumBEMF >300)
//                P4OUT |= BIT5;
//            if (t_com > 10)
//                P4OUT |= BIT7;
            if(SensorlessTrapController.SumBEMF >=  SensorlessTrapController.BEMFThreshold)                            //compare the sum to the commutation threshold
            {
                SensorlessTrapController.SumBEMF = 0;
                SensorlessTrapController.ADCcnt = 0;
                UpdateNextCommutation();
                PWM_SetCommutation(SensorlessTrapController.CurrentCommState);
                SetPWMDutyCycle(SensorlessTrapController.CurrentDutyCycle);
                SensorlessTrapController.ADCchange = TRUE;
//               if (close_loop_cycle<120) close_loop_cycle++;
            }
            if((SensorlessTrapController.CurrentCommState == 5) || (SensorlessTrapController.CurrentCommState ==6))
            {
                ReadCurrentShunt();                             //Reads CSA value and triggers OC faults for Motor current greater than Set Limit , phase A current is monitored
            }
        }
    }
    TB0CCTL1 &= ~CCIFG;
}

#pragma vector = WDT_VECTOR
__interrupt void WDT_ISR(void)
{
}

//#pragma vector = DMA_VECTOR
//__interrupt void DMA_ISR(void)
//{
//}

/* ISR for port 2 Interrupts
 * check  for Hall sensor-2 , Hall sensor-3 Interrupt
 * */
#pragma vector = PORT4_VECTOR
__interrupt void PORT4_ISR(void)
{
    if(P4IFG & BIT1)                                         // If Fault pin from P3.4 <==> P2.7  Interrupt is triggered , Take corresponding action by Reading fault status from SPI
    {
//        if(SensorlessTrapController.DeviceID & BIT0)                  // Initialize the SPI variables and settings only if the device is "S" Variant
//        {
//            unsigned int faultStatus = SPI_Read(SPI_REG_FAULT_STAT); // Check fault register;
//
//            if(faultStatus & FAULT_MASK)                                          // If it is Fault
//            {
//                if(faultStatus & UVLO_MASK )
//                {
//                    ApplicationStatus.previousstate =
//                    ApplicationStatus.currentstate;
//                    ApplicationStatus.currentstate = FAULT;
//                    ApplicationStatus.fault = VOLTAGE;
//                }
//                else if(faultStatus & VDS_OCP_MASK)
//                {
//                    ApplicationStatus.previousstate =
//                        ApplicationStatus.currentstate;
//                    ApplicationStatus.currentstate = FAULT;
//                    ApplicationStatus.fault = OVERCURRENT;
//                }
//                else if(faultStatus & OTSD_MASK)
//                {
//                    ApplicationStatus.previousstate =
//                        ApplicationStatus.currentstate;
//                    ApplicationStatus.currentstate = FAULT;
//                    ApplicationStatus.fault = OVERTEMPERATURE;
//                }
//                else if(faultStatus & GDF_MASK)
//                {
//                    ApplicationStatus.previousstate =
//                        ApplicationStatus.currentstate;
//                    ApplicationStatus.currentstate = FAULT;
//                    ApplicationStatus.fault = GATE_DRIVER;
//                }
//                else
//                {
//                    ApplicationStatus.previousstate =
//                        ApplicationStatus.currentstate;
//                    ApplicationStatus.currentstate = FAULT;
//                    ApplicationStatus.fault = UNKNOWN;
//                }
//            }
//            else                      // If the Fault pin is toggling because of warnings then clear the warnings
//            {
//                if(ApplicationStatus.currentstate != FAULT)
//                {
//                    SPI_Write(SPI_REG_DRV_CTRL,
//                              (SPI_Read(SPI_REG_DRV_CTRL) | CLR_FLT_MASK));                       //If device is not in fault status clear and Ignore any warnings arise
//                }
//            }
//            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
//            ApplicationStatus.currentstate = FAULT;
//            ApplicationStatus.fault = UNKNOWN;
//        }
//        else
        {

            ApplicationStatus.previousstate = ApplicationStatus.currentstate;
            ApplicationStatus.currentstate = FAULT;
            ApplicationStatus.fault = UNKNOWN;
        }
        P4IFG &= ~BIT1;
    }
//    if(P2IF & BIT1)
//    {
//        signal_start=0;
//        press_count=0;
//        stop_count=0;
//        HostController.EnabledGateDrivers = 0x00;
//        HostController.Start_Stop_Motor=0x01;
//
//        P2IFG &= ~BIT1;
//    }

}

#pragma vector = ADC_VECTOR
__interrupt void ADC_ISR(void)
{
    if(ApplicationStatus.currentstate == MOTOR_IPD)
    {
        if(SensorlessTrapController.IPDStatus == ADC_READ)            // wait till Timer reaches certain limit
        {
            switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
            {
                case ADCIV_NONE:
                    break;
                case ADCIV_ADCOVIFG:
                    break;
                case ADCIV_ADCTOVIFG:
                    break;
                case ADCIV_ADCHIIFG:
                    break;
                case ADCIV_ADCLOIFG:
                    break;
                case ADCIV_ADCINIFG:
                    break;
                case ADCIV_ADCIFG:
                    SensorlessTrapController.IPDCurrentRiseValue[SensorlessTrapController.IPDState] = ADCMEM0 & 0x0FFF;
                    __bic_SR_register_on_exit(LPM0_bits);            // Clear CPUOFF bit from LPM0
                    break;
                default:
                    break;
            }
            ADCCTL0 &= ~ADCENC;
            ADCCTL0 &= ~ADCSC;                                 // Make SC bit low to make SHI low
//            P4OUT &= ~BIT1;
            if(SensorlessTrapController.DeviceID & BIT2)
            {
                SensorlessTrapController.IPDCurrentRiseValue[
                    SensorlessTrapController.IPDState] -= 2048;
            }
            SensorlessTrapController.IPDCurrentRiseValue[
                SensorlessTrapController.IPDState] = abs(
                SensorlessTrapController.IPDCurrentRiseValue[
                    SensorlessTrapController.IPDState]);
            if(SensorlessTrapController.IPDCurrentRiseValue[
                   SensorlessTrapController.IPDState] >
               SensorlessTrapController.MotorPhaseCurrentLimit)                                                                                               /* Motor Phase Current Limit*/
            {
                ApplicationStatus.previousstate =
                    ApplicationStatus.currentstate;
                ApplicationStatus.currentstate = FAULT;
                ApplicationStatus.fault = OVERCURRENT;
            }
            SensorlessTrapController.IPDCount = 0;            //clear the time count
            SensorlessTrapController.IPDStatus = BRAKE;
        }
    }
}

#pragma vector = RTC_VECTOR
__interrupt void RTC_ISR(void)
{
}


//#pragma vector = USCI_A0_VECTOR
//__interrupt void USCIA_ISR(void)
//{
//}
//
//#pragma vector = USCI_B0_VECTOR
//__interrupt void USCIB_ISR(void)
//{
//}
/* ISR to handle UART interrupts from GUI */
//#pragma vector = USCI_A1_VECTOR
//__interrupt void USCIA1_ISR(void)
//{
//    switch ( UCA1IV )
//      {
//        case 2:
//            /*
//             * Handle RX interrupt
//             */
////            #ifndef MDBUSERIAL_USE_USB
////                mdbuSerial_handleRX();
////            #endif
//            break;
//       case 4:
//            /*
//             * Handle TX interrupt
//             */
////            #ifndef MDBUSERIAL_USE_USB
////                mdbuSerial_handleTX();
////            #endif
//            break;
//      }
//}

#pragma vector = USCI_B1_VECTOR
__interrupt void USCIB1_ISR(void)
{
}

#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR(void)
{
}

#pragma vector = SYSNMI_VECTOR
__interrupt void SYSNMI_ISR(void)
{
}

//#pragma vector = COMP_B_VECTOR
//__interrupt void COMP_B_ISR(void)
//{
//}




