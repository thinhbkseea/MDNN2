
/*
 * init.c
 *
 *  Created on: Nov 30, 2022
 *      Author: thinh
 */



#ifndef INIT_H_
#include <Init.h>
#endif

#include "global.h"

// Controller
extern SENSORLESS_TRAP_Obj SensorlessTrapController;
extern APPLICATION_STATUS ApplicationStatus;
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

/*function
 * Init_Application()
 * initializes the application structure
 * */
void Init_Application()
{

    // Variables Initialisation for MDBU serial communication
    /*This variable is used to measure the speed in timer counts of 16 bit resolution, Timer counts is inversely proportional to speed i.e. Maximum speed count represents minimum speed in frequency, 16 bit max value = 0xFFFFh */
    SensorlessTrapController.SPDFdbk = 0x0FFFF ;
    SensorlessTrapController.ISCMinBEMF = ISC_MIN_BEMF;     /* This variable holds the ISC_MIN_BEMF to set the min BEMF value above which ISC routine is executed */
    SensorlessTrapController.ISCBrakeTime = ISC_BRAKE_TIME;     /* This variable holds the Motor parameter  (4) ISC brake time to set the amount of time to brake the motor during ISC routine */
    SensorlessTrapController.IPDBrakeTime = IPD_ADD_BRAKE;     /* This variable holds the Motor parameter  (5) ISC brake time to set the amount of time to brake the motor during ISC routine */
    SensorlessTrapController.IPDPulseTime = IPD_PULSE_TIME;     /* This variable holds the Motor parameter  (6) ISC brake time to set the amount of time to brake the motor during ISC routine */
    SensorlessTrapController.IPDDecayConstant = IPD_DECAY_CONSTANT;     /* This variable holds the Motor parameter  (7) ISC brake time to set the amount of time to brake the motor during ISC routine */
    SensorlessTrapController.AlignSector = ALIGN_SECTOR;         /* this variable holds the Motor parameter (8) Align sector to which the rotor is aligned to at the motor start up*/
    SensorlessTrapController.AlignWaitTime = ALIGN_WAIT_TIME;    /* This variable holds the Motor parameter (9) Align wait time to set the amount of time for which voltage pulses are given during Aligning the rotor at start up */
    SensorlessTrapController.AccelRate = ACCEL_RATE;        /* This variable holds the Motor parameter (10) Accel rate defines the Open loop Blind  acceleration rate */
    SensorlessTrapController.AccelStop = ACCEL_STOP;        /* This variable holds the Motor parameter (11) Accel stop defines the open loop to closed loop hand off velocity */
    SensorlessTrapController.AccelVelocity =  ACCEL_VELOCITY_INIT;  ;    /* This variable holds the Motor parameter (12) Accel velocity init defines the open loop initial velocity */
    SensorlessTrapController.BEMFThreshold = BEMF_THRESHOLD; /*This variable holds the Motor parameter (13) BEMF_threshold to set the BEMF integration threshold value */
    SensorlessTrapController.RampRateDelay = RAMP_RATE_DELAY;    /* This variable holds the Motor parameter (14) Ramp rate delay to set the acceleration/ desceleration of the motor */
    SensorlessTrapController.CommutationBlankTime =  COMMUTATION_BLANK_TIME;  ;    /* This variable holds the Motor parameter (16) Accel velocity init defines the open loop initial velocity */
    SensorlessTrapController.PWMBlankCounts = PWM_BLANK_COUNTS;   /* This Variable holds the Motor parameter (17) to set the number of PWM cycles to after which BEMF is sampled after a commutation is taken place */
    SensorlessTrapController.MaxDutyCycle = MAX_DUTY_CYCLE;     /* This Variable holds the Motor parameter (18) to set the PWM Maximum duty cycle */
    SensorlessTrapController.MinOffDutyCycle = MIN_OFF_DUTY;  /* This Variable holds the Motor parameter (19) to set the PWM Minimum off duty cycle to spin the motor */
    SensorlessTrapController.MinOnDutyCycle = MIN_ON_DUTY;   /* This Variable holds the Motor parameter (20) to set the PWM Minimum on duty cycle to start the motor */
    SensorlessTrapController.StartupDutyCycle = START_UP_DUTY_CYCLE; /* This Variable holds the Motor parameter (21) to set the PWM duty cycle at strtup*/
    SensorlessTrapController.UnderVolLim = UNDER_VOLTAGE_LIMIT;      /* This Variable holds the Motor parameter (23) to set the supply undervoltage limit */
    SensorlessTrapController.OverVolLim = OVER_VOLTAGE_LIMIT;       /* This Variable holds the Motor parameter  (24) to set the supply overvoltage limit */
    SensorlessTrapController.StallDetectRev = STALLDETECT_REV_THRESHOLD ;  /* This Variable holds the Motor parameter  (25) to set the minimum revolutions to detect for a stall fault */

    SensorlessTrapController.StallDetecttime = STALLDETECT_TIMER_THRESHOLD;     /* This Variable holds the Motor parameter  (26) to set the time after which stall fault is triggered */
    SensorlessTrapController.MotorPhaseCurrentLimit = MOTOR_PHASE_CURRENT_LIMIT; /* This Variable holds the Motor parameter (27) to set the motor Phase current limit */
    SensorlessTrapController.AutoFaultRecoveryTime = AUTO_FAULT_RECOVERY_TIME;/* This Variable holds the Motor parameter (28) to set the time limit after which faults are automatically recovered */
    SensorlessTrapController.Align_IPD = ALIGN_OR_IPD;        /* This Variable holds the Motor parameter (28) to set the time limit after which faults are automatically recovered */
    SensorlessTrapController.PWMPeriod = PWM_PERIOD;        /* This Variable holds the Motor parameter (22) to set the PWM switching frequency as 25Mhz / PWM_PERIOD */
    SensorlessTrapController.Counter_1M_Second = (24000 / SensorlessTrapController.PWMPeriod);
    SensorlessTrapController.Counter_1_Second = SensorlessTrapController.Counter_1M_Second * 1000;
    SensorlessTrapController.PWM_Mode = 0;//6x PWM Mode
    ApplicationStatus.currentstate = SYSTEM_INIT;
    ApplicationStatus.previousstate = SYSTEM_INIT;
    ApplicationStatus.fault = NOFAULT;
    /* Configure Port 3.7 & Port 8.2 as Inputs to Identify the Device connected */
//    P3DIR &= ~BIT7; //input
//    P3REN &= ~BIT7;                      // When a GPIO pin is configured as Input,
//    P8DIR &= ~BIT2;
//    P8REN &= ~BIT2;                             // When a GPIO pin is configured as Input,
//    SensorlessTrapController.DeviceID = ALGO_ID;  // Device ID for DRV8323s Sensorless , // read the GPIO pins to know the actual device
//    SensorlessTrapController.DeviceID = ((SensorlessTrapController.DeviceID<<1) + ((0xFF & BIT7)>>7));
//    SensorlessTrapController.DeviceID = ((SensorlessTrapController.DeviceID<<1) + ((0 & BIT2)>>2));
    SensorlessTrapController.DeviceID = 0x02;   //BIT0 - SPI variant, BIT1 - sensorless, BIT2 - DRV8323x
    /* Initialize MDBU Serial physical layer */
//    #ifdef MDBUSERIAL_USE_USB //dtbui
//      USB_setup(TRUE,TRUE);       /* MDBU Serial Protocol over USB */
//    #else
//      UART_Init();                /* MDBU Serial Protocol over UART */
//    #endif
   /*Disable the gate drivers by Default */
   P2DIR |= BIT0;
   P2OUT &= ~BIT0;


    /* Analog initialization */
//    DRV8x_Analog_Init();
}

/*function
 * HostControllerInit()
 * Initializes thevariables used in Host controller state machine
 */
void HostControllerInit(void)
{
    HostController.EnabledGateDrivers = 0x00;
    HostController.Start_Stop_Motor = 0x01;
}
/*function
 * Init_SensorlessTrapController()
 * initializes the motor structure
 * */
void sensorlessTrapController_Init()
{
//IPD Variables
    SensorlessTrapController.IPDCount = 0x00;
    SensorlessTrapController.IPDDone = FALSE;
    SensorlessTrapController.IPDMaxCRV = 0;
    SensorlessTrapController.IPDCurrentRiseValue[1] = 0;
    SensorlessTrapController.IPDCurrentRiseValue[2] = 0;
    SensorlessTrapController.IPDCurrentRiseValue[3] = 0;
    SensorlessTrapController.IPDCurrentRiseValue[4] = 0;
    SensorlessTrapController.IPDCurrentRiseValue[5] = 0;
    SensorlessTrapController.IPDCurrentRiseValue[6] = 0;
    SensorlessTrapController.IPDStart = FALSE;
    SensorlessTrapController.IPDState = 0;

    //Align Variables
    SensorlessTrapController.AlignComplete = FALSE;
    SensorlessTrapController.AlignWaitCounter = 0;
    SensorlessTrapController.StartAlign = FALSE;

    //Open Loop Acceleration Variables
    SensorlessTrapController.AccelCounter = 0;
    SensorlessTrapController.AccelDistance = ACCEL_30_DEGREES;
    SensorlessTrapController.AccelDone = FALSE;
    SensorlessTrapController.AccelVelocityInit =  SensorlessTrapController.AccelVelocity; // Initial value of openloop acceleration from GUI

    //Closed Loop Variables
    SensorlessTrapController.ADCchange = FALSE;
    SensorlessTrapController.ADCcnt = 0;
    SensorlessTrapController.ADCdelay = 0x00;
    SensorlessTrapController.ADCready = FALSE;
    SensorlessTrapController.ADCswitch = FALSE;
    SensorlessTrapController.BEMFtrigger = FALSE;
    SensorlessTrapController.CommStateDetect = 0x00;
    SensorlessTrapController.CTvoltage = 0x00;
    SensorlessTrapController.GetBEMF = 0x00;
    SensorlessTrapController.SpeedChange = FALSE;
    SensorlessTrapController.Direction_flag = FALSE;
    SensorlessTrapController.Direction=TRUE;
    SensorlessTrapController.SpeedDivider = 0x00;
    SensorlessTrapController.SumBEMF = 0x00;
    SensorlessTrapController.StallDetectCounter = 0;
    SensorlessTrapController.StallDetectDelay = 0;
    //System Variables
    SensorlessTrapController.CurrentCommState = 0xFF;
    SensorlessTrapController.CurrentDutyCycle = 0x0000;
    SensorlessTrapController.faultreg = 0x00;
    SensorlessTrapController.SystemCount = 0x00;
    SensorlessTrapController.TargetDutyCycle = 0x0000;
    SensorlessTrapController.VCCvoltage = 0x00;
    /* This is the variable that is used to count the time from when a fault is cleared to when the
            motor starts again */
    SensorlessTrapController.RestartDelay = 0x00;
    SensorlessTrapController.RestartDelayCounter = 0x00;

}
/*function
 * UART initialisation to communicate through UART at 9600 BAUD rate for GUI
 */
void UART_Init(void)
{
    UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;                     // SMCLK
    UCA1BR0 = 157; // 24MHz -9600 Baud 24.15M/9600= 2515.625 , 2515.625/16 = 157.23 , 0.23 *16 = 3
    UCA1BR1 = 0;                              //
    UCA1MCTLW = /*UCBRS_0 +*/UCBRF_3 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0,
    UCA1CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}
void ADC_Init(void)
{
    ADCCTL0 = ADCON | ADCSHT_2; // Turn on ADC, avoid overflow of results , select 16 clock cycles for sampling
    ADCCTL1 = ADCSHP | ADCCONSEQ_0;// | ADCSSEL_3; // Use sampling timer, Single sequence, start conversion from memory address 0 , Select clock as SMCLK
    ADCCTL2 = ADCRES_2;
    ADCMCTL0 = ADCINCH_8; //   channel = A6 (Read the speed input from pot 0-3.3v ) ,
    ADCIE = 0x00;
}

/*
 * function ADC_IPD_INIT , samples 6 adc channels from current shunt amplifiers of 3 phases of BLDC motor ,
 * generates interrupt after all 6 conversions , total conversion time for each channel is approximately 110 clock cycles
 */

void ADC_IPD_Init(void)
{
    if(SensorlessTrapController.DeviceID & BIT2)    //sensorless
    {
//        ADCCTL0 = 0x00;
//        ADCCTL0 = ADCON | ADCSHT_2;                            // Turn on ADC, avoid overflow of results , select 16 clock cycles for sampling current , 4 clk cycles for sampling BEMF
        ADCCTL1 |= ADCSHP | ADCCONSEQ_0;       // Use sampling timer, Single sequence, start conversion from memory address 0 , Select clock as SMCLK
        //ADCCTL2 = ADCRES1;            // select 12 bit resolution //dtbui
        ADCMCTL0 = ADCINCH_8;                           //   channel = A3 (Read the CSA reading from Phase C )
//        ADCMCTL1 = ADCINCH_4;                          //   channel = A12 (Read the CSA reading from Phase A )
//        ADCMCTL2 = ADCINCH_5;                           //   channel = A4 (Read the CSA reading from Phase B )
//        ADCMCTL3 = ADCINCH_4;                          //   channel = A12 (Read the CSA reading from Phase A )
//        ADCMCTL4 = ADCINCH_6;                           //   channel = A3 (Read the CSA reading from Phase C )
//        ADCMCTL5 = ADCINCH_5;         //   channel = A4 (Read the CSA reading from Phase B ) , End of Sequence
        //ADCIE = ADCIE0;
    }
    else
    {
        ADCCTL0 = ADCON | ADCSHT_2;
        ADCCTL1 = ADCSHP | ADCCONSEQ_0; // Use sampling timer, Single sequence, start conversion from memory address 0 , Select clock as SMCLK
        ADCMCTL0 |= ADCINCH_4;
        ADCIE = ADCIE0;
    }
}

/* Function Clock Initialization to make use of 25MHZ setting DCO in Clock module of MSP4320F5529 */

void UCS_Init(void)
{
    WDTCTL = WDTPW + WDTHOLD;                       // Stop WDT

    // FRAM init
    FRCTL0 = (FRCTLPW | NWAITS_2); // 2 wait states for 24MHz

    CSCTL1 = (DCORSEL2 | DCORSEL1 | DCORSEL0); // 24MHz
    CSCTL2 = (FLLN9 | FLLN7 | FLLN6 | FLLN5 | FLLN0); // 32.768kHz x 737 (736.995, 0x2E1) = 24.15MHz
    CSCTL3 = (SELREF0);
    CSCTL4 = 0x0000;
    CSCTL5 = 0x0000; // 24.15MHz as MCLK and SMCLK
    CSCTL6 = XT1DRIVE1 | XT1DRIVE0;
    CSCTL7 = ENSTFCNT1;
    CSCTL8 = (MODCLKREQEN | ACLKREQEN);

    do
    {
        CSCTL7 &= ~(FLLULIFG | DCOFFG); //CSCTL7 &= ~(XT1OFFG | DCOFFG);

        SFRIFG1 &= ~OFIFG;
    }
    while (CSCTL7 & FLLULIFG); //while (SFRIFG1 & OFIFG);
}
///* Function Vcore to Use 25mhz DCO in clock module of MSP430F5529 use Vcore up settings to make DCO support 25Mhz*/
//void SetVcoreUp(unsigned int level)
//{
//    // Open PMM registers for write
//    PMMCTL0_H = PMMPW_H;
//    // Set SVS/SVM high side new level
//    SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
//    // Set SVM low side to new level
//    SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
//    // Wait till SVM is settled
//    while((PMMIFG & SVSMLDLYIFG) == 0)
//    {
//        ;
//    }
//    // Clear already set flags
//    PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
//    // Set VCore to new level
//    PMMCTL0_L = PMMCOREV0 * level;
//    // Wait till new level reached
//    if((PMMIFG & SVMLIFG))
//    {
//        while((PMMIFG & SVMLVLRIFG) == 0)
//        {
//            ;
//        }
//    }
//    // Set SVS/SVM low side to new level
//    SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
//    // Lock PMM registers for write access
//    PMMCTL0_H = 0x00;
//}

void GPIO_Init(void)
{

    //SYSCFG2 |= TB3TRGSEL;   // high-impedence output enable selection
/* Debug Pin */


/* LED initialization ports P1.0 and P4.7 */

    P2DIR |= BIT4 | BIT5 | BIT6;    //LED 0,1,2
    P2OUT &= ~(BIT4 | BIT5 | BIT6);
    P3DIR |= BIT1 | BIT2 | BIT3 | BIT4; //LED 3.4.5.6
    P3OUT &= ~(BIT1 | BIT2 | BIT3 | BIT4);


/* PWM Initialization Using Ports P1.3, P1.5 , P2.5 for A , B , C Phases High side , P1.2, P1.4 , P2.4 for A , B , C Phases Low side respectively */
    P6DIR |= (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5);
    P6OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5);

    P6SEL0 |= (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5);


//    /*  Enable Button Switch S2 at P1.1 for sensing Direction Change by enabling Interrupts */ //dtbui
//        P1DIR &= ~BIT1;
//        P1REN |= BIT1;                              // When a GPIO pin is configured as Input, Enable resistance to the pin by setting Px.REN and PX.OUT
//        P1OUT |= BIT1;
//        P1IE |= BIT1;
//        P1IFG |= 0x00;
//        P1IES |= BIT1;
//
//    // Enable input signal
    P2DIR &= ~(BIT1 | BIT2 | BIT3);
    P2REN |= BIT1 | BIT2 | BIT3; // When a GPIO pin is configured as Input, Enable resistance to the pin by setting Px.REN and PX.OUT
    P2OUT |= BIT1 | BIT2 | BIT3;

    P2IE = 0x00;
    P2IE |=  BIT1 | BIT3;
    P2IES |= BIT1; //Ngat theo suon xuong: 1 -> 0
//    P2IES &= ~BIT1; //Ngat theo suon len: 0 -> 1
    P2IES &= ~BIT3;
// Configure port 2.3 as input for POT
//    P2DIR &= ~BIT3;
//    P2REN |= BIT3; // When a GPIO pin is configured as Input, Enable resistance to the pin by setting Px.REN and PX.OUT
//    P2OUT |= BIT3;
//    P2IE |=  BIT3;
//    P2IES |= BIT3;

/* Configure Port 4.1 as input for sensing faults and enable Interrupt */
    P4DIR &= ~BIT1;
    P4REN |= BIT1; // When a GPIO pin is configured as Input, Enable resistance to the pin by setting Px.REN and PX.OUT
    P4OUT |= BIT1;
    P4IE = 0x00;
    P4IE |= BIT1;
    P4IES |= BIT1;


/* ADC Input Channels Selected as ADC function eliminates parasitic currents flow */
    P1SEL0 |= BIT0 | BIT1| BIT2 | BIT3 | BIT4;
    P1SEL1 |= BIT0 | BIT1| BIT2 | BIT3 | BIT4;
    P5SEL0 |= BIT0;
    P5SEL1 |= BIT0;
/* UART Pins initialisation to select UART functionality*/
    P4SEL0 = BIT3 + BIT2;                      // P4.4,5 = USCI_A1 TXD/RXD
/* Configure pin for DRV832xH devices */


    //Configure the Mode pin
//    if(SensorlessTrapController.PWM_Mode == 0) // If 6x PWM mode configure the Mode pin to pull down
//    {
//
//        P3DIR |= BIT0;   // Set the pin as Output
//        P3OUT &= ~BIT0;  // Set the pin logic low
//    }
//    else // If 1x PWM mode configure mode pin to High Impedence state
//    {
//
//        P3DIR &= ~BIT0;  // Set the pin as Input
//        P3REN &= ~BIT0;  // Diable the the Pull up/ pull down set the pin to high impedence
//    }
// Configure unused pin


// Reset port interrupt
    PM5CTL0 &= ~LOCKLPM5;       // unlock GPIO
    P2IFG = 0x00;
    P4IFG = 0x00;
/* Cho phep ngat toan cuc */
//    _BIS_SR(GIE);
//    _BIS_SR(CPUOFF + GIE); // Enter LPM0 w/ interrupt - Sleep and wait interrupt event
}

/* USCI Initialization
   The USCI is initialized with the following setting
   3-pin, 8-bit SPI master,Clock polarity high,MSB
 */
//void SPI_Init(void)
//{
//    /* SPI Ports Initialization * Port 3.0, 3.1, 3.2 is used for SIMO SOMI and SCLK respectively , Port 2.0 is Used for nSCS enable*/
//    P4SEL0 |= (BIT5 + BIT6 + BIT7);
//
//    P4DIR |= BIT4;
//    P4OUT |= BIT4;
//                                           // Set P2.3 to output direction for nSCS
//
//    UCB1CTLW0 |= UCSWRST;                       // **Put state machine in reset**
//    UCB1CTLW0 |= UCMST | UCSYNC | UCMSB;           // 3-pin, 8-bit SPI master
//    // Clock polarity high, MSB
//    UCB1CTLW0 |= UCSSEL_3;                         // MCLK
//    UCB1BRW = 10;                 // Master Clock divided by 10 used by USCI clk
////    UCB1BR1 = 0;                                  //
//    UCB1CTLW0 &= ~UCSWRST;                   // **Initialize USCI state machine**
//
//    DRV83xxSPISet(); // make nSCS pin of DRV83xx low to start communication with master SPI;
//    DRV83xxSPIReset(); // make nSCS pin of DRV83xx High to stop communication with master SPI;
//    SPIDelay();
//}

/*function
 * Timer_IPD_Init()
 * initializes the TimerA1 for Initial position detection
 * */
void TIMER_IPD_Init()   //Timer B2 CC0 (A1 CC0)
{
    TB2CTL = MC_0 | TBCLR;
    TB2CTL = TBSSEL_2 | TBCLR | MC_1; /* set continues up count mode , Select timer clock source as SMCLK 25Mhz, Clock division by 1*/
    TB2CCR0 = SensorlessTrapController.IPDPulseTime; /* set period value for IPD , No of clock cycles a voltage pulse is applied across a phase */
    TB2R = 0x0000; /* reset counter */
    TB2EX0 = TBIDEX_0;
    TB2CCTL0 |= CCIE;
}

/*function
 * Timer_IPD_Init()
 * initializes the TimerA1 for Speed Calculatation
 * */
void TIMER_SPD_Init()   //Timer B2 CC0 (A1 CC0)
{
    TB2CTL = TBSSEL_2 | TBCLR | MC_1 | ID_3 ;   /* set continues UP count mode , Clk Div / 8 */
    TB2CCR0 = 0x0FFFF;                              /* This sets  Timer to count for 65535 counts at 25Mhz */
    TB2EX0 = TBIDEX_4;                           /* Divide clock by 5 , to make overall clock run at 25Mhz/(2*5) = 25MHz */
    TB2CCTL0 |= CCIE;                            /* enable TimerA1 period match interrupt */
    TB2R = 0x0000;                              /* reset counter */
}

/*function
 * TimerB0_Init()
 * initializes the TimerB0 to Monitor BEMF for every falling edge
 * */
void TimerB0_Init(void)
{
    TB0CTL = TBSSEL_2 | TBCLR | MC_1;           /* set SMCLK to run timer, Clear the Timer , continuous up count mode   */
    TB0CCR0 = SensorlessTrapController.PWMPeriod;                           /* set Period value as PWM_period*/
    TB0CCR1 = (SensorlessTrapController.PWMPeriod -  SensorlessTrapController.PWMBlankCounts);
    TB0CCTL0 |= CCIE;                                           /* Enable TimerB0 Period match interrupt for Stall fault detectection and fault recovery */
    TB0CCTL1 |= CCIE;                                           /* Enable TimerB0 Compare match interrupt for State machine */
    TB0R = 0x0000;                                              /* reset counter */
}

/*function
 * TimerA0_Init()
 * initializes the TimerA0 to generate PWM pulses from TA0.1, TA0.2, TA0.3, TA0.4  also generate Period Interrupt for State machine
 * */
void TimerB3_Init(void)
{
    TB3CTL = TBSSEL_2 | TBCLR | MC_1;           /* set SMCLK to run timer, Clear the Timer , continuous up count mode   */
    TB3CCR0 = SensorlessTrapController.PWMPeriod;                           /* set Period value as PWM_period*/

    TB3CCTL0 |= CCIE;                                           /* Enable TimerA0 period match interrupt for State machine */
    TB3R = 0x0000;                                              /* reset counter */

    /*set out put mode to GPIO low */
    TB3CCTL1 = OUTMOD_0;
    TB3CCTL2 = OUTMOD_0;
    TB3CCTL3 = OUTMOD_0;
    TB3CCTL4 = OUTMOD_0;
    TB3CCTL5 = OUTMOD_0;
    TB3CCTL6 = OUTMOD_0;

    /* reset compare */
    TB3CCR1 = 0x00;
    TB3CCR2 = 0x00;
    TB3CCR3 = 0x00;
    TB3CCR4 = 0x00;
    TB3CCR5 = 0x00;
    TB3CCR6 = 0x00;

    TB3CCTL1 = OUTMOD_7;
    TB3CCTL2 = OUTMOD_3;
    TB3CCTL3 = OUTMOD_7;
    TB3CCTL4 = OUTMOD_3;
    TB3CCTL5 = OUTMOD_7;
    TB3CCTL6 = OUTMOD_3;
}

/*function
 * TimerA2_Init()
 * initializes the TimerA2 to generate PWM pulses from TA2.2
 * */
void TimerB1_Init(void)
{
    TB1CTL = TBSSEL_2 | TBCLR | MC_1;           /* set SMCLK to run timer, Clear the Timer , continuous up count mode   */
    TB1CCR0 = SensorlessTrapController.PWMPeriod;                           /* set Period value as PWM_period*/

    TB1CCTL0 |=  CCIE;

    TB1R = 0x0000;                                              /* reset Timer 2 counter */

}

void IPD_Shunt_Amplifier_Control_Init(void)
{
    if(SensorlessTrapController.DeviceID & BIT0)                  // Initialize the SPI variables and settings only if the device is "S" Variant
    {
       // SPI_Write(SPI_REG_CSA_CTRL,(SPI_Read(SPI_REG_CSA_CTRL) | (CSA_GAIN << 6)));      // Write appropriate Bits for setting CSA gain to 80v/v

    }
}

/*function
 * DRV8x_Digital_Init(void)
 * initializes the MSP430F5529 IP's
 * */
void DRV8x_Digital_Init(void)
{
    UCS_Init();                                 // Clock Initialization
    ADC_Init();
    GPIO_Init();                // GPIO ports Initialization
    //UART_Init();
    TimerB0_Init();
    TimerB3_Init();             // Timer A0 Initialization to generate 4 PWM's for switches
    TimerB1_Init();             // Timer A2 Initialization to generate 4 PWM's for switches

    //__bis_SR_register(GIE);                                     /*enable global interrupt */
    __enable_interrupt();           // Enter LPM3 w/ interrupts
}

/*function
 * Read_SPI_Registers_Init(void)
 * Read SPI register values to debug
 * */
void DRV8x_Analog_Init(void)
{
    if(SensorlessTrapController.DeviceID & BIT0)                  // Initialize the SPI variables and settings only if the device is "S" Variant
    {
        SPI_Init();                 // Initialize EVM8305 SPI in slave mode
        DRV832x_Register_Read();
        DRV832x_Register_Init();
        DRV832x_Register_Write();
//        DRV832x_Register_Read();
    }
}
/*function
 * Init_IPD()
 * Initialize parameters for using IPD
 * */
void IPD_Init(void)
{
    TIMER_IPD_Init();
    ADC_IPD_Init();
    IPD_Shunt_Amplifier_Control_Init();
    DisableGateDrivers();
}

//void DRV832x_Register_Read(void)
//{
//    uint16_t regValue;
//
//    // Read Register 0x00
//    regValue = SPI_Read(SPI_REG_FAULT_STAT);
//
//    Fault_Status_Reg.REG0_FAULT = (regValue & FAULT_MASK) >> 10;
//    Fault_Status_Reg.REG0_VDS_OCP = (regValue & VDS_OCP_MASK) >> 9;
//    Fault_Status_Reg.REG0_GDF = (regValue & GDF_MASK) >> 8;
//    Fault_Status_Reg.REG0_UVLO = (regValue & UVLO_MASK) >> 7;
//    Fault_Status_Reg.REG0_OTSD = (regValue & OTSD_MASK) >> 6;
//    Fault_Status_Reg.REG0_VDS_HA = (regValue & VDS_HA_MASK) >> 5;
//    Fault_Status_Reg.REG0_VDS_LA = (regValue & VDS_LA_MASK) >> 4;
//    Fault_Status_Reg.REG0_VDS_HB = (regValue & VDS_HB_MASK) >> 3;
//    Fault_Status_Reg.REG0_VDS_LB = (regValue & VDS_LB_MASK) >> 2;
//    Fault_Status_Reg.REG0_VDS_HC = (regValue & VDS_HC_MASK) >> 1;
//    Fault_Status_Reg.REG0_VDS_LC = (regValue & VDS_LC_MASK) >> 0;
//
//    // Read Register 0x01
//    regValue = SPI_Read(SPI_REG_VGS_STAT);
//
//    VGS_Status_Reg.REG1_SA_OC = (regValue & SA_OC_MASK) >> 10;
//    VGS_Status_Reg.REG1_SB_OC = (regValue & SB_OC_MASK) >> 9;
//    VGS_Status_Reg.REG1_SC_OC = (regValue & SC_OC_MASK) >> 8;
//    VGS_Status_Reg.REG1_OTW = (regValue & OTW_MASK) >> 7;
//    VGS_Status_Reg.REG1_CPUV = (regValue & CPUV_MASK) >> 6;
//    VGS_Status_Reg.REG1_VGS_HA = (regValue & VGS_HA_MASK) >> 5;
//    VGS_Status_Reg.REG1_VGS_LA = (regValue & VGS_LA_MASK) >> 4;
//    VGS_Status_Reg.REG1_VGS_HB = (regValue & VGS_HB_MASK) >> 3;
//    VGS_Status_Reg.REG1_VGS_LB = (regValue & VGS_LB_MASK) >> 2;
//    VGS_Status_Reg.REG1_VGS_HC = (regValue & VGS_HC_MASK) >> 1;
//    VGS_Status_Reg.REG1_VGS_LC = (regValue & VGS_LC_MASK) >> 0;
//
//    // Read Register 0x02
//    regValue = SPI_Read(SPI_REG_DRV_CTRL);
//
//    Driver_Control_Reg.REG2_OCP_ACT = (regValue & OCP_ACT_MASK) >> 10;
//    Driver_Control_Reg.REG2_DIS_CPUV = (regValue & DIS_CPUV_MASK) >> 9;
//    Driver_Control_Reg.REG2_DIS_GDF = (regValue & DIS_GDF_MASK) >> 8;
//    Driver_Control_Reg.REG2_OTW_REP = (regValue & OTW_REP_MASK) >> 7;
//    Driver_Control_Reg.REG2_PWM_MODE = (regValue & PWM_MODE_MASK) >> 5;
//    Driver_Control_Reg.REG2_PWM_COM = (regValue & PWM_COM_MASK) >> 4;
//    Driver_Control_Reg.REG2_PWM_DIR = (regValue & PWM_DIR_MASK) >> 3;
//    Driver_Control_Reg.REG2_COAST = (regValue & COAST_MASK) >> 2;
//    Driver_Control_Reg.REG2_BRAKE = (regValue & BRAKE_MASK) >> 1;
//    Driver_Control_Reg.REG2_CLR_FLT = (regValue & CLR_FLT_MASK) >> 0;
//
//    // Read Register 0x03
//    regValue = SPI_Read(SPI_REG_GATE_DRV_HS);
//
//    Gate_Drive_HS_Reg.REG3_LOCK = (regValue & LOCK_MASK) >> 8;
//    Gate_Drive_HS_Reg.REG3_IDRIVEP_HS = (regValue & IDRIVEP_HS_MASK) >> 4;
//    Gate_Drive_HS_Reg.REG3_IDRIVEN_HS = (regValue & IDRIVEN_HS_MASK) >> 0;
//
//    // Read Register 0x04
//    regValue = SPI_Read(SPI_REG_GATE_DRV_LS);
//
//    Gate_Drive_LS_Reg.REG4_CBC = (regValue & CBC_MASK) >> 10;
//    Gate_Drive_LS_Reg.REG4_TDRIVE = (regValue & TDRIVE_MASK) >> 8;
//    Gate_Drive_LS_Reg.REG4_IDRIVEP_LS = (regValue & IDRIVEP_LS_MASK) >> 4;
//    Gate_Drive_LS_Reg.REG4_IDRIVEN_LS = (regValue & IDRIVEN_LS_MASK) >> 0;
//
//    // Read Register 0x05
//    regValue = SPI_Read(SPI_REG_OCP_CTRL);
//
//    OCP_Control_Reg.REG5_TRETRY = (regValue & TRETRY_MASK) >> 10;
//    OCP_Control_Reg.REG5_DEAD_TIME = (regValue & DEAD_TIME_MASK) >> 8;
//    OCP_Control_Reg.REG5_OCP_MODE = (regValue & OCP_MODE_MASK) >> 6;
//    OCP_Control_Reg.REG5_OCP_DEG = (regValue & OCP_DEG_MASK) >> 4;
//    OCP_Control_Reg.REG5_VDS_LVL = (regValue & VDS_LVL_MASK) >> 0;
//
//    if (SensorlessTrapController.DeviceID & BIT2)
//    {
//        // Read Register 0x06
//        regValue = SPI_Read(SPI_REG_CSA_CTRL);
//
//        CSA_Control_Reg.REG6_CSA_FET = (regValue & CSA_FET_MASK) >> 10;
//        CSA_Control_Reg.REG6_VREF_DIV = (regValue & VREF_DIV_MASK) >> 9;
//        CSA_Control_Reg.REG6_LS_REF = (regValue & LS_REF_MASK) >> 8;
//        CSA_Control_Reg.REG6_CSA_GAIN = (regValue & CSA_GAIN_MASK) >> 6;
//        CSA_Control_Reg.REG6_DIS_SEN = (regValue & DIS_SEN_MASK) >> 5;
//        CSA_Control_Reg.REG6_CSA_CAL_A = (regValue & CSA_CAL_A_MASK) >> 4;
//        CSA_Control_Reg.REG6_CSA_CAL_B = (regValue & CSA_CAL_B_MASK) >> 3;
//        CSA_Control_Reg.REG6_CSA_CAL_C = (regValue & CSA_CAL_C_MASK) >> 2;
//        CSA_Control_Reg.REG6_SEN_LVL = (regValue & SEN_LVL_MASK) >> 0;
//    }
//
//}
//
//void DRV832x_Register_Init(void)
//{
//    // Set Register 0x02
//    Driver_Control_Reg.REG2_OCP_ACT = OCP_ACT;
//    Driver_Control_Reg.REG2_DIS_CPUV = DIS_CPUV;
//    Driver_Control_Reg.REG2_DIS_GDF = DIS_GDF;
//    Driver_Control_Reg.REG2_OTW_REP = OTW_REP;
//    Driver_Control_Reg.REG2_PWM_MODE = PWM_MODE;
//    Driver_Control_Reg.REG2_PWM_COM = PWM_COM;
//    Driver_Control_Reg.REG2_PWM_DIR = PWM_DIR;
//    Driver_Control_Reg.REG2_COAST = COAST_BIT;
//    Driver_Control_Reg.REG2_BRAKE = BRAKE_BIT;
//    Driver_Control_Reg.REG2_CLR_FLT = CLR_FLT;
//
//    // Set Register 0x03
//    Gate_Drive_HS_Reg.REG3_LOCK = LOCK_BIT;
//    Gate_Drive_HS_Reg.REG3_IDRIVEP_HS = IDRIVEP_HS;
//    Gate_Drive_HS_Reg.REG3_IDRIVEN_HS = IDRIVEN_HS;
//
//    // Set Register 0x04
//    Gate_Drive_LS_Reg.REG4_CBC = CBC;
//    Gate_Drive_LS_Reg.REG4_TDRIVE = TDRIVE;
//    Gate_Drive_LS_Reg.REG4_IDRIVEP_LS = IDRIVEP_LS;
//    Gate_Drive_LS_Reg.REG4_IDRIVEN_LS = IDRIVEN_LS;
//
//    // Set Register 0x05
//    OCP_Control_Reg.REG5_TRETRY = TRETRY;
//    OCP_Control_Reg.REG5_DEAD_TIME = DEAD_TIME;
//    OCP_Control_Reg.REG5_OCP_MODE = OCP_MODE;
//    OCP_Control_Reg.REG5_OCP_DEG = OCP_DEG;
//    OCP_Control_Reg.REG5_VDS_LVL = VDS_LVL;
//
//    // Set Register 0x06
//    CSA_Control_Reg.REG6_CSA_FET = CSA_FET;
//    CSA_Control_Reg.REG6_VREF_DIV = VREF_DIV;
//    CSA_Control_Reg.REG6_LS_REF = LS_REF;
//    CSA_Control_Reg.REG6_CSA_GAIN = CSA_GAIN;
//    CSA_Control_Reg.REG6_DIS_SEN = DIS_SEN;
//    CSA_Control_Reg.REG6_CSA_CAL_A = CSA_CAL_A;
//    CSA_Control_Reg.REG6_CSA_CAL_B = CSA_CAL_B;
//    CSA_Control_Reg.REG6_CSA_CAL_C = CSA_CAL_C;
//    CSA_Control_Reg.REG6_SEN_LVL = SEN_LVL;
//}
//
//void DRV832x_Register_Write(void)
//{
//    uint16_t regValue;
//
//    // Write Register 0x02
//    regValue = (Driver_Control_Reg.REG2_OCP_ACT << 10)
//            | (Driver_Control_Reg.REG2_DIS_CPUV << 9)
//            | (Driver_Control_Reg.REG2_DIS_GDF << 8)
//            | (Driver_Control_Reg.REG2_OTW_REP << 7)
//            | (Driver_Control_Reg.REG2_PWM_MODE << 5)
//            | (Driver_Control_Reg.REG2_PWM_COM << 4)
//            | (Driver_Control_Reg.REG2_PWM_DIR << 3)
//            | (Driver_Control_Reg.REG2_COAST << 2)
//            | (Driver_Control_Reg.REG2_BRAKE << 1)
//            | (Driver_Control_Reg.REG2_CLR_FLT);
//
//    SPI_Write(SPI_REG_DRV_CTRL, regValue);
//    Reg_Map_Cache.Driver_Control_Reg2 = regValue;
//
//    // Write Register 0x03
//    regValue = (Gate_Drive_HS_Reg.REG3_LOCK << 8)
//            | (Gate_Drive_HS_Reg.REG3_IDRIVEP_HS << 4)
//            | (Gate_Drive_HS_Reg.REG3_IDRIVEN_HS);
//
//    SPI_Write(SPI_REG_GATE_DRV_HS, regValue);
//    Reg_Map_Cache.Gate_Drive_HS_Reg3 = regValue;
//
//    // Write Register 0x04
//    regValue = (Gate_Drive_LS_Reg.REG4_CBC << 10)
//            | (Gate_Drive_LS_Reg.REG4_TDRIVE << 8)
//            | (Gate_Drive_LS_Reg.REG4_IDRIVEP_LS << 4)
//            | (Gate_Drive_LS_Reg.REG4_IDRIVEN_LS);
//
//    SPI_Write(SPI_REG_GATE_DRV_LS, regValue);
//    Reg_Map_Cache.Gate_Drive_LS_Reg4 = regValue;
//
//    // Write Register 0x05
//    regValue = (OCP_Control_Reg.REG5_TRETRY << 10)
//            | (OCP_Control_Reg.REG5_DEAD_TIME << 8)
//            | (OCP_Control_Reg.REG5_OCP_MODE << 6)
//            | (OCP_Control_Reg.REG5_OCP_DEG << 4)
//            | (OCP_Control_Reg.REG5_VDS_LVL);
//
//    SPI_Write(SPI_REG_OCP_CTRL, regValue);
//    Reg_Map_Cache.OCP_Control_Reg5 = regValue;
//
//    if (SensorlessTrapController.DeviceID & BIT2)
//    {
//        // Write Register 0x06
//        regValue = (CSA_Control_Reg.REG6_CSA_FET << 10)
//                | (CSA_Control_Reg.REG6_VREF_DIV << 9)
//                | (CSA_Control_Reg.REG6_LS_REF << 8)
//                | (CSA_Control_Reg.REG6_CSA_GAIN << 6)
//                | (CSA_Control_Reg.REG6_DIS_SEN << 4)
//                | (CSA_Control_Reg.REG6_CSA_CAL_A << 3)
//                | (CSA_Control_Reg.REG6_CSA_CAL_C << 2)
//                | (CSA_Control_Reg.REG6_SEN_LVL);
//
//        /* This register exists only in DRV8323S */
//        if (SensorlessTrapController.DeviceID & BIT2)
//        {
//            SPI_Write(SPI_REG_CSA_CTRL, regValue);
//            Reg_Map_Cache.CSA_Control_Reg6 = regValue;
//        }
//
//    }
//}



