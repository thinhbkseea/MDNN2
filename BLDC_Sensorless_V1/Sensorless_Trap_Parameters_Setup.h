/*
 * Sensorless_Trap_Parameters_Setup.h
 *
 *  Created on: Nov 30, 2022
 *      Author: thinh
 */


// USER DEFINES
#define ALGO_ID                     1       // 1 Indicates sensorless , 0 indicates sensored algorithms
#define ALIGN_OR_IPD                1       //  0 indicates Six pulse method , 1 indicates rotor align
//ISC User Parameters
#define ISC_MIN_BEMF                100 //150     // Minimum BEMF below which programs skips to IPD after brake time set by ISC BRAKE TIME in ms to disscipate the residual BEMF( 1.4v *4096)/57.5 = 100
#define ISC_BRAKE_TIME              45 //60      // [this number+1]*1ms = brake time appled once BEMF of motor is less than ISC_MIN_BEMF, if motor is spinning  during start up
//IPD User Parameters
#define IPD_ADD_BRAKE               30      // the (IPDRiseTime/512)+this number  IPDRiseTime is measured using timerA at 12.5MHz so 400us = 5000
#define IPD_PULSE_TIME              3000        // set no of clock cycles (1 clock cycles = 40nS) a voltage pulse is applied on a phase for Initial Position detection by Six pulse method
#define IPD_DECAY_CONSTANT          3       // how many times longer to coast the motor and wait until next pulse after braking in IPD

//Align User Parameters
#define ALIGN_SECTOR                1       // Align commutation sequence (1-6)
#define ALIGN_WAIT_TIME             150//500//150     // Number of PWM cycles to Wait during align align time seconds = [this number * 41us]

//Open Loop Acceleration User Parameters
#define ACCEL_RATE                  40//40      // {Hz/s}
#define ACCEL_STOP                  50000//50000   // {mHz}
#define ACCEL_VELOCITY_INIT         28000//10000   // {mHz} 28000

//Closed Loop User Parameters
#define BEMF_THRESHOLD              1700//1960    // BEMF Integration threshold according to calculations of BEMF waveform
#define RAMP_RATE_DELAY             50  //100   // This number controls the acceleration,  duty cycle is updated after ( RAMP_RATE_DELAY * 1000) clock cycles
#define RAMP_RATE                   1 //1       // This is the change in dutycycle (increment/decrement) for every update
#define COMMUTATION_BLANK_TIME      5       // How many PWM cycles to blank before sampling the BEMF
#define PWM_BLANK_COUNTS            20//5       // How many Clock cycles before the center of PWM  the BEMF is sampled

#define MAX_DUTY_CYCLE              1000        // relative to PWM_PERIOD
#define MIN_OFF_DUTY                0//250         // relative to PWM_PERIOD
#define MIN_ON_DUTY                 10//260         // relative to PWM_PERIOD
#define START_UP_DUTY_CYCLE         200 //100 //200 ok  //300      // relative to PWM_PERIOD
#define PWM_FACTOR                  0           //  ADC 12 bit to PWM width ratio , by default 0 represents 12 bit scaling

/* Fault handling setup */                      /*ADC Max ref voltage is 3.3v ,  VCC is scaled by 0.0573 internally (5/88 Ohms bridge) so that VCC ref input to ADC never cross 3.3v The maximum supply voltage is 57.5v.*/
#define UNDER_VOLTAGE_LIMIT (2742)//2742               /* Under Voltage set for below 10.0V - (10*4096)/57.5 = 712  */
#define OVER_VOLTAGE_LIMIT  (3561)              /* Over Voltage set for above 20.0V  - (20*4096)/57.5 = 1424 */ //dtbui
#define STALLDETECT_REV_THRESHOLD   (1)         /* Number of revolutions below which stall fault will be flagged */
#define STALLDETECT_TIMER_THRESHOLD (200)       /* Time in milli seconds above which if motor doesnt spin min revolutions specified above(STALLDETECT_REV_THRESHOLD) a stall fault is triggered */
#define MOTOR_PHASE_CURRENT_LIMIT (2048) //(900) //1800 Vso = 1.65-Imax*Gcsa*Rsense = 1.65-30*10*0.002 = 1.05V (1300)744
#define AUTO_FAULT_RECOVERY_TIME (3000)         /*  Delay in milli Seconds after which system reinitialises itself if fault gets cleared */

/* DRV8323 SPI REGISTER SETTINGS*/
/* SPI_REG_02 : DRIVER CONTROL */
#define OCP_ACT        (0x00)         /*  */
#define DIS_CPUV    (0x00)         /*  */
#define DIS_GDF     (0x00)         /*  */
#define OTW_REP     (0x00)         /*  */
#define PWM_MODE    (0x00)         /*  */
#define PWM_COM     (0x00)         /*  */
#define PWM_DIR     (0x00)         /*  */
#define COAST_BIT   (0x00)         /*  */
#define BRAKE_BIT   (0x00)         /*  */
#define CLR_FLT     (0x00)         /*  */

/* SPI_REG_03 : GATE DRIVE HS */
#define LOCK_BIT    (0x03)         /*  */
#define IDRIVEP_HS  (0x02)         /* dtbui */
#define IDRIVEN_HS  (0x02)         /*  */

/* SPI_REG_04 : GATE DRIVE LS */
#define CBC         (0x01)         /*  */
#define TDRIVE      (0x03)         /*  */
#define IDRIVEP_LS  (0x02)         /* dtbui */
#define IDRIVEN_LS  (0x02)         /*  */

/* SPI_REG_05 : OCP CONTROL */
#define TRETRY      (0x00)         /*  */
#define DEAD_TIME   (0x00)         /*  */
#define OCP_MODE    (0x01)         /*  */
#define OCP_DEG     (0x01)         /*  */
#define VDS_LVL     (0x09)         /*  */

/* SPI_REG_06 : CSA CONTROL */
#define CSA_FET     (0x00)         /*  */
#define VREF_DIV    (0x01)         /*  */
#define LS_REF      (0x00)         /*  */
#define CSA_GAIN    (0x01)         /*  */
#define DIS_SEN     (0x00)         /*  */
#define CSA_CAL_A   (0x00)         /*  */
#define CSA_CAL_B   (0x00)         /*  */
#define CSA_CAL_C   (0x00)         /*  */
#define SEN_LVL     (0x03)         /*  */

