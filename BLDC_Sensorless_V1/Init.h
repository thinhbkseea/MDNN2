/*
 * init.h
 *
 *  Created on: Nov 30, 2022
 *      Author: thinh
 */




void Init_Application(void);
void SensorlessTrapController_Init(void);
void Init_IPD(void);
void DRV8x_Digital_Init(void);  // initialize MSP430 */
void UCS_Init(void);                    // Clock Initialization
void GPIO_Init(void);            // initialize GPIO ports
void UART_Init(void);
void ADC_Init(void);             // initialize ADC
void TIMER_IPD_Init();                   // initialize Timer for IPD to measure the time for which a pulse is to be applied
void TimerB3_Init(void);         // initialize Timer A0  to generate 4 PWM's for switches
void TIMERB1_Init(void);         // initialize Timer A1  for Vcc reading
//void TimerB2_Init(void);         // initialize Timer A2  to generate 2 PWM's for A phase
void TimerB0_Init(void);          // initialize Timer B0  for Stall fault detection and Fault recovery time
//void SetVcoreUp(unsigned int level);   // initialize  SetVcoreUp for powering module to increase the DCO clock to 25Mhz
void DRV8x_Analog_Init(void);   // initialize DRV8x Analog front end initialization */
void SPI_Init(void);                    /* Initialize SPI  function definition */
void Application_Init(void);    /* initialize application variables*/
void IPD_Init(void);                     // Initialize parameters for using IPD
void ADC_IPD_Init(void);         // Initialize ADC settings for IPD
void Shunt_Amplifier_Control_Init(void); //Initialize SPI settings for CSA
void IPD_Shunt_Amplifier_Control_Init(void); //Initialize SPI settings for CSA during IPD
void DRV832x_Register_Read(void);
void DRV832x_Register_Init(void);
void DRV832x_Register_Write(void);
void TIMER_SPD_Init(void);     // Initialize timer to read the electrical speed of the motor
//void UART_Init(void);
