#include <msp430.h>
unsigned int time_count = 0;
unsigned int time = 0;
unsigned int count = 0;
_Bool signal = 0;
_Bool start_stop=0;
unsigned int PWM_DUTY_CYCLE=1000;
unsigned int adc_value = 0;
#define SPI_READ_COMMAND    0x80   // SPI read command
#define SPI_DELAY 10
unsigned short SPI_DATA;
void SPI_init(void);
void SPI_write(unsigned short regAddr, unsigned short data);
unsigned short SPI_read(unsigned short regAddr);
void SPIDelay(void);

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
void GPIO_Init(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
//    P4DIR |= BIT5 | BIT6; //  Set P4.5 P4.6 to output direction for LED1 LED2
//    P4OUT |= BIT5 | BIT6;
    P4DIR |= BIT7;
    P4OUT &= ~BIT7;
//    P1SEL0 |= BIT0 | BIT1| BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
//    P1SEL1 |= BIT0 | BIT1| BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
//    P1DIR |= BIT0;
//    P1OUT &= ~BIT0;
//    P2DIR |= BIT0;
//    P2OUT &= ~BIT0;
    /* Configure pin for DRV832xH devices */
//        P3SEL0 &= ~(BIT0 | BIT5 | BIT6 | BIT7);  // Select the GPIO functionality of the pin
//        P3SEL1 &= ~(BIT0 | BIT5 | BIT6 | BIT7);
//        P2SEL0 &= ~BIT7;
//        P2SEL1 &= ~BIT7;
        //turn off calib
//        P3DIR |= BIT7;   // Set the pin as Output
//        P3OUT &= ~BIT7;  // Set the pin logic low
        //Configure the VDS, IDRIVE, GAIN pin as input
//        P3DIR &= ~(BIT5 | BIT6);
//        P3REN &= ~(BIT5 | BIT6);
//        P2DIR &= ~(BIT7);
//        P2REN &= ~(BIT7);
        //Configure the Mode pin


//            P3DIR |= BIT0;   // Set the pin as Output
//            P3OUT &= ~BIT0;  // Set the pin logic low

    /* PWM Initialization Using Ports P1.3, P1.5 , P2.5 for A , B , C Phases High side , P1.2, P1.4 , P2.4 for A , B , C Phases Low side respectively */
        P6DIR |= (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5);
        P6OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5);
//
        P6SEL0 |= (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5);
//    P2DIR &= ~BIT6;
//    P2REN |= BIT6; // When a GPIO pin is configured as Input, Enable resistance to the pin by setting Px.REN and PX.OUT
//    P2OUT |= BIT6;
//    P2IE |=  BIT6;
//    P2IES |= BIT6;
    P2DIR &= ~BIT3;
    P2REN |= BIT3; // When a GPIO pin is configured as Input, Enable resistance to the pin by setting Px.REN and PX.OUT
    P2OUT |= BIT3;
    P2IE |= BIT3;
    P2IES |= BIT3;
//    P2DIR &= ~(BIT1 | BIT2);
//    P2REN |= BIT1 | BIT2; // When a GPIO pin is configured as Input, Enable resistance to the pin by setting Px.REN and PX.OUT
//    P2OUT |= BIT1 | BIT2;
   // P2IE = 0x00;
//    P2IE |=  BIT1 | BIT2;
    P2DIR |= BIT4 | BIT5 | BIT6;
    P2OUT &= ~(BIT4 | BIT5 | BIT6);
    P3DIR |= BIT4 | BIT1 | BIT2 | BIT3;
    P3OUT &= ~(BIT4 | BIT1 | BIT2 | BIT3);
    P4DIR |= BIT5 | BIT6 | BIT7;
    P4OUT &= ~(BIT5 | BIT6 | BIT7);

    P4DIR &= ~BIT1;
    P4REN |= BIT1; // When a GPIO pin is configured as Input, Enable resistance to the pin by setting Px.REN and PX.OUT
    P4OUT |= BIT1;
    P4IE |= BIT1;
    P4IES |= BIT1;
   PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode
    P2IFG = 0x00;
//    P4IFG = 0x00;
}
void TimerB3_Init(void)
{
    TB3CTL = TBSSEL_2 | TBCLR | MC_1;           /* set SMCLK to run timer, Clear the Timer , continuous up count mode   */
    TB3CCR0 = 1000;                           /* set Period value as PWM_period*/

    TB3CCTL0 |= CCIE;                                           /* Enable TimerA0 period match interrupt for State machine */
    TB3R = 0x0000;                                              /* reset counter */

    TB3CCTL1 = OUTMOD_0;
    TB3CCTL2 = OUTMOD_0;
    TB3CCTL3 = OUTMOD_0;
    TB3CCTL4 = OUTMOD_0;
    TB3CCTL5 = OUTMOD_0;
    TB3CCTL6 = OUTMOD_0;
//
//    /* reset compare */
    TB3CCR1 = 0x00;
    TB3CCR2 = 0x00;
    TB3CCR3 = 0x00;
    TB3CCR4 = 0x00;
    TB3CCR5 = 0x00;
    TB3CCR6 = 0x00;
//
    TB3CCTL1 = OUTMOD_7;
    TB3CCTL2 = OUTMOD_3;
    TB3CCTL3 = OUTMOD_7;
    TB3CCTL4 = OUTMOD_3;
    TB3CCTL5 = OUTMOD_7;
    TB3CCTL6 = OUTMOD_3;
//
//    TB3CCR1 = 0;
//    TB3CCR2 = 0;
//    TB3CCR3 = 0;
//    TB3CCR4 = 0;
//    TB3CCR5 = 0;
//    TB3CCR6 = 0;
}
/**
 * main.c
 */
void ADC_Init()
{
    ADCCTL0 = ADCON | ADCSHT_2; // Turn on ADC, avoid overflow of results , select 16 clock cycles for sampling
    ADCCTL1 = ADCSHP | ADCCONSEQ_0 | ADCSSEL_2; // Use sampling timer, Single sequence, start conversion from memory address 0 , Select clock as SMCLK
    ADCCTL2 = ADCRES_2;
    ADCMCTL0 = ADCINCH_7; //   channel = A6 (Read the speed input from pot 0-3.3v ) ,
    ADCIE = 0x00;
}
void SPI_init(void)
{
    // Configure SPI pins
    P1SEL0 |= BIT1 | BIT2 | BIT3;
    P3DIR |= BIT4;
    P3OUT |= BIT4;

    UCB0CTLW0 |= UCSWRST;   // Put SPI module in reset state

    UCB0CTLW0 |= UCCKPH | UCMSB | UCMST | UCSYNC;   // SPI mode, MSB first, master mode, synchronous mode

    UCB0CTLW0 |= UCSSEL_2;   // Use SMCLK as the clock source

    UCB0BR0 = 0x02;   // Set SPI clock prescaler
    UCB0BR1 = 0x00;

    UCB0CTLW0 &= ~UCSWRST;   // Release SPI module from reset state

    // Configure SPI pins
    // ...
}

void SPI_write(unsigned short regAddr, unsigned short data)
{
    // Enable SPI communication by setting appropriate GPIO pins
    // ...
    P3OUT &= ~BIT4;

    UCB0TXBUF = regAddr;   // Send register address

    while (!(UCB0IFG & UCRXIFG));   // Wait for SPI receive buffer to be ready

    UCB0TXBUF = data;   // Send data to write

    while (!(UCB0IFG & UCRXIFG));   // Wait for SPI receive buffer to be ready

    // Disable SPI communication by resetting appropriate GPIO pins
    // ...
    P3OUT |= BIT4;
    SPIDelay();
}
unsigned short SPI_read(unsigned short regAddr)
{
    unsigned short data;

    // Enable SPI communication by setting appropriate GPIO pins
    // ...
    P3OUT &= ~BIT4;
    UCB0TXBUF = SPI_READ_COMMAND | (regAddr & 0x7F);   // Send SPI read command and register address

    while (!(UCB0IFG & UCRXIFG));   // Wait for SPI receive buffer to be ready

    data = UCB0RXBUF;   // Read received data

    // Disable SPI communication by resetting appropriate GPIO pins
    // ...
    P3OUT |= BIT4;
    SPIDelay();
    return data;
}
void SPIDelay()
{
    volatile unsigned int Delay_Count;

    for(Delay_Count = SPI_DELAY; Delay_Count > 0; Delay_Count--)
    {
        ;                                                                                // Wait for slave to initialize
    }
}
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    UCS_Init();                                 // Clock Initialization
    GPIO_Init();
//    ADC_Init();
    TimerB3_Init();
//    SPI_init();   // Initialize SPI module
//    SPI_DATA=SPI_read(0);
//    SPI_DATA=SPI_read(1);
//    SPI_DATA=SPI_read(2);
//    SPI_DATA=SPI_read(3);
//    SPI_DATA=SPI_read(4);
//    SPI_write(3, 0x02);
//    SPI_write(4, 0x02);
//    SPI_DATA=SPI_read(3);
    __enable_interrupt();
    //__no_operation();                             // For debug
    while(1)
    {
        //__bis_SR_register(GIE);

            TB3CCR1 = PWM_DUTY_CYCLE;
            TB3CCR2 = PWM_DUTY_CYCLE;
            TB3CCR3 = PWM_DUTY_CYCLE;
            TB3CCR4 = PWM_DUTY_CYCLE;
            TB3CCR5 = PWM_DUTY_CYCLE;
            TB3CCR6 = PWM_DUTY_CYCLE;


//        if(start_stop==1)
//        {
//            if(signal==1)
//            {
////                P1OUT |= BIT0;                      // Toggle P1.0 using exclusive-OR
//                P4OUT |= BIT7;
//                P4OUT &= ~(BIT5|BIT6);
//            }
//            else
//            {
////                P1OUT &= ~BIT0;
//                P4OUT &= ~BIT7;
//                P4OUT |= (BIT5|BIT6);
//            }
//
//        }
//        else
//        {
////                        P1OUT &= ~BIT0;
//                    P4OUT &= ~BIT7;
//                    P4OUT |= (BIT5|BIT6);
//        }
//        if(signal==1)
//
//        {
//            ADCMCTL0 = ADCINCH_7;         //   channel = A6 (Read the pot)
//            ADCCTL0 |= ADCENC;                             // Enable Conversions
//            ADCCTL0 |= ADCSC;                      // Start sampling of channels
//            while (ADCCTL1 & ADCBUSY_L)
//            {
//            };
//
//            ADCCTL0 &= ~ADCENC;                      // End sampling of channels
//            ADCCTL0 &= ~ADCSC;                            // Disable conversions
//
//            adc_value = ADCMEM0 & 0x0FFF; // Filter the result and read only last 12 bits because MSP430F5529 has 12bit ADC
//            //SensorlessTrapController.TargetDutyCycle >>= PWM_FACTOR; //2;PWM_FACTOR;                         //12 bit ADC result has to be scaled to 10 bit value
//            //
//            if (ADCMEM0 > 3500)
//                P4OUT |= BIT7;
//            else
//                P4OUT &= ~BIT6;
//            signal = 0;
//        }

        if (signal ==1)
        {
            P2OUT |= BIT4 | BIT5 | BIT6;
            P3OUT &= ~(BIT4 | BIT1 | BIT2 | BIT3);
//            P4OUT |= BIT7 | BIT5 | BIT6;
        }
        else
        {
            P2OUT &= ~(BIT4 | BIT5 | BIT6);
            P3OUT |= (BIT4 | BIT1 | BIT2 | BIT3);
//            P4OUT &= ~(BIT7 | BIT5 | BIT6);
        }

    }
}
#pragma vector=TIMER3_B0_VECTOR
__interrupt void TIMER3_B0_ISR(void)
{
    time_count++;
    if(time_count==12000)
    {
        signal^=1;
        time_count=0;

    }
}
#pragma vector=TIMER3_B1_VECTOR
__interrupt void TIMER3_B1_ISR(void)
{
}

#pragma vector = PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    if (P2IFG & BIT3)           //start stop with POT input
    {
//        start_stop ^= 1;
////        PWM_DUTY_CYCLE+=100;
////        if(PWM_DUTY_CYCLE>=1000)
////            PWM_DUTY_CYCLE=1000;
//        if(start_stop==1)
//        {
//            PWM_DUTY_CYCLE=300;
//            P4OUT |= BIT7;
//        }
//
//        else
//        {
//            PWM_DUTY_CYCLE=0;
//            P4OUT &= ~BIT7;
//        }

        P2IFG &= ~BIT3;
    }
//    if (P2IFG & BIT6)
//    {
////        count ++;
////        if(count==1)
////        {
////            P2OUT |= BIT0;
////            PWM_DUTY_CYCLE=300;
////        }
////        else
////        {
//////            P2OUT &= ~BIT0;
//////            PWM_DUTY_CYCLE =0;
//////            P4OUT &= ~BIT7;
////            PWM_DUTY_CYCLE += 100;
////        }
//        signal = 1;
//        P2IFG &= ~BIT6;
//    }
//    if (P2IFG & BIT1)           //start stop with POT input
//    {
//
//        PWM_DUTY_CYCLE += 100;
//        P2IFG &= ~BIT1;
//    }
//    if (P2IFG & BIT2)           //start stop with POT input
//    {
//
//        PWM_DUTY_CYCLE += 100;
//        P2IFG &= ~BIT2;
//    }
}

#pragma vector = PORT4_VECTOR
__interrupt void PORT4_ISR(void)
{
    if (P4IFG & BIT1)           //start stop with POT input
    {
        start_stop = 0;
        PWM_DUTY_CYCLE-=100;
        if(PWM_DUTY_CYCLE<=0)
            PWM_DUTY_CYCLE=0;
        //P2OUT &= ~BIT0;
        //P4OUT |= BIT7;
        //PWM_DUTY_CYCLE =0;

        P4IFG &= ~BIT1;
    }

}

