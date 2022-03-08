#include "msp.h"

//assumes system clock is at 48 MHz
//input dividers on Timer A = div 1, input to Timer A = 48 MHz
//Divide by 512 -> PWM frequency = 93.75 kHz

#define SMCLK   BIT0        //port 7.0
#define A8_IN   BIT5        //port 4.5 ADC input channel 8

#define PWM_PERIOD  512     //9-bit DAC (2^9 = 512)
#define ADCSCALE    5       //14-bit ADC; reduce by 5 bits to match DAC

#define SR_12kHz    4096
#define FSAMP       SR_12kHz //sampling frequency

#define N       3       // Number of things to mult by
#define FACT    1374    // Factor that gets divided by
#define DC      256     // offset

void main(void)
{
    volatile unsigned int ADC_In=0;

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    //P7->DIR |= SMCLK;
    //P7->SEL0 |= SMCLK;

    P6->DIR |= BIT0;        //sampling flag P6.0
    P6->DIR |= BIT1;        // Set 6.1 up to measure sampling time
    P6->OUT = 0;            //for measuring sampling frequency and ADC conversion time

    P5->DIR |= BIT6;        //PWM output P5.6
    P5->SEL0 |= BIT6;
    P5->SEL1 &= ~BIT6;

    P4->SEL0 |= A8_IN;      //use analog input #8 = P4.5 for ADC input
    P4->SEL1 |= A8_IN;

    //Disable conversion while setting up ADC registers
    ADC14->CTL0 &= ~ADC14_CTL0_ENC;

    ADC14->CTL0 = ADC14_CTL0_SHP | ADC14_CTL0_SSEL__SMCLK | ADC14_CTL0_DIV__4
            | ADC14_CTL0_SHT0__16 | ADC14_CTL0_ON;

    //Memory Control Register 0 to receive ADC samples on input #8
    ADC14->MCTL[0] = 8u;

    //Sets up timer to creating ADC sampling clock
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__1 | TIMER_A_CTL_MC__CONTINUOUS;
    TIMER_A0->CCR[0] = FSAMP-1;
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;

    //Sets up timer for PWM output
    TIMER_A2->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__1 | TIMER_A_CTL_MC__UP;
    TIMER_A2->CCR[0] = PWM_PERIOD-1;
    TIMER_A2->CCR[1] = PWM_PERIOD/2;
    TIMER_A2->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
    //TIMER_A2->CTL |= TIMER_A_CTL_IE;

    NVIC_EnableIRQ(TA0_0_IRQn);
    //NVIC_EnableIRQ(TA2_N_IRQn);
    __enable_interrupts();


    // Make x[N] of array length size(h_lpf) = {0}
    int val[4*N] = { 0 };
    // old int h[N] = { 0, 1634, -764, 100, 200, 100 };
    int h[2*N] = { 0, -764, 1634, 100, 100, 200 };
    // Variable that gets assigned sum which will get assigned to y[n]
    int sum=0;
    int i;
    int counter = 0;

    while (1)  {

        if ((ADC14->IFGR0 & ADC14_IFGR0_IFG0) != 0)  {

            P6->OUT = BIT1; //Turn high when starting to process

            ADC_In = ((ADC14->MEM[0]) >> ADCSCALE);
            val[counter+2*N] = val[counter+3*N] = ADC_In-DC; //Store x value

            // Convolve all values except for the very middle value that gets calc'd last
            for (i=0; i<N; i++) {
                sum += (h[i] * val[i+counter]) + (h[i+N] * val[i+counter+2*N]);
            }

            // Now put sum in PWM
            TIMER_A2->CCR[0] = 0;               //disable timer
            TIMER_A2->CCR[1] = (sum/FACT) + DC;    //load new duty cycle value
            TIMER_A2->CCR[0] = PWM_PERIOD-1;    //enable timer

            val[counter] = val[counter+N] = sum/FACT; // Store y value

            //Increment and rotate counter if need be
            counter += 1;
            if(counter>=N) counter = 0;

            sum = 0; // Reset sum

            P6->OUT = 0; // Turn low when done processing

        }

    }

}

void TA0_0_IRQHandler(void)  {
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    TIMER_A0->CCR[0] += FSAMP;
    P6->OUT |= 0x01;
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
}
