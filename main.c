#include "tm4c123gh6pm.h"

/**
 * main.c
 */

//**********Function Declarations**********//
void init_PWM (void);
void init_timers (void);
void init_gpio(void);

//**********main() function***********//
int main(void)
{
    init_PWM();
    init_gpio();
    init_timers();
	while (1)
	{
	}
}

//**********Function Definitions**********//

//Initialize PWM Generator 2 (M1PWM5) with 1-kHz frequency, a 10% duty cycle for M1PWM5 to control the RED LED
// Assume the system clock to be 16 MHz
// Guide: p.1239
void init_PWM (void)
{
    SYSCTL_RCGCPWM_R |= 0x2;   // Enable and provide a clock to PWM module 1 in Run mode,  p.354
    SYSCTL_RCGC0_R |=  0x100000; // Step 1: Enable the PWM Generator's clock gating(p.456) so that it receives a clock and functions
    SYSCTL_RCGC2_R |= 0x020; // Step 2: Enable clock for GPIO Port F module (p.464)

    // Step 3: Initialize GPIOAFSEL to enable the appropriate pins for their alternate function, which is PF1 (p.1233)
    GPIO_PORTF_AFSEL_R |= 0x02;

    // Step 4: Configure the PMCn fields in the GPIOPCTL register to assign the PWM signals to the appropriate
    // pins (see page 688 and Table 23-5 on page 1351).
    GPIO_PORTF_PCTL_R |= (GPIO_PORTF_PCTL_R & 0xFFFFFF0F) + 0x50;  // pin 4-6 enabled for MUX control 1, p.689

    // Step 5: Configure the Run-Mode Clock Configuration (RCC) register in the System Control module
    // to use the PWM divide (USEPWMDIV) and set the divider (PWMDIV) to divide by 64 for a system clock frequency of 250 KHz (p.254)
//    SYSCTL_RCC_R &= ~0x000E0000;// Clear the PWMDIV bits for division by 64 (leave reserve bit 16 out)
    SYSCTL_RCC_R &= ~0x00100000; // Clear the USEPWMDIV bit so that the system clock is the source for the PWM clock

    // Step 6: Configure the PWM generator for countdown mode with immediate updates to the parameters.
    PWM1_2_CTL_R &= ~0x7FFFF;    // Clear MODE bit (1) to configure PWM generator for Count-Down mode

    PWM1_2_GENB_R = 0x0000080C; // Set ACTLOAD bits to so that pwmB will be driven High when the counter matches the value in PWM1LOAD
                                // Set ACTCMPBD bits (11:10) for Comparator B Down so pwmB will be driven Low when counter matches comparator B

    // There is not clock divider, so system clock is 16 MHz.
    PWM1_2_LOAD_R = 16000 - 1;
    // Step 9:
    PWM1_2_CMPB_R = 14400 - 1; // Set value 224 into CMPB for 10% DutyCycle:  16000*(1 - 10/100) = 14400
    // Step 10:
    PWM1_2_CTL_R = 0x00000001;   // Enable the PWM generation block
    // Step 11:
    PWM1_ENABLE_R = 0x00000020; // The generated pwm2B' signal is passed to the MnPWM5
}

void init_gpio(void)
{
    volatile unsigned long delay_clk;
    SYSCTL_RCGC2_R |= 0x00000020;
    delay_clk = SYSCTL_RCGC2_R;

    GPIO_PORTF_PCTL_R |= 0x50;

    //Unlock and set SW2
    GPIO_PORTF_LOCK_R |= 0x4C4F434B;        // Unlock the GPIOCR register for modification
    GPIO_PORTF_CR_R |= 0x01;                // Un-commit GPIO pin PF[0] for modification of SW2

    GPIO_PORTF_DEN_R |= 0x1F;   //Enable Pins 0-4 in Port F
    GPIO_PORTF_DIR_R &= ~0x11;  // Set Pins 0, 4 as Input
    GPIO_PORTF_DIR_R |= 0x0E;   //Set Pins 1-3 as Outputs
    GPIO_PORTF_PUR_R |= 0x11;   //Enable Pull-Up Resistor for SW1 and SW2
    GPIO_PORTF_AFSEL_R &= ~0x1D; //Disable alternate functions on PF0-PF4, except PF1
    GPIO_PORTF_AFSEL_R |= 0x02; // Enable alternate functions on PF1

            //Lock SW2
    GPIO_PORTF_CR_R &= ~0x1;                // Re-commit GPIO in PF[0]
    GPIO_PORTF_LOCK_R |= 0x1;               // Lock the GPIOCR register to prevent further modification

    GPIO_PORTF_DATA_R |= 0x02;
}


void init_timers (void)
{
    //***Setting Timer0 to 32-bit Periodic Mode***
                                //and interrupt number 19 (bit in interrupt registers)  (p.142)
    SYSCTL_RCGCTIMER_R |= 0x03;       //Enable and provide a clock to 16/32-bit general-purpose timer module 0 and 1 in Run mode, p.337
    TIMER0_CTL_R &= ~0x1;            //Ensure the timer is disabled before making changes, p.722, step 1
    TIMER0_CFG_R &= 0x11111000;     //Selects the 32-bit timer configuration, p.722 Step 2
    TIMER0_TAMR_R |= 0x02;          //Configure the TAMR field in the GPTMTnMR for Periodic mode, p.722 step 3
    TIMER0_TAMR_R &= ~(0x08);       //Set direction of counter (count down), step 4
    TIMER0_TAILR_R = 0x00F42400;   //Load the start value into the GPTM Timer n Interval Load Register (GPTMTAILR), step 5
                                    // Count down from 16,000,000 in a 16MHz clock for 1 second to trigger ISR
    // Add interrupts
    TIMER0_IMR_R |= 0x01;            //Set Interrupt Mask on Overflow (or timeout)
    //Enable the NVIC register to get interrupts to jump to vector
    NVIC_EN0_R |= 0x00080000;   //Timer 0A is interrupt vector 35. (Table 2-9: Interrupt (p.104))
    TIMER0_ICR_R |= 0x01;           // Clear the ISR flag prior to enabling the timer
    TIMER0_CTL_R |= 0x01;            //Set the TAEN bit in TIMER0_CTL register to enable the timer and start counting, step 7

    //Timer1A - for switch debouncing
    TIMER1_CTL_R &= ~0x01;          // Disable timer by clearing TnEn bit
    TIMER1_CFG_R = ~0xFF;           // Write CFG with a value of 0x0000.0000
    TIMER1_TAMR_R |= 0x02;          // 0x02 for periodic counter
    TIMER1_TAMR_R &= ~(0x08);       // Set direction of counter (count down)
    TIMER1_TAILR_R = 0x00186A00;    // Preload value 1.6 million. Will take 0.1 second to count.

    TIMER1_IMR_R |= 0x01;
    TIMER1_CTL_R |= 0x01;
}

