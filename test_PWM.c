#include "tm4c123gh6pm.h"

/**
 * main.c
 */

//**********Function Declarations**********//
void init_PWM (void);

//**********main() function***********//
int main(void)
{
	while (1)
	{

	}
}

//**********Function Definitions**********//

//Initialize PWM Generator 0 with 25-kHz frequency, a 25% duty cycle
//on the MnPWM0 pin (and a 75% duty cycle on the MnPWM1 pin, for testing).
// Guide: p.1239
void init_PWM (void)
{
    SYSCTL_RCGC0_R = 0x00100000; // Step 1: Enable the PWM Generator 0 (p.456)
    SYSCTL_RCGC2_R |= 0x00000020; // Step 2: Enable clock to GPIO Port F module (p.464)

    // Step 3: Initialize GPIOAFSEL to enable the appropriate pins for their alternate function, which is PB6,7 (p.1233)
    GPIO_PORTB_AFSEL_R |= 0x12;

    // Step 4: Configure the PMCn fields in the GPIOPCTL register to assign the PWM signals to the appropriate
    // pins (see page 688 and Table 23-5 on page 1351).
    GPIO_PORTB_PCTL_R |= 0x12;  // Turn on pin 1 and 4 of Port B

    // Step 5: Configure the Run-Mode Clock Configuration (RCC) register in the System Control module
    // to use the PWM divide (USEPWMDIV) and set the divider (PWMDIV) to divide by 2 (000) (p.254)
    SYSCTL_RCC_R |= 0x00100000; //Set the USEPWMDIV bit
    SYSCTL_RCC_R &= ~0x000E0000;// Clear the PWMDIV bits for division by 2 (leave reserve bit 16 out)

    // Step 6: Configure the PWM generator for countdown mode with immediate updates to the parameters.
    PWM0_CTL_R = 0x00000000;    // Clear MODE bit (1) to configure PWM gen for Count-Down mode

    PWM0_0_GENA_R = 0x0000008C; // Set ACTLOAD bits to so that pwmA will be driven High when the counter matches the value in PWM0LOAD
                                // Set bit 7 (ACTCMPAD) for Comparator A Down so pwmA will be driven Low when counter matches comparator A

    PWM0_0_GENB_R = 0x0000080C; // Set ACTLOAD bits to so that pwmB will be driven High when the counter matches the value in PWM0LOAD
                                // Set bit 7 (ACTCMPBD) for Comparator A Down so pwmB will be driven Low when counter matches comparator B

    // Step 7: Set the period. For a 25-KHz frequency, the period = 1/25,000, or 40 microseconds. The PWM
    // clock source is 10 MHz; the system clock divided by 2. Thus there are 400 clock ticks per period.
    // Use this value to set the PWM0LOAD register. In Count-Down mode, set the LOAD field in the
    // PWM0LOAD register to the requested period minus one.
    PWM0_0_LOAD_R = 0x0000018F; // Set value 399 into LOAD
    // Step 8:
    PWM0_0_CMPA_R = 0x0000012B; // Set value 299 into CMPA for 25% DutyCycle:   400/(1-0.25)
    // Step 9:
    PWM0_0_CMPB_R = 0x00000063; // Set value 99 into CMPB for 75% DutyCycle:    400/(1-0.75)
    // Step 10:
    PWM0_CTL_R = 0x000000001;   // Enable the PWM generation block
    // Step 8:
    PWM0_ENABLE_R = 0x00000003; // The generated pwm0A' and 0B' signals are passed to the M0PWM0 and M0PWM1 pins, respectively
}
