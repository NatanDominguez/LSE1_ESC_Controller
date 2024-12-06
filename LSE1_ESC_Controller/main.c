//*****************************************************************************
//
// hello.c - Simple hello world example.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/debug.h>
#include <driverlib/fpu.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <driverlib/sysctl.h>
#include <driverlib/pwm.h>
#include <driverlib/interrupt.h>
#include <inc/hw_ints.h>
#include <driverlib/timer.h>
#include <driverlib/sysctl.h>
#include <inc/hw_ints.h>

//*****************************************************************************
//
//
//
//*****************************************************************************
uint8_t state;
uint8_t phase;
uint8_t timerA_flag=0;
//uint8_t variable_random;

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Hello World (hello)</h1>
//!
//! A very simple ``hello world'' example.  It simply displays ``Hello World!''
//! on the UART and is a starting point for more complicated applications.
//!
//! UART0, connected to the Virtual Serial Port and running at
//! 115,200, 8-N-1, is used to display messages from this application.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*************************************

void IntPortFHandler(void);

void Configure_GPIO(void)
{

        // Enable the GPIO port that is used for the on-board LED.
        //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){}
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)){}
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){}

        //
        // Enable the GPIO pins for button PF2
        //
        GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);

        //GPIOIntEnable(GPIO_PORTF_BASE,GPIO_PIN_4);

        //Habilita pull-up

        GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

        IntEnable(INT_GPIOF);

        GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
        GPIOIntRegister(GPIO_PORTF_BASE,IntPortFHandler); // Registrar la ISR para el puerto F
        GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);  // Habilitar interrupción para PF4


        // OUTPUT INIT
        /*
         * LA -> PC5 (OUTPUT)
         * LB -> PF2 (OUTPUT)
         * LC -> PA7 (OUTPUT)
         */

        // PC5
        GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5);
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);  //INICIAL STATE OFF

        // PF2
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);  //INICIAL STATE OFF

        // PA7
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);  //INICIAL STATE OFF

}




void IntPortFHandler(void)
{
    IntMasterDisable();
    uint32_t status = GPIOIntStatus(GPIO_PORTF_BASE,true);
    GPIOIntClear(GPIO_PORTF_BASE,GPIO_INT_PIN_4);

    if ((status & GPIO_PIN_4)==GPIO_PIN_4){
        state ^= 1;

        if(state == 1){
            PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), false);
        }
        else{
            PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);
        }
    }
    IntMasterEnable();
}





/*****************************************************************************

TIMER


****************************************************************************/

//*****************************************************************************
void Timer0IntHandler(void){
    IntMasterDisable();
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    phase++;

    IntMasterEnable();
}


void Timer1IntHandler(void){
}


void Configure_Timer0(uint32_t ticks){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    //
    // Wait for the Timer0 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
    {
    }

    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
    //
    // Set the count time for the the one-shot timer (TimerA).
    //
    TimerLoadSet(TIMER0_BASE, TIMER_A, ticks);
    //
    //

    TimerIntRegister(TIMER0_BASE,TIMER_A,Timer0IntHandler);
    //
    // Enable the timers.


    // evento de interrupcion que salta cuando el timer cuenta hasta ticks
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); 
    
    TimerEnable(TIMER0_BASE, TIMER_A);

    IntEnable(INT_TIMER0A);
    //

}

//PF4 GPIO INPUT CONFIG

//*****************************************************************************
void Configure_PWM(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
    {

    }


    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
        {

        }


    GPIOPinConfigure(GPIO_PB7_M0PWM1);

    GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_7);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 400);
    //
    // Set the pulse width of PWM0 for a 25% duty cycle.
    //
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 350);
    //
    // Set the pulse width of PWM1 for a 75% duty cycle.
    //
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 50);
    //
    // Start the timers in generator 0.
    //
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    //
    // Enable the outputs.
    //
    PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);



}


int main(void){
    //volatile uint32_t ui32Loop;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
     FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);


    IntMasterEnable();
    Configure_PWM();
    Configure_GPIO();
    uint32_t ticks=10000;
    Configure_Timer0(ticks);
    //
    //
    //

    /* INICIAL STATE (PWMA ON; PWMB OFF; PWMC OFF; AL OFF; BL ON; CL OFF) */
    phase = 1;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0);           // AL INICIAL STATE OFF
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);  // BL INICIAL STATE ON
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);  // CL INICIAL STATE OFF

    while(1){

        if(phase > 6)   phase = 1; // ENSURE PERIODICITY

        //DE MOMENT OPEN-LOOP (NO ACTIVEM INTERRUPCIONS PER DETECTAR PAS-ZERO)
        
        if(phase == 1){         // PWMA -> ON; PWMC -> OFF

        }
        else if(phase == 2){    // BL -> OFF; CL -> ON
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
        }
        else if(phase == 3){    // PWMA -> OFF; PWMB -> ON

        }
        else if(phase == 4){    // AL -> ON; CL -> OFF
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);

        }
        else if(phase == 5){    // PWMB -> OFF; PWMC -> ON

        }
        else{   //phase = 6 ||     AL -> OFF; BL -> ON
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        }

    }
}
