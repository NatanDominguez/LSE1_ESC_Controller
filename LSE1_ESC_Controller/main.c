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
uint8_t state_l;
uint8_t phase;
uint8_t timerA_flag=0;
uint32_t ticks=40000;
uint8_t state;
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

        // Enable the GPIO port F / C / A
        //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){}
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)){}
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){}


        //
        //INICIALIZE BUTTON PF2
        //
        GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);

        //Habilita pull-up
        GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
        IntEnable(INT_GPIOF);
        GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
        GPIOIntRegister(GPIO_PORTF_BASE,IntPortFHandler); // Registrar la ISR para el puerto F
        GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);  // Habilitar interrupci�n para PF4


        // OUTPUT INIT
        /*
         * LA -> PC4 (OUTPUT)
         * LB -> PF2 (OUTPUT)
         * LC -> PA6 (OUTPUT)
         */

        // PC5
        GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);  //INICIAL STATE OFF

        // PF2
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);  //INICIAL STATE OFF

        // PA6
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);  //INICIAL STATE OFF

}




void IntPortFHandler(void)
{
    IntMasterDisable();
    uint32_t status = GPIOIntStatus(GPIO_PORTF_BASE,true);
    GPIOIntClear(GPIO_PORTF_BASE,GPIO_INT_PIN_4);

    if ((status & GPIO_PIN_4)==GPIO_PIN_4){
        state_l ^= 1;

        if(state == 1){
            PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);
            PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);
            PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, false);
        }
        else{

            PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);//A
            PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);//B
            PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);//C
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

    if(state == 0){
        ticks -= 10;
        TimerLoadSet(TIMER0_BASE, TIMER_A, ticks);
    }
    if(ticks <= 10000){
        ticks = 10000;
        state = 1;
        TimerLoadSet(TIMER0_BASE, TIMER_A, ticks);
    }

    IntMasterEnable();
}


void Timer1IntHandler(void){
    IntMasterDisable();
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //

    IntMasterEnable();
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

void Configure_Timer1(uint32_t ticks){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    //
    // Wait for the Timer0 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1))
    {
    }

    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
    //
    // Set the count time for the the one-shot timer (TimerA).
    //
    TimerLoadSet(TIMER1_BASE, TIMER_A, ticks);
    //
    //

    TimerIntRegister(TIMER1_BASE,TIMER_A,Timer1IntHandler);
    //
    // Enable the timers.


    // evento de interrupcion que salta cuando el timer cuenta hasta ticks
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    TimerEnable(TIMER1_BASE, TIMER_A);

    IntEnable(INT_TIMER1A);
    //

}

//PF4 GPIO INPUT CONFIG

//*****************************************************************************
void Configure_PWM(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
    {

    }

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1))
    {

    }


    GPIOPinConfigure(GPIO_PC5_M0PWM7);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinConfigure(GPIO_PA7_M1PWM3);

    GPIOPinTypePWM(GPIO_PORTC_BASE,GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTF_BASE,GPIO_PIN_3);
    GPIOPinTypePWM(GPIO_PORTA_BASE,GPIO_PIN_7);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);


    /*
    FASE A ------------- PC5 -> GEN 3   PWM0
    FASE B ------------- PF3 -> GEN 3   PWM1
    FASE C ------------- PA7 -> GEN 1   PWM1
    */

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 400);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 400);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 400);
    //
    // Set the pulse width of PWM0 for a 25% duty cycle.
    //
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 350);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 350);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 350);

    //
    // Start the timers in corresponding generators.
    //
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
    //
    // Enable the outputs.
    //


    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);


}


int main(void){

    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);


    IntMasterEnable();
    Configure_GPIO();
    Configure_PWM();
    Configure_Timer0(ticks);
    //
    //
    //

    /* INICIAL STATE (PWMA ON; PWMB OFF; PWMC OFF; AL OFF; BL ON; CL OFF) */
    phase = 1;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0);           // AL INICIAL STATE OFF
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);  // BL INICIAL STATE ON
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);  // CL INICIAL STATE OFF

    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);//A
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);//B
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT,false);//C


    while(1){

        if(phase > 6)   phase = 1; // ENSURE PERIODICITY

        //DE MOMENT OPEN-LOOP (NO ACTIVEM INTERRUPCIONS PER DETECTAR PAS-ZERO)

        if(phase == 1){         // PWMA -> ON; PWMC -> OFF
            PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);//A
            PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT,false);//C
        }
        else if(phase == 2){    // BL -> OFF; CL -> ON
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);

        }
        else if(phase == 3){    // PWMA -> OFF; PWMB -> ON
            PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);//A
            PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);//B
        }
        else if(phase == 4){    // AL -> ON; CL -> OFF
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);

        }
        else if(phase == 5){    // PWMB -> OFF; PWMC -> ON
            PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);//B
            PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT,true);//C
        }
        else{   //phase = 6 ||     AL -> OFF; BL -> ON
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        }

    }
}
