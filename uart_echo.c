//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2012-2016 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 2.1.3.156 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
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

//*****************************************************************************
//
// The UART 0 interrupt handler.
//
//*****************************************************************************
void
UART0IntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status. 1-Sends the UART base 2- The actual UART actions that trigger the interrupt.
    //
    ui32Status = ROM_UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts. 1-Sends the UART base 2- Sends the interruption status.
    //
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO. 
    //
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
			  // Send character from UART0 (PC) to UART4 (ESP8266)
        //
        ROM_UARTCharPutNonBlocking(UART4_BASE,
                                   ROM_UARTCharGetNonBlocking(UART0_BASE));

        
        // Blink the LED to show a character transfer is occuring.1-Port base 2- bit representation o pins 3- Value to write
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    }
}
//*****************************************************************************
//
// The UART 4 interrupt handler.
//
//*****************************************************************************
void
UART4IntHandler(void){
    uint32_t ui32Status;

    //
    // Get the interrrupt status. 1-Sends the UART base 2- The actual UART actions that trigger the interrupt.
    //
    ui32Status = ROM_UARTIntStatus(UART4_BASE, true);

    //
    // Clear the asserted interrupts. 1-Sends the UART base 2- Sends the interruption status.
    //
    ROM_UARTIntClear(UART4_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO. 
    //
    while(ROM_UARTCharsAvail(UART4_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
			  // Gets the character from UART4 (ESP8266)
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE,
                                   ROM_UARTCharGetNonBlocking(UART4_BASE));

    }	
	
}
//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}

//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************
int
main(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART pins.(UART0)
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Set GPIO C4 and C5 as UART pins.(UART4)
    //
    GPIOPinConfigure(GPIO_PC4_U4RX);
    GPIOPinConfigure(GPIO_PC5_U4TX);
    ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    //
    // Configure the UART0 for 115,200, 8-N-1 operation.
    //
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    //
    // Configure the UART4 for 115,200, 8-N-1 operation.
    //										 
    ROM_UARTConfigSetExpClk(UART4_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    //
    // Enable the UART0 interrupt.
    //
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    //
    // Enable the UART4 interrupt.
    //
    ROM_IntEnable(INT_UART4);
    ROM_UARTIntEnable(UART4_BASE, UART_INT_RX | UART_INT_RT);
    //
    // Prompt for text to be entered.
    //
    //UARTSend((uint8_t *)"\033[2JEnter text: ", 16);

    //
    // Loop forever echoing data through the UART.
    //
    while(1)
    {
    }
}
