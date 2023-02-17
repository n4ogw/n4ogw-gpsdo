/**
  Generated Pin Manager File

  Company:
    Microchip Technology Inc.

  File Name:
    pin_manager.c

  Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC18F27Q84
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.36 and above
        MPLAB             :  MPLAB X 6.00

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "pin_manager.h"





void PIN_MANAGER_Initialize(void)
{
    /**
    LATx registers
    */
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;

    /**
    TRISx registers
    */
    TRISE = 0x08;
    TRISA = 0xFF;
    TRISB = 0xF7;
    TRISC = 0x95;

    /**
    ANSELx registers
    */
    ANSELC = 0x02;
    ANSELB = 0xC3;
    ANSELA = 0xFE;

    /**
    WPUx registers
    */
    WPUE = 0x00;
    WPUB = 0x00;
    WPUA = 0x00;
    WPUC = 0x00;

    /**
    ODx registers
    */
    ODCONA = 0x00;
    ODCONB = 0x00;
    ODCONC = 0x00;

    /**
    SLRCONx registers
    */
    SLRCONA = 0xFF;
    SLRCONB = 0xC1;
    SLRCONC = 0x02;

    /**
    INLVLx registers
    */
    INLVLA = 0xFF;
    INLVLB = 0xFB;
    INLVLC = 0x32;
    INLVLE = 0x08;





   
    
	
    CLCIN7PPS = 0x0D;   //RB5->CLC8:CLCIN7;    
    U2RXPPS = 0x12;   //RC2->UART2:RX2;    
    RC3PPS = 0x23;   //RC3->UART2:TX2;    
    CLCIN6PPS = 0x0C;   //RB4->CLC1:CLCIN6;    
    CLCIN3PPS = 0x10;   //RC0->CLC7:CLCIN3;    
    U1RXPPS = 0x17;   //RC7->UART1:RX1;    
    CLCIN0PPS = 0x00;   //RA0->CLC3:CLCIN0;    
    SMT1WINPPS = 0x10;   //RC0->SMT1:SMT1WIN;    
    RB3PPS = 0x07;   //RB3->CLC7:CLC7;    
    CLCIN2PPS = 0x0A;   //RB2->CLC7:CLCIN2;    
    SMT1SIGPPS = 0x0A;   //RB2->SMT1:SMT1SIG;    
    RC5PPS = 0x18;   //RC5->PWM1_16BIT:PWM11;    
    RC6PPS = 0x20;   //RC6->UART1:TX1;    
}
  
void PIN_MANAGER_IOC(void)
{   
}

/**
 End of File
*/