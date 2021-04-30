/*
 * File:   timer.c
 * Author: andres 2
 *
 * Created on 15 de abril de 2021, 12:25 AM
 */

#include "xc.h"
#include <p33FJ128GP802.h>
#include <stdio.h>
#include <stdlib.h>

_FOSCSEL(FNOSC_FRCPLL);
_FOSC(FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMD_NONE);
_FWDT(FWDTEN_OFF);

#define FOSC 7372800*3
#define FCY FOSC/2
#define BAUDRATE 9600
#define BRGVAL ((FCY/BAUDRATE)/16)- 1
#define Nop()   __builtin_nop()


/*
FOSC = 7.3728 × 12 ÷ 2 ÷ 2 = 22.1184MHz
FCY = 22.1184 ÷ 2 = 11.0592 MHz
 * Fulero, pero para UART funciona excelente
 * 11059200/16/9600-1=71 ¡¡¡Da un entero!!!
 * 0% de error en la transmision
 */

void InitClock();
void Init_UART1();
void WriteUART1(unsigned int data);
void delay_us(unsigned int delay_count);
void delay_ms(unsigned long delay_count);
void writestr(char str[20]);
void conftimer(void);

unsigned int signal=0;

int main(void)
{
    InitClock(); // Inicialización del PLL
    Init_UART1(); // Inicialización de la UART @9600-8N1
    conftimer();  //inicialización TIMER2
    IFS0bits.T2IF=0;
    IEC0bits.T2IE=1;
    while (1)
    {
        while(IFS0bits.T2IF==0);
        WriteUART1(signal);
    }
    return 0;
}

void __attribute__((__interrupt__,no_auto_psv)) _T2Interrupt(void)
{
    IFS0bits.T2IF=0;
    IEC0bits.T2IE=0;
    signal = ~signal;
    IEC0bits.T2IE=1;
}

void conftimer(void)
{
    T2CONbits.T32=0; //SI QUEREMOS UN TIMER DE 32 O 16 BITS
    T2CONbits.TCKPS=0b11; //ELEGIMOS EL PRESCALER
    TMR2=0;
    PR2=43199;//  T2PERIODO=[(PR2) + 1] ? TCY ? (TMR2 Prescale Value)
        //        00 1:1
        //        01 1:8
        //        10 1:64
        //        11 1:256
    T2CONbits.TCS=0; //ELEGIMOS QUE QUEREMOS SEGUIR LAS INSTRUCCIONES INTERNAS COMO ORIGEN DEL TIMER
    T2CONbits.TON=1;//ENCENDEMOS EL TIMER2
}

void InitClock()
{
    PLLFBD = 10; // M = 12
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    CLKDIVbits.PLLPRE = 0; // N1 = 2
    OSCTUN = 0;
    RCONbits.SWDTEN = 0;
}

void Init_UART1()
{
    AD1PCFGL = 0xFFFF; // all pins as digital
    TRISA = 0;
    TRISB = 0;
    LATA = 0;
    LATB = 0;
    RPOR7bits.RP15R = 0b00011; //U1TX to RP15 = pin 26 
    U1MODEbits.STSEL = 0; // 1-stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U1MODEbits.ABAUD = 0; // Autobaud Disabled
    U1BRG = BRGVAL; // BAUD Rate Setting for 9600

    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable Tx
    //IEC0bits.U1RXIE = 1; // enable UART1 receiver ISR
}

void WriteUART1(unsigned int data)
{
    while (U1STAbits.TRMT == 0);
    if (U1MODEbits.PDSEL == 3)
    {
        U1TXREG = data;
    }
    else
    {
        U1TXREG = data & 0xFF;
    }
}

void delay_us(unsigned int delay_count)
{
    delay_count = delay_count * FCY / 1000000;

    while (delay_count--);
}

void delay_ms(unsigned long delay_count)
{
    delay_count = delay_count * FCY / 1000;
    while (delay_count--);
}

void writestr(char str[20])
{
    int i = 0;
    while (str[i] != '\0')
    {
        U1TXREG = str[i];
        delay_us(90);
        i++;
    }
    U1TXREG = '\n';
    delay_us(90);
}
