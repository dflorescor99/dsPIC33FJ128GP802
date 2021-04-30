/*
 * File:   newmainXC16.c
 * Author: andres 2
 *
 * Created on 13 de abril de 2021, 07:06 PM
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
// estados [xn-2 xn-1 xn yn yn-1 yn-2]
#define xn states[2]
#define xn1 states[1]
#define xn2 states[0]
#define yn  states[3]
#define yn1 states[4]
#define yn2 states[5]

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

int main(void)
{
    InitClock(); // Inicialización del PLL
    Init_UART1(); // Inicialización de la UART @9600-8N1
    float states[6] = {0, 0, 0, 0, 0, 0};
    // states [xn-2 xn-1 xn yn yn-1 yn-2] Estados de la entrada y salida
    xn = 1.0; // valor inicial del escalón
    char *ptr; // puntero para que apunte al valor float a plotear
    int x;
    ptr = (char *) &yn;

    while (1)
    {
        yn = 1.9842 * yn1 - yn2 + 0.0013 * xn1 + 0.0013 * xn2; // ecuación del seno
        yn2 = yn1;
        yn1 = yn;
        xn2 = xn1;
        xn1 = xn;

        ptr = (char *) &yn;
        putc(0x03,stdout);
        for (x = 0; x < sizeof (xn); x++)
            WriteUART1(*ptr++);
        putc(0xFC,stdout);

    }

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
