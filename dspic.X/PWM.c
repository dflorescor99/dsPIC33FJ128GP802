/*
 * File:   PWM.c
 * Author: andres 2
 *
 * Created on 15 de abril de 2021, 08:47 PM
 */

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

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Mode (Primary Oscillator (XT, HS, EC) w/ PLL)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Source (XT Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

#define FOSC 11059200*3
#define FCY FOSC/2
#define PERIODO 0.00395
#define PRESCALER 0
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
void InitT2(void);
void InitPWM(void);

unsigned int signal=0;
//uint16_t PRT2=0;

int main(void)
{
    uint16_t duty=0;
    InitClock();                 // Inicialización del PLL
    while(OSCCONbits.LOCK != 1); // Un tiempo de espera para que el PLL se enganche
    //Init_UART1();                // Inicialización de la UART @9600-8N1
    InitT2();                 // Inicialización TIMER2
    InitPWM();
    while (1)
    {
        while(duty<=PR2)
        {
            OC1RS=duty;
            delay_ms(10);
            duty=duty+1;
        }
        duty = 0;
        //WriteUART1(signal);
    }
    return 0;
}

//void __attribute__((__interrupt__,no_auto_psv)) _T2Interrupt(void)
//{
//    IFS0bits.T2IF=0;
//    IEC0bits.T2IE=0;
//    signal = ~signal;
//    IEC0bits.T2IE=1;
//}

void InitT2(void)
{
    T2CONbits.T32=0; //SI QUEREMOS UN TIMER DE 32 O 16 BITS
    T2CONbits.TCKPS=PRESCALER; //ELEGIMOS EL PRESCALER
    TMR2=0;
    PR2=63;
//    if ( ((int)(PERIODO*FOSC/(1)-1) <0xFFFF && ((int)(PERIODO*FOSC/1))-1)> 0)
//    {
//        PR2=(int)(PERIODO*FOSC/(1))-1;
//    }
    // SI EL PERIODO ES MUY GRANDE O MUY CHICO PARA LA CONFIGURACION QUE TENEMOS
    //CARGA EL REGISTRO CON 0xFF
    T2CONbits.TCS=0; //ELEGIMOS QUE QUEREMOS SEGUIR LAS INSTRUCCIONES INTERNAS COMO ORIGEN DEL TIMER
    T2CONbits.TON=1;//ENCENDEMOS EL TIMER2
}

void InitPWM(void)
{
    OC1R=0;                  //TON PARA EL PRIMER CICLO
    OC1RS=0;                 //REPRESENTA EL TIEMPO QUE EL PWM ESTA EN ALTO
                             //EL DUTY CYCLE ES OC1RS/PRx
    OC1CONbits.OCM=0b110;    //ELEGIMOS QUE FUNCIONE COMO PWM
    OC1CONbits.OCTSEL=0;     //ELEGIMOS EL TIMER 2 COMO LA FUENTE DEL PWM
    RPOR7bits.RP14R=0b10010; //OC1 SALE POR LA PATA 25
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


