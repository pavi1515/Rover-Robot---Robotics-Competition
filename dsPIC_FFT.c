/*
 * File:   main.c
 * Author: brennanleblanc
 *
 * Created on February 25, 2024, 6:21 PM
 */


// DSPIC33EP32MC202 Configuration Bit Settings

// 'C' source line config statements

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25           // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)

// FOSCSEL
#pragma config FNOSC = FRCDIVN          // Oscillator Source Selection (Internal Fast RC (FRC) Oscillator with postscaler)
#pragma config PWMLOCK = ON             // PWM Lock Enable bit (Certain PWM registers may only be written after key sequence)
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <dsp.h>
#include <p33EP32MC202.h>
#define ARR_LIM 256 // 256 Sample FFT

int* adcIn;
int arrI = 0;
int high = 0;

void main(void) {
    adcIn = malloc(256*sizeof(int));
    // Baud: 9600, type: 8N1
    TRISBbits.TRISB15 = 1;
    RPINR18 = 0b0101111;
    RPOR4bits.RP43R = 0b000001;
    U1BRG = 95;
    U1STAbits.UTXEN = 1;
    U1MODEbits.BRGH = 1;
    U1MODEbits.UARTEN = 1;
    
    INTCON2bits.GIE = 1;
    IEC0bits.U1RXIE = 1;
    IFS0bits.U1RXIF = 0;
    
    
    while (1);
    
    return;
}

void doFFT() {
    fractcomplex* inputData = malloc(256*sizeof(fractcomplex));
    int i = 0;
    for (i = 0; i < 256; i++)
        inputData[i].real = adcIn[i] * (3.3 / 1024);
    fractional* coeffs = malloc(256*sizeof(fractional));
    coeffs = SquareMagnitudeCplx(256, inputData, coeffs);
    int16_t fftMinBin = 0;
    fractional fftMin; 
    fftMin = VectorMin(128, coeffs, &fftMinBin);
    
    uint16_t fmin = fftMinBin * (250000/256);
    
    U1TXREG = fmin & 255;
    while (U1STAbits.UTXBF);
    U1TXREG = (fmin >> 8) & 255;
    free(inputData);
    free(coeffs);
    return;
}

// ISR for RX in UART1
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt() {
    // Since 10-bit ADC, lower 8 will be sent then the upper 2
    if (high) {
        adcIn[arrI] += (U1RXREG & 255) << 8; // Upper 2-bits
        arrI++;
        high--;
    } else {
        adcIn[arrI] = U1RXREG & 255; // Lower 8-bits
        high++;
    }
    if (arrI == 256)
        doFFT();
    IFS0bits.U1RXIF = 0; // Clear interrupt
}

