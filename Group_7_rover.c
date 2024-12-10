 
// PIC16F18855 Configuration Bit Settings
 
// 'C' source line config statements
 
// CONFIG1
 
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
 
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
 
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
 
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
 
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (FSCM timer disabled)
 
// CONFIG2
 
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
 
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
 
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
 
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
 
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
 
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
 
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
 
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)
 
// CONFIG3
 
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
 
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
 
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
 
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)
 
// CONFIG4
 
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
 
#pragma config SCANE = available// Scanner Enable bit (Scanner module is available for use)
 
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)
 
// CONFIG5
 
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
 
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)
 
// #pragma config statements should precede project file includes.
 
// Use project enums instead of #define for ON and OFF.
 
#define _XTAL_FREQ 32000000
 
#include <xc.h>
 
 
uint16_t halleffectValue = 0;
 
int doneWithHall = 0;
 
int taskDecider = 0;
 
int done = 0;
 
int doneWithFFT = 0;
 
int count = 0;
 
int buffer_value[26];

int buffer_pcls_response[12];
 
uint16_t x = 0;


void getUserDataResponse() {
    //getUserDataSenderUser();
    int var = 0;
    while (var < 26){
        while (!PIR3bits.RCIF); // RCIF 0 indicates that the EUSART buffer is empty. 
        buffer_value[var] = RC1REG;
        var++;
    }
 
}
void getUserDataSenderUser() {
    //TX1STAbits.TXEN = 1;
    int signal[] = {0xFE, 0x19, 0x01, 0x05, 0x00, 0x00}; //data
 
 
    // while loop to send the data
    for (int i = 0; i < 6; i++) {
 
        while (!TX1STAbits.TRMT);
 
        TX1REG = signal[i];
 
    }
    getUserDataResponse(); // calls this method to store the data from pcls into buffer_value
    //TX1STAbits.TXEN = 0;

 
}
 
void __interrupt() myISR() {
 
    
    //
    if (PIR1bits.ADIF == 1) {
 
        if (taskDecider == 0) {
 
            halleffectValue = (ADRESH << 8) + ADRESL;
 
            //TRISBbits.TRISB4 = 1;
 
            if (halleffectValue > 0x50) {
 
                transmitCustomLaserSettings(); //Doesn't exist
 
                // How to get the code.
 
            } else {
 
                //LATAbits.LATA2 = 0;
 
            }
 
            PIR1bits.ADIF = 0;
 
            ADCON0bits.ADGO = 1;
 
            doneWithHall = 1;
 
        } else {
 
            TX1REGbits.TX1REG = ADRESL;
 
            while (!TX1STAbits.TRMT);
 
            TX1REGbits.TX1REG = ADRESH;
 
            if (done == 256) {
 
                doneWithFFT = 1;
 
                ADCON0bits.ADON = 0;
 
            }
 
            ADCON0bits.ADGO = 1;
 
            PIR1bits.ADIF = 0;
 
        }
 
    }

 
}
 
 
void setMotorSettings(int motionA, int pwmA, int motionB, int pwmB) {
 
    int signal[] = {0xFE, 0x19, 0x01, 0x06, 0x04, 0x00, motionA, pwmA, motionB, pwmB};
 
    int count = 0;
 
    for (count = 0; count < 10; count++) {
 
        while (!TX1STAbits.TRMT);
 
        TX1REG = signal[count];
 
    }
 
}
 
void shootLaserSettings() {
    int signal[] = {0xFE, 0x19, 0x01, 0x09, 0x01, 0x00, 0x02};
 
    int count = 0;
 
    for (count = 0; count < 7; count++) {
 
        while (!TX1STAbits.TRMT);
 
        TX1REG = signal[count];
 
    }
 
}
 
void getPCLSInfoCommand() {
    int signal[] = {0xFE, 0x19, 0x01, 0x04, 0x00, 0x00};
    int count = 0;
    for (count = 0; count < 6; count++) {
        while (!TX1STAbits.TRMT); // The TRMT is set when the TSR register is empty.
        TX1REG = signal[count];
    }
    getPCLSResponse();
}
 
void getPCLSResponse() {
   
    int var = 0;
    while (var < 12){
        while (!PIR3bits.RCIF);
        buffer_pcls_response[var] = RC1REG;
        var++;
}

void surfaceExplorationCommand(int code) {
    int signal[] = {0xFE, 0x19, 0x01, 0x0A, 0x04, 0x00, 0x0, 0x0, code, 0x00, 0x00};
    int count = 0;
    for (count = 0; count < 11; count++) {
        while (!TX1STAbits.TRMT); // The TRMT is set when the TSR register is empty.
        TX1REG = signal[count];
    }
}
 
void receiveRepairCodeCommand() {
    int signal[] = {0xFE, 0x19, 0x03, 0x09, 0x00, 0x00};
    int count = 0;
    for (count = 0; count < 6; count++) {
        while (!TX1STAbits.TRMT); // The TRMT is set when the TSR register is empty.
        TX1REG = signal[count];
    }
}

 
void hallEffectSensorImplementation(){

  // ADC setup
  ANSELBbits.ANSB1 = 1;
  //TRISAbits.TRISA4 = 1;
  TRISBbits.TRISB1 = 1;
  ANSELAbits.ANSA2 = 1;
  TRISAbits.TRISA2 = 0;
  ADCON0bits.ADCS = 0;
  ADCLKbits.ADCCS = 0b111111;
  ADREFbits.ADNREF = 0;
  ADREFbits.ADPREF = 0b00;
  ADPCHbits.ADPCH = 0b001001;
  ADCON0bits.ADON = 1;
  PIE1bits.ADIE = 1;
  //INTCONbits.PEIE = 1;
  //INTCONbits.GIE = 1;
  ADACQbits.ADACQ = 0b00000001;
  ADCON0bits.ADFRM0 = 1;
  ADCON0bits.ADGO = 1;
  if(PIR1bits.ADIF == 1){
         x = (ADRESH << 8) + ADRESL;
         //TRISBbits.TRISB4 = 1;
         if (x > 0x0143){ // 0x0143 is the test value found.
            LATAbits.LATA2 = 1;
         }else{
             LATAbits.LATA2 = 0;
         }
         PIR1bits.ADIF = 0;
         ADCON0bits.ADGO = 1;
     }
}

 
void FFT() {
 
    ANSELCbits.ANSC7 = 1;
 
    TRISCbits.TRISC7 = 1;
 
    ADPCHbits.ADPCH = 0b010111;
 
    ADCON0bits.ADON = 1;
 
    doneWithHall = 0;
 
}
 

 
int hallEffectCounter = 0;
 
void main(void) {
    //Initial UART Setup 
    RC5PPS = 0x10; //setting rc5 as the transmission
 
    RXPPS = 0X16; // setting 0x16 as the receiving
 
 	// Baud rate set up below
    BAUD1CONbits.BRG16 = 1; 
 
    TX1STAbits.BRGH = 1;
 
    SP1BRGL = 0b1000100; // value of 0x44 or 68 is found for n. 
 
    SP1BRGH = 0;
 
 	// Setting up the UART communication.
    ANSELCbits.ANSC6 = 0;
 
    TX1STAbits.SYNC = 0;
 
    RC1STAbits.SPEN = 1;
//
//        INTCONbits.PEIE = 1;
//    
//        INTCONbits.GIE = 1;
//    
	
	
    PIE3bits.RCIE = 1;
 
    RC1STAbits.CREN = 1; // enabling the receiver circuitry for the EUSART
 
    TX1STAbits.TXEN = 1;
 
 
    TRISAbits.TRISA5 = 1; // Used for the LED.
 
 
    //ADC Setup
 
    ADCON0bits.ADCS = 0;
 
    ADCLKbits.ADCCS = 0b111111;
 
    ADREFbits.ADNREF = 0;
 
    ADREFbits.ADPREF = 0b00;
 
    PIE1bits.ADIE = 1;
 
    ADACQbits.ADACQ = 0b00000001;
 
    ADCON0bits.ADFRM0 = 1;
 
    ADCON0bits.ADGO = 1;
 
   
 
    while (1) {
 
 
        getUserDataSenderUser(); // telling PCLS to send the user data
        __delay_ms(100);   // delay of 100 ms

        volatile uint16_t rightXFinal = (buffer_value[7] << 8) | buffer_value[6];  // the right joystick x value
        volatile uint16_t rightYFinal = (buffer_value[9] << 8) | buffer_value[8];  // right joystick y value
        volatile uint16_t leftYFinal = (buffer_value[11] << 8) | buffer_value[10];  //left joystick y value
        volatile uint16_t switchAFinal = (buffer_value[15] << 8) | buffer_value[14]; //3e8 is on and 7d0 is off
        volatile uint16_t switchBFinal = (buffer_value[17] << 8) | buffer_value[16]; // switch b value 3E8 ON 7D0 OFF
        volatile uint16_t switchCFinal = (buffer_value[19] << 8) | buffer_value[18]; // switch b value 5DC IS NEW VAL
        volatile uint16_t switchDFinal = (buffer_value[21] << 8) | buffer_value[20]; // switch b value
        //hallEffectSensorImplementation()
//        if ((rightYFinal == 0x07D0) && (leftYFinal == 0x07C6)){
//            setMotorSettings(0x1, 0x64, 0x1, 0x64);
//            
//        } B 

        //both on
        if (((leftYFinal >= 0x07C0 ) && ((leftYFinal <= 0x07CF ))  )  && ((rightYFinal >= 0x072C ) && ((rightYFinal <= 0x07D0 )))){
            setMotorSettings(0x2, 0x64, 0x1, 0x64);
        }
        //right on left off
        else if (((leftYFinal >= 0x055) && ((leftYFinal <= 0x0638 ))  )  && ((rightYFinal >= 0x072C ) && ((rightYFinal <= 0x07D0 )))){
            setMotorSettings(0x0, 0x64, 0x1, 0x64);
        }
        //left on right off
        else if (((leftYFinal >=  0x07C0 ) && ((leftYFinal <= 0x07CF ))  )  && ((rightYFinal >= 0x05DC ) && ((rightYFinal <= 0x065D )))){
            setMotorSettings(0x2, 0x64, 0x0, 0x64);
        }
        //both reverse
        else if (((leftYFinal >= 0x03E8 ) && ((leftYFinal <= 0x0466 ))  )  && ((rightYFinal >= 0x03E8 ) && ((rightYFinal <= 0x0481 )))){
            setMotorSettings(0x1, 0x64, 0x2, 0x64);
        }
        else{
            setMotorSettings(0x0, 0x64, 0x0, 0x64);
        }


        if (switchAFinal == 0x03E8){
            hallEffectSensorImplementation();
        }
        if (switchBFinal == 0x07D0){
            hallEffectCounter++; // checking which pyramid is magnetic
        }
        if (switchDFinal == 0x03E8){
            surfaceExplorationCommand(hallEffectCounter); // sending command to the home base with the pyramid value. 
        }
        if (switchCFinal == 0x07D0){
            shootLaserSettings();		// used to shoot the laser.
        }
        
        /**
        getPCLSInfoCommand(); // used to check the health of the pcls and get the repair codes. 
        
        if (buffer_pcls_response[11] == 0x0){
        	receiveRepairCodeCommand(); // used to receive the repair codes. 
        }
 		*/
 		
 		
    }
 
}