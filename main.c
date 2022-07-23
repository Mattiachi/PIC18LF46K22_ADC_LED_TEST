/*
 * Object   :   RN2483 Blink & ADC test
 * Author   :   Chiappalone Mattia
 * Date     :   August 2022
 * Device   :   PIC18LF46K22
 * Compiler :   XC8 2.40
 * MPLAB    :   MPLABX 3.40
 * 
 * Description: 
 * This program blinks the green led once a second, then uses the ADC to read
 * the voltage on the analog pins and outputs the result turning the orange led
 * on for a number of milliseconds equals to the ADC reading (10 bits ADC)
 * 
 * 
 * RN2483 pin | PIC18LF46K22 pin |    Peripheral
 *    GPIO10  |     RC5          |    Orange Led
 *    GPIO11  |     RD5          |    Green Led
 *    GPIO12  |     RD7  (AN27)  |    Temperature Sensor ()
 *    GPIO13  |     RD6 (AN26)   |    Light sensor
 */

#include "mcc_generated_files/mcc.h"


#define LED_GREEN PORTDbits.RD5
#define LED_ORANGE PORTCbits.RC5
#define TEMP 27    //is the ADC channel of the pin 
#define LIGHT 26


void IO_pins_init(void);
void ADC_init(void);
void ADC_SelChannel(uint8_t c);
uint16_t ADC_Read(uint8_t channel);

uint16_t temp, light;

void main (void){
    IO_pins_init();
    ADC_init();
    
    while(1){        
        LED_GREEN = 1;
        __delay_ms(1000);
        LED_GREEN = 0;
        __delay_ms(1000);
        
        temp = ADC_Read(TEMP);
        light = ADC_Read(LIGHT);
        
        //This is to test the ADC: the Orange LED stays high for x ms where x is the ADC value
        LED_ORANGE = 1;
        for (uint16_t time = 0; time < light; time++){
            __delay_ms(1);
        }
        LED_ORANGE = 0;
    }
}

void IO_pins_init(void){
    //TRISx = 1 input ; TRISx = 0 output
    TRISCbits.TRISC5 = 0;       // Orange led
    TRISDbits.TRISD5 = 0;       // Green led
    TRISDbits.TRISD6 = 1;       // Light sensor
    TRISDbits.TRISD7 = 1;       // Temperature sensor
    //ANSELx = 0 digital, ANSELx = 1 analog
    ANSELCbits.ANSC5 = 0;       // Orange led
    ANSELDbits.ANSD5 = 0;       // Green led
    ANSELDbits.ANSD6 = 1;       // Light sensor
    ANSELDbits.ANSD7 = 1;       // Temperature sensor
}   

void ADC_init(void){
    ADCON0bits.GO = 0;   // starts conversion if set
    ADCON2bits.ADFM = 1; // right justified (MSB conversione sono il bit 1 di ADRESH e il LSB il bit 0 di ADRESL )
    ADCON2bits.ADCS = 3; // clock is derived from internal oscillator 
    return;
}

void ADC_SelChannel(uint8_t c){
   ADCON0bits.CHS = c;    //Set the channel (analog pin))
   return;
}

uint16_t ADC_Read(uint8_t channel){
    uint16_t result = 0;
    ADCON0bits.ADON = 1;    //Turns on ADC
    ADC_SelChannel(channel);
    ADCON0bits.GO = 1;      //Starts conversion
    while(ADCON0bits.GO);
    result = (ADRESH<<8) | ADRESL;
    ADCON0bits.ADON = 0;    //turns off ADC
    return result;
}
