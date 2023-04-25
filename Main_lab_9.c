/* 
 * File:   Prelab_9.c
 * Author: Luis Antunez
 *
 * Created on 23 de abril de 2023, 05:53 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_CLKOUT// Oscillator Selection bits (RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// Librerias 
#include <xc.h>
#include <pic16f887.h>
#include <stdint.h>

// Constantes 
#define _XTAL_FREQ 8000000 // Valor del ciclo de reloj

// Variables

uint8_t address, data, pot;
uint8_t sleep = 0; 

// Prototipos
void setup(void);
uint8_t read(uint8_t address);
void write (uint8_t address, uint8_t data);

// Interrupcion
void __interrupt() isr(void)
{
    if (INTCONbits.RBIF)
    {
        if (PORTBbits.RB0 == 0) // Si el pin RB0 ha cambiado a bajo, entrar en modo de suspensión
        {
            /*SLEEP();
            INTCONbits.RBIF = 0; // Borrar la bandera de interrupción por cambio de estado del puerto B*/
            sleep =1;
        }
        else if (PORTBbits.RB1 == 0 )
        {
            //INTCONbits.RBIF = 0; // Borrar la bandera de interrupción por cambio de estado del puerto B
            sleep =0;
        }
        else if (PORTBbits.RB2 == 0)
        {
            write(address,pot); // Iniciar proceso de escritura
            sleep = 0;
        }    
        else if (PORTBbits.RB3 == 0) // Lectura
        {
            PORTD=0;            // Reseteo del puerto D en caso de que tenga algun dato
            data = read(address); // El valor almacenado en la direccion se pasa a la variable data
            PORTD = data;       // Representar el valor de data en el puerto D
            sleep = 0;
        
            
        }
        INTCONbits.RBIF = 0;
    }
}

// Codigo principal 
void main (void){
   
    setup();
    while (1) 
    {
        ADCON0bits.CHS = 0b00000001; // Seleccion del canal AN1
        ADCON0bits.GO =1;  // Habilita las conversiones de analogico a digital
        __delay_ms(10);
        while (ADCON0bits.GO_DONE); // Verificacion del canal AN1
        int adc = ADRESH;           // Mueve el valor almacenado en ADRESH a adc
        pot = ADRESH;
        PORTC = (char) adc;         // Mueve el valor de adc al puerto C
        __delay_ms(10);
        
        if (sleep ==1)
        {
            SLEEP();
        
        }
        
    }
        
        
        
}
    
void setup(void)
{
    ANSEL =0b00000010;
    ANSELH=0; 
    //Configuracion de entradas y salidas
    TRISB = 0b00001111;
    TRISC = 0;
    TRISD = 0;
    // Limpiamos los puertos
    PORTA=0;
    PORTB=0;
    PORTC=0;
    PORTD=0;
    
    OSCCONbits.IRCF =0b0110; // Oscilador de 4MHz
    OSCCONbits.SCS = 1;      // Oscilador interno
    
    OPTION_REGbits.nRBPU = 0;
    WPUBbits.WPUB0 = 1;
    WPUBbits.WPUB1 = 1;
    WPUBbits.WPUB2 = 1;
    WPUBbits.WPUB3 = 1;
    WPUBbits.WPUB4 = 1;
    IOCB= 0b01111111;
    
    // Interrupcion del purto B
    INTCONbits.RBIF = 0; // Borrar la bandera de interrupción por cambio de estado del puerto B
    INTCONbits.RBIE = 1; // Habilitar la interrupción por cambio de estado del puerto B
    INTCONbits.GIE = 1; // Habilitar la interrupción global
    
    // Configuracion del ADC
    ADCON0bits.ADCS = 0b01; // divisor de reloj de 32
    __delay_ms(10);
    ADCON1bits.ADFM = 0;    // Justificado a la izquierda 
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;    // Referencia de voltaje 0
    ADCON0bits.ADON = 1;   // habilitar el adc
    ADIF =0;
    return;
}

uint8_t read (uint8_t address) // Lectura de la EEPROM
{
    while (WR || RD); // Verificacion si hay procesos de lectura o escritura
    EEADR = address;  // Direccion de la memoria a leer
    EECON1bits.EEPGD = 0; // Lectura de la EEPROM
    EECON1bits.RD = 1;    // Obtener datos de la EEPROM
    return EEDAT;       // Regresar el dato
}

void write (uint8_t address, uint8_t data)
{
    uint8_t interStatus;  // Estado de la interrupcion
    while (WR); // Verificacion si hay un processo de escritura en progreso
    EEADR = address; // Direccion de memoria a escribir
    EEDAT = data;   // dato a escribir
    EECON1bits.EEPGD = 0; // Acceso a memoria de datos EEPROM
    EECON1bits.WREN = 1; // Habilitar la escritura en la EEPROM
    interStatus = GIE;
    INTCONbits.GIE =0;  // Deshabilitar interrupciones
    EECON2 = 0x55;      
    EECON2 = 0xAA;      // Secuencia de escritura
    EECON1bits.WR = 1;  // Iniciar con la escritura
    EECON1bits.WREN=0;  // Deshabilitar la escritura
    INTCONbits.GIE = interStatus;  // Habilitar interrupciones
}

