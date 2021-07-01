/**@brief:  Este programa configura el módulo IoT LARA-R2.
 * @device: DSPIC30F4013
 */

#include "xc.h"
#include <stdio.h>
#include <libpic30.h>
#include <string.h>
#include <p30F4013.h>

/********************************************************************************/
/* 						BITS DE CONFIGURACIÓN									*/	
/********************************************************************************/
/* SE DESACTIVA EL CLOCK SWITCHING Y EL FAIL-SAFE CLOCK MONITOR (FSCM) Y SE 	*/
/* ACTIVA EL OSCILADOR INTERNO (FAST RC) PARA TRABAJAR							*/
/* FSCM: PERMITE AL DISPOSITIVO CONTINUAR OPERANDO AUN CUANDO OCURRA UNA FALLA 	*/
/* EN EL OSCILADOR. CUANDO OCURRE UNA FALLA EN EL OSCILADOR SE GENERA UNA 		*/
/* TRAMPA Y SE CAMBIA EL RELOJ AL OSCILADOR FRC  								*/
/********************************************************************************/
//_FOSC(CSW_FSCM_OFF & FRC); 
#pragma config FOSFPR = FRC             // Oscillator (Internal Fast RC (No change to Primary Osc Mode bits))
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)
/********************************************************************************/
/* SE DESACTIVA EL WATCHDOG														*/
//_FWDT(WDT_OFF); 
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)
/********************************************************************************/
/* SE ACTIVA EL POWER ON RESET (POR), BROWN OUT RESET (BOR), 					*/	
/* POWER UP TIMER (PWRT) Y EL MASTER CLEAR (MCLR)								*/
/* POR: AL MOMENTO DE ALIMENTAR EL DSPIC OCURRE UN RESET CUANDO EL VOLTAJE DE 	*/	
/* ALIMENTACIÓN ALCANZA UN VOLTAJE DE UMBRAL (VPOR), EL CUAL ES 1.85V			*/
/* BOR: ESTE MODULO GENERA UN RESET CUANDO EL VOLTAJE DE ALIMENTACIÓN DECAE		*/
/* POR DEBAJO DE UN CIERTO UMBRAL ESTABLECIDO (2.7V) 							*/
/* PWRT: MANTIENE AL DSPIC EN RESET POR UN CIERTO TIEMPO ESTABLECIDO, ESTO 		*/
/* AYUDA A ASEGURAR QUE EL VOLTAJE DE ALIMENTACIÓN SE HA ESTABILIZADO (16ms) 	*/
/********************************************************************************/
//_FBORPOR( PBOR_ON & BORV27 & PWRT_16 & MCLR_EN ); 
// FBORPOR
#pragma config FPWRT  = PWRT_16          // POR Timer Value (16ms)
#pragma config BODENV = BORV20           // Brown Out Voltage (2.7V)
#pragma config BOREN  = PBOR_ON          // PBOR Enable (Enabled)
#pragma config MCLRE  = MCLR_EN          // Master Clear Enable (Enabled)
/********************************************************************************/
/*SE DESACTIVA EL CÓDIGO DE PROTECCIÓN											*/
/********************************************************************************/
//_FGS(CODE_PROT_OFF)
// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

/********************************************************************************/
/* SECCION DE DECLARACION DE CONSTANTES CON DEFINE								*/
/********************************************************************************/
#define EVER 1
#define ACK 0X06        // Frame received with success, commands understood and commands executed with success
#define NACK 0X15        // Frame received with success, however commands not executed with success
#define CSFAIL 0X51     // Frame received with success, however the checksum of the frame did not match

void __attribute__((__interrupt__)) _U2RXInterrupt( void );

void iniPerifericos( void ); 

void iniUART1( void );
void iniUART2( void );

void iniInterrupciones( void );
void habilitaUARTS( void );

extern void RETARDO_300ms( void );
extern void RETARDO_1s( void );
extern void RETARDO_50us( void );
extern void RETARDO_15ms( void );

void obtenParametros( void );
void enviaFrame( unsigned char registro, unsigned char bytes, unsigned char checksum );
void enviaDato( unsigned char dato );

unsigned char state;
unsigned char checksumResponse;

int main(void) {
      
    iniPerifericos();    
    iniUART1();
    iniUART2(); 
    iniInterrupciones();
    habilitaUARTS();  
    
    state = 1;
    checksumResponse = 0;
    
    obtenParametros();

    while( EVER )
    {  
        Nop();
    }
    return 0;
}

/* @brief: Esta funcion inicializa los perifericos a utilizar */
void iniPerifericos()
{    
    PORTB = 0;
    Nop();
    LATB = 0;
    Nop();
    TRISB = 0;
    Nop();
    ADPCFG = 0xFFFF;
    
    PORTD = 0;
    Nop();
    LATD = 0;
    Nop();
    TRISD = 0;
    Nop();
        
    PORTC = 0;
    Nop();
    LATC = 0;
    Nop();
    TRISC = 0;
    Nop();
    
    PORTF = 0;
    Nop();
    LATF = 0;
    Nop();
    TRISF = 0;
    Nop();
    
    /* Configuracion U1RX que se comunicara con la PC */
    TRISCbits.TRISC14 = 1;    //U1ARX
    /* Configuracion U2RX que se comunicara con el sensor de monitoreo */
    TRISFbits.TRISF4 = 1;    //U2ARX
}

/* @brief: Configuracion del UART 1. El UART1 configura el FT232 para el envio
 * de datos a la PC
 * Velocidad: 9600 baudios
 * Trama: 8 bits x dato, sin paridad, 1 bit de inicio, 1 bit de paro
 */
void iniUART1()
{
    U1MODE = 0X0420;  // 2 para Auto Baud Enable bit
    U1STA = 0X8000;   // 8 para UTXISEL: Transmission Interrupt Mode Selection bit
    U1BRG = 11;       // BaudRate 9600
}

/* @brief: Configuracion del UART 2. El UART2 se configura para que se comunique
 * con el sensor de monitoreo
 * Velocidad: 9600 baudios
 * Trama: 8 bits x dato, sin paridad, 1 bit de inicio, 1 bit de paro
 */
void iniUART2()
{
    U2MODE = 0X0000;
    U2STA = 0X8000;
    U2BRG = 11;
}

/* @brief: Inicializacion de las interrupciones que se utilizaran */
void iniInterrupciones()
{    
    IFS1bits.U2RXIF = 0;    //UART2 Receiver Interrupt Flag Status bit
    IEC1bits.U2RXIE = 1;    //UART2 Receiver Interrupt Enable bit (Interrupt request enabled)  
}

/* @brief: Habilitacion de UART1 y UART2 */
void habilitaUARTS()
{
    U1MODEbits.UARTEN = 1;  //UART1 Enable bit: 1 para habilitar
    U1STAbits.UTXEN = 1;    //Transmit Enable bit: 1 para habilitar

    U2MODEbits.UARTEN = 1;  //UART2 Enable bit: 1 para habilitar
    U2STAbits.UTXEN = 1;   //Transmit Enable bit: 1 para habilitar
}

/* @brief: Interrupcion que se genera cada vez que se recibe un dato en UART2 */
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt( void )
{    
    char resp;
    resp = U2RXREG;
    U1TXREG = resp; 
    
//    if( state == 1 ){
//        if( resp == ACK ){
//            state++;
//            checksumResponse += resp;
//            U1TXREG = resp;
//        }
//        else if( resp == NACK ){
//            U1TXREG = 0X02;
//            state--;
//        }
//        else if( resp == CSFAIL ){
//            U1TXREG = 0X44;
//            state--;
//        }
//        else state--;
//    }
//    
//    else if( state == 2 ) U1TXREG = resp; 
    
    IFS1bits.U2RXIF = 0;
}

/* @brief: Obtencion de los parametros proporcionados por el sensor */
void obtenParametros()
{
    // Voltaje
    enviaFrame( 0X06, 0X02, 0X44 );
    RETARDO_15ms();
    
    // Frecuencia
    enviaFrame( 0X08, 0X02, 0X46 );
    RETARDO_15ms();
    
    // Corriente
    enviaFrame( 0x0E, 0X04, 0X4E );
    RETARDO_15ms();
    
    // Factor de Potencia
    enviaFrame( 0x0C, 0X02, 0X4A );
    RETARDO_15ms();
    
    // Potencia Activa
    enviaFrame( 0x12, 0X04, 0X52 );
    RETARDO_15ms();
    
    // Potencia Aparente
    enviaFrame( 0x1A, 0X04, 0X5A );
    RETARDO_15ms();
    
}

/* @brief: Funcion que manda el frame al sensor para recibir los datos solicitados
 * @param: Registro que se desea leer, bytes n�mero de bytes que se van a leer,
 * checksum del frame a enviar
 */
void enviaFrame( unsigned char registro, unsigned char bytes, unsigned char checksum )
{
    enviaDato( 0XA5 );          // Header byte
    RETARDO_15ms();
    
    enviaDato( 0X08 );          // Numero de bytes en frame
    RETARDO_15ms();
    
    enviaDato( 0X41 );          // Command set address pointer
    RETARDO_15ms();
    
    enviaDato( 0X00 );          // Addr High
    RETARDO_15ms();
    
    enviaDato( registro );      // Addr Low
    RETARDO_15ms();
    
    enviaDato( 0X4E );          // Command read register
    RETARDO_15ms();
    
    enviaDato( bytes );         // Numero de bytes a leer
    RETARDO_15ms();
    
    enviaDato( checksum );     // Checksum
    RETARDO_15ms();
}
    
/* @brief: Envio de cada caracter del frame al sensor por medio de UART2 
 * @param: Caracter a enviar por medio de UART2
 */
void enviaDato( unsigned char dato )
{
    IFS1bits.U2TXIF = 0;
    U2TXREG = dato;
    while( !IFS1bits.U2TXIF );
    IFS1bits.U2TXIF = 0;
}
