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
#define NACK 0X15       // Frame received with success, however commands not executed with success
#define CSFAIL 0X51     // Frame received with success, however the checksum of the frame did not match

/********************************************************************************/
/* DECLARACIONES GLOBALES														*/
/********************************************************************************/
/*DECLARACI?N DE LA ISR QUE SE VAN A UTILIZAR EN EL PROGRAMA    				*/
/********************************************************************************/
void __attribute__((__interrupt__)) _U1RXInterrupt( void );
void __attribute__((__interrupt__)) _T1Interrupt( void );

void iniPerifericos( void ); 

void iniUART1( void );
void iniUART2( void );
void iniTimer1( void );

void iniInterrupciones( void );
void habilitaUARTS( void );
void habilitaTimer( void );

extern void RETARDO_300ms( void );
extern void RETARDO_1s( void );
extern void RETARDO_50us( void );
extern void RETARDO_15ms( void );

void obtenParametros( void );
void enviaFrame( unsigned char registro, unsigned char bytes, unsigned char checksum );
void enviaDato( unsigned char dato );
void mandaServer( void );

unsigned char segundos;
unsigned char mandaFrame;
unsigned char muestras[19] = {};
unsigned char state;
unsigned char numBytes;
int checksum;
int i;

int main(void) {
      
    iniPerifericos();    
    iniUART1();
    iniUART2(); 
    iniTimer1();
    iniInterrupciones();
    
    habilitaUARTS(); 
        
    segundos = 0;
    mandaFrame = 0;
    state = 1;
   
    habilitaTimer();
    
    while( EVER )
    {   
        if( mandaFrame ){
            state = 1;
            i = 0;
            checksum = 0;
            obtenParametros();
            mandaFrame = 0;
        }
        asm( "pwrsav #1" );
    }
    return 0;
}

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
    /* Configuracion U2RX que se comunicara con el modulo IoT */
    TRISFbits.TRISF4 = 1;    //U2ARX
}

/******************************************************************************
* CONFIGURACION DEL UART 1. EL UART 1 CONFIGURA EL FT232 PARA ENVIO DE DATOS A LA PC
* VELOCIDAD: 115200 BAUDIOS
* TRAMA: 8 BITS X DATO, SIN PARIDAD, 1 BIT DE INICIO, 1 BIT DE PARO
******************************************************************************/
void iniUART1()
{
    U1MODE = 0X0400;  // 4 para ALTIO UART communicates using UxATX and UxARX  
    U1STA = 0X8000;   // 8 para UTXISEL: Transmission Interrupt Mode Selection bit
    U1BRG = 11;       // BaudRate 9600
}

/******************************************************************************
* CONFIGURACION DEL UART 2. EL UART 2 CONFIGURA EL MODEM PARA ENVIO DE
* COMANDOS AT Y RECEPCION DE RESPUESTAS DEL MODEM
* VELOCIDAD: 115200 BAUDIOS
* TRAMA: 8 BITS X DATO, SIN PARIDAD, 1 BIT DE PARO
******************************************************************************/
void iniUART2()
{
    U2MODE = 0X0020;  // 2 para Auto Baud Enable bit
    U2STA = 0X8000;
    U2BRG = 11;
}

/****************************************************************************/
/* DESCRICION: INICIALIZAR TMR1, PR1 & TICON                 				*/
/* PARAMETROS: NINGUNO                                                      */
/* RETORNO: NINGUNO															*/
/****************************************************************************/
void iniTimer1()
{
    TMR1 = 0;         // Iniciamos el contador en 0
    PR1 = 0X7080;     // PRx = 28800
    T1CON = 0X0020;   // 8 para activar TIMER1 y 2 para usar la pre-escala de 64
}

/****************************************************************************/
/* DESCRICION:	ESTA RUTINA INICIALIZA LAS INTERRPCIONES    				*/
/* PARAMETROS: NINGUNO                                                      */
/* RETORNO: NINGUNO															*/
/****************************************************************************/
void iniInterrupciones()
{    
    IFS0bits.U1RXIF = 0;    //UART1 Receiver Interrupt Flag Status bit
    IEC0bits.U1RXIE = 1;    //UART1 Receiver Interrupt Enable bit (Interrupt request enabled)
    
    IFS0bits.T1IF = 0;      //Bit de la bandera de estado de la interrupcion
    IEC0bits.T1IE = 1;      //Habilitacion del bit de interrupcion
}

void habilitaUARTS()
{
    U1MODEbits.UARTEN = 1;  //UART1 Enable bit: 1 para habilitar
    U1STAbits.UTXEN = 1;    //Transmit Enable bit: 1 para habilitar

    U2MODEbits.UARTEN = 1;  //UART2 Enable bit: 1 para habilitar
    U2STAbits.UTXEN = 1;   //Transmit Enable bit: 1 para habilitar
}

void habilitaTimer()
{
    T1CONbits.TON = 1; 
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt( void )
{
    char response; 
    response = U1RXREG;
    
    if( state == 1 ){        // Recepcion del ACK o NACK por parte del sensor
        if( response == ACK ){
            state++;
            checksum += response;
        }
    }
    
    else if( state == 2 ){   // Recepcion del numero de bytes a leer
        numBytes = response;
        checksum += numBytes;
        state++;
    }
    
    else if( state < numBytes ){    // Recepcion de los datos
        if( ( state > 3 && state < 7 ) || ( state > 10 && state < 13 ) || ( state > 22 && state < 27 ) );
        else{
            muestras[i] = response;
            i++;
        }
        state++;
        checksum += response;
    }
    
    else if( state == numBytes ){    // Verificacion del checksum
        unsigned char mod = checksum % 256;
        unsigned char aux = response;
        if( mod == aux )
            mandaServer();
    }
       
    IFS0bits.U1RXIF = 0;
}

/********************************************************************************/
/* DESCRICION:	ISR (INTERRUPT SERVICE ROUTINE) DEL TIMER 1						*/
/* LA RUTINA TIENE QUE SER GLOBAL PARA SER UNA ISR								*/	
/* SE USA PUSH.S PARA GUARDAR LOS REGISTROS W0, W1, W2, W3, C, Z, N Y DC EN LOS */
/* REGISTROS SOMBRA																*/
/********************************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt( void )
{           
    if( segundos < 2 )
        segundos++;
    else{
        mandaFrame = 1;
        segundos = 0;
    }
    
    IFS0bits.T1IF = 0;
}

void obtenParametros()
{
    /*      Estructura de los bytes a leer          */
    /* | System Status | System Version | Voltaje RMS  | Line Frequency | Thermisor Voltage | */
    /* | Power Factor  |   Current RMS  | Active Power | Reactive Power |   Apparent Power  | */
    enviaFrame( 0x02, 0X1C, 0X5A );
    RETARDO_15ms();  
}

/* @brief: Funcion que manda el frame al sensor para recibir los datos solicitados
 * @param: Registro que se desea leer, bytes n�mero de bytes que se van a leer,
 * checksum del frame a enviar
 */
void enviaFrame( unsigned char registro, unsigned char bytes, unsigned char checksum )
{
    enviaDato( 0XA5 );          // Header byte        
    enviaDato( 0X08 );          // Numero de bytes en frame        
    enviaDato( 0X41 );          // Command set address pointer        
    enviaDato( 0X00 );          // Addr High        
    enviaDato( registro );      // Addr Low        
    enviaDato( 0X4E );          // Command read register        
    enviaDato( bytes );         // Numero de bytes a leer        
    enviaDato( checksum );     // Checksum
}
    
/* @brief: Envio de cada caracter del frame al sensor por medio de UART2 
 * @param: Caracter a enviar por medio de UART2
 */
void enviaDato( unsigned char dato )
{
    IFS0bits.U1TXIF = 0;
    U1TXREG = dato;
    while( !IFS0bits.U1TXIF );
    IFS0bits.U1TXIF = 0;
}

void mandaServer()
{
    int j;
    IFS1bits.U2TXIF = 0;
        
    for( j = 0; j < 19; j++ ){
        U2TXREG = muestras[j];
        while( !IFS1bits.U2TXIF );
        IFS1bits.U2TXIF = 0;
    }
}
