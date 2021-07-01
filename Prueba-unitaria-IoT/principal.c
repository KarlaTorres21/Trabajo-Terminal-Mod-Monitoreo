/**@brief:  Este programa configura el módulo IoT LARA-R2.
 * @device: DSPIC30F4013
 */

#include "xc.h"
#include <stdio.h>
#include <libpic30.h>
#include <string.h>

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
//#define MUESTRAS 64

/********************************************************************************/
/* DECLARACIONES GLOBALES														*/
/********************************************************************************/
/*DECLARACI?N DE LA ISR DEL TIMER 1 USANDO __attribute__						*/
/********************************************************************************/
void __attribute__((__interrupt__)) _U2RXInterrupt( void );


//****Comandos AT****
char CMD_AT[] = "AT\r";
char CMD_ATE0[] = "ATE0\r";
char CMD_AT_CMGF[] = "AT+CMGF=1\r";
char CMD_AT_CMGS[] = "AT+CMGS=\"+525511950664\"\r";
char CMD_MENSAJE[] = "Temperatura: \x1A\r";
/* Comprobamos el estado del perfil de conecion GPRS */
char CMD_UPSND[] = "AT+UPSND=0,8\r";
/* We removed that invalid context */
char CMD_CGDCONT[] = "AT+CGDCONT=8\r";
/* El perfil #0 es mapeado en CID = 1 */
char CMD_UPSD[] = "AT+UPSD=0,100,1\r";
/* Activacion del perfil PSD #0 */
char CMD_UPSDA[] = "AT+UPSDA=0,3\r";
/* Creacion del socket tipo TCP */
char CMD_USOCR[] = "AT+USOCR=6\r";
/* Conexion al servidor */
char CMD_USOCO[] = "AT+USOCO=0,\"8.tcp.ngrok.io\",14423\r";
/* Manda mensaje */
char CMD_USOWR[] = "AT+USOWR=0,13\r";
/* Se cierra la conexion */
char CMD_USOCL[] = "AT+USOCL=0\r";
/* Se cierra conexion PDP */
char CMD_UPSDA_CERRAR[] = "AT+UPSDA=0,4\r";

char CMD_USODL[] = "AT+USODL=0\r";              // Se activa the direct link mode for socket #0
char CMD_STOP_DIRECT_LINK[] = "+++";            // Exit from direct link mode

char responseReconexion[40];
unsigned char j;

char count;

void iniPerifericos( void ); 
void iniUART1( void );
void iniUART2( void );
void iniInterrupciones( void );
void habilitaUARTS( void );
void iniIoT( void );

void enviarComandoGSM( char comando[] );

void printUART1(char* cadena);
void printUART2(char* cadena);

extern void imprimeUART1( char* cadena );
extern void comandoAT( char* cadena );

extern void RETARDO_300ms( void );
extern void RETARDO_1s( void );
extern void RETARDO_50us( void );

unsigned char muestras[19] = {0X10, 0XEA, 0X04, 0XA7, 0XEA, 0XD3, 0X45, 0XE0, 0X11, 0X00, 
                              0X00, 0X44, 0X0C, 0X00, 0X00, 0X7C, 0X16, 0X00, 0X00};
unsigned char reconexionTCP;

int main(void) {
    
    iniPerifericos();
    iniUART1();
    iniUART2();
    iniInterrupciones();
    habilitaUARTS();    
    iniIoT();
    
    reconexionTCP = 0;
    imprimeUART1("\r\n inicializando... \r\n");
    
    RETARDO_1s();
    RETARDO_1s();
    RETARDO_1s();
    RETARDO_1s();
    RETARDO_1s();
    
    enviarComandoGSM( CMD_AT );  
    enviarComandoGSM( CMD_ATE0 );
    
    RETARDO_1s();
    RETARDO_1s();
    RETARDO_1s();
    RETARDO_1s();
    RETARDO_1s();
    
    enviarComandoGSM( CMD_UPSND );
    enviarComandoGSM( CMD_CGDCONT );
    enviarComandoGSM( CMD_UPSD );
    enviarComandoGSM( CMD_UPSDA );
    enviarComandoGSM( CMD_UPSND );
    enviarComandoGSM( CMD_USOCR );
    
    /* Re-conexion */
    reconexionTCP = 1;
    while( reconexionTCP ){
        j = 0;
        enviarComandoGSM( CMD_USOCO ); 
        if( responseReconexion[2] == 69 ){
            RETARDO_1s();
            RETARDO_1s();
            RETARDO_1s();
            RETARDO_1s();
            RETARDO_1s();
            enviarComandoGSM( CMD_USOCR );
        }
        else
            reconexionTCP = 0;
    }
       
    enviarComandoGSM( CMD_USODL );
    int j;
    for( j = 0; j < 19; j++ ){
        U2TXREG = muestras[j];
        while( !IFS1bits.U2TXIF );
        IFS1bits.U2TXIF = 0;
    }
    
    RETARDO_1s();
    
    enviarComandoGSM(CMD_STOP_DIRECT_LINK);
    enviarComandoGSM( CMD_USOCL );
    enviarComandoGSM( CMD_UPSDA_CERRAR );
    enviarComandoGSM( CMD_UPSND );
    
    while( EVER ){
        Nop();
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
    
    /* Inicialización del mikroBUS2 para el módulo 4G */
    /*                | Pin MikroBUS IoT | Pin MikroBUS2 TESE |*/
    TRISBbits.TRISB5 = 1;  // STA                 AN5               
    TRISDbits.TRISD1 = 0;  // PWK                 RST
    TRISBbits.TRISB8 = 0;  // RTS                 CS
    TRISDbits.TRISD3 = 1;  // RI                  PWM
    TRISDbits.TRISD9 = 1;  // CTS                 INT
}

/******************************************************************************
* CONFIGURACION DEL UART 1. EL UART 1 CONFIGURA EL FT232 PARA ENVIO DE DATOS A LA PC
* VELOCIDAD: 115200 BAUDIOS
* TRAMA: 8 BITS X DATO, SIN PARIDAD, 1 BIT DE INICIO, 1 BIT DE PARO
******************************************************************************/
void iniUART1()
{
    U1MODE = 0X0420; // 2 para Auto Baud Enable bit
    U1STA = 0X8000;  // 8 para UTXISEL: Transmission Interrupt Mode Selection bit
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
    U2MODE = 0X0020;
    U2STA = 0X8000;
    U2BRG = 11;
}

/****************************************************************************/
/* DESCRICION:	ESTA RUTINA INICIALIZA LAS INTERRPCIONES    				*/
/* PARAMETROS: NINGUNO                                                      */
/* RETORNO: NINGUNO															*/
/****************************************************************************/
void iniInterrupciones()
{    
    IFS1bits.U2RXIF = 0;    //UART2 Receiver Interrupt Flag Status bit
    IEC1bits.U2RXIE = 1;    //UART2 Receiver Interrupt Enable bit (Interrupt request enabled)
}

void habilitaUARTS()
{
    U1MODEbits.UARTEN = 1;  //UART1 Enable bit: 1 para habilitar
    U1STAbits.UTXEN = 1;    //Transmit Enable bit: 1 para habilitar
    
    U2MODEbits.UARTEN = 1;  //UART2 Enable bit: 1 para habilitar
    U2STAbits.UTXEN = 1;   //Transmit Enable bit: 1 para habilitar
}

void iniIoT()
{
    LATBbits.LATB0 = 1;    //LED
    LATDbits.LATD1 = 1;    //PWK
    RETARDO_1s();
    
    LATDbits.LATD1 = 0;    //PWK
    RETARDO_50us(); 
    
    LATDbits.LATD1 = 1;    //PWK
    Nop();
    
    LATBbits.LATB0 = 0;    //LED
}

void enviarComandoGSM(char comando[]){
    IFS1bits.U2TXIF = 0;
    
    printUART1(comando);
    printUART1("\n");
    
    printUART2(comando);
    
    count = 2;
    j = 0;
    //Espera respuesta
    while(count > 0){
        U1TXREG = 'x';
        RETARDO_1s();
    }   //count disminuye con la interrupci?n U2RXInterrupt
    
}

/********************************************************************************/
/* DESCRICION:	ISR (INTERRUPT SERVICE ROUTINE) DEL TIMER 1						*/
/* LA RUTINA TIENE QUE SER GLOBAL PARA SER UNA ISR								*/	
/* SE USA PUSH.S PARA GUARDAR LOS REGISTROS W0, W1, W2, W3, C, Z, N Y DC EN LOS */
/* REGISTROS SOMBRA																*/
/********************************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt( void )
{
    LATBbits.LATB0 = 1;     //LED
    char resp;
    
    resp = U2RXREG;
    U1TXREG = resp;
    
    if(resp == 13){     //<CR>
        resp = '.';
    }
    else if(resp == 10){     //<LF>
        resp = '_';
        count--;
    }
    else if(resp == 62){    //'>'
        count--;
    }
    else if(resp == 32){   //' '
        resp = '-';
    }
    else if( resp == 64 )
        count--;
    
    if( reconexionTCP == 1 ){
        responseReconexion[j] = resp;
        j++;
    }
    
    LATBbits.LATB0 = 0;     //LED
    IFS1bits.U2RXIF = 0;
}

void printUART1(char* cadena) {
    int i;
    for(i = 0; cadena[i] != '\0'; i++) {
        U1TXREG = cadena[i];
        
        //Mientras no se genere la interrupci�n, va a esperar
        while(!IFS0bits.U1TXIF);
        IFS0bits.U1TXIF = 0;
    }
}

void printUART2(char* cadena) {
    int i;
    for(i = 0; cadena[i] != '\0'; i++) {
        U2TXREG = cadena[i];
        
        //Mientras no se genere la interrupci�n, va a esperar
        while(!IFS1bits.U2TXIF);
        IFS1bits.U2TXIF = 0;
    }
}
