/**@brief:  Este programa realiza la toma de parametros del sensor MCP39F511A, los cuales son enviados
 * posteriormente al modulo IoT 4G-LTE LARA
 * @device: DSPIC30F4013
 */

#include "xc.h"
#include <stdio.h>
#include <libpic30.h>
#include <string.h>
#include <p30F4013.h>

/********************************************************************************/
/* 						BITS DE CONFIGURACION									*/	
/********************************************************************************/
/* SE DESACTIVA EL CLOCK SWITCHING Y EL FAIL-SAFE CLOCK MONITOR (FSCM) Y SE 	*/
/* ACTIVA EL OSCILADOR INTERNO (FAST RC) PARA TRABAJAR							*/
/* FSCM: PERMITE AL DISPOSITIVO CONTINUAR OPERANDO AUN CUANDO OCURRA UNA FALLA 	*/
/* EN EL OSCILADOR. CUANDO OCURRE UNA FALLA EN EL OSCILADOR SE GENERA UNA 		*/
/* TRAMPA Y SE CAMBIA EL RELOJ AL OSCILADOR FRC  								*/
/********************************************************************************/
#pragma config FOSFPR = FRC             // Oscillator (Internal Fast RC (No change to Primary Osc Mode bits)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)
/********************************************************************************/
/* SE DESACTIVA EL WATCHDOG														*/
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)
/********************************************************************************/
/* SE ACTIVA EL POWER ON RESET (POR), BROWN OUT RESET (BOR), 					*/	
/* POWER UP TIMER (PWRT) Y EL MASTER CLEAR (MCLR)								*/
/* POR: AL MOMENTO DE ALIMENTAR EL DSPIC OCURRE UN RESET CUANDO EL VOLTAJE DE 	*/	
/* ALIMENTACIÓN ALCANZA UN VOLTAJE DE UMBRAL (VPOR), EL CUAL ES 1.85V			*/
/* BOR: ESTE MODULO GENERA UN RESET CUANDO EL VOLTAJE DE ALIMENTACIÓN DECAE	*/
/* POR DEBAJO DE UN CIERTO UMBRAL ESTABLECIDO (2.7V) 							*/
/* PWRT: MANTIENE AL DSPIC EN RESET POR UN CIERTO TIEMPO ESTABLECIDO, ESTO 		*/
/* AYUDA A ASEGURAR QUE EL VOLTAJE DE ALIMENTACIÓN SE HA ESTABILIZADO (16ms) 	*/
/********************************************************************************/
#pragma config FPWRT  = PWRT_16          // POR Timer Value (16ms)
#pragma config BODENV = BORV20           // Brown Out Voltage (2.7V)
#pragma config BOREN  = PBOR_ON          // PBOR Enable (Enabled)
#pragma config MCLRE  = MCLR_EN          // Master Clear Enable (Enabled)
/********************************************************************************/
/*SE DESACTIVA EL CODIGO DE PROTECCION											*/
/********************************************************************************/
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
/* DECLARACION DE LAS ISR's QUE SE VAN A UTILIZAR EN EL PROGRAMA    			*/
/********************************************************************************/
void __attribute__((__interrupt__)) _U1RXInterrupt( void );
void __attribute__((__interrupt__)) _U2RXInterrupt( void );
void __attribute__((__interrupt__)) _T1Interrupt( void );

//****Comandos AT****
char CMD_AT[] = "AT\r";
char CMD_ATE0[] = "ATE0\r";
char CMD_UPSND[] = "AT+UPSND=0,8\r";            // Comprobamos el estado del perfil de conexion GPRS
char CMD_CGDCONT[] = "AT+CGDCONT=8\r";          // We removed that invalid context
char CMD_UPSD[] = "AT+UPSD=0,100,1\r";          // El perfil #0 es mapeado en CID = 1
char CMD_UPSDA[] = "AT+UPSDA=0,3\r";            // Activacion del perfil PSD #0
char CMD_USOCR[] = "AT+USOCR=6\r";              // Creacion del socket tipo TCP
char CMD_USOCO[] = "AT+USOCO=0,\"3.tcp.ngrok.io\",22434\r";     // Conexion al servidor
char CMD_USOWR[] = "AT+USOWR=0,13\r";           // Indica el tamanio del mensaje a mandar
char CMD_USOCL[] = "AT+USOCL=0\r";              // Se cierra la conexion
char CMD_UPSDA_CERRAR[] = "AT+UPSDA=0,4\r";     // Se cierra conexion PDP
char CMD_USODL[] = "AT+USODL=0\r";              // Se activa the direct link mode for socket #0
char CMD_STOP_DIRECT_LINK[] = "+++";            // Exit from direct link mode

void iniPerifericos( void ); 
void iniUART1( void );
void iniUART2( void );
void iniTimer1( void );
void iniIoT( void );
void iniInterrupciones( void );

void habilitaUARTS( void );
void habilitaTimer( void );

extern void RETARDO_1s( void );
extern void RETARDO_50us( void );
extern void RETARDO_15ms( void );

/* Funciones que se ocupan para el envio y recepcion de los datos del sensor MCP39F511A */
void obtenParametros( void );
void enviaFrame( unsigned char registro, unsigned char bytes, unsigned char checksum );
void enviaDato( unsigned char dato );
void mandaServer( void );

/* Funcion que se ocupa para el envio de comandos AT */
void comandoAT( char comando[] );

/* Declaracion de variables globales */
unsigned char mandaFrame;
unsigned char muestras[19] = {};
unsigned char state;
unsigned char numBytes;
int checksum;
int i;

char tiempoComando;

/* Variables globales para la re conexion */
char responseReconexion[40];
unsigned char positionReconexion;
unsigned char reconexionTCP;

int main(void) {
      
    iniPerifericos();    
    iniUART1();
    iniUART2(); 
    iniTimer1();
    iniIoT();
    iniInterrupciones();
    
    habilitaUARTS(); 
    
    reconexionTCP = 0;
    
    RETARDO_1s();
    RETARDO_1s();
    RETARDO_1s();
    RETARDO_1s();
    RETARDO_1s();
    
    comandoAT( CMD_AT );  
    comandoAT( CMD_ATE0 );    
    comandoAT( CMD_CGDCONT );
    comandoAT( CMD_UPSD );
    
    RETARDO_1s();
    RETARDO_1s();
    RETARDO_1s();
    
    comandoAT( CMD_UPSDA );
    comandoAT( CMD_UPSND );
    comandoAT( CMD_USOCR );
    
    reconexionTCP = 1;
    while( reconexionTCP ){
        positionReconexion = 0;
        comandoAT( CMD_USOCO );
        if( responseReconexion[2] == 69 ){
            RETARDO_1s();
            RETARDO_1s();
            RETARDO_1s();
            RETARDO_1s();
            comandoAT( CMD_USOCR );
        }
        else
            reconexionTCP = 0;
    }
    
    comandoAT( CMD_USODL );
            
    mandaFrame = 0;        
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

/* @brief: Funcion que inicializa los perifericos a utilizar tanto para la parte del 
 * sensor como para el modulo IoT */
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
    
    /* Configuracion U1RX que se usara para el sensor */
    TRISCbits.TRISC14 = 1;    //U1ARX
    /* Configuracion U2RX que se usara para el modulo IoT */
    TRISFbits.TRISF4 = 1;     //U2RX
    
    /* Inicializacion del mikroBUS2 para el moulo 4G */
    /*                | Pin MikroBUS IoT | Pin MikroBUS2 TESE |*/
    TRISBbits.TRISB5 = 1;  // STA                 AN5               
    TRISDbits.TRISD1 = 0;  // PWK                 RST
    TRISBbits.TRISB8 = 0;  // RTS                 CS
    TRISDbits.TRISD3 = 1;  // RI                  PWM
    TRISDbits.TRISD9 = 1;  // CTS                 INT
}

/* @brief: Configuracion del UART 1. En el UART 1 se conectara el sensor de monitoreo
 * de corriente y voltaje para la toma de parametros y el posterior envio de estos 
 * al modulo IoT. La velocidad que se configuro es 9600 Baudios 
 * Trama: 8 bits x dato, sin paridad, 1 bit de inicio, 1 bit de stop
 */
void iniUART1()
{
    U1MODE = 0X0400;  // 4 para ALTIO UART communicates using UxATX and UxARX  
    U1STA = 0X8000;   // 8 para UTXISEL: Transmission Interrupt Mode Selection bit
    U1BRG = 11;       // BaudRate 9600
}

/* @brief: Configuracion del UART 2. En el UART 2 se conectara el modulo IoT que enviara
 * cada uno de los parametros al servidor envevido a traves de la red 4G. La velocidad 
 * que se configuro es 9600 Baudios 
 * Trama: 8 bits x dato, sin paridad, 1 bit de inicio, 1 bit de paro
 */
void iniUART2()
{
    U2MODE = 0X0020;  // 2 para Auto Baud Enable bit
    U2STA = 0X8000;   // 8 para UTXISEL: Transmission Interrupt Mode Selection bit
    U2BRG = 11;       // BaudRate 9600 
}

/* @brief: Configuracion del TIMER 1. El TIMER 1 permitira la toma de muestras en un lapso
 * de tiempo periodico especificado por PRx, este valor se calculo con base en la frecuencia
 * requerida que es de 1Hz y la frecuencia del oscilador interno
 */
void iniTimer1()
{
    TMR1 = 0;         // Iniciamos el contador en 0
    PR1 = 0X7080;     // PRx = 28800
    T1CON = 0X0020;   // 2 para usar la pre-escala de 64
}

/* @brief: Inicializacion del modulo IoT siguiendo las especificaciones de la hoja
 * de datos del modulo LARA que se esta usando
 */
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

/* @brief: Inicializacion de las interrupciones, para este caso se usaran las interrupciones
 * UxRX de ambos UART's, asi como la interrupcion del TIMER 1
 */
void iniInterrupciones()
{    
    IFS0bits.U1RXIF = 0;    // UART1 Receiver Interrupt Flag Status bit
    IEC0bits.U1RXIE = 1;    // UART1 Receiver Interrupt Enable bit (Interrupt request enabled)
    
    IFS1bits.U2RXIF = 0;    // UART2 Receiver Interrupt Flag Status bit
    IEC1bits.U2RXIE = 1;    // UART2 Receiver Interrupt Enable bit (Interrupt request enabled)
        
    IFS0bits.T1IF = 0;      // Bit de la bandera de estado de la interrupcion
    IEC0bits.T1IE = 1;      // Habilitacion del bit de interrupcion
}

/* @brief: Esta funcion habilita ambos UART's */
void habilitaUARTS()
{
    U1MODEbits.UARTEN = 1;   // UART1 Enable bit: 1 para habilitar
    U1STAbits.UTXEN = 1;     // Transmit Enable bit: 1 para habilitar

    U2MODEbits.UARTEN = 1;   // UART2 Enable bit: 1 para habilitar
    U2STAbits.UTXEN = 1;    // Transmit Enable bit: 1 para habilitar
}

/* @brief: Esta funcion habilita el TIMER 1 */
void habilitaTimer()
{
    T1CONbits.TON = 1; 
}

/* @brief: Esta funcion procesa la respuesta obtenida por parte del sensor de monitoreo
 * MCP39F511A, la trama que se recibe tiene el siguiente formato
 * | System Status (2 bytes) | System Version (2 bytes)    | Voltage RMS (2 bytes)    | 
 * | Frequency (2 bytes)     | Thermisor Voltage (2 bytes) | Power Factor (2 bytes)   |  
 * | Current RMS (4 bytes)   | Active Power (4 bytes)      | Reactive Power (4 bytes) |
 * | Apparent Power (4 bytes)| 
 * Sin embargo, solo se toman aquellos parametros que se seleccionaron para la elaboracion
 * del Trabajo Terminal 
 */
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt( void )
{
    char response; 
    response = U1RXREG;
    
    if( state == 1 ){               // Recepcion del ACK o NACK por parte del sensor
        if( response == ACK ){
            state++;
            checksum += response;
        }
    }
    
    else if( state == 2 ){          // Recepcion del numero de bytes a leer
        numBytes = response;
        checksum += numBytes;
        state++;
    }
    
    else if( state < numBytes ){    // Recepcion de los datos
        if( ( state > 3 && state < 7 ) || ( state > 10 && state < 13 ) || ( state > 22 && state < 27 ) ); // Se excluyen los que no se utilizaran en el TT
        else{
            muestras[i] = response; // Almacenamiento de las muestras que son de nuestro interes
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

/* @brief: Esta funcion realiza las tareas de la interrupcion de recepcion del UART2,
 * la cual se encarga de esperar la respuesta del modulo IoT hasta que se terminen de
 * ejecutar los comandos AT
 */
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt( void )
{
    LATBbits.LATB0 = 1;         // LED
    char response;
    
    response = U2RXREG;
    
    if( response == 10 ){       // <LF>
        response = '_';
        tiempoComando--;
    }
    
    else if( response == 62 )   // '>'
        tiempoComando--;

    else if( response == 64 )   //  @
        tiempoComando--;
    
    if( reconexionTCP == 1 ){
        if( response == 13 )
            response = '.';
        else if( response == 32 )
            response = '-';
        responseReconexion[positionReconexion] = response;
        positionReconexion++;
    }
       
    LATBbits.LATB0 = 0;         // LED
    IFS1bits.U2RXIF = 0;
}

/* @brief: Esta funcion realiza las tareas del TIMER 1, las cuales consisten en la activacion
 * de ciertas banderas que ayudan a determinar el momento en que se deben tomar y envias las 
 * muestras del sensor al modulo IoT 
 */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt( void )
{           
    mandaFrame = 1;  
    IFS0bits.T1IF = 0;
}

/* @brief: Especificacion de los parametros que se desean obtener del sensor */
void obtenParametros()
{
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
    enviaDato( checksum );      // Checksum
}
    
/* @brief: Envio de cada caracter del frame al sensor por medio de UART1 
 * @param: Caracter a enviar por medio de UART1
 */
void enviaDato( unsigned char dato )
{
    IFS0bits.U1TXIF = 0;
    U1TXREG = dato;
    while( !IFS0bits.U1TXIF );  // Esperamos a que se genere la interrupcion para saber que el dato ya se mando
    IFS0bits.U1TXIF = 0;
}

void comandoAT( char comando[] )
{
    IFS1bits.U2TXIF = 0;
    int j;
    
    for( j = 0; comando[j] != '\0'; j++){
        U2TXREG = comando[j];
        while( !IFS1bits.U2TXIF );  //Mientras no se genere la interrupcion, va a esperar
        IFS1bits.U2TXIF = 0;
    }
   
    tiempoComando = 2;
    while( tiempoComando > 0 ){
        RETARDO_1s();
    }  // tiempoComando disminuye con la interrupcion U2RXInterrupt  
}

/* @brief: Envio de la trama al modulo IoT por medio de UART2. El formato de la trama
 * es el siguiente:
 * | Identificador (2 bytes) | System Status (1 byte)  | Voltage RMS (2 bytes) |
 * | Frequency (2 bytes)     | Power Factor (2 bytes)  | Current RMS (4 bytes) |  
 * | Active Power (4 bytes)  | Apparent Power (4 bytes)| 
 */
void mandaServer()
{        
    int j;
    IFS1bits.U2TXIF = 0;

    U2TXREG = 0X41;
    U2TXREG = 0X41;
    
    for( j = 0; j < 19; j++ ){
        U2TXREG = muestras[j];
        while( !IFS1bits.U2TXIF );
        IFS1bits.U2TXIF = 0;
    }
}
