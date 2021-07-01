# Trabajo-Terminal-Mod-Monitoreo
 Code necessary to configure a DSPIC30F4013 microcontroller to communicate via UART2 with a 4G-LTE LARA-R2 IoT module. The UART1 is also configured for microcontroller-sensor MCP39F511A communication, a state machine was implemented that discards those bytes belonging to parameters that are not going to be monitored.

## Tools used 💻
* C - *Programming language* - Implementation of the logic to obtain parameters and send them.
* Assembler - *Programming language* - Implementation of routines.
* MPLAB - *Development environment* - Creation of the project in C language and assembler.
