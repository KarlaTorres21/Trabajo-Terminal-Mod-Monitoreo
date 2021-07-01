.include "p30F4013.inc"
    
.GLOBAL	_RETARDO_5ms
.GLOBAL	RETARDO_30ms
.GLOBAL	_RETARDO_300ms
.GLOBAL	_RETARDO_1s
.GLOBAL _RETARDO_50us
.GLOBAL _imprimeUART1
.GLOBAL _comandoAT
    
_comandoAT:
    PUSH	W0
    PUSH	W1

    inicio:
	MOV.B	    [W0++],	W1
	CP0.B	    W1
	BRA	    Z,		fin
	BCLR	    IFS1,	#U2TXIF
	MOV	    W1,		U2TXREG

    comparacion:
	BTSS	    IFS1,	#U2TXIF
	GOTO	    comparacion
	GOTO	    inicio
	
    fin:
	POP	    W0
	POP	    W1
       
_imprimeUART1:
	PUSH	W0
	PUSH	W1
	
	enviaCaracter:
	    MOV.B	[W0++],	    W1
	    CP0.B	W1
	    BRA		Z,	    finImprime
	    BCLR	IFS0,	    #U1TXIF
	    MOV		W1,	    U1TXREG
	
	comparacionFin:
	    BTSS	IFS0,	    #U1TXIF
	    GOTO	comparacionFin
	    GOTO	enviaCaracter
	    
	finImprime:
	    POP	    W0
	    POP	    W1
	    RETURN    
    
_RETARDO_50us:
    REPEAT	#100
    NOP

    RETURN
    
;;******************************************************************************
;;DESCRICION:	ESTA RUTINA GENERA UN RETARDO DE 30ms
;;PARAMETROS: 	NINGUNO
;;RETORNO: 	NINGUNO
;;******************************************************************************
RETARDO_30ms:
    CALL	_RETARDO_5ms
    CALL	_RETARDO_5ms
    CALL	_RETARDO_5ms
    CALL	_RETARDO_5ms
    CALL	_RETARDO_5ms
    CALL	_RETARDO_5ms

    RETURN
		
;******************************************************************************
;DESCRICION:	ESTA RUTINA GENERA UN RETARDO DE 300ms
;PARAMETROS: 	NINGUNO
;RETORNO: 	NINGUNO
;******************************************************************************
_RETARDO_300ms:
    DO		#9,		CICLO_RETARDO_300ms
    CALL	RETARDO_30ms
    
    CICLO_RETARDO_300ms:
	NOP

    RETURN

;******************************************************************************
;DESCRICION:	ESTA RUTINA GENERA UN RETARDO DE 5ms
;PARAMETROS: 	NINGUNO
;RETORNO: 	NINGUNO
;******************************************************************************
_RETARDO_5ms:
    PUSH	W0
    MOV		#3276,		W0
    
CICLO1_5ms:
    DEC		W0,		W0
    BRA		NZ,		CICLO1_5ms

    POP		W0

    RETURN

;******************************************************************************
;DESCRICION:	ESTA RUTINA GENERA UN RETARDO DE 1S
;PARAMETROS: 	NINGUNO
;RETORNO: 	NINGUNO
;******************************************************************************
_RETARDO_1s:
    PUSH	W0
    PUSH	W1
    MOV		#10,			W1
    
CICLO2_1S:
	    CLR		W0
	    
CICLO1_1S:
	    DEC		W0,		W0
	    BRA		NZ,		CICLO1_1S

	    DEC		W1,		W1
	    BRA		NZ,		CICLO2_1S

	    POP		W1
	    POP		W0

	    RETURN

