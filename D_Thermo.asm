    __CONFIG  _CP_OFF & _CPD_OFF & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT & _MCLRE_OFF & _FCMEN_OFF & _IESO_OFF
                LIST        P = 16F690
    #include "P16F690.inc"
    errorlevel -302	; suppress the bank sel check warnings

;**********REGISTERS LABEL EQUATES**********************************************
W           EQU 00h
GO          EQU 01h
UNITS	    EQU 20h
TENS	    EQU 21h
REG0	    EQU 22h
REG1	    EQU 23h
REG2	    EQU 24h
CSTATUS	    EQU 25h
COUNT	    EQU 26h
ADVALUE	    EQU 27h
W_TEMP	    EQU	28h
DELLOOP     EQU	29h
LEVEL       EQU 2Ah
LowLEVEL    EQU 2Bh
HighLEVEL   EQU 2Ch
MyREG       EQU 2Dh
SetCOUNT    EQU 2Eh
SetAlarm    EQU 2Fh
W_TEMP2	    EQU	30h
;*******************************************************************************

;**********BEGIN****************************************************************
	ORG     0x00    ; Program Start address
	goto    Setup   ; Jump to program setup
;*******************************************************************************

;**********INITIAL TEMPERATURE VALUES*******************************************
	ORG	0x2100
	DE	14,19
;*******************************************************************************
	
;**********ISR******************************************************************
        ORG         0x04            ; ISR starting address
        clrwdt                      ; Clear Watch Dog Timer
        movwf       W_TEMP          ; Context saving of WREG
        bcf         STATUS,RP0       ; Bank0
        bcf         STATUS,RP1
        btfsc       INTCON,T0IF     ; Check which ISR to run
        goto        TMR0ISR
        btfsc       PIR1,TMR2IF     ; No match has occured
        goto        TMR2ISR
	btfsc	    T1CON,TMR1ON
	goto	    TMR1ISR
        goto        ISREND

TMR0ISR btfss       INTCON,T0IE     ;
        goto        ISREND
	movlw	    HIGH LookUp         ;Avoid PCL rollover
	movwf	    PCLATH                                ;Move HIGH address of the lookup table into this
	movf	    TENS,W            
	call	    LookUp                                 ;Lookup table
        movwf       PORTC
        movlw       b'00010000'     ; Enable TENS SSD in PORTB
        movwf       PORTB
        call        Del1ms          ; Time delay to allow the SSD to be on

        movf        UNITS,W         ; Load UNITS value in WREG
        call        LookUp          ; Convert value to BCD for display in PORTC
        movwf       PORTC
        movlw       b'00100000'     ; Enable UNITS SSD in PORTB
        movwf       PORTB
        call        Del1ms          ; Time delay to allow the SSD to be on
        goto        ISREND

TMR1ISR
        decfsz      SetAlarm
        goto        ISREND
        movlw       d'15'
        movwf       SetAlarm           ; Timer2 count value
	movlw	    02h
        xorwf       MyREG
	goto	    ISREND
	
TMR2ISR decfsz      SetCOUNT
        goto        ISREND
        movlw       d'46'
        movwf       SetCOUNT           ; Timer2 count value
        bcf         MyREG,0

ISREND  movfw       W_TEMP              ; Retrieve contents of WREG from previous routine
        bcf         STATUS,RP0       ; Bank0
        bcf         STATUS,RP1
        bcf         INTCON,T0IF         ; Clear Timer0 Overflow Interrupt Flag
        bcf         PIR1,TMR2IF         ; No match has occured
        retfie
;*******************************************************************************

;**********SETUP****************************************************************
Setup   bcf     STATUS,RP0	; Select Bank 2
        bsf     STATUS,RP1	; Select Bank 2
        bsf     ANSEL,ANS0	; Enable analog I/O
        bcf     STATUS,RP1
        bsf     STATUS,RP0	; Select bank 1
        movlw	b'00110000'	; Select ADC FRC clock
        movwf	ADCON1
        bsf     TRISA,TRISA0	; PORTA, 0 as input
	bcf	TRISA,TRISA1
        movlw   0C0h
        movwf   TRISB       ; PORTB used as control port for multiplexing <4-5>, and button inputs <6-7>
        clrf    TRISC		; PORTC used for output to SSD
        clrf	OPTION_REG	;
        bsf     OPTION_REG,PS2	; Prescalar 100 1:32
        bsf     INTCON,GIE      ; Enable global interrupt
        bsf     INTCON,T0IE     ; Enable Timer0 Overflow Interrupt
	bsf	INTCON,PEIE
        bcf     STATUS,RP0  ; Select Bank 0
	clrf    T1CON
        movlw	0x81	    ; ADC result conversion right justified and ADC enabled
        movwf   ADCON0
	clrf	UNITS
	clrf	TENS
	movlw   d'15'
        movwf   SetAlarm           ; Timer1 count value
        movlw   d'46'
        movwf   SetCOUNT           ; Timer2 count value
        goto    Loop
;*******************************************************************************

;**********MAIN LOOP************************************************************
Loop    
        call	Delay           ; 340ms Delay
        call	Delay
        call	Sample          ; Sample analog input and perform ADC
	movwf	W_TEMP2
	call	Bin2BcdStart	; Convert ADC result to BCD
        bcf     STATUS,RP1
        bcf     STATUS,RP0      ; Select bank 0
        movwf	UNITS           ; Save BCD result in UNITS and TENS registers
        movwf	TENS
        movlw	b'00001111'
        andwf	UNITS,1         ; UNITS = WREG lower nibble
        swapf	TENS,1
        andwf	TENS,1          ; TENS = WREG higher nibble
        call	Delay
        call	Delay
	
	movlw	00h
        call    ReadEEPROM
        movwf   LowLEVEL
	
	movlw	01h
        call    ReadEEPROM
        movwf   HighLEVEL
	
	movf	W_TEMP2,W
	call    ValidateLimits
	movf	W_TEMP2,W
	

        btfss   PORTB,6         ; Check if LOW button is pressed
        call    SetLow
        btfss   PORTB,7         ; Check if HIGH button is pressed
        call    SetHigh

        goto    Loop            ; Repeat loop indefinetly
;*******************************************************************************

;**********READ EEPROM************************************************************
ReadEEPROM
        BANKSEL EEADR               ;
        MOVWF   EEADR               ;Data Memory Address to read
        BANKSEL EECON1              ;
        BCF     EECON1,EEPGD        ;Point to DATA memory
        BSF     EECON1,RD           ;EE Read
        BANKSEL EEDAT               ;
        MOVF    EEDAT,W             ;W = EEDAT
        BCF     STATUS,RP1          ;Bank 0
        return
;*******************************************************************************

;**********WRITE EEPROM************************************************************
WriteEEPROM
        BANKSEL EEADR ;
        MOVWF   EEADR ;Data Memory Address to write
        call    LoadW
	BANKSEL EEDAT ;
        MOVWF   EEDAT ;Data Memory Value to write
        BANKSEL EECON1 ;
        BCF     EECON1, EEPGD ;Point to DATA memory
        BSF     EECON1, WREN ;Enable writes
        BCF     INTCON, GIE ;Disable INTs.
        BTFSC   INTCON, GIE ;SEE AN576
        GOTO    $-2
        MOVLW   55h ;
        MOVWF   EECON2 ;Write 55h
        MOVLW   0AAh ;
        MOVWF   EECON2 ;Write AAh
        BSF     EECON1, WR ;Set WR bit to begin write
        BSF     INTCON, GIE ;Enable INTs.
        BCF     EECON1, WREN ;Disable writes
        BANKSEL 0x00 ;Bank 0
        return
;*******************************************************************************

;**********LOAD W WITH LEVEL****************************************************
LoadW   BCF     STATUS,RP0
        BCF     STATUS,RP1
        movf    LEVEL,W
        return
;*******************************************************************************	

;*******************************SET TEMP****************************************
;**********SET LOW TEMP*********************************************************
SetLow  bsf     MyREG,0
        movlw   d'6'
        movwf   DELLOOP
LowDeb1 call    Delay           ; 340ms*6 = 2.04s
        decfsz  DELLOOP         ; Button debouncing
        goto    LowDeb1

        btfsc   PORTB,6         ; Check if LOW button is still pressed
        return

        call    SetupTMR2
LowLoop	movlw	00h
        call    ReadEEPROM
        movwf   LowLEVEL
        call	Bin2BcdStart	; Convert low temperature level to BCD
        bcf     STATUS,RP1
        bcf     STATUS,RP0      ; Select bank 0
        movwf	UNITS           ; Save BCD result in UNITS and TENS registers
        movwf	TENS
        movlw	b'00001111'
        andwf	UNITS,1         ; UNITS = WREG lower nibble
        swapf	TENS,1
        andwf	TENS,1          ; TENS = WREG higher nibble

        bcf     INTCON,T0IE     ; Disable Timer0 Overflow Interrupt
        clrf    PORTB           ; Turn off  SSDs (flash)

        movlw   d'3'
        movwf   DELLOOP         ; 1s delay for flashing
LowDeb2 call    Delay           ; 340ms*3 = 1.02s
        decfsz  DELLOOP         ;
        goto    LowDeb2

        bsf     INTCON,T0IE     ; Enable Timer0 Overflow Interrupt (Flash)

        ; Setting the temperature
        btfsc   PORTB,6         ; Is LOW button pressed?
        goto    CheckB7         ; Is 3s cycle over?
        decf    LowLEVEL        ; If LOW button is pressed, decrement low limit
	movf	LowLEVEL,W
	movwf	LEVEL
	movlw	00h
        call    WriteEEPROM
        movlw   d'46'           ; Continue 3s cycle
        movwf   SetCOUNT

CheckB7 btfsc   PORTB,7         ; Is HIGH button pressed?
        goto    CheckMyREG      ; Is 3s cycle over?
        incf    LowLEVEL        ; If HIGH button is pressed, increment low limit
	movf	LowLEVEL,W
	movwf	LEVEL
	movlw	00h
        call    WriteEEPROM
        movlw   d'46'           ; Continue 3s cycle
        movwf   SetCOUNT

CheckMyREG
        btfss   MyREG,0         ; If MyReg,0 is cleared, 3 seconds have elapsed
        goto    SetLowEND       ; therefore end subroutine, else continue

        movlw   d'3'
        movwf   DELLOOP         ; 1s delay for flashing
LowDeb3 call    Delay           ; 340ms*3 = 1.02s
        decfsz  DELLOOP         ; Button debouncing
        goto    LowDeb3
        goto    LowLoop         ; Repeat loop until Timer2 interrupt (3 second have elapsed)

SetLowEND
        bcf     T2CON,TMR2ON    ; Turn off Timer2
        bsf     INTCON,T0IE     ; Enable Timer0 Overflow Interrupt (Back to normal operation)
        return
;*******************************************************************************

;**********SET HIGH TEMP********************************************************
SetHigh bsf     MyREG,0
        movlw   d'6'
        movwf   DELLOOP
HighDeb1
        call    Delay           ; 340ms*6 = 2.04s
        decfsz  DELLOOP         ; Button debouncing
        goto    HighDeb1

        btfsc   PORTB,7         ; Check if HIGH button is still pressed
        return

        call    SetupTMR2
HighLoop
	movlw	01h
        call    ReadEEPROM
        movwf   HighLEVEL
        call	Bin2BcdStart	; Convert low temperature level to BCD
        bcf     STATUS,RP1
        bcf     STATUS,RP0      ; Select bank 0
        movwf	UNITS           ; Save BCD result in UNITS and TENS registers
        movwf	TENS
        movlw	b'00001111'
        andwf	UNITS,1         ; UNITS = WREG lower nibble
        swapf	TENS,1
        andwf	TENS,1          ; TENS = WREG higher nibble

        bcf     INTCON,T0IE     ; Disable Timer0 Overflow Interrupt
        clrf    PORTB           ; Turn off  SSDs (flash)

        movlw   d'3'
        movwf   DELLOOP         ; 1s delay for flashing
HighDeb2
        call    Delay           ; 340ms*3 = 1.02s
        decfsz  DELLOOP         ;
        goto    HighDeb2

        bsf     INTCON,T0IE     ; Enable Timer0 Overflow Interrupt (Flash)

        ; Setting the temperature
        btfsc   PORTB,6         ; Is LOW button pressed?
        goto    CheckB7H        ; Is 3s cycle over?
        decf    HighLEVEL       ; If LOW button is pressed, decrement low limit
        movf	HighLEVEL,W
	movwf	LEVEL
	movlw	01h
        call    WriteEEPROM
        movlw   d'46'           ; Continue 3s cycle
        movwf   SetCOUNT

CheckB7H
        btfsc   PORTB,7         ; Is HIGH button pressed?
        goto    CheckMyREGH     ; Is 3s cycle over?
        incf    HighLEVEL       ; If HIGH button is pressed, increment low limit
        movf	HighLEVEL,W
	movwf	LEVEL
	movlw	01h
	call	WriteEEPROM
        movlw   d'46'           ; Continue 3s cycle
        movwf   SetCOUNT

CheckMyREGH
        btfss   MyREG,0         ; If MyReg,0 is cleared, 3 seconds have elapsed
        goto    SetHighEND      ; therefore end subroutine, else continue

        movlw   d'3'
        movwf   DELLOOP         ; 1s delay for flashing
HighDeb3
        call    Delay           ; 340ms*3 = 1.02s
        decfsz  DELLOOP         ; Button debouncing
        goto    HighDeb3
        goto    HighLoop         ; Repeat loop until Timer2 interrupt (3 second have elapsed)

SetHighEND
        bcf     T2CON,TMR2ON    ; Turn off Timer2
        bsf     INTCON,T0IE     ; Enable Timer0 Overflow Interrupt (Back to normal operation)
        return
;*******************************************************************************
;*******************************/SET TEMP***************************************

;**********VALIDATE LIMITS******************************************************
ValidateLimits	
	subwf	HighLEVEL,W
	btfss	STATUS,C
	goto	AlarmONH
	
	movf	W_TEMP2,W
	subwf	LowLEVEL,W
	btfss	STATUS,DC
	goto	AlarmOFF
	goto	AlarmONL
	
AlarmONH
	bsf     T1CON,TMR1ON
	btfsc   PIR1,TMR1IF
        call    04h
	
	btfss	MyREG,1
	goto	PWMLH
        goto    PWMHH            
	
PWMLH	bcf	PORTA,1
	goto	ValidateLimits
PWMHH	bsf	PORTA,1
	goto	ValidateLimits
	
AlarmONL
	bsf     T1CON,TMR1ON
	btfsc   PIR1,TMR1IF
        call    04h
	
	btfss	MyREG,1
	goto	PWMLL
        goto    PWMHL            
	
PWMLL	bcf	PORTA,1
	goto	ValidateLimits
PWMHL	bsf	PORTA,1
	goto	ValidateLimits
	
AlarmOFF
	bcf	PORTA,1
	bcf     T1CON,TMR1ON
	return
;*******************************************************************************

;**********SETUP AND ENABLE TIMER2**********************************************
SetupTMR2
        bcf     STATUS,RP0       ; Bank0
        bcf     STATUS,RP1
        clrf    T2CON
        comf    T2CON           ; Prescalar = 16, Turn on Timer2, Postscaler = 16
        clrf    TMR2            ; Initial count value = 0
        bcf     PIR1,TMR2IF     ; No match has occured
        bsf     INTCON,PEIE     ; Enable peripheral interrupts
        bsf     INTCON,GIE      ; Enable global interrupt
        bsf     STATUS,RP0      ; Bank1
        clrf    PR2
        comf    PR2             ; Final count value = 255
        bsf     PIE1,TMR2IE     ; Enale Timer2 Interrupt
        bcf     STATUS,RP0      ; Bank0
        movlw   d'46'
        movwf   SetCOUNT        ; Timer2 count value
        return
;*******************************************************************************

;**********ADC CONVERSION*******************************************************
Sample	bcf     STATUS,RP1
        bcf     STATUS,RP0      ; Select bank 0
        bsf     INTCON,T0IE
        call    SampleTime          ; Acquisition delay
        bsf     ADCON0,GO       ; Start A/D conversion cycle
        btfsc   ADCON0,GO       ; Polling: Check if ADC is complete
        goto    $-1

        bcf     STATUS,C
        bsf     STATUS,RP0      ; Select bank 1
        rrf     ADRESL,C        ; Divide ADC result by 2 and save result in WREG
        bcf     STATUS,RP0      ; Select bank 0
        return
;*******************************************************************************

;**********LOOKUP TABLE*********************************************************
LookUp  addwf   PCL	; Add counter value to PC, PC will jump to that specific line and fetch (place in WREG) the corresponding SSD code
        retlw   40h	; 0	    (common anode display)
        retlw   79h	; 1
        retlw   24h	; 2
        retlw   30h	; 3
        retlw   19h	; 4
        retlw   12h	; 5
        retlw   02h	; 6
        retlw   78h	; 7
        retlw   00h	; 8
        retlw   10h	; 9
;*******************************************************************************

;**********BIN TO BCD***********************************************************
Bin2BcdStart
        movwf	REG0            ; Value will be manipulated from REG0.
        clrf	REG1
        clrf	REG2
        movlw	08h             ; total number of iterations = 8 for 8 bits
        movwf	COUNT
        movlw	b'00000011'     ; add 3 for binary to bcd
        movwf	ADVALUE
Bin2Bcd call	CheckIfGreater	; Check if lower nibbles of REG1 and REG2 are >= 5.
        bcf     STATUS,C            ; Clear carry flag of STATUS before every rotation.
        clrf	CSTATUS         ; Each iteration starts off with a clean CSTATUS.
        rlf     REG0,1              ; Shift register value to the left and shift MSB to CSTATUS
        btfsc	STATUS,C        ; via the carry flag.
        incf	CSTATUS,1       ; Store carry flag of REG0 in CSTATUS.
        bcf     STATUS,C            ;
        rlf     CSTATUS,1           ; Shift register value left and reserve bit 0 for C flag of REG1
        bcf     STATUS,C
        rlf     REG1,1              ; Shift register value to the left and shift MSB to CSTATUS
        btfsc	REG1,04h        ; via the bit 4 in high nibble (overflow of lower nibble)
        incf	CSTATUS,1       ; Store carry flag of REG1 lower nibble in CSTATUS
        bcf     STATUS,C
        rlf     REG2,1

	; CSTATUS [X|X|X|X|X|X|X|REG0|REG1] Now CSTATUS contains the C flag for registers listed
	; X - Don't care

        btfsc	CSTATUS,1       ; If bit 1 of CSTATUS is set
        incf	REG1,1          ; add 1 to REG1.
        btfsc	CSTATUS,0       ; If bit 2 of CSTATUS is set
        incf	REG2,1          ; add 1 to REG2
        decfsz	COUNT,1         ; while COUNT != 0 from 8
        goto	Bin2Bcd         ; do next iteration

Concatenate
; Now combine the low nibbles of REG1 and REG2 and store result in WREG
        movlw	b'00001111'
        andwf	REG1,1      ; Block out the high nibbles
        andwf	REG2,1      ;
        swapf	REG2,0      ; Low nibble of REG2 should be the high nibble
        addwf	REG1,0      ; of WREG
        return
;*******************************************************************************

;**********CHECK IF GREATER THAN 5**********************************************
CheckIfGreater
Check1  movf    REG1,0
        andlw   b'00001111'     ; Only need the lower niblle
        sublw   05h
        btfsc   STATUS,Z        ; If DC flag = 1, value > 5
        goto    add3to1
        btfss   STATUS,C        ; If carry flag = 0, value = 5
        goto    add3to1
        goto    Check2

add3to1 movf    ADVALUE,0
        addwf   REG1,1

Check2  movf    REG2,0
        andlw   b'00001111'     ; Only need the lower niblle
        sublw   05h
        btfsc   STATUS,Z        ; If DC flag = 1, value > 5
        goto    add3to2
        btfss   STATUS,C        ; If carry flag = 0, value = 5
        goto    add3to2
        goto    CheckIfGreaterEnd

add3to2 movf    ADVALUE,0
        addwf   REG2,1

CheckIfGreaterEnd
        return
;*******************************************************************************

;**********DELAY 1MS************************************************************
Del1ms  movlw   d'250'	    ; Initial value
loop1ms addlw   d'255'	    ; Dec WREG
        btfss   STATUS,Z    ; Zero flag set?
        goto    loop1ms	    ; No, keep looping
        return              ; Yes, 1ms done
;*******************************************************************************

;**********340ms DELAY**********************************************************
Delay	movlw	d'255'
        movwf	COUNT
Begin	movlw   d'250'	    ; Initial value
loopDel addlw   d'255'	    ; Dec WREG
        btfss   STATUS,Z    ; Zero flag set?
        goto    loopDel	    ; No, keep looping
        decfsz	COUNT
        goto	Begin
        return              ; Yes, 340ms done
;*******************************************************************************

;**********Acquisition DELAY****************************************************
SampleTime
        nop
        nop
        nop
        nop
        nop
        nop
        return
;*******************************************************************************
        end