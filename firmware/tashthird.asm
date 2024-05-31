;;; 80 characters wide please ;;;;;;;;;;;;;;;;;;;;;;;;;; 8-space tabs please ;;;


;
;;;
;;;;;  TashThird: ADB Modem Emulator
;;;
;


;;; Connections ;;;

;;;                                                            ;;;
;                            .--------.                          ;
;                    Supply -|01 \/ 08|- Ground                  ;
;           ADB <-->    RA5 -|02    07|- RA0    ---> UART TX     ;
;    Pushbutton --->    RA4 -|03    06|- RA1    <--- UART RX     ;
;      UART RTS --->    RA3 -|04    05|- RA2    ---> UART CTS    ;
;                            '--------'                          ;
;;;                                                            ;;;


;;; Assembler Directives ;;;

	list		P=PIC12F1840, F=INHX32, ST=OFF, MM=OFF, R=DEC, X=ON
	#include	P12F1840.inc
	errorlevel	-302	;Suppress "register not in bank 0" messages
	__config	_CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
			;_FOSC_INTOSC	Internal oscillator, I/O on RA5
			;_WDTE_OFF	Watchdog timer disabled
			;_PWRTE_ON	Keep in reset for 64 ms on start
			;_MCLRE_OFF	RA3/!MCLR is RA3
			;_CP_OFF	Code protection off
			;_CPD_OFF	Data memory protection off
			;_BOREN_OFF	Brownout reset off
			;_CLKOUTEN_OFF	CLKOUT disabled, I/O on RA4
			;_IESO_OFF	Internal/External switch not needed
			;_FCMEN_OFF	Fail-safe clock monitor not needed
	__config	_CONFIG2, _WRT_OFF & _PLLEN_ON & _STVREN_ON & _LVP_ON
			;_WRT_OFF	Write protection off
			;_PLLEN_ON	4x PLL on
			;_STVREN_ON	Stack over/underflow causes reset
			;_LVP_OFF	High-voltage on Vpp to program


;;; Macros ;;;

DELAY	macro	value		;Delay 3*W cycles, set W to 0
	movlw	value
	decfsz	WREG,F
	bra	$-1
	endm

DNOP	macro
	bra	$+1
	endm


;;; Constants ;;;

ADBADDR	equ	0x05	;ADB default address for the modem
ADBHDLR	equ	0x36	;ADB handler ID for the modem

;WARNING: do NOT use RA2 for ADB, the Schmitt Trigger takes too long to react
ADB_PIN	equ	RA5	;Pin on PORTA where ADB is connected
PB_PIN	equ	RA4	;Pin on PORTA where pushbutton is connected
RTS_PIN	equ	RA3	;Pin on PORTA where RTS is connected
CTS_PIN	equ	RA2	;Pin on PORTA where CTS is connected

			;AP_FLAG:
AP_RST	equ	7	;Set when a reset condition is detected, user clears
AP_COL	equ	6	;Set when the transmission collided, user clears
AP_RXCI	equ	5	;Set when command byte in AP_BUF, user clears
AP_RXDI	equ	4	;Set when data byte in AP_BUF, user clears
AP_DONE	equ	3	;Set when transmission or reception done, user clears
AP_TXI	equ	2	;User sets after filling AP_BUF, interrupt clears
AP_SRQ	equ	1	;User sets to request service, user clears
AP_RISE	equ	0	;Set when FSA should be entered on a rising edge too

			;AM_R3H:
AM_COL	equ	7	;Set when a collision has occurred
AM_EXCE	equ	6	;Clear when an exceptional event has occurred
AM_SRQE	equ	5	;Set when service requests are enabled

			;AM_FLAG:
AM_ISON	equ	7	;Set when modem is "on" (always responds to Talk 0)
AM_DRVR	equ	6	;Set when modem driver is installed
AM_FLOW	equ	5	;Set when flow control is desired (stop Listen 0 cmds)
AM_FLOC	equ	4	;Set when flow control bit has changed
AM_STAT	equ	3	;Set when modem should send an 0x88 status to Mac
AM_0X95	equ	2	;Set when modem should send an 0x95 byte before next
AM_MOD1	equ	1	;Modem mode: 00: echo (default)  02: 1200 baud
AM_MOD0	equ	0	;            01: 300 baud        03: 2400 baud


;;; Variable Storage ;;;

	cblock	0x70	;Bank-common registers
	
	AP_FLAG	;ADB flags
	AP_FSAP	;Pointer to where to resume ADB state machine
	AP_SR	;ADB shift register
	AP_BUF	;ADB buffer
	AP_DTMR	;ADB down-cycle timer value
	AM_R3H	;ADB modem register 3 high byte
	AM_FLAG	;ADB modem flags
	AM_RXPU	;ADB modem Rx push pointer
	AM_RXPO	;ADB modem Rx pop pointer
	AM_TXPU	;ADB modem Tx push pointer
	AM_TXPO	;ADB modem Tx pop pointer
	PB_SR	;Pushbutton shift register
	X3
	X2
	X1
	X0
	
	endc

	;Linear memory:
	;0x2000-0x207F - ADB modem Rx (Listen 0) queue
	;0x2080-0x20BF - ADB modem Tx (Talk 0) queue


;;; Vectors ;;;

	org	0x0		;Reset vector
	goto	Init

	org	0x4		;Interrupt vector


;;; Interrupt Handler ;;;

Interrupt
	movlp	0		;Copy the Timer0 flag into the carry bit so it
	bcf	STATUS,C	; doesn't change on us mid-stream
	btfsc	INTCON,TMR0IF	; "
	bsf	STATUS,C	; "
	btfsc	STATUS,C	;If the Timer0 flag is set and the interrupt is
	btfss	INTCON,TMR0IE	; enabled, handle it as an event for the ADB
	bra	$+2		; state machine
	call	IntAdbTimer	; "
	movlp	0		; "
	movlb	7		;If the ADB pin has had a negative or positive
	btfsc	IOCAF,ADB_PIN	; edge, handle it as an event for the ADB state
	call	IntAdbEdge	; machine
	movlp	0		; "
	movlb	7		;If the RTS pin was asserted (low), reenable Tx
	btfsc	IOCAF,RTS_PIN	; interrupts
	call	IntRtsAsserted	; "
	movlb	0		;If Timer1 has overflowed, service the push-
	btfsc	PIR1,TMR1IF	; button
	call	IntPushbutton	; "
	movlb	0		;If the UART transmitter wants a byte, handle
	btfsc	PIR1,TXIF	; it
	call	IntTx		; "
	movlb	0		;If the UART receiver has a byte, handle it
	btfsc	PIR1,RCIF	; "
	call	IntRx		; "
	retfie

IntAdbTimer
	movlb	1		;Disable the Timer0 interrupt
	bcf	INTCON,TMR0IE	; "
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag and its mirror
	bcf	STATUS,C	; in the carry bit
	return
	
IntAdbEdge
	movlw	1 << ADB_PIN	;Toggle the edge that the IOC interrupt catches
	xorwf	IOCAN,F		; "
	xorwf	IOCAP,F		; "
	bcf	IOCAF,ADB_PIN	;Clear the interrupt flag
	btfsc	IOCAN,ADB_PIN	;If the edge we just caught is a rising edge,
	bra	IntAdbRising	; jump ahead, otherwise fall through
	;fall through

IntAdbFalling
	movlb	0		;If Timer0 overflowed, this falling edge is
	btfsc	STATUS,C	; the first after a too-long period, so handle
	bra	IntAdbTimeout	; it as a timeout
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag
	return

IntAdbRising
	movlb	0		;If Timer0 overflowed, this rising edge is at
	btfsc	STATUS,C	; the end of a reset pulse
	bra	IntAdbReset	; "
	movf	TMR0,W		;Save the current value of Timer0 so it can be
	movwf	AP_DTMR		; considered after its corresponding falling
	clrf	TMR0		; edge, then clear it and its flag
	bcf	INTCON,TMR0IF	; "
	btfss	AP_FLAG,AP_RISE	;If the flag isn't set that the state machine
	return			; wants to be resumed on a rising edge, done
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag
	return

IntAdbReset
	bsf	AP_FLAG,AP_RST	;Set the reset flag
	clrf	AP_DTMR		;Clear the down timer
	;fall through

IntAdbTimeout
	clrf	AP_FSAP		;Reset the ADB state machine
	clrf	TMR0		;Reset Timer0 and its flag and disable its
	bcf	INTCON,TMR0IF	; interrupt
	bcf	INTCON,TMR0IE	; "
	return

IntTx
	movf	AM_TXPU,W	;If the queue is empty, disable the Tx
	xorwf	AM_TXPO,W	; interrupt and return
	btfsc	STATUS,Z	; "
	bra	IntTx0		; "
	movlb	0		;If the RTS pin is high (not asserted), disable
	btfsc	PORTA,RTS_PIN	; the Tx interrupt and return
	bra	IntTx0		; "
	movf	AM_TXPO,W	;Load the Tx pop pointer into FSR0
	movwf	FSR0L		; "
	moviw	FSR0++		;Pop the top byte off the Tx queue and load it
	movlb	3		; for transmission, advancing the pointer
	movwf	TXREG		; "
	movf	FSR0L,W		;Copy the advanced pointer back into the Tx pop
	andlw	B'10111111'	; pointer with bit 6 snuffed (wrapping it)
	movwf	AM_TXPO		; "
	subwf	AM_TXPU,W	;Queue length is push point minus pop point
	btfsc	WREG,4		;If Tx queue length < 32, deassert flow control
	return			; and flag that the bit has changed if it has
	btfsc	AM_FLAG,AM_FLOW	; "
	bsf	AM_FLAG,AM_FLOC	; "
	bcf	AM_FLAG,AM_FLOW	; "
	return
IntTx0	movlb	1		;Disable TX interrupt and return
	bcf	PIE1,TXIE	; "
	return			; "

IntRx
	movf	AM_RXPU,W	;Load the queue push point into FSR0 so we can
	movwf	FSR0L		; dereference it
	incf	AM_RXPU,F	;Advance and wrap the queue push point
	bcf	AM_RXPU,7	; "
	movlb	3		;Get the byte from the UART and push it onto the
	movf	RCREG,W		; queue
	movwf	INDF0		; "
	movf	AM_RXPO,W	;Queue length is push point minus pop point
	subwf	AM_RXPU,W	; "
	andlw	B'01111111'	; "
	sublw	96		;Set carry if queue length <= 96
	movlb	2		;Deassert CTS, i.e. set it to 1, if queue length
	btfss	STATUS,C	; was > 96, i.e. more than 3/4 full
	bsf	LATA,CTS_PIN	; "
	return

IntRtsAsserted
	bcf	IOCAF,RTS_PIN	;Clear the interrupt
	movlb	1		;Enable Tx interrupts; if the queue is empty,
	bsf	PIE1,TXIE	; the interrupt will just get disabled again
	return

IntPushbutton
	movlb	0		;Clear the interrupt
	bcf	PIR1,TMR1IF	; "
	bcf	STATUS,C	;Rotate the current state of the pushbutton into
	btfsc	PORTA,PB_PIN	; the shift register
	bsf	STATUS,C	; "
	rlf	PB_SR,F		; "
	movf	PB_SR,W		;If we didn't catch a stable falling edge, do
	xorlw	B'10000000'	; nothing
	btfss	STATUS,Z	; "
	return			; "
	movf	AM_FLAG,W	;Switch off based on the current modem mode
	andlw	B'00000011'	; "
	brw			; "
	bra	IntPMo1		; "
	bra	IntPMo2		; "
	bra	IntPMo3		; "
	bcf	AM_FLAG,AM_MOD1	;Modem mode is 2400 baud, so set to echo and
	bcf	AM_FLAG,AM_MOD0	; flag that a status needs to be sent
	bsf	AM_FLAG,AM_STAT	; "
	movlb	3		;Disable UART receiver
	bcf	RCSTA,CREN	; "
	return
IntPMo1	bsf	AM_FLAG,AM_MOD0	;Modem mode is echo, so set to 300 baud and flag
	bsf	AM_FLAG,AM_STAT	; that a status needs to be sent
	movlb	3		; "
	movlw	0x68		; "
	movwf	SPBRGH		; "
	movlw	0x2A		; "
	movwf	SPBRGL		; "
	bsf	RCSTA,CREN	;Enable UART receiver
	return
IntPMo2	bcf	AM_FLAG,AM_MOD0	;Modem mode is 300 baud, so set to 1200 baud and
	bsf	AM_FLAG,AM_MOD1	; flag that a status needs to be sent
	bsf	AM_FLAG,AM_STAT	; "
	movlb	3		; "
	movlw	0x1A		; "
	movwf	SPBRGH		; "
	movlw	0x0A		; "
	movwf	SPBRGL		; "
	return
IntPMo3	bsf	AM_FLAG,AM_MOD0	;Modem mode is 1200 baud, so set to 2400 baud
	bsf	AM_FLAG,AM_STAT	; and flag that a status needs to be sent
	movlb	3		; "
	movlw	0x0D		; "
	movwf	SPBRGH		; "
	movlw	0x04		; "
	movwf	SPBRGL		; "
	return
	

;;; Mainline ;;;

Init
	banksel	OSCCON		;32 MHz (w/PLL) high-freq internal oscillator
	movlw	B'11110000'
	movwf	OSCCON

	banksel	RCSTA		;UART async mode, but receiver not enabled and
	movlw	B'01001000'	; baud rate not set just yet
	movwf	BAUDCON
	movlw	B'00100110'
	movwf	TXSTA
	movlw	B'10000000'
	movwf	RCSTA
	clrf	TXREG
	
	banksel	IOCAN		;ADB and RTS set IOCAF on negative edge
	movlw	(1 << ADB_PIN) | (1 << RTS_PIN)
	movwf	IOCAN

	banksel	OPTION_REG	;Timer0 uses instruction clock, 1:32 prescaler,
	movlw	B'01010100'	; thus ticking every 4 us; weak pull-ups on
	movwf	OPTION_REG

	banksel	T1CON		;Timer1 ticks once per instruction cycle
	movlw	B'00000001'
	movwf	T1CON

	banksel	ANSELA		;All pins digital, not analog
	clrf	ANSELA

	banksel	LATA		;Ready to pull ADB low when output, CTS asserted
	clrf	LATA

	banksel	TRISA		;Tx and CTS out, ADB is open-collector output,
	movlw	B'00111010'	; currently floating, pushbutton, Rx, RTS input
	movwf	TRISA

	banksel	PIE1		;Receive and Timer1 interrupts enabled
	movlw	(1 << RCIE) | (1 << TMR1IE)
	movwf	PIE1

	movlw	12		;Delay approximately 2 ms at an instruction
	movwf	AP_BUF		; clock of 2 MHz until the PLL kicks in and the
PllWait	DELAY	110		; instruction clock gears up to 8 MHz
	decfsz	AP_BUF,F
	bra	PllWait

	movlw	B'11001000'	;On-change interrupt, peripheral interrupts (for
	movwf	INTCON		; UART) and interrupt subsystem on

	movlw	0x20		;Set up FSRs to point more or less permanently
	movwf	FSR0H		; to linear memory
	movwf	FSR1H

	clrf	AP_FLAG		;Initialize key globals
	clrf	AP_FSAP
	;fall through


;;; Mainline ;;;

AdbReset
	bcf	AP_FLAG,AP_RST	;Clear reset flag
	movlw	0x60 | ADBADDR	;(Re-)initialize key globals
	movwf	AM_R3H		; "
	movlw	1 << AM_STAT	; "
	movwf	AM_FLAG		; "
	clrf	AM_RXPU		; "
	clrf	AM_RXPO		; "
	movlw	0x80		; "
	movwf	AM_TXPU		; "
	movwf	AM_TXPO		; "
	;fall through

CheckFlow
	movf	AM_RXPO,W	;Queue length is push point minus pop point
	subwf	AM_RXPU,W	; "
	andlw	B'01111111'	; "
	sublw	96		;Set carry if Rx queue length <= 96
	movlb	2		;Assert CTS, i.e. set it to 0, if queue length
	btfsc	STATUS,C	; was <= 96, i.e. less than or equal to 3/4 full
	bcf	LATA,CTS_PIN	; "
	movf	AM_TXPO,W	;Again, queue length is push point minus pop
	subwf	AM_TXPU,W	; point
	;TODO in echo mode, check Rx queue instead
	btfss	WREG,4		;If Tx queue length >= 32, assert flow control
	bra	CheckF0		; and flag that the bit has changed if it has
	btfss	AM_FLAG,AM_FLOW	; "
	bsf	AM_FLAG,AM_FLOC	; "
	bsf	AM_FLAG,AM_FLOW	; "
CheckF0	;fall through

Main
	btfsc	AP_FLAG,AP_RST	;Branch to reset if a reset was received, else
	bra	AdbReset	; wait until a command was received
	btfss	AP_FLAG,AP_RXCI	; "
	bra	Main		; "
	bcf	AP_FLAG,AP_RXCI	;Clear the command flag
	bcf	AP_FLAG,AP_RXDI	;Clear other flags too from data activity that
	bcf	AP_FLAG,AP_DONE	; might have happened while waiting for a
	bcf	AP_FLAG,AP_COL	; command
	bcf	AP_FLAG,AP_TXI	; "
	bcf	AP_FLAG,AP_SRQ	;Not calling for service just yet
	movf	AP_BUF,W	;If the low four bits of the command are zero,
	andlw	B'00001111'	; this is a SendReset command and should be
	btfsc	STATUS,Z	; treated the same as a reset pulse
	bra	AdbReset	; "
	swapf	AM_R3H,W	;If the device being addressed does not match
	xorwf	AP_BUF,W	; the address of the modem, skip ahead to see if
	andlw	B'11110000'	; we need to send an SRQ
	btfss	STATUS,Z	; "
	bra	Main0		; "
	movf	AP_BUF,W	;Switch handler by the low four bits of the
	andlw	B'00001111'	; command
	brw			; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	Main		; "
	goto	AdbListen0	; "
	goto	AdbListen1	; "
	goto	AdbListen2	; "
	goto	AdbListen3	; "
	goto	AdbTalk0	; "
	goto	AdbTalk1	; "
	goto	AdbTalk2	; "
	goto	AdbTalk3	; "
Main0	btfss	AM_R3H,AM_SRQE	;If SRQs are disabled, we can't send them anyway
	bra	Main		; so loop around
	btfss	AM_FLAG,AM_STAT	;If we have a status to send or there's an 0x95
	btfsc	AM_FLAG,AM_0X95	; heading the queue, that's something to send,
	bsf	AP_FLAG,AP_SRQ	; so raise SRQ
	movf	AM_RXPO,W	;If the Rx queue is non-empty or the flow
	xorwf	AM_RXPU,W	; control status has changed, we need to send a
	btfsc	STATUS,Z	; data packet, so raise SRQ
	btfsc	AM_FLAG,AM_FLOC	; "
	bsf	AP_FLAG,AP_SRQ	; "
	bra	Main		;Return to wait for service

AdbListen0
	btfss	AM_FLAG,AM_MOD1	;If we're not in echo mode, skip ahead to write
	btfsc	AM_FLAG,AM_MOD0	; to Tx queue so data is written to UART instead
	bra	AdbLs02		; of Rx queue so data is echoed over ADB
	movf	AM_RXPO,W	;Queue length is push point minus pop point
	subwf	AM_RXPU,W	; "
	andlw	B'01111111'	; "
	sublw	120		;Clear carry if queue length > 120
	btfss	STATUS,C	;If queue length > 120, we don't have space for
	goto	CheckFlow	; this Listen, so don't listen
	movf	AM_RXPU,W	;Load the Rx pointer into FSR0 so we can write
	movwf	FSR0L		; to queue
	movlw	7		;Repeat this loop seven times
AdbLs00	call	AdbGetNext	;Wait until the next of eight bytes is received
	xorwf	AP_BUF,F	;Exchange the received byte into W and the count
	xorwf	AP_BUF,W	; into AP_BUF
	xorwf	AP_BUF,F	; "
	movwi	FSR0++		;Put the received byte into the queue, advance
	bcf	FSR0L,7		; and wrap the pointer
	movf	AP_BUF,W	;Restore the loop count to W and loop if there
	decfsz	WREG,W		; are bytes left to receive
	bra	AdbLs00		; "
	call	AdbGetNext	;Get the last of eight bytes
	movf	AP_BUF,W	;Put the received byte into the queue but don't
	movwf	INDF0		; advance the pointer
	andlw	B'11100000'	;If the eighth byte began with something other
	xorlw	B'10000000'	; than 0x80 or 0x90, it's a literal byte and the
	btfss	STATUS,Z	; payload was 8 full bytes so skip ahead; else
	bra	AdbLs01		; it was the number of bytes in the low nibble
	movf	AP_BUF,W	;Get the low nibble and advance the pointer by
	andlw	B'00000111'	; that many, wrapping it, and return
	addwf	AM_RXPU,F	; "
	bcf	AM_RXPU,7	; "
	goto	CheckFlow	; "
AdbLs01	movlw	8		;Advance the pointer by 8 and wrap it and return
	addwf	AM_RXPU,F	; "
	bcf	AM_RXPU,7	; "
	goto	CheckFlow	; "
AdbLs02	movf	AM_TXPO,W	;Queue length is push point minus pop point
	subwf	AM_TXPU,W	; "
	andlw	B'00111111'	; "
	sublw	56		;Clear carry if queue length > 56
	btfss	STATUS,C	;If queue length > 56, we don't have space for
	goto	CheckFlow	; this Listen, so don't listen
	movf	AM_TXPU,W	;Load the Rx pointer into FSR0 so we can write
	movwf	FSR0L		; to queue
	movlw	7		;Repeat this loop seven times
AdbLs03	call	AdbGetNext	;Wait until the next of eight bytes is received
	xorwf	AP_BUF,F	;Exchange the received byte into W and the count
	xorwf	AP_BUF,W	; into AP_BUF
	xorwf	AP_BUF,F	; "
	movwi	FSR0++		;Put the received byte into the queue, advance
	bcf	FSR0L,6		; and wrap the pointer
	movf	AP_BUF,W	;Restore the loop count to W and loop if there
	decfsz	WREG,W		; are bytes left to receive
	bra	AdbLs03		; "
	call	AdbGetNext	;Get the last of eight bytes
	movf	AP_BUF,W	;Put the received byte into the queue but don't
	movwf	INDF0		; advance the pointer
	andlw	B'11100000'	;If the eighth byte began with something other
	xorlw	B'10000000'	; than 0x80 or 0x90, it's a literal byte and the
	btfss	STATUS,Z	; payload was 8 full bytes so skip ahead; else
	bra	AdbLs04		; it was the number of bytes in the low nibble
	movf	AP_BUF,W	;Get the low nibble and advance the pointer by
	andlw	B'00000111'	; that many, wrapping it, and return
	addwf	AM_TXPU,F	; "
	bcf	AM_TXPU,6	; "
	goto	CheckFlow	; "
AdbLs04	movlw	8		;Advance the pointer by 8 and wrap it and return
	addwf	AM_TXPU,F	; "
	bcf	AM_TXPU,6	; "
	goto	CheckFlow	; "

AdbListen1
	call	AdbGetNext	;Wait for (and then ignore) first byte
	call	AdbGetNext	;Wait for second byte
	bcf	AM_FLAG,AM_DRVR	;Copy the driver-installed and modem-is-on flags
	btfsc	AP_BUF,0	; "
	bsf	AM_FLAG,AM_DRVR	; "
	bcf	AM_FLAG,AM_ISON	; "
	btfsc	AP_BUF,1	; "
	bsf	AM_FLAG,AM_ISON	; "
	goto	Main		;Return to main

AdbListen2
	goto	Main ;TODO support break bit (14)

AdbListen3
	call	AdbGetNext	;Wait for first byte
	movf	AP_BUF,W	;Save the first byte of the listen in FSR0 for
	movwf	FSR0L		; later use (as a temp var, no dereferencing)
	call	AdbGetNext	;Wait for second byte
	movf	AP_BUF,W	;If handler ID is 0x00, it means to change the
	btfsc	STATUS,Z	; device's address and SRQ enable bit
	bra	ADLsn31		; unconditionally
	addlw	2		;If handler ID is 0xFE, it means to change the
	btfsc	STATUS,Z	; device's address if a collision hasn't been
	bra	ADLsn30		; detected
	goto	Main		;Other than that, we don't change handler ID
ADLsn30	btfss	AM_R3H,AM_COL	;If a collision has not been detected, skip
	bra	ADLsn32		; ahead to change the address; if one has been
	bcf	AM_R3H,AM_COL	; detected, clear it and ignore this command
	goto	Main		; "
ADLsn31	bcf	AM_R3H,AM_SRQE	;Copy the state of the SRQ enable bit to the SRQ
	btfsc	FSR0L,AM_SRQE	; enable flag and to our copy of register 3
	bsf	AM_R3H,AM_SRQE	; "
ADLsn32	movlw	B'00001111'	;Accept the low four bits of the first received
	andwf	FSR0L,F		; byte as our new address and we're done
	movf	AM_R3H,W	; "
	andlw	B'11110000'	; "
	iorwf	FSR0L,W		; "
	movwf	AM_R3H		; "
	goto	Main		; "

AdbGetNext
	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command or
	btfsc	AP_FLAG,AP_RXCI	; the receive is done, return to main without
	bra	AL0GNx0		; processing input; we expect all eight bytes
	btfsc	AP_FLAG,AP_DONE	; and nothing less will do
	bra	AL0GNx0		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for a data byte
	bra	AdbGetNext	; "
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag and return
	return			; "
AL0GNx0	movlb	31		;Return to main instead of to caller
	movlw	high Main	; "
	movwf	TOSH		; "
	movlw	low Main	; "
	movwf	TOSL		; "
	return			; "

AdbTalk0
	btfsc	AM_FLAG,AM_STAT	;If we need to send a status, jump ahead to do
	bra	ATk0_00		; that
	movf	AM_RXPO,W	;If the Rx queue is non-empty, we have data to
	xorwf	AM_RXPU,W	; send, so skip ahead to send a data packet
	btfsc	STATUS,Z	; "
	btfsc	AM_FLAG,AM_0X95	; "
	bra	ATk0_02		; "
	btfss	AM_FLAG,AM_ISON	;If the modem is "on" or the flow control bit
	btfsc	AM_FLAG,AM_FLOC	; has changed, send a data packet even if it has
	bra	ATk0_02		; no data in it
	goto	Main		;Otherwise, send nothing
ATk0_00	bcf	AM_FLAG,AM_STAT	;Lower the status-send flag
	movlw	0xF0		;If we're not in echo mode, set the CD and OH
	btfss	AM_FLAG,AM_MOD1	; lights on, as well as the bits that seem to be
	btfsc	AM_FLAG,AM_MOD0	; set when a connection is established
	movlw	0xCC		; "
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	movlw	0x00		;I believe the second byte of the 0x88 status is
	btfsc	AM_FLAG,AM_DRVR	; the same as the second byte of Listen 1
	iorlw	0x01		; "
	btfsc	AM_FLAG,AM_ISON	; "
	iorlw	0x02		; "
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	clrf	AP_BUF		;Third byte of 0x88 status is zero
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	call	ATk0_01		;Stuff the appropriate byte for baud rate
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	movlw	0x0C		;I don't actually know what goes in the fifth
	movwf	AP_BUF		; byte, but I see 0x0C in there a lot, so...
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	movlw	0x88		;The real A300 transmits 0x88 for its last three
	movwf	AP_BUF		; bytes, so let's do the same
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitDone	;Wait for transmitter to be done
	goto	Main		;Return to main when it is
ATk0_01	movf	AM_FLAG,W	;Call into this lookup table for the baud rate
	andlw	B'00000011'	; indicator:
	brw			; "
	retlw	0x00		; Echo mode
	retlw	0x80		; 300 baud
	retlw	0x60		; 1200 baud
	retlw	0x30		; 2400 baud
ATk0_02	call	AdbRxPop	;Try to pop a byte from the queue; if we were
	btfsc	STATUS,Z	; unsuccessful, skip ahead to send seven dummy
	bra	ATk0_03		; bytes and a control byte
	movwf	AP_BUF		;Load the byte for transmission
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	call	AdbRxPop	;Try to pop a byte from the queue; if we were
	btfsc	STATUS,Z	; unsuccessful, skip ahead to send six dummy
	bra	ATk0_04		; bytes and a control byte
	movwf	AP_BUF		;Load the byte for transmission
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	call	AdbRxPop	;Try to pop a byte from the queue; if we were
	btfsc	STATUS,Z	; unsuccessful, skip ahead to send five dummy
	bra	ATk0_05		; bytes and a control byte
	movwf	AP_BUF		;Load the byte for transmission
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	call	AdbRxPop	;Try to pop a byte from the queue; if we were
	btfsc	STATUS,Z	; unsuccessful, skip ahead to send four dummy
	bra	ATk0_06		; bytes and a control byte
	movwf	AP_BUF		;Load the byte for transmission
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	call	AdbRxPop	;Try to pop a byte from the queue; if we were
	btfsc	STATUS,Z	; unsuccessful, skip ahead to send three dummy
	bra	ATk0_07		; bytes and a control byte
	movwf	AP_BUF		;Load the byte for transmission
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	call	AdbRxPop	;Try to pop a byte from the queue; if we were
	btfsc	STATUS,Z	; unsuccessful, skip ahead to send two dummy
	bra	ATk0_08		; bytes and a control byte
	movwf	AP_BUF		;Load the byte for transmission
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	call	AdbRxPop	;Try to pop a byte from the queue; if we were
	btfsc	STATUS,Z	; unsuccessful, skip ahead to send one dummy
	bra	ATk0_09		; byte and a control byte
	movwf	AP_BUF		;Load the byte for transmission
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	call	AdbRxPeek	;Peek at the byte on top of the queue; if it is
	addlw	-128		; between 0x80 and 0x9F, skip ahead to send a
	addlw	-32		; control byte instead of this byte since we
	movlw	0		; can't send this as an eighth byte; also, if
	btfsc	STATUS,C	; the flow control bit has changed, skip ahead
	btfsc	AM_FLAG,AM_FLOC	; to send a control byte
	bra	ATk0_10		; "
	call	AdbRxPop	;Try to pop a byte from the queue; if we were
	btfsc	STATUS,Z	; unsuccessful, skip ahead to send a control
	bra	ATk0_10		; byte
	movwf	AP_BUF		;Load the byte for transmission
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitDone	;Wait for transmitter to be done
	goto	Main		;Return to main
ATk0_03	addlw	1		;Increment the dummy byte count
	clrf	AP_BUF		;Send a dummy byte
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
ATk0_04	addlw	1		;Increment the dummy byte count
	clrf	AP_BUF		;Send a dummy byte
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
ATk0_05	addlw	1		;Increment the dummy byte count
	clrf	AP_BUF		;Send a dummy byte
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
ATk0_06	addlw	1		;Increment the dummy byte count
	clrf	AP_BUF		;Send a dummy byte
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
ATk0_07	addlw	1		;Increment the dummy byte count
	clrf	AP_BUF		;Send a dummy byte
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
ATk0_08	addlw	1		;Increment the dummy byte count
	clrf	AP_BUF		;Send a dummy byte
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
ATk0_09	addlw	1		;Increment the dummy byte count
	clrf	AP_BUF		;Send a dummy byte
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
ATk0_10	xorlw	0xFF		;Subtract num dummy bytes from 7 to get number
	addlw	0x88		; of data bytes and set MSB to get ctrl byte
	bcf	AM_FLAG,AM_FLOC	;Lower the flow-control-changed flag
	btfsc	AM_FLAG,AM_FLOW	;Reflect the flow control bit in the control
	iorlw	0x10		; byte and send it
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	; "
	call	AdbWaitDone	;Wait for transmitter to be done
	goto	Main		;Return to main

AdbTalk1
	movlw	0xF0		;If we're not in echo mode, set the CD and OH
	btfss	AM_FLAG,AM_MOD1	; lights on, as well as the bits that seem to be
	btfsc	AM_FLAG,AM_MOD0	; set when a connection is established
	movlw	0xCC		; "
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	movlw	0x00		;I believe the second byte of the 0x88 status is
	btfsc	AM_FLAG,AM_DRVR	; the same as the second byte of Listen 1
	iorlw	0x01		; "
	btfsc	AM_FLAG,AM_ISON	; "
	iorlw	0x02		; "
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	clrf	AP_BUF		;Third byte of 0x88 status is zero
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	call	ATk0_01		;Stuff the appropriate byte for baud rate
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitDone	;Wait for transmitter to be done
	goto	Main		;Return to main when it is

AdbTalk2
	movlw	0x14		;Firmware revision 1.4 (or so we pretend)
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	movlw	0xBC		;First byte of ID 12345678
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	movlw	0x61		;Second byte of ID 12345678
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	movlw	0x4E		;Third byte of ID 12345678
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	movlw	0x64		;I don't know what this means
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	movlw	0x3A		;Manufacture date 1991-02-10
	movwf	AP_BUF		; "
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitDone	;Wait for transmitter to be done
	goto	Main		;Return to main when it is

AdbTalk3
	movf	AM_R3H,W	;Load the high byte of register 3 for transmit,
	andlw	B'01110000'	; clearing the MSB (which we use as a collision
	movwf	AP_BUF		; flag) and the address
	movlb	0		;Get a pseudorandom four-bit number and put it
	movf	TMR1H,W		; into the low nibble of the buffer; this way
	xorwf	TMR1L,W		; we replace address (which the host already
	andlw	B'00001111'	; knows) with a random number, which helps with
	iorwf	AP_BUF,F	; collision detection
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitNext	;Wait for transmitter to be ready for next byte
	movlw	ADBHDLR		;Load the handler ID (which cannot be changed)
	movwf	AP_BUF		; for transmission
	bsf	AP_FLAG,AP_TXI	;Raise the flag to transmit the byte
	call	AdbWaitDone	;Wait for transmitter to be done
	goto	Main		;Return to main when it is

AdbWaitNext
	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	bra	AdbWNx1		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AdbWNx0		; "
	btfsc	AP_FLAG,AP_TXI	;Otherwise, wait for the transmitter to be ready
	bra	AdbWaitNext	; for the next byte and return when it is
	return			; "
AdbWNx0	bsf	AM_R3H,AM_COL	;We collided, so set the collision flag
AdbWNx1	movlb	31		;Return to main instead of to caller
	movlw	high Main	; "
	movwf	TOSH		; "
	movlw	low Main	; "
	movwf	TOSL		; "
	return			; "

AdbWaitDone
	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command,
	btfsc	AP_FLAG,AP_RXCI	; break off transmitting and return to main
	bra	AdbWNx1		; "
	btfsc	AP_FLAG,AP_COL	;If the flag is up for a collision, handle it
	bra	AdbWNx0		; "
	btfss	AP_FLAG,AP_DONE	;Otherwise, wait for the transmitter to be done
	bra	AdbWaitDone	; and return when it is
	return			; "

AdbRxPop
	btfsc	AM_FLAG,AM_0X95	;Special handling is required if there's a
	bra	ARxPop0		; doubled-up 0x95 in queue
	movf	AM_RXPO,W	;If queue is empty, set Z flag and return with
	movwf	FSR0L		; 0 in W
	xorwf	AM_RXPU,W	; "
	btfsc	STATUS,Z	; "
	retlw	0		; "
	incf	AM_RXPO,F	;Else, dereference, increment, and loop the
	bcf	AM_RXPO,7	; pointer
	movf	INDF0,W		; "
	xorlw	0x95		;If the byte we dereferenced was 0x95, set the
	btfsc	STATUS,Z	; flag so we double it up next time
	bsf	AM_FLAG,AM_0X95	; "
	xorlw	0x95		; "
	bcf	STATUS,Z	;Clear Z flag so caller doesn't think queue is
	return			; empty and return
ARxPop0	bcf	AM_FLAG,AM_0X95	;Clear the 0x95 flag since we're popping it
	bcf	STATUS,Z	;Clear Z flag so caller doesn't think queue is
	retlw	0x95		; empty and return the doubled-up 0x95

AdbRxPeek
	bcf	STATUS,Z	;If there's a doubled-up 0x95 in queue, return
	btfsc	AM_FLAG,AM_0X95	; that with Z flag clear
	retlw	0x95		; "
	movf	AM_RXPO,W	;If queue is empty, set Z flag and return with
	movwf	FSR0L		; 0 in W
	xorwf	AM_RXPU,W	; "
	btfsc	STATUS,Z	; "
	retlw	0		; "
	movf	INDF0,W		;Else, dereference the pointer and return its
	bcf	STATUS,Z	; value in W with Z flag always low (even if W
	return			; is 0)


;;; State Machines ;;;

AdbFsa	org	0xF00

AdbFsaIdle
	clrf	TMR0		;Reset timer
	movf	AP_DTMR,W	;If the down time was 194-206 ticks (800 us +/-
	addlw	-207		; 3%), this is an attention pulse, so prepare
	btfsc	STATUS,C	; the shift register to receive a command byte
	retlw	low AdbFsaIdle	; and transition to receive the first bit
	addlw	13		; "
	btfss	STATUS,C	; "
	retlw	low AdbFsaIdle	; "
	movlw	0x01		; "
	movwf	AP_SR		; "
	retlw	low AdbFsaCmdBit; "

AdbFsaCmdBit
	btfsc	AP_DTMR,7	;If either the down time or the up time is over
	retlw	low AdbFsaIdle	; 127 (508 us, ridiculous), throw up our hands
	btfsc	TMR0,7		; and wait for an attention pulse
	retlw	low AdbFsaIdle	; "
	movf	TMR0,W		;Sum the value of Timer0 (the up time) with the
	addwf	AP_DTMR,W	; down time, then divide by two to get the mid-
	lsrf	WREG,W		; point; subtract the up time so carry contains
	subwf	TMR0,W		; 1 if up time was greater than the midpoint (a
	rlf	AP_SR,F		; 1 bit) else 0, rotate bit into shift register
	clrf	TMR0		;Reset Timer0 for next time
	btfss	STATUS,C	;If we rotated a 0 out of shift register, then
	retlw	low AdbFsaCmdBit; there are more command bits to come
	movf	AP_SR,W		;Else, move the contents of the filled shift
	movwf	AP_BUF		; register into the buffer and set flag to say
	bsf	AP_FLAG,AP_RXCI	; that a command has been received
	movlw	-12		;Set a timer to expire and interrupt after 48us
	movwf	TMR0		; so that the user program has time to decide
	bsf	INTCON,TMR0IE	; whether or not to request service or transmit
	bsf	AP_FLAG,AP_RISE	;Set to catch rising edge that starts Tlt
	retlw	low AdbFsaSrq	;Set to enter the state handling service reqs

AdbFsaSrq
	btfsc	BSR,0		;If for some reason we're here because of an
	bra	AFSrq0		; edge, cancel our timer interrupt, stop
	bcf	INTCON,TMR0IE	; catching rising edges, reset timer, and bail
	bcf	AP_FLAG,AP_RISE	; out to waiting for an attention pulse
	clrf	TMR0		; "
	retlw	low AdbFsaIdle	; "
AFSrq0	btfss	AP_FLAG,AP_SRQ	;If the user didn't call for a service request,
	retlw	low AdbFsaTlt	; just wait for Tlt to begin
	bcf	TRISA,ADB_PIN	;If the user did call for a service request,
	movlw	-63		; pull the pin low and set a timer for 252 us
	movlb	0		; above the 48 us we already waited to release
	movwf	TMR0		; it
	bsf	INTCON,TMR0IE	; "
	retlw	low AdbFsaSrqEnd; "

AdbFsaSrqEnd
	btfss	BSR,0		;On the off chance we're here because edge, go
	retlw	low AdbFsaSrqEnd; around again until the timer expires
	bsf	TRISA,ADB_PIN	;Release the pin that we pulled low to request
	retlw	low AdbFsaTlt	; service and wait for Tlt (could be right now)

AdbFsaTlt
	bcf	AP_FLAG,AP_RISE	;No longer need to catch rising edges
	movlw	-65		;Shorten the timeout period to 260 us, which is
	movwf	TMR0		; slightly longer than Tlt is expected to be
	bsf	INTCON,TMR0IE	;Timer interrupts whether we transmit or not
	btfss	AP_FLAG,AP_TXI	;If the user doesn't wish to transmit, just
	retlw	low AdbFsaTltEnd; wait for data to start
	movf	TMR1H,W		;Get a pseudorandom between 0 and 15, adjust it
	xorwf	TMR1L,W		; to between 199 and 214; that will make Timer0
	andlw	B'00001111'	; overflow in between 168us and 228us, which is
	addlw	-57		; close enough to the specced range of 160us to
	movwf	TMR0		; 240us to wait before transmitting
	movlw	B'11000000'	;Set the shift register so it outputs a 1 start
	movwf	AP_SR		; bit and then loads data from the buffer
	retlw	low AdbFsaTxBitD;Bring us to the transmission start state

AdbFsaTxBitD
	btfsc	BSR,0		;If we're here because of a timer interrupt,
	bra	AFTxBD0		; as we hope, skip ahead
	bsf	AP_FLAG,AP_COL	;If not, set the collision flag, clear the
	bcf	AP_FLAG,AP_TXI	; transmit flag, cancel the timer, and go back
	clrf	TMR0		; to waiting for an attention pulse
	bcf	INTCON,TMR0IE	; "
	retlw	low AdbFsaIdle	; "
AFTxBD0	bcf	TRISA,ADB_PIN	;Pull the pin low
	lslf	AP_SR,F		;Shift the next bit to send into carry bit
	btfss	STATUS,Z	;If we shifted the placeholder out of the shift
	bra	AFTxBD1		; register, continue, else skip ahead
	btfss	AP_FLAG,AP_TXI	;If there's no new byte ready to load, clear
	bcf	STATUS,C	; carry so we send a zero as our last bit
	btfss	AP_FLAG,AP_TXI	;If there's a new byte ready to load, load it,
	bra	AFTxBD1		; shift its MSB out and a 1 placeholder into
	rlf	AP_BUF,W	; its LSB and clear the transmit flag; else
	movwf	AP_SR		; leave the shift register all zeroes as a
	bcf	AP_FLAG,AP_TXI	; signal to the up phase state that we're done
AFTxBD1	movlw	-8		;Set a timer to interrupt in 8 cycles (32us) if
	movlb	0		; sending a 1 bit, double that to 16 cycles
	btfss	STATUS,C	; (64us) if we're sending a 0 bit, and also
	lslf	WREG,W		; save this value for use by the up phase state
	movwf	TMR0		; "
	movwf	AP_DTMR		; "
	bsf	INTCON,TMR0IE	; "
	retlw	low AdbFsaTxBitU; "

AdbFsaTxBitU
	btfss	BSR,0		;If we're here because of the falling edge we
	retlw	low AdbFsaTxBitU; just triggered, ignore and return posthaste
	bsf	TRISA,ADB_PIN	;Release the pin
	DELAY	2		;Wait for it to actually go high
	movlb	0		;If the pin is still low, we've collided; set
	btfsc	PORTA,ADB_PIN	; the collision flag, clear the transmit flag,
	bra	AFTxBU0		; and go back to waiting for an attention pulse
	bsf	AP_FLAG,AP_COL	; "
	bcf	AP_FLAG,AP_TXI	; "
	retlw	low AdbFsaIdle	; "
AFTxBU0	movf	AP_SR,W		;If the down phase let the shift register stay
	btfsc	STATUS,Z	; at zero, the bit we just transmitted is the
	bsf	AP_FLAG,AP_DONE	; stop bit and transmission is over, so set the
	btfsc	STATUS,Z	; done flag and return to waiting for an
	retlw	low AdbFsaIdle	; attention pulse
	movlw	B'00001000'	;Whatever delay (8 or 16) we did during the
	xorwf	AP_DTMR,W	; down phase, set a timer to do the other one
	movwf	TMR0		; "
	bsf	INTCON,TMR0IE	; "
	movlb	7		;Reverse the IOC interrupt and clear the flag
	bcf	IOCAP,ADB_PIN	; set by releasing the pin so the timer we just
	bsf	IOCAN,ADB_PIN	; set doesn't immediately get reset
	bcf	IOCAF,ADB_PIN	; "
	retlw	low AdbFsaTxBitD;Timer will take us to the down phase again

AdbFsaTltEnd
	btfsc	BSR,0		;If we're here because of a timer interrupt, the
	bra	AFTltE0		; transmission never started, so skip ahead
	clrf	TMR0		;This state is the end of Tlt and the start of
	retlw	low AdbFsaRxStrt; host or other device transmitting data
AFTltE0	bsf	AP_FLAG,AP_DONE	;Set the done flag because the data payload is
	retlw	low AdbFsaIdle	; effectively done and return to idle

AdbFsaRxStrt
	movlw	0x01		;Start bit should be 1, but who cares, just set
	movwf	AP_SR		; up the shift register to receive the first
	clrf	TMR0		; data bit
	bsf	AP_FLAG,AP_RISE	;Catch rising edges to set receive timeout timer
	retlw	low AdbFsaRxBitD; "

AdbFsaRxBitD
	movlw	-31		;Set the timeout timer for 124 us, up time on a
	movwf	TMR0		; received bit should never be this long, and
	bsf	INTCON,TMR0IE	; wait for a falling edge or a timeout
	retlw	low AdbFsaRxBitU; "

AdbFsaRxBitU
	btfss	BSR,0		;If we got here because of a timer overflow, the
	bra	AFRxBU0		; data payload must be done, so disable catching
	bcf	AP_FLAG,AP_RISE	; rising edges, set the done flag, and return to
	bsf	AP_FLAG,AP_DONE	; idle
	retlw	low AdbFsaIdle	; "
AFRxBU0	movlw	31		;Compensate for us setting Timer0 to time out
	addwf	TMR0,F		; early
	btfsc	AP_DTMR,7	;If the down time is over 127 (508 us,
	bcf	AP_FLAG,AP_RISE	; ridiculous), throw up our hands and wait for
	btfsc	AP_DTMR,7	; an attention pulse
	retlw	low AdbFsaIdle	; "
	movf	TMR0,W		;Sum the value of Timer0 (the up time) with the
	addwf	AP_DTMR,W	; down time, then divide by two to get the mid-
	lsrf	WREG,W		; point; subtract the up time so carry contains
	subwf	TMR0,W		; 1 if up time was greater than the midpoint (a
	rlf	AP_SR,F		; 1 bit) else 0, rotate bit into shift register
	clrf	TMR0		;Reset Timer0 for next time
	btfss	STATUS,C	;If we rotated a 0 out of shift register, then
	retlw	low AdbFsaRxBitD; there are more data bits to come
	movf	AP_SR,W		;Else, move the contents of the filled shift
	movwf	AP_BUF		; register into the buffer and set flag to say
	bsf	AP_FLAG,AP_RXDI	; that a data byte has been received
	movlw	0x01		;Set up the shift register to receive the next
	movwf	AP_SR		; bit and wait for it
	retlw	low AdbFsaRxBitD; "


;;; End of Program ;;;
	end
