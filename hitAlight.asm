.include "/usr/share/avra/m16def.inc"

.def lastCandidatesB = r10
.def lastCandidatesA = r11
.def switchedOnB = r12
.def switchedOnA = r13
.def switchStatusB = r14
.def switchStatusA = r15
.def temp1 = r16
.def temp2 = r17
.def temp3 = r18
.def counter = r19
.def statusRegister = r20
.def LEDsA = r21
.def LEDsB = r22
.def scoreA = r23
.def scoreB = r24

;Constants
.equ TimerHitBit=1
.equ SwitchBit=2
.equ gameDuration=58594 ;this many 1024/clock seconds will pass until the game ends 58594 (30s at 2MHZ)
.equ countdownStep=1953 ;this many 1024/clock seconds will pass until the countdown is decreased by one (1s @ 2 MhY)
.equ blinkDuration=980 ;this many 1024/clock seconds will be half the frequency of the "winner blinking"
.equ blinkRepetition=12 ;this many Times will the winner's LEDs blink, before all start twinkeling again
.equ twinkleSpeed=200 ;After this many 1024/clock seconds the next twinkle "frame" will be created and shown
.equ switchDebounceDelay=40
;---------------------------------------------
;This Mem-area stores the Timers for switches
;---------------------------------------------
.equ LEDsAMem=SRAM_START
.equ DisplayMem=LEDsAMem
.equ LEDsBMem=LEDsAMem+1
.equ ScoreMem=LEDsBMem+1

.cseg
.org 0x000 
	rjmp RESET ; Reset Handler
;.org $002 rjmp EXT_INT0 ; IRQ0 Handler
;.org $004 rjmp EXT_INT1 ; IRQ1 Handler
;.org $006 rjmp TIM2_COMP ; Timer2 Compare Handler
;.org 0x008 rjmp TIM2_OVF ; Timer2 Overflow Handler
;.org $00A rjmp TIM1_CAPT ; Timer1 Capture Handler
.org 0x00C 
	rjmp TIM1_COMPA ; Timer1 CompareA Handler
;.org 0x00E rjmp TIM1_COMPB ; Timer1 CompareB Handler
;.org $010 rjmp TIM1_OVF ; Timer1 Overflow Handler
;.org 0x012 rjmp TIM0_OVF ; Timer0 Overflow Handler
;.org $014 rjmp SPI_STC ; SPI Transfer Complete Handler
;.org $016 rjmp USART_RXC ; USART RX Complete Handler
;.org $018 rjmp USART_UDRE ; UDR Empty Handler
;.org $01A rjmp USART_TXC ; USART TX Complete Handler
;.org $01C rjmp ADC ; ADC Conversion Complete Handler
;.org $01E rjmp EE_RDY ; EEPROM Ready Handler
;.org $020 rjmp ANA_COMP ; Analog Comparator Handler
;.org 0x022 rjmp TWSI ; Two-wire Serial Interface Handler
;.org $024 rjmp EXT_INT2 ; IRQ2 Handler
.org 0x026 
	rjmp TIM0_COMP ; Timer0 Compare Handler
;.org $028 rjmp SPM_RDY ; Store Program Memory Ready Handler

RESET:
;---------------------------------------------
;Init Stack
;---------------------------------------------
LDI   temp1,HIGH(RAMEND) 
OUT   SPH,temp1
LDI   temp1,LOW(RAMEND)
OUT   SPL,temp1


clr statusRegister
clr switchStatusA
clr switchStatusB
clr scoreA
clr scoreB
;---------------------------------------------
;Init IO
;Set Port A und C as Input
;---------------------------------------------
clr temp1
out DDRA,temp1
out DDRC,temp1
ser temp1
out PortA, temp1
out PortC, temp1

;----------------------------------------------
;Init SPI Pins
;----------------------------------------------
ldi temp1, (1<<PB4|1<<PB5|1<<PB7)
out DDRB,temp1

;----------------------------------------------
;Init SPI 
;----------------------------------------------
ldi temp1, (1<<SPE | 1<<MSTR | 1<<DORD)
out SPCR, temp1
ldi temp1, ( 1<<SPI2X)
out SPSR, temp1

;----------------------------------------------
;Init Timers
;----------------------------------------------
ldi temp1, (1 <<CS12 | 1<<CS10 | 1<<WGM12)	;16Bit Timer 1 runs at clock/1024
out TCCR1B,temp1

ldi temp1, (1<<CS00 | 1<<CS02 | 1<<WGM01 )	;8Bit Timer runs at clock 
out TCCR0, temp1

ldi temp1, (1<<CS20)				;8Bit Timer2 runs at clock as source for random bits
out TCCR2, temp1

ldi temp1, (1<<OCIE1A| 1<<OCIE0)
out TIMSK, temp1

ldi temp1, switchDebounceDelay
out OCR0, temp1

;----------------------------------------------
;Enable Interrupts
;----------------------------------------------
sei

;---------------------------------------------------
;PreGaming mode
;twinkle, until a button is pressed
;Keep score from previous game untouched
;---------------------------------------------------
pregameMode:

clr switchedOnA
clr switchedOnB

ldi temp1,HIGH(twinkleSpeed)
out OCR1AH,temp1
ldi temp1,LOW(twinkleSpeed)
out OCR1AL,temp1

pregameLoop:
	sbrs statusRegister,TimerHitBit
	rjmp pregameLoopNoNewLights
	rcall doNewLightsA				;If the Timer hits, create a new light pattern
	rcall doNewLightsB				;If the Timer hits, create a new light pattern
pregameLoopNoNewLights:
	cbr statusRegister, (1<<TimerHitBit)		
	rcall output					;Write the new pattern to the LEDs
	rcall input					;Read the switches
	or switchedOnA, switchedOnB
	tst switchedOnA
breq pregameLoop					;Start the game if anything was pressed

;---------------------------------------------------
;Countdown mode
;Keep Twinkeling 
;Count the scorediplay from 3 to 0
;---------------------------------------------------
countdownMode:

ldi temp1,HIGH(countdownStep)
out OCR1AH,temp1
ldi temp1,LOW(countdownStep)
out OCR1AL,temp1

clr temp1		;Reset Timer
out TCNT1H, temp1
out TCNT1L, temp1

ldi ScoreA,3
ldi ScoreB,3

clr LEDsA
clr LEDsB

rcall output

countdown_loop:

	sbrs statusRegister,TimerHitBit		;Decrease the Scoredisplay until it reaches 0
		rjmp countdownNoCount
		dec ScoreA
		dec ScoreB
		cbr statusRegister, (1<<TimerHitBit)
		rcall output
	countdownNoCount:

	tst ScoreB
brne countdown_loop


;---------------------------------------------------
;Gaming mode
;---------------------------------------------------
rcall doNewLightsA				;Create new light pattern as start for the game
rcall doNewLightsB				

clr scoreA					;clear Score values
clr scoreB

rcall output					;Output the Score and lights

clr temp1					;Reset Timer
out TCNT1H, temp1
out TCNT1L, temp1

ldi temp1,HIGH(gameDuration)			;Set Timermatch to 30s
out OCR1AH,temp1
ldi temp1,LOW(gameDuration)
out OCR1AL,temp1

mainLoop:
	sbrs statusRegister,SwitchBit
	rjmp mainLoopDoNothing
	rcall input
	rcall calcscore

	tst switchedOnA
	breq mainLoopNothingHappenedA
	rcall doNewLightsA				;Create new light pattern as start for the game
mainLoopNothingHappenedA:

	tst switchedOnB
	breq mainLoopNothingHappenedB
	rcall doNewLightsB				
mainLoopNothingHappenedB:

	rcall output
	cbr statusRegister, (1<<SwitchBit)
mainLoopDoNothing:
	sbrs statusRegister,TimerHitBit
rjmp mainLoop


cbr statusRegister, (1<<TimerHitBit)
;---------------------------------------------------
;Find the winner and make his lights blink for some time
;---------------------------------------------------
clr temp1		;Reset Timer
out TCNT1H, temp1
out TCNT1L, temp1

ldi temp1,HIGH(blinkDuration)
out OCR1AH,temp1
ldi temp1,LOW(blinkDuration)
out OCR1AL,temp1

clr LEDsA
clr LEDsB

ldi counter, blinkRepetition

endloop:
	sbrs statusRegister,TimerHitBit
rjmp endloop

	cp ScoreA, ScoreB
	breq endloopDraw
	brlo endloopBWins
	;A wins
	com LEDsA
	rjmp endloopEnd
	;B wins
endloopBWins:
	com LEDsB
	rjmp endloopEnd
	
	;Draw
endloopDraw:
	com LEDsA
	com LEDsB
	rjmp endloopEnd
	endloopEnd:

	push counter
	rcall output
	pop counter

	cbr statusRegister,(1<<TimerHitBit)

	dec counter
brne endloop

rjmp pregameMode		;start everything from start

;---------------------------------------------------
;Make sure the buttons are debounced
;---------------------------------------------------
input:
	in temp1, PinA				;read Data
	com temp1				;invert, because if the button is pressed it pulls to low
	and switchStatusA, temp1		;reset all unpressed switches
	eor temp1, switchstatusA		;check what button is pressed but not marked as such
	and lastCandidatesA, temp1		;check which new button was already pressed at the last check
	mov switchedOnA, lastCandidatesA	;set the switched on marks for score evaluation
	or switchStatusA, lastCandidatesA	;Mark button as pressed, if it was pressed the last and this time this code was run
	mov lastCandidatesA, temp1		;save the candidates for next check

	in temp1, PinC				;read Data
	com temp1				;invert, because if the button is pressed it pulls to low
	and switchStatusB, temp1		;reset all unpressed switches 
	eor temp1, switchstatusB		;check what button is pressed but not marked as such
	and lastCandidatesB, temp1		;check which new button was already pressed at the last check
	mov switchedOnB, lastCandidatesB	;set the switched on marks for score evaluation
	or switchStatusB, lastCandidatesB	;Mark button as pressed, if it was pressed the last and this time this code was run
	mov lastCandidatesB, temp1		;save the candidates for next check
ret

calcscore:
	mov temp1, LEDsA		;generate Bitmask that marks active LEDs which have been switched
	and temp1, switchedOnA
	rcall hammingWeight
	add ScoreA, temp1

	mov temp1, LEDsA		;generate Bitmask that marks incative LEDs which have been switched
	com temp1
	and temp1, switchedOnA
	rcall hammingWeight
	sub ScoreA, temp1
	

	;Do the same for B

	mov temp1, LEDsB		;generate Bitmask, that scores
	and temp1, switchedOnB
	rcall hammingWeight
	add ScoreB, temp1

	mov temp1, LEDsB		;generate negative Scores
	com temp1
	and temp1, switchedOnB
	rcall hammingWeight
	sub ScoreB, temp1
ret

hammingWeight:
	mov temp2, temp1
	lsr temp2
	andi temp1, 0b01010101
	andi temp2, 0b01010101
	add temp1, temp2
	mov temp2, temp1
	lsr temp2
	lsr temp2
	andi temp1, 0b00110011
	andi temp2, 0b00110011
	add temp1, temp2
	mov temp2, temp1
	swap temp2		;instead of 4 lsr 
	andi temp1, 0b00001111
	andi temp2, 0b00001111
	add temp1, temp2
ret

output:
	ldi YH,HIGH(LEDsAMem)		;Store LEDA Byte in Mem
	ldi YL,LOW(LEDsAMem)
	st Y+, LEDsA
	st Y+, LEDsB  			;Store LEDB Byte in Mem

	mov temp1, ScoreA		;Convert the Score to Digits and write them to mem
	clr counter
	outputCalcDigitsA:
		inc counter
		subi temp1,10
	brpl outputCalcDigitsA
		dec counter
		subi temp1, -10

	rcall convertToDigit	
	st Y+,temp3
	mov temp1,counter
	rcall convertToDigit	
	st Y+,temp3


	mov temp1, ScoreB
	clr counter
	outputCalcDigitsB:
		inc counter
		subi temp1,10
	brpl outputCalcDigitsB
		dec counter
		subi temp1, -10

	rcall convertToDigit	
	st Y+,temp3
	mov temp1,counter
	rcall convertToDigit	
	st Y+,temp3

	ldi YH, HIGH(displayMem)	;Push the Bytes in the Memory to SPI
	ldi YL, LOW(displayMem)
	
	ldi counter, 6

	outputLoop:
		ld temp1,Y+
		out SPDR, temp1
		outputWait:
		sbis SPSR,SPIF
		rjmp outputWait
		dec counter
	brne outputLoop
	sbi PortB, PB4			;Flush Shift registers
	cbi PortB, PB4

ret

convertToDigit:
	ldi ZH, HIGH(digits*2)
	ldi ZL, LOW(digits*2)

	clr temp2

	subi temp1,10
	brpl convertToDigitOverHundred
	subi temp1, -10
	rjmp convertToDigitLoadValue
	
convertToDigitOverHundred:
	ldi temp2, 0b00010000
	
convertToDigitLoadValue:
	add ZL, temp1
	brcc convertToDigitNoCarry
		inc ZH
	convertToDigitNoCarry:
	lpm temp3, Z
	eor temp3, temp2
ret

digits:
.db ~0b11101101, ~0b00000101
.db ~0b10101011, ~0b00101111
.db ~0b01000111, ~0b01101110
.db ~0b11101110, ~0b00100101
.db ~0b11101111, ~0b01101111


TIM1_COMPA:
	push temp1
	in temp1, SREG
	push temp1
	sbr statusRegister, (1<<TimerHitBit)
	pop temp1
	out SREG, temp1
	pop temp1
reti

TIM0_COMP:
	push temp1
	in temp1, SREG
	push temp1
	sbr statusRegister, (1<<SwitchBit)
	pop temp1
	out SREG, temp1
	pop temp1
reti


doNewLightsA:
	rcall doNewLightPattern			;get a new random pattern into temp1

	mov LEDsA, temp1
	mov temp2, switchedOnA			;disable the LEDs, whose switch was pushed
	com temp2
	and LEDsA, temp2
ret


doNewLightsB:
	rcall doNewLightPattern			;get a new random pattern into temp1

	mov LEDsB, temp1
	mov temp2, switchedOnB			;disable the LEDs, whose switch was pushed
	com temp2
	and LEDsB, temp2
ret

doNewLightPattern:
	ldi ZH, HIGH(bytesWith4BitsOn*2)	;Init Pointer to Dataarea where patterns are stored
	ldi ZL, LOW(bytesWith4BitsOn*2)

	in temp1, TCNT2				;Get value of Timer 0
	andi temp1, 0b00111111			;Reduce it to a max value of 31
	in temp2, TCNT1L			;Get lower value of Timer 1
	andi temp2, 0b00000110			;Reduce it to a max value of 3	
	lsr temp2
	add temp1, temp2			;Add these to have a pseudo random number from 0 to 34
	add ZL, temp1				;And then add this number to the pointer to get a random pattern 
	brcc doNewLightsNoOverflow
		inc ZH
	doNewLightsNoOverflow:

	lpm temp1, Z				;Load the pattern

	in temp2, TCNT1L			;Depending on the LSB of timer 1 invert the pattern or do not
	andi temp2, 0b00000001
	breq doNewLightsNoInvert
	com temp1
doNewLightsNoInvert:
ret

bytesWith4BitsOn:	;35 Bytes with 4 Bits on and 1 with 5.  Invert for another 35
.db 0b11110000, 0b11101000
.db 0b11100100, 0b11100010
.db 0b11100001, 0b11011000
.db 0b11010100, 0b11010010
.db 0b11010001, 0b11001100
.db 0b11001010, 0b11001001
.db 0b11000110, 0b11000101
.db 0b11000011, 0b10111000
.db 0b10110100, 0b10110010
.db 0b10110001, 0b10101100
.db 0b10101010, 0b10101001
.db 0b10100110, 0b10100101
.db 0b10100011, 0b10011100
.db 0b10011010, 0b10011001
.db 0b10010110, 0b10010101
.db 0b10010011, 0b10001110
.db 0b10001101, 0b10001011
.db 0b10000111, 0b11001010
