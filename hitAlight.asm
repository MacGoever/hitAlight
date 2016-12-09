;.include "c:/m16def.inc"
.include "/usr/share/avra/m16def.inc"

.def switchedOnB = r11
.def switchedOnA = r12
.def switchStatusB = r13
.def switchStatusA = r14
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
.equ NumberOfStableSwitchReads=10
.equ InputBit=0
.equ CalcScoreBit=1
.equ OutputBit=2
.equ LightsBit=3
.equ TimerHitBit=4
.equ switchCounterStepLength=5 ;this many 1024/clock seconds will pass between switchchecks
.equ gameDuration=58594 ;this many 1024/clock seconds will pass until the game ends 58594 (30s at 2MHZ)
.equ countdownStep=1953 ;this many 1024/clock seconds will pass until the countdown is decreased by one (1s @ 2 MhY)
.equ blinkDuration=980 ;this many 1024/clock seconds will be half the frequency of the "winner blinking"
.equ blinkRepetition=12 ;this many Times will the winner's LEDs blink, before all start twinkeling again
.equ LEDOffTimeout=15 ;After switched off by a button, the LED will stay off this many 256*1204/clock seconds
.equ LEDOnTimeout=10 ;After switched on, the LED will stay off this many 256*1204/clock seconds
;---------------------------------------------
;This Mem-area stores the Timers for switches
;---------------------------------------------
.equ switchTimerMemA=SRAM_START
.equ switchTimerMemB=switchTimerMemA+8
.equ LEDsAMem=switchTimerMemB+8
.equ DisplayMem=LEDsAMem
.equ LEDsBMem=LEDsAMem+1
.equ ScoreMem=LEDsBMem+1
.equ LEDTimerMemA=ScoreMem+8
.equ LEDTimerMemB=LEDTimerMemA+8

.cseg
.org 0x000 
	rjmp RESET ; Reset Handler
;.org $002 rjmp EXT_INT0 ; IRQ0 Handler
;.org $004 rjmp EXT_INT1 ; IRQ1 Handler
;.org $006 rjmp TIM2_COMP ; Timer2 Compare Handler
.org 0x008 
	rjmp TIM2_OVF ; Timer2 Overflow Handler
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
.org $026 
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
ldi temp1, (1 << wgm01 | 1<<CS02 | 1<<CS00)
out TCCR0, temp1
ldi temp1, switchCounterStepLength
out OCR0,temp1

ldi temp1, (1 <<CS12 | 1<<CS10 | 1<<WGM12)
out TCCR1B,temp1

ldi temp1, (1<<CS22 | 1<<CS21 | 1<<CS20)
out TCCR2, temp1

ldi temp1, (1<<OCIE0|1<<OCIE1A| 1<<TOIE2)
out TIMSK, temp1


;----------------------------------------------
;Enable Interrupts
;----------------------------------------------
sei

;---------------------------------------------------
;PreGaming mode
;twinkle, until a button is pressed
;Keep Score untouched
;---------------------------------------------------
pregameMode:

clr switchedOnA
clr switchedOnB

ser LEDsA
ser LEDsB

rcall output

pregame_loop:
	sbrc statusRegister,LightsBit
	rcall PreGameTwinkling
	rcall input
	or switchedOnA, switchedOnB
	tst switchedOnA
breq pregame_loop

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
		mov ScoreB,ScoreA
		cbr statusRegister, (1<<TimerHitBit)
		rcall output
	countdownNoCount:

	tst ScoreB
brne countdown_loop


;---------------------------------------------------
;Gaming mode
;---------------------------------------------------
ser LEDsA					;Set All LEDs
ser LEDsB
ldi counter, 8					;And set corresponding Mem Timers
ldi YH, HIGH(LEDTimerMemA)
ldi YL, LOW(LEDTimerMemA)
ldi temp1, LEDOnTimeout

initMemALoop:
	st Y+, temp1
	dec counter
brne initMemALoop

ldi counter, 8					;And set corresponding Mem Timers
ldi YH, HIGH(LEDTimerMemB)
ldi YL, LOW(LEDTimerMemB)
ldi temp1, LEDOnTimeout

initMemBLoop:
	st Y+, temp1
	dec counter
brne initMemBLoop

clr scoreA					;clear Score values
clr scoreB

rcall output					;Output the Score and lights

clr temp1		;Reset Timer
out TCNT1H, temp1
out TCNT1L, temp1

ldi temp1,HIGH(gameDuration)	;Set Timermatch to 30s
out OCR1AH,temp1
ldi temp1,LOW(gameDuration)
out OCR1AL,temp1

main_loop:
	sbrc statusRegister, InputBit
	rcall input
	sbrc statusRegister, CalcScoreBit
	rcall calcscore
	sbrc statusRegister,OutputBit
	rcall output
	sbrc statusRegister,LightsBit
	rcall lights
	sbrs statusRegister,TimerHitBit
rjmp main_loop


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
;Do some twinkling
;---------------------------------------------------
PreGameTwinkling:
	cbr statusRegister,(1<<LightsBit)
	com LEDsA
	com LEDsB
	rcall output
ret
;---------------------------------------------------
;Make sure the buttons are debounced
;---------------------------------------------------
input:
	in temp1, PinA				;read Data
	com temp1				;invert, because if the button is pressed it pulls to low
	eor temp1, switchstatusA		;check what button has changed
	ldi YH,HIGH(switchTimerMemA)
	ldi YL,LOW(switchTimerMemA)
	rcall checkinput			;alter the timers
	eor switchstatusA, temp3		;if a switc is stable alter the global status
	mov switchedOnA, temp3			
	and switchedOnA, switchStatusA		;check which switch has changed it's status to on

;do the same for the other port

	in temp1, PinC				;read Data
	com temp1				;invert, because if the button is pressed it pulls to low
	eor temp1, switchstatusB		;check what button has changed
	ldi YH,HIGH(switchTimerMemB)
	ldi YL,LOW(switchTimerMemB)
	rcall checkinput			;alter the timers
	eor switchstatusB, temp3		;if a switc is stable alter the global status
	mov switchedOnB, temp3			
	and switchedOnB, switchStatusB		;check which switch has changed it's status to on

	cbr statusRegister, (1<<InputBit)	;Wait for the timer to recheck here
ret

checkinput:

	clr temp3
	
	ldi counter,8
	
	ld temp2,Y
	
	checkInputLoop:
	lsl temp1
	brcc checkinputNoSwitchChange
	tst temp2
	brne checkinputLoopNotFirst
	ldi temp2,NumberOfStableSwitchReads
	clc 
	rjmp checkinputLoopDone
	
	checkinputLoopNotFirst:
	dec temp2
	clc
	brne checkinputLoopDone
	sec
	rjmp checkinputLoopDone

	checkinputNoSwitchChange:
	clr temp2
	clc 

	checkinputLoopDone:

	rol temp3
	st Y+,temp2
	ld temp2,Y

	dec counter
	brne checkInputLoop

	tst temp3
	breq checkinputNoStatusChange
	sbr statusRegister, (1<<CalcScoreBit)
	checkinputNoStatusChange:
ret


calcscore:
	mov temp1, LEDsA		;generate Bitmask that marks active LEDs which have been switched
	and temp1, switchedOnA
	mov temp2, LEDsA		;generate Bitmask that marks incative LEDs which have been switched
	com temp2
	and temp2, switchedOnA

	ldi counter, 8
	calcScoreALoop:
		lsl temp1
		brcc calcScoreANoPoint
			inc scoreA
		calcScoreANoPoint:
		lsl temp2
		brcc calcScoreANoFail
			dec scoreA
			brpl calcScoreANoFail
			clr scoreA
		calcScoreANoFail:
	dec counter
	brne calcScoreALoop
	
	and switchedOnA, LEDsA		;clear the Lamps that have been switched
	eor LEDsA, switchedOnA
					;set Timers for LEDs that have been switched off, to keep them of for a defined time
	ldi counter, 8
	ldi YH, HIGH(LEDTimerMemA)
	ldi YL, LOW(LEDTimerMemA)
	calcScoreATimerLoop:
		ld temp1, Y
		lsl switchedOnA
		brcc calcScoreATimerNoSet
			ldi temp1, LEDOffTimeout
		calcScoreATimerNoSet:
		st Y+, temp1
	dec counter
	brne calcScoreATimerLoop

	clr switchedOnA			;clear this, because it was counted

	;Do the same for B

	mov temp1, LEDsB		;generate Bitmask, that scores
	and temp1, switchedOnB
	mov temp2, LEDsB		;generate negative Scores
	com temp2
	and temp2, switchedOnB

	ldi counter, 8
	calcScoreBLoop:
		lsl temp1
		brcc calcScoreBNoPoint
			inc scoreB
		calcScoreBNoPoint:
		lsl temp2
		brcc calcScoreBNoFail
			dec scoreB
			brpl calcScoreBNoFail
			clr scoreB
		calcScoreBNoFail:

	dec counter
	brne calcScoreBLoop
	
	and switchedOnB, LEDsB		;clear the Lamps that have been switched
	eor LEDsB, switchedOnB
					;set Timers for LEDs that have been switched off, to keep them of for a defined time
	ldi counter, 8
	ldi YH, HIGH(LEDTimerMemB)
	ldi YL, LOW(LEDTimerMemB)
	calcScoreBTimerLoop:
		ld temp1, Y
		lsl switchedOnB
		brcc calcScoreBTimerNoSet
			ldi temp1, LEDOffTimeout
		calcScoreBTimerNoSet:
		st Y+, temp1
	dec counter
	brne calcScoreBtimerLoop

	clr switchedOnB
	
	sbr statusRegister, (1<<OutputBit)
	cbr statusRegister, (1<<calcScoreBit)
ret

output:
	ldi YH,HIGH(LEDsAMem)		;Store LEDA Byte in Mem
	ldi YL,LOW(LEDsAMem)
	st Y, LEDsA

	ldi YH,HIGH(LEDsBMem)		;Store LEDB Byte in Mem
	ldi YL,LOW(LEDsBMem)
	st Y, LEDsB

	ldi YH, HIGH(ScoreMem)
	ldi YL, LOW(ScoreMem)

	mov temp1, ScoreA		;Convert the Score to Digits and write them to mem
	clr temp2
	outputCalcDigitsA:
		inc temp2
		subi temp1,10
	brpl outputCalcDigitsA
		dec temp2
		subi temp1, -10

	rcall convertToDigit	
	st Y+,temp3
	mov temp1,temp2
	rcall convertToDigit	
	st Y+,temp3


	mov temp1, ScoreB
	clr temp2
	outputCalcDigitsB:
		inc temp2
		subi temp1,10
	brpl outputCalcDigitsB
		dec temp2
		subi temp1, -10

	rcall convertToDigit	
	st Y+,temp3
	mov temp1,temp2
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

	cbr statusRegister, (1<<OutputBit)
ret

convertToDigit:
	ldi ZH, HIGH(digits*2)
	ldi ZL, LOW(digits*2)
	
	add ZL, temp1
	brcc convertToDigitNoCarry
		inc ZH
	convertToDigitNoCarry:
	lpm temp3, Z
ret

digits:
.db ~0b11101101, ~0b00000101, ~0b10101011, ~0b00101111, ~0b01000111, ~0b01101110, ~0b11101110, ~0b00100101, ~0b11101111, ~0b01101111


lights:						;decrease each LED timer. If it hits zero, change the LED
	ldi counter, 8
	ldi YH, HIGH(LEDTimerMemA)
	ldi YL, LOW(LEDTimerMemA)
	clr temp2
	mov temp3, LEDsA

	lightsATimerLoop:		
		lsl temp2
		ld temp1, Y
		dec temp1
		brne lightsATimerNoYero
			sbrc temp3,7
			ldi temp1, LEDOnTimeout			;This long the LED will be on
			sbrs temp3,7
			ldi temp1, LEDOffTimeout		;This long the LED will be off
			inc temp2
		lightsATimerNoYero:
		lsl temp3
		st Y+, temp1
	dec counter
	brne lightsAtimerLoop
	eor LEDsA, temp2

	ldi counter, 8
	ldi YH, HIGH(LEDTimerMemB)
	ldi YL, LOW(LEDTimerMemB)
	clr temp2
	mov temp3, LEDsB

	lightsBTimerLoop:
		lsl temp2
		ld temp1, Y
		dec temp1
		brne lightsBTimerNoYero
			sbrc temp3,7
			ldi temp1, LEDOnTimeout			;This long the LED will be on
			sbrs temp3,7
			ldi temp1, LEDOffTimeout		;This long the LED will be off
			inc temp2
		lightsBTimerNoYero:
		lsl temp3
		st Y+, temp1
	dec counter
	brne lightsBtimerLoop
	eor LEDsB, temp2

	cbr statusRegister, (1<<LightsBit)
	sbr statusRegister, (1<<OutputBit)
ret

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
	sbr statusRegister, (1<<InputBit)
	pop temp1
	out SREG, temp1
	pop temp1
reti

TIM2_OVF:
	push temp1
	in temp1, SREG
	push temp1
	sbr statusRegister, (1<<LightsBit)
	pop temp1
	out SREG, temp1
	pop temp1
reti

