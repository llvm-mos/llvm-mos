	.text
	.file	"print_int.ll"
	.globl	print_int                       ; -- Begin function print_int
	.type	print_int,@function
print_int:                              ; @print_int
; %bb.0:                                ; %entry
	cmp	#10
	bcc	LBB0_2
LBB0_1:                                 ; %if.end.preheader
	sta	_SaveA
	lda	mos8(__rc4)
	pha
	lda	_SaveA
	sta	mos8(__rc4)
	ldx	#10
	jsr	__udivqi3
	jsr	print_int
	lda	mos8(__rc4)
	ldx	#10
	jsr	__umodqi3
	sta	_SaveA
	pla
	sta	mos8(__rc4)
	lda	_SaveA
LBB0_2:                                 ; %if.then
	clc
	adc	#48
	;APP
	jsr	65490
	;NO_APP
	rts
.Lfunc_end0:
	.size	print_int, .Lfunc_end0-print_int
                                        ; -- End function
