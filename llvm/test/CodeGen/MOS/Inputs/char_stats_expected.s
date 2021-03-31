	.text
	.file	"char_stats.ll"
	.globl	char_stats                      ; -- Begin function char_stats
	.type	char_stats,@function
char_stats:                             ; @char_stats
; %bb.0:                                ; %entry
	clc
	lda	mos8(__rc1)
	adc	#254
	sta	mos8(__rc1)
	lda	mos8(__rc4)
	pha
	lda	mos8(__rc5)
	pha
	ldx	#0
	clc
	lda	mos8(__rc0)
	sta	mos8(__rc4)
	lda	mos8(__rc1)
	sta	mos8(__rc5)
	lda	mos8(__rc4)
	sta	mos8(__rc2)
	lda	mos8(__rc5)
	sta	mos8(__rc3)
	lda	#0
	ldy	#2
	jsr	memset
LBB0_1:                                 ; %while.body
                                        ; =>This Inner Loop Header: Depth=1
	jsr	next_char
	cmp	#0
	beq	LBB0_3
LBB0_2:                                 ; %while.body
                                        ;   in Loop: Header=BB0_1 Depth=1
	asl
	sta	mos8(__rc2)
	lda	#0
	rol
	sta	mos8(__rc3)
	clc
	lda	mos8(__rc0)
	ldx	mos8(__rc1)
	clc
	adc	mos8(__rc2)
	tay
	txa
	adc	mos8(__rc3)
	sty	mos8(__rc2)
	sta	mos8(__rc3)
	ldy	#0
	lda	(mos8(__rc2)),y
	sta	mos8(__rc6)
	ldy	#1
	lda	(mos8(__rc2)),y
	tax
	clc
	lda	mos8(__rc6)
	adc	#1
	tay
	txa
	adc	#0
	tax
	tya
	ldy	#0
	sta	(mos8(__rc2)),y
	txa
	ldy	#1
	sta	(mos8(__rc2)),y
	jmp	LBB0_1
LBB0_3:                                 ; %while.end
	lda	mos8(__rc4)
	sta	mos8(__rc2)
	lda	mos8(__rc5)
	sta	mos8(__rc3)
	jsr	report_counts
	pla
	sta	mos8(__rc5)
	pla
	sta	mos8(__rc4)
	clc
	lda	mos8(__rc1)
	adc	#2
	sta	mos8(__rc1)
	rts
.Lfunc_end0:
	.size	char_stats, .Lfunc_end0-char_stats
                                        ; -- End function
