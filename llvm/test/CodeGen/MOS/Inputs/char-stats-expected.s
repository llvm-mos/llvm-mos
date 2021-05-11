	.text
	.file	"char-stats.ll"
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
	clc
	lda	mos8(__rc0)
	sta	mos8(__rc4)
	lda	mos8(__rc1)
	sta	mos8(__rc5)
	lda	mos8(__rc4)
	sta	mos8(__rc2)
	lda	mos8(__rc5)
	sta	mos8(__rc3)
	ldx	#0
	ldy	#2
	lda	#0
	jsr	memset
LBB0_1:                                 ; %while.body
                                        ; =>This Inner Loop Header: Depth=1
	jsr	next_char
	cmp	#0
	beq	LBB0_3
; %bb.2:                                ; %while.body
                                        ;   in Loop: Header=BB0_1 Depth=1
	asl
	tay
	lda	#0
	rol
	tax
	clc
	tya
	adc	mos8(__rc4)
	sta	mos8(__rc2)
	txa
	adc	mos8(__rc5)
	sta	mos8(__rc3)
	ldy	#0
	lda	(mos8(__rc2)),y
	clc
	adc	#1
	sta	(mos8(__rc2)),y
	ldy	#1
	lda	(mos8(__rc2)),y
	adc	#0
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
