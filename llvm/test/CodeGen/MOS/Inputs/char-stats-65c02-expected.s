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
	lda	mos8(__rc6)
	pha
	lda	mos8(__rc7)
	pha
	clc
	lda	mos8(__rc0)
	sta	mos8(__rc6)
	lda	mos8(__rc1)
	sta	mos8(__rc7)
	lda	mos8(__rc6)
	sta	mos8(__rc2)
	lda	mos8(__rc7)
	sta	mos8(__rc3)
	ldx	#0
	lda	#2
	sta	mos8(__rc4)
	lda	#0
	jsr	__memset
.LBB0_1:                                ; %while.body
                                        ; =>This Inner Loop Header: Depth=1
	jsr	next_char
	cmp	#0
	beq	.LBB0_3
; %bb.2:                                ; %while.body
                                        ;   in Loop: Header=BB0_1 Depth=1
	sta	mos8(__rc2)
	asl	mos8(__rc2)
	lda	#0
	sta	mos8(__rc3)
	rol	mos8(__rc3)
	clc
	lda	mos8(__rc0)
	ldx	mos8(__rc1)
	clc
	adc	mos8(__rc2)
	sta	mos8(__rc2)
	txa
	adc	mos8(__rc3)
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
	bra	.LBB0_1
.LBB0_3:                                ; %while.end
	lda	mos8(__rc6)
	sta	mos8(__rc2)
	lda	mos8(__rc7)
	sta	mos8(__rc3)
	jsr	report_counts
	pla
	sta	mos8(__rc7)
	pla
	sta	mos8(__rc6)
	clc
	lda	mos8(__rc1)
	adc	#2
	sta	mos8(__rc1)
	rts
.Lfunc_end0:
	.size	char_stats, .Lfunc_end0-char_stats
                                        ; -- End function
	.ident	"clang version 12.0.0 (git@github.com:mysterymath/clang6502.git b8d4efa1d0099ce79290e539ba71fa8599aaa274)"
	.section	".note.GNU-stack","",@progbits
