	.text
	.file	"char-stats-norecurse.ll"
	.globl	char_stats                      ; -- Begin function char_stats
	.type	char_stats,@function
char_stats:                             ; @char_stats
; %bb.0:                                ; %entry
	lda	mos8(__rc4)
	sta	__char_stats_sstk+512           ; 1-byte Folded Spill
	lda	mos8(__rc5)
	sta	__char_stats_sstk+513           ; 1-byte Folded Spill
	lda	#mos16lo(__char_stats_sstk)
	ldx	#mos16hi(__char_stats_sstk)
	sta	mos8(__rc4)
	stx	mos8(__rc5)
	ldx	#0
	lda	mos8(__rc4)
	sta	mos8(__rc2)
	lda	mos8(__rc5)
	sta	mos8(__rc3)
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
	sta	mos8(__rc3)
	lda	#0
	rol
	tay
	ldx	#mos16lo(__char_stats_sstk)
	stx	mos8(__rc2)
	clc
	lda	mos8(__rc3)
	adc	mos8(__rc2)
	sta	mos8(__rc2)
	lda	#mos16hi(__char_stats_sstk)
	sta	mos8(__rc3)
	tya
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
	jmp	LBB0_1
LBB0_3:                                 ; %while.end
	lda	mos8(__rc4)
	sta	mos8(__rc2)
	lda	mos8(__rc5)
	sta	mos8(__rc3)
	jsr	report_counts
	lda	__char_stats_sstk+513           ; 1-byte Folded Reload
	sta	mos8(__rc5)
	lda	__char_stats_sstk+512           ; 1-byte Folded Reload
	sta	mos8(__rc4)
	rts
.Lfunc_end0:
	.size	char_stats, .Lfunc_end0-char_stats
                                        ; -- End function
	.type	__char_stats_sstk,@object       ; @__char_stats_sstk
	.local	__char_stats_sstk
	.comm	__char_stats_sstk,514,1
