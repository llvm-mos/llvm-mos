	.text
	.file	"char_stats_norecurse.ll"
	.globl	char_stats                      ; -- Begin function char_stats
	.type	char_stats,@function
char_stats:                             ; @char_stats
; %bb.0:                                ; %entry
	lda	__rc4
	sta	char_stats_sstk+512
	lda	__rc5
	sta	char_stats_sstk+513
	ldx	#0
	lda	#mos16lo(char_stats_sstk)
	sta	__rc4
	lda	#mos16hi(char_stats_sstk)
	sta	__rc5
	lda	__rc4
	sta	__rc2
	lda	__rc5
	sta	__rc3
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
	sta	__rc2
	lda	#0
	rol
	sta	__rc3
	lda	#mos16lo(char_stats_sstk)
	ldx	#mos16hi(char_stats_sstk)
	clc
	adc	__rc2
	tay
	txa
	adc	__rc3
	sty	__rc2
	sta	__rc3
	ldy	#0
	lda	(__rc2),y
	sta	__rc6
	ldy	#1
	lda	(__rc2),y
	tax
	clc
	lda	__rc6
	adc	#1
	tay
	txa
	adc	#0
	tax
	tya
	ldy	#0
	sta	(__rc2),y
	txa
	ldy	#1
	sta	(__rc2),y
	jmp	LBB0_1
LBB0_3:                                 ; %while.end
	lda	__rc4
	sta	__rc2
	lda	__rc5
	sta	__rc3
	jsr	report_counts
	lda	char_stats_sstk+513
	sta	__rc5
	lda	char_stats_sstk+512
	sta	__rc4
	rts
.Lfunc_end0:
	.size	char_stats, .Lfunc_end0-char_stats
                                        ; -- End function
	.type	char_stats_sstk,@object         ; @char_stats_sstk
	.local	char_stats_sstk
	.comm	char_stats_sstk,514,1
