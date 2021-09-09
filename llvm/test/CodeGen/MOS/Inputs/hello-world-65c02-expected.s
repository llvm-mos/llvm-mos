	.text
	.file	"hello-world.ll"
	.globl	main                            ; -- Begin function main
	.type	main,@function
main:                                   ; @main
; %bb.0:                                ; %entry
	clc
	lda	mos8(__rc0)
	adc	#254
	pha
	lda	mos8(__rc1)
	adc	#255
	sta	mos8(__rc1)
	pla
	sta	mos8(__rc0)
                                        ; kill: killed $p
	lda	mos8(__rc6)
	pha
	lda	#14
	ldy	#1
	sta	(mos8(__rc0)),y                 ; 1-byte Folded Spill
	lda	#72
	ldy	#0
	sta	(mos8(__rc0)),y                 ; 1-byte Folded Spill
	ldy	#mos16lo(.str+1)
	ldx	#mos16hi(.str+1)
.LBB0_1:                                ; %while.body
                                        ; =>This Inner Loop Header: Depth=1
	sty	mos8(__rc6)
	sty	mos8(__rc2)
	stx	mos8(__rc3)
	ldy	#0
	lda	(mos8(__rc0)),y                 ; 1-byte Folded Reload
	;APP
	jsr	65490
	;NO_APP
	ldy	#0
	lda	(mos8(__rc2)),y
	ldy	#0
	sta	(mos8(__rc0)),y                 ; 1-byte Folded Spill
	clc
	ldy	#1
	lda	(mos8(__rc0)),y                 ; 1-byte Folded Reload
	adc	#-1
	sta	mos8(__rc2)
	clc
	lda	mos8(__rc6)
	adc	#1
	tay
	txa
	adc	#0
	tax
	lda	mos8(__rc2)
	pha
	phy
	ldy	#1
	sta	(mos8(__rc0)),y                 ; 1-byte Folded Spill
	ply
	pla
	cmp	#0
	bne	.LBB0_1
; %bb.2:                                ; %while.end
	ldx	#0
	lda	#0
	sta	__save_a
	pla
	sta	mos8(__rc6)
	clc
	lda	mos8(__rc0)
	adc	#2
	sta	mos8(__rc0)
	lda	mos8(__rc1)
	adc	#0
	sta	mos8(__rc1)
	lda	__save_a
	rts
.Lfunc_end0:
	.size	main, .Lfunc_end0-main
                                        ; -- End function
	.type	.str,@object                    ; @.str
	.section	.rodata.str1.1,"aMS",@progbits,1
.str:
	.asciz	"HELLO, WORLD!\n"
	.size	.str, 15

	.section	".note.GNU-stack","",@progbits
