	.text
	.file	"hello-world.ll"
	.globl	main                            ; -- Begin function main
	.type	main,@function
main:                                   ; @main
; %bb.0:                                ; %entry
	ldx	#0
	lda	#72
LBB0_1:                                 ; %while.body
                                        ; =>This Inner Loop Header: Depth=1
	;APP
	jsr	65490
	;NO_APP
	lda	.str+1,x
	inx
	cpx	#14
	bne	LBB0_1
LBB0_2:                                 ; %while.end
	ldx	#0
	lda	#0
	rts
.Lfunc_end0:
	.size	main, .Lfunc_end0-main
                                        ; -- End function
	.type	.str,@object                    ; @.str
	.section	.rodata.str1.1,"aMS",@progbits,1
.str:
	.asciz	"HELLO, WORLD!\n"
	.size	.str, 15

