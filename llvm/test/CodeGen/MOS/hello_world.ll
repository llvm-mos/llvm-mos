// RUN: split-file --no-leading-lines %s %t
// RUN: llc -O2 -verify-machineinstrs -o %t/got.s %t/hello_world.ll
// RUN: diff --strip-trailing-cr -u %t/want.s %t/got.s

//--- hello_world.ll
target datalayout = "e-p:16:8:8-i16:8:8-i32:8:8-i64:8:8-f32:8:8-f64:8:8-a:8:8-Fi8-n8"
target triple = "mos"

@.str = private unnamed_addr constant [15 x i8] c"HELLO, WORLD!\0A\00", align 1

; Function Attrs: nounwind optsize
define i16 @main() local_unnamed_addr #0 {
entry:
  br label %while.body

while.body:                                       ; preds = %entry, %while.body
  %mos-indexiv.iv = phi i8 [ 0, %entry ], [ %mos-indexiv.iv.next, %while.body ]
  %0 = phi i8 [ 72, %entry ], [ %3, %while.body ]
  %1 = zext i8 %mos-indexiv.iv to i16
  %uglygep = getelementptr i8, i8* getelementptr inbounds ([15 x i8], [15 x i8]* @.str, i16 0, i16 1), i16 %1
  %2 = tail call i8 asm sideeffect "JSR\09$$FFD2", "=a,0"(i8 %0) #1, !srcloc !1
  %3 = load i8, i8* %uglygep, align 1, !tbaa !2
  %mos-indexiv.iv.next = add nuw nsw i8 %mos-indexiv.iv, 1
  %exitcond = icmp eq i8 %mos-indexiv.iv.next, 14
  br i1 %exitcond, label %while.end, label %while.body, !llvm.loop !5

while.end:                                        ; preds = %while.body
  ret i16 0
}

attributes #0 = { nounwind optsize "disable-tail-calls"="false" "frame-pointer"="all" "less-precise-fpmad"="false" "min-legal-vector-width"="0" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="true" "stack-protector-buffer-size"="8" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #1 = { nounwind }

!llvm.module.flags = !{!0}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{i32 108}
!2 = !{!3, !3, i64 0}
!3 = !{!"omnipotent char", !4, i64 0}
!4 = !{!"Simple C/C++ TBAA"}
!5 = distinct !{!5, !6}
!6 = !{!"llvm.loop.mustprogress"}

//--- want.s
	.text
	.file	"hello_world.ll"
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
	lda	.str+1,llvm_mos_x
	inx
	cpx	#14
	bne	LBB0_1
LBB0_2:                                 ; %while.end
	lda	#0
	ldx	#0
	rts
.Lfunc_end0:
	.size	main, .Lfunc_end0-main
                                        ; -- End function
	.type	.str,@object                    ; @.str
	.section	.rodata.str1.1,"aMS",@progbits,1
.str:
	.asciz	"HELLO, WORLD!\n"
	.size	.str, 15

