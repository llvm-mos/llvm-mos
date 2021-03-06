// RUN: split-file --no-leading-lines %s %t
// RUN: llc -O2 -align-large-globals=false -verify-machineinstrs -o %t/got.s %t/char_stats_norecurse.ll
// RUN: diff --strip-trailing-cr -u %t/want.s %t/got.s

//--- char_stats_norecurse.ll
target datalayout = "e-p:16:8:8-i16:8:8-i32:8:8-i64:8:8-f32:8:8-f64:8:8-a:8:8-Fi8-n8"
target triple = "mos"

; Function Attrs: nounwind optsize
define void @char_stats() local_unnamed_addr #0 {
entry:
  %counts = alloca [256 x i16], align 1
  %0 = bitcast [256 x i16]* %counts to i8*
  call void @llvm.lifetime.start.p0i8(i64 512, i8* nonnull %0) #4
  call void @llvm.memset.p0i8.i16(i8* nonnull align 1 dereferenceable(512) %0, i8 0, i16 512, i1 false)
  %call1 = tail call zeroext i8 bitcast (i8 (...)* @next_char to i8 ()*)() #5
  %tobool.not2 = icmp eq i8 %call1, 0
  br i1 %tobool.not2, label %while.end, label %while.body

while.body:                                       ; preds = %entry, %while.body
  %call3 = phi i8 [ %call, %while.body ], [ %call1, %entry ]
  %idxprom = zext i8 %call3 to i16
  %arrayidx = getelementptr inbounds [256 x i16], [256 x i16]* %counts, i16 0, i16 %idxprom
  %1 = load i16, i16* %arrayidx, align 1, !tbaa !2
  %inc = add nsw i16 %1, 1
  store i16 %inc, i16* %arrayidx, align 1, !tbaa !2
  %call = tail call zeroext i8 bitcast (i8 (...)* @next_char to i8 ()*)() #5
  %tobool.not = icmp eq i8 %call, 0
  br i1 %tobool.not, label %while.end, label %while.body, !llvm.loop !6

while.end:                                        ; preds = %while.body, %entry
  %arraydecay = getelementptr inbounds [256 x i16], [256 x i16]* %counts, i16 0, i16 0
  call void @report_counts(i16* nonnull %arraydecay) #5
  call void @llvm.lifetime.end.p0i8(i64 512, i8* nonnull %0) #4
  ret void
}

; Function Attrs: argmemonly nofree nosync nounwind willreturn
declare void @llvm.lifetime.start.p0i8(i64 immarg, i8* nocapture) #1  

; Function Attrs: argmemonly nofree nosync nounwind willreturn writeonly
declare void @llvm.memset.p0i8.i16(i8* nocapture writeonly, i8, i16, i1 immarg) #2

; Function Attrs: nocallback optsize
declare zeroext i8 @next_char(...) local_unnamed_addr #3

; Function Attrs: nocallback optsize
declare void @report_counts(i16*) local_unnamed_addr #3

; Function Attrs: argmemonly nofree nosync nounwind willreturn
declare void @llvm.lifetime.end.p0i8(i64 immarg, i8* nocapture) #1

attributes #0 = { nounwind optsize "disable-tail-calls"="false" "frame-pointer"="all" "less-precise-fpmad"="false" "min-legal-vector-width"="0" "no-infs-fp-math"="false" "no-jump-tables"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="true" "stack-protector-buffer-size"="8" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #1 = { argmemonly nofree nosync nounwind willreturn }
attributes #2 = { argmemonly nofree nosync nounwind willreturn writeonly }
attributes #3 = { nocallback optsize "disable-tail-calls"="false" "frame-pointer"="all" "less-precise-fpmad"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "no-signed-zeros-fp-math"="false" "no-trapping-math"="true" "stack-protector-buffer-size"="8" "unsafe-fp-math"="false" "use-soft-float"="false" }
attributes #4 = { nounwind }
attributes #5 = { nocallback nounwind optsize }

!llvm.module.flags = !{!0}
!llvm.ident = !{!1}

!0 = !{i32 1, !"wchar_size", i32 4}
!1 = !{!"clang version 12.0.0 (git@github.com:mysterymath/clang6502.git 948c84f839c7884dc9f820cf333312348759050e)"}
!2 = !{!3, !3, i64 0}
!3 = !{!"int", !4, i64 0}
!4 = !{!"omnipotent char", !5, i64 0}
!5 = !{!"Simple C/C++ TBAA"}
!6 = distinct !{!6, !7}
!7 = !{!"llvm.loop.mustprogress"}

//--- want.s
.code
.global	char__stats                     ; -- Begin function char_stats
char__stats:                            ; @char_stats
; %bb.0:                                ; %entry
	LDX	#0
	LDA	#<char__stats__sstk
	STA	z:__ZP__0
	LDA	#>char__stats__sstk
	STA	z:__ZP__1
	LDA	#0
	LDY	#2
	JSR	memset
LBB0__1:                                ; %while.body
                                        ; =>This Inner Loop Header: Depth=1
	JSR	next__char
	CMP	#0
	BEQ	LBB0__3
LBB0__2:                                ; %while.body
                                        ;   in Loop: Header=BB0_1 Depth=1
	ASL	A
	STA	z:__ZP__0
	LDA	#0
	ROL	A
	STA	z:__ZP__1
	LDA	#<char__stats__sstk
	LDX	#>char__stats__sstk
	CLC
	ADC	z:__ZP__0
	STA	z:__ZP__0
	TXA
	ADC	z:__ZP__1
	STA	z:__ZP__1
	LDY	#0
	LDA	(__ZP__0),Y
	CLC
	ADC	#1
	STA	(__ZP__0),Y
	LDY	#1
	LDA	(__ZP__0),Y
	ADC	#0
	STA	(__ZP__0),Y
	JMP	LBB0__1
LBB0__3:                                ; %while.end
	LDA	#<char__stats__sstk
	STA	z:__ZP__0
	LDA	#>char__stats__sstk
	STA	z:__ZP__1
	JSR	report__counts
	RTS
                                        ; -- End function
.bss
char__stats__sstk:                      ; @char_stats_sstk
	.res	512

.global	__ZP__0
.global	__ZP__1
.global	memset
.global	next__char
.global	report__counts
