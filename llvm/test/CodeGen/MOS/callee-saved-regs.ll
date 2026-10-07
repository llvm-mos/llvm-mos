; RUN: llc -verify-machineinstrs -O3 < %s | FileCheck %s

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"


; Function Attrs: noreturn nounwind
define dso_local void @test_callee_saved_regs() local_unnamed_addr #0 {
; CHECK-LABEL: test_callee_saved_regs:
; CHECK-LABEL: %bb.0: ; %entry
; CHECK-NOT:		lda	__rc20
; CHECK-NOT:		pha
; CHECK-NOT:		lda __rc21
; CHECK-NOT:		pha
; CHECK-NOT:		lda __rc22
; CHECK-NOT:		pha
; CHECK-NEXT: ldy	#0
; CHECK-NEXT: tya
entry:
  br label %for.cond

; CHECK-LABEL: .LBB0_1:
; CHECK:	      sta	__rc22
; CHECK-NEXT:	  tax
; CHECK-NEXT:	  sty	__rc21
; CHECK-NEXT:	  tya
; CHECK-NEXT:	  jsr	g
; CHECK-NEXT:	  sta	__rc20
; CHECK-NEXT:	  ldx	__rc22
; CHECK-NEXT:	  lda	__rc21
; CHECK-NEXT:	  jsr	g
; CHECK-NEXT:	  ldy	__rc20
; CHECK-NEXT:	  jmp	.LBB0_1
for.cond:                                         ; preds = %for.cond, %entry
  %x1.0 = phi i8 [ 0, %entry ], [ %call, %for.cond ]
  %x2.0 = phi i8 [ 0, %entry ], [ %call2, %for.cond ]
  %call = tail call zeroext i8 @g(i8 noundef zeroext %x1.0, i8 noundef zeroext %x2.0) #2
  %call2 = tail call zeroext i8 @g(i8 noundef zeroext %x1.0, i8 noundef zeroext %x2.0) #2
  br label %for.cond
}

declare dso_local zeroext i8 @g(i8 noundef zeroext, i8 noundef zeroext) local_unnamed_addr #1

attributes #0 = {noreturn nounwind}

; Suffixed ISR should only save A, X, Y (MOS_SuffixedInterrupt_CSR) — no RC
; registers are saved since the ISR uses a private imaginary-register set.
define void @test_suffixed_isr_callee_saved() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
; CHECK-LABEL: test_suffixed_isr_callee_saved:
; CHECK-NEXT: ; %bb.0:
; CHECK-NEXT: cld
; CHECK-NOT: __rc2
; CHECK-NOT: __rc3
; CHECK: pha
; CHECK-NEXT: txa
; CHECK-NEXT: pha
; CHECK-NEXT: tya
; CHECK-NEXT: pha
; CHECK: pla
; CHECK-NEXT: tay
; CHECK-NEXT: pla
; CHECK-NEXT: tax
; CHECK-NEXT: pla
; CHECK-NEXT: rti
entry:
  ret void
}
