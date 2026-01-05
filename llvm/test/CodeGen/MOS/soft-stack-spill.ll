; RUN: llc -O2 < %s | FileCheck %s
;
; Test that spilling to soft stack during register allocation does not crash.
;
; This was a regression where storeRegToStackSlot() created virtual registers
; for the STStk/LDStk scratch operand during register allocation. These virtual
; registers were never allocated, causing VirtRegRewriter to crash with:
;   "Remaining virtual register %141...in instruction: STStk"
;
; The fix uses the reserved RS8 register as the scratch operand instead of
; creating a virtual register. See MOSInstrInfo::loadStoreRegStackSlot().
;
; Reduced from picolibc scanf implementation.

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos-unknown-unknown"

; CHECK-LABEL: scanf_getc:
; CHECK: sta (__rc0),y
; CHECK: lda (__rc0),y
; CHECK: rts
define fastcc i32 @scanf_getc(ptr %stream, i32 %0, i1 %cmp.i, i1 %cmp6.not.i, i1 %exitcond.not.i, i1 %cmp14.i) {
entry:
  %cmp = icmp eq i32 %0, 1
  br i1 %cmp, label %if.end, label %common.ret

if.end:
  br i1 %cmp.i, label %common.ret, label %if.end.i

if.end.i:
  br i1 %cmp6.not.i, label %for.cond.preheader.i, label %common.ret

for.cond.preheader.i:
  br i1 %exitcond.not.i, label %common.ret, label %for.body.i

for.body.i:
  ; Indirect call in loop creates register pressure requiring spills.
  ; The spilled values must survive across the call.
  %1 = load ptr, ptr %stream, align 1
  %call13.i8 = tail call i16 %1(ptr null)
  br i1 %cmp14.i, label %common.ret, label %for.body.i

common.ret:
  ret i32 0
}
