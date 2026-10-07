; RUN: llc -verify-machineinstrs -O2 < %s 2>%t.err | FileCheck %s
; RUN: FileCheck %s --check-prefix=WARN < %t.err

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; Suffixed norecurse ISR whose shared helper contains inline asm.
; The helper is called from both mainline and the ISR, so it must be cloned.
define void @nmi_handler() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
entry:
  call void @shared_helper()
  ret void
}

define void @shared_helper() {
entry:
  store volatile i8 42, ptr @gv
  call void asm sideeffect "jsr asm_target", ""()
  call void asm sideeffect "lda __rc2", "~{a}"()
  ret void
}

@gv = global i8 0

define void @main() {
entry:
  call void @shared_helper()
  ret void
}

; The ISR should only save A, X, Y — not RC2..RC31.
; CHECK-LABEL: nmi_handler:
; CHECK-NEXT: ; %bb.0:
; CHECK-NEXT: cld
; CHECK: pha
; CHECK-NEXT: txa
; CHECK-NEXT: pha
; CHECK-NEXT: tya
; CHECK-NEXT: pha
; The ISR calls the clone, not the original helper.
; CHECK: jsr shared_helper__nmi
; CHECK: pla
; CHECK-NEXT: tay
; CHECK-NEXT: pla
; CHECK-NEXT: tax
; CHECK-NEXT: pla
; CHECK-NEXT: rti

; No +256 hardware stack guard (no tsx) — the private SP cannot race with
; mainline SP updates.
; CHECK-NOT: tsx

; The clone exists alongside the original helper.
; CHECK-LABEL: shared_helper__nmi:
; CHECK: ldx #42
; CHECK-NEXT: stx gv
; CHECK: jsr asm_target
; CHECK: lda __rc2
; CHECK: rts

; The suffixed zero-page registers are declared at the end of the assembly file.
; CHECK: .zeropage __rc0__nmi
; CHECK: .zeropage __rc31__nmi

; Inline asm text is not rewritten for the suffix: calls in it are not
; redirected to per-suffix clones, and __rc references stay in the main register set.
; WARN-DAG: warning: [MOS interrupt] inline assembly in the call tree of interrupt handler "nmi_handler" contains jsr/jmp; textual call targets are not redirected
; WARN-DAG: warning: [MOS interrupt] inline assembly in the call tree of interrupt handler "nmi_handler" references imaginary-register symbols (__rc...)
