; RUN: llc -verify-machineinstrs -O2 < %s | FileCheck %s --check-prefix=OPT
; RUN: llc -verify-machineinstrs -O0 < %s | FileCheck %s --check-prefix=O0

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; A suffixed norecurse ISR whose tree contains a recursive helper. The
; recursive helper's clone lacks "nonreentrant", so the prologue must
; initialize the private soft stack pointer from __stack<suffix>. This is safe
; only because a norecurse handler cannot preempt itself.
define void @nmi_handler() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
entry:
  call void @recursive_helper()
  ret void
}

@g_counter = global i8 0

define void @recursive_helper() {
entry:
  %c = load i8, ptr @g_counter
  %t = icmp eq i8 %c, 0
  br i1 %t, label %ret, label %rec
rec:
  %dec = add i8 %c, -1
  store i8 %dec, ptr @g_counter
  call void @recursive_helper()
  br label %ret
ret:
  ret void
}

define void @main() {
entry:
  ret void
}

; At -O2: the prologue inits __rc0__nmi/__rc1__nmi from __stack__nmi because
; the recursive helper's clone cannot be "nonreentrant". The init comes after
; A/X/Y are saved, so A needs no extra save around it.
; OPT-LABEL: nmi_handler:
; OPT-NEXT: ; %bb.0:
; OPT-NEXT: cld
; OPT-NEXT: pha
; OPT-NEXT: txa
; OPT-NEXT: pha
; OPT-NEXT: tya
; OPT-NEXT: pha
; OPT-NEXT: lda #mos16lo(__stack__nmi)
; OPT-NEXT: sta __rc0__nmi
; OPT-NEXT: lda #mos16hi(__stack__nmi)
; OPT-NEXT: sta __rc1__nmi
; OPT-NEXT: jsr recursive_helper__nmi
; OPT-NEXT: pla
; OPT-NEXT: tay
; OPT-NEXT: pla
; OPT-NEXT: tax
; OPT-NEXT: pla
; OPT-NEXT: rti

; At -O0: nothing is "nonreentrant", so every function may use the soft stack
; and the init is always emitted.
; O0-LABEL: nmi_handler:
; O0-NEXT: ; %bb.0:
; O0-NEXT: cld
; O0-NEXT: pha
; O0-NEXT: txa
; O0-NEXT: pha
; O0-NEXT: tya
; O0-NEXT: pha
; O0-NEXT: lda #mos16lo(__stack__nmi)
; O0-NEXT: sta __rc0__nmi
; O0-NEXT: lda #mos16hi(__stack__nmi)
; O0-NEXT: sta __rc1__nmi

; Suffixed zero-page registers.
; OPT: .zeropage __rc0__nmi
; OPT: .zeropage __rc31__nmi
