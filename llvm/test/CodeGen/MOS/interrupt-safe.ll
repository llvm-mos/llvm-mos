; RUN: llc -verify-machineinstrs -O2 < %s | FileCheck %s
; RUN: sed 's/^;PTR //' %s | not llc -O2 2>&1 | FileCheck %s --check-prefix=PTR
; RUN: sed 's/"interrupt-safe"//' %s | not llc -O2 2>&1 | FileCheck %s --check-prefix=PLAIN

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; An "interrupt-safe" declaration (e.g. an assembly routine that only uses
; A/X/Y) may be called from the ISR tree as-is: it isn't cloned or suffixed.
declare void @asm_fn(i8) "interrupt-safe"

; An interrupt_safe routine taking a pointer receives it in imaginary registers,
; which would be the suffixed registers, not the ones the routine uses.
declare void @asm_ptr(ptr) "interrupt-safe"

@g = global i8 0

define void @nmi() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
  %v = load volatile i8, ptr @g
  call void @asm_fn(i8 %v)
;PTR call void @asm_ptr(ptr @g)
  ret void
}

define void @main() {
  ret void
}

; CHECK-LABEL: nmi:
; CHECK: jsr asm_fn
; CHECK-NOT: asm_fn__nmi
; Being callback-free, it doesn't stop the NMI from getting a static stack.
; CHECK-NOT: __stack__nmi

; PTR: error:{{.*}}[MOS interrupt] arguments/return of interrupt_safe 'asm_ptr' use imaginary registers; only A/X/Y allowed

; PLAIN: error:{{.*}}[MOS interrupt] call to external function 'asm_fn' in the call tree of interrupt handler "nmi": it is not available as bitcode (defined in assembly or a non-LTO object); mark it interrupt_safe if it only uses A/X/Y
