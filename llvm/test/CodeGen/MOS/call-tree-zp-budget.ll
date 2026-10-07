; RUN: llc -verify-machineinstrs -O2 -zp-avail=8 < %s | FileCheck %s --check-prefix=FULL
; RUN: llc -verify-machineinstrs -O2 -zp-avail=9 < %s | FileCheck %s --check-prefix=ROOM

target datalayout = "e-m:e-p:16:8-p1:8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mos"

; The __nmi imaginary registers also live in zero page. The NMI uses
; pairs 1 and 2 (__rc2__nmi..__rc4__nmi); pairs 0 (soft stack pointer) and 8
; (scavenger temps) are always reserved. That's 8 bytes, so with 8 bytes
; available nothing else can be promoted to zero page, and with 9 bytes @hot
; can.
@g = global i16 0
@h = global i16 0
@hot = internal global i8 0

define void @nmi() "interrupt-norecurse" "interrupt-rc-suffix"="__nmi" {
  %a = load volatile i16, ptr @g
  %b = load volatile i16, ptr @h
  %m = call i16 @work(i16 %a, i16 %b)
  store volatile i16 %m, ptr @g
  ret void
}

define internal i16 @work(i16 %a, i16 %b) noinline {
  %x = add i16 %a, %b
  %y = xor i16 %x, 85
  ret i16 %y
}

define void @main() {
entry:
  br label %loop
loop:
  %v = load volatile i8, ptr @hot
  %w = add i8 %v, 1
  store volatile i8 %w, ptr @hot
  br label %loop
}

; FULL-NOT: mos8(hot)
; FULL: ldx hot
; FULL-NOT: mos8(hot)
; ROOM: ldx mos8(hot)
