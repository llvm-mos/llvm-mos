; RUN: opt -enable-new-pm=0 -mos-norecurse -verify -S %s | FileCheck %s
target datalayout = "e-p:16:8:8-p1:8:8-i16:8:8-i32:8:8-i64:8:8-f32:8:8-f64:8:8-a:8:8-Fi8-n8"
target triple = "mos"

declare void @external_mayrecurse()
declare void @external_nocallback() nocallback

define void @recurses_directly() {
; CHECK: define void @recurses_directly() {
  call void @recurses_directly()
  ret void
}

define void @recurses_indirectlya() {
; CHECK: define void @recurses_indirectlya() {
  call void @recurses_indirectlyb()
  ret void
}

define void @recurses_indirectlyb() {
; CHECK: define void @recurses_indirectlyb() {
  call void @recurses_indirectlya()
  ret void
}

define void @may_recurse_external_call() {
; CHECK: define void @may_recurse_external_call() {
  call void @external_mayrecurse()
  ret void
}

define void @may_recurse_inline_asm() {
; CHECK: define void @may_recurse_inline_asm() {
  call void asm "", ""()
  ret void
}

define void @may_recurse_interrupt() "interrupt" {
; CHECK: define void @may_recurse_interrupt() #1 {
  call void @may_recurse_interrupt_callee()
  ret void
}

define void @may_recurse_interrupt_callee() {
; CHECK: define void @may_recurse_interrupt_callee() {
  ret void
}

define i8 @__udivqi3(i8 %a, i8 %b) {
  ; CHECK: define i8 @__udivqi3(i8 %a, i8 %b) {
  ret i8 %a
}

define void @no_recurse_nocallback() {
; CHECK: define void @no_recurse_nocallback() #2 {
  call void @external_nocallback()
  ret void
}

define void @no_recurse_inline_asm_nocallback() {
; CHECK: define void @no_recurse_inline_asm_nocallback() #2 {
  call void asm "", ""() nocallback
  ret void
}


define void @no_recurse_once_removed() {
; CHECK: define void @no_recurse_once_removed() #2 {
  call void @no_recurse_nocallback()
  ret void
}

define void @interrupt_norecurse_a() "interrupt-norecurse" {
; CHECK: define void @interrupt_norecurse_a() #3 {
  call void @called_by_one_interrupt_norecurse()
  call void @called_by_one_interrupt_norecurse_and_main()
  call void @called_by_two_interrupt_norecurse()
  ret void
}

define void @interrupt_norecurse_b() "interrupt-norecurse" {
; CHECK: define void @interrupt_norecurse_b() #3 {
  call void @called_by_two_interrupt_norecurse()
  ret void
}

define void @called_by_one_interrupt_norecurse() {
; CHECK: define void @called_by_one_interrupt_norecurse() #2 {
  ret void
}

define void @called_by_two_interrupt_norecurse() {
; CHECK: define void @called_by_two_interrupt_norecurse() {
  ret void
}

define void @called_by_one_interrupt_norecurse_and_main() {
; CHECK: define void @called_by_one_interrupt_norecurse_and_main() {
  ret void
}

define void @main() {
  call void @called_by_one_interrupt_norecurse_and_main()
  ret void
}

; CHECK: #1 = { "interrupt" }
; CHECK: #2 = { norecurse }
; CHECK: #3 = { norecurse "interrupt-norecurse" }
