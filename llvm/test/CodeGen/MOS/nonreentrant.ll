; RUN: opt -passes=mos-nonreentrant,verify -S %s | FileCheck %s
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

define void @reentrant_interrupt() "interrupt" {
; CHECK: define void @reentrant_interrupt() #1 {
  call void @reentrant_interrupt_callee()
  ret void
}

define void @reentrant_interrupt_callee() {
; CHECK: define void @reentrant_interrupt_callee() #2 {
  ret void
}

define i8 @__udivqi3(i8 %a, i8 %b) {
  ; CHECK: define i8 @__udivqi3(i8 %a, i8 %b) #2 {
  ret i8 %a
}

define void @no_recurse_nocallback() {
; CHECK: define void @no_recurse_nocallback() #3 {
  call void @external_nocallback()
  ret void
}

define void @no_recurse_inline_asm_nocallback() {
; CHECK: define void @no_recurse_inline_asm_nocallback() #3 {
  call void asm "", ""() nocallback
  ret void
}


define void @no_recurse_once_removed() {
; CHECK: define void @no_recurse_once_removed() #3 {
  call void @no_recurse_nocallback()
  ret void
}

define void @interrupt_norecurse_a() "interrupt-norecurse" {
; CHECK: define void @interrupt_norecurse_a() #4 {
  call void @called_by_one_interrupt_norecurse()
  call void @called_by_one_interrupt_norecurse_and_main()
  call void @called_by_two_interrupt_norecurse()
  ret void
}

define void @interrupt_norecurse_b() "interrupt-norecurse" {
; CHECK: define void @interrupt_norecurse_b() #4 {
  call void @called_by_two_interrupt_norecurse()
  ret void
}

define void @called_by_one_interrupt_norecurse() {
; CHECK: define void @called_by_one_interrupt_norecurse() #3 {
  ret void
}

define void @called_by_two_interrupt_norecurse() {
; CHECK: define void @called_by_two_interrupt_norecurse() #2 {
  ret void
}

define void @called_by_one_interrupt_norecurse_and_main() {
; CHECK: define void @called_by_one_interrupt_norecurse_and_main() #2 {
  ret void
}

define void @main() {
  call void @called_by_one_interrupt_norecurse_and_main()
  ret void
}

; CHECK: #0 = { nocallback }
; CHECK: #1 = { norecurse "interrupt" }
; CHECK: #2 = { norecurse }
; CHECK: #3 = { norecurse "nonreentrant" }
; CHECK: #4 = { norecurse "interrupt-norecurse" "nonreentrant" }
