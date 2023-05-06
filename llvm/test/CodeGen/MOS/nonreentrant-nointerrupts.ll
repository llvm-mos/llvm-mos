; RUN: opt -passes=mos-nonreentrant,verify -S %s | FileCheck %s
target datalayout = "e-p:16:8:8-p1:8:8-i16:8:8-i32:8:8-i64:8:8-f32:8:8-f64:8:8-a:8:8-Fi8-n8"
target triple = "mos"

define i8 @__udivqi3(i8 %a, i8 %b) {
  ; CHECK: define i8 @__udivqi3(i8 %a, i8 %b) #0 {
  ret i8 %a
}

; CHECK: #0 = { norecurse "nonreentrant" }
