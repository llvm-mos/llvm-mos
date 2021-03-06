; RUN: llc -verify-machineinstrs -o - %s | FileCheck %s

target triple = "mos"

@a_a = private constant i8 2
@a.a = private constant i8 2

; CHECK-LABEL: a__a:
; CHECK:       .byt 2
; CHECK-LABEL: a_2Ea:
; CHECK:       .byt 2
