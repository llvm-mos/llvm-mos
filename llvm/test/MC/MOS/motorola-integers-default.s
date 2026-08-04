; RUN: llvm-mc -triple mos -show-encoding < %s | FileCheck %s
; RUN: llvm-mc -triple mos -show-encoding -motorola-integers=false < %s | FileCheck %s --check-prefix=DISABLED

	lda #$ea ; CHECK: encoding: [0xa9,0xea]
; DISABLED: encoding: [0xa9,A]
