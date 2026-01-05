; RUN: not llvm-mc -triple mos -mcpu=mos6502 %s 2>&1 | FileCheck %s

; Test assembler error diagnostics for the MOS target.

; CHECK: [[#@LINE+1]]:1: error: invalid instruction
xyz

; CHECK: [[#@LINE+1]]:1: error: too few operands for instruction
lda

; CHECK: [[#@LINE+1]]:1: error: instruction requires: FeatureW65816
brl $1234

; CHECK: [[#@LINE+1]]:1: error: instruction requires: FeatureSPC700
mov a, #$12

; bra is available on multiple CPUs, so we get multiple near-misses
; CHECK: [[#@LINE+2]]:1: error: invalid instruction, any one of the following would fix this:
; CHECK: [[#@LINE+1]]:1: note: instruction requires: Feature65C02
bra $10

; CHECK: [[#@LINE+1]]:1: error: instruction requires: FeatureW65816Or65EL02
rep #$30

; CHECK: [[#@LINE+1]]:1: error: instruction requires: FeatureW65816Or65EL02
sep #$30
