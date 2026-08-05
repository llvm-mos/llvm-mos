; RUN: llvm-mc -triple mos -mcpu=mosw65816 -motorola-integers --filetype=obj -o=%t.obj %s
; RUN: llvm-objdump -d --mcpu=mosw65816 %t.obj | FileCheck %s

; RUN: not llvm-mc -triple mos -mcpu=mos65el02 -motorola-integers %s 2>&1 | FileCheck --check-prefix=EL02 %s

; COP is W65816-only: opcode $02 is NXT on the 65EL02 (a different
; instruction that happens to share the encoding), so every `cop` below must
; be rejected there even though it assembles cleanly under mosw65816. No
; existing test pinned this -- asm-errors.s only runs at -mcpu=mos6502, so a
; refactor onto the shared FeatureW65816Or65EL02 predicate used by COP's
; neighbours would otherwise pass the whole suite.

; cop's signature operand accepts arbitrary expressions, not just literal
; immediates.
sig = $5a
; EL02: [[#@LINE+1]]:2: error: instruction requires: FeatureW65816
	cop #sig                        ; CHECK: 02 5a cop #$5a

; WDC reserves $80-$ff for internal use and recommends confining user
; signature bytes to $00-$7f; $7f is the top of that range.
; EL02: [[#@LINE+1]]:2: error: instruction requires: FeatureW65816
	cop #$7f                        ; CHECK: 02 7f cop #$7f
