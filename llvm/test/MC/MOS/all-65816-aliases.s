; RUN: llvm-mc -assemble --print-imm-hex --show-encoding -triple mos -motorola-integers --mcpu=mosw65816 < %s | FileCheck %s

	; Allow using sp instead of s to denote the stack.

 	ora	$ea,sp                  ; CHECK: encoding: [0x03,0xea]
 	ora	($ea,sp),y              ; CHECK: encoding: [0x13,0xea]
 	and	$ea,sp                  ; CHECK: encoding: [0x23,0xea]
 	and	($ea,sp),y              ; CHECK: encoding: [0x33,0xea]
 	eor	$ea,sp                  ; CHECK: encoding: [0x43,0xea]
 	eor	($ea,sp),y              ; CHECK: encoding: [0x53,0xea]
 	adc	$ea,sp                  ; CHECK: encoding: [0x63,0xea]
 	adc	($ea,sp),y              ; CHECK: encoding: [0x73,0xea]
 	sta	$ea,sp                  ; CHECK: encoding: [0x83,0xea]
 	sta	($ea,sp),y              ; CHECK: encoding: [0x93,0xea]
 	lda	$ea,sp                  ; CHECK: encoding: [0xa3,0xea]
 	lda	($ea,sp),y              ; CHECK: encoding: [0xb3,0xea]
 	cmp	$ea,sp                  ; CHECK: encoding: [0xc3,0xea]
 	cmp	($ea,sp),y              ; CHECK: encoding: [0xd3,0xea]
 	sbc	$ea,sp                  ; CHECK: encoding: [0xe3,0xea]
 	sbc	($ea,sp),y              ; CHECK: encoding: [0xf3,0xea]
