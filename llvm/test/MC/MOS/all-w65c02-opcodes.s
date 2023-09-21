; RUN: llvm-mc -assemble --print-imm-hex --show-encoding -triple mos -motorola-integers --mcpu=mosw65c02 < %s | FileCheck %s

	wai                         ; CHECK: encoding: [0xcb]
	stp                         ; CHECK: encoding: [0xdb]
