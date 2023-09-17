; RUN: llvm-mc -assemble --print-imm-hex --show-encoding -triple mos -motorola-integers --mcpu=mos4510 < %s | FileCheck %s

	map                             ; CHECK: encoding: [0x5c]
	eom                             ; CHECK: encoding: [0xea]
