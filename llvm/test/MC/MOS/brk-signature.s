; RUN: llvm-mc -triple mos -show-encoding < %s | FileCheck %s

	brk     ; CHECK: encoding: [0x00]
	brk #66 ; CHECK: encoding: [0x00,0x42]
