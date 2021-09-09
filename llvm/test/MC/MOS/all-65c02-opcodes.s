; RUN: llvm-mc -assemble --print-imm-hex --show-encoding -triple mos --mcpu=mos65c02 < %s | FileCheck %s

 	phx                         ; CHECK: encoding: [0xda]
 	plx                         ; CHECK: encoding: [0xfa]
 	phy                         ; CHECK: encoding: [0x5a]
 	ply                         ; CHECK: encoding: [0x7a]
