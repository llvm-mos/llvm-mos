; RUN: llvm-mc -assemble --print-imm-hex --show-encoding -triple mos --mcpu=mos65c02 < %s | FileCheck %s

 	phx                         ; CHECK: encoding: [0xda]
 	plx                         ; CHECK: encoding: [0xfa]
 	phy                         ; CHECK: encoding: [0x5a]
 	ply                         ; CHECK: encoding: [0x7a]

 	bra	$ea                     ; CHECK: encoding: [0x80,0xea]

 	stz	$ea                     ; CHECK: encoding: [0x64,0xea]
 	stz	$eaea                   ; CHECK: encoding: [0x9c,0xea,0xea]
 	stz	$ea,x                   ; CHECK: encoding: [0x74,0xea]
 	stz	$eaea,x                 ; CHECK: encoding: [0x9e,0xea,0xea]
