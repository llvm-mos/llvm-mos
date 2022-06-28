; RUN: llvm-mc -assemble --print-imm-hex --show-encoding -triple mos --mcpu=mos65c02 < %s | FileCheck %s

 	inc                         ; CHECK: encoding: [0x1a]
 	dec                         ; CHECK: encoding: [0x3a]
 	phx                         ; CHECK: encoding: [0xda]
 	plx                         ; CHECK: encoding: [0xfa]
 	phy                         ; CHECK: encoding: [0x5a]
 	ply                         ; CHECK: encoding: [0x7a]

 	bra	$ea                     ; CHECK: encoding: [0x80,0xea]

 	stz	$ea                     ; CHECK: encoding: [0x64,0xea]
 	stz	$eaea                   ; CHECK: encoding: [0x9c,0xea,0xea]
 	stz	$ea,x                   ; CHECK: encoding: [0x74,0xea]
 	stz	$eaea,x                 ; CHECK: encoding: [0x9e,0xea,0xea]

 	bit	$ea,x                   ; CHECK: encoding: [0x34,0xea]
 	bit	$eaea,x                 ; CHECK: encoding: [0x3c,0xea,0xea]
 	bit	#$ea                    ; CHECK: encoding: [0x89,0xea]

 	tsb	$ea                     ; CHECK: encoding: [0x04,0xea]
 	tsb	$eaea                   ; CHECK: encoding: [0x0c,0xea,0xea]

 	trb	$ea                     ; CHECK: encoding: [0x14,0xea]
 	trb	$eaea                   ; CHECK: encoding: [0x1c,0xea,0xea]

 	jmp	($eaea,x)               ; CHECK: encoding: [0x7c,0xea,0xea]

