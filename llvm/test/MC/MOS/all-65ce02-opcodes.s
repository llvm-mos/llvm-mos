; RUN: llvm-mc -assemble --print-imm-hex --show-encoding -triple mos --mcpu=mosw65ce02 < %s | FileCheck %s

	inz                         ; CHECK: encoding: [0x1b]
	dez                         ; CHECK: encoding: [0x3b]
	neg                         ; CHECK: encoding: [0x42]
	asr                         ; CHECK: encoding: [0x43]
	taz                         ; CHECK: encoding: [0x4b]
	tza                         ; CHECK: encoding: [0x6b]
	phz                         ; CHECK: encoding: [0xdb]
	plz                         ; CHECK: encoding: [0xfb]

        rtn     #$ea                ; CHECK: encoding: [0x62,0xea]
        dew     $ea                 ; CHECK: encoding: [0xc3,0xea]
        inw     $ea                 ; CHECK: encoding: [0xe3,0xea]

        ldz     #$ea                ; CHECK: encoding: [0xa3,0xea]
 	ldz	$eaea               ; CHECK: encoding: [0xab,0xea,0xea]
 	ldz	$eaea,x             ; CHECK: encoding: [0xbb,0xea,0xea]

