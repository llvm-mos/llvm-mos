; RUN: split-file %s %t
; RUN: llvm-mc -triple mos -mcpu=mosw65816 --show-encoding %t/constants.s | FileCheck %s --check-prefix=ENC
; RUN: llvm-mc -triple mos -mcpu=mosw65816 -filetype=obj %t/constants.s -o %t.o
; RUN: llvm-objdump -d --mcpu=mosw65816 %t.o | FileCheck %s --check-prefix=DIS
; RUN: llvm-mc -triple mos -mcpu=mosw65816 --show-encoding %t/symbols.s | FileCheck %s --check-prefix=FIXUP
; RUN: llvm-mc -triple mos -mcpu=mosw65816 -filetype=obj %t/symbols.s -o %t.sym.o
; RUN: llvm-readobj -r %t.sym.o | FileCheck %s --check-prefix=RELOC
; RUN: llvm-mc -triple mos -mcpu=mosw65816 -filetype=obj %t/resolved.s -o %t.resolved.o
; RUN: llvm-objdump -s %t.resolved.o | FileCheck %s --check-prefix=RESOLVED

; Assembly operands are source,destination; instruction bytes are opcode,destination,source.
; Distinct banks expose the order in both encoding and decoding.
;--- constants.s
mvn #127, #0
; ENC: encoding: [0x54,0x00,0x7f]
; DIS: 54 00 7f {{.*}}mvn #$7f,#$0
mvp #18, #52
; ENC: encoding: [0x44,0x34,0x12]
; DIS: 44 34 12 {{.*}}mvp #$12,#$34
mvn #0, #255
; ENC: encoding: [0x54,0xff,0x00]
; DIS: 54 ff 00 {{.*}}mvn #$0,#$ff
mvp #255, #0
; ENC: encoding: [0x44,0x00,0xff]
; DIS: 44 00 ff {{.*}}mvp #$ff,#$0

; Symbolic operands must relocate the same bytes as immediate constants.
;--- symbols.s
mvn #source, #destination
; FIXUP: mvn
; FIXUP-DAG: offset: 2, value: source, kind: Imm8
; FIXUP-DAG: offset: 1, value: destination, kind: Imm8
mvp #source, #destination
; FIXUP: mvp
; FIXUP-DAG: offset: 2, value: source, kind: Imm8
; FIXUP-DAG: offset: 1, value: destination, kind: Imm8
; RELOC-DAG: 0x2 R_MOS_IMM8 source
; RELOC-DAG: 0x1 R_MOS_IMM8 destination
; RELOC-DAG: 0x5 R_MOS_IMM8 source
; RELOC-DAG: 0x4 R_MOS_IMM8 destination

; Forward-defined bank constants exercise fixup application without a linker.
;--- resolved.s
mvn #source, #destination
mvp #source, #destination
.set source, 18
.set destination, 52
; RESOLVED: 54341244 3412
