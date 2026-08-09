# REQUIRES: mos
# RUN: llvm-mc -filetype=obj -triple=mos -mcpu=mos65ce02 %s -o %t.o
# RUN: ld.lld %t.o -o %t
# RUN: llvm-objdump -d %t | FileCheck %s

## 4510 and 45GS02 objects also carry EF_MOS_ARCH_65CE02, so they take the
## same PC-relative base.
# RUN: llvm-mc -filetype=obj -triple=mos -mcpu=mos4510 %s -o %t4510.o
# RUN: ld.lld %t4510.o -o %t4510
# RUN: llvm-objdump -d %t4510 | FileCheck %s
# RUN: llvm-mc -filetype=obj -triple=mos -mcpu=mos45gs02 %s -o %t45gs02.o
# RUN: ld.lld %t45gs02.o -o %t45gs02
# RUN: llvm-objdump -d %t45gs02 | FileCheck %s

## A 65CE02 16-bit branch is relative to opcode + 2, like the 8-bit branches,
## not to the end of its 3-byte encoding. Both branches here cross a section
## boundary, so the assembler cannot resolve them and they exercise
## R_MOS_PCREL_16 in the linker.

.section .text.branch,"ax",@progbits
  beq target

.section .text.target,"ax",@progbits
target:
  rts

.section .text.backward,"ax",@progbits
  bne target

## The symbolic target must be <target>, not <target+0x1>: the encoding, the
## linker and the disassembler all have to agree on the same base.
# CHECK:      Disassembly of section .text:
# CHECK:      f3 01 00 beq ${{[0-9a-f]+}} <target>
# CHECK:      <target>:
# CHECK-NEXT: 60 rts
# CHECK-NEXT: d3 fd ff bne ${{[0-9a-f]+}} <target>
