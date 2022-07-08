# RUN: llvm-mc -filetype=obj -triple=mos %s | llvm-readobj --file-headers - | FileCheck %s -check-prefixes=CHECK,6502
# RUN: llvm-mc -filetype=obj -triple=mos -mcpu=mos6502x %s | llvm-readobj --file-headers - | FileCheck %s -check-prefixes=CHECK,6502X
# RUN: llvm-mc -filetype=obj -triple=mos -mcpu=mos65c02 %s | llvm-readobj --file-headers - | FileCheck %s -check-prefixes=CHECK,65C02
# RUN: llvm-mc -filetype=obj -triple=mos -mcpu=mosr65c02 %s | llvm-readobj --file-headers - | FileCheck %s -check-prefixes=CHECK,R65C02
# RUN: llvm-mc -filetype=obj -triple=mos -mcpu=mosw65c02 %s | llvm-readobj --file-headers - | FileCheck %s -check-prefixes=CHECK,W65C02
# RUN: llvm-mc -filetype=obj -triple=mos -mcpu=mosw65816 %s | llvm-readobj --file-headers - | FileCheck %s -check-prefixes=CHECK,W65816
# RUN: llvm-mc -filetype=obj -triple=mos -mcpu=mosw65el02 %s | llvm-readobj --file-headers - | FileCheck %s -check-prefixes=CHECK,65EL02
# RUN: llvm-mc -filetype=obj -triple=mos -mcpu=mos65ce02 %s | llvm-readobj --file-headers - | FileCheck %s -check-prefixes=CHECK,65CE02

# returns with 42 in accumulator
.globl _start
_start:
  lda #42
  rts

// CHECK:       Machine: EM_MOS (0x1966)
//  6502:       Flags [
//  6502-NEXT:     EF_MOS_ARCH_6502 (0x1)
//  6502-NEXT:     EF_MOS_ARCH_6502_BCD (0x2)
// 6502X:       Flags [
// 6502X-NEXT:     EF_MOS_ARCH_6502 (0x1)
// 6502X-NEXT:     EF_MOS_ARCH_6502X (0x4)
// 6502X-NEXT:     EF_MOS_ARCH_6502_BCD (0x2)
// 65C02:       Flags [
// 65C02-NEXT:     EF_MOS_ARCH_6502 (0x1)
// 65C02-NEXT:     EF_MOS_ARCH_6502_BCD (0x2)
// 65C02-NEXT:     EF_MOS_ARCH_65C02 (0x8)
// R65C02:      Flags [
// R65C02-NEXT:    EF_MOS_ARCH_6502 (0x1)
// R65C02-NEXT:    EF_MOS_ARCH_6502_BCD (0x2)
// R65C02-NEXT:    EF_MOS_ARCH_65C02 (0x8)
// R65C02-NEXT:    EF_MOS_ARCH_R65C02 (0x10)
// W65C02:      Flags [
// W65C02-NEXT:    EF_MOS_ARCH_6502 (0x1)
// W65C02-NEXT:    EF_MOS_ARCH_6502_BCD (0x2)
// W65C02-NEXT:    EF_MOS_ARCH_65C02 (0x8)
// W65C02-NEXT:    EF_MOS_ARCH_R65C02 (0x10)
// W65C02-NEXT:    EF_MOS_ARCH_W65C02 (0x20)
// W65816:      Flags [
// W65816-NEXT:    EF_MOS_ARCH_6502 (0x1)
// W65816-NEXT:    EF_MOS_ARCH_6502_BCD (0x2)
// W65816-NEXT:    EF_MOS_ARCH_65C02 (0x8)
// W65816-NEXT:    EF_MOS_ARCH_R65C02 (0x10)
// W65816-NEXT:    EF_MOS_ARCH_W65816 (0x100)
// W65816-NEXT:    EF_MOS_ARCH_W65C02 (0x20)
// 65EL02:      Flags [
// 65EL02-NEXT:    EF_MOS_ARCH_6502 (0x1)
// 65EL02-NEXT:    EF_MOS_ARCH_6502_BCD (0x2)
// 65EL02-NEXT:    EF_MOS_ARCH_65C02 (0x8)
// 65EL02-NEXT:    EF_MOS_ARCH_65EL02 (0x200)
// 65EL02-NEXT:    EF_MOS_ARCH_R65C02 (0x10)
// 65EL02-NEXT:    EF_MOS_ARCH_W65C02 (0x20)
// 65CE02:      Flags [
// 65CE02-NEXT:    EF_MOS_ARCH_6502 (0x1)
// 65CE02-NEXT:    EF_MOS_ARCH_6502_BCD (0x2)
// 65CE02-NEXT:    EF_MOS_ARCH_65C02 (0x8)
// 65CE02-NEXT:    EF_MOS_ARCH_65CE02 (0x400)
// 65CE02-NEXT:    EF_MOS_ARCH_R65C02 (0x10)
// 65CE02-NEXT:    EF_MOS_ARCH_W65C02 (0x20)
// CHECK-NEXT:   ]
