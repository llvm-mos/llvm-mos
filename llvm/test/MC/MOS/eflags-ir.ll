; RUN: llc -mtriple=mos -mcpu=mos6502 -filetype=obj | llvm-readobj --file-headers - | FileCheck -check-prefixes=CHECK,6502 %s
; RUN: llc -mtriple=mos -mcpu=mos6502x -filetype=obj | llvm-readobj --file-headers - | FileCheck -check-prefixes=CHECK,6502X %s
; RUN: llc -mtriple=mos -mcpu=mos65c02 -filetype=obj | llvm-readobj --file-headers - | FileCheck -check-prefixes=CHECK,65C02 %s
; RUN: llc -mtriple=mos -mcpu=mosr65c02 -filetype=obj | llvm-readobj --file-headers - | FileCheck -check-prefixes=CHECK,R65C02 %s
; RUN: llc -mtriple=mos -mcpu=mosw65c02 -filetype=obj | llvm-readobj --file-headers - | FileCheck -check-prefixes=CHECK,W65C02 %s
; RUN: llc -mtriple=mos -mcpu=mosw65816 -filetype=obj | llvm-readobj --file-headers - | FileCheck -check-prefixes=CHECK,W65816 %s
; RUN: llc -mtriple=mos -mcpu=mos65el02 -filetype=obj | llvm-readobj --file-headers - | FileCheck -check-prefixes=CHECK,65EL02 %s
; RUN: llc -mtriple=mos -mcpu=mos65ce02 -filetype=obj | llvm-readobj --file-headers - | FileCheck -check-prefixes=CHECK,65CE02 %s
; RUN: llc -mtriple=mos -mcpu=moshuc6280 -filetype=obj | llvm-readobj --file-headers - | FileCheck -check-prefixes=CHECK,HUC6280 %s
; RUN: llc -mtriple=mos -mcpu=mos65dtv02 -filetype=obj | llvm-readobj --file-headers - | FileCheck -check-prefixes=CHECK,65DTV02 %s
; RUN: llc -mtriple=mos -mcpu=mos4510 -filetype=obj | llvm-readobj --file-headers - | FileCheck -check-prefixes=CHECK,4510 %s
; RUN: llc -mtriple=mos -mcpu=mos45gs02 -filetype=obj | llvm-readobj --file-headers - | FileCheck -check-prefixes=CHECK,45GS02 %s
; RUN: llc -mtriple=mos -mcpu=mosspc700 -filetype=obj | llvm-readobj --file-headers - | FileCheck -check-prefixes=CHECK,SPC700 %s

; CHECK:        Machine: EM_MOS (0x1966)
; 6502:         Flags [
; 6502-NEXT:      EF_MOS_ARCH_6502 (0x1)
; 6502-NEXT:      EF_MOS_ARCH_6502_BCD (0x2)
; 6502X:        Flags [
; 6502X-NEXT:     EF_MOS_ARCH_6502 (0x1)
; 6502X-NEXT:     EF_MOS_ARCH_6502X (0x4)
; 6502X-NEXT:     EF_MOS_ARCH_6502_BCD (0x2)
; 65C02:        Flags [
; 65C02-NEXT:     EF_MOS_ARCH_6502 (0x1)
; 65C02-NEXT:     EF_MOS_ARCH_6502_BCD (0x2)
; 65C02-NEXT:     EF_MOS_ARCH_65C02 (0x8)
; R65C02:       Flags [
; R65C02-NEXT:    EF_MOS_ARCH_6502 (0x1)
; R65C02-NEXT:    EF_MOS_ARCH_6502_BCD (0x2)
; R65C02-NEXT:    EF_MOS_ARCH_65C02 (0x8)
; R65C02-NEXT:    EF_MOS_ARCH_R65C02 (0x10)
; W65C02:       Flags [
; W65C02-NEXT:    EF_MOS_ARCH_6502 (0x1)
; W65C02-NEXT:    EF_MOS_ARCH_6502_BCD (0x2)
; W65C02-NEXT:    EF_MOS_ARCH_65C02 (0x8)
; W65C02-NEXT:    EF_MOS_ARCH_R65C02 (0x10)
; W65C02-NEXT:    EF_MOS_ARCH_W65C02 (0x20)
; W65816:       Flags [
; W65816-NEXT:    EF_MOS_ARCH_6502 (0x1)
; W65816-NEXT:    EF_MOS_ARCH_6502_BCD (0x2)
; W65816-NEXT:    EF_MOS_ARCH_65C02 (0x8)
; W65816-NEXT:    EF_MOS_ARCH_W65816 (0x100)
; W65816-NEXT:    EF_MOS_ARCH_W65C02 (0x20)
; 65EL02:       Flags [
; 65EL02-NEXT:    EF_MOS_ARCH_6502 (0x1)
; 65EL02-NEXT:    EF_MOS_ARCH_6502_BCD (0x2)
; 65EL02-NEXT:    EF_MOS_ARCH_65C02 (0x8)
; 65EL02-NEXT:    EF_MOS_ARCH_65EL02 (0x200)
; 65EL02-NEXT:    EF_MOS_ARCH_W65C02 (0x20)
; 65CE02:       Flags [
; 65CE02-NEXT:    EF_MOS_ARCH_6502 (0x1)
; 65CE02-NEXT:    EF_MOS_ARCH_6502_BCD (0x2)
; 65CE02-NEXT:    EF_MOS_ARCH_65C02 (0x8)
; 65CE02-NEXT:    EF_MOS_ARCH_65CE02 (0x400)
; 65CE02-NEXT:    EF_MOS_ARCH_R65C02 (0x10)
; HUC6280:      Flags [
; HUC6280-NEXT:   EF_MOS_ARCH_6502 (0x1)
; HUC6280-NEXT:   EF_MOS_ARCH_6502_BCD (0x2)
; HUC6280-NEXT:   EF_MOS_ARCH_65C02 (0x8)
; HUC6280-NEXT:   EF_MOS_ARCH_HUC6280 (0x800)
; HUC6280-NEXT:   EF_MOS_ARCH_R65C02 (0x10)
; 65DTV02:      Flags [
; 65DTV02-NEXT:   EF_MOS_ARCH_6502 (0x1)
; 65DTV02-NEXT:   EF_MOS_ARCH_6502_BCD (0x2)
; 65DTV02-NEXT:   EF_MOS_ARCH_65DTV02 (0x1000)
;  4510:        Flags [
;  4510-NEXT:     EF_MOS_ARCH_4510 (0x2000)
;  4510-NEXT:     EF_MOS_ARCH_6502 (0x1)
;  4510-NEXT:     EF_MOS_ARCH_6502_BCD (0x2)
;  4510-NEXT:     EF_MOS_ARCH_65C02 (0x8)
;  4510-NEXT:     EF_MOS_ARCH_65CE02 (0x400)
;  4510-NEXT:     EF_MOS_ARCH_R65C02 (0x10)
; 45GS02:       Flags [
; 45GS02-NEXT:    EF_MOS_ARCH_4510 (0x2000)
; 45GS02-NEXT:    EF_MOS_ARCH_45GS02 (0x4000)
; 45GS02-NEXT:    EF_MOS_ARCH_6502 (0x1)
; 45GS02-NEXT:    EF_MOS_ARCH_6502_BCD (0x2)
; 45GS02-NEXT:    EF_MOS_ARCH_65C02 (0x8)
; 45GS02-NEXT:    EF_MOS_ARCH_65CE02 (0x400)
; 45GS02-NEXT:    EF_MOS_ARCH_R65C02 (0x10)
; SPC700:       Flags [
; SPC700-NEXT:    EF_MOS_ARCH_SPC700 (0x20000)
; CHECK-NEXT:   ]

define void @func0() {
entry:
  ret void
}

