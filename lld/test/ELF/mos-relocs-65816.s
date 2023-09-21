# REQUIRES: mos
# RUN: llvm-mc -filetype=obj -triple=mos -motorola-integers --mcpu=mosw65816 -mos-force-pcrel-reloc %s -o %t.o
# RUN: llvm-objdump -r %t.o | FileCheck %s --check-prefix=RELOCS
# RUN: ld.lld %t.o --defsym=adr24=0x123456 -o %t
# RUN: llvm-objdump -d --no-show-raw-insn --print-imm-hex %t | FileCheck %s

.section .R_MOS_ADDR24,"ax",@progbits
  lda adr24
# RELOCS-LABEL: RELOCATION RECORDS FOR [.R_MOS_ADDR24]:
# RELOCS-NEXT: OFFSET   TYPE                     VALUE
# RELOCS-NEXT: 00000001 R_MOS_ADDR24             adr24
# CHECK-LABEL: section .R_MOS_ADDR24:
# CHECK: lda $123456

.section .R_MOS_PCREL_16,"ax",@progbits
rel16prev:
  brl rel16next
  brl rel16prev
rel16next:
# RELOCS-LABEL: RELOCATION RECORDS FOR [.R_MOS_PCREL_16]:
# RELOCS-NEXT: OFFSET   TYPE                     VALUE
# RELOCS-NEXT: 00000001 R_MOS_PCREL_16           .R_MOS_PCREL_16+0x6
# RELOCS-NEXT: 00000004 R_MOS_PCREL_16           .R_MOS_PCREL_16
# CHECK-LABEL: section .R_MOS_PCREL_16:
# CHECK: brl $100bd
# CHECK: brl $100b7

.section .R_MOS_ADDR24_BANK,"ax",@progbits
  lda #mos24bank(adr24)
# RELOCS-LABEL: RELOCATION RECORDS FOR [.R_MOS_ADDR24_BANK]:
# RELOCS-NEXT: OFFSET   TYPE                     VALUE
# RELOCS-NEXT: 00000001 R_MOS_ADDR24_BANK        adr24
# CHECK-LABEL: section .R_MOS_ADDR24_BANK:
# CHECK: lda #$12

.section .R_MOS_ADDR24_SEGMENT,"ax",@progbits
  lda mos24segment(adr24)
# RELOCS-LABEL: RELOCATION RECORDS FOR [.R_MOS_ADDR24_SEGMENT]:
# RELOCS-NEXT: OFFSET   TYPE                     VALUE
# RELOCS-NEXT: 00000001 R_MOS_ADDR24_SEGMENT     adr24
# CHECK-LABEL: section .R_MOS_ADDR24_SEGMENT:
# CHECK: lda $3456

.section .R_MOS_ADDR24_SEGMENT_LO,"ax",@progbits
  lda #mos24segmentlo(adr24)
# RELOCS-LABEL: RELOCATION RECORDS FOR [.R_MOS_ADDR24_SEGMENT_LO]:
# RELOCS-NEXT: OFFSET   TYPE                     VALUE
# RELOCS-NEXT: 00000001 R_MOS_ADDR24_SEGMENT_LO  adr24
# CHECK-LABEL: section .R_MOS_ADDR24_SEGMENT_LO:
# CHECK: lda #$56

.section .R_MOS_ADDR24_SEGMENT_HI,"ax",@progbits
  lda #mos24segmenthi(adr24)
# RELOCS-LABEL: RELOCATION RECORDS FOR [.R_MOS_ADDR24_SEGMENT_HI]:
# RELOCS-NEXT: OFFSET   TYPE                     VALUE
# RELOCS-NEXT: 00000001 R_MOS_ADDR24_SEGMENT_HI  adr24
# CHECK-LABEL: section .R_MOS_ADDR24_SEGMENT_HI:
# CHECK: lda #$34
