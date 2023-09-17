# REQUIRES: mos
# RUN: llvm-mc -filetype=obj -triple=mos -motorola-integers -mos-force-pcrel-reloc %s -o %t.o
# RUN: llvm-objdump -r %t.o | FileCheck %s --check-prefix=RELOCS
# RUN: ld.lld %t.o --defsym=adr16=0x1234 --defsym=data32=0x12345678 -o %t
# RUN: llvm-objdump -d --no-show-raw-insn --print-imm-hex %t | FileCheck %s
# RUN: llvm-readelf -x .R_MOS_DATA %t | FileCheck %s --check-prefix=DATA

.section .zp,"",@nobits
adrzp: .ds.b 1

.section .R_MOS_ADDR8,"ax",@progbits
  lda adrzp
# RELOCS-LABEL: RELOCATION RECORDS FOR [.R_MOS_ADDR8]:
# RELOCS-NEXT: OFFSET   TYPE                     VALUE
# RELOCS-NEXT: 00000001 R_MOS_ADDR8              .zp
# CHECK-LABEL: section .R_MOS_ADDR8:
# CHECK: lda $0

.section .R_MOS_ADDR16,"ax",@progbits
  lda adr16
# RELOCS-LABEL: RELOCATION RECORDS FOR [.R_MOS_ADDR16]:
# RELOCS-NEXT: OFFSET   TYPE                     VALUE
# RELOCS-NEXT: 00000001 R_MOS_ADDR16             adr16
# CHECK-LABEL: section .R_MOS_ADDR16:
# CHECK: lda $1234

.section .R_MOS_ADDR16_LO,"ax",@progbits
  lda #mos16lo(adr16)
# RELOCS-LABEL: RELOCATION RECORDS FOR [.R_MOS_ADDR16_LO]:
# RELOCS-NEXT: OFFSET   TYPE                     VALUE
# RELOCS-NEXT: 00000001 R_MOS_ADDR16_LO          adr16
# CHECK-LABEL: section .R_MOS_ADDR16_LO:
# CHECK: lda #$34

.section .R_MOS_ADDR16_HI,"ax",@progbits
  lda #mos16hi(adr16)
# RELOCS-LABEL: RELOCATION RECORDS FOR [.R_MOS_ADDR16_HI]:
# RELOCS-NEXT: OFFSET   TYPE                     VALUE
# RELOCS-NEXT: 00000001 R_MOS_ADDR16_HI          adr16
# CHECK-LABEL: section .R_MOS_ADDR16_HI:
# CHECK: lda #$12

.section .R_MOS_PCREL_8,"ax",@progbits
relprev:
  bpl relnext
  bpl relprev
relnext:
# RELOCS-LABEL: RELOCATION RECORDS FOR [.R_MOS_PCREL_8]:
# RELOCS-NEXT: OFFSET   TYPE                     VALUE
# RELOCS-NEXT: 00000001 R_MOS_PCREL_8            .R_MOS_PCREL_8+0x4
# RELOCS-NEXT: 00000003 R_MOS_PCREL_8            .R_MOS_PCREL_8
# CHECK-LABEL: section .R_MOS_PCREL_8:
# CHECK: bpl $100c6
# CHECK: bpl $100c2

.section .R_MOS_DATA,"a",@progbits
  .long data32
  .byte adr16@mos16hi
# RELOCS-LABEL: RELOCATION RECORDS FOR [.R_MOS_DATA]:
# RELOCS-NEXT: OFFSET   TYPE                     VALUE
# RELOCS-NEXT: 00000000 R_MOS_FK_DATA_4          data32
# RELOCS-NEXT: 00000004 R_MOS_ADDR16_HI          adr16
# DATA-LABEL: section '.R_MOS_DATA':
# DATA-NEXT: 0x{{[0-9a-f]+}} 78563412 12
