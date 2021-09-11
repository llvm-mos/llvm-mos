# REQUIRES: mos
# RUN: llvm-mc -filetype=obj -triple=mos %s -o %t.o
# RUN: ld.lld %t.o --defsym=adr16=0x1234 --defsym=adr24=0x123456 -o %t
# RUN: llvm-objdump -d --no-show-raw-insn --print-imm-hex %t | FileCheck %s
# RUN: llvm-readelf -x .rodata -x .R_MOS_ADDR24 %t | FileCheck %s --check-prefix=ADDR24

.section .zp,"",@nobits
adrzp: .ds.b 1

.section .R_MOS_ADDR8,"ax",@progbits
  lda adrzp
# CHECK-LABEL: section .R_MOS_ADDR8:
# CHECK: lda $0

.section .R_MOS_ADDR16,"ax",@progbits
  lda adr16
# CHECK-LABEL: section .R_MOS_ADDR16:
# CHECK: lda $1234

.section .R_MOS_ADDR16_LO,"ax",@progbits
  lda #mos16lo(adr16)
# CHECK-LABEL: section .R_MOS_ADDR16_LO:
# CHECK: lda #$34

.section .R_MOS_ADDR16_HI,"ax",@progbits
  lda #mos16hi(adr16)
# CHECK-LABEL: section .R_MOS_ADDR16_HI:
# CHECK: lda #$12

.section .R_MOS_PCREL_8,"ax",@progbits
relprev:
  bpl relnext
  bpl relprev
relnext:
# CHECK-LABEL: section .R_MOS_PCREL_8:
# CHECK: bpl $2
# CHECK: bpl $fc

.section .R_MOS_ADDR24,"a",@progbits
  .long adr24
# ADDR24-LABEL: section '.R_MOS_ADDR24':
# ADDR24-NEXT: 0x000100b4 56341200

.section .R_MOS_ADDR24_BANK,"ax",@progbits
  lda #mos24bank(adr24)
# CHECK-LABEL: section .R_MOS_ADDR24_BANK:
# CHECK: lda #$12

.section .R_MOS_ADDR24_SEGMENT,"ax",@progbits
  lda mos24segment(adr24)
# CHECK-LABEL: section .R_MOS_ADDR24_SEGMENT:
# CHECK: lda $3456

.section .R_MOS_ADDR24_SEGMENT_LO,"ax",@progbits
  lda #mos24segmentlo(adr24)
# CHECK-LABEL: section .R_MOS_ADDR24_SEGMENT_LO:
# CHECK: lda #$56

.section .R_MOS_ADDR24_SEGMENT_HI,"ax",@progbits
  lda #mos24segmenthi(adr24)
# CHECK-LABEL: section .R_MOS_ADDR24_SEGMENT_HI:
# CHECK: lda #$34
