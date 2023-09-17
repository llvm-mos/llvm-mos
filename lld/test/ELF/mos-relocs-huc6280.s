# REQUIRES: mos
# RUN: llvm-mc -filetype=obj -triple=mos -motorola-integers --mcpu=moshuc6280 %s -o %t.o
# RUN: llvm-objdump -r %t.o | FileCheck %s --check-prefix=RELOCS
# RUN: ld.lld %t.o --defsym=sym1=0x1234 --defsym=sym2=0x5678 --defsym=sym3=0x9abc --defsym=sym4=0x5a -o %t
# RUN: llvm-objdump -d --no-show-raw-insn --print-imm-hex %t | FileCheck %s

.section .multiarg_huc_block_move,"ax",@progbits
  tii sym1, sym2, #sym3
# RELOCS-LABEL: RELOCATION RECORDS FOR [.multiarg_huc_block_move]:
# RELOCS-NEXT: OFFSET   TYPE                     VALUE
# RELOCS-NEXT: 00000001 R_MOS_ADDR16             sym1
# RELOCS-NEXT: 00000003 R_MOS_ADDR16             sym2
# RELOCS-NEXT: 00000005 R_MOS_IMM16              sym3
# CHECK-LABEL: section .multiarg_huc_block_move:
# CHECK: tii $1234,$5678,#$9abc

.section .multiarg_huc_tst,"ax",@progbits
  tst #sym4, sym1
# RELOCS-LABEL: RELOCATION RECORDS FOR [.multiarg_huc_tst]:
# RELOCS-NEXT: OFFSET   TYPE                     VALUE
# RELOCS-NEXT: 00000001 R_MOS_IMM8               sym4
# RELOCS-NEXT: 00000002 R_MOS_ADDR16             sym1
# CHECK-LABEL: section .multiarg_huc_tst:
# CHECK: tst #$5a,$1234
