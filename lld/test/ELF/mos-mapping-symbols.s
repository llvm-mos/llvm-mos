# REQUIRES: mos
# RUN: llvm-mc -filetype=obj -triple=mos -motorola-integers --mcpu=mosw65816 %s -o %t.o
# RUN: ld.lld %t.o -o %t
# RUN: llvm-objdump -d --no-show-raw-insn --show-all-symbols --print-imm-hex %t | FileCheck %s

  lda #$eaea
# CHECK-LABEL: <$ml.0>:
# CHECK-NEXT: lda #$eaea
  lda #$ea
# CHECK-LABEL: <$mh.1>:
# CHECK-NEXT: lda #$ea
  ldx #$eaea
# CHECK-LABEL: <$xl.2>:
# CHECK-NEXT: ldx #$eaea
  ldx #$ea
# CHECK-LABEL: <$xh.3>:
# CHECK-NEXT: ldx #$ea
