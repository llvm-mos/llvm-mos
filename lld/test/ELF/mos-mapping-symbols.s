# REQUIRES: mos
# RUN: llvm-mc -filetype=obj -triple=mos -motorola-integers --mcpu=mosw65816 %s -o %t.o
# RUN: ld.lld %t.o -o %t
# RUN: llvm-objdump -d --no-show-raw-insn --print-imm-hex %t | FileCheck %s

$ml.0:
  lda #$eaea
# CHECK-LABEL: <$ml.0>:
# CHECK-NEXT: lda #$eaea
$mh.1:
  lda #$ea
# CHECK-LABEL: <$mh.1>:
# CHECK-NEXT: lda #$ea
$xl.2:
  ldx #$eaea
# CHECK-LABEL: <$xl.2>:
# CHECK-NEXT: ldx #$eaea
$xh.3:
  ldx #$ea
# CHECK-LABEL: <$xh.3>:
# CHECK-NEXT: ldx #$ea
