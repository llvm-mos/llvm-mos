; RUN: llvm-mc -triple mos -mcpu=mos-6502 -show-encoding < %s | FileCheck %s
foo:
  lda   5
;; CHECK: lda   5         ; encoding: [0x01 0x05]
