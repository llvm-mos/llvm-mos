; RUN: llvm-mc -triple mos -show-encoding < %s | FileCheck %s


foo:
  ora #15

;; CHECK: ora    (00, x)          ; encoding: [0x01]
