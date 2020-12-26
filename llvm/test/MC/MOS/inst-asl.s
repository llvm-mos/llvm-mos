; RUN: llvm-mc -triple mos -show-encoding < %s | FileCheck %s

foo:
  asl

; CHECK: brk                ; encoding: [0x0a]
