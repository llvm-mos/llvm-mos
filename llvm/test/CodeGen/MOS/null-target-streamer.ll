; RUN: llc -mtriple=mos -filetype=null %s
; RUN: llvm-as %s -o - | llvm-nm - | FileCheck %s

; Code generation without output and parsing module-level assembly for the IR
; symbol table both use the null streamer.

target triple = "mos"

module asm ".zeropage zp_var"

; CHECK: T f
define void @f() {
  ret void
}
