; RUN: llc -mtriple=mos -verify-machineinstrs < %s | FileCheck %s

; A direct-page (addrspace 1) pointer is 8-bit (datalayout p1:8:8). Passed as an
; argument it must take an 8-bit register, not a 16-bit RS pair — the latter made
; argument materialization emit an illegal size-mismatched COPY ((p1) = COPY $rs,
; def 8 / src 16) that -verify-machineinstrs rejects and that crashes a release build.
; The pointer's value is an 8-bit direct-page offset, so the deref is a zero-page
; indexed access.

define i8 @load_dp(ptr addrspace(1) %p) {
; CHECK-LABEL: load_dp:
; CHECK:       lda 0,x
; CHECK:       rts
  %v = load i8, ptr addrspace(1) %p
  ret i8 %v
}

define void @store_dp(ptr addrspace(1) %p, i8 %v) {
; CHECK-LABEL: store_dp:
; CHECK:       sta 0,x
; CHECK:       rts
  store i8 %v, ptr addrspace(1) %p
  ret void
}
