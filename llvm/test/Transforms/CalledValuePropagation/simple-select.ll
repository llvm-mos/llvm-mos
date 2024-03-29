; RUN: opt -passes=called-value-propagation -S < %s | FileCheck %s

target triple = "aarch64"

@global_function = internal unnamed_addr global ptr null, align 8
@global_scalar = internal unnamed_addr global i64 zeroinitializer

; This test checks that we propagate the functions through a select
; instruction, and attach !callees metadata to the call. Such metadata can
; enable optimizations of this code sequence.
;
; For example, since both of the targeted functions have the "norecurse"
; attribute, the function attributes pass can be made to infer that
; "@test_select" is also norecurse. This would allow the globals optimizer to
; localize "@global_scalar". The function could then be further simplified to
; always return the constant "1", eliminating the load and store instructions.
;
; CHECK: call void %tmp0(), !callees ![[MD:[0-9]+]]
; CHECK: ![[MD]] = !{ptr @norecurse_1, ptr @norecurse_2}
;
define i64 @test_select_entry(i1 %flag) {
entry:
  %tmp0 = call i64 @test_select(i1 %flag)
  ret i64 %tmp0
}

define internal i64 @test_select(i1 %flag) {
entry:
  %tmp0 = select i1 %flag, ptr @norecurse_1, ptr @norecurse_2
  store i64 1, ptr @global_scalar
  call void %tmp0()
  %tmp1 = load i64, ptr @global_scalar
  ret i64 %tmp1
}

declare void @norecurse_1() #0
declare void @norecurse_2() #0

attributes #0 = { norecurse }
