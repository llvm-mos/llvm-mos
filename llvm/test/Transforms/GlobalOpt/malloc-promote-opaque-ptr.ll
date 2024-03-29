; NOTE: Assertions have been autogenerated by utils/update_test_checks.py UTC_ARGS: --check-globals
; RUN: opt -S -passes=globalopt < %s | FileCheck %s

@g1 = internal global ptr null
@g2 = internal global ptr null
@g3 = internal global ptr null

declare noalias ptr @malloc(i64) allockind("alloc,uninitialized") allocsize(0)

;.
; CHECK: @[[G1_BODY_0:[a-zA-Z0-9_$"\\.-]+]] = internal unnamed_addr global i64 undef
; CHECK: @[[G2_BODY_0:[a-zA-Z0-9_$"\\.-]+]] = internal unnamed_addr global i32 undef
; CHECK: @[[G2_BODY_1:[a-zA-Z0-9_$"\\.-]+]] = internal unnamed_addr global i32 undef
; CHECK: @[[G2_BODY_2:[a-zA-Z0-9_$"\\.-]+]] = internal unnamed_addr global i32 undef
; CHECK: @[[G3_BODY:[a-zA-Z0-9_$"\\.-]+]] = internal unnamed_addr global [8 x i8] undef
;.
define void @test_store(i64 %a, i32 %b) {
; CHECK-LABEL: @test_store(
; CHECK-NEXT:    store i64 [[A:%.*]], ptr @g1.body.0, align 8
; CHECK-NEXT:    store i32 [[B:%.*]], ptr @g2.body.0, align 4
; CHECK-NEXT:    store i32 [[B]], ptr @g2.body.1, align 4
; CHECK-NEXT:    store i32 [[B]], ptr @g2.body.2, align 4
; CHECK-NEXT:    store i64 [[A]], ptr @g3.body, align 4
; CHECK-NEXT:    store i32 [[B]], ptr @g3.body, align 4
; CHECK-NEXT:    ret void
;
  %m1 = call ptr @malloc(i64 8)
  store ptr %m1, ptr @g1
  %a1 = load ptr, ptr @g1

  %m2 = call ptr @malloc(i64 16)
  store ptr %m2, ptr @g2
  %a2 = load ptr, ptr @g2

  %m3 = call ptr @malloc(i64 8)
  store ptr %m3, ptr @g3
  %a3 = load ptr, ptr @g3

  store i64 %a, ptr %a1

  ; Access types at different offsets.
  store i32 %b, ptr %a2
  %a2.4 = getelementptr i8, ptr %a2, i64 4
  store i32 %b, ptr %a2.4
  %a2.10 = getelementptr i8, ptr %a2, i64 10
  store i32 %b, ptr %a2.10, align 2

  ; Access two different types at the same offset.
  store i64 %a, ptr %a3
  store i32 %b, ptr %a3

  ret void
}

define void @test_load() {
; CHECK-LABEL: @test_load(
; CHECK-NEXT:    [[TMP1:%.*]] = load i64, ptr @g1.body.0, align 8
; CHECK-NEXT:    [[TMP2:%.*]] = load i32, ptr @g2.body.0, align 4
; CHECK-NEXT:    [[TMP3:%.*]] = load i32, ptr @g2.body.1, align 4
; CHECK-NEXT:    [[TMP4:%.*]] = load i32, ptr @g2.body.2, align 4
; CHECK-NEXT:    [[TMP5:%.*]] = load i64, ptr @g3.body, align 4
; CHECK-NEXT:    ret void
;
  %a1 = load ptr, ptr @g1
  load i64, ptr %a1

  %a2 = load ptr, ptr @g2
  load i32, ptr %a2
  %a2.4 = getelementptr i8, ptr %a2, i64 4
  load i32, ptr %a2.4
  %a2.10 = getelementptr i8, ptr %a2, i64 10
  load i32, ptr %a2.10, align 2

  %a3 = load ptr, ptr @g3
  load i64, ptr %a3
  ret void
}
