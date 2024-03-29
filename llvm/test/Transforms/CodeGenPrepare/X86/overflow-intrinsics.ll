; NOTE: Assertions have been autogenerated by utils/update_test_checks.py
; RUN: opt -passes='require<profile-summary>,function(codegenprepare)' -S < %s | FileCheck %s
; RUN: opt -enable-debugify -passes='require<profile-summary>,function(codegenprepare)' -S < %s 2>&1 | FileCheck %s -check-prefix=DEBUG

target datalayout = "e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64"
target triple = "x86_64-apple-darwin10.0.0"

define i64 @uaddo1_overflow_used(i64 %a, i64 %b) nounwind ssp {
; CHECK-LABEL: @uaddo1_overflow_used(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i64, i1 } @llvm.uadd.with.overflow.i64(i64 [[B:%.*]], i64 [[A:%.*]])
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i64, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV:%.*]] = extractvalue { i64, i1 } [[TMP1]], 1
; CHECK-NEXT:    [[Q:%.*]] = select i1 [[OV]], i64 [[B]], i64 42
; CHECK-NEXT:    ret i64 [[Q]]
;
  %add = add i64 %b, %a
  %cmp = icmp ult i64 %add, %a
  %Q = select i1 %cmp, i64 %b, i64 42
  ret i64 %Q
}

define i64 @uaddo1_math_overflow_used(i64 %a, i64 %b, ptr %res) nounwind ssp {
; CHECK-LABEL: @uaddo1_math_overflow_used(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i64, i1 } @llvm.uadd.with.overflow.i64(i64 [[B:%.*]], i64 [[A:%.*]])
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i64, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV:%.*]] = extractvalue { i64, i1 } [[TMP1]], 1
; CHECK-NEXT:    [[Q:%.*]] = select i1 [[OV]], i64 [[B]], i64 42
; CHECK-NEXT:    store i64 [[MATH]], ptr [[RES:%.*]]
; CHECK-NEXT:    ret i64 [[Q]]
;
  %add = add i64 %b, %a
  %cmp = icmp ult i64 %add, %a
  %Q = select i1 %cmp, i64 %b, i64 42
  store i64 %add, ptr %res
  ret i64 %Q
}

define i64 @uaddo2_overflow_used(i64 %a, i64 %b) nounwind ssp {
; CHECK-LABEL: @uaddo2_overflow_used(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i64, i1 } @llvm.uadd.with.overflow.i64(i64 [[B:%.*]], i64 [[A:%.*]])
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i64, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV:%.*]] = extractvalue { i64, i1 } [[TMP1]], 1
; CHECK-NEXT:    [[Q:%.*]] = select i1 [[OV]], i64 [[B]], i64 42
; CHECK-NEXT:    ret i64 [[Q]]
;
  %add = add i64 %b, %a
  %cmp = icmp ult i64 %add, %b
  %Q = select i1 %cmp, i64 %b, i64 42
  ret i64 %Q
}

define i64 @uaddo2_math_overflow_used(i64 %a, i64 %b, ptr %res) nounwind ssp {
; CHECK-LABEL: @uaddo2_math_overflow_used(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i64, i1 } @llvm.uadd.with.overflow.i64(i64 [[B:%.*]], i64 [[A:%.*]])
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i64, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV:%.*]] = extractvalue { i64, i1 } [[TMP1]], 1
; CHECK-NEXT:    [[Q:%.*]] = select i1 [[OV]], i64 [[B]], i64 42
; CHECK-NEXT:    store i64 [[MATH]], ptr [[RES:%.*]]
; CHECK-NEXT:    ret i64 [[Q]]
;
  %add = add i64 %b, %a
  %cmp = icmp ult i64 %add, %b
  %Q = select i1 %cmp, i64 %b, i64 42
  store i64 %add, ptr %res
  ret i64 %Q
}

define i64 @uaddo3_overflow_used(i64 %a, i64 %b) nounwind ssp {
; CHECK-LABEL: @uaddo3_overflow_used(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i64, i1 } @llvm.uadd.with.overflow.i64(i64 [[B:%.*]], i64 [[A:%.*]])
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i64, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV:%.*]] = extractvalue { i64, i1 } [[TMP1]], 1
; CHECK-NEXT:    [[Q:%.*]] = select i1 [[OV]], i64 [[B]], i64 42
; CHECK-NEXT:    ret i64 [[Q]]
;
  %add = add i64 %b, %a
  %cmp = icmp ugt i64 %b, %add
  %Q = select i1 %cmp, i64 %b, i64 42
  ret i64 %Q
}

define i64 @uaddo3_math_overflow_used(i64 %a, i64 %b, ptr %res) nounwind ssp {
; CHECK-LABEL: @uaddo3_math_overflow_used(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i64, i1 } @llvm.uadd.with.overflow.i64(i64 [[B:%.*]], i64 [[A:%.*]])
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i64, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV:%.*]] = extractvalue { i64, i1 } [[TMP1]], 1
; CHECK-NEXT:    [[Q:%.*]] = select i1 [[OV]], i64 [[B]], i64 42
; CHECK-NEXT:    store i64 [[MATH]], ptr [[RES:%.*]]
; CHECK-NEXT:    ret i64 [[Q]]
;
  %add = add i64 %b, %a
  %cmp = icmp ugt i64 %b, %add
  %Q = select i1 %cmp, i64 %b, i64 42
  store i64 %add, ptr %res
  ret i64 %Q
}

; TODO? CGP sinks the compare before we have a chance to form the overflow intrinsic.

define i64 @uaddo4(i64 %a, i64 %b, i1 %c) nounwind ssp {
; CHECK-LABEL: @uaddo4(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[ADD:%.*]] = add i64 [[B:%.*]], [[A:%.*]]
; CHECK-NEXT:    br i1 [[C:%.*]], label [[NEXT:%.*]], label [[EXIT:%.*]]
; CHECK:       next:
; CHECK-NEXT:    [[TMP0:%.*]] = icmp ugt i64 [[B]], [[ADD]]
; CHECK-NEXT:    [[Q:%.*]] = select i1 [[TMP0]], i64 [[B]], i64 42
; CHECK-NEXT:    ret i64 [[Q]]
; CHECK:       exit:
; CHECK-NEXT:    ret i64 0
;
entry:
  %add = add i64 %b, %a
  %cmp = icmp ugt i64 %b, %add
  br i1 %c, label %next, label %exit

next:
  %Q = select i1 %cmp, i64 %b, i64 42
  ret i64 %Q

exit:
  ret i64 0
}

define i64 @uaddo5(i64 %a, i64 %b, ptr %ptr, i1 %c) nounwind ssp {
; CHECK-LABEL: @uaddo5(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[ADD:%.*]] = add i64 [[B:%.*]], [[A:%.*]]
; CHECK-NEXT:    store i64 [[ADD]], ptr [[PTR:%.*]]
; CHECK-NEXT:    br i1 [[C:%.*]], label [[NEXT:%.*]], label [[EXIT:%.*]]
; CHECK:       next:
; CHECK-NEXT:    [[TMP0:%.*]] = icmp ugt i64 [[B]], [[ADD]]
; CHECK-NEXT:    [[Q:%.*]] = select i1 [[TMP0]], i64 [[B]], i64 42
; CHECK-NEXT:    ret i64 [[Q]]
; CHECK:       exit:
; CHECK-NEXT:    ret i64 0
;
entry:
  %add = add i64 %b, %a
  store i64 %add, ptr %ptr
  %cmp = icmp ugt i64 %b, %add
  br i1 %c, label %next, label %exit

next:
  %Q = select i1 %cmp, i64 %b, i64 42
  ret i64 %Q

exit:
  ret i64 0
}

; Instcombine folds (a + b <u a)  to (a ^ -1 <u b). Make sure we match this
; pattern as well.
define i64 @uaddo6_xor(i64 %a, i64 %b) {
; CHECK-LABEL: @uaddo6_xor(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i64, i1 } @llvm.uadd.with.overflow.i64(i64 [[A:%.*]], i64 [[B:%.*]])
; CHECK-NEXT:    [[OV:%.*]] = extractvalue { i64, i1 } [[TMP1]], 1
; CHECK-NEXT:    [[Q:%.*]] = select i1 [[OV]], i64 [[B]], i64 42
; CHECK-NEXT:    ret i64 [[Q]]
;
  %x = xor i64 %a, -1
  %cmp = icmp ult i64 %x, %b
  %Q = select i1 %cmp, i64 %b, i64 42
  ret i64 %Q
}

define i64 @uaddo6_xor_commuted(i64 %a, i64 %b) {
; CHECK-LABEL: @uaddo6_xor_commuted(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i64, i1 } @llvm.uadd.with.overflow.i64(i64 [[A:%.*]], i64 [[B:%.*]])
; CHECK-NEXT:    [[OV:%.*]] = extractvalue { i64, i1 } [[TMP1]], 1
; CHECK-NEXT:    [[Q:%.*]] = select i1 [[OV]], i64 [[B]], i64 42
; CHECK-NEXT:    ret i64 [[Q]]
;
  %x = xor i64 %a, -1
  %cmp = icmp ult i64 %x, %b
  %Q = select i1 %cmp, i64 %b, i64 42
  ret i64 %Q
}

declare void @use(i64)

define i64 @uaddo6_xor_multi_use(i64 %a, i64 %b) {
; CHECK-LABEL: @uaddo6_xor_multi_use(
; CHECK-NEXT:    [[X:%.*]] = xor i64 -1, [[A:%.*]]
; CHECK-NEXT:    [[CMP:%.*]] = icmp ult i64 [[X]], [[B:%.*]]
; CHECK-NEXT:    [[Q:%.*]] = select i1 [[CMP]], i64 [[B]], i64 42
; CHECK-NEXT:    call void @use(i64 [[X]])
; CHECK-NEXT:    ret i64 [[Q]]
;
  %x = xor i64 -1, %a
  %cmp = icmp ult i64 %x, %b
  %Q = select i1 %cmp, i64 %b, i64 42
  call void @use(i64 %x)
  ret i64 %Q
}

; Make sure we do not use the XOR binary operator as insert point, as it may
; come before the second operand of the overflow intrinsic.
define i1 @uaddo6_xor_op_after_XOR(i32 %a, ptr %b.ptr) {
; CHECK-LABEL: @uaddo6_xor_op_after_XOR(
; CHECK-NEXT:    [[B:%.*]] = load i32, ptr [[B_PTR:%.*]], align 8
; CHECK-NEXT:    [[TMP1:%.*]] = call { i32, i1 } @llvm.uadd.with.overflow.i32(i32 [[A:%.*]], i32 [[B]])
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i32, i1 } [[TMP1]], 1
; CHECK-NEXT:    [[OV:%.*]] = xor i1 [[OV1]], true
; CHECK-NEXT:    ret i1 [[OV]]
;
  %x = xor i32 %a, -1
  %b = load i32, ptr %b.ptr, align 8
  %cmp14 = icmp ugt i32 %b, %x
  %ov = xor i1 %cmp14, true
  ret i1 %ov
}

; When adding 1, the general pattern for add-overflow may be different due to icmp canonicalization.
; PR31754: https://bugs.llvm.org/show_bug.cgi?id=31754

define i1 @uaddo_i64_increment(i64 %x, ptr %p) {
; CHECK-LABEL: @uaddo_i64_increment(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i64, i1 } @llvm.uadd.with.overflow.i64(i64 [[X:%.*]], i64 1)
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i64, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i64, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i64 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %a = add i64 %x, 1
  %ov = icmp eq i64 %a, 0
  store i64 %a, ptr %p
  ret i1 %ov
}

define i1 @uaddo_i8_increment_noncanonical_1(i8 %x, ptr %p) {
; CHECK-LABEL: @uaddo_i8_increment_noncanonical_1(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i8, i1 } @llvm.uadd.with.overflow.i8(i8 1, i8 [[X:%.*]])
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i8, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i8, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i8 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %a = add i8 1, %x        ; commute
  %ov = icmp eq i8 %a, 0
  store i8 %a, ptr %p
  ret i1 %ov
}

define i1 @uaddo_i32_increment_noncanonical_2(i32 %x, ptr %p) {
; CHECK-LABEL: @uaddo_i32_increment_noncanonical_2(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i32, i1 } @llvm.uadd.with.overflow.i32(i32 [[X:%.*]], i32 1)
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i32, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i32, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i32 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %a = add i32 %x, 1
  %ov = icmp eq i32 0, %a   ; commute
  store i32 %a, ptr %p
  ret i1 %ov
}

define i1 @uaddo_i16_increment_noncanonical_3(i16 %x, ptr %p) {
; CHECK-LABEL: @uaddo_i16_increment_noncanonical_3(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i16, i1 } @llvm.uadd.with.overflow.i16(i16 1, i16 [[X:%.*]])
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i16, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i16, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i16 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %a = add i16 1, %x        ; commute
  %ov = icmp eq i16 0, %a   ; commute
  store i16 %a, ptr %p
  ret i1 %ov
}

; The overflow check may be against the input rather than the sum.

define i1 @uaddo_i64_increment_alt(i64 %x, ptr %p) {
; CHECK-LABEL: @uaddo_i64_increment_alt(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i64, i1 } @llvm.uadd.with.overflow.i64(i64 [[X:%.*]], i64 1)
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i64, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i64, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i64 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %a = add i64 %x, 1
  store i64 %a, ptr %p
  %ov = icmp eq i64 %x, -1
  ret i1 %ov
}

; Make sure insertion is done correctly based on dominance.

define i1 @uaddo_i64_increment_alt_dom(i64 %x, ptr %p) {
; CHECK-LABEL: @uaddo_i64_increment_alt_dom(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i64, i1 } @llvm.uadd.with.overflow.i64(i64 [[X:%.*]], i64 1)
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i64, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i64, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i64 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %ov = icmp eq i64 %x, -1
  %a = add i64 %x, 1
  store i64 %a, ptr %p
  ret i1 %ov
}

; The overflow check may be against the input rather than the sum.

define i1 @uaddo_i64_decrement_alt(i64 %x, ptr %p) {
; CHECK-LABEL: @uaddo_i64_decrement_alt(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i64, i1 } @llvm.uadd.with.overflow.i64(i64 [[X:%.*]], i64 -1)
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i64, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i64, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i64 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %a = add i64 %x, -1
  store i64 %a, ptr %p
  %ov = icmp ne i64 %x, 0
  ret i1 %ov
}

; Make sure insertion is done correctly based on dominance.

define i1 @uaddo_i64_decrement_alt_dom(i64 %x, ptr %p) {
; CHECK-LABEL: @uaddo_i64_decrement_alt_dom(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i64, i1 } @llvm.uadd.with.overflow.i64(i64 [[X:%.*]], i64 -1)
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i64, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i64, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i64 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %ov = icmp ne i64 %x, 0
  %a = add i64 %x, -1
  store i64 %a, ptr %p
  ret i1 %ov
}

; No transform for illegal types.

define i1 @uaddo_i42_increment_illegal_type(i42 %x, ptr %p) {
; CHECK-LABEL: @uaddo_i42_increment_illegal_type(
; CHECK-NEXT:    [[A:%.*]] = add i42 [[X:%.*]], 1
; CHECK-NEXT:    [[OV:%.*]] = icmp eq i42 [[A]], 0
; CHECK-NEXT:    store i42 [[A]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV]]
;
  %a = add i42 %x, 1
  %ov = icmp eq i42 %a, 0
  store i42 %a, ptr %p
  ret i1 %ov
}

define i1 @usubo_ult_i64_overflow_used(i64 %x, i64 %y, ptr %p) {
; CHECK-LABEL: @usubo_ult_i64_overflow_used(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i64, i1 } @llvm.usub.with.overflow.i64(i64 [[X:%.*]], i64 [[Y:%.*]])
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i64, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i64, i1 } [[TMP1]], 1
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %s = sub i64 %x, %y
  %ov = icmp ult i64 %x, %y
  ret i1 %ov
}

define i1 @usubo_ult_i64_math_overflow_used(i64 %x, i64 %y, ptr %p) {
; CHECK-LABEL: @usubo_ult_i64_math_overflow_used(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i64, i1 } @llvm.usub.with.overflow.i64(i64 [[X:%.*]], i64 [[Y:%.*]])
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i64, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i64, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i64 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %s = sub i64 %x, %y
  store i64 %s, ptr %p
  %ov = icmp ult i64 %x, %y
  ret i1 %ov
}

; Verify insertion point for single-BB. Toggle predicate.

define i1 @usubo_ugt_i32(i32 %x, i32 %y, ptr %p) {
; CHECK-LABEL: @usubo_ugt_i32(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i32, i1 } @llvm.usub.with.overflow.i32(i32 [[X:%.*]], i32 [[Y:%.*]])
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i32, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i32, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i32 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %ov = icmp ugt i32 %y, %x
  %s = sub i32 %x, %y
  store i32 %s, ptr %p
  ret i1 %ov
}

; Constant operand should match.

define i1 @usubo_ugt_constant_op0_i8(i8 %x, ptr %p) {
; CHECK-LABEL: @usubo_ugt_constant_op0_i8(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i8, i1 } @llvm.usub.with.overflow.i8(i8 42, i8 [[X:%.*]])
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i8, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i8, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i8 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %s = sub i8 42, %x
  %ov = icmp ugt i8 %x, 42
  store i8 %s, ptr %p
  ret i1 %ov
}

; Compare with constant operand 0 is canonicalized by commuting, but verify match for non-canonical form.

define i1 @usubo_ult_constant_op0_i16(i16 %x, ptr %p) {
; CHECK-LABEL: @usubo_ult_constant_op0_i16(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i16, i1 } @llvm.usub.with.overflow.i16(i16 43, i16 [[X:%.*]])
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i16, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i16, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i16 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %s = sub i16 43, %x
  %ov = icmp ult i16 43, %x
  store i16 %s, ptr %p
  ret i1 %ov
}

; Subtract with constant operand 1 is canonicalized to add.

define i1 @usubo_ult_constant_op1_i16(i16 %x, ptr %p) {
; CHECK-LABEL: @usubo_ult_constant_op1_i16(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i16, i1 } @llvm.usub.with.overflow.i16(i16 [[X:%.*]], i16 44)
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i16, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i16, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i16 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %s = add i16 %x, -44
  %ov = icmp ult i16 %x, 44
  store i16 %s, ptr %p
  ret i1 %ov
}

define i1 @usubo_ugt_constant_op1_i8(i8 %x, ptr %p) {
; CHECK-LABEL: @usubo_ugt_constant_op1_i8(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i8, i1 } @llvm.usub.with.overflow.i8(i8 [[X:%.*]], i8 45)
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i8, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i8, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i8 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %ov = icmp ugt i8 45, %x
  %s = add i8 %x, -45
  store i8 %s, ptr %p
  ret i1 %ov
}

; Special-case: subtract 1 changes the compare predicate and constant.

define i1 @usubo_eq_constant1_op1_i32(i32 %x, ptr %p) {
; CHECK-LABEL: @usubo_eq_constant1_op1_i32(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i32, i1 } @llvm.usub.with.overflow.i32(i32 [[X:%.*]], i32 1)
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i32, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i32, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i32 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %s = add i32 %x, -1
  %ov = icmp eq i32 %x, 0
  store i32 %s, ptr %p
  ret i1 %ov
}

; Special-case: subtract from 0 (negate) changes the compare predicate.

define i1 @usubo_ne_constant0_op1_i32(i32 %x, ptr %p) {
; CHECK-LABEL: @usubo_ne_constant0_op1_i32(
; CHECK-NEXT:    [[TMP1:%.*]] = call { i32, i1 } @llvm.usub.with.overflow.i32(i32 0, i32 [[X:%.*]])
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i32, i1 } [[TMP1]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i32, i1 } [[TMP1]], 1
; CHECK-NEXT:    store i32 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
  %s = sub i32 0, %x
  %ov = icmp ne i32 %x, 0
  store i32 %s, ptr %p
  ret i1 %ov
}

; This used to verify insertion point for multi-BB, but now we just bail out.

declare void @call(i1)

define i1 @usubo_ult_sub_dominates_i64(i64 %x, i64 %y, ptr %p, i1 %cond) {
; CHECK-LABEL: @usubo_ult_sub_dominates_i64(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    br i1 [[COND:%.*]], label [[T:%.*]], label [[F:%.*]]
; CHECK:       t:
; CHECK-NEXT:    [[S:%.*]] = sub i64 [[X:%.*]], [[Y:%.*]]
; CHECK-NEXT:    store i64 [[S]], ptr [[P:%.*]]
; CHECK-NEXT:    br i1 [[COND]], label [[END:%.*]], label [[F]]
; CHECK:       f:
; CHECK-NEXT:    ret i1 [[COND]]
; CHECK:       end:
; CHECK-NEXT:    [[OV:%.*]] = icmp ult i64 [[X]], [[Y]]
; CHECK-NEXT:    ret i1 [[OV]]
;
entry:
  br i1 %cond, label %t, label %f

t:
  %s = sub i64 %x, %y
  store i64 %s, ptr %p
  br i1 %cond, label %end, label %f

f:
  ret i1 %cond

end:
  %ov = icmp ult i64 %x, %y
  ret i1 %ov
}

define i1 @usubo_ult_cmp_dominates_i64(i64 %x, i64 %y, ptr %p, i1 %cond) {
; CHECK-LABEL: @usubo_ult_cmp_dominates_i64(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    br i1 [[COND:%.*]], label [[T:%.*]], label [[F:%.*]]
; CHECK:       t:
; CHECK-NEXT:    [[OV:%.*]] = icmp ult i64 [[X:%.*]], [[Y:%.*]]
; CHECK-NEXT:    call void @call(i1 [[OV]])
; CHECK-NEXT:    br i1 [[OV]], label [[END:%.*]], label [[F]]
; CHECK:       f:
; CHECK-NEXT:    ret i1 [[COND]]
; CHECK:       end:
; CHECK-NEXT:    [[TMP0:%.*]] = call { i64, i1 } @llvm.usub.with.overflow.i64(i64 [[X]], i64 [[Y]])
; CHECK-NEXT:    [[MATH:%.*]] = extractvalue { i64, i1 } [[TMP0]], 0
; CHECK-NEXT:    [[OV1:%.*]] = extractvalue { i64, i1 } [[TMP0]], 1
; CHECK-NEXT:    store i64 [[MATH]], ptr [[P:%.*]]
; CHECK-NEXT:    ret i1 [[OV1]]
;
entry:
  br i1 %cond, label %t, label %f

t:
  %ov = icmp ult i64 %x, %y
  call void @call(i1 %ov)
  br i1 %ov, label %end, label %f

f:
  ret i1 %cond

end:
  %s = sub i64 %x, %y
  store i64 %s, ptr %p
  ret i1 %ov
}

; Verify that crazy/non-canonical code does not crash.

define void @bar() {
; CHECK-LABEL: @bar(
; CHECK-NEXT:    [[CMP:%.*]] = icmp eq i64 1, -1
; CHECK-NEXT:    [[FROMBOOL:%.*]] = zext i1 [[CMP]] to i8
; CHECK-NEXT:    unreachable
;
  %cmp = icmp eq i64 1, -1
  %frombool = zext i1 %cmp to i8
  unreachable
}

define void @foo() {
; CHECK-LABEL: @foo(
; CHECK-NEXT:    [[SUB:%.*]] = add nsw i64 1, 1
; CHECK-NEXT:    [[CONV:%.*]] = trunc i64 [[SUB]] to i32
; CHECK-NEXT:    unreachable
;
  %sub = add nsw i64 1, 1
  %conv = trunc i64 %sub to i32
  unreachable
}

; Similarly for usubo.

define i1 @bar2() {
; CHECK-LABEL: @bar2(
; CHECK-NEXT:    [[CMP:%.*]] = icmp eq i64 1, 0
; CHECK-NEXT:    ret i1 [[CMP]]
;
  %cmp = icmp eq i64 1, 0
  ret i1 %cmp
}

define i64 @foo2(ptr %p) {
; CHECK-LABEL: @foo2(
; CHECK-NEXT:    [[SUB:%.*]] = add nsw i64 1, -1
; CHECK-NEXT:    ret i64 [[SUB]]
;
  %sub = add nsw i64 1, -1
  ret i64 %sub
}

; Avoid hoisting a math op into a dominating block which would
; increase the critical path.

define void @PR41129(ptr %p64) {
; CHECK-LABEL: @PR41129(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[KEY:%.*]] = load i64, ptr [[P64:%.*]], align 8
; CHECK-NEXT:    [[COND17:%.*]] = icmp eq i64 [[KEY]], 0
; CHECK-NEXT:    br i1 [[COND17]], label [[TRUE:%.*]], label [[FALSE:%.*]]
; CHECK:       false:
; CHECK-NEXT:    [[ANDVAL:%.*]] = and i64 [[KEY]], 7
; CHECK-NEXT:    store i64 [[ANDVAL]], ptr [[P64]]
; CHECK-NEXT:    br label [[EXIT:%.*]]
; CHECK:       true:
; CHECK-NEXT:    [[SVALUE:%.*]] = add i64 [[KEY]], -1
; CHECK-NEXT:    store i64 [[SVALUE]], ptr [[P64]]
; CHECK-NEXT:    br label [[EXIT]]
; CHECK:       exit:
; CHECK-NEXT:    ret void
;
entry:
  %key = load i64, ptr %p64, align 8
  %cond17 = icmp eq i64 %key, 0
  br i1 %cond17, label %true, label %false

false:
  %andval = and i64 %key, 7
  store i64 %andval, ptr %p64
  br label %exit

true:
  %svalue = add i64 %key, -1
  store i64 %svalue, ptr %p64
  br label %exit

exit:
  ret void
}

; Check that every instruction inserted by -passes='require<profile-summary>,function(codegenprepare)' has a debug location.
; DEBUG: CheckModuleDebugify: PASS

