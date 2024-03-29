; NOTE: Assertions have been autogenerated by utils/update_test_checks.py
; RUN: opt -mtriple=arm -passes=typepromotion,verify  -S %s -o - | FileCheck %s

define zeroext i16 @overflow_add(i16 zeroext %a, i16 zeroext %b) {
; CHECK-LABEL: @overflow_add(
; CHECK-NEXT:    [[ADD:%.*]] = add i16 [[A:%.*]], [[B:%.*]]
; CHECK-NEXT:    [[OR:%.*]] = or i16 [[ADD]], 1
; CHECK-NEXT:    [[CMP:%.*]] = icmp ugt i16 [[OR]], 1024
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP]], i16 2, i16 5
; CHECK-NEXT:    ret i16 [[RES]]
;
  %add = add i16 %a, %b
  %or = or i16 %add, 1
  %cmp = icmp ugt i16 %or, 1024
  %res = select i1 %cmp, i16 2, i16 5
  ret i16 %res
}

define zeroext i16 @overflow_sub(i16 zeroext %a, i16 zeroext %b) {
; CHECK-LABEL: @overflow_sub(
; CHECK-NEXT:    [[ADD:%.*]] = sub i16 [[A:%.*]], [[B:%.*]]
; CHECK-NEXT:    [[OR:%.*]] = or i16 [[ADD]], 1
; CHECK-NEXT:    [[CMP:%.*]] = icmp ugt i16 [[OR]], 1024
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP]], i16 2, i16 5
; CHECK-NEXT:    ret i16 [[RES]]
;
  %add = sub i16 %a, %b
  %or = or i16 %add, 1
  %cmp = icmp ugt i16 %or, 1024
  %res = select i1 %cmp, i16 2, i16 5
  ret i16 %res
}

define zeroext i16 @overflow_mul(i16 zeroext %a, i16 zeroext %b) {
; CHECK-LABEL: @overflow_mul(
; CHECK-NEXT:    [[ADD:%.*]] = mul i16 [[A:%.*]], [[B:%.*]]
; CHECK-NEXT:    [[OR:%.*]] = or i16 [[ADD]], 1
; CHECK-NEXT:    [[CMP:%.*]] = icmp ugt i16 [[OR]], 1024
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP]], i16 2, i16 5
; CHECK-NEXT:    ret i16 [[RES]]
;
  %add = mul i16 %a, %b
  %or = or i16 %add, 1
  %cmp = icmp ugt i16 %or, 1024
  %res = select i1 %cmp, i16 2, i16 5
  ret i16 %res
}

define zeroext i16 @overflow_shl(i16 zeroext %a, i16 zeroext %b) {
; CHECK-LABEL: @overflow_shl(
; CHECK-NEXT:    [[ADD:%.*]] = shl i16 [[A:%.*]], [[B:%.*]]
; CHECK-NEXT:    [[OR:%.*]] = or i16 [[ADD]], 1
; CHECK-NEXT:    [[CMP:%.*]] = icmp ugt i16 [[OR]], 1024
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP]], i16 2, i16 5
; CHECK-NEXT:    ret i16 [[RES]]
;
  %add = shl i16 %a, %b
  %or = or i16 %add, 1
  %cmp = icmp ugt i16 %or, 1024
  %res = select i1 %cmp, i16 2, i16 5
  ret i16 %res
}

define i32 @overflow_add_no_consts(i8 zeroext %a, i8 zeroext %b, i8 zeroext %limit) {
; CHECK-LABEL: @overflow_add_no_consts(
; CHECK-NEXT:    [[ADD:%.*]] = add i8 [[A:%.*]], [[B:%.*]]
; CHECK-NEXT:    [[CMP:%.*]] = icmp ugt i8 [[ADD]], [[LIMIT:%.*]]
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP]], i32 8, i32 16
; CHECK-NEXT:    ret i32 [[RES]]
;
  %add = add i8 %a, %b
  %cmp = icmp ugt i8 %add, %limit
  %res = select i1 %cmp, i32 8, i32 16
  ret i32 %res
}

define i32 @overflow_add_const_limit(i8 zeroext %a, i8 zeroext %b) {
; CHECK-LABEL: @overflow_add_const_limit(
; CHECK-NEXT:    [[ADD:%.*]] = add i8 [[A:%.*]], [[B:%.*]]
; CHECK-NEXT:    [[CMP:%.*]] = icmp ugt i8 [[ADD]], -128
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP]], i32 8, i32 16
; CHECK-NEXT:    ret i32 [[RES]]
;
  %add = add i8 %a, %b
  %cmp = icmp ugt i8 %add, 128
  %res = select i1 %cmp, i32 8, i32 16
  ret i32 %res
}

define i32 @overflow_add_positive_const_limit(i8 zeroext %a) {
; CHECK-LABEL: @overflow_add_positive_const_limit(
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[A:%.*]] to i32
; CHECK-NEXT:    [[ADD:%.*]] = add i32 [[TMP1]], -255
; CHECK-NEXT:    [[CMP:%.*]] = icmp ugt i32 [[ADD]], -128
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP]], i32 8, i32 16
; CHECK-NEXT:    ret i32 [[RES]]
;
  %add = add i8 %a, 1
  %cmp = icmp ugt i8 %add, 128
  %res = select i1 %cmp, i32 8, i32 16
  ret i32 %res
}

define i32 @unsafe_add_underflow(i8 zeroext %a) {
; CHECK-LABEL: @unsafe_add_underflow(
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[A:%.*]] to i32
; CHECK-NEXT:    [[ADD:%.*]] = add i32 [[TMP1]], -2
; CHECK-NEXT:    [[CMP:%.*]] = icmp ugt i32 [[ADD]], -2
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP]], i32 8, i32 16
; CHECK-NEXT:    ret i32 [[RES]]
;
  %add = add i8 %a, -2
  %cmp = icmp ugt i8 %add, 254
  %res = select i1 %cmp, i32 8, i32 16
  ret i32 %res
}

define i32 @safe_add_underflow(i8 zeroext %a) {
; CHECK-LABEL: @safe_add_underflow(
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[A:%.*]] to i32
; CHECK-NEXT:    [[ADD:%.*]] = add i32 [[TMP1]], -1
; CHECK-NEXT:    [[CMP:%.*]] = icmp ugt i32 [[ADD]], 254
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP]], i32 8, i32 16
; CHECK-NEXT:    ret i32 [[RES]]
;
  %add = add i8 %a, -1
  %cmp = icmp ugt i8 %add, 254
  %res = select i1 %cmp, i32 8, i32 16
  ret i32 %res
}

define i32 @safe_add_underflow_neg(i8 zeroext %a) {
; CHECK-LABEL: @safe_add_underflow_neg(
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[A:%.*]] to i32
; CHECK-NEXT:    [[ADD:%.*]] = add i32 [[TMP1]], -2
; CHECK-NEXT:    [[CMP:%.*]] = icmp ule i32 [[ADD]], 250
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP]], i32 8, i32 16
; CHECK-NEXT:    ret i32 [[RES]]
;
  %add = add i8 %a, -2
  %cmp = icmp ule i8 %add, -6
  %res = select i1 %cmp, i32 8, i32 16
  ret i32 %res
}

define i32 @overflow_sub_negative_const_limit(i8 zeroext %a) {
; CHECK-LABEL: @overflow_sub_negative_const_limit(
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[A:%.*]] to i32
; CHECK-NEXT:    [[SUB:%.*]] = sub i32 [[TMP1]], 255
; CHECK-NEXT:    [[CMP:%.*]] = icmp ugt i32 [[SUB]], -128
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP]], i32 8, i32 16
; CHECK-NEXT:    ret i32 [[RES]]
;
  %sub = sub i8 %a, -1
  %cmp = icmp ugt i8 %sub, 128
  %res = select i1 %cmp, i32 8, i32 16
  ret i32 %res
}

; This is valid so long as the icmp immediate is sext.
define i32 @sext_sub_underflow(i8 zeroext %a) {
; CHECK-LABEL: @sext_sub_underflow(
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[A:%.*]] to i32
; CHECK-NEXT:    [[SUB:%.*]] = sub i32 [[TMP1]], 6
; CHECK-NEXT:    [[CMP:%.*]] = icmp ugt i32 [[SUB]], -6
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP]], i32 8, i32 16
; CHECK-NEXT:    ret i32 [[RES]]
;
  %sub = sub i8 %a, 6
  %cmp = icmp ugt i8 %sub, 250
  %res = select i1 %cmp, i32 8, i32 16
  ret i32 %res
}

define i32 @safe_sub_underflow(i8 zeroext %a) {
; CHECK-LABEL: @safe_sub_underflow(
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[A:%.*]] to i32
; CHECK-NEXT:    [[SUB:%.*]] = sub i32 [[TMP1]], 1
; CHECK-NEXT:    [[CMP:%.*]] = icmp ule i32 [[SUB]], 254
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP]], i32 8, i32 16
; CHECK-NEXT:    ret i32 [[RES]]
;
  %sub = sub i8 %a, 1
  %cmp = icmp ule i8 %sub, 254
  %res = select i1 %cmp, i32 8, i32 16
  ret i32 %res
}

define i32 @safe_sub_underflow_neg(i8 zeroext %a) {
; CHECK-LABEL: @safe_sub_underflow_neg(
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[A:%.*]] to i32
; CHECK-NEXT:    [[SUB:%.*]] = sub i32 [[TMP1]], 4
; CHECK-NEXT:    [[CMP:%.*]] = icmp uge i32 [[SUB]], 251
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP]], i32 8, i32 16
; CHECK-NEXT:    ret i32 [[RES]]
;
  %sub = sub i8 %a, 4
  %cmp = icmp uge i8 %sub, -5
  %res = select i1 %cmp, i32 8, i32 16
  ret i32 %res
}

; This is valid so long as the icmp immediate is sext.
define i32 @sext_sub_underflow_neg(i8 zeroext %a) {
; CHECK-LABEL: @sext_sub_underflow_neg(
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[A:%.*]] to i32
; CHECK-NEXT:    [[SUB:%.*]] = sub i32 [[TMP1]], 4
; CHECK-NEXT:    [[CMP:%.*]] = icmp ult i32 [[SUB]], -3
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP]], i32 8, i32 16
; CHECK-NEXT:    ret i32 [[RES]]
;
  %sub = sub i8 %a, 4
  %cmp = icmp ult i8 %sub, -3
  %res = select i1 %cmp, i32 8, i32 16
  ret i32 %res
}

define i32 @safe_sub_imm_var(ptr %b) {
; CHECK-LABEL: @safe_sub_imm_var(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[TMP0:%.*]] = load i8, ptr [[B:%.*]], align 1
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[TMP0]] to i32
; CHECK-NEXT:    [[SUB:%.*]] = sub nuw nsw i32 248, [[TMP1]]
; CHECK-NEXT:    [[CMP:%.*]] = icmp ugt i32 [[SUB]], 252
; CHECK-NEXT:    [[CONV4:%.*]] = zext i1 [[CMP]] to i32
; CHECK-NEXT:    ret i32 [[CONV4]]
;
entry:
  %0 = load i8, ptr %b, align 1
  %sub = sub nuw nsw i8 -8, %0
  %cmp = icmp ugt i8 %sub, 252
  %conv4 = zext i1 %cmp to i32
  ret i32 %conv4
}

define i32 @safe_sub_var_imm(ptr %b) {
; CHECK-LABEL: @safe_sub_var_imm(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[TMP0:%.*]] = load i8, ptr [[B:%.*]], align 1
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[TMP0]] to i32
; CHECK-NEXT:    [[SUB:%.*]] = sub nuw nsw i32 [[TMP1]], 248
; CHECK-NEXT:    [[CMP:%.*]] = icmp ugt i32 [[SUB]], 252
; CHECK-NEXT:    [[CONV4:%.*]] = zext i1 [[CMP]] to i32
; CHECK-NEXT:    ret i32 [[CONV4]]
;
entry:
  %0 = load i8, ptr %b, align 1
  %sub = sub nuw nsw i8 %0, -8
  %cmp = icmp ugt i8 %sub, 252
  %conv4 = zext i1 %cmp to i32
  ret i32 %conv4
}

define i32 @safe_add_imm_var(ptr %b) {
; CHECK-LABEL: @safe_add_imm_var(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[TMP0:%.*]] = load i8, ptr [[B:%.*]], align 1
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[TMP0]] to i32
; CHECK-NEXT:    [[ADD:%.*]] = add nuw nsw i32 129, [[TMP1]]
; CHECK-NEXT:    [[CMP:%.*]] = icmp ugt i32 [[ADD]], 127
; CHECK-NEXT:    [[CONV4:%.*]] = zext i1 [[CMP]] to i32
; CHECK-NEXT:    ret i32 [[CONV4]]
;
entry:
  %0 = load i8, ptr %b, align 1
  %add = add nuw nsw i8 -127, %0
  %cmp = icmp ugt i8 %add, 127
  %conv4 = zext i1 %cmp to i32
  ret i32 %conv4
}

define i32 @safe_add_var_imm(ptr %b) {
; CHECK-LABEL: @safe_add_var_imm(
; CHECK-NEXT:  entry:
; CHECK-NEXT:    [[TMP0:%.*]] = load i8, ptr [[B:%.*]], align 1
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[TMP0]] to i32
; CHECK-NEXT:    [[ADD:%.*]] = add nuw nsw i32 [[TMP1]], 129
; CHECK-NEXT:    [[CMP:%.*]] = icmp ugt i32 [[ADD]], 127
; CHECK-NEXT:    [[CONV4:%.*]] = zext i1 [[CMP]] to i32
; CHECK-NEXT:    ret i32 [[CONV4]]
;
entry:
  %0 = load i8, ptr %b, align 1
  %add = add nuw nsw i8 %0, -127
  %cmp = icmp ugt i8 %add, 127
  %conv4 = zext i1 %cmp to i32
  ret i32 %conv4
}

define i8 @convert_add_order(i8 zeroext %arg) {
; CHECK-LABEL: @convert_add_order(
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[ARG:%.*]] to i32
; CHECK-NEXT:    [[MASK_0:%.*]] = and i32 [[TMP1]], 1
; CHECK-NEXT:    [[MASK_1:%.*]] = and i32 [[TMP1]], 2
; CHECK-NEXT:    [[SHL:%.*]] = or i32 [[TMP1]], 1
; CHECK-NEXT:    [[ADD:%.*]] = add nuw i32 [[SHL]], 10
; CHECK-NEXT:    [[CMP_0:%.*]] = icmp ult i32 [[ADD]], 60
; CHECK-NEXT:    [[SUB:%.*]] = add nsw i32 [[SHL]], -40
; CHECK-NEXT:    [[CMP_1:%.*]] = icmp ult i32 [[SUB]], 20
; CHECK-NEXT:    [[MASK_SEL:%.*]] = select i1 [[CMP_1]], i32 [[MASK_0]], i32 [[MASK_1]]
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP_0]], i32 [[MASK_SEL]], i32 [[TMP1]]
; CHECK-NEXT:    [[TMP2:%.*]] = trunc i32 [[RES]] to i8
; CHECK-NEXT:    ret i8 [[TMP2]]
;
  %mask.0 = and i8 %arg, 1
  %mask.1 = and i8 %arg, 2
  %shl = or i8 %arg, 1
  %add = add nuw i8 %shl, 10
  %cmp.0 = icmp ult i8 %add, 60
  %sub = add nsw i8 %shl, -40
  %cmp.1 = icmp ult i8 %sub, 20
  %mask.sel = select i1 %cmp.1, i8 %mask.0, i8 %mask.1
  %res = select i1 %cmp.0, i8 %mask.sel, i8 %arg
  ret i8 %res
}

define i8 @underflow_if_sub(i32 %arg, i8 zeroext %arg1) {
; CHECK-LABEL: @underflow_if_sub(
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[ARG1:%.*]] to i32
; CHECK-NEXT:    [[CMP:%.*]] = icmp sgt i32 [[ARG:%.*]], 0
; CHECK-NEXT:    [[CONV:%.*]] = zext i1 [[CMP]] to i32
; CHECK-NEXT:    [[AND:%.*]] = and i32 [[ARG]], [[CONV]]
; CHECK-NEXT:    [[TRUNC:%.*]] = trunc i32 [[AND]] to i8
; CHECK-NEXT:    [[TMP2:%.*]] = zext i8 [[TRUNC]] to i32
; CHECK-NEXT:    [[CONV1:%.*]] = add nuw nsw i32 [[TMP2]], 245
; CHECK-NEXT:    [[CMP_1:%.*]] = icmp ult i32 [[CONV1]], [[TMP1]]
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP_1]], i32 [[CONV1]], i32 100
; CHECK-NEXT:    [[TMP3:%.*]] = trunc i32 [[RES]] to i8
; CHECK-NEXT:    ret i8 [[TMP3]]
;
  %cmp = icmp sgt i32 %arg, 0
  %conv = zext i1 %cmp to i32
  %and = and i32 %arg, %conv
  %trunc = trunc i32 %and to i8
  %conv1 = add nuw nsw i8 %trunc, -11
  %cmp.1 = icmp ult i8 %conv1, %arg1
  %res = select i1 %cmp.1, i8 %conv1, i8 100
  ret i8 %res
}

define i8 @underflow_if_sub_signext(i32 %arg, i8 signext %arg1) {
; CHECK-LABEL: @underflow_if_sub_signext(
; CHECK-NEXT:    [[TMP1:%.*]] = zext i8 [[ARG1:%.*]] to i32
; CHECK-NEXT:    [[CMP:%.*]] = icmp sgt i32 [[ARG:%.*]], 0
; CHECK-NEXT:    [[CONV:%.*]] = zext i1 [[CMP]] to i32
; CHECK-NEXT:    [[AND:%.*]] = and i32 [[ARG]], [[CONV]]
; CHECK-NEXT:    [[TRUNC:%.*]] = trunc i32 [[AND]] to i8
; CHECK-NEXT:    [[TMP2:%.*]] = zext i8 [[TRUNC]] to i32
; CHECK-NEXT:    [[CONV1:%.*]] = add nuw nsw i32 [[TMP2]], 245
; CHECK-NEXT:    [[CMP_1:%.*]] = icmp ugt i32 [[TMP1]], [[CONV1]]
; CHECK-NEXT:    [[RES:%.*]] = select i1 [[CMP_1]], i32 [[CONV1]], i32 100
; CHECK-NEXT:    [[TMP3:%.*]] = trunc i32 [[RES]] to i8
; CHECK-NEXT:    ret i8 [[TMP3]]
;
  %cmp = icmp sgt i32 %arg, 0
  %conv = zext i1 %cmp to i32
  %and = and i32 %arg, %conv
  %trunc = trunc i32 %and to i8
  %conv1 = add nuw nsw i8 %trunc, -11
  %cmp.1 = icmp ugt i8 %arg1, %conv1
  %res = select i1 %cmp.1, i8 %conv1, i8 100
  ret i8 %res
}
