// REQUIRES: powerpc-registered-target
// RUN: %clang_cc1 -triple powerpc64-unknown-linux-gnu \
// RUN:   -emit-llvm %s -o - -target-cpu pwr7 | FileCheck %s
// RUN: %clang_cc1 -triple powerpc64le-unknown-linux-gnu \
// RUN:   -emit-llvm %s -o - -target-cpu pwr8 | FileCheck %s
// RUN: %clang_cc1 -triple powerpc-unknown-aix \
// RUN:   -emit-llvm %s -o - -target-cpu pwr7 | FileCheck %s
// RUN: %clang_cc1 -triple powerpc64-unknown-aix \
// RUN:   -emit-llvm %s -o - -target-cpu pwr7 | FileCheck %s

extern unsigned int ui;
extern unsigned long long ull;

void test_builtin_ppc_rldimi() {
  // CHECK-LABEL: test_builtin_ppc_rldimi
  // CHECK:       %res = alloca i64, align 8
  // CHECK-NEXT:  [[RA:%[0-9]+]] = load i64, ptr @ull, align 8
  // CHECK-NEXT:  [[RB:%[0-9]+]] = load i64, ptr @ull, align 8
  // CHECK-NEXT:  [[RC:%[0-9]+]] = call i64 @llvm.ppc.rldimi(i64 [[RA]], i64 [[RB]], i32 63, i64 72057593769492480)
  // CHECK-NEXT:  store i64 [[RC]], ptr %res, align 8
  // CHECK-NEXT:  ret void

  /*shift = 63, mask = 0x00FFFFFFF0000000 = 72057593769492480, ~mask = 0xFF0000000FFFFFFF = -72057593769492481*/
  unsigned long long res = __builtin_ppc_rldimi(ull, ull, 63, 0x00FFFFFFF0000000);
}

void test_builtin_ppc_rlwimi() {
  // CHECK-LABEL: test_builtin_ppc_rlwimi
  // CHECK:       %res = alloca i32, align 4
  // CHECK-NEXT:  [[RA:%[0-9]+]] = load i32, ptr @ui, align 4
  // CHECK-NEXT:  [[RB:%[0-9]+]] = load i32, ptr @ui, align 4
  // CHECK-NEXT:  [[RC:%[0-9]+]] = call i32 @llvm.ppc.rlwimi(i32 [[RA]], i32 [[RB]], i32 31, i32 16776960)
  // CHECK-NEXT:  store i32 [[RC]], ptr %res, align 4
  // CHECK-NEXT:  ret void

  /*shift = 31, mask = 0xFFFF00 = 16776960, ~mask = 0xFFFFFFFFFF0000FF = -16776961*/
  unsigned int res = __builtin_ppc_rlwimi(ui, ui, 31, 0xFFFF00);
}

void test_builtin_ppc_rlwnm() {
  // CHECK-LABEL: test_builtin_ppc_rlwnm
  // CHECK:       %res = alloca i32, align 4
  // CHECK-NEXT:  [[RA:%[0-9]+]] = load i32, ptr @ui, align 4
  // CHECK-NEXT:  [[RB:%[0-9]+]] = call i32 @llvm.ppc.rlwnm(i32 [[RA]], i32 31, i32 511)
  // CHECK-NEXT:  store i32 [[RB]], ptr %res, align 4
  // CHECK-NEXT:  ret void

  /*shift = 31, mask = 0x1FF = 511*/
  unsigned int res = __builtin_ppc_rlwnm(ui, 31, 0x1FF);
}

void test_builtin_ppc_rlwnm2(unsigned int shift) {
  // CHECK-LABEL: test_builtin_ppc_rlwnm2
  // CHECK:       %shift.addr = alloca i32, align 4
  // CHECK-NEXT:  %res = alloca i32, align 4
  // CHECK-NEXT:  store i32 %shift, ptr %shift.addr, align 4
  // CHECK-NEXT:  [[RA:%[0-9]+]] = load i32, ptr @ui, align 4
  // CHECK-NEXT:  [[RB:%[0-9]+]] = load i32, ptr %shift.addr, align 4
  // CHECK-NEXT:  [[RC:%[0-9]+]] = call i32 @llvm.ppc.rlwnm(i32 [[RA]], i32 [[RB]], i32 511)
  // CHECK-NEXT:  store i32 [[RC]], ptr %res, align 4
  // CHECK-NEXT:  ret void

  /*mask = 0x1FF = 511*/
  unsigned int res = __builtin_ppc_rlwnm(ui, shift, 0x1FF);
}

// CHECK-LABEL: @testrotatel4(
// CHECK:         [[TMP:%.*]] = call i32 @llvm.fshl.i32(i32 {{%.*}}, i32 {{%.*}}, i32 {{%.*}})
// CHECK-NEXT:    ret i32 [[TMP]]
//
unsigned int testrotatel4(unsigned int rs, unsigned int shift) {
  return __rotatel4(rs, shift);
}

// CHECK-LABEL: @testrotatel8(
// CHECK:         [[TMP:%.*]] = call i64 @llvm.fshl.i64(i64 {{%.*}}, i64 {{%.*}}, i64 {{%.*}})
// CHECK-NEXT:    ret i64 [[TMP]]
//
unsigned long long testrotatel8(unsigned long long rs, unsigned long long shift) {
  return __rotatel8(rs, shift);
}

// CHECK-LABEL: @testrdlam(
// CHECK:         [[TMP0:%.*]] = call i64 @llvm.fshl.i64(i64 {{%.*}}, i64 {{%.*}}, i64 {{%.*}})
// CHECK-NEXT:    [[TMP1:%.*]] = and i64 [[TMP0]], 7
// CHECK-NEXT:    ret i64 [[TMP1]]
//
unsigned long long testrdlam(unsigned long long rs, unsigned int shift) {
  // The third parameter is a mask that must be a constant that represents a
  // contiguous bit field.
  return __rdlam(rs, shift, 7);
}
