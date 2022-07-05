// RUN: %clang_cc1 -triple mos -O2 -emit-llvm %s -o - | FileCheck %s

// Test MOSs inline assembly.

char c;
_Bool b;

void test_a() {
  // CHECK-LABEL: define dso_local void @test_a() {{.*}} {
  // CHECK: [[V:%[0-9]+]] = load i8, ptr @c
  // CHECK: tail call void asm sideeffect "", "a"(i8 [[V]])
  asm volatile("" :: "a"(c));
}

void test_x() {
  // CHECK-LABEL: define dso_local void @test_x() {{.*}} {
  // CHECK: [[V:%[0-9]+]] = load i8, ptr @c
  // CHECK: tail call void asm sideeffect "", "x"(i8 [[V]])
  asm volatile("" :: "x"(c));
}

void test_y() {
  // CHECK-LABEL: define dso_local void @test_y() {{.*}} {
  // CHECK: [[V:%[0-9]+]] = load i8, ptr @c
  // CHECK: tail call void asm sideeffect "", "y"(i8 [[V]])
  asm volatile("" :: "y"(c));
}

void test_d() {
  // CHECK-LABEL: define dso_local void @test_d() {{.*}} {
  // CHECK: [[V:%[0-9]+]] = load i8, ptr @c
  // CHECK: tail call void asm sideeffect "", "d"(i8 [[V]])
  asm volatile("" :: "d"(c));
}

void test_c() {
  // CHECK-LABEL: define dso_local void @test_c() {{.*}} {
  // CHECK: [[V:%[0-9]+]] = load i8, ptr @b
  // CHECK: %tobool = icmp ne i8 [[V]], 0
  // CHECK: tail call void asm sideeffect "", "c"(i1 %tobool)
  asm volatile("" :: "c"(b));
}

void test_v() {
  // CHECK-LABEL: define dso_local void @test_v() {{.*}} {
  // CHECK: [[V:%[0-9]+]] = load i8, ptr @b
  // CHECK: %tobool = icmp ne i8 [[V]], 0
  // CHECK: tail call void asm sideeffect "", "v"(i1 %tobool)
  asm volatile("" :: "v"(b));
}

void test_leaf_asm() {
  // CHECK-LABEL: define dso_local void @test_leaf_asm() {{.*}} {
  // CHECK: tail call void asm sideeffect "", ""() #2
  // CHECK: #2 = { nocallback nounwind }
  __attribute__((leaf)) asm volatile("");
}
