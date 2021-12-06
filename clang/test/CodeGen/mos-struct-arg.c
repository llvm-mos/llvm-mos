// RUN: %clang_cc1 -triple mos -O2 -emit-llvm %s -o - | FileCheck %s

struct S { int a; };

// CHECK-LABEL: define dso_local void @test_struct_param(%struct.S* noalias nocapture writeonly sret(%struct.S) align 1 %agg.result, %struct.S* nocapture readonly %s) {{.*}} {
struct S test_struct_param(struct S s) {
  // CHECK:      %0 = getelementptr %struct.S, %struct.S* %s, i16 0, i32 0
  // CHECK-NEXT: %1 = getelementptr inbounds %struct.S, %struct.S* %agg.result, i16 0, i32 0
  // CHECK-NEXT: %2 = load i16, i16* %0, align 1
  // CHECK-NEXT: store i16 %2, i16* %1, align 1
  // CHECK-NEXT: ret void
  return s;
}
