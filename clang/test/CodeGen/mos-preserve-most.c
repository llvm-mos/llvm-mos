// RUN: %clang_cc1 -triple mos -emit-llvm %s -o - | FileCheck %s

// The preserve_most attribute selects the PreserveMost calling convention, under
// which a function preserves nearly all imaginary registers.

__attribute__((preserve_most)) unsigned char f(unsigned char a, unsigned char b,
                                               unsigned char c) {
  return a + b + c;
}

__attribute__((preserve_most)) void g(void);

void h(void) { g(); }

// CHECK: define {{.*}}preserve_mostcc zeroext i8 @f(
// CHECK: call preserve_mostcc void @g()
// CHECK: declare preserve_mostcc void @g()
