// RUN: not %clang_cc1 -triple mos -O2 -emit-llvm %s -o - 2>&1 | FileCheck %s


void test_clobber_conflict() {
  // CHECK: 6:37: error: asm-specifier for input or output variable conflicts with asm clobber list
  asm volatile("" :: "a"((char)0) : "a");

  // CHECK: 9:37: error: asm-specifier for input or output variable conflicts with asm clobber list
  asm volatile("" :: "x"((char)0) : "x");

  // CHECK: 12:37: error: asm-specifier for input or output variable conflicts with asm clobber list
  asm volatile("" :: "y"((char)0) : "y");

  // CHECK: 15:37: error: asm-specifier for input or output variable conflicts with asm clobber list
  asm volatile("" :: "c"((char)0) : "c");

  // CHECK: 18:37: error: asm-specifier for input or output variable conflicts with asm clobber list
  asm volatile("" :: "v"((char)0) : "v");

  // CHECK: 22:31: error: asm-specifier for input or output variable conflicts with asm clobber list
  register char a asm("a") = (char)0;
  asm volatile("" :: "d"(a) : "a");

  // CHECK: 25:31: error: asm-specifier for input or output variable conflicts with asm clobber list
  asm volatile("" :: "g"(a) : "a");

  // CHECK: 28:31: error: asm-specifier for input or output variable conflicts with asm clobber list
  asm volatile("" :: "r"(a) : "a");

  // CHECK: 31:31: error: asm-specifier for input or output variable conflicts with asm clobber list
  asm volatile("" :: "R"(a) : "a");
}
