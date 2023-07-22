// RUN: %clang_cc1 -triple mos -emit-llvm %s -o -

// Test MOSs alignment

struct short_test {
  char a;
  short b;
};

_Static_assert(sizeof(struct short_test) == 3, "incorrect short alignment");

