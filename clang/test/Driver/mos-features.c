// Check for AArch64 out-of-line atomics default settings.
// RUN: %clang -target mos -fstatic-stack \
// RUN: -### -c %s 2>&1 | FileCheck -check-prefix=CHECK-STATIC-STACK %s
// RUN: %clang -target mos -fno-static-stack \
// RUN: -### -c %s 2>&1 | FileCheck -check-prefix=CHECK-NO-STATIC-STACK %s

// CHECK-STATIC-STACK: "-target-feature" "+static-stack"
// CHECK-NO-STATIC-STACK: "-target-feature" "-static-stack"
