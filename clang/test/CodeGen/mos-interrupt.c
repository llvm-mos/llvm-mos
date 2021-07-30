// RUN: %clang_cc1 -triple mos -O2 -emit-llvm %s -o - | FileCheck %s

// CHECK-LABEL: define dso_local void @interrupt() local_unnamed_addr #0 {
__attribute__((interrupt)) void interrupt(void) {
}

// CHECK-LABEL: define dso_local void @interrupt_norecurse() local_unnamed_addr #1 {
__attribute__((interrupt_norecurse)) void interrupt_norecurse(void) {
}

// CHECK-LABEL: define dso_local void @no_isr() local_unnamed_addr #2 {
__attribute__((interrupt, no_isr)) void no_isr(void) {
}

// CHECK: attributes #0 = { {{.*}} "interrupt" {{.*}} }
// CHECK: attributes #1 = { {{.*}} "interrupt-norecurse" {{.*}} }
// CHECK: attributes #2 = { {{.*}} "interrupt"{{.*}}"no-isr" {{.*}} }

