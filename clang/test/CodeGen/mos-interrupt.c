// RUN: %clang_cc1 -triple mos -O2 -emit-llvm %s -o - | FileCheck %s

__attribute__((interrupt_safe)) void asm_fn(unsigned char);

// CHECK-LABEL: define dso_local void @interrupt() local_unnamed_addr #0 {
__attribute__((interrupt)) void interrupt(void) {
}

// CHECK-LABEL: define dso_local void @interrupt_norecurse() local_unnamed_addr #1 {
__attribute__((interrupt_norecurse)) void interrupt_norecurse(void) {
}

// CHECK-LABEL: define dso_local void @no_isr() local_unnamed_addr #2 {
__attribute__((interrupt, no_isr)) void no_isr(void) {
}

// CHECK-LABEL: define dso_local void @interrupt_norecurse_sfx() local_unnamed_addr #3 {
__attribute__((interrupt_norecurse("__x"))) void interrupt_norecurse_sfx(void) {
  asm_fn(1);
}

// CHECK: declare void @asm_fn(i8 noundef zeroext) local_unnamed_addr #4

// CHECK: attributes #0 = { {{.*}} "interrupt" {{.*}} }
// CHECK: attributes #1 = { {{.*}} "interrupt-norecurse" {{.*}} }
// CHECK: attributes #2 = { {{.*}} "interrupt"{{.*}}"no-isr" {{.*}} }
// CHECK: attributes #3 = { {{.*}} "interrupt-norecurse"{{.*}}"interrupt-rc-suffix"="__x" {{.*}} }
// CHECK: attributes #4 = { {{.*}}"interrupt-safe"{{.*}} }

