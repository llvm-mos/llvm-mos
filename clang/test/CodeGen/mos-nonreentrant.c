// RUN: %clang -target mos -S -emit-llvm %s -o - -freentrant -fnonreentrant | FileCheck %s --check-prefix=NONREENTRANT
// RUN: %clang -target mos -S -emit-llvm %s -o - -fnonreentrant -freentrant | FileCheck %s --check-prefix=REENTRANT

// CHECK: define dso_local void @reentrant() #0 {
__attribute__((reentrant, nonreentrant)) void reentrant(void) {
	reentrant();
}

// CHECK: define dso_local void @nonreentrant() #1 {
__attribute__((nonreentrant, reentrant)) void nonreentrant(void) {
	nonreentrant();
}

// CHECK: define dso_local void @nonreentrant() #1 {
__attribute__((reentrant)) void nonreentrant2(void);
__attribute__((nonreentrant)) void nonreentrant2(void) {
	nonreentrant2();
}

// NONREENTRANT: define dso_local void @foo() #1 {
// REENTRANT: define dso_local void @foo() #0 {
void foo(void) {
	foo();
}

// CHECK-NOT: attributes #0 = { {{.*}} "nonreentrant" {{.*}} }
// CHECK: attributes #1 = { {{.*}} "nonreentrant" {{.*}} }
