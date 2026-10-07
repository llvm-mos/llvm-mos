// RUN: %clang_cc1 -triple mos -fsyntax-only -verify %s

// expected-no-diagnostics

// Valid: no argument.
__attribute__((interrupt)) void ok1(void) {}
__attribute__((interrupt_norecurse)) void ok2(void) {}

// Valid: string argument with valid identifier suffix.
__attribute__((interrupt_norecurse("__nmi"))) void ok3(void) {}
__attribute__((interrupt_norecurse("_irq"))) void ok4(void) {}
__attribute__((interrupt_norecurse("ABC123"))) void ok5(void) {}

// Valid: redeclaration with the same suffix, or inheriting it.
__attribute__((interrupt_norecurse("__a"))) void ok6(void);
__attribute__((interrupt_norecurse("__a"))) void ok6(void) {}
__attribute__((interrupt_norecurse("__b"))) void ok7(void);
void ok7(void) {}
