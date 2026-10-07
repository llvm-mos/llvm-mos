// RUN: %clang_cc1 -triple mos -fsyntax-only -verify %s

// Valid on declarations.
__attribute__((interrupt_safe)) void asm_fn(unsigned char);
__attribute__((interrupt_safe)) unsigned char asm_fn2(void);

// Takes no arguments.
__attribute__((interrupt_safe(1))) void bad_args(void);
// expected-error@-1 {{'interrupt_safe' attribute takes no arguments}}

// Rejected on definitions.
__attribute__((interrupt_safe)) void c_fn(void) {}
// expected-error@-1 {{'interrupt_safe' attribute only applies to function declarations, not definitions}}

// Also rejected when inherited by a later definition.
__attribute__((interrupt_safe)) void later_def(void); // expected-error {{'interrupt_safe' attribute only applies to function declarations, not definitions}}
void later_def(void) {}

// Not a function.
__attribute__((interrupt_safe)) int var;
// expected-warning@-1 {{'interrupt_safe' attribute only applies to functions}}
