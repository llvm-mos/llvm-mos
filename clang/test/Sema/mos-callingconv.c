// RUN: %clang_cc1 -triple mos -std=c17 -fsyntax-only -verify %s

// preserve_most is the only non-default calling convention supported on MOS.
void __attribute__((preserve_most)) pm(unsigned char a);

// Other calling conventions are still ignored with a warning.
void __attribute__((preserve_all)) pa(void); // expected-warning {{'preserve_all' calling convention is not supported for this target}}
void __attribute__((fastcall)) fc(void); // expected-warning {{'fastcall' calling convention is not supported for this target}}

// preserve_most passes every argument in A, X, Y, or on the stack, and has no
// variadic form.
void __attribute__((preserve_most)) pm_va(unsigned char a, ...); // expected-error {{variadic function cannot use preserve_most calling convention}}
void __attribute__((preserve_most)) pm_va_def(unsigned char a, ...) {} // expected-error {{variadic function cannot use preserve_most calling convention}}
typedef void __attribute__((preserve_most)) pm_va_fn(int, ...); // expected-error {{variadic function cannot use preserve_most calling convention}}

// Unprototyped declarations use the variadic call rules.
void __attribute__((preserve_most)) pm_knr(); // expected-error {{function with no prototype cannot use the preserve_most calling convention}}

// Variadic functions with the default convention are unaffected.
void va(unsigned char a, ...);
