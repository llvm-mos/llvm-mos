// RUN: %clang_cc1 -triple mos -fsyntax-only -verify %s

// A suffix on plain (reentrant) interrupt is rejected.
__attribute__((interrupt("__nmi"))) void plain_sfx(void) {}
// expected-error@-1 {{'interrupt' attribute does not accept a register suffix; use interrupt_norecurse("__nmi") instead}}

// Empty string suffix.
__attribute__((interrupt_norecurse(""))) void bad_empty(void) {}
// expected-error@-1 {{'interrupt_norecurse' attribute suffix '' is not a valid identifier}}

// Invalid characters in suffix.
__attribute__((interrupt_norecurse("nm-i"))) void bad_chars(void) {}
// expected-error@-1 {{'interrupt_norecurse' attribute suffix 'nm-i' is not a valid identifier}}

// Numeric start of suffix.
__attribute__((interrupt_norecurse("1nmi"))) void bad_num(void) {}
// expected-error@-1 {{'interrupt_norecurse' attribute suffix '1nmi' is not a valid identifier}}

// Invalid suffix on plain interrupt reports the identifier problem first.
__attribute__((interrupt("1nmi"))) void bad_plain_num(void) {}
// expected-error@-1 {{'interrupt' attribute suffix '1nmi' is not a valid identifier}}

// Too many arguments.
__attribute__((interrupt_norecurse("__nmi", "extra"))) void bad_two_args(void) {}
// expected-error@-1 {{'interrupt_norecurse' attribute takes no more than 1 argument}}

// Non-string argument.
__attribute__((interrupt_norecurse(123))) void bad_non_string(void) {}
// expected-error@-1 {{expected string literal as argument of 'interrupt_norecurse' attribute}}

// Redeclaration with a different suffix.
__attribute__((interrupt_norecurse("__a"))) void redecl(void); // expected-note {{previous attribute is here}}
__attribute__((interrupt_norecurse("__b"))) void redecl(void) {}
// expected-error@-1 {{'interrupt_norecurse' attribute suffix '__b' conflicts with suffix '__a' on a previous declaration}}

// Redeclaration dropping the suffix also changes the register set.
__attribute__((interrupt_norecurse("__a"))) void redecl2(void); // expected-note {{previous attribute is here}}
__attribute__((interrupt_norecurse)) void redecl2(void) {}
// expected-error@-1 {{'interrupt_norecurse' attribute suffix '' conflicts with suffix '__a' on a previous declaration}}
