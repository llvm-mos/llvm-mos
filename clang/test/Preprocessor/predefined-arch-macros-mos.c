// RUN: %clang -E -dM %s -o - 2>&1 -target mos \
// RUN:   | FileCheck -match-full-lines %s -check-prefix=CHECK_MOS_6502
// CHECK_MOS_6502: #define __mos6502__ 1
// CHECK_MOS_6502: #define __mos__ 1

// RUN: %clang -E -dM %s -o - 2>&1 -target mos -mcpu=mos6502x \
// RUN:   | FileCheck -match-full-lines %s -check-prefix=CHECK_MOS_6502X
// CHECK_MOS_6502X: #define __mos6502__ 1
// CHECK_MOS_6502X: #define __mos6502x__ 1
// CHECK_MOS_6502X: #define __mos__ 1

// RUN: %clang -E -dM %s -o - 2>&1 -target mos -mcpu=mos65c02 \
// RUN:   | FileCheck -match-full-lines %s -check-prefix=CHECK_MOS_65C02
// CHECK_MOS_65C02: #define __mos6502__ 1
// CHECK_MOS_65C02: #define __mos65c02__ 1
// CHECK_MOS_65C02: #define __mos__ 1

// RUN: %clang -E -dM %s -o - 2>&1 -target mos -mcpu=mosr65c02 \
// RUN:   | FileCheck -match-full-lines %s -check-prefix=CHECK_MOS_R65C02
// CHECK_MOS_R65C02: #define __mos6502__ 1
// CHECK_MOS_R65C02: #define __mos65c02__ 1
// CHECK_MOS_R65C02: #define __mos__ 1
// CHECK_MOS_R65C02: #define __mosr65c02__ 1

// RUN: %clang -E -dM %s -o - 2>&1 -target mos -mcpu=mosw65c02 \
// RUN:   | FileCheck -match-full-lines %s -check-prefix=CHECK_MOS_W65C02
// CHECK_MOS_W65C02: #define __mos6502__ 1
// CHECK_MOS_W65C02: #define __mos65c02__ 1
// CHECK_MOS_W65C02: #define __mos__ 1
// CHECK_MOS_W65C02: #define __mosr65c02__ 1
// CHECK_MOS_W65C02: #define __mosw65c02__ 1

// RUN: %clang -E -dM %s -o - 2>&1 -target mos -mcpu=mosw65816 \
// RUN:   | FileCheck -match-full-lines %s -check-prefix=CHECK_MOS_W65816
// CHECK_MOS_W65816: #define __mos6502__ 1
// CHECK_MOS_W65816: #define __mos65c02__ 1
// CHECK_MOS_W65816: #define __mos__ 1
// CHECK_MOS_W65816: #define __mosw65816__ 1
// CHECK_MOS_W65816: #define __mosw65c02__ 1

// RUN: %clang -E -dM %s -o - 2>&1 -target mos -mcpu=mos65el02 \
// RUN:   | FileCheck -match-full-lines %s -check-prefix=CHECK_MOS_65EL02
// CHECK_MOS_65EL02: #define __mos6502__ 1
// CHECK_MOS_65EL02: #define __mos65c02__ 1
// CHECK_MOS_65EL02: #define __mos65el02__ 1
// CHECK_MOS_65EL02: #define __mos__ 1
// CHECK_MOS_65EL02: #define __mosw65c02__ 1

// RUN: %clang -E -dM %s -o - 2>&1 -target mos -mcpu=mos65ce02 \
// RUN:   | FileCheck -match-full-lines %s -check-prefix=CHECK_MOS_65CE02
// CHECK_MOS_65CE02: #define __mos6502__ 1
// CHECK_MOS_65CE02: #define __mos65c02__ 1
// CHECK_MOS_65CE02: #define __mos65ce02__ 1
// CHECK_MOS_65CE02: #define __mos__ 1
// CHECK_MOS_65CE02: #define __mosr65c02__ 1

// RUN: %clang -E -dM %s -o - 2>&1 -target mos -mcpu=moshuc6280 \
// RUN:   | FileCheck -match-full-lines %s -check-prefix=CHECK_MOS_HUC6280
// CHECK_MOS_HUC6280: #define __mos6502__ 1
// CHECK_MOS_HUC6280: #define __mos65c02__ 1
// CHECK_MOS_HUC6280: #define __mos__ 1
// CHECK_MOS_HUC6280: #define __moshuc6280__ 1
// CHECK_MOS_HUC6280: #define __mosr65c02__ 1

// RUN: %clang -E -dM %s -o - 2>&1 -target mos -mcpu=mos65dtv02 \
// RUN:   | FileCheck -match-full-lines %s -check-prefix=CHECK_MOS_65DTV02
// CHECK_MOS_65DTV02: #define __mos6502__ 1
// CHECK_MOS_65DTV02: #define __mos65dtv02__ 1
// CHECK_MOS_65DTV02: #define __mos__ 1

// RUN: %clang -E -dM %s -o - 2>&1 -target mos -mcpu=mos4510 \
// RUN:   | FileCheck -match-full-lines %s -check-prefix=CHECK_MOS_4510
// CHECK_MOS_4510: #define __mos4510__ 1
// CHECK_MOS_4510: #define __mos6502__ 1
// CHECK_MOS_4510: #define __mos65c02__ 1
// CHECK_MOS_4510: #define __mos65ce02__ 1
// CHECK_MOS_4510: #define __mos__ 1
// CHECK_MOS_4510: #define __mosr65c02__ 1

// RUN: %clang -E -dM %s -o - 2>&1 -target mos -mcpu=mos45gs02 \
// RUN:   | FileCheck -match-full-lines %s -check-prefix=CHECK_MOS_45GS02
// CHECK_MOS_45GS02: #define __mos4510__ 1
// CHECK_MOS_45GS02: #define __mos45gs02__ 1
// CHECK_MOS_45GS02: #define __mos6502__ 1
// CHECK_MOS_45GS02: #define __mos65c02__ 1
// CHECK_MOS_45GS02: #define __mos65ce02__ 1
// CHECK_MOS_45GS02: #define __mos__ 1
// CHECK_MOS_45GS02: #define __mosr65c02__ 1

// RUN: %clang -E -dM %s -o - 2>&1 -target mos -mcpu=mosspc700 \
// RUN:   | FileCheck -match-full-lines %s -check-prefix=CHECK_MOS_SPC700
// CHECK_MOS_SPC700: #define __mos__ 1
// CHECK_MOS_SPC700: #define __mosspc700__ 1
