// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// Common .zeropage declarations for llvm-mos imaginary registers __rcN.
//
// __rcN are defined by imag-regs.ld at final-link time and live in zero
// page.  The mos assembler does not know that until link time, so by
// default it emits the 3-byte absolute form of lda/sta/... on any symbol
// whose address is not yet known.  We give it the hint here so it uses
// the 2-byte zero-page encoding instead.  Missing this bloats functions
// by ~30% and can push conditional branches out of the +/-127 range
// where the assembler silently truncates them.
//
// See https://llvm-mos.org/wiki/Assembler_relaxation.
//
// This file is included from every mos/*.S that references __rcN so the
// hint list stays in one place.  Declaring an unused __rcN is harmless
// -- the directive only affects addressing-mode selection when the
// symbol is actually referenced by an instruction.

.zeropage __rc2
.zeropage __rc3
.zeropage __rc4
.zeropage __rc5
.zeropage __rc6
.zeropage __rc7
.zeropage __rc8
.zeropage __rc9
.zeropage __rc10
.zeropage __rc11
.zeropage __rc12
.zeropage __rc13
.zeropage __rc14
.zeropage __rc15
.zeropage __rc16
.zeropage __rc17
.zeropage __rc18
.zeropage __rc19
