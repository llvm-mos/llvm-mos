//===-- MOSCallingConv.td - Calling Conventions MOS --------*- tablegen -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This describes the calling conventions for the MOS architecture.
//
//===----------------------------------------------------------------------===//
//
// The C calling convention is mostly described inline. There are a few
// additional points worth mentioning:
//
// - A, X, Y, C, N, V, Z, and RS1-RS9 are caller-saved. PC, S, D, I, RS0, and
//   RS10-RS15 are callee-saved.
//
// - Aggregate types (structs, arrays, etc.) are passed by pointer. The pointer
//   is managed entirely by the caller, and may or may not be on the soft stack.
//   The callee is free to write to the memory; the caller must consider the
//   memory overwritten by the call. This is handled directly by Clang; LLVM
//   itself should never see aggregates.
//
// - Aggregate types are returned by a pointer passed as an implicit first
//   argument. The resulting function returns void. This is handled directly by
//   Clang; LLVM itself should never see aggregates.
//
// - Variable arguments (those within the ellipses of the argument list) are
//   passed through the stack. Named arguments before the variable arguments are
//   passed as usual: first in registers, then stack. Note that the variable
//   argument and regular calling convention differ; thus, variable argument
//   functions must only be called if prototyped. The C standard requires this,
//   but many platforms do not; their variable argument and regular calling
//   conventions are identical. A notable exception is Apple ARM64.
//
// - Values may be returned on the soft stack if insufficiently many registers
//   are available. Callers must reserve sufficient space for this as they do
//   for arguments. The space reserved for arguments may overlap freely with the
//   space used for return values; thus only enough space for the larger of the
//   two need be allocated.
//
// This calling convention was roughly based on that of RISC-V.
//
// For insight into the design of performant calling conventions, see the
// following work by Davidson and Whalley. By their convention, this plaftorm
// uses the "smarter hybrid" method, since LLVM performs both shrink wrapping
// and caller save-restore placement optimizations, while using both
// callee-saved and caller-saved registers when appropriate.
//
// Methods for Saving and Restoring Register Values across Function Calls
// Software--Practice and Experience Vol 21(2), 149-165 (February 1991)
// http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.14.4669&rep=rep1&type=pdf
//
//===----------------------------------------------------------------------===//

let Entry = 1 in {
def CC_MOS : CallingConv<[
  CCIfType<[i1], CCPromoteToType<i8>>,

  // Pointers are preferentially assigned to imaginary registers so indirect
  // addressing modes work without additional copying.
  //
  // RS0 is skipped since it's the stack pointer.
  CCIfPtr<CCAssignToReg<[RS1, RS2, RS3, RS4, RS5, RS6, RS7]>>,

  // 8-bit values are assigned to A, then X, then the argument imaginary registers.
  CCIfType<[i8], CCAssignToReg<[
    A, X, RC2, RC3, RC4, RC5, RC6, RC7, RC8, RC9, RC10, RC11, RC12, RC13, RC14,
    RC15
  ]>>,

  // Values that wouldn't fit anywhere else are passed directly on the stack.
  CCAssignToStack<0, 1>,
]>;

// Calling convention for the variable section of a variadic function call.
// Named arguments in such functions still use the above calling convention.
def CC_MOS_VarArgs : CallingConv<[
  CCIfType<[i1], CCPromoteToType<i8>>,

  // All values are passed directly on the stack.
  CCAssignToStack<0, 1>,
]>;
}

// RS10-RS15 are callee-saved. These are represented implicitly by their
// subregisters, since the subregisters are what must be saved and restored.
def MOS_CSR : CalleeSavedRegs<(sequence "RC%u", 20, 31)>;

// Interrupts save all registers except the stack pointers (which are restored
// upon return by any function) and the flags (which are implicitly saved by the
// interrupt).
def MOS_Interrupt_CSR :
  CalleeSavedRegs<(add A, X, Y, (sequence "RC%u", 2, 31))>;
