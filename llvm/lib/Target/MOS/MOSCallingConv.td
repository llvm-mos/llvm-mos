//===-- MOSCallingConv.td - Calling Conventions for MOS ----*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
// This describes the calling conventions for MOS architecture.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// MOS Return Value Calling Convention
//===----------------------------------------------------------------------===//

// Calling convention for variadic functions.
def ArgCC_MOS_Vararg : CallingConv
<[
  // i16 are always passed through the stack with an alignment of 1.
  CCAssignToStack<2, 1>
]>;

