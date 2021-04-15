//===-- MOSInstrGISel.td - MOS Generic Instructions --------*- tablegen -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// MOS Generic Instruction Definitions.
//
// These generic instructions capture aspects of the abstract semantics of 6502
// instructions. Often, an existing generic opcode contains more functionality
// than any one 6502 instruction can actually perform. In such cases, the
// legalizer lowers these constructs into a combination of other generic opcodes
// and the target-specific ones here.
//
// Note that a 6502 instruction may map to some combination of generic opcodes;
// that's fine. The problem occurs whenever a generic instruction would map to a
// large sequence of 6502 instructions. Handling those cases in the instruction
// selector, can't take advantage of the superior optimization opportunities
// afforded by the legalizer, GlobalISel CSE, and post-legalizer combiner
// passes.
//
//===----------------------------------------------------------------------===//

class MOSGenericInstruction : GenericInstruction {
  let Namespace = "MOS";
  let hasSideEffects = false;
}

// Produce the flag values equaivalent to "SEC; LDA #$l; SBC #$r". Only supports
// 8-bit comparisons.
def G_CMP : MOSGenericInstruction {
  let OutOperandList = (outs type0:$c, type0:$n, type0:$v, type0:$z);
  let InOperandList = (ins type1:$l, type1:$r);
}

// Shift an 8-bit value left one bit, placing $carry_in in the LSB and placing
// the previous MSB bit into $carry_out. This "even" version can be chained to
// perform multi-byte shifts.
def G_SHLE : MOSGenericInstruction {
  let OutOperandList = (outs type0:$dst, type1:$carry_out);
  let InOperandList = (ins type0:$src, type1:$carry_in);
}
