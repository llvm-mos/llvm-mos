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

// Version of G_BRCOND that takes an immediate argument for which boolean value
// triggers the branch. This could be done in the instruction selector, but that
// would require making 1-bit G_XORs legal. However, they are only legal when
// used in this way, so the legalizer handles this instead, that way the
// remaining unhandled G_XORs can be widened to 8 bits.
def G_BRCOND_IMM : MOSGenericInstruction {
  let OutOperandList = (outs);
  let InOperandList = (ins type0:$tst, unknown:$truebb, i1imm:$tstval);

  let isBranch = true;
  let isTerminator = true;
}

// Produce the flag values equaivalent to "SEC; LDA #$l; SBC #$r". Only supports
// 8-bit comparisons. At most one of N or Z can be live out of this instruction,
// since saving N somewhere permanent clobbers Z, and vice versa.
def G_CMP : MOSGenericInstruction {
  let OutOperandList = (outs type0:$c, type0:$n, type0:$v, type0:$z);
  let InOperandList = (ins type1:$l, type1:$r);
}

// Return the result of adding a pointer value and an unsigned 8-bit offset.
// Fragement with other opcodes to construct the indexed addressing modes on the
// 6502.
def G_INDEX : MOSGenericInstruction {
  let OutOperandList = (outs type0:$ptr);
  let InOperandList = (ins type0:$base, type1:$offset);
}

// 8-bit subtraction with carry in and out. The carry uses the 6502 convention
// that a zero carry indicates a borrow. G_USUBE instead uses a one carry to
// indicate a borrow.
def G_SBC : MOSGenericInstruction {
  let OutOperandList = (outs type0:$dst, type1:$carry_out);
  let InOperandList = (ins type0:$src1, type0:$src2, type1:$carry_in);
}

// Shift an 8-bit value left one bit, placing $carry_in in the LSB and placing
// the previous MSB bit into $carry_out. This "even" version can be chained to
// perform multi-byte shifts.
def G_SHLE : MOSGenericInstruction {
  let OutOperandList = (outs type0:$dst, type1:$carry_out);
  let InOperandList = (ins type0:$src, type1:$carry_in);
}

// Logically shift an 8-bit value right one bit, placing $carry_in in the MSB
// and placing the previous LSB bit into $carry_out. This "even" version can be
// chained to perform multi-byte shifts.
def G_LSHRE : MOSGenericInstruction {
  let OutOperandList = (outs type0:$dst, type1:$carry_out);
  let InOperandList = (ins type0:$src, type1:$carry_in);
}
