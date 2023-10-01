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

// Generalized 8-bit subtraction and comparison operation. Subsumes SBC, CMP,
// CPX, and CPY. Depending on the outputs and inputs are used, this will select
// to one or more of these instructions.
def G_SBC : MOSGenericInstruction {
  let OutOperandList = (outs type0:$a, type1:$cout, type1:$n, type1:$v, type1:$z);
  let InOperandList = (ins type0:$l, type0:$r, type1:$cin);
}

// Generalized 8-bit load using the zero page indexed addressing mode. The base
// argument isn't a pointer; it's either a global value or an 8-bit immediate.
// The index argument is an 8-bit scalar. Additionally expects a
// MachineMemOperand.
def G_LOAD_ZP_IDX : MOSGenericInstruction {
  let OutOperandList = (outs type0:$dst);
  let InOperandList = (ins unknown:$base, type1:$index);
  let mayLoad = true;
}

// Generalized 8-bit load using the absolute addressing mode. The address
// argument isn't a pointer; it's either a global value or a 16-bit immediate.
// Additionally expects a MachineMemOperand.
def G_LOAD_ABS : MOSGenericInstruction {
  let OutOperandList = (outs type0:$dst);
  let InOperandList = (ins unknown:$addr);
  let mayLoad = true;
}

// Generalized 8-bit load using the absolute indexed addressing mode. The base
// argument isn't a pointer; it's either a global value or a 16-bit immediate.
// The index argument is an 8-bit scalar. Additionally expects a
// MachineMemOperand.
def G_LOAD_ABS_IDX : MOSGenericInstruction {
  let OutOperandList = (outs type0:$dst);
  let InOperandList = (ins unknown:$base, type1:$index);
  let mayLoad = true;
}

// Generalized 8-bit load using the indirect addressing mode. Additionally
// expects a MachineMemOperand.
def G_LOAD_INDIR : MOSGenericInstruction {
  let Predicates = [Has65C02];
  let OutOperandList = (outs type0:$dst);
  let InOperandList = (ins ptype1:$base);
  let mayLoad = true;
}

// Generalized 8-bit load using the indirect indexed addressing mode. The index
// argument is an 8-bit scalar. Additionally expects a MachineMemOperand.
def G_LOAD_INDIR_IDX : MOSGenericInstruction {
  let OutOperandList = (outs type0:$dst);
  let InOperandList = (ins ptype1:$base, type1:$index);
  let mayLoad = true;
}

// Generalized 8-bit store using the zero page indexed addressing mode. The base
// argument isn't a pointer; it's either a global value or a 8-bit immediate.
// The index argument is an 8-bit scalar. Additionally expects a
// MachineMemOperand.
def G_STORE_ZP_IDX : MOSGenericInstruction {
  let OutOperandList = (outs);
  let InOperandList = (ins type0:$src, unknown:$base, type1:$index);
  let mayStore = true;
}

// Generalized 8-bit store using the absolute addressing mode. The address
// argument isn't a pointer; it's either a global value or a 16-bit immediate.
// Additionally expects a MachineMemOperand.
def G_STORE_ABS : MOSGenericInstruction {
  let OutOperandList = (outs);
  let InOperandList = (ins type0:$src, unknown:$addr);
  let mayStore = true;
}

// Generalized 8-bit store using the absolute indexed addressing mode. The base
// argument isn't a pointer; it's either a global value or a 16-bit immediate.
// The index argument is an 8-bit scalar. Additionally expects a
// MachineMemOperand.
def G_STORE_ABS_IDX : MOSGenericInstruction {
  let OutOperandList = (outs);
  let InOperandList = (ins type0:$src, unknown:$base, type1:$index);
  let mayStore = true;
}

// Generalized 8-bit store using the indirect addressing mode. Additionally
// expects a MachineMemOperand.
def G_STORE_INDIR : MOSGenericInstruction {
  let Predicates = [Has65C02];
  let OutOperandList = (outs);
  let InOperandList = (ins type0:$src, ptype1:$base);
  let mayStore = true;
}

// Generalized 8-bit store using the indirect indexed addressing mode. The index
// argument is an 8-bit scalar. Additionally expects a MachineMemOperand.
def G_STORE_INDIR_IDX : MOSGenericInstruction {
  let OutOperandList = (outs);
  let InOperandList = (ins type0:$src, ptype1:$base, type1:$index);
  let mayStore = true;
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

// These opcodes represent multi-byte increment and decrement operations. Each
// use operand is one byte; either a register or an absolute address. The def
// operands correspond to only the register use operands, and they must be tied
// accordingly. Arguments are little-endian.
class MOSGIncDec : MOSGenericInstruction {
  let OutOperandList = (outs variable_ops);
  let InOperandList = (ins variable_ops);
}

def G_INC : MOSGIncDec;
def G_DEC : MOSGIncDec;

// These provide a way for the instruction selector to temporarily mark a G_INC
// in progress as mayLoad/mayStore once it obtains an addressing mode argument.
let mayLoad = true, mayStore = true in {
  def G_INC_TMP : MOSGIncDec;
  def G_DEC_TMP : MOSGIncDec;
}

def G_CMPZ : MOSGenericInstruction {
  let OutOperandList = (outs type0:$z);
  let InOperandList = (ins variable_ops);
}

// Indexed indirect branch.
def G_BRINDIRECT_IDX : MOSGenericInstruction {
  let Predicates = [HasJMPIdxIndir];
  let OutOperandList = (outs);
  let InOperandList = (ins type0:$src, type1:$index);

  let isBranch = true;
  let isTerminator = true;
  let isBarrier = true;
  let isIndirectBranch = true;
}
