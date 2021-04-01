//===-- MOSInstrLogical.td - Describe MOS Logical Instrs --*- tablegen -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
//
// This file describes the MOS logical instruction set.
//
// Instructions with identical effects except the placement of their operands
// are joined together into instructions with register classes directly
// representing those constraints.
//
// This may take more than one instruction. For example, the register transfer
// instructions TAX, TXA, TAY, and TYA are split into TA_ and T_A, since
// register classes are too crude of an instrument to specify that either the
// source or target must be A. An XY register class fills in the remaining
// freedom that the _ may be fulfilled with either X or Y.
//
// Doing things this way allows the code generator to pretend that the 6502 is
// much more regular than it really is, right up until AsmPrinter brings the
// instructions down to their real, bitwise representation. This shields most of
// LLVM from the rigors of the real, highly irregular 6502 instruction set. For
// example, it brings the number of implicit physical registers down to the
// level typical of a LLVM backend. Codegen passes can handle implicit physical
// registers every once in a while, but they have a hard time dealing with every
// single instruction using them for nearly every operand. Most of the issues are
// probably just oversights, but there's a lot to be gained by tightly limiting
// "weirdness" as much as possible.
//
//===---------------------------------------------------------------------===//

class MOSLogicalInstr : Instruction {
  let Namespace = "MOS";

  dag OutOperandList = (outs);
  dag InOperandList = (ins);

  let hasSideEffects = false;
  let mayLoad = false;
  let mayStore = false;
  let isPseudo = true;
  let isCodeGenOnly = true;
}

def label : Operand<OtherVT>;

def ADCimm : MOSLogicalInstr, PseudoInstExpansion<(ADC_Immediate imm8:$r)> {
  dag OutOperandList = (outs Ac:$dst, Cc:$carryout);
  dag InOperandList = (ins Ac:$l, i8imm:$r, Cc:$carryin);
  let Constraints = "$dst = $l, $carryout = $carryin";

  let Defs = [NZ];

  let isAdd = true;
  let isReMaterializable = true;
}

def ADCimag8 : MOSLogicalInstr, PseudoInstExpansion<(ADC_ZeroPage addr8:$r)> {
  dag OutOperandList = (outs Ac:$dst, Cc:$carryout);
  dag InOperandList = (ins Ac:$l, Imag8:$r, Cc:$carryin);
  // Don't give the carries a tied constraint, since having two tied operands
  // inhibits commutation of $l and $r.
  let Constraints = "$dst = $l";

  let Defs = [NZ];

  let isAdd = true;
  let isCommutable = true;
  let isReMaterializable = true;
}

def ASL : MOSLogicalInstr {
  dag OutOperandList = (outs AImag8:$dst, Cc:$carry);
  dag InOperandList = (ins AImag8:$src);
  let Constraints = "$dst = $src";

  let Defs = [NZ];

  let isReMaterializable = true;
}

// Branches to $tgt if $flag = $val.
def BR : MOSLogicalInstr {
  dag InOperandList = (ins label:$tgt, Flag:$flag, i1imm:$val);

  let isBranch = true;
  let isTerminator = true;
}

def CMPimm : MOSLogicalInstr {
  dag InOperandList = (ins GPR:$l, i8imm:$r);

  let Defs = [C, NZ];

  let isCompare = true;
  let isReMaterializable = true;

  // Keep CMPimm adjacent to the branch that uses it.
  // This prevents NZ from ever being live outside of the terminators.
  let isTerminator = true;
}

def IN_ : MOSLogicalInstr {
  dag OutOperandList = (outs XY:$dst);
  dag InOperandList = (ins XY:$src);
  let Constraints = "$src = $dst";

  let Defs = [NZ];

  let Pattern = [(set XY:$dst, (add XY:$src, 1))];


  let isAdd = true;
  let isReMaterializable = true;
}

def JMP : MOSLogicalInstr, PseudoInstExpansion<(JMP_Absolute addr16:$tgt)> {
  dag InOperandList = (ins label:$tgt);

  let Pattern = [(br bb:$tgt)];

  let isBarrier = true;
  let isBranch = true;
  let isTerminator = true;
}

def JSR : MOSLogicalInstr, PseudoInstExpansion<(JSR_Absolute addr16:$tgt)> {
  dag InOperandList = (ins label:$tgt);

  let isCall = true;
}

def LDCimm : MOSLogicalInstr {
  dag OutOperandList = (outs Cc:$dst);
  dag InOperandList = (ins i1imm:$val);

  let isReMaterializable = true;
  let isAsCheapAsAMove = true;
}

def LDimm : MOSLogicalInstr {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins i8imm:$val);

  let Defs = [NZ];

  let Pattern = [(set GPR:$dst, imm:$val)];

  let isAsCheapAsAMove = true;
  let isMoveImm = true;
  let isReMaterializable = true;
}

def LDimag8 : MOSLogicalInstr {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins Imag8:$src);

  let Defs = [NZ];

  let isReMaterializable = true;
}

def LDabs : MOSLogicalInstr {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins i16imm:$src);

  let Defs = [NZ];

  let isReMaterializable = true;
  let mayLoad = true;
}

def LDAidx : MOSLogicalInstr {
  dag InOperandList = (ins i16imm:$addr, XY:$idx);

  let Defs = [A, NZ];

  let isReMaterializable = true;
  let mayLoad = true;
}

def LDXidx : MOSLogicalInstr, PseudoInstExpansion<(LDX_AbsoluteY addr16:$addr)> {
  dag InOperandList = (ins i16imm:$addr);

  let Defs = [X, NZ];
  let Uses = [Y];

  let isReMaterializable = true;
  let mayLoad = true;
}

def LDYidx : MOSLogicalInstr, PseudoInstExpansion<(LDY_AbsoluteX addr16:$addr)> {
  dag InOperandList = (ins i16imm:$addr);

  let Defs = [Y, NZ];
  let Uses = [X];

  let isReMaterializable = true;
  let mayLoad = true;
}

def LDyindir : MOSLogicalInstr, PseudoInstExpansion<(LDA_IndirectIndexed addr8:$addr)> {
  dag OutOperandList = (outs Ac:$dst);
  dag InOperandList = (ins Imag16:$addr, Yc:$offset);

  let Defs = [NZ];

  let mayLoad = true;
}

def PHA : MOSLogicalInstr, PseudoInstExpansion<(PHA_Implied)> {
  let Uses = [A];

  let mayStore = true;
}

def PLA : MOSLogicalInstr, PseudoInstExpansion<(PLA_Implied)> {
  let Defs = [A, NZ];

  let mayLoad = true;
}

def PHP : MOSLogicalInstr, PseudoInstExpansion<(PHP_Implied)> {
  let Uses = [P];

  let mayStore = true;
}

def PLP : MOSLogicalInstr, PseudoInstExpansion<(PLP_Implied)> {
  let Defs = [P];

  let mayLoad = true;
}

def ROL : MOSLogicalInstr {
  dag OutOperandList = (outs AImag8:$dst, Cc:$carryout);
  dag InOperandList = (ins AImag8:$src, Cc:$carryin);
  let Constraints = "$dst = $src, $carryout = $carryin";

  let Defs = [NZ];

  let isReMaterializable = true;
}

def RTS : MOSLogicalInstr, PseudoInstExpansion<(RTS_Implied)> {
  let isBarrier = true;
  let isReturn = true;
  let isTerminator = true;
}

def SBCimm : MOSLogicalInstr, PseudoInstExpansion<(SBC_Immediate imm8:$r)> {
  dag OutOperandList = (outs Ac:$dst, Cc:$carryout);
  dag InOperandList = (ins Ac:$l, i8imm:$r, Cc:$carryin);
  let Constraints = "$dst = $l, $carryout = $carryin";

  let Defs = [NZ];

  let isReMaterializable = true;
}

def SBCimag8 : MOSLogicalInstr, PseudoInstExpansion<(SBC_ZeroPage addr8:$r)> {
  dag OutOperandList = (outs Ac:$dst, Cc:$carryout);
  dag InOperandList = (ins Ac:$l, Imag8:$r, Cc:$carryin);
  // Don't give the carries a tied constraint, since having two tied operands
  // inhibits commutation of $l and $r.
  let Constraints = "$dst = $l";

  let Defs = [NZ];

  let isReMaterializable = true;
}

def STabs : MOSLogicalInstr {
  dag InOperandList = (ins GPR:$src, i16imm:$dst);

  let mayStore = true;
}

def STidx : MOSLogicalInstr {
  dag InOperandList = (ins Ac:$src, i16imm:$addr, XY:$idx);

  let mayStore = true;
}

def STyindir : MOSLogicalInstr, PseudoInstExpansion<(STA_IndirectIndexed addr8:$addr)> {
  dag InOperandList = (ins Ac:$src, Imag16:$addr, Yc:$offset);

  let mayStore = true;
}

def STimag8 : MOSLogicalInstr {
  dag OutOperandList = (outs Imag8:$dst);
  dag InOperandList = (ins GPR:$src);
}

def TA_ : MOSLogicalInstr {
  dag OutOperandList = (outs XY:$dst);

  let Defs = [NZ];
  let Uses = [A];

  let isAsCheapAsAMove = true;
  let isReMaterializable = true;
}

def T_A : MOSLogicalInstr {
  dag InOperandList = (ins XY:$src);

  let Defs = [A, NZ];

  let isAsCheapAsAMove = true;
  let isReMaterializable = true;
}

def TSX : MOSLogicalInstr, PseudoInstExpansion<(TSX_Implied)> {
  let Defs = [X, NZ];

  let isAsCheapAsAMove = true;
  let isReMaterializable = true;
}
