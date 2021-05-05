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
// When viewed logically, the 6502 instruction set is more regular than it might
// appear. For example, consider the three instructions: LDA #imm, LDX #imm, and
// LDY #imm. Each of these instructions takes a single immediate operand, loads
// one of A, X, or Y, and sets NZ based on the immediate value. Even though
// there are three separate opcodes, the only difference in their behavior is
// which register is loaded.
//
// The logical instruction set merges these instructions together into a single
// logical LDImm #imm instruction. The logical instruction takes a "GPR"
// register class as an argument. When the logical instructions are lowered to
// real instructions in AsmPrinter, the register operand is examined to
// determine which real instruction is emitted. This scheme comes with some
// considerable advantages.
//
// First, selecting a LDImm instruction doesn't constrain where the destination
// can be placed other than that it needs to be one of A, X, or Y. This allows
// earlier passes that cannot effectively make such decisions to defer them to
// the register allocator.
//
// Second, the description of the side effects and characteristics of LDImm only
// need to be specified once. Any switch() statements need to have only one
// entry for LDImm, not three for LDAImm, LDXImm, and LDYImm. This considerably
// reduces the amount of code needed to cover the 6502 instruction set.
//
// Third, exposing these regularities to LLVM passes make the 6502 more like a
// regular CISC target (X86-ish). LLVM already has very good code paths for
// dealing with this sort of architecture, and maintaining a logical instruction
// set allows us to stay within those "sane", well-tested code paths, for the
// most part.
//
// By examining its register operands, every logical instruction can be mapped
// to exactly one real target instruction. Pseudoinstructions where this
// property does not hold are recorded in MOSInstrPseudos.td instead. These are
// more traditional pseudo-instructions; they may expand out to short snippets
// of code required to perform higher-level operations that cannot be expaneded
// inline, for one reason or another.
//
// All instructions take 1-register register classes for fixed operands: e.g.
// Ac, Xc, Yc, etc. This allows the each instruction to natively use virtual
// registers that can be resolved by the register allocator or register
// scavenger. NZ is not included, since it is handled via bundling and
// terminators; too many instructions clobber it for it to have useful live
// ranges.
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
  let isReMaterializable = true;
}

// Operand type for basic block and function labels.
def label : Operand<OtherVT>;

//===---------------------------------------------------------------------===//
// Addition/Subtraction Instructions
//===---------------------------------------------------------------------===//

class MOSAddSub : MOSLogicalInstr {
  dag OutOperandList = (outs Ac:$dst, Cc:$carryout, Vc:$vout);
  let Constraints = "$dst = $l, $carryout = $carryin";

  let Defs = [NZ];
}
class MOSAddSubOp<Operand op> : MOSAddSub {
  dag InOperandList = (ins Ac:$l, op:$r, Cc:$carryin);
}
class MOSAddSubRC<RegisterClass rc> : MOSAddSub {
  dag InOperandList = (ins Ac:$l, rc:$r, Cc:$carryin);
}

let isAdd = true in {
  def ADCImm : MOSAddSubOp<i8imm>, PseudoInstExpansion<(ADC_Immediate imm8:$r)>;
  def ADCImag8 : MOSAddSubRC<Imag8>, PseudoInstExpansion<(ADC_ZeroPage addr8:$r)> {
    // Don't give the carries a tied constraint, since having two tied operands
    // inhibits commutation of $l and $r.
    let Constraints = "$dst = $l";
    let isCommutable = true;
  }
}

def SBCImm : MOSAddSubOp<i8imm>, PseudoInstExpansion<(SBC_Immediate imm8:$r)>;
def SBCImag8 : MOSAddSubRC<Imag8>, PseudoInstExpansion<(SBC_ZeroPage addr8:$r)>;

//===---------------------------------------------------------------------===//
// Comparison Instructions
//===---------------------------------------------------------------------===//

// CMP imm, CPX imm, CPY imm
def CMPImm : MOSLogicalInstr {
  dag OutOperandList = (outs Cc:$carry);
  dag InOperandList = (ins GPR:$l, i8imm:$r);

  let Defs = [NZ];

  let isCompare = true;
}

//===---------------------------------------------------------------------===//
// BIT Instructions
//===---------------------------------------------------------------------===//
// Ever the odd duck.
//===---------------------------------------------------------------------===//

def BITAbs : MOSLogicalInstr {
  dag OutOperandList = (outs Vc:$v);
  dag InOperandList = (ins Ac:$l, i16imm:$r);

  let Defs = [NZ];
}

//===---------------------------------------------------------------------===//
// Bitwise instructions
//===---------------------------------------------------------------------===//

class MOSBitwiseInstr : MOSLogicalInstr {
  let OutOperandList = (outs Ac:$dst);
  let InOperandList = (ins Ac:$l, Imag8:$r);
  let Constraints = "$dst = $l";

  let Defs = [NZ];

  let isCommutable = true;
}

def ANDImag8 : MOSBitwiseInstr;
def EORImag8 : MOSBitwiseInstr;
def ORAImag8 : MOSBitwiseInstr;

//===---------------------------------------------------------------------===//
// Shift/Rotate Instructions
//===---------------------------------------------------------------------===//

class MOSShiftRotate : MOSLogicalInstr {
  dag OutOperandList = (outs AImag8:$dst, Cc:$carryout);
  let Defs = [NZ];
}

class MOSShift : MOSShiftRotate {
  dag InOperandList = (ins AImag8:$src);
  let Constraints = "$dst = $src";
}

class MOSRotate : MOSShiftRotate {
  dag InOperandList = (ins AImag8:$src, Cc:$carryin);
  let Constraints = "$dst = $src, $carryout = $carryin";
}

// ASL A, ASL zp
def ASL : MOSShift;
def LSR : MOSShift;

// ROL A, ROL zp
def ROL : MOSRotate;
def ROR : MOSRotate;

//===---------------------------------------------------------------------===//
// Increment/Decrement Instructions
//===---------------------------------------------------------------------===//

// INX, INY
def IN : MOSLogicalInstr {
  dag OutOperandList = (outs XY:$dst);
  dag InOperandList = (ins XY:$src);
  let Constraints = "$src = $dst";

  let Defs = [NZ];

  let Pattern = [(set XY:$dst, (add XY:$src, 1))];

  let isAdd = true;
}

//===---------------------------------------------------------------------===//
// Immediate Load Instructions
//===---------------------------------------------------------------------===//

class MOSImmediateLoad<RegisterClass dst, Operand val> : MOSLogicalInstr {
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = (ins val:$val);

  let isAsCheapAsAMove = true;
  let isMoveImm = true;
}

// CLC, SEC
def LDCImm : MOSImmediateLoad<Cc, i1imm>;

def CLV : MOSImmediateLoad<Vc, i1imm> {
  dag InOperandList = (ins);
}

// LDA imm, LDX imm, LDY imm
def LDImm : MOSImmediateLoad<GPR, i8imm> {
  let Defs = [NZ];
}

//===---------------------------------------------------------------------===//
// Register Transfer Instructions
//===---------------------------------------------------------------------===//

// Instructions that transfer from one register class to another.
class MOSTransfer<RegisterClass dst, RegisterClass src> : MOSLogicalInstr {
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = (ins src:$src);

  let Defs = [NZ];

  let isAsCheapAsAMove = true;
}

// LDA zp, LDX zp, LDY zp
def LDImag8 : MOSTransfer<GPR, Imag8>;
// STA zp, STX zp, STY zp
def STImag8 : MOSTransfer<Imag8, GPR> {
  // Unlike all other "register transfer" instructions, this doesn't clobber NZ,
  // since imaginary registers aren't real.
  let Defs = [];
}

// TAX, TAY
def TA : MOSTransfer<XY, Ac>;
// TXA, TYA
def T_A : MOSTransfer<Ac, XY>;

//===---------------------------------------------------------------------===//
// Stack Instructions
//===---------------------------------------------------------------------===//

// PHA, PHP
def PH : MOSLogicalInstr {
  dag InOperandList = (ins AP:$src);

  let mayStore = true;
}

// PLA, PLP
def PL : MOSLogicalInstr {
  dag OutOperandList = (outs AP:$dst);

  let Defs = [NZ];

  let mayLoad = true;
}

//===---------------------------------------------------------------------===//
// Control Flow Instructions
//===---------------------------------------------------------------------===//

// Covers all branch instructions. Branches to $tgt if $flag = $val.
def BR : MOSLogicalInstr {
  dag InOperandList = (ins label:$tgt, Flag:$flag, i1imm:$val);

  let isBranch = true;
  let isTerminator = true;
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

def RTS : MOSLogicalInstr, PseudoInstExpansion<(RTS_Implied)> {
  let isBarrier = true;
  let isReturn = true;
  let isTerminator = true;
}

//===---------------------------------------------------------------------===//
// Load/Store Instructions
//===---------------------------------------------------------------------===//

class MOSLoad : MOSLogicalInstr {
  let Defs = [NZ];
  let mayLoad = true;
}

// LDA abs, LDX abs, LDY abs
def LDAbs : MOSLoad {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins i16imm:$src);
}

class MOSLoadIndexed<RegisterClass dst, RegisterClass idx> : MOSLoad {
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = (ins i16imm:$addr, idx:$idx);
}
// LDA abs,x; LDA abs,y
def LDAIdx : MOSLoadIndexed<Ac, XY>;
// LDX abs,y
def LDXIdx : MOSLoadIndexed<Xc, Yc>,
             PseudoInstExpansion<(LDX_AbsoluteY addr16:$addr)>;
// LDY abs,x
def LDYIdx : MOSLoadIndexed<Yc, Xc>,
             PseudoInstExpansion<(LDY_AbsoluteX addr16:$addr)>;

// LDA (zp),y
def LDYIndir : MOSLoad, PseudoInstExpansion<(LDA_IndirectIndexed addr8:$addr)> {
  dag OutOperandList = (outs Ac:$dst);
  dag InOperandList = (ins Imag16:$addr, Yc:$offset);
}

class MOSStore : MOSLogicalInstr {
  let mayStore = true;
}
// STA abs, STX abs, STZ abs
def STAbs : MOSStore {
  dag InOperandList = (ins GPR:$src, i16imm:$dst);
}
// STA abs,x; STA abs,y
def STIdx : MOSStore {
  dag InOperandList = (ins Ac:$src, i16imm:$addr, XY:$idx);
}
// STA (zp),y
def STYIndir : MOSStore, PseudoInstExpansion<(STA_IndirectIndexed addr8:$addr)> {
  dag InOperandList = (ins Ac:$src, Imag16:$addr, Yc:$offset);
}
