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
// scavenger. NZ is modeled implicitly for the most part; too many instructions
// clobber it for it to be useful outside very specific scenarios.
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

//===---------------------------------------------------------------------===//
// Addition/Subtraction Instructions
//===---------------------------------------------------------------------===//

class MOSAddSub : MOSLogicalInstr {
  dag OutOperandList = (outs Ac:$dst, Cc:$carryout, Vc:$vout);
  let Constraints = "$dst = $l, $carryout = $carryin";
}
class MOSAddSubOp<Operand op> : MOSAddSub {
  dag InOperandList = (ins Ac:$l, op:$r, Cc:$carryin);
}
class MOSAddSubRC<RegisterClass rc> : MOSAddSub {
  dag InOperandList = (ins Ac:$l, rc:$r, Cc:$carryin);
}

let isAdd = true in {
  def ADCImm : MOSAddSubOp<imm8>, PseudoInstExpansion<(ADC_Immediate imm8:$r)>;
  def ADCImag8 : MOSAddSubRC<Imag8>, PseudoInstExpansion<(ADC_ZeroPage addr8:$r)> {
    // Don't give the carries a tied constraint, since having two tied operands
    // inhibits commutation of $l and $r.
    let Constraints = "$dst = $l";
    let isCommutable = true;
  }
  let mayLoad = true in {
    def ADCZpIdx : MOSAddSub {
      dag InOperandList = (ins Ac:$l, addr8:$addr, Xc:$idx, Cc:$carryin);
    }
    def ADCAbs :
      MOSAddSubOp<addr16>, PseudoInstExpansion<(ADC_ZeroPage addr8:$r)>;
    def ADCAbsIdx : MOSAddSub {
      dag InOperandList = (ins Ac:$l, addr16:$addr, XY:$idx, Cc:$carryin);
    }
    def ADCIndir :
        MOSAddSub, PseudoInstExpansion<(ADC_Indirect addr8:$addr)> {
      let Predicates = [Has65C02];
      dag InOperandList = (ins Ac:$l, Imag16:$addr, Cc:$carryin);
    }
    def ADCIndirIdx :
        MOSAddSub, PseudoInstExpansion<(ADC_IndirectIndexed addr8:$addr)> {
      dag InOperandList = (ins Ac:$l, Imag16:$addr, Yc:$idx, Cc:$carryin);
    }
  }
}

def SBCImm : MOSAddSubOp<imm8>, PseudoInstExpansion<(SBC_Immediate imm8:$r)>;
def SBCImag8 : MOSAddSubRC<Imag8>, PseudoInstExpansion<(SBC_ZeroPage addr8:$r)>;
let mayLoad = true in {
  def SBCZpIdx : MOSAddSub {
    dag InOperandList = (ins Ac:$l, addr8:$addr, Xc:$idx, Cc:$carryin);
  }
  def SBCAbs :
      MOSAddSubOp<addr16>, PseudoInstExpansion<(SBC_ZeroPage addr8:$r)>;
  def SBCAbsIdx : MOSAddSub {
    dag InOperandList = (ins Ac:$l, addr16:$addr, XY:$idx, Cc:$carryin);
  }
  def SBCIndir :
      MOSAddSub, PseudoInstExpansion<(SBC_Indirect addr8:$addr)> {
    let Predicates = [Has65C02];
    dag InOperandList = (ins Ac:$l, Imag16:$addr, Cc:$carryin);
  }
  def SBCIndirIdx :
      MOSAddSub, PseudoInstExpansion<(SBC_IndirectIndexed addr8:$addr)> {
    dag InOperandList = (ins Ac:$l, Imag16:$addr, Yc:$idx, Cc:$carryin);
  }
}

//===---------------------------------------------------------------------===//
// Bitwise instructions
//===---------------------------------------------------------------------===//

class MOSBitwiseBase : MOSLogicalInstr {
  let OutOperandList = (outs Ac:$dst);
  let Constraints = "$dst = $l";
}

multiclass MOSBitwiseInstr<SDNode node> {
  def Imm :
      MOSBitwiseBase,
      PseudoInstExpansion<(!cast<Instruction>(NAME#"_Immediate") imm8:$r)> {
    let OutOperandList = (outs Ac:$dst);
    let InOperandList = (ins Ac:$l, imm8:$r);

    let Pattern = [(set Ac:$dst, (node Ac:$l, imm:$r))];
  }

  def Imag8 :
      MOSBitwiseBase,
      PseudoInstExpansion<(!cast<Instruction>(NAME#"_ZeroPage") addr8:$r)> {
    let InOperandList = (ins Ac:$l, Imag8:$r);

    let Pattern = [(set Ac:$dst, (node Ac:$l, Imag8:$r))];

    let isCommutable = true;
  }

  let mayLoad = true in {
    def ZpIdx : MOSBitwiseBase {
      let InOperandList = (ins Ac:$l, addr8:$addr, Xc:$idx);
    }

    def Abs :
        MOSBitwiseBase,
        PseudoInstExpansion<(!cast<Instruction>(NAME#"_ZeroPage") addr8:$r)> {
      let InOperandList = (ins Ac:$l, addr16:$r);
    }

    def AbsIdx : MOSBitwiseBase {
      let InOperandList = (ins Ac:$l, addr16:$addr, XY:$idx);
    }

    def Indir :
       MOSBitwiseBase,
       PseudoInstExpansion<
           (!cast<Instruction>(NAME#"_Indirect") addr8:$addr)> {
      let InOperandList = (ins Ac:$l, Imag16:$addr);
    }

    def IndirIdx :
       MOSBitwiseBase,
       PseudoInstExpansion<
           (!cast<Instruction>(NAME#"_IndirectIndexed") addr8:$addr)> {
      let InOperandList = (ins Ac:$l, Imag16:$addr, Yc:$idx);
    }
  }
}

defm AND : MOSBitwiseInstr<and>;
defm EOR : MOSBitwiseInstr<xor>;
defm ORA : MOSBitwiseInstr<or>;

//===---------------------------------------------------------------------===//
// Shift/Rotate Instructions
//===---------------------------------------------------------------------===//

class MOSShiftRotate : MOSLogicalInstr {
  dag OutOperandList = (outs AImag8:$dst, Cc:$carryout);
}

class MOSShift : MOSShiftRotate {
  dag InOperandList = (ins AImag8:$src);
  let Constraints = "$dst = $src";
}
class MOSRotate : MOSShiftRotate {
  dag InOperandList = (ins AImag8:$src, Cc:$carryin);
  let Constraints = "$dst = $src, $carryout = $carryin";
}

class MOSShiftRotateRMW : MOSLogicalInstr {
  dag OutOperandList = (outs Cc:$carryout);
  let mayLoad = true;
  let mayStore = true;
}

class MOSShiftRMW : MOSShiftRotateRMW {}
class MOSRotateRMW : MOSShiftRotateRMW {
  let Constraints = "$carryout = $carryin";
}

def ASL : MOSShift;
def LSR : MOSShift;

def ASLAbs : MOSShiftRMW, PseudoInstExpansion<(ASL_ZeroPage addr8:$addr)> {
  dag InOperandList = (ins addr16:$addr);
}
def ASLIdx : MOSShiftRMW {
  dag InOperandList = (ins addr16:$addr, Xc:$idx);
}
def LSRAbs : MOSShiftRMW, PseudoInstExpansion<(LSR_ZeroPage addr8:$addr)> {
  dag InOperandList = (ins addr16:$addr);
}
def LSRIdx : MOSShiftRMW {
  dag InOperandList = (ins addr16:$addr, Xc:$idx);
}

def ROL : MOSRotate;
def ROR : MOSRotate;

def ROLAbs : MOSRotateRMW, PseudoInstExpansion<(ROL_ZeroPage addr8:$addr)> {
  dag InOperandList = (ins addr16:$addr, Cc:$carryin);
}
def ROLIdx : MOSRotateRMW {
  dag InOperandList = (ins addr16:$addr, Xc:$idx, Cc:$carryin);
}
def RORAbs : MOSRotateRMW, PseudoInstExpansion<(ROR_ZeroPage addr8:$addr)> {
  dag InOperandList = (ins addr16:$addr, Cc:$carryin);
}
def RORIdx : MOSRotateRMW {
  dag InOperandList = (ins addr16:$addr, Xc:$idx, Cc:$carryin);
}

//===---------------------------------------------------------------------===//
// BIT Instructions
//===---------------------------------------------------------------------===//
// Ever the odd duck.
//===---------------------------------------------------------------------===//

def BITAbs : MOSLogicalInstr, PseudoInstExpansion<(BIT_ZeroPage addr8:$r)> {
  dag OutOperandList = (outs Vc:$v);
  dag InOperandList = (ins Ac:$l, addr16:$r);
  let mayLoad = true;
}

def BITImmHUC6280 : MOSLogicalInstr, PseudoInstExpansion<(BIT_Immediate imm8:$r)> {
  let Predicates = [Has65C02];

  dag OutOperandList = (outs Vc:$v);
  dag InOperandList = (ins Ac:$l, imm8:$r);
}

//===---------------------------------------------------------------------===//
// Control Flow Instructions
//===---------------------------------------------------------------------===//

class MOSBranch : MOSLogicalInstr {
  let isBranch = true;
  let isTerminator = true;
}

// Covers all branch instructions. Branches to $tgt if $flag = $val.
def BR : MOSBranch {
  dag InOperandList = (ins label:$tgt, Flag:$flag, i1imm:$val);
}

class MOSUnconditionalBranch : MOSBranch {
  let isBarrier = true;
}

def BRA : MOSUnconditionalBranch {
  let Predicates = [HasBRA];

  let InOperandList = (ins label:$tgt);
  let Pattern = [(br bb:$tgt)];
}

def JMP : MOSUnconditionalBranch, PseudoInstExpansion<(JMP_Absolute addr16:$tgt)> {
  let InOperandList = (ins label:$tgt);
  let Pattern = [(br bb:$tgt)];
}

def JMPIndir : MOSUnconditionalBranch, PseudoInstExpansion<(JMP_Indirect16 addr16:$tgt)> {
  let InOperandList = (ins Imag16:$tgt);
  let isIndirectBranch = true;
}

def JMPIdxIndir : MOSUnconditionalBranch, PseudoInstExpansion<(JMP_IndexedIndirect addr16:$tgt)> {
  let Predicates = [HasJMPIdxIndir];
  let InOperandList = (ins addr16:$tgt, Xc:$idx);
  let isIndirectBranch = true;
}

def JSR : MOSLogicalInstr, PseudoInstExpansion<(JSR_Absolute addr16:$tgt)> {
  dag InOperandList = (ins label:$tgt);

  let isCall = true;
}

def TailJMP : MOSLogicalInstr, PseudoInstExpansion<(JMP_Absolute addr16:$tgt)> {
  dag InOperandList = (ins label:$tgt);

  let isBarrier = true;
  let isTerminator = true;
  let isCall = true;
  let isReturn = true;
}

class MOSReturn<Instruction Opcode> : MOSLogicalInstr, PseudoInstExpansion<(Opcode)> {
  let isBarrier = true;
  let isReturn = true;
  let isTerminator = true;
}

def RTS : MOSReturn<RTS_Implied>;
def RTI : MOSReturn<RTI_Implied>;

//===---------------------------------------------------------------------===//
// Immediate Load Instructions
//===---------------------------------------------------------------------===//

class MOSImmediateLoad<RegisterClass dst, Operand val> : MOSLogicalInstr {
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = (ins val:$val);

  let isAsCheapAsAMove = true;
  let isMoveImm = true;
}

def CLV : MOSImmediateLoad<Vc, i1imm>, PseudoInstExpansion<(CLV_Implied)> {
  dag InOperandList = (ins);
}

// CLC, SEC
def LDCImm : MOSImmediateLoad<Cc, i1imm>;

// LDA imm, LDX imm, LDY imm
def LDImm : MOSImmediateLoad<GPR, imm8>;

// CLA, CLX, CLY
let Predicates = [HasHUC6280] in {
  def CL : MOSImmediateLoad<GPR, imm8> {
    dag InOperandList = (ins);
  }
}

//===---------------------------------------------------------------------===//
// Comparison Instructions
//===---------------------------------------------------------------------===//

class MOSCMP : MOSLogicalInstr {
  dag OutOperandList = (outs Cc:$carry);
  let isCompare = true;
}

def CMPImm : MOSCMP { dag InOperandList = (ins GPR:$l, imm8:$r); }
def CMPImag8 : MOSCMP { dag InOperandList = (ins GPR:$l, Imag8:$r); }

let mayLoad = true in {
  def CMPZpIdx : MOSCMP { dag InOperandList = (ins Ac:$l, addr8:$addr, Xc:$idx); }
  def CMPAbs : MOSCMP { dag InOperandList = (ins GPR:$l, addr16:$addr); }
  def CMPAbsIdx : MOSCMP { dag InOperandList = (ins Ac:$l, addr16:$addr, XY:$idx); }
  def CMPIndir : MOSCMP, PseudoInstExpansion<(CMP_Indirect addr8:$addr)> {
    let Predicates = [Has65C02];
    dag InOperandList = (ins Ac:$l, Imag16:$addr);
  }
  def CMPIndirIdx : MOSCMP, PseudoInstExpansion<(CMP_IndirectIndexed addr8:$addr)> {
    dag InOperandList = (ins Ac:$l, Imag16:$addr, Yc:$idx);
  }
}

//===---------------------------------------------------------------------===//
// Increment/Decrement Instructions
//===---------------------------------------------------------------------===//

multiclass MOSIncDec {
  def NAME : MOSLogicalInstr {
    dag OutOperandList = (outs XY:$dst);
    dag InOperandList = (ins XY:$src);
    let Constraints = "$src = $dst";
  }

  def _CMOS : MOSLogicalInstr {
    dag OutOperandList = (outs GPR:$dst);
    dag InOperandList = (ins GPR:$src);
    let Constraints = "$src = $dst";
    let Predicates = [HasGPRIncDec];
  }

  def CImag8 : MOSLogicalInstr, PseudoInstExpansion<(!cast<Instruction>(NAME # C_ZeroPage) addr8:$dst)> {
    dag OutOperandList = (outs Imag8:$dst);
    dag InOperandList = (ins Imag8:$src);
    let Constraints = "$src = $dst";
  }

  let mayLoad = true, mayStore = true in {
    def CAbs : MOSLogicalInstr, PseudoInstExpansion<(!cast<Instruction>(NAME # C_ZeroPage) addr8:$addr)> {
      dag InOperandList = (ins addr16:$addr);
    }
    def CIdx : MOSLogicalInstr {
      dag InOperandList = (ins addr16:$addr, Xc:$idx);
    }
  }
}

let mayLoad = true, mayStore = true in {
  def DCPImag8 : MOSLogicalInstr, PseudoInstExpansion<(!cast<Instruction>(DCP_ZeroPage) addr8:$dst)> {
    dag OutOperandList = (outs Cc:$carry);
    dag InOperandList = (ins Ac:$value, Imag8:$dst);
  }

  def DCPAbs : MOSLogicalInstr, PseudoInstExpansion<(!cast<Instruction>(DCP_ZeroPage) addr8:$addr)> {
    dag OutOperandList = (outs Cc:$carry);
    dag InOperandList = (ins Ac:$value, addr16:$addr);
  }
}

// DEX, DEY, DEC
defm DE : MOSIncDec;

// INX, INY, INC
let isAdd = true in {
  defm IN : MOSIncDec;
}

//===---------------------------------------------------------------------===//
// Register Transfer Instructions
//===---------------------------------------------------------------------===//

// Instructions that transfer from one register class to another.
class MOSTransfer<RegisterClass dst, RegisterClass src> : MOSLogicalInstr {
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = (ins src:$src);

  let isAsCheapAsAMove = true;
}

// LDA zp, LDX zp, LDY zp
def LDImag8 : MOSTransfer<GPR, Imag8>;
// STA zp, STX zp, STY zp
def STImag8 : MOSTransfer<Imag8, GPR>;
// MOV zp, zp
def MOVImag8 : MOSTransfer<Imag8, Imag8>,
    PseudoInstExpansion<(SPC700_MOV_ZeroPageZeroPage addr8:$dst, addr8at2:$src)> {
  let Predicates = [HasSPC700];
}

// TAX, TAY
def TA : MOSTransfer<XY, Ac>;
// TXA, TYA
def T_A : MOSTransfer<Ac, XY>;

let Predicates = [HasW65816Or65EL02] in {
// TXY, TYX
def TX : MOSTransfer<XY, XY>;
}

let Predicates = [HasHUC6280] in {
// SAX, SAY, SXY
def SWAP : MOSLogicalInstr {
  dag OutOperandList = (outs GPR:$dst2, GPR:$dst1);
  dag InOperandList = (ins GPR:$src1, GPR:$src2);
  let Constraints = "$src1 = $dst1, $src2 = $dst2";

  let isAsCheapAsAMove = true;
}
}

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

  let mayLoad = true;
}

let Predicates = [HasGPRStackRegs] in {
  // PHA, PHP
  def PH_CMOS : MOSLogicalInstr {
    dag InOperandList = (ins AXYP:$src);

    let mayStore = true;
  }

  // PLA, PLX, PLY, PLP
  def PL_CMOS : MOSLogicalInstr {
    dag OutOperandList = (outs AXYP:$dst);

    let mayLoad = true;
  }
}

//===---------------------------------------------------------------------===//
// Load/Store Instructions
//===---------------------------------------------------------------------===//

class MOSLoad : MOSLogicalInstr {
  let mayLoad = true;
}

// LDA abs, LDX abs, LDY abs
def LDAbs : MOSLoad {
  dag OutOperandList = (outs GPR:$dst);
  dag InOperandList = (ins addr16:$src);
}

class MOSLoadIndexed<RegisterClass dst, RegisterClass idx> : MOSLoad {
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = (ins addr16:$addr, idx:$idx);
}
// LDA zp,x
def LDAZpIdx : MOSLoadIndexed<Ac, Xc>;
// LDA abs,x; LDA abs,y
def LDAAbsIdx : MOSLoadIndexed<Ac, XY>;
// LDX abs,y
def LDXIdx : MOSLoadIndexed<Xc, Yc>;
// LDY abs,x
def LDYIdx : MOSLoadIndexed<Yc, Xc>;

// LDA (zp)
def LDIndir : MOSLoad, PseudoInstExpansion<(LDA_Indirect addr8:$addr)> {
  let Predicates = [Has65C02];
  dag OutOperandList = (outs Ac:$dst);
  dag InOperandList = (ins Imag16:$addr);
}

// LDA (zp),y
def LDIndirIdx : MOSLoad, PseudoInstExpansion<(LDA_IndirectIndexed addr8:$addr)> {
  dag OutOperandList = (outs Ac:$dst);
  dag InOperandList = (ins Imag16:$addr, Yc:$offset);
}

class MOSStore : MOSLogicalInstr {
  let mayStore = true;
}
// STA zp,x
def STZpIdx : MOSStore {
  dag InOperandList = (ins Ac:$src, addr8:$addr, Xc:$idx);
}
// STA abs, STX abs
def STAbs : MOSStore {
  dag InOperandList = (ins GPR:$src, addr16:$dst);
}
// STA abs,x; STA abs,y
def STAbsIdx : MOSStore {
  dag InOperandList = (ins Ac:$src, addr16:$addr, XY:$idx);
}

// STA (zp)
def STIndir : MOSStore, PseudoInstExpansion<(STA_Indirect addr8:$addr)> {
  let Predicates = [Has65C02];
  dag InOperandList = (ins Ac:$src, Imag16:$addr);
}

// STA (zp),y
def STIndirIdx : MOSStore, PseudoInstExpansion<(STA_IndirectIndexed addr8:$addr)> {
  dag InOperandList = (ins Ac:$src, Imag16:$addr, Yc:$offset);
}

let Predicates = [Has65C02] in {
  // STZ abs
  def STZAbs : MOSStore, PseudoInstExpansion<(STZ_ZeroPage addr8:$addr)> {
    dag InOperandList = (ins addr16:$addr);
  }
  // STZ abs,x
  def STZIdx : MOSStore {
    dag InOperandList = (ins addr16:$addr, Xc:$idx);
  }
  // STZ zp
  def STZImag8 : MOSStore, PseudoInstExpansion<(STZ_ZeroPage addr8:$addr)> {
    dag OutOperandList = (outs Imag8:$addr);
  }
}

//===---------------------------------------------------------------------===//
// HuC Block Copy Instructions
//===---------------------------------------------------------------------===//

let Predicates = [HasHUC6280] in {
  // (TII, TDD) abs, abs, imm16
  def HuCMemcpy : MOSLogicalInstr {
    dag InOperandList = (ins addr16:$source, addr16:$dest, imm16:$length,
                         i1imm:$descending);

    let mayLoad = true;
    let mayStore = true;
  }
}

//===---------------------------------------------------------------------===//
// Addition/Subtraction Patterns
//===---------------------------------------------------------------------===//

def : Pat<(add Ac:$l, (i8 imm:$imm)), (ADCImm Ac:$l, imm:$imm, (LDCImm 0))>;
def : Pat<(add Ac:$l, Imag8:$r), (ADCImag8 Ac:$l, Imag8:$r, (LDCImm 0))>;
def : Pat<(sub Ac:$l, (i8 imm:$imm)), (SBCImm Ac:$l, imm:$imm, (LDCImm -1))>;
def : Pat<(sub Ac:$l, Imag8:$r), (SBCImag8 Ac:$l, Imag8:$r, (LDCImm -1))>;

//===---------------------------------------------------------------------===//
// Integer Extension/Truncation Patterns
//===---------------------------------------------------------------------===//

def : Pat<(i16 (anyext Anyi8:$s)), (INSERT_SUBREG (IMPLICIT_DEF), Anyi8:$s, sublo)>;

def : Pat<(i1 (trunc Ac:$s)), (EXTRACT_SUBREG (ANDImm Ac:$s, 1), sublsb)>;
def : Pat<(i8 (trunc Imag16:$s)), (EXTRACT_SUBREG Imag16:$s, sublo)>;

//===---------------------------------------------------------------------===//
// Constant Patterns
//===---------------------------------------------------------------------===//

def : Pat<(i1 imm:$s), (LDImm1 imm:$s)>;
def : Pat<(i8 imm:$s), (LDImm imm:$s)>;
