//===-- MOSInstrInfo.td - MOS Instruction Formats ----------*- tablegen -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// MOS Instruction Format Definitions.
//
//===----------------------------------------------------------------------===//

/// Predicates. Useful for limiting instructions to particular hardware modes
/// or particular hardware implementations.
def Has6502 : Predicate<"Subtarget->has6502()">,
              AssemblerPredicate<(all_of Feature6502), "6502"> {
  let PredicateName = "Feature6502";
}

def Has6502X : Predicate<"Subtarget->has6502X()">,
               AssemblerPredicate<(all_of Feature6502X), "6502X"> {
  let PredicateName = "Feature6502X";
}

def Has65C02 : Predicate<"Subtarget->has65C02()">,
               AssemblerPredicate<(all_of Feature65C02), "65C02"> {
  let PredicateName = "Feature65C02";
}

def Has65CE02 : Predicate<"Subtarget->has65CE02()">,
               AssemblerPredicate<(all_of Feature65CE02), "65CE02"> {
  let PredicateName = "Feature65CE02";
}

def HasHUC6280 : Predicate<"Subtarget->hasHUC6280()">,
               AssemblerPredicate<(all_of FeatureHUC6280), "HuC6280"> {
  let PredicateName = "FeatureHUC6280";
}

def HasR65C02 : Predicate<"Subtarget->hasR65C02()">,
               AssemblerPredicate<(all_of FeatureR65C02), "R65C02"> {
  let PredicateName = "FeatureR65C02";
}

def HasW65C02 : Predicate<"Subtarget->hasW65C02()">,
               AssemblerPredicate<(all_of FeatureW65C02), "W65C02"> {
  let PredicateName = "FeatureW65C02";
}

def HasW65816Or65EL02 : Predicate<"(Subtarget->hasW65816() || Subtarget->has65EL02())">,
                AssemblerPredicate<(any_of FeatureW65816, Feature65EL02), "W65816 or 65EL02"> {
  let PredicateName = "FeatureW65816Or65EL02";
}

def HasW65816 : Predicate<"Subtarget->hasW65816()">,
                AssemblerPredicate<(all_of FeatureW65816), "W65816"> {
  let PredicateName = "FeatureW65816";
}

def Has65EL02 : Predicate<"Subtarget->has65EL02()">,
                AssemblerPredicate<(all_of Feature65EL02), "65EL02"> {
  let PredicateName = "Feature65EL02";
}

def Has6502XOrDTV : Predicate<"(Subtarget->has6502X() || Subtarget->has65DTV02())">,
                AssemblerPredicate<(any_of Feature6502X, Feature65DTV02), "6502X (C64DTV subset)"> {
  let PredicateName = "Feature6502XOrDTV";
}

def Has65DTV02 : Predicate<"Subtarget->has65DTV02()">,
                AssemblerPredicate<(all_of Feature65DTV02), "6502 (C64DTV)"> {
  let PredicateName = "Feature65DTV02";
}

def Has4510 : Predicate<"Subtarget->has4510()">,
                AssemblerPredicate<(all_of Feature4510), "4510"> {
  let PredicateName = "Feature4510";
}

def Has45GS02 : Predicate<"Subtarget->has45GS02()">,
                AssemblerPredicate<(all_of Feature45GS02), "45GS02"> {
  let PredicateName = "Feature45GS02";
}

def HasSPC700 : Predicate<"Subtarget->hasSPC700()">,
               AssemblerPredicate<(all_of FeatureSPC700), "SPC700"> {
  let PredicateName = "FeatureSPC700";
}

def HasBRA : Predicate<"Subtarget->hasBRA()">;
def HasGPRStackRegs : Predicate<"Subtarget->hasGPRStackRegs()">;
def HasGPRIncDec : Predicate<"Subtarget->hasGPRIncDec()">;
def HasJMPIdxIndir : Predicate<"Subtarget->hasJMPIdxIndir()">;

/// These instruction format definitions exist, thanks to Neil Parker's
/// analysis of the 65xx instruction set, at:
/// http://nparker.llx.com/a2/opcodes.html
/// Parker's analysis is closely reflected in the structure of this file, and
/// his words are quoted liberally herein.  Parker's sense-making of the 6502
/// and family was critical in creating these format definitions, and we're
/// indebted to him for the excellent analysis.

/// The base MOS instruction class, from which all instructions descend.
class Inst<string asmstr> : Instruction {
  let Namespace = "MOS";

  // None of the MC instructions are considered to have any output operands.
  // Side-effects of the MC instructions
  let OutOperandList = (outs);

  let AsmString = asmstr;

  // Note: These properties are not tracked for the real instruction set, which
  // is used only in the MC layer. See the logical instruction set for the
  // definitions of these properties.
  let mayLoad = true;
  let mayStore = true;
  let hasSideEffects = true;

  field bits<56> SoftFail = 0;
  let DecoderNamespace = "MOS";

  bit MLow = 0;
  bit MHigh = 0;
  bit XLow = 0;
  bit XHigh = 0;
  let TSFlags{0} = MLow;
  let TSFlags{1} = MHigh;
  let TSFlags{2} = XLow;
  let TSFlags{3} = XHigh;
}

class InstConditionalBranch {
  bit isBranch = true;
  bit isTerminator = true;
}

class InstUnconditionalBranch : InstConditionalBranch {
  bit isBarrier = true;
}

class InstCall {
  bit isCall = true;
}

class InstReturn {
  bit isBarrier = true;
  bit isReturn = true;
  bit isTerminator = true;
}

/// Makes sure that immediate mode operands fit into the required bit field
/// size.  See also *isImm8 functions in MOS code.
class MOSAsmOperand<string name> : AsmOperandClass {
  let Name = name;
  let DiagnosticType = !strconcat("Invalid", name);
}

class ImmediateAsmOperand<string name> : MOSAsmOperand<name> {
    let RenderMethod = "addImmOperands";
    let DiagnosticType = "immediate";
  }

class PCRelativeOperand<string name> : MOSAsmOperand<name>;

/// An 8-bit (i.e. zero page) address.
class Addr8Operand<string name> : MOSAsmOperand<name>;

/// A 16-bit address.
class Addr16Operand<string name> : MOSAsmOperand<name>;

/// A 24-bit address.
class Addr24Operand<string name> : MOSAsmOperand<name>;

/// This operand will only match a value from 0 to 7 inclusive.
def imm3 : Operand<i32> {
  let ParserMatchClass = ImmediateAsmOperand<"Imm3">;
  let DecoderMethod = "decodeUImmOperand<3>";
  let OperandType = "OPERAND_IMM3";
  let OperandNamespace = "MOSOp";
  let Type = i8;
}

/// This operand will only match a value from 0 to 15 inclusive.
def imm4 : Operand<i32> {
  let ParserMatchClass = ImmediateAsmOperand<"Imm4">;
  let DecoderMethod = "decodeUImmOperand<4>";
  let OperandType = "OPERAND_IMM4";
  let OperandNamespace = "MOSOp";
  let Type = i8;
}

/// This operand will only match a value from 0 to 255 inclusive.
class imm8at<int offset> : Operand<i32> {
  let ParserMatchClass = ImmediateAsmOperand<"Imm8">;
  let EncoderMethod = "encodeImm<MOS::Imm8, " # offset # ">";
  let DecoderMethod = "decodeUImmOperand<8>";
  let OperandType = "OPERAND_IMM8";
  let OperandNamespace = "MOSOp";
  let Type = i8;
}
def imm8 : imm8at<1>;
def imm8at2 : imm8at<2>;

/// This operand will only match a value from 0 to 65535 inclusive.
class imm16at<int offset> : Operand<i32> {
  let ParserMatchClass = ImmediateAsmOperand<"Imm16">;
  let EncoderMethod = "encodeImm<MOS::Imm16, " # offset # ">";
  let DecoderMethod = "decodeUImmOperand<16>";
  let OperandType = "OPERAND_IMM16";
  let OperandNamespace = "MOSOp";
  let Type = i16;
}
def imm16 : imm16at<1>;
def imm16at5 : imm16at<5>;

/// An 8-bit pc-relative reference, for branch instructions.
class pcrel8at<int offset> : Operand<i32> {
  let ParserMatchClass = PCRelativeOperand<"PCRel8">;
  let EncoderMethod = "encodeImm<MOS::PCRel8, " # offset # ">";
  let DecoderMethod = "decodeSImmOperand<8>";
  let PrintMethod = "printBranchOperand";
  let OperandType = "OPERAND_PCREL";
}
def pcrel8 : pcrel8at<1>;
def pcrel8at2 : pcrel8at<2>;

/// A 16-bit pc-relative reference, for 65816 BRL instruction.
class pcrel16at<int offset> : Operand<i32> {
  let ParserMatchClass = PCRelativeOperand<"PCRel16">;
  let EncoderMethod = "encodeImm<MOS::PCRel16, " # offset # ">";
  let DecoderMethod = "decodeSImmOperand<16>";
  let PrintMethod = "printBranchOperand";
  let OperandType = "OPERAND_PCREL";
}
def pcrel16 : pcrel16at<1>;

class addr8at<int offset> : Operand<i32> {
  let ParserMatchClass = Addr8Operand<"Addr8">;
  let EncoderMethod = "encodeImm<MOS::Addr8, " # offset # ">";
  let DecoderMethod = "decodeAddr8Operand";
  let OperandType = "OPERAND_ADDR8";
  let OperandNamespace = "MOSOp";
  let Type = i8;
}
def addr8 : addr8at<1>;
def addr8at2 : addr8at<2>;
def addr8at3 : addr8at<3>;
def addr8at4 : addr8at<4>;

class addr16at<int offset> : Operand<i32> {
  let ParserMatchClass = Addr16Operand<"Addr16">;
  let EncoderMethod = "encodeImm<MOS::Addr16, " # offset # ">";
  let DecoderMethod = "decodeUImmOperand<16>";
  let OperandType = "OPERAND_ADDR16";
  let OperandNamespace = "MOSOp";
  let Type = i16;
}
def addr16 : addr16at<1>;
def addr16at2 : addr16at<2>;
def addr16at3 : addr16at<3>;

class addr24at<int offset> : Operand<i32> {
  let ParserMatchClass = Addr24Operand<"Addr24">;
  let EncoderMethod = "encodeImm<MOS::Addr24, " # offset # ">";
  let DecoderMethod = "decodeUImmOperand<24>";
  let OperandType = "OPERAND_ADDR24";
  let OperandNamespace = "MOSOp";
}
def addr24 : addr24at<1>;

/// All non-virtual MOS opcodes are 8 bits long.
class Opcode<bits<8> opcode = 0> {
  bits<8> op = opcode;
}
def DefaultOpcode : Opcode<0>;

/// Per Parker, opcodes on the MOS series tend to fit into the pattern aaabbbcc
/// where bbb is the addressing mode, and aaacc describe the instruction in
/// question. This is not a fixed rule however, especially for later processors
/// in the MOS series.
/// Parker: "Most instructions that explicitly reference memory locations have
/// bit patterns of the form aaabbbcc. The aaa and cc bits determine the
/// opcode, and the bbb bits determine the addressing mode."
class OpcodeABC<bits<3> aaa = 0, bits<3> bbb = 0, bits<2> cc = 0> : Opcode<0> {
  let op{7-5} = aaa;
  let op{4-2} = bbb;
  let op{1-0} = cc;
}

/// Instructions tend to break down into similar addressing modes by the cc
/// bits. So we deal with base instruction classes in four groups.
class OpcodeC0<bits<3> aaa = 0, bits<3> bbb = 0> : OpcodeABC<aaa, bbb, 0b00>;
class OpcodeC1<bits<3> aaa = 0, bits<3> bbb = 0> : OpcodeABC<aaa, bbb, 0b01>;
class OpcodeC2<bits<3> aaa = 0, bits<3> bbb = 0> : OpcodeABC<aaa, bbb, 0b10>;
class OpcodeC3<bits<3> aaa = 0, bits<3> bbb = 0> : OpcodeABC<aaa, bbb, 0b11>;

/// Addressing modes
/// ----------------
/// Addressing modes on 65xx do not perfectly map to bit fields across all
/// instructions. So we define the concepts of the addressing modes here, and
/// define how the operands should be parsed per addressing mode.
/// I wrote the operands in the a i r y format because tablegen seems to like
/// to collapse multiple characters into one token if they are not separated
/// by spaces.  We'll remove those spaces when we print out the operands for
/// any instruction. See also MOSInstPrinter.cpp.
class AddressingMode {
  string OperandsStr = ?;
  dag InOperandList = ?;
}

def Implicit : AddressingMode {
  string OperandsStr = "";
  dag InOperandList = (ins);
}

def Accumulator : AddressingMode {
  string OperandsStr = "";
  dag InOperandList = (ins);
}

def Immediate : AddressingMode {
  let OperandsStr = "#$param";
  let InOperandList = (ins imm8:$param);
}

def Immediate16 : AddressingMode {
  let OperandsStr = "#$param";
  let InOperandList = (ins imm16:$param);
}

def ZeroPage : AddressingMode {
  let OperandsStr = "$param";
  let InOperandList = (ins addr8:$param);
}

def ZeroPageX : AddressingMode {
  let OperandsStr = "$param , x";
  let InOperandList = (ins addr8:$param);
}

def ZeroPageY : AddressingMode {
  let OperandsStr = "$param , y";
  let InOperandList = (ins addr8:$param);
}

def Relative : AddressingMode {
  let OperandsStr = "$param";
  let InOperandList = (ins pcrel8:$param);
}

def Relative16 : AddressingMode {
  let OperandsStr = "$param";
  let InOperandList = (ins pcrel16:$param);
}

def Absolute : AddressingMode {
  let OperandsStr = "$param";
  let InOperandList = (ins addr16:$param);
}

def AbsoluteLong : AddressingMode {
  let OperandsStr = "$param";
  let InOperandList = (ins addr24:$param);
}

def AbsoluteX : AddressingMode {
  let OperandsStr = "$param , x";
  let InOperandList = (ins addr16:$param);
}

def AbsoluteXLong : AddressingMode {
  let OperandsStr = "$param , x";
  let InOperandList = (ins addr24:$param);
}

def AbsoluteY : AddressingMode {
  let OperandsStr = "$param , y";
  let InOperandList = (ins addr16:$param);
}

def Indirect : AddressingMode {
  let OperandsStr = "( $param )";
  let InOperandList = (ins addr8:$param);
}

def Indirect16 : AddressingMode {
  let OperandsStr = "( $param )";
  let InOperandList = (ins addr16:$param);
}

def IndexedIndirect16 : AddressingMode {
  let OperandsStr = "( $param , x )";
  let InOperandList = (ins addr16:$param);
}

def IndirectLong : AddressingMode {
  let OperandsStr = "[ $param ]";
  let InOperandList = (ins addr8:$param);
}

def IndexedIndirect : AddressingMode {
  let OperandsStr = "( $param , x )";
  let InOperandList = (ins addr8:$param);
}

def IndirectIndexed : AddressingMode {
  let OperandsStr = "( $param ) , y";
  let InOperandList = (ins addr8:$param);
}

def IndirectIndexedLong : AddressingMode {
  let OperandsStr = "[ $param ] , y";
  let InOperandList = (ins addr8:$param);
}

def IndirectIndexedLongZ : AddressingMode {
  let OperandsStr = "[ $param ] , z";
  let InOperandList = (ins addr8:$param);
}

def IndirectZeroPageZ : AddressingMode {
  let OperandsStr = "( $param ) , z";
  let InOperandList = (ins addr8:$param);
}

def StackRelative : AddressingMode {
  let OperandsStr = "$param , s";
  let InOperandList = (ins addr8:$param);
}

def IndirectStackRelativeY : AddressingMode {
  let OperandsStr = "( $param , s ) , y";
  let InOperandList = (ins addr8:$param);
}

def RStackRelative : AddressingMode {
  let OperandsStr = "$param , r";
  let InOperandList = (ins addr8:$param);
}

def IndirectRStackRelativeY : AddressingMode {
  let OperandsStr = "( $param , r ) , y";
  let InOperandList = (ins addr8:$param);
}

def ZeroPageAt2 : AddressingMode {
  let OperandsStr = "$param";
  let InOperandList = (ins addr8at2:$param);
}

def ZeroPageXAt2 : AddressingMode {
  let OperandsStr = "$param , x";
  let InOperandList = (ins addr8at2:$param);
}

def AbsoluteAt2 : AddressingMode {
  let OperandsStr = "$param";
  let InOperandList = (ins addr16at2:$param);
}

def AbsoluteXAt2 : AddressingMode {
  let OperandsStr = "$param , x";
  let InOperandList = (ins addr16at2:$param);
}

/// An unsized MOS instruction, to which an addressing mode may be applied.
class InstAddressMode<string asmstr, Opcode op = DefaultOpcode,
                      AddressingMode mode = Implicit> : Inst<asmstr> {
  Opcode opcode = op;
  let InOperandList = mode.InOperandList;
  string OperandsStr;
  let OperandsStr = mode.OperandsStr;
}

/// An 8 bit real MOS instruction.
class Inst8<string asmstr, Opcode op = DefaultOpcode,
            AddressingMode mode = Implicit> :
    InstAddressMode<asmstr, op, mode> {
  let Size = 1;
  bits<8> Inst;
  let Inst{7-0} = opcode.op;
}

/// A 16 bit real MOS instruction.  Always an 8 bit opcode and an 8 bit
/// operand.
class Inst16<string opcodestr, Opcode op = DefaultOpcode,
             AddressingMode mode = Implicit> :
    InstAddressMode<opcodestr, op, mode> {
  let Size = 2;
  bits<8> param;
  bits<16> Inst;
  let Inst{7-0} = opcode.op;
  let Inst{15-8} = param;
  let AsmString = opcodestr # " " # mode.OperandsStr;
}

/// A 24 bit real MOS instruction. Always an 8 bit opcode and a 16 bit
/// operand.
class Inst24<string opcodestr, Opcode op = DefaultOpcode,
             AddressingMode mode = Implicit> :
    InstAddressMode<opcodestr, op, mode> {
  let Size = 3;
  bits<16> param;
  bits<24> Inst;
  let Inst{7-0} = opcode.op;
  let Inst{23-8} = param;
  let AsmString = opcodestr # " " # mode.OperandsStr;
}

/// A 32 bit real 65816 instruction. Always an 8 bit opcode and a 24 bit
/// operand.
class Inst32<string opcodestr, Opcode op = DefaultOpcode,
             AddressingMode mode = Implicit> :
    InstAddressMode<opcodestr, op, mode> {
  let Size = 4;
  bits<24> param;
  bits<32> Inst;
  let Inst{7-0} = opcode.op;
  let Inst{31-8} = param;
  let AsmString = opcodestr # " " # mode.OperandsStr;
}

/// A 24 bit immediate prefixed HuC6280 instruction.
/// Always an 8 bit opcode, an 8 bit immediate and an 8 bit operand.
class InstImmediate24<string opcodestr, Opcode op = DefaultOpcode,
             AddressingMode mode = Implicit> :
    InstAddressMode<opcodestr, op, mode> {
  let Size = 3;
  bits<8> imm;
  bits<8> param;
  bits<24> Inst;

  Opcode opcode = op;
  let InOperandList = !con((ins imm8:$imm), mode.InOperandList);
  string OperandsStr;
  let OperandsStr = "#$imm , " # mode.OperandsStr;

  let Inst{7-0} = opcode.op;
  let Inst{15-8} = imm;
  let Inst{23-16} = param;
  let AsmString = opcodestr # " " # OperandsStr;
}

/// A 32 bit immediate prefixed HuC6280 instruction.
/// Always an 8 bit opcode, an 8 bit immediate and a 16 bit operand.
class InstImmediate32<string opcodestr, Opcode op = DefaultOpcode,
             AddressingMode mode = Implicit> :
    InstAddressMode<opcodestr, op, mode> {
  let Size = 4;
  bits<8> imm;
  bits<16> param;
  bits<32> Inst;

  Opcode opcode = op;
  let InOperandList = !con((ins imm8:$imm), mode.InOperandList);
  string OperandsStr;
  let OperandsStr = "#$imm , " # mode.OperandsStr;

  let Inst{7-0} = opcode.op;
  let Inst{15-8} = imm;
  let Inst{31-16} = param;
  let AsmString = opcodestr # " " # OperandsStr;
}

/// A R65C02 bit branch instruction.
class InstBitBranch<string opcodestr, Opcode op = DefaultOpcode> :
    Inst<opcodestr>, InstConditionalBranch {
  let Size = 3;
  bits<24> Inst;
  bits<3> tbit;
  bits<8> source;
  bits<8> location;

  Opcode opcode = op;
  let InOperandList = (ins imm3:$tbit, addr8:$source, pcrel8at2:$location);
  string OperandsStr;
  let OperandsStr = "$tbit , $source , $location";

  let Inst{7} = opcode.op{7};
  let Inst{6-4} = tbit;
  let Inst{3-0} = opcode.op{3-0};
  let Inst{15-8} = source;
  let Inst{23-16} = location;
  let AsmString = opcodestr # " " # OperandsStr;
}

/// A R65C02 bit modify instruction.
class InstBitModify<string opcodestr, Opcode op = DefaultOpcode,
                    AddressingMode mode = Implicit> :
    InstAddressMode<opcodestr, op, mode> {
  let Size = 2;
  bits<8> param;
  bits<16> Inst;
  bits<3> tbit;

  Opcode opcode = op;
  let InOperandList = !con((ins imm3:$tbit), mode.InOperandList);
  string OperandsStr;
  let OperandsStr = "$tbit , " # mode.OperandsStr;

  let Inst{7} = opcode.op{7};
  let Inst{6-4} = tbit;
  let Inst{3-0} = opcode.op{3-0};
  let Inst{15-8} = param;
  let AsmString = opcodestr # " " # OperandsStr;
}

/// 45GS02: NOP prefixed opcode with 8-bit address operand (flat memory access)
class Inst4502Nop16<string opcodestr, Opcode op = DefaultOpcode, AddressingMode mode = IndirectIndexedLongZ> :
    Inst<opcodestr> {
  let Size = 3;
  bits<24> Inst;
  bits<8> param;

  Opcode opcode = op;
  let InOperandList = (ins addr8at2:$param);

  let Inst{7-0} = 0xea; // NOP
  let Inst{15-8} = opcode.op;
  let Inst{23-16} = param;
  let AsmString = opcodestr # " " # mode.OperandsStr;
}

/// 45GS02: NEG NEG prefixed opcode with 8-bit address operand
class Inst4502NegNeg8<string opcodestr, Opcode op = DefaultOpcode, AddressingMode mode = ZeroPageX> :
    Inst<opcodestr> {
  let Size = 4;
  bits<32> Inst;
  bits<8> param;

  Opcode opcode = op;
  let InOperandList = (ins addr8at3:$param);
  let Inst{7-0} = 0x42;  // NEG
  let Inst{15-8} = 0x42; // NEG
  let Inst{23-16} = opcode.op;
  let Inst{31-24} = param;
  let AsmString = opcodestr # " " # mode.OperandsStr;
}

/// 45GS02: NEG NEG prefixed opcode; no operand
class Inst4502NegNegImplicit<string opcodestr, Opcode op = DefaultOpcode, AddressingMode mode = Implicit> :
    Inst<opcodestr> {
  let Size = 3;
  bits<24> Inst;
  let InOperandList = mode.InOperandList;

  Opcode opcode = op;
  let Inst{7-0} = 0x42;  // NEG
  let Inst{15-8} = 0x42; // NEG
  let Inst{23-16} = opcode.op;
  let AsmString = opcodestr;
}

/// 45GS02: NEG NEG prefixed opcode with 16-bit address operand
class Inst4502NegNeg16<string opcodestr, Opcode op = DefaultOpcode, AddressingMode mode = AbsoluteX> :
    Inst<opcodestr> {
  let Size = 5;
  bits<40> Inst;
  bits<16> param;

  Opcode opcode = op;
  let InOperandList = (ins addr16at3:$param);

  let Inst{7-0} = 0x42;  // NEG
  let Inst{15-8} = 0x42; // NEG
  let Inst{23-16} = opcode.op;
  let Inst{39-24} = param;
  let AsmString = opcodestr # " " # mode.OperandsStr;
}

/// 45GS02: NEG NEG NOP prefixed opcode with 8-bit address operand
class Inst4502NegNegNop8<string opcodestr, Opcode op = DefaultOpcode, AddressingMode mode = IndirectLong> :
    Inst<opcodestr> {
  let Size = 5;
  bits<40> Inst;
  bits<8> param;

  Opcode opcode = op;
  let InOperandList = (ins addr8at4:$param);

  let Inst{7-0} = 0x42;  // NEG
  let Inst{15-8} = 0x42; // NEG
  let Inst{23-16} = 0xea;// NOP
  let Inst{31-24} = opcode.op;
  let Inst{39-32} = param;
  let AsmString = opcodestr # " " # mode.OperandsStr;
}

/// A HuC6280 block instruction.
class InstHuCBlockMove<string opcodestr, Opcode op = DefaultOpcode> :
    Inst<opcodestr> {
  let Size = 7;
  bits<56> Inst;
  bits<16> bsource;
  bits<16> bdest;
  bits<16> blength;

  Opcode opcode = op;
  let InOperandList = (ins addr16:$bsource, addr16at3:$bdest, imm16at5:$blength);
  string OperandsStr;
  let OperandsStr = "$bsource , $bdest , #$blength";

  let Inst{7-0} = opcode.op;
  let Inst{23-8} = bsource;
  let Inst{39-24} = bdest;
  let Inst{55-40} = blength;
  let AsmString = opcodestr # " " # OperandsStr;
}

/// A 65816 memory move instruction.
class Inst816MemoryMove<string opcodestr, Opcode op = DefaultOpcode> :
    Inst<opcodestr> {
  let Size = 3;
  bits<24> Inst;
  bits<8> sourceBank;
  bits<8> destinationBank;

  Opcode opcode = op;
  let InOperandList = (ins imm8:$sourceBank, imm8at2:$destinationBank);
  string OperandsStr;
  let OperandsStr = "#$sourceBank , #$destinationBank";

  let Inst{7-0} = opcode.op;
  let Inst{15-8} = sourceBank;
  let Inst{23-16} = destinationBank;
  let AsmString = opcodestr # " " # OperandsStr;
}

/// Some one-byte opcodes tend to have fixed lower 4 bits, and vary in the
/// higher four bits.
class InstNybble<string asmstr, bits<4> low = 0, bits<4> high = 0> :
    Inst8<asmstr> {
  let Inst{7-4} = high;
  let Inst{3-0} = low;
}

/// Instructions where the low nybble is 0x0.
class InstLow0<string asmstr, bits<4> high = 0> : InstNybble<asmstr, 0x0, high>;
/// Instructions where the low nybble is 0x2.
class InstLow2<string asmstr, bits<4> high = 0> : InstNybble<asmstr, 0x2, high>;
/// Instructions where the low nybble is 0x3.
class InstLow3<string asmstr, bits<4> high = 0> : InstNybble<asmstr, 0x3, high>;
/// Instructions where the low nybble is 0x4.
class InstLow4<string asmstr, bits<4> high = 0> : InstNybble<asmstr, 0x4, high>;
/// Instructions where the low nybble is 0x8.
class InstLow8<string asmstr, bits<4> high = 0> : InstNybble<asmstr, 0x8, high>;
/// Instructions where the low nybble is 0xa.
class InstLowA<string asmstr, bits<4> high = 0> : InstNybble<asmstr, 0xa, high>;
/// Instructions where the low nybble is 0xb.
class InstLowB<string asmstr, bits<4> high = 0> : InstNybble<asmstr, 0xb, high>;


/// Opcode groups based on cc
/// -------------------------

/// OpcodeC1 class instructions
/// "Because the cc = 01 instructions are relatively regular, we sketch them
/// out first."
multiclass CC1_NoImmediate<bits<3> aaa, string OpcodeStr = "nop", list<Predicate> preds = []> {
/// For cc = 01, then the meaning of bbb is:
/// 000	(zero page,X)
/// 001	zero page
/// 010	#immediate
/// 011	absolute
/// 100	(zero page),Y
/// 101	zero page,X
/// 110	absolute,Y
/// 111	absolute,X
  let Predicates = preds in {
    def _IndexedIndirect :
      Inst16<OpcodeStr, OpcodeC1< aaa, 0b000>, IndexedIndirect>;

    def _ZeroPage :
      Inst16<OpcodeStr, OpcodeC1< aaa, 0b001>, ZeroPage>;

    /// "The only irregularity is the absence of the nonsensical immediate STA
    /// instruction."
    /// skip immediate instruction types here for 0b010

    def _Absolute :
      Inst24<OpcodeStr, OpcodeC1<aaa, 0b011>, Absolute>;

    def _IndirectIndexed :
      Inst16<OpcodeStr, OpcodeC1<aaa, 0b100>, IndirectIndexed>;

    def _ZeroPageX :
      Inst16<OpcodeStr, OpcodeC1<aaa, 0b101>, ZeroPageX>;

    def _AbsoluteY :
      Inst24<OpcodeStr, OpcodeC1<aaa, 0b110>, AbsoluteY>;

    def _AbsoluteX :
      Inst24<OpcodeStr, OpcodeC1<aaa, 0b111>, AbsoluteX>;
  }

/// For cc = 10 on 65C02 and 65816, then the meaning of bbb is:
/// 100	(zero page)
/// For cc = 11 on 65816, then the meaning of bbb is:
/// 000 (stack pointer)
/// 001	[zero page]
/// 011	absolute long
/// 100 (stack pointer),Y
/// 101	[zero page],Y
/// 111	absolute long,X
  let Predicates = preds # [HasW65816Or65EL02] in {
    def _StackRelative :
      Inst16<OpcodeStr, OpcodeC3<aaa, 0b000>, StackRelative>;

    def _IndirectStackRelativeY :
      Inst16<OpcodeStr, OpcodeC3<aaa, 0b100>, IndirectStackRelativeY>;
  }

  let Predicates = preds # [Has65C02] in {
    def _Indirect :
      Inst16<OpcodeStr, OpcodeC2<aaa, 0b100>, Indirect>;
  }
  
  let Predicates = preds # [HasW65816] in {
    let DecoderNamespace = "w65816" in {
      def _IndirectLong :
        Inst16<OpcodeStr, OpcodeC3<aaa, 0b001>, IndirectLong>;

      def _AbsoluteLong :
        Inst32<OpcodeStr, OpcodeC3<aaa, 0b011>, AbsoluteLong>;

      def _IndirectIndexedLong :
        Inst16<OpcodeStr, OpcodeC3<aaa, 0b101>, IndirectIndexedLong>;

      def _AbsoluteXLong :
        Inst32<OpcodeStr, OpcodeC3<aaa, 0b111>, AbsoluteXLong>;
    }
  }

  let Predicates = preds # [Has65EL02] in {
    let DecoderNamespace = "65el02" in {
      def _RStackRelative :
        Inst16<OpcodeStr, OpcodeC3<aaa, 0b001>, RStackRelative>;

      def _RIndirectStackRelativeY :
        Inst16<OpcodeStr, OpcodeC3<aaa, 0b101>, IndirectRStackRelativeY>;
    }
  }
}

/// All cc=01 instruction types, for the majority of opcodes where cc == 01.
multiclass CC1_All<bits<3> aaa, string OpcodeStr = "nop", list<Predicate> preds = []> :
    CC1_NoImmediate<aaa, OpcodeStr, preds> {
  // handle immediate CC1 instruction types here for opcodes that fit the
  // CC1 pattern
  def _Immediate : Inst16<OpcodeStr, OpcodeC1< aaa, 0b010 >, Immediate> {
    let Predicates = preds;
    let MHigh = 1;
  }

  // 16-bit immediate operand on 65816 when M flag is low.
  def _Immediate16 : Inst24<OpcodeStr, OpcodeC1< aaa, 0b010 >, Immediate16> {
    let Predicates = preds # [HasW65816Or65EL02];
    let DecoderNamespace = "MOSMLow";
    let MLow = 1;
  }
}

/// "Next we consider the cc = 10 instructions. These have a completely
/// different set of opcodes:
///
/// aaa opcode
/// 000 ASL
/// 001 ROL
/// 010 LSR
/// 011 ROR
/// 100 STX
/// 101 LDX
/// 110 DEC
/// 111 INC
///
/// The addressing modes are similar to the 01 case, but not quite the same:
/// bbb	addressing mode
/// 000 #immediate
/// 001 zero page
/// 010 accumulator
/// 011 absolute
/// 101 zero page,X
/// 111 absolute,X
/// Note that bbb = 100 and 110 are missing. Also, with STX and LDX,
/// "zero page,X" addressing becomes "zero page,Y", and with LDX, "absolute,X"
/// becomes "absolute,Y".
///
/// These fit together like this:"
///
///             ASL ROL LSR ROR STX LDX DEC INC
/// #                               A2
/// zp          06  26  46  66  86  A6  C6  E6
/// A           0A  2A  4A  6A
/// abs         0E  2E  4E  6E  8E  AE  CE  EE
/// zp,X/zp,    16  36  56  76  96  B6  D6  F6
/// abs,X/abs,Y	1E	3E	5E	7E      BE  DE  FE
///
/// "

/// This multiclass covers ASL, ROL, LSR and ROR instructions in the
/// chart above.
multiclass CC2_Shift<bits<3> aaa, string OpcodeStr = "nop"> {
  def _ZeroPage : Inst16<OpcodeStr, OpcodeC2<aaa, 0b001>, ZeroPage>;

  def _Accumulator : Inst8<OpcodeStr, OpcodeC2<aaa, 0b010>, Accumulator>;

  def _Absolute : Inst24<OpcodeStr, OpcodeC2<aaa, 0b011>, Absolute>;

  def _ZeroPageX : Inst16<OpcodeStr, OpcodeC2<aaa, 0b101>, ZeroPageX>;

  def _AbsoluteX : Inst24<OpcodeStr, OpcodeC2<aaa, 0b111>, AbsoluteX>;
}

/// This multiclass covers regular instructions for STX, LDX, INC and DEC
/// in the chart above.
multiclass CC2_NonShift<bits<3> aaa, string OpcodeStr = "nop"> {
  def _ZeroPage : Inst16<OpcodeStr, OpcodeC2<aaa, 0b001>, ZeroPage>;

  def _Absolute : Inst24<OpcodeStr, OpcodeC2<aaa, 0b011>, Absolute>;

  def _ZeroPageX : Inst16<OpcodeStr, OpcodeC2<aaa, 0b101>, ZeroPageX>;
}

/// "Next, the cc = 00 instructions. Again, the opcodes are different:
/// aaa opcode
/// 001 BIT
/// 010 JMP
/// 011 JMP (abs)
/// 100 STY
/// 101 LDY
/// 110 CPY
/// 111 CPX
/// The addressing modes are the same as the 10 case, except that accumulator
/// mode is missing.
/// bbb addressing mode
/// 000 #immediate
/// 001 zero page
/// 011 absolute
/// 101 zero page,X
/// 111 absolute,X
/// And here's how they fit together:
///
///       BIT JMP JMP() STY LDY CPY CPX
/// #                       A0  C0  E0
/// zp    24            84  A4  C4  E4
/// abs   2C  4C  6C    8C  AC  CC  EC
/// zp,X                94  B4
/// abs,X                   BC

/// The 9 instructions in the top right corner
multiclass CC0_Regular< bits<3> aaa, string OpcodeStr = "nop"> {
  def _Immediate : Inst16<OpcodeStr, OpcodeC0<aaa, 0b000>, Immediate> {
    let XHigh = 1;
  }

  // 16-bit immediate operand on 65816 when X flag is low.
  def _Immediate16 : Inst24<OpcodeStr, OpcodeC0<aaa, 0b000>, Immediate16> {
    let Predicates = [HasW65816Or65EL02];
    let DecoderNamespace = "MOSXLow";
    let XLow = 1;
  }

  def _ZeroPage : Inst16<OpcodeStr, OpcodeC0<aaa, 0b001>, ZeroPage>;

  def _Absolute : Inst24<OpcodeStr, OpcodeC0<aaa, 0b011>, Absolute>;
}

/// Matching CC1_NoImmediate, but with cc = 11. Common match for 6502X.
multiclass CC3_NoImmediate<bits<3> aaa, string OpcodeStr = "nop"> {
  def _IndexedIndirect :
    Inst16<OpcodeStr, OpcodeC3< aaa, 0b000>, IndexedIndirect>;

  def _ZeroPage :
    Inst16<OpcodeStr, OpcodeC3< aaa, 0b001>, ZeroPage>;

  /// "The only irregularity is the absence of the nonsensical immediate STA
  /// instruction."
  /// skip immediate instruction types here for 0b010

  def _Absolute :
    Inst24<OpcodeStr, OpcodeC3<aaa, 0b011>, Absolute>;

  def _IndirectIndexed :
    Inst16<OpcodeStr, OpcodeC3<aaa, 0b100>, IndirectIndexed>;

  def _ZeroPageX :
    Inst16<OpcodeStr, OpcodeC3<aaa, 0b101>, ZeroPageX>;

  def _AbsoluteY :
    Inst24<OpcodeStr, OpcodeC3<aaa, 0b110>, AbsoluteY>;

  def _AbsoluteX :
    Inst24<OpcodeStr, OpcodeC3<aaa, 0b111>, AbsoluteX>;
}

/// "The conditional branch instructions all have the form xxy10000. The flag
/// indicated by xx is compared with y, and the branch is taken if they are
/// equal.
/// xx flag
/// 00 negative
/// 01 overflow
/// 10 carry
/// 11 zero
/// This gives the following branches:
/// BPL BMI BVC BVS BCC BCS BNE BEQ
/// 10  30  50  70  90  B0  D0  F0

class ConditionalBranch<string opcodestr, bits<2> flagType, bits<1> value> :
    Inst16<opcodestr, DefaultOpcode, Relative>, InstConditionalBranch {
   bits<3> a;
   let a{2-1} = flagType;
   let a{0} = value;
   let opcode = OpcodeABC<a, 0b100, 0b00>;
}
