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
  // Side-effects of the MC instructiosn
  let OutOperandList = (outs);

  let AsmString = asmstr;

  // Note: These properties are not tracked for the real instruction set, which
  // is used only in the MC layer. See the logical instruction set for the
  // definitions of these properties.
  let mayLoad = true;
  let mayStore = true;
  let hasSideEffects = true;

  field bits<32> SoftFail = 0;
  let DecoderNamespace = "MOS";
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

/// This operand will only match a value from 0 to 255 inclusive.
def imm8 : Operand<i32> {
  let ParserMatchClass = ImmediateAsmOperand<"Imm8">;
  let EncoderMethod = "encodeImm<MOS::Imm8, 1>";
}

/// This operand will only match a value from 0 to 65535 inclusive.
/// Only 16-bit variants and virtual instructions should need this.
def imm16 : Operand<i32> {
  let ParserMatchClass = ImmediateAsmOperand<"Imm16">;
  let EncoderMethod = "encodeImm<MOS::Imm16, 1>";
}

/// An 8-bit pc-relative reference, for branch instructions.
def pcrel8 : Operand<i32> {
  let ParserMatchClass = PCRelativeOperand<"PCRel8">;
  let EncoderMethod = "encodeImm<MOS::PCRel8, 1>";
}

def addr8 : Operand<i32> {
  let ParserMatchClass = Addr8Operand<"Addr8">;
  let EncoderMethod = "encodeImm<MOS::Addr8, 1>";
}

def addr16 : Operand<i32> {
  let ParserMatchClass = Addr16Operand<"Addr16">;
  let EncoderMethod = "encodeImm<MOS::Addr16, 1>";
}

/// This operand will only match a value from 256 to 65536 inclusive.
def imm8to16 : Operand<i32> {
  let ParserMatchClass = ImmediateAsmOperand<"Imm8To16">;
  let EncoderMethod = "encodeImm<MOS::Addr16, 1>";
}

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

def Absolute : AddressingMode {
  let OperandsStr = "$param";
  let InOperandList = (ins addr16:$param);
}

def AbsoluteX : AddressingMode {
  let OperandsStr = "$param , x";
  let InOperandList = (ins addr16:$param);
}

def AbsoluteY : AddressingMode {
  let OperandsStr = "$param , y";
  let InOperandList = (ins addr16:$param);
}

def Indirect : AddressingMode {
  let OperandsStr = "( $param )";
  let InOperandList = (ins addr16:$param);
}

def IndexedIndirect : AddressingMode {
  let OperandsStr = "( $param , x )";
  let InOperandList = (ins addr8:$param);
}

def IndirectIndexed : AddressingMode {
  let OperandsStr = "( $param ) , y";
  let InOperandList = (ins addr8:$param);
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

/// Some one-byte opcodes tend to have fixed lower 4 bits, and vary in the
/// higher four bits.
class InstNybble<string asmstr, bits<4> low = 0, bits<4> high = 0> :
    Inst8<asmstr> {
  let Inst{7-4} = high;
  let Inst{3-0} = low;
}

/// Instructions where the low nybble is 0x0.
class InstLow0<string asmstr, bits<4> high = 0> : InstNybble<asmstr, 0x0, high>;
/// Instructions where the low nybble is 0x8.
class InstLow8<string asmstr, bits<4> high = 0> : InstNybble<asmstr, 0x8, high>;
/// Instructions where the low nybble is 0xa.
class InstLowA<string asmstr, bits<4> high = 0> : InstNybble<asmstr, 0xa, high>;

/// Opcode groups based on cc
/// -------------------------

/// OpcodeC1 class instructions
/// "Because the cc = 01 instructions are relatively regular, we sketch them
/// out first."
multiclass CC1_NoImmediate<bits<3> aaa, string OpcodeStr = "nop">
{
/// For cc = 01, then the meaning of bbb is:
/// 000	(zero page,X)
/// 001	zero page
/// 010	#immediate
/// 011	absolute
/// 100	(zero page),Y
/// 101	zero page,X
/// 110	absolute,Y
/// 111	absolute,X
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

/// All cc=01 instruction types, for the majority of opcodes where cc == 01.
multiclass CC1_All<bits<3> aaa, string OpcodeStr = "nop"> :
    CC1_NoImmediate<aaa, OpcodeStr> {
  // handle immediate CC1 instruction types here for opcodes that fit the
  // CC1 pattern
  def _Immediate : Inst16<OpcodeStr, OpcodeC1< aaa, 0b010 >, Immediate>;
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
///                                 A2
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
  def _Immediate : Inst16<OpcodeStr, OpcodeC0<aaa, 0b000>, Immediate>;

  def _ZeroPage : Inst16<OpcodeStr, OpcodeC0<aaa, 0b001>, ZeroPage>;

  def _Absolute : Inst24<OpcodeStr, OpcodeC0<aaa, 0b011>, Absolute>;
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
    Inst16<opcodestr, DefaultOpcode, Relative> {
   bits<3> a;
   let a{2-1} = flagType;
   let a{0} = value;
   let opcode = OpcodeABC<a, 0b100, 0b00>;
}

/// Predicates. Useful for limiting instructions to particular hardware modes
/// or particular hardware implementations.
def Has6502 : Predicate<"Subtarget->has6502()">,
              AssemblerPredicate<(all_of Feature6502), "Feature6502"> {
  let PredicateName = "Feature6502";
}
