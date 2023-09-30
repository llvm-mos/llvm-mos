//===-- MOSInstrInfoSPC700.td - SPC700 Instruction defs ----*- tablegen -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes the SPC700 instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

/// Sony developed a 6502-like CPU used, among other products, in the SPC700.
/// For unknown reasons, while many of its opcodes map closely to those of
/// the 6502, their encoding is completely different.

class MOSSPC700<Instruction from, Instruction to> {
  Instruction From = from;
  Instruction To = to;
}

class MOSSPC700Mode<string MosInstPrefix, string InstPrefix, string Suffix>
  : MOSSPC700<!cast<Instruction>(MosInstPrefix # Suffix),
              !cast<Instruction>(InstPrefix # Suffix)>;

/// == SPC700 addressing mode syntax ==

def SPC700ZeroPageZeroPage : AddressingMode {
  let OperandsStr = "$dest , $src";
  let InOperandList = (ins addr8:$dest, addr8at2:$src);
}

def SPC700ZeroPageImmediate : AddressingMode {
  let OperandsStr = "$dest , # $src";
  let InOperandList = (ins addr8:$dest, imm8at2:$src);
}

def SPC700ZeroPageX : AddressingMode {
  let OperandsStr = "$param + x";
  let InOperandList = (ins addr8:$param);
}

def SPC700ZeroPageY : AddressingMode {
  let OperandsStr = "$param + y";
  let InOperandList = (ins addr8:$param);
}

def SPC700ZeroPageRelative : AddressingMode {
  let OperandsStr = "$dest , $src";
  let InOperandList = (ins addr8:$dest, pcrel8at2:$src);
}

def SPC700ZeroPageXRelative : AddressingMode {
  let OperandsStr = "$dest + x , $src";
  let InOperandList = (ins addr8:$dest, pcrel8at2:$src);
}

def SPC700AbsoluteX : AddressingMode {
  let OperandsStr = "$param + x";
  let InOperandList = (ins addr16:$param);
}

def SPC700AbsoluteY : AddressingMode {
  let OperandsStr = "$param + y";
  let InOperandList = (ins addr16:$param);
}

def SPC700IndirectXIndirectY : AddressingMode {
  let OperandsStr = "( x ) , ( y )";
  let InOperandList = (ins);
}

def SPC700IndirectX : AddressingMode {
  let OperandsStr = "( x )";
  let InOperandList = (ins);
}

def SPC700IndirectXAutoIncrement : AddressingMode {
  let OperandsStr = "( x ) +";
  let InOperandList = (ins);
}

def SPC700IndexedIndirect : AddressingMode {
  let OperandsStr = "[ $param + x ]";
  let InOperandList = (ins addr8:$param);
}

def SPC700IndexedIndirect16 : AddressingMode {
  let OperandsStr = "[ $param + x ]";
  let InOperandList = (ins addr16:$param);
}

def SPC700IndirectIndexed : AddressingMode {
  let OperandsStr = "[ $param ] + y";
  let InOperandList = (ins addr8:$param);
}

/// == SPC700 operand types ==

/// A 13-bit address.
class Addr13Operand<string name> : MOSAsmOperand<name>;

class addr13at<int offset> : Operand<i32> {
  let ParserMatchClass = Addr13Operand<"Addr13">;
  let EncoderMethod = "encodeImm<MOS::Addr13, " # offset # ">";
  let DecoderMethod = "decodeUImmOperand<13>";
  let OperandType = "OPERAND_ADDR13";
  let OperandNamespace = "MOSOp";
  let Type = i16;
}
def addr13 : addr13at<1>;

/// == SPC700 instruction formats ==

class SPC700Inst8<string prefix, string suffix, Opcode op = DefaultOpcode,
                  AddressingMode mode = Implicit> :
    InstAddressMode<prefix, op, mode> {
  let Size = 1;
  bits<8> Inst;
  let Inst{7-0} = opcode.op;
  let AsmString = prefix # " " # mode.OperandsStr # " " # suffix;
}

class SPC700Inst16<string prefix, string suffix, Opcode op = DefaultOpcode,
                   AddressingMode mode = Implicit> :
    InstAddressMode<prefix, op, mode> {
  let Size = 2;
  bits<8> param;
  bits<16> Inst;
  let Inst{7-0} = opcode.op;
  let Inst{15-8} = param;
  let AsmString = prefix # " " # mode.OperandsStr # " " # suffix;
}

class SPC700Inst24<string prefix, string suffix, Opcode op = DefaultOpcode,
                   AddressingMode mode = Implicit> :
    InstAddressMode<prefix, op, mode> {
  let Size = 3;
  bits<16> param;
  bits<24> Inst;
  let Inst{7-0} = opcode.op;
  let Inst{23-8} = param;
  let AsmString = prefix # " " # mode.OperandsStr # " " # suffix;
}

class SPC700Inst24TwoOp<string prefix, string suffix, Opcode op = DefaultOpcode,
                        AddressingMode mode = Implicit> :
    InstAddressMode<prefix, op, mode> {
  let Size = 3;
  bits<8> dest;
  bits<8> src;
  bits<24> Inst;
  let Inst{7-0} = opcode.op;
  let Inst{15-8} = dest;
  let Inst{23-16} = src;
  let AsmString = prefix # " " # mode.OperandsStr # " " # suffix;
}

multiclass SPC700LoadStoreA<string prefix, string suffix, bits<3> aaa> {
  def _ZeroPage :
    SPC700Inst16<prefix, suffix, OpcodeABC<aaa, 0b001, 0b00>, ZeroPage>;

  def _Absolute :
    SPC700Inst24<prefix, suffix, OpcodeABC<aaa, 0b001, 0b01>, Absolute>;

  def _IndirectX :
    SPC700Inst8<prefix, suffix, OpcodeABC<aaa, 0b001, 0b10>, SPC700IndirectX>;

  def _IndexedIndirect :
    SPC700Inst16<prefix, suffix, OpcodeABC<aaa, 0b001, 0b11>, SPC700IndexedIndirect>;
      
  def _ZeroPageX :
    SPC700Inst16<prefix, suffix, OpcodeABC<aaa, 0b101, 0b00>, SPC700ZeroPageX>;

  def _AbsoluteX :
    SPC700Inst24<prefix, suffix, OpcodeABC<aaa, 0b101, 0b01>, SPC700AbsoluteX>;

  def _AbsoluteY :
    SPC700Inst24<prefix, suffix, OpcodeABC<aaa, 0b101, 0b10>, SPC700AbsoluteY>;

  def _IndirectIndexed :
    SPC700Inst16<prefix, suffix, OpcodeABC<aaa, 0b101, 0b11>, SPC700IndirectIndexed>;
}

multiclass SPC700MosAliasLoadStoreA<string MosOpcodeStr, string InstPrefix> {
  def : InstAlias<MosOpcodeStr # " $param", (!cast<Instruction>(InstPrefix # "_ZeroPage") addr8:$param)>;
  def : InstAlias<MosOpcodeStr # " $param", (!cast<Instruction>(InstPrefix # "_Absolute") addr16:$param)>;
  def : InstAlias<MosOpcodeStr # " ( x )", (!cast<Instruction>(InstPrefix # "_IndirectX"))>;
  def : InstAlias<MosOpcodeStr # " ( $param , x )", (!cast<Instruction>(InstPrefix # "_IndexedIndirect") addr8:$param)>;
  def : InstAlias<MosOpcodeStr # " $param , x", (!cast<Instruction>(InstPrefix # "_ZeroPageX") addr8:$param)>;
  def : InstAlias<MosOpcodeStr # " $param , x", (!cast<Instruction>(InstPrefix # "_AbsoluteX") addr16:$param)>;
  def : InstAlias<MosOpcodeStr # " $param , y", (!cast<Instruction>(InstPrefix # "_AbsoluteY") addr16:$param)>;
  def : InstAlias<MosOpcodeStr # " ( $param ) , y", (!cast<Instruction>(InstPrefix # "_IndirectIndexed") addr8:$param)>;
}

multiclass SPC700MosSpcLoadStoreA<string MosInstPrefix, string InstPrefix> {
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_ZeroPage">;
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_Absolute">;
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_IndexedIndirect">;
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_ZeroPageX">;
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_AbsoluteX">;
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_AbsoluteY">;
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_IndirectIndexed">;
}

multiclass SPC700LoadStoreX<string prefix, string suffix, bits<3> aaa> {
  def _Absolute :
    SPC700Inst24<prefix, suffix, OpcodeABC<aaa, 0b010, 0b01>, Absolute>;

  def _ZeroPage :
    SPC700Inst16<prefix, suffix, OpcodeABC<aaa, 0b110, 0b00>, ZeroPage>;

  def _ZeroPageY :
    SPC700Inst16<prefix, suffix, OpcodeABC<aaa, 0b110, 0b01>, SPC700ZeroPageY>;
}

multiclass SPC700MosAliasLoadStoreX<string MosOpcodeStr, string InstPrefix> {
  def : InstAlias<MosOpcodeStr # " $param", (!cast<Instruction>(InstPrefix # "_Absolute") addr16:$param)>;
  def : InstAlias<MosOpcodeStr # " $param", (!cast<Instruction>(InstPrefix # "_ZeroPage") addr8:$param)>;
  def : InstAlias<MosOpcodeStr # " $param , y", (!cast<Instruction>(InstPrefix # "_ZeroPageY") addr8:$param)>;
}

multiclass SPC700MosSpcLoadStoreX<string MosInstPrefix, string InstPrefix> {
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_Absolute">;
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_ZeroPage">;
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_ZeroPageY">;
}

multiclass SPC700LoadStoreY<string prefix, string suffix, bits<3> aaa> {
  def _ZeroPage :
    SPC700Inst16<prefix, suffix, OpcodeABC<aaa, 0b010, 0b11>, ZeroPage>;

  def _Absolute :
    SPC700Inst24<prefix, suffix, OpcodeABC<aaa, 0b011, 0b00>, Absolute>;

  def _ZeroPageX :
    SPC700Inst16<prefix, suffix, OpcodeABC<aaa, 0b110, 0b11>, SPC700ZeroPageX>;
}

multiclass SPC700MosAliasLoadStoreY<string MosOpcodeStr, string InstPrefix> {
  def : InstAlias<MosOpcodeStr # " $param", (!cast<Instruction>(InstPrefix # "_ZeroPage") addr8:$param)>;
  def : InstAlias<MosOpcodeStr # " $param", (!cast<Instruction>(InstPrefix # "_Absolute") addr16:$param)>;
  def : InstAlias<MosOpcodeStr # " $param , x", (!cast<Instruction>(InstPrefix # "_ZeroPageX") addr8:$param)>;
}

multiclass SPC700MosSpcLoadStoreY<string MosInstPrefix, string InstPrefix> {
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_ZeroPage">;
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_Absolute">;
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_ZeroPageX">;
}

multiclass SPC700Arithmetic<string OpcodeStr, bits<3> aaa>
    : SPC700LoadStoreA<OpcodeStr # " a ,", "", aaa> {
  def _Immediate :
    SPC700Inst16<OpcodeStr # " a ,", "", OpcodeABC<aaa, 0b010, 0b00>, Immediate>;

  def _ZeroPageZeroPage :
    SPC700Inst24TwoOp<OpcodeStr, "", OpcodeABC<aaa, 0b010, 0b01>, SPC700ZeroPageZeroPage>;

  def _ZeroPageImmediate :
    SPC700Inst24TwoOp<OpcodeStr, "", OpcodeABC<aaa, 0b110, 0b00>, SPC700ZeroPageImmediate>;

  def _IndirectXIndirectY :
    SPC700Inst8<OpcodeStr, "", OpcodeABC<aaa, 0b110, 0b01>, SPC700IndirectXIndirectY>;
}

multiclass SPC700MosAliasArithmetic<string MosOpcodeStr, string InstPrefix>
    : SPC700MosAliasLoadStoreA<MosOpcodeStr, InstPrefix> {
  def : InstAlias<MosOpcodeStr # " #$param", (!cast<Instruction>(InstPrefix # "_Immediate") imm8:$param)>;
}

multiclass SPC700MosSpcArithmetic<string MosInstPrefix, string InstPrefix>
    : SPC700MosSpcLoadStoreA<MosInstPrefix, InstPrefix> {
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_Immediate">;
}

multiclass SPC700ShiftInc<string OpcodeStr, bits<3> aaa> {
  def _ZeroPage :
    SPC700Inst16<OpcodeStr, "", OpcodeABC<aaa, 0b010, 0b11>, ZeroPage>;

  def _ZeroPageX :
    SPC700Inst16<OpcodeStr, "", OpcodeABC<aaa, 0b110, 0b11>, SPC700ZeroPageX>;

  def _Absolute :
    SPC700Inst24<OpcodeStr, "", OpcodeABC<aaa, 0b011, 0b00>, Absolute>;

  def _Accumulator :
    SPC700Inst8<OpcodeStr, "a", OpcodeABC<aaa, 0b111, 0b00>, Accumulator>;
}

multiclass SPC700MosAliasShiftInc<string MosOpcodeStr, string InstPrefix> {
  def : InstAlias<MosOpcodeStr, (!cast<Instruction>(InstPrefix # "_Accumulator"))>;
  def : InstAlias<MosOpcodeStr # " $param , x", (!cast<Instruction>(InstPrefix # "_ZeroPageX") addr8:$param)>;
}

multiclass SPC700MosSpcShiftInc<string MosInstPrefix, string InstPrefix> {
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_ZeroPage">;
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_ZeroPageX">;
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_Absolute">;
  def : MOSSPC700Mode<MosInstPrefix, InstPrefix, "_Accumulator">;
}

class OpcodeSPC700Cmp<bits<1> a = 0, bits<1> b = 0>
    : Opcode<0> {
  let op{7-7} = 0;
  let op{6-6} = a;
  let op{5-5} = b;
  let op{4-0} = 0x1E;
}

multiclass SPC700CmpE<string OpcodeStr, bits<1> a = 0> {
  def _Absolute :
    SPC700Inst24<OpcodeStr, "", OpcodeSPC700Cmp<a, 0>, Absolute>;

  def _ZeroPage :
    SPC700Inst16<OpcodeStr, "", OpcodeSPC700Cmp<a, 1>, ZeroPage>;
}

class OpcodeSPC700PushPop<bits<1> a = 0, bits<2> bb = 0, bits<5> ccccc = 0>
    : Opcode<0> {
  let op{7-7} = a;
  let op{6-5} = bb;
  let op{4-0} = ccccc;
}

multiclass SPC700PushPop<string OpcodeStr, bits<1> a, bits<5> ccccc> {
  def _PSW :
    SPC700Inst8<OpcodeStr, "psw", OpcodeSPC700PushPop<a, 0b00, ccccc>, Implicit>;
  def _A :
    SPC700Inst8<OpcodeStr, "a", OpcodeSPC700PushPop<a, 0b01, ccccc>, Implicit>;
  def _X :
    SPC700Inst8<OpcodeStr, "x", OpcodeSPC700PushPop<a, 0b10, ccccc>, Implicit>;
  def _Y :
    SPC700Inst8<OpcodeStr, "y", OpcodeSPC700PushPop<a, 0b11, ccccc>, Implicit>;
}

multiclass SPC700MosAliasPushPop<string MosOpcodeStr, string InstPrefix> {
  def : InstAlias<MosOpcodeStr # "p", (!cast<Instruction>(InstPrefix # "_PSW"))>;
  def : InstAlias<MosOpcodeStr # "a", (!cast<Instruction>(InstPrefix # "_A"))>;
  def : InstAlias<MosOpcodeStr # "x", (!cast<Instruction>(InstPrefix # "_X"))>;
  def : InstAlias<MosOpcodeStr # "y", (!cast<Instruction>(InstPrefix # "_Y"))>;
}

class SPC700InstBit13<string prefix, string suffix, Opcode op = DefaultOpcode> :
    Inst<prefix> {
  let Size = 3;
  bits<24> Inst;
  bits<3> tbit;
  bits<13> source;

  Opcode opcode = op;
  let InOperandList = (ins addr13:$source, imm3:$tbit);
  string OperandsStr;
  let OperandsStr = "$source . $tbit";

  let Inst{7-0} = opcode.op;
  let Inst{20-8} = source;
  let Inst{23-21} = tbit;
  let AsmString = prefix # " " # OperandsStr # " " # suffix;
}

class SPC700InstBitBranch<string opcodestr, Opcode op = DefaultOpcode> :
    Inst<opcodestr>, InstConditionalBranch {
  let Size = 3;
  bits<24> Inst;
  bits<3> tbit;
  bits<8> source;
  bits<8> location;

  Opcode opcode = op;
  let InOperandList = (ins imm3:$tbit, addr8:$source, pcrel8at2:$location);
  string OperandsStr;
  let OperandsStr = "$source . $tbit , $location";

  let Inst{7-5} = tbit;
  let Inst{4-0} = opcode.op{4-0};
  let Inst{15-8} = source;
  let Inst{23-16} = location;
  let AsmString = opcodestr # " " # OperandsStr;
}

class SPC700InstBitModify<string opcodestr, Opcode op = DefaultOpcode,
                    AddressingMode mode = Implicit> :
    InstAddressMode<opcodestr, op, mode> {
  let Size = 2;
  bits<8> param;
  bits<16> Inst;
  bits<3> tbit;

  Opcode opcode = op;
  let InOperandList = !con((ins imm3:$tbit), mode.InOperandList);
  string OperandsStr;
  let OperandsStr = mode.OperandsStr # " . $tbit";

  let Inst{7-5} = tbit;
  let Inst{4-0} = opcode.op{4-0};
  let Inst{15-8} = param;
  let AsmString = opcodestr # " " # OperandsStr;
}

/// == SPC700 opcode definitions ==

let Predicates = [HasSPC700] in {
let DecoderNamespace = "spc700" in {

// x0 opcodes
def SPC700_NOP_Implied : InstLow0<"nop", 0x0>;
def SPC700_CLRP_Implied : InstLow0<"clrp", 0x2>;
def SPC700_SETP_Implied : InstLow0<"setp", 0x4>;
def SPC700_CLRC_Implied : InstLow0<"clrc", 0x6>;
def SPC700_SETC_Implied : InstLow0<"setc", 0x8>;
def SPC700_EI_Implied : InstLow0<"ei", 0xA>;
def SPC700_DI_Implied : InstLow0<"di", 0xC>;
def SPC700_CLRV_Implied : InstLow0<"clrv", 0xE>;

def SPC700_BPL_Relative : ConditionalBranch<"bpl", 0b00, 0b0>;
def SPC700_BMI_Relative : ConditionalBranch<"bmi", 0b00, 0b1>;
def SPC700_BVC_Relative : ConditionalBranch<"bvc", 0b01, 0b0>;
def SPC700_BVS_Relative : ConditionalBranch<"bvs", 0b01, 0b1>;
def SPC700_BCC_Relative : ConditionalBranch<"bcc", 0b10, 0b0>;
def SPC700_BCS_Relative : ConditionalBranch<"bcs", 0b10, 0b1>;
def SPC700_BNE_Relative : ConditionalBranch<"bne", 0b11, 0b0>;
def SPC700_BEQ_Relative : ConditionalBranch<"beq", 0b11, 0b1>;
 
// x1 opcodes
def SPC700_TCALL_Implied : Inst<"tcall $idx"> {
  let Size = 1;
  bits<8> Inst;
  bits<4> idx;

  let InOperandList = (ins imm4:$idx);
  let Inst{7-4} = idx;
  let Inst{3-0} = 1;
}

// x2, x3 opcodes
def SPC700_SET1_ZeroPage : SPC700InstBitModify<"set1", Opcode<0x02>, ZeroPage>;
def SPC700_CLR1_ZeroPage : SPC700InstBitModify<"clr1", Opcode<0x12>, ZeroPage>;
def SPC700_BBS_ZeroPage : SPC700InstBitBranch<"bbs", Opcode<0x03>>;
def SPC700_BBC_ZeroPage : SPC700InstBitBranch<"bbc", Opcode<0x13>>;

// x4, x5, x6, x7, x8, x9 opcodes
defm SPC700_OR : SPC700Arithmetic<"or", 0b000>;
defm SPC700_AND : SPC700Arithmetic<"and", 0b001>;
defm SPC700_EOR : SPC700Arithmetic<"eor", 0b010>;
defm SPC700_CMP : SPC700Arithmetic<"cmp", 0b011>;
defm SPC700_ADC : SPC700Arithmetic<"adc", 0b100>;
defm SPC700_SBC : SPC700Arithmetic<"sbc", 0b101>;
defm SPC700_STORE_A : SPC700LoadStoreA<"mov", " , a", 0b110>;
defm SPC700_LOAD_A : SPC700LoadStoreA<"mov a, ", "", 0b111>;
defm SPC700_STORE_X : SPC700LoadStoreX<"mov", " , x", 0b110>;
defm SPC700_LOAD_X : SPC700LoadStoreX<"mov x, ", "", 0b111>;
def SPC700_CMP_X_Immediate :
  SPC700Inst16<"cmp x ,", "", OpcodeABC<0b110, 0b010, 0b00>, Immediate>;
def SPC700_LOAD_A_Immediate :
  SPC700Inst16<"mov a ,", "", OpcodeABC<0b111, 0b010, 0b00>, Immediate>;

// xA
def SPC700_OR1_C_Absolute : SPC700InstBit13<"or1 c ,", "", Opcode<0x0A>>;
def SPC700_NOR1_C_Absolute : SPC700InstBit13<"or1 c , /", "", Opcode<0x2A>>;
def SPC700_AND1_C_Absolute : SPC700InstBit13<"and1 c ,", "", Opcode<0x4A>>;
def SPC700_NAND1_C_Absolute : SPC700InstBit13<"and1 c , /", "", Opcode<0x6A>>;
def SPC700_EOR1_C_Absolute : SPC700InstBit13<"eor1 c ,", "", Opcode<0x8A>>;
def SPC700_LOAD1_C_Absolute : SPC700InstBit13<"mov1 c ,", "", Opcode<0xAA>>;
def SPC700_STORE1_C_Absolute : SPC700InstBit13<"mov1", ", c", Opcode<0xCA>>;
def SPC700_NOT1_Absolute : SPC700InstBit13<"not1", "", Opcode<0xEA>>;
def SPC700_DECW_ZeroPage : SPC700Inst16<"decw", "", Opcode<0x1A>, ZeroPage>;
def SPC700_INCW_ZeroPage : SPC700Inst16<"incw", "", Opcode<0x3A>, ZeroPage>;
def SPC700_CMPW_YA_ZeroPage
    : SPC700Inst16<"cmpw ya ,", "", Opcode<0x5A>, ZeroPage>;
def SPC700_ADDW_YA_ZeroPage
    : SPC700Inst16<"addw ya ,", "", Opcode<0x7A>, ZeroPage>;
def SPC700_SUBW_YA_ZeroPage
    : SPC700Inst16<"subw ya ,", "", Opcode<0x9A>, ZeroPage>;
def SPC700_LOAD_YA_ZeroPage
    : SPC700Inst16<"movw ya ,", "", Opcode<0xBA>, ZeroPage>;
def SPC700_STORE_YA_ZeroPage
    : SPC700Inst16<"movw", ", ya", Opcode<0xDA>, ZeroPage>;
def SPC700_MOV_ZeroPageZeroPage :
  SPC700Inst24TwoOp<"mov", "", Opcode<0xFA>, SPC700ZeroPageZeroPage>;

// xB, xC opcodes
defm SPC700_ASL : SPC700ShiftInc<"asl", 0b000>;
defm SPC700_ROL : SPC700ShiftInc<"rol", 0b001>;
defm SPC700_LSR : SPC700ShiftInc<"lsr", 0b010>;
defm SPC700_ROR : SPC700ShiftInc<"ror", 0b011>;
defm SPC700_DEC : SPC700ShiftInc<"dec", 0b100>;
defm SPC700_INC : SPC700ShiftInc<"inc", 0b101>;
defm SPC700_STORE_Y : SPC700LoadStoreY<"mov", " , y", 0b110>;
defm SPC700_LOAD_Y : SPC700LoadStoreY<"mov y, ", "", 0b111>;
def SPC700_DEC_Y : Inst8<"dec y", Opcode<0xDC>>;
def SPC700_INC_Y : Inst8<"inc y", Opcode<0xFC>>;

// xD opcodes
defm SPC700_PUSH : SPC700PushPop<"push", 0b0, 0x0D>;
def SPC700_DEC_X : Inst8<"dec x", Opcode<0x1D>>;
def SPC700_INC_X : Inst8<"inc x", Opcode<0x3D>>;
def SPC700_MOV_X_A : Inst8<"mov x , a", Opcode<0x5D>>;
def SPC700_MOV_A_X : Inst8<"mov a , x", Opcode<0x7D>>;
def SPC700_LOAD_Y_Immediate : SPC700Inst16<"mov y ,", "", Opcode<0x8D>, Immediate>;
def SPC700_MOV_X_SP : Inst8<"mov x , sp", Opcode<0x9D>>;
def SPC700_CMP_Y_Immediate : SPC700Inst16<"cmp y ,", "", Opcode<0xAD>, Immediate>;
def SPC700_MOV_SP_X : Inst8<"mov sp , x", Opcode<0xBD>>;
def SPC700_LOAD_X_Immediate : SPC700Inst16<"mov x ,", "", Opcode<0xCD>, Immediate>;
def SPC700_MOV_A_Y : Inst8<"mov a , y", Opcode<0xDD>>;
def SPC700_NOTC_Implied : Inst8<"notc", Opcode<0xED>>;
def SPC700_MOV_Y_A : Inst8<"mov y , a", Opcode<0xFD>>;

// xE opcodes
def SPC700_TSET1 : SPC700Inst24<"tset1", "", Opcode<0x0E>, Absolute>;
defm SPC700_CMP_X : SPC700CmpE<"cmp x ,", 0>;
def SPC700_CBNE_ZeroPageRelative
  : SPC700Inst24TwoOp<"cbne", "", Opcode<0x2E>, SPC700ZeroPageRelative>;
def SPC700_TCLR1 : SPC700Inst24<"tclr1", "", Opcode<0x4E>, Absolute>;
defm SPC700_CMP_Y : SPC700CmpE<"cmp y ,", 1>;
def SPC700_DBNZ_ZeroPageRelative
  : SPC700Inst24TwoOp<"dbnz", "", Opcode<0x6E>, SPC700ZeroPageRelative>;
defm SPC700_POP : SPC700PushPop<"pop", 0b1, 0x0E>;
def SPC700_DIV_YA_X : Inst8<"div ya , x", Opcode<0x9E>>;
def SPC700_DAS_A : Inst8<"das a", Opcode<0xBE>>;
def SPC700_CBNE_ZeroPageXRelative
  : SPC700Inst24TwoOp<"cbne", "", Opcode<0xDE>, SPC700ZeroPageXRelative>;
def SPC700_DBNZ_Y_Relative
  : SPC700Inst16<"dbnz y ,", "", Opcode<0xFE>, Relative>;

// xF opcodes
def SPC700_BRK_Implied : Inst8<"brk", Opcode<0x0F>>;
def SPC700_JMP_IndexedIndirect16
    : Inst24<"jmp", Opcode<0x1F>, SPC700IndexedIndirect16>, InstUnconditionalBranch {
  let isIndirectBranch = true;  
}
def SPC700_BRA_Relative
    : Inst16<"bra", Opcode<0x2F>, Relative>, InstConditionalBranch;
def SPC700_CALL_Absolute : Inst24<"call", Opcode<0x3F>, Absolute>, InstCall;
def SPC700_PCALL_LastPage : Inst16<"pcall", Opcode<0x4F>, ZeroPage>, InstCall;
def SPC700_JMP_Absolute
    : Inst24<"jmp", Opcode<0x5F>, Absolute>, InstUnconditionalBranch;
def SPC700_RET_Implied : Inst8<"ret", Opcode<0x6F>>, InstReturn;
def SPC700_RETI_Implied : Inst8<"reti", Opcode<0x7F>>, InstReturn;
def SPC700_MOV_ZeroPageImmediate
    : SPC700Inst24TwoOp<"mov", "", Opcode<0x8F>, SPC700ZeroPageImmediate>;
def SPC700_XCN_A : Inst8<"xcn a", Opcode<0x9F>>;
def SPC700_STORE_A_IndirectAutoIncrement
    : SPC700Inst8<"mov", ", a", Opcode<0xAF>, SPC700IndirectXAutoIncrement>;
def SPC700_LOAD_A_IndirectAutoIncrement
    : SPC700Inst8<"mov a ,", "", Opcode<0xBF>, SPC700IndirectXAutoIncrement>;
def SPC700_MUL_YA : Inst8<"mul ya", Opcode<0xCF>>;
def SPC700_DAA_A : Inst8<"daa a", Opcode<0xDF>>;
def SPC700_SLEEP_Implied : Inst8<"sleep", Opcode<0xEF>>;
def SPC700_STOP_Implied : Inst8<"stop", Opcode<0xFF>>;

} // DecoderNamespace = "spc700"

// MOS aliases

// x0 opcodes
def : InstAlias<"clp", (SPC700_CLRP_Implied)>;
def : InstAlias<"sep", (SPC700_SETP_Implied)>;
def : InstAlias<"clc", (SPC700_CLRC_Implied)>;
def : InstAlias<"sec", (SPC700_SETC_Implied)>;
def : InstAlias<"cli", (SPC700_EI_Implied)>;
def : InstAlias<"sei", (SPC700_DI_Implied)>;
def : InstAlias<"clv", (SPC700_CLRV_Implied)>;

// x2, x3 opcodes
def : InstAlias<"bbr $tbit , $source , $location", (SPC700_BBC_ZeroPage imm3:$tbit, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"bbs $tbit , $source , $location", (SPC700_BBS_ZeroPage imm3:$tbit, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"rmb $tbit , $source", (SPC700_CLR1_ZeroPage imm3:$tbit, addr8:$source)>;
def : InstAlias<"smb $tbit , $source", (SPC700_SET1_ZeroPage imm3:$tbit, addr8:$source)>;

def : InstAlias<"bbr0 $source , $location", (SPC700_BBC_ZeroPage 0, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"bbr1 $source , $location", (SPC700_BBC_ZeroPage 1, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"bbr2 $source , $location", (SPC700_BBC_ZeroPage 2, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"bbr3 $source , $location", (SPC700_BBC_ZeroPage 3, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"bbr4 $source , $location", (SPC700_BBC_ZeroPage 4, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"bbr5 $source , $location", (SPC700_BBC_ZeroPage 5, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"bbr6 $source , $location", (SPC700_BBC_ZeroPage 6, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"bbr7 $source , $location", (SPC700_BBC_ZeroPage 7, addr8:$source, pcrel8at2:$location)>;

def : InstAlias<"bbs0 $source , $location", (SPC700_BBS_ZeroPage 0, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"bbs1 $source , $location", (SPC700_BBS_ZeroPage 1, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"bbs2 $source , $location", (SPC700_BBS_ZeroPage 2, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"bbs3 $source , $location", (SPC700_BBS_ZeroPage 3, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"bbs4 $source , $location", (SPC700_BBS_ZeroPage 4, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"bbs5 $source , $location", (SPC700_BBS_ZeroPage 5, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"bbs6 $source , $location", (SPC700_BBS_ZeroPage 6, addr8:$source, pcrel8at2:$location)>;
def : InstAlias<"bbs7 $source , $location", (SPC700_BBS_ZeroPage 7, addr8:$source, pcrel8at2:$location)>;

def : InstAlias<"rmb0 $source", (SPC700_CLR1_ZeroPage 0, addr8:$source)>;
def : InstAlias<"rmb1 $source", (SPC700_CLR1_ZeroPage 1, addr8:$source)>;
def : InstAlias<"rmb2 $source", (SPC700_CLR1_ZeroPage 2, addr8:$source)>;
def : InstAlias<"rmb3 $source", (SPC700_CLR1_ZeroPage 3, addr8:$source)>;
def : InstAlias<"rmb4 $source", (SPC700_CLR1_ZeroPage 4, addr8:$source)>;
def : InstAlias<"rmb5 $source", (SPC700_CLR1_ZeroPage 5, addr8:$source)>;
def : InstAlias<"rmb6 $source", (SPC700_CLR1_ZeroPage 6, addr8:$source)>;
def : InstAlias<"rmb7 $source", (SPC700_CLR1_ZeroPage 7, addr8:$source)>;

def : InstAlias<"smb0 $source", (SPC700_SET1_ZeroPage 0, addr8:$source)>;
def : InstAlias<"smb1 $source", (SPC700_SET1_ZeroPage 1, addr8:$source)>;
def : InstAlias<"smb2 $source", (SPC700_SET1_ZeroPage 2, addr8:$source)>;
def : InstAlias<"smb3 $source", (SPC700_SET1_ZeroPage 3, addr8:$source)>;
def : InstAlias<"smb4 $source", (SPC700_SET1_ZeroPage 4, addr8:$source)>;
def : InstAlias<"smb5 $source", (SPC700_SET1_ZeroPage 5, addr8:$source)>;
def : InstAlias<"smb6 $source", (SPC700_SET1_ZeroPage 6, addr8:$source)>;
def : InstAlias<"smb7 $source", (SPC700_SET1_ZeroPage 7, addr8:$source)>;

// x4, x5, x6, x7, x8, x9 opcodes
defm : SPC700MosAliasArithmetic<"ora", "SPC700_OR">;
defm : SPC700MosAliasArithmetic<"and", "SPC700_AND">;
defm : SPC700MosAliasArithmetic<"eor", "SPC700_EOR">;
defm : SPC700MosAliasArithmetic<"cmp", "SPC700_CMP">;
defm : SPC700MosAliasArithmetic<"adc", "SPC700_ADC">;
defm : SPC700MosAliasArithmetic<"sbc", "SPC700_SBC">;
defm : SPC700MosAliasLoadStoreA<"sta", "SPC700_STORE_A">;
defm : SPC700MosAliasLoadStoreA<"lda", "SPC700_LOAD_A">;
defm : SPC700MosAliasLoadStoreX<"stx", "SPC700_STORE_X">;
defm : SPC700MosAliasLoadStoreX<"ldx", "SPC700_LOAD_X">;
def : InstAlias<"cpx #$param", (SPC700_CMP_X_Immediate imm8:$param)>;
def : InstAlias<"lda #$param", (SPC700_LOAD_A_Immediate imm8:$param)>;

// xB, xC opcodes
defm : SPC700MosAliasShiftInc<"asl", "SPC700_ASL">;
defm : SPC700MosAliasShiftInc<"rol", "SPC700_ROL">;
defm : SPC700MosAliasShiftInc<"lsr", "SPC700_LSR">;
defm : SPC700MosAliasShiftInc<"ror", "SPC700_ROR">;
defm : SPC700MosAliasShiftInc<"dec", "SPC700_DEC">;
defm : SPC700MosAliasShiftInc<"inc", "SPC700_INC">;
def : InstAlias<"dey", (SPC700_DEC_Y)>;
def : InstAlias<"iny", (SPC700_INC_Y)>;
defm : SPC700MosAliasLoadStoreY<"sty", "SPC700_STORE_Y">;
defm : SPC700MosAliasLoadStoreY<"ldy", "SPC700_LOAD_Y">;

// xD opcodes
defm : SPC700MosAliasPushPop<"ph", "SPC700_PUSH">;
def : InstAlias<"dex", (SPC700_DEC_X)>;
def : InstAlias<"inx", (SPC700_INC_X)>;
def : InstAlias<"tax", (SPC700_MOV_X_A)>;
def : InstAlias<"txa", (SPC700_MOV_A_X)>;
def : InstAlias<"tsx", (SPC700_MOV_X_SP)>;
def : InstAlias<"txs", (SPC700_MOV_SP_X)>;
def : InstAlias<"tya", (SPC700_MOV_A_Y)>;
def : InstAlias<"tay", (SPC700_MOV_Y_A)>;
def : InstAlias<"ldx #$param", (SPC700_LOAD_X_Immediate imm8:$param)>;
def : InstAlias<"ldy #$param", (SPC700_LOAD_Y_Immediate imm8:$param)>;
def : InstAlias<"cpy #$param", (SPC700_CMP_Y_Immediate imm8:$param)>;

// xE opcodes
def : InstAlias<"cpx $param", (SPC700_CMP_X_ZeroPage addr8:$param)>;
def : InstAlias<"cpy $param", (SPC700_CMP_Y_ZeroPage addr8:$param)>;
def : InstAlias<"cpx $param", (SPC700_CMP_X_Absolute addr16:$param)>;
def : InstAlias<"cpy $param", (SPC700_CMP_Y_Absolute addr16:$param)>;
def : InstAlias<"tsb $addr", (SPC700_TSET1 addr16:$addr)>;
def : InstAlias<"trb $addr", (SPC700_TCLR1 addr16:$addr)>;
defm : SPC700MosAliasPushPop<"pl", "SPC700_POP">;
def : InstAlias<"div", (SPC700_DIV_YA_X)>;
def : InstAlias<"das", (SPC700_DAS_A)>;

// xF opcodes
def : InstAlias<"jsr $addr", (SPC700_CALL_Absolute addr16:$addr)>;
def : InstAlias<"jmp ( $addr , x )", (SPC700_JMP_IndexedIndirect16 addr16:$addr)>;
def : InstAlias<"rts", (SPC700_RET_Implied)>;
def : InstAlias<"rti", (SPC700_RETI_Implied)>;
def : InstAlias<"xcn", (SPC700_XCN_A)>;
def : InstAlias<"mul", (SPC700_MUL_YA)>;
def : InstAlias<"daa", (SPC700_DAA_A)>;
def : InstAlias<"wai", (SPC700_SLEEP_Implied)>;
def : InstAlias<"stp", (SPC700_STOP_Implied)>;

} // Predicates = [HasSPC700]

// Zero page relaxation table entries.

def : ZPIRE< SPC700_ADC_ZeroPage, SPC700_ADC_Absolute >;
def : ZPIRE< SPC700_ADC_ZeroPageX, SPC700_ADC_AbsoluteX >;
def : ZPIRE< SPC700_AND_ZeroPage, SPC700_AND_Absolute >;
def : ZPIRE< SPC700_AND_ZeroPageX, SPC700_AND_AbsoluteX >;
def : ZPIRE< SPC700_ASL_ZeroPage, SPC700_ASL_Absolute >;
def : ZPIRE< SPC700_CMP_ZeroPage, SPC700_CMP_Absolute >;
def : ZPIRE< SPC700_CMP_ZeroPageX, SPC700_CMP_AbsoluteX >;
def : ZPIRE< SPC700_CMP_X_ZeroPage, SPC700_CMP_X_Absolute >;
def : ZPIRE< SPC700_CMP_Y_ZeroPage, SPC700_CMP_Y_Absolute >;
def : ZPIRE< SPC700_DEC_ZeroPage, SPC700_DEC_Absolute >;
def : ZPIRE< SPC700_EOR_ZeroPage, SPC700_EOR_Absolute >;
def : ZPIRE< SPC700_EOR_ZeroPageX, SPC700_EOR_AbsoluteX >;
def : ZPIRE< SPC700_INC_ZeroPage, SPC700_INC_Absolute >;
def : ZPIRE< SPC700_LOAD_A_ZeroPage, SPC700_LOAD_A_Absolute >;
def : ZPIRE< SPC700_LOAD_A_ZeroPageX, SPC700_LOAD_A_AbsoluteX >;
def : ZPIRE< SPC700_LOAD_X_ZeroPage, SPC700_LOAD_X_Absolute >;
def : ZPIRE< SPC700_LOAD_Y_ZeroPage, SPC700_LOAD_Y_Absolute >;
def : ZPIRE< SPC700_STORE_A_ZeroPage, SPC700_STORE_A_Absolute >;
def : ZPIRE< SPC700_STORE_A_ZeroPageX, SPC700_STORE_A_AbsoluteX >;
def : ZPIRE< SPC700_STORE_X_ZeroPage, SPC700_STORE_X_Absolute >;
def : ZPIRE< SPC700_STORE_Y_ZeroPage, SPC700_STORE_Y_Absolute >;
def : ZPIRE< SPC700_LSR_ZeroPage, SPC700_LSR_Absolute >;
def : ZPIRE< SPC700_OR_ZeroPage, SPC700_OR_Absolute >;
def : ZPIRE< SPC700_OR_ZeroPageX, SPC700_OR_AbsoluteX >;
def : ZPIRE< SPC700_ROL_ZeroPage, SPC700_ROL_Absolute >;
def : ZPIRE< SPC700_ROR_ZeroPage, SPC700_ROR_Absolute >;
def : ZPIRE< SPC700_SBC_ZeroPage, SPC700_SBC_Absolute >;
def : ZPIRE< SPC700_SBC_ZeroPageX, SPC700_SBC_AbsoluteX >;

// MOS -> SPC700 translation table.

// x0 opcodes
def : MOSSPC700<CLC_Implied, SPC700_CLRC_Implied>;
def : MOSSPC700<SEC_Implied, SPC700_SETC_Implied>;
def : MOSSPC700<CLI_Implied, SPC700_EI_Implied>;
def : MOSSPC700<SEI_Implied, SPC700_DI_Implied>;
def : MOSSPC700<CLV_Implied, SPC700_CLRV_Implied>;

// x2, x3 opcodes
def : MOSSPC700<BBR_ZeroPage, SPC700_BBC_ZeroPage>;
def : MOSSPC700<BBS_ZeroPage, SPC700_BBS_ZeroPage>;
def : MOSSPC700<RMB_ZeroPage, SPC700_CLR1_ZeroPage>;
def : MOSSPC700<SMB_ZeroPage, SPC700_SET1_ZeroPage>;

// x4, x5, x6, x7, x8, x9 opcodes
defm : SPC700MosSpcArithmetic<"ORA", "SPC700_OR">;
defm : SPC700MosSpcArithmetic<"AND", "SPC700_AND">;
defm : SPC700MosSpcArithmetic<"EOR", "SPC700_EOR">;
defm : SPC700MosSpcArithmetic<"CMP", "SPC700_CMP">;
defm : SPC700MosSpcArithmetic<"ADC", "SPC700_ADC">;
defm : SPC700MosSpcArithmetic<"SBC", "SPC700_SBC">;
defm : SPC700MosSpcLoadStoreA<"STA", "SPC700_STORE_A">;
defm : SPC700MosSpcLoadStoreA<"LDA", "SPC700_LOAD_A">;
defm : SPC700MosSpcLoadStoreX<"STX", "SPC700_STORE_X">;
defm : SPC700MosSpcLoadStoreX<"LDX", "SPC700_LOAD_X">;
def : MOSSPC700<CPX_Immediate, SPC700_CMP_X_Immediate>;
def : MOSSPC700<LDA_Immediate, SPC700_LOAD_A_Immediate>;

// xB, xC opcodes
defm : SPC700MosSpcShiftInc<"ASL", "SPC700_ASL">;
defm : SPC700MosSpcShiftInc<"ROL", "SPC700_ROL">;
defm : SPC700MosSpcShiftInc<"LSR", "SPC700_LSR">;
defm : SPC700MosSpcShiftInc<"ROR", "SPC700_ROR">;
defm : SPC700MosSpcShiftInc<"DEC", "SPC700_DEC">;
defm : SPC700MosSpcShiftInc<"INC", "SPC700_INC">;
def : MOSSPC700<DEY_Implied, SPC700_DEC_Y>;
def : MOSSPC700<INY_Implied, SPC700_INC_Y>;
defm : SPC700MosSpcLoadStoreY<"STY", "SPC700_STORE_Y">;
defm : SPC700MosSpcLoadStoreY<"LDY", "SPC700_LOAD_Y">;

// xD opcodes
def : MOSSPC700<PHP_Implied, SPC700_PUSH_PSW>;
def : MOSSPC700<PHA_Implied, SPC700_PUSH_A>;
def : MOSSPC700<PHX_Implied, SPC700_PUSH_X>;
def : MOSSPC700<PHY_Implied, SPC700_PUSH_Y>;
def : MOSSPC700<DEX_Implied, SPC700_DEC_X>;
def : MOSSPC700<INX_Implied, SPC700_INC_X>;
def : MOSSPC700<TAX_Implied, SPC700_MOV_X_A>;
def : MOSSPC700<TXA_Implied, SPC700_MOV_A_X>;
def : MOSSPC700<TSX_Implied, SPC700_MOV_X_SP>;
def : MOSSPC700<TXS_Implied, SPC700_MOV_SP_X>;
def : MOSSPC700<TYA_Implied, SPC700_MOV_A_Y>;
def : MOSSPC700<TAY_Implied, SPC700_MOV_Y_A>;
def : MOSSPC700<LDX_Immediate, SPC700_LOAD_X_Immediate>;
def : MOSSPC700<LDY_Immediate, SPC700_LOAD_Y_Immediate>;
def : MOSSPC700<CPY_Immediate, SPC700_CMP_Y_Immediate>;

// xE opcodes
def : MOSSPC700<CPX_ZeroPage, SPC700_CMP_X_ZeroPage>;
def : MOSSPC700<CPY_ZeroPage, SPC700_CMP_Y_ZeroPage>;
def : MOSSPC700<CPX_Absolute, SPC700_CMP_X_Absolute>;
def : MOSSPC700<CPY_Absolute, SPC700_CMP_Y_Absolute>;
def : MOSSPC700<TSB_Absolute, SPC700_TSET1>;
def : MOSSPC700<TRB_Absolute, SPC700_TCLR1>;
def : MOSSPC700<PLP_Implied, SPC700_POP_PSW>;
def : MOSSPC700<PLA_Implied, SPC700_POP_A>;
def : MOSSPC700<PLX_Implied, SPC700_POP_X>;
def : MOSSPC700<PLY_Implied, SPC700_POP_Y>;

// xF opcodes
def : MOSSPC700<JSR_Absolute, SPC700_CALL_Absolute>;
def : MOSSPC700<JMP_IndexedIndirect, SPC700_JMP_IndexedIndirect16>;
def : MOSSPC700<RTS_Implied, SPC700_RET_Implied>;
def : MOSSPC700<RTI_Implied, SPC700_RETI_Implied>;
def : MOSSPC700<WAI_Implied, SPC700_SLEEP_Implied>;
def : MOSSPC700<STP_Implied, SPC700_STOP_Implied>;

def MOSSPC700Table : GenericTable {
  let FilterClass = "MOSSPC700";
  let CppTypeName = "MOSSPC700Entry";
  let Fields = ["From", "To"];

  let PrimaryKey = ["From"];
  let PrimaryKeyName = "getMOSSPC700Entry";
}
