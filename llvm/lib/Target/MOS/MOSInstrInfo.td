//===-- MOSInstrInfo.td - MOS Instruction defs -------------*- tablegen -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes the MOS instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

/// These instruction format definitions exist, thanks to Neil Parker's
/// analysis of the 65xx instruction set, at:
/// http://nparker.llx.com/a2/opcodes.html
/// Parker's analysis is closely reflected in the structure of this file, and
/// his words are quoted liberally herein.  Parker's sense-making of the 6502
/// and family was critical in creating these format definitions, and we're
/// indebted to him for the excellent analysis.
///
/// These instruction definitions provide the instruction set for the LLVM MC
/// layer. They correspond to the real instruction set of the 6502. Given their
/// irregularity, the side effects and behavior of these instructions are not
/// themselves modeled. Instead, a regularized virtual instruction set was formed
/// out of pseudo-instructions, and these were fully modelled for the code
/// generator's purposes. Lowering of these pseudoinstructions occurs in
/// AsmPrinter, which is the boundary between the CodeGen and MC layers.

include "MOSInstrFormats.td"

defm ORA : CC1_All<0b000, "ora", [Has6502]>;
defm AND : CC1_All<0b001, "and", [Has6502]>;
defm EOR : CC1_All<0b010, "eor", [Has6502]>;
defm ADC : CC1_All<0b011, "adc", [Has6502]>;
defm LDA : CC1_All<0b101, "lda", [Has6502]>;
defm CMP : CC1_All<0b110, "cmp", [Has6502]>;
defm SBC : CC1_All<0b111, "sbc", [Has6502]>;

/// The STA # instruction would be nonsensical, so skip it.
defm STA : CC1_NoImmediate<0b100, "sta", [Has6502]>;

let Predicates = [Has6502] in {

/// CC2 regular instructions
defm ASL : CC2_Shift<0b000, "asl">;
defm ROL : CC2_Shift<0b001, "rol">;
defm LSR : CC2_Shift<0b010, "lsr">;
defm ROR : CC2_Shift<0b011, "ror">;

def : InstAlias<"asl a", (ASL_Accumulator)>;
def : InstAlias<"rol a", (ROL_Accumulator)>;
def : InstAlias<"lsr a", (LSR_Accumulator)>;
def : InstAlias<"ror a", (ROR_Accumulator)>;

defm DEC : CC2_NonShift<0b110, "dec">;
defm INC : CC2_NonShift<0b111, "inc">;

/// CC2 exceptional instructions
///             ASL ROL LSR ROR STX LDX DEC INC
/// #                               A2
/// zp          06  26  46  66  86  A6  C6  E6
/// A           0A  2A  4A  6A
/// abs         0E  2E  4E  6E  8E  AE  CE  EE
/// zp,X/zp,Y   16  36  56  76  96  B6  D6  F6
/// abs,X/abs,Y	1E	3E	5E	7E      BE  DE  FE

def STX_ZeroPage :
	Inst16<"stx", OpcodeC2<0b100, 0b001>, ZeroPage>;
def STX_ZeroPageY :
	Inst16<"stx", OpcodeC2<0b100, 0b101>, ZeroPageY>;
def STX_Absolute :
	Inst24<"stx", OpcodeC2<0b100, 0b011>, Absolute>;

def LDX_Immediate :
	Inst16<"ldx", OpcodeC2<0b101, 0b000>, Immediate>;
def LDX_ZeroPage :
	Inst16<"ldx", OpcodeC2<0b101, 0b001>, ZeroPage>;
def LDX_Absolute :
	Inst24<"ldx", OpcodeC2<0b101, 0b011>, Absolute>;
def LDX_ZeroPageY :
	Inst16<"ldx", OpcodeC2<0b101, 0b101>, ZeroPageY>;
def LDX_AbsoluteY :
	Inst24<"ldx", OpcodeC2<0b101, 0b111>, AbsoluteY>;

def DEC_AbsoluteX :
	Inst24<"dec", OpcodeC2<0b110, 0b111>, AbsoluteX>;
def INC_AbsoluteX :
	Inst24<"inc", OpcodeC2<0b111, 0b111>, AbsoluteX>;


/// CC0 regular instructions, the nine in the top left corner of this diagram:
///       BIT JMP JMP() STY LDY CPY CPX
/// #                       A0  C0  E0
/// zp    24            84  A4  C4  E4
/// abs   2C  4C  6C    8C  AC  CC  EC
/// zp,X                94  B4
/// abs,X                   BC
///
/// 101 LDY
/// 110 CPY
/// 111 CPX

defm LDY : CC0_Regular<0b101, "ldy">;
defm CPY : CC0_Regular<0b110, "cpy">;
defm CPX : CC0_Regular<0b111, "cpx">;

/// CC0 irregular instructions, e.g. most of them

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

def BIT_ZeroPage :
	Inst16<"bit", OpcodeC0<0b001, 0b001>, ZeroPage>;
def BIT_Absolute :
	Inst24<"bit", OpcodeC0<0b001, 0b011>, Absolute>;
def JMP_Absolute :
	Inst24<"jmp", OpcodeC0<0b010, 0b011>, Absolute>;
def JMP_Indirect16 :
	Inst24<"jmp", OpcodeC0<0b011, 0b011>, Indirect16>;
def STY_ZeroPage :
	Inst16<"sty", OpcodeC0<0b100, 0b001>, ZeroPage>;
def STY_Absolute :
	Inst24<"sty", OpcodeC0<0b100, 0b011>, Absolute>;
def STY_ZeroPageX :
	Inst16<"sty", OpcodeC0<0b100, 0b101>, ZeroPageX>;
def LDY_ZeroPageX :
	Inst16<"ldy", OpcodeC0<0b101, 0b101>, ZeroPageX>;
def LDY_AbsoluteX :
	Inst24<"ldy", OpcodeC0<0b101, 0b111>, AbsoluteX>;

/// The conditional branch instructions all have the form xxy10000. The flag
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

def BPL_Relative : ConditionalBranch<"bpl", 0b00, 0b0>;
def BMI_Relative : ConditionalBranch<"bmi", 0b00, 0b1>;
def BVC_Relative : ConditionalBranch<"bvc", 0b01, 0b0>;
def BVS_Relative : ConditionalBranch<"bvs", 0b01, 0b1>;
def BCC_Relative : ConditionalBranch<"bcc", 0b10, 0b0>;
def BCS_Relative : ConditionalBranch<"bcs", 0b10, 0b1>;
def BNE_Relative : ConditionalBranch<"bne", 0b11, 0b0>;
def BEQ_Relative : ConditionalBranch<"beq", 0b11, 0b1>;

/// Single-byte opcodes
/// BRK JSR abs RTI RTS
/// 00  20      40  60
/// (JSR is the only absolute-addressing instruction that doesn't fit the
/// aaabbbcc pattern.)
///
/// Other single-byte instructions:
/// PHP PLP PHA PLA DEY TAY INY INX
/// 08  28  48  68  88  A8  C8  E8
/// CLC SEC CLI SEI TYA CLV CLD SED
/// 18  38  58  78  98  B8  D8  F8
/// TXA TXS TAX TSX DEX NOP
/// 8A  9A  AA  BA  CA  EA

def BRK_Implied: InstLow0<"brk", 0b0000>;
/// JSR is the only instruction that does not follow the current pattern.
def JSR_Absolute: Inst24<"jsr", Opcode<0x20>, Absolute>;
def RTI_Implied: InstLow0<"rti", 0b0100>;
def RTS_Implied: InstLow0<"rts", 0b0110>;

def PHP_Implied: InstLow8<"php", 0b0000>;
def PLP_Implied: InstLow8<"plp", 0b0010>;
def PHA_Implied: InstLow8<"pha", 0b0100>;
def PLA_Implied: InstLow8<"pla", 0b0110>;
def DEY_Implied: InstLow8<"dey", 0b1000>;
def TAY_Implied: InstLow8<"tay", 0b1010>;
def INY_Implied: InstLow8<"iny", 0b1100>;
def INX_Implied: InstLow8<"inx", 0b1110>;

def CLC_Implied: InstLow8<"clc", 0b0001>;
def SEC_Implied: InstLow8<"sec", 0b0011>;
def CLI_Implied: InstLow8<"cli", 0b0101>;
def SEI_Implied: InstLow8<"sei", 0b0111>;
def TYA_Implied: InstLow8<"tya", 0b1001>;
def CLV_Implied: InstLow8<"clv", 0b1011>;
def CLD_Implied: InstLow8<"cld", 0b1101>;
def SED_Implied: InstLow8<"sed", 0b1111>;

def TXA_Implied: InstLowA<"txa", 0b1000>;
def TXS_Implied: InstLowA<"txs", 0b1001>;
def TAX_Implied: InstLowA<"tax", 0b1010>;
def TSX_Implied: InstLowA<"tsx", 0b1011>;
def DEX_Implied: InstLowA<"dex", 0b1100>;
def NOP_Implied: InstLowA<"nop", 0b1110>;

} // Predicates = [Has6502]

/// Opcode naming and order follows the "NMOS 6510 Unintended Opcodes" eBook.

let Predicates = [Has6502XOrDTV] in {

let DecoderNamespace = "6502x" in {

defm SLO : CC3_NoImmediate<0b000, "slo">;
defm RLA : CC3_NoImmediate<0b001, "rla">;
defm SRE : CC3_NoImmediate<0b010, "sre">;
defm RRA : CC3_NoImmediate<0b011, "rra">;

def SAX_IndexedIndirect : Inst16<"sax", OpcodeC3<0b100, 0b000>, IndexedIndirect>;
def SAX_ZeroPage : Inst16<"sax", OpcodeC3<0b100, 0b001>, ZeroPage>;
def SAX_Absolute : Inst24<"sax", OpcodeC3<0b100, 0b011>, Absolute>;
def SAX_ZeroPageY : Inst16<"sax", OpcodeC3<0b100, 0b101>, ZeroPageY>;

def LAX_IndexedIndirect : Inst16<"lax", OpcodeC3<0b101, 0b000>, IndexedIndirect>;
def LAX_ZeroPage : Inst16<"lax", OpcodeC3<0b101, 0b001>, ZeroPage>;
def LAX_Absolute : Inst24<"lax", OpcodeC3<0b101, 0b011>, Absolute>;
def LAX_IndirectIndexed : Inst16<"lax", OpcodeC3<0b101, 0b100>, IndirectIndexed>;
def LAX_ZeroPageY : Inst16<"lax", OpcodeC3<0b101, 0b101>, ZeroPageY>;
def LAX_AbsoluteY : Inst24<"lax", OpcodeC3<0b101, 0b111>, AbsoluteY>;

defm DCP : CC3_NoImmediate<0b110, "dcp">;
defm ISC : CC3_NoImmediate<0b111, "isc">;

def ALR_Immediate: Inst16<"alr", OpcodeC3<0b010, 0b010>, Immediate>;
def ARR_Immediate: Inst16<"arr", OpcodeC3<0b011, 0b010>, Immediate>;

def ANE_Immediate: Inst16<"ane", OpcodeC3<0b100, 0b010>, Immediate>;

def LAX_Immediate: Inst16<"lax", OpcodeC3<0b101, 0b010>, Immediate>;

}

} // Predicates = [Has6502XOrDTV]

let Predicates = [Has6502X] in {

let DecoderNamespace = "6502x" in {

def ANC_Immediate: Inst16<"anc", OpcodeC3<0b001, 0b010>, Immediate>;
def SBX_Immediate: Inst16<"sbx", OpcodeC3<0b110, 0b010>, Immediate>;

def LAS_AbsoluteY : Inst24<"las", OpcodeC3<0b101, 0b110>, AbsoluteY>;

def SHA_IndirectIndexed : Inst16<"sha", OpcodeC3<0b100, 0b100>, IndirectIndexed>;
def SHA_AbsoluteY : Inst24<"sha", OpcodeC3<0b100, 0b111>, AbsoluteY>;

def SHX_AbsoluteY : Inst24<"shx", OpcodeC2<0b100, 0b111>, AbsoluteY>;

def SHY_AbsoluteX : Inst24<"shy", OpcodeC0<0b100, 0b111>, AbsoluteX>;

def TAS_AbsoluteY : Inst24<"tas", OpcodeC3<0b100, 0b110>, AbsoluteY>;

def NOP_Immediate: Inst16<"nop", OpcodeC2<0b000, 0b110>, Immediate>;
def NOP_ZeroPage: Inst16<"nop", OpcodeC0<0b000, 0b001>, ZeroPage>;
def NOP_ZeroPageX: Inst16<"nop", OpcodeC0<0b000, 0b101>, ZeroPageX>;
def NOP_AbsoluteX: Inst24<"nop", OpcodeC0<0b000, 0b111>, AbsoluteX>;

}

} // Predicates = [Has6502X]

let Predicates = [Has65DTV02] in {

let DecoderNamespace = "65dtv02" in {

def BRA_Relative_DTV02 : Inst16<"bra", Opcode<0x12>, Relative>;
def SAC_Immediate : Inst16<"sac", Opcode<0x32>, Immediate>;
def SIR_Immediate : Inst16<"sir", Opcode<0x42>, Immediate>;

}

} // Predicates = [Has65DTV02]

let Predicates = [Has65C02] in {

/// Single-byte opcodes
/// INCA DECA PHY PLY PHX PLX
/// 1A   3A   5A  7A  DA  FA

def INC_Accumulator: InstLowA<"inc", 0b0001>;
def DEC_Accumulator: InstLowA<"dec", 0b0011>;
def : InstAlias<"inc a", (INC_Accumulator)>;
def : InstAlias<"dec a", (DEC_Accumulator)>;
def : InstAlias<"ina", (INC_Accumulator), 0>;
def : InstAlias<"dea", (DEC_Accumulator), 0>;
def PHY_Implied: InstLowA<"phy", 0b0101>; /// Push Y
def PLY_Implied: InstLowA<"ply", 0b0111>; /// Pull Y
def PHX_Implied: InstLowA<"phx", 0b1101>; /// Push X
def PLX_Implied: InstLowA<"plx", 0b1111>; /// Pull X

/// Unconditional relative branch instruction
/// BRA
/// 80

def BRA_Relative : Inst16<"bra", OpcodeC0<0b100, 0b000>, Relative>;

def JMP_IndexedIndirect : Inst24<"jmp", Opcode<0x7c>, IndexedIndirect16>;

def BIT_ZeroPageX : Inst16<"bit", Opcode<0x34>, ZeroPageX>;
def BIT_AbsoluteX : Inst24<"bit", Opcode<0x3c>, AbsoluteX>;
def BIT_Immediate : Inst16<"bit", Opcode<0x89>, Immediate>;

def TRB_ZeroPage : Inst16<"trb", Opcode<0x14>, ZeroPage>; /// Test and Reset Bits
def TRB_Absolute : Inst24<"trb", Opcode<0x1c>, Absolute>; /// Test and Reset Bits

def TSB_ZeroPage : Inst16<"tsb", Opcode<0x04>, ZeroPage>; /// Test and Set Bits
def TSB_Absolute : Inst24<"tsb", Opcode<0x0c>, Absolute>; /// Test and Set Bits

///       STZ
/// zp    64
/// abs   9C
/// zp,X  74
/// abs,X 9E

def STZ_ZeroPage : Inst16<"stz", OpcodeC0<0b011, 0b001>, ZeroPage>;
def STZ_Absolute : Inst24<"stz", OpcodeC0<0b100, 0b111>, Absolute>;
def STZ_ZeroPageX : Inst16<"stz", OpcodeC0<0b011, 0b101>, ZeroPageX>;
def STZ_AbsoluteX : Inst24<"stz", OpcodeC2<0b100, 0b111>, AbsoluteX>;

} // Predicates = [Has65C02]

let Predicates = [HasR65C02] in {

let DecoderNamespace = "r65c02" in {
/// Branch Bit Reset
def BBR0_ZeroPage : InstBranchBit<"bbr0", Opcode<0x0f>>;
def BBR1_ZeroPage : InstBranchBit<"bbr1", Opcode<0x1f>>;
def BBR2_ZeroPage : InstBranchBit<"bbr2", Opcode<0x2f>>;
def BBR3_ZeroPage : InstBranchBit<"bbr3", Opcode<0x3f>>;
def BBR4_ZeroPage : InstBranchBit<"bbr4", Opcode<0x4f>>;
def BBR5_ZeroPage : InstBranchBit<"bbr5", Opcode<0x5f>>;
def BBR6_ZeroPage : InstBranchBit<"bbr6", Opcode<0x6f>>;
def BBR7_ZeroPage : InstBranchBit<"bbr7", Opcode<0x7f>>;

/// Branch Bit Set
def BBS0_ZeroPage : InstBranchBit<"bbs0", Opcode<0x8f>>;
def BBS1_ZeroPage : InstBranchBit<"bbs1", Opcode<0x9f>>;
def BBS2_ZeroPage : InstBranchBit<"bbs2", Opcode<0xaf>>;
def BBS3_ZeroPage : InstBranchBit<"bbs3", Opcode<0xbf>>;
def BBS4_ZeroPage : InstBranchBit<"bbs4", Opcode<0xcf>>;
def BBS5_ZeroPage : InstBranchBit<"bbs5", Opcode<0xdf>>;
def BBS6_ZeroPage : InstBranchBit<"bbs6", Opcode<0xef>>;
def BBS7_ZeroPage : InstBranchBit<"bbs7", Opcode<0xff>>;

/// Reset Memory Bit
def RMB0_ZeroPage : Inst16<"rmb0", Opcode<0x07>, ZeroPage>;
def RMB1_ZeroPage : Inst16<"rmb1", Opcode<0x17>, ZeroPage>;
def RMB2_ZeroPage : Inst16<"rmb2", Opcode<0x27>, ZeroPage>;
def RMB3_ZeroPage : Inst16<"rmb3", Opcode<0x37>, ZeroPage>;
def RMB4_ZeroPage : Inst16<"rmb4", Opcode<0x47>, ZeroPage>;
def RMB5_ZeroPage : Inst16<"rmb5", Opcode<0x57>, ZeroPage>;
def RMB6_ZeroPage : Inst16<"rmb6", Opcode<0x67>, ZeroPage>;
def RMB7_ZeroPage : Inst16<"rmb7", Opcode<0x77>, ZeroPage>;

/// Set Memory Bit
def SMB0_ZeroPage : Inst16<"smb0", Opcode<0x87>, ZeroPage>;
def SMB1_ZeroPage : Inst16<"smb1", Opcode<0x97>, ZeroPage>;
def SMB2_ZeroPage : Inst16<"smb2", Opcode<0xa7>, ZeroPage>;
def SMB3_ZeroPage : Inst16<"smb3", Opcode<0xb7>, ZeroPage>;
def SMB4_ZeroPage : Inst16<"smb4", Opcode<0xc7>, ZeroPage>;
def SMB5_ZeroPage : Inst16<"smb5", Opcode<0xd7>, ZeroPage>;
def SMB6_ZeroPage : Inst16<"smb6", Opcode<0xe7>, ZeroPage>;
def SMB7_ZeroPage : Inst16<"smb7", Opcode<0xf7>, ZeroPage>;
}

} // Predicates = [HasR65C02]

let Predicates = [HasW65C02] in {

def STP_Implied : InstLowB<"stp", 0b1101>;
def WAI_Implied : InstLowB<"wai", 0b1100>;

} // Predicates = [HasW65C02]

let Predicates = [Has65CE02] in {

/// Layout follows CMOS 65CE02 micrprocessor datasheet:
/// http://archive.6502.org/datasheets/mos_65ce02_mpu.pdf

/// Branches and jumps

let DecoderNamespace = "65ce02" in {
def BPL_Relative16 : Inst24<"bpl", Opcode<0x13>, Relative16>;
def BMI_Relative16 : Inst24<"bmi", Opcode<0x33>, Relative16>;
def BVC_Relative16 : Inst24<"bvc", Opcode<0x53>, Relative16>;
def BSR_Relative16 : Inst24<"bsr", Opcode<0x63>, Relative16>;
def BVS_Relative16 : Inst24<"bvs", Opcode<0x73>, Relative16>;
def BRA_Relative16 : Inst24<"bra", Opcode<0x83>, Relative16>; /// BRU
def BCC_Relative16 : Inst24<"bcc", Opcode<0x93>, Relative16>;
def BCS_Relative16 : Inst24<"bcs", Opcode<0xb3>, Relative16>;
def BNE_Relative16 : Inst24<"bne", Opcode<0xd3>, Relative16>;
def BEQ_Relative16 : Inst24<"beq", Opcode<0xf3>, Relative16>;

def JSR_AbsoluteIndirect: Inst24<"jsr", Opcode<0x22>, Indirect16>;
def JSR_AbsoluteIndirectX: Inst24<"jsr", Opcode<0x23>, IndexedIndirect16>;

def RTN_Immediate : Inst16<"rtn", Opcode<0x62>, Immediate>;

/// Arithmetic Operations

def NEG_Implied: InstLow2<"neg", 0b0100>; /// NEGate (or 2's complement) accumulator

def ASR_Implied: InstLow3<"asr", 0b0100>; /// Arithmetic Shift Right accumulator
def ASR_ZeroPage: Inst16<"asr", Opcode<0x44>, ZeroPage>; 
def ASR_ZeroPageX: Inst16<"asr", Opcode<0x54>, ZeroPageX>; 

def INW_ZeroPage : Inst16<"inw", OpcodeC3<0b111, 0b000>, ZeroPage>; /// Inc. word
def DEW_ZeroPage : Inst16<"dew", OpcodeC3<0b110, 0b000>, ZeroPage>; /// Dec. word

def INZ_Implied: InstLowB<"inz", 0b0001>; /// INcrement Z register;
def DEZ_Implied: InstLowB<"dez", 0b0011>; /// DEcrement Z register;

def ASW_Absolute : Inst24<"asw", Opcode<0xcb>, Absolute>; /// Arithmetic Shift left Word
def ROW_Absolute : Inst24<"row", Opcode<0xeb>, Absolute>; /// ROtate left Word

def ORA_IndirectZeroPageZ : Inst16<"ora", Opcode<0x12>, IndirectZeroPageZ>;
def AND_IndirectZeroPageZ : Inst16<"and", Opcode<0x32>, IndirectZeroPageZ>;
def EOR_IndirectZeroPageZ : Inst16<"eor", Opcode<0x52>, IndirectZeroPageZ>;
def ADC_IndirectZeroPageZ : Inst16<"adc", Opcode<0x72>, IndirectZeroPageZ>;
def CMP_IndirectZeroPageZ : Inst16<"cmp", Opcode<0xd2>, IndirectZeroPageZ>;
def SBC_IndirectZeroPageZ : Inst16<"sbc", Opcode<0xf2>, IndirectZeroPageZ>;

def CPZ_Immediate : Inst16<"cpz", OpcodeC2<0b110, 0b000>, Immediate>;
def CPZ_ZeroPage : Inst16<"cpz", OpcodeC0<0b110, 0b101>, ZeroPage>;
def CPZ_Absolute : Inst24<"cpz", OpcodeC0<0b110, 0b111>, Absolute>;

/// Loads, Stores, Pushes, Pulls and Transfers

def LDA_IndirectZeroPageZ : Inst16<"lda", OpcodeC2<0b101, 0b100>, IndirectZeroPageZ>;
def LDA_IndirectStackRelativeY_CE02 : Inst16<"lda", Opcode<0xe2>, IndirectStackRelativeY>;

def LDZ_Immediate : Inst16<"ldz", OpcodeC3<0b101, 0b000>, Immediate>;
def LDZ_Absolute : Inst24<"ldz", OpcodeC3<0b101, 0b010>, Absolute>;
def LDZ_AbsoluteX : Inst24<"ldz", OpcodeC3<0b101, 0b110>, AbsoluteX>;

def STA_IndirectZeroPageZ : Inst16<"sta", Opcode<0x92>, IndirectZeroPageZ>;
def STA_IndirectStackRelativeY_CE02 : Inst16<"sta", Opcode<0x82>, IndirectStackRelativeY>;
def STX_AbsoluteY : Inst24<"stx", Opcode<0x9b>, AbsoluteY>;
def STY_AbsoluteX : Inst24<"sty", Opcode<0x8b>, AbsoluteX>;

def CLE_Implied: InstLow2<"cle", 0b0000>; /// CLear stack Extend disable bit
def SEE_Implied: InstLow3<"see", 0b0000>; /// SEt stack Extend disable bit

def PHW_Immediate : Inst24<"phw", Opcode<0xf4>, Immediate16>; /// PusH Data Immediate (Word)
def PHW_Absolute : Inst24<"phw", Opcode<0xfc>, Absolute>; /// PusH Data Absolute (Word)

def PHZ_Implied: InstLowB<"phz", 0b1101>; /// PusH Z register onto stack
def PLZ_Implied: InstLowB<"plz", 0b1111>; /// PulL Z register from stack

def TAZ_Implied: InstLowB<"taz", 0b0100>; /// Transfer Accumulator to Z register
def TZA_Implied: InstLowB<"tza", 0b0110>; /// Transfer Z register to Accumulator

def TAB_Implied: InstLowB<"tab", 0b0101>; /// Transfer Accumulator to Base page reg.
def TBA_Implied: InstLowB<"tba", 0b0111>; /// Transfer Base page register to Acc.

def TSY_Implied: InstLowB<"tsy", 0b0000>; /// Transfer Stack pointer high byte to Y
def TYS_Implied: InstLowB<"tys", 0b0010>; /// Transfer Y reg. to Stack pointer high byte
}

} // Predicates = [Has65CE02]

let Predicates = [Has4510] in {

let DecoderNamespace = "65ce02" in {
def MAP_Implied: Inst8<"map", Opcode<0x5c>>;
}

def : MnemonicAlias<"eom", "nop">;

} // Predicates = [Has4510]

let Predicates = [HasHUC6280] in {

let DecoderNamespace = "huc6280" in {
/// Swap, Clear Registers
def SXY_Implied : InstLow2<"sxy", 0b0000>; /// Swap X and Y
def SAX_Implied : InstLow2<"sax", 0b0010>; /// Swap A and X
def SAY_Implied : InstLow2<"say", 0b0100>; /// Swap A and Y
def CLA_Implied : InstLow2<"cla", 0b0110>; /// Clear A
def CLX_Implied : InstLow2<"clx", 0b1000>; /// Clear X
def CLY_Implied : InstLow2<"cly", 0b1100>; /// Clear Y

/// VDC Control
def ST0_Immediate : Inst16<"st0", Opcode<0x03>, Immediate>; /// Store to VDC Port 0
def ST1_Immediate : Inst16<"st1", Opcode<0x13>, Immediate>; /// Store to VDC Port 1
def ST2_Immediate : Inst16<"st2", Opcode<0x23>, Immediate>; /// Store to VDC Port 2

/// Memory Control
def TMA_Immediate : Inst16<"tma", Opcode<0x43>, Immediate>; /// Store Memory Mapping Register (Mask) in A
def TAM_Immediate : Inst16<"tam", Opcode<0x53>, Immediate>; /// Store A in Memory Mapping Register (Mask)

/// Block Move
def TII_HuCBlockMove : InstHuCBlockMove<"tii", Opcode<0x73>>; /// Transfer Increment Increment
def TDD_HuCBlockMove : InstHuCBlockMove<"tdd", Opcode<0xC3>>; /// Transfer Decrement Decrement
def TIN_HuCBlockMove : InstHuCBlockMove<"tin", Opcode<0xD3>>; /// Transfer Increment None
def TIA_HuCBlockMove : InstHuCBlockMove<"tia", Opcode<0xE3>>; /// Transfer Increment Alternate
def TAI_HuCBlockMove : InstHuCBlockMove<"tai", Opcode<0xF3>>; /// Transfer Alternate Increment

/// Test
def TST_ZeroPage : InstImmediate24<"tst", Opcode<0x83>, ZeroPageAt2>; // Test Immediate with Zero Page
def TST_Absolute : InstImmediate32<"tst", Opcode<0x93>, AbsoluteAt2>; // Test Immediate with Absolute
def TST_ZeroPageX : InstImmediate24<"tst", Opcode<0xA3>, ZeroPageXAt2>; // Test Immediate with Zero Page, X
def TST_AbsoluteX : InstImmediate32<"tst", Opcode<0xB3>, AbsoluteXAt2>; // Test Immediate with Absolute, X

/// Relative Branch
def BSR_Relative : Inst16<"bsr", Opcode<0x44>, Relative>;

/// CPU Control
def CSL_Implied : InstLow4<"csl", 0b0101>; /// Set CPU Clock Low
def CSH_Implied : InstLow4<"csh", 0b1101>; /// Set CPU Clock High
def SET_Implied : InstLow4<"set", 0b1111>; /// Set T Flag
}

}

let Predicates = [HasW65816Or65EL02] in {

/// Direct P flag set/reset with mask immediate
/// SEP REP
/// E2  C2

def SEP_Immediate : Inst16<"sep", OpcodeC2<0b111, 0b000>, Immediate>;
def REP_Immediate : Inst16<"rep", OpcodeC2<0b110, 0b000>, Immediate>;

/// 16-bit immediate load
/// LDX
/// A2

def LDX_Immediate16 : Inst24<"ldx", OpcodeC2<0b101, 0b000>, Immediate16> {
  let DecoderNamespace = "MOSXLow";
}

/// 16-bit indexed-indirect jumps
/// JSR
/// FC

def JSR_IndexedIndirect16 :
  Inst24<"jsr", OpcodeC0<0b111, 0b111>, IndexedIndirect16>;

/// Stack manipulation instructions
/// PEA PEI
/// F4  D4

def PEA_Absolute : Inst24<"pea", Opcode<0xF4>, Absolute>;
def PEI_Indirect : Inst16<"pei", Opcode<0xD4>, Indirect>;

/// Register transfer instructions
/// TXY TYX
/// 9B  BB

def TXY_Implied : InstLowB<"txy", 0b1001>;
def TYX_Implied : InstLowB<"tyx", 0b1011>;

/// Register exchange instructions
/// XBA XCE
/// EB  FB

def XBA_Implied : InstLowB<"xba", 0b1110>;
def XCE_Implied : InstLowB<"xce", 0b1111>;

} // Predicates = [HasW65816Or65EL02]

let Predicates = [HasW65816] in {

let DecoderNamespace = "w65816" in {
/// Unconditional relative branch instruction
/// BRL
/// 82

def BRL_Relative16 : Inst24<"brl", OpcodeC2<0b100, 0b000>, Relative16>;

/// 24-bit long-jumps
/// JMP JSL
/// 5C  22

def JMP_AbsoluteLong : Inst32<"jmp", OpcodeC0<0b010, 0b111>, AbsoluteLong>;
def JSL_AbsoluteLong : Inst32<"jsl", OpcodeC2<0b001, 0b000>, AbsoluteLong>;

/// Return from long-jump subroutine
/// RTL
/// 6B

def RTL_Implied : Inst8<"rtl", OpcodeC3<0b011, 0b010>>;

/// 16-bit indirect long-jump
/// JML
/// DC
def JML_Indirect16 : Inst24<"jml", OpcodeC0<0b110, 0b111>, Indirect16>;

/// Memory moves
/// MVN MVP
/// 54  44

def MVN_816MemoryMove : Inst816MemoryMove<"mvn", Opcode<0x54>>; // Move Negative
def MVP_816MemoryMove : Inst816MemoryMove<"mvp", Opcode<0x44>>; // Move Positive

/// Stack manipulation instructions
/// PER PHB PHD PHK PLB PLD
/// 62  8B  0B  4B  AB  2B

def PER_Relative16 : Inst24<"per", Opcode<0x62>, Relative16>;
def PHB_Implied : InstLowB<"phb", 0b1000>;
def PHD_Implied : InstLowB<"phd", 0b0000>;
def PHK_Implied : InstLowB<"phk", 0b0100>;
def PLB_Implied : InstLowB<"plb", 0b1010>;
def PLD_Implied : InstLowB<"pld", 0b0010>;

/// Register transfer instructions
/// TCD TCS TDC TSC
/// 5B  1B  7B  3B

def TCD_Implied : InstLowB<"tcd", 0b0101>;
def TCS_Implied : InstLowB<"tcs", 0b0001>;
def TDC_Implied : InstLowB<"tdc", 0b0111>;
def TSC_Implied : InstLowB<"tsc", 0b0011>;

def WDM_Immediate : Inst16<"wdm", Opcode<0x42>, Immediate>;
}

} // Predicates = [HasW65816]

let Predicates = [Has65EL02] in {

let DecoderNamespace = "65el02" in {
def NXT_Implied : InstLow2<"nxt", 0b0000>;
def ENT_Implied : InstLow2<"ent", 0b0010>;
def NXA_Implied : InstLow2<"nxa", 0b0100>;

def REA_Absolute : Inst24<"rea", Opcode<0x44>, Absolute>;
def REI_Indirect : Inst16<"rei", Opcode<0x54>, Indirect>;

def PER_Relative : Inst16<"per", Opcode<0x62>, Relative>;
def RER_Relative : Inst16<"rer", Opcode<0x82>, Relative>;

def RHI_Implied : InstLowB<"rhi", 0b0000>;
def RHX_Implied : InstLowB<"rhx", 0b0001>;
def RLI_Implied : InstLowB<"rli", 0b0010>;
def RLX_Implied : InstLowB<"rlx", 0b0011>;
def RHA_Implied : InstLowB<"rha", 0b0100>;
def RHY_Implied : InstLowB<"rhy", 0b0101>;
def RLA_Implied : InstLowB<"rla", 0b0110>;
def RLY_Implied : InstLowB<"rly", 0b0111>;
def TXR_Implied : InstLowB<"txr", 0b1000>;
def TRX_Implied : InstLowB<"trx", 0b1010>;

def TXI_Implied : Inst8<"txi", Opcode<0x5C>>;
def TIX_Implied : Inst8<"tix", Opcode<0xDC>>;

def MUL_ZeroPage_EL02  : Inst16<"mul", Opcode<0x0F>, ZeroPage>;
def MUL_ZeroPageX_EL02 : Inst16<"mul", Opcode<0x1F>, ZeroPageX>;
def MUL_Absolute_EL02  : Inst24<"mul", Opcode<0x2F>, Absolute>;
def MUL_AbsoluteX_EL02 : Inst24<"mul", Opcode<0x3F>, AbsoluteX>;
def DIV_ZeroPage_EL02  : Inst16<"div", Opcode<0x4F>, ZeroPage>;
def DIV_ZeroPageX_EL02 : Inst16<"div", Opcode<0x5F>, ZeroPageX>;
def DIV_Absolute_EL02  : Inst24<"div", Opcode<0x6F>, Absolute>;
def DIV_AbsoluteX_EL02 : Inst24<"div", Opcode<0x7F>, AbsoluteX>;

def ZEA_Implied : Inst8<"zea", Opcode<0x8F>>;
def SEA_Implied : Inst8<"sea", Opcode<0x9F>>;
def TDA_Implied : Inst8<"tda", Opcode<0xAF>>;
def TAD_Implied : Inst8<"tad", Opcode<0xBF>>;
def PLD_Implied_EL02 : Inst8<"pld", Opcode<0xCF>>;
def PHD_Implied_EL02 : Inst8<"phd", Opcode<0xDF>>;
def MMU_Immediate : Inst16<"mmu", Opcode<0xEF>, Immediate>;

}

} // Predicates = [Has65EL02]

// Operand type for basic block and function labels.
def label : Operand<OtherVT>;

include "MOSInstrInfoTables.td"
include "MOSInstrPseudos.td"
include "MOSInstrLogical.td"
include "MOSInstrGISel.td"
