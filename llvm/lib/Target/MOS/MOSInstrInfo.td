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
include "MOSInstrGISel.td"

let Predicates = [Has6502] in
{
defm ORA : CC1_All<0b000, "ora">;
defm AND : CC1_All<0b001, "and">;
defm EOR : CC1_All<0b010, "eor">;
defm ADC : CC1_All<0b011, "adc">;
defm LDA : CC1_All<0b101, "lda">;
defm CMP : CC1_All<0b110, "cmp">;
defm SBC : CC1_All<0b111, "sbc">;

/// The STA # instruction would be nonsensical, so skip it.
defm STA : CC1_NoImmediate<0b100, "sta">;

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
def JMP_Indirect :
	Inst24<"jmp", OpcodeC0<0b011, 0b011>, Indirect>;
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

include "MOSInstrInfoTables.td"
include "MOSInstrPseudos.td"
