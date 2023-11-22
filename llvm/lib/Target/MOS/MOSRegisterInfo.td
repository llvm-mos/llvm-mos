//===-- MOSRegisterInfo.td - MOS Register defs -------------*- tablegen -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Subregister indices
//===----------------------------------------------------------------------===//

class MOSSubRegIndex<int size, int offset = 0>: SubRegIndex<size, offset> {
  let Namespace = "MOS";
}

// Low byte of a 16-bit imaginary pointer register.
def sublo : MOSSubRegIndex<8>;

// High byte of a 16-bit imaginary pointer register.
def subhi : MOSSubRegIndex<8, 8>;

// Carry bit of the 8-bit processor status register.
def subcarry : MOSSubRegIndex<1>;

// Two NZ bits of the 8-bit processor status register.
def subnz : MOSSubRegIndex<2>;

// N bit of the 2-bit NZ register.
def subn : MOSSubRegIndex<1>;

// Z bit of the 2-bit NZ register.
def subz : MOSSubRegIndex<1>;

// V bit of the 8-bit processor status register.
def subv : MOSSubRegIndex<1>;

// Least-significant bit of an 8-bit register. Writes to this are considered to
// write zeroes to the other bits (for now), so we can reduce LSB writes to
// zero-extension.
def sublsb : MOSSubRegIndex<1>;

//===----------------------------------------------------------------------===//
//  Real registers
//===----------------------------------------------------------------------===//

// All MOS registers have the prefix llvm_mos_ added to them, to avoid conflicts
// with legacy assembly code that depends on the register names as variables.
// (The MOS instructions generally don't name the target register as a
// parameter. The target registers are implied by the opcode.)
class MOSReg<bits<16> num, string name>
  : Register<!strconcat("llvm_mos_", name)>, DwarfRegNum<[num]> {
  field bits<16> Num = num;
  let HWEncoding = num;
  let Namespace = "MOS";
  let AltNames = [name];
}

// An 8-bit value register (GPR or imaginary). Contains a 1-bit LSB subregister
// to allow them to store boolean values.
multiclass MOSReg8<bits<16> num, string name> {
  def LSB : MOSReg<!add(num, 1), name#"LSB">;

  def NAME : MOSReg<num, name> {
    let SubRegs = [!cast<Register>(NAME#LSB)];
    let SubRegIndices = [sublsb];
  }
}

defm A : MOSReg8<0, "a">;
defm X : MOSReg8<2, "x">;
defm Y : MOSReg8<4, "y">;
def S : MOSReg<6, "s">;

def C : MOSReg<7, "C">;
def N : MOSReg<8, "N">;
def V : MOSReg<9, "V">;
def Z : MOSReg<10, "Z">;

// N and Z are always set together, so model them as one register with two
// one-bit subregisters.
def NZ : MOSReg<11, "NZ"> {
  let SubRegs = [N, Z];
  let SubRegIndices = [subn, subz];
  let CoveredBySubRegs = true;
}

def P : MOSReg<12, "P"> {
  let SubRegs = [C, NZ, V];
  let SubRegIndices = [subcarry, subnz, subv];
}

//===----------------------------------------------------------------------===//
//  Imaginary registers
//===----------------------------------------------------------------------===//

// This section describes the dimension of the imaginary registers for the MOS
// targets. Imaginary registers are locations in the zero page that llvm can
// treat as physical registers for the purpose of register allocation.

// Imaginary registers are reserved in pairs, which allows them to be used as
// pointers with the 6502's indirect addressing modes. The number of imaginary
// register pointer pairs is configurable at code generation time via the
// --num-imag-ptrs flag. See MOSRegisterInfo.cpp for details.

// The two bytes of any given pointer must be contiguous, but otherwise, there
// are no constraints on the relative layout of the pointers. For example, RS0
// could be allocated to 0x34 and 0x35, while RS1 is allocated to 0x12 and 0x34.
// The code generator emits generic symbol references for imaginary registers,
// and the linker script for a given memory layout later assigns these to their
// real locations.

// The first 16-bit imaginary register is reserved as the soft stack pointer.
// The second 16-bit imaginary register is reserved as a frame pointer whenever
// necessary, on a function-by-function basis.

// 8-bit imaginary registers.
multiclass MOSImagReg8<bits<16> num, string name> : MOSReg8<num, name>;

// 16-bit imaginary registers (consecutive pairs of 8-bit registers).
class MOSImagReg16<bits<16> num, string name, list<Register> subregs>
    : MOSReg<num, name> {
  let SubRegs = subregs;
  let SubRegIndices = [sublo, subhi];
  let CoveredBySubRegs = 1;
}

defvar MaxImag8Regs = 256;
// The starting DWARF number for the imaginary registers.
defvar Imag8RegsOffset = 0x10;

defvar MaxImag16Regs = !sra(MaxImag8Regs, 1);
defvar Imag16RegsOffset = !add(Imag8RegsOffset, !shl(MaxImag8Regs,1));

// Now we enumerate the imaginary registers.
// Imaginary 8-bit registers, starting with the prefix rc
foreach I = 0...!add(MaxImag8Regs, -1) in {
  // There exist MaxImag8Regs rcXX registers...
  defm RC#I: MOSImagReg8<!add(!shl(I,1), Imag8RegsOffset), "rc"#!cast<string>(I)>;
}

// Imaginary 16-bit registers, starting with the prefix rs
foreach I = 0...!add(MaxImag16Regs, -1) in {
  // There exist MaxImag16Regs rsXX registers...
  def RS#I: MOSImagReg16<!add(I, Imag16RegsOffset), "rs"#!cast<string>(I),
      [!cast<Register>("RC"#!shl(I, 1)),
       !cast<Register>("RC"#!add(!shl(I,1),1))]>;
}

//===----------------------------------------------------------------------===//
//  Register classes
//===----------------------------------------------------------------------===//

class MOSRegClass<list<ValueType> regTypes, int alignment, dag regList>
     : RegisterClass<"MOS", regTypes, alignment, regList>;

class MOSReg1Class<dag RegList>: MOSRegClass<[i1], 8, RegList> {
  // The register is 1-bit wide, but takes 8 bits to spill.
  let RegInfos = RegInfoByHwMode<[DefaultMode], [RegInfo<1, 8, 8>]>;
}
class MOSReg8Class<dag RegList>: MOSRegClass<[i8], 8, RegList>;
class MOSReg16Class<dag RegList>: MOSRegClass<[i16], 8, RegList>;

// Single register classes.
let isPressureFineGrained = true in {
  def Ac : MOSReg8Class<(add A)>;
  def Xc : MOSReg8Class<(add X)>;
  def Yc : MOSReg8Class<(add Y)>;
  def Cc : MOSReg1Class<(add C)>;
  def Vc : MOSReg1Class<(add V)>;
  def Pc : MOSReg8Class<(add P)>;
}

// "General purpose" register classes.
let isPressureFineGrained = true in {
  def GPR : MOSReg8Class<(add X, Y, A)>;
  def GPR_LSB : MOSReg1Class<(add XLSB, YLSB, ALSB)>;
  def CV_GPR_LSB : MOSReg1Class<(add Cc, GPR_LSB, Vc)>;
}
def Imag8 : MOSReg8Class<(sequence "RC%u", 0, !add(MaxImag8Regs, -1))>;
def Imag16 : MOSReg16Class<(sequence "RS%u", 0, !add(MaxImag16Regs, -1))>;

// Instruction-specific register classes.
def AImag8 : MOSReg8Class<(add Imag8, A)>;
let isPressureFineGrained = true in {
  def AY : MOSReg8Class<(add A, Y)>;
  def XY : MOSReg8Class<(add X, Y)>;
}
let isAllocatable = false in {
  def Flag : MOSReg1Class<(add C, N, V, Z)>;
  def AP : MOSReg8Class<(add A, P)>;
  def AXYP : MOSReg8Class<(add A, X, Y, P)>;
}

// While not directly used, unless this is present, the TableGen-erated intersection
// register class between Anyi1 and Flags will not be allocatable.
def AllocatableFlags : MOSReg1Class<(add Cc, Vc)>;

// MOS assembly mnemonics may only have the following registers as parameters
// following the comma in a mnemonic, e.g. lda hello, x
def MOSAsmParamRegClass : MOSReg8Class<(add X, Y, S)>;

// Sum types of all available storage locations for a size.

def Anyi1 : MOSReg1Class<(add (sequence "RC%uLSB", 0, !add(MaxImag8Regs, -1)), CV_GPR_LSB)>;
def Anyi8 : MOSReg8Class<(add Imag8, GPR)>;

def Fake : MOSReg<13, "Fake">;
let isAllocatable = false in {
  def Fakei32 : MOSRegClass<[i32], 8, (add Fake)>;
  def Fakei64 : MOSRegClass<[i64], 8, (add Fake)>;
}
