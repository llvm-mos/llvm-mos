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

def A : MOSReg<0, "a">;
def X : MOSReg<1, "x">;
def Y : MOSReg<2, "y">;
def S : MOSReg<3, "s">;

def C : MOSReg<4, "C">;
def N : MOSReg<5, "N">;
def Z : MOSReg<6, "Z">;

// N and Z are always set together, so model them as one register with two
// one-bit subregisters.
def NZ : MOSReg<7, "NZ"> {
  let SubRegs = [N, Z];
  let SubRegIndices = [subn, subz];
  let CoveredBySubRegs = true;
}

def P : MOSReg<8, "P"> {
  let SubRegs = [C, NZ];
  let SubRegIndices = [subcarry, subnz];
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
class MOSImagReg8<bits<16> num, string name> : MOSReg<num, name>;

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
defvar Imag16RegsOffset = !add(Imag8RegsOffset, MaxImag8Regs);

// Now we enumerate the imaginary registers.
// Imaginary 8-bit registers, starting with the prefix rc
foreach I = 0...!add(MaxImag8Regs, -1) in {
  // There exist MaxImag8Regs rcXX registers...
  def RC#I: MOSImagReg8<!add(I, Imag8RegsOffset), "rc"#!cast<string>(I)>;
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

class MOSReg1Class<dag RegList>: MOSRegClass<[i8], 8, RegList>;
class MOSReg8Class<dag RegList>: MOSRegClass<[i8], 8, RegList>;
class MOSReg16Class<dag RegList>: MOSRegClass<[i16], 8, RegList>;

// Single register classes.
def Ac : MOSReg8Class<(add A)>;
def Yc : MOSReg8Class<(add Y)>;
def Cc : MOSReg1Class<(add C)>;

// "General purpose" register classes.
def GPR : MOSReg8Class<(add A, X, Y)>;
def Imag8 : MOSReg8Class<(sequence "RC%u", 0, !add(MaxImag8Regs, -1))>;
def Imag16 : MOSReg16Class<(sequence "RS%u", 0, !add(MaxImag16Regs, -1))>;

// Instruction-specific register classes.
def AImag8 : MOSReg8Class<(add A, Imag8)>;
def XY : MOSReg8Class<(add X, Y)>;
def Flag : MOSReg1Class<(add C, N, Z)> {
  let isAllocatable = false;
}

// MOS assembly mnemonics may only have the following registers as parameters
// following the comma in a mnemonic, e.g. lda hello, x
def MOSAsmParamRegClass : MOSReg8Class<(add X, Y, S)>;

// Sum types of all available storage locations for a size.
def Anyi1 : MOSReg1Class<(add GPR, Imag8, Cc)>;
def Anyi8 : MOSReg8Class<(add GPR, Imag8)>;

// Any location where any supported size could be placed.
def Anyc : MOSRegClass<[i1, i8, i16], 8, (add Anyi8, Imag16)> {
  let isAllocatable = false;
}
