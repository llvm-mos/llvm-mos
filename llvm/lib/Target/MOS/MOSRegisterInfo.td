//===-- MOSRegisterInfo.td - MOS Register defs -------------*- tablegen -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Real registers
//===----------------------------------------------------------------------===//

// The 65xx series of microprocessors uses an 8-bit accumulator register, A, for
// nearly all math operations.  Index registers X and Y are 8-bits.  On the
// 65816 series of microprocessors, these registers are extended to 16 bits.
// There is an 8-bit stack pointer, S, that refers to memory starting from
// 0x1ff and growing downward.
// The 65xx also has a processor status register, P, that holds carry, zero,
// negative, and overflow bits which are set as the result of the most
// recent math operation.
// The 6502's program counter register, PC, is 16 bits in length.

// All MOS registers have the prefix llvm_mos_ added to them, to avoid
// conflicts with legacy assembly code that depends on the register names
// as variables.  (The MOS instructions generally don't name the target
// register as a parameter.  The target registers are implied by the opcode.)
class MOSReg<bits<16> num,
             string name,
             list<Register> subregs = [],
             list<string> altNames = []
 >
  : RegisterWithSubRegs<!strconcat("llvm_mos_", name), subregs>,
    DwarfRegNum<[num]>
{
  field bits<16> Num = num;
  let HWEncoding = num;
  let Namespace = "MOS";
  let SubRegs = subregs;
  let AltNames = [name];
}

class MOSReg8Class< dag RegList >: RegisterClass< "MOS", [i8], 8, RegList >;
class MOSReg16Class< dag RegList >: RegisterClass< "MOS", [i16], 16, RegList >;

def A : MOSReg< 0, "a" >;
def X : MOSReg< 1, "x" >;
def Y : MOSReg< 2, "y" >;
def P : MOSReg< 3, "p" >;
def S : MOSReg< 4, "s">;
def PC : MOSReg< 5, "pc">;

// Enumerate the instanced classes of 8 and 16 bit real registers.
def RealReg8 : MOSReg8Class<(add A, X, Y, P, S)>;

//===----------------------------------------------------------------------===//
//  Imaginary registers
//===----------------------------------------------------------------------===//

// This section describes the dimension of the imaginary registers for the MOS
// targets.  We call them imaginary, because LLVM has a different meaning for
// virtual.  Imaginary registers are a bank of continuous memory in MOS
// zero page, that llvm can treat as physical registers for the purpose of
// register allocation.  This puts llvm in the position of managing a large
// chunk of zero page memory.

// Define the size and DWARF numbering for these imaginary registers.  These
// constants were chosen so that the number of imaginary registers could be
// changed at one central location, and the changes will propagate through
// the backend.

// The number of imaginary character registers. The whole zero page is filled
// with imaginary registers, but the code generator can be configured to only
// use a fraction of them. The first 16-bit imaginary register is reserved as
// the soft stack pointer. The second 16-bit imaginary register is reserved as a
// frame pointer whenever necessary, on a function-by-function basis.
defvar MaxImag8Regs = 256;
// The starting DWARF number for the imaginary registers.
defvar Imag8RegsOffset = 0x10;

defvar MaxImag16Regs = !sra(MaxImag8Regs, 1);
defvar Imag16RegsOffset = !add(Imag8RegsOffset, MaxImag8Regs);

// Define subregister indexes, for imaginary registers.  The only 65xx series
// processor with overlapping registers is the 65816 and variants.  The 8-bit
// variants do not overlap.
class MOSSubRegIndex<int size, int offset = 0>: SubRegIndex<size, offset>
{
  let Namespace = "MOS";
}

// Define the ability to extract subregisters from imaginary registers.
def sub_8lo       : MOSSubRegIndex<8>;
def sub_8hi       : MOSSubRegIndex<8, 8>;

// Definition for 8-bit imaginary registers.
class MOSImagReg8<bits<16> num,
            string name,
            list<Register> subregs = [],
            list<string> altNames = []>
        : MOSReg< num, name, subregs, altNames >;

// Definition for 16-bit imaginary registers.
class MOSImagReg16<bits<16> num,
            string name,
            list<Register> subregs = [],
            list<string> altNames = []>
            : MOSReg< num, name, subregs, altNames>
{
  let SubRegIndices = [sub_8lo, sub_8hi];
  let CoveredBySubRegs = 1;
}

// Now we enumerate the imaginary registers.
// Imaginary 8-bit registers, starting with the prefix rc
foreach I = 0...!add(MaxImag8Regs, -1) in
{
  // There exist MaxImag8Regs rcXX registers...
  def RC#I:
    MOSImagReg8< !add(I, Imag8RegsOffset), "rc"#!cast<string>(I) >;
 }
// And they are all part of the ImagReg8.
def ImagReg8 :
  RegisterClass<"MOS", [i8], 8, (sequence "RC%u", 0, !add(MaxImag8Regs, -1))>;

// Imaginary 16-bit registers, starting with the prefix rs
foreach I = 0...!add(MaxImag16Regs, -1) in
{
  // There exist MaxImag16Regs rsXX registers...
  def RS#I:
  MOSImagReg16< !add(I, Imag16RegsOffset), "rs"#!cast<string>(I),
    [!cast<Register>("RC"#!shl(I, 1)),
      !cast<Register>("RC"#!add(!shl(I,1),1))]>;
}
// And they are all part of the ImagReg16.
def ImagReg16 :
  RegisterClass<"MOS", [i16], 16,
    (sequence "RS%u", 0, !add(MaxImag16Regs, -1))>;

// Register banks are more-or-less unused, so place all registers in one.
def AnyRegBank : RegisterBank<"Any", [RealReg8, ImagReg8, ImagReg16]>;
