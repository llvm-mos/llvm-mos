//===-- MOSRegisterInfo.td - MOS Register defs -------------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the MOS register file
//===----------------------------------------------------------------------===//


// Most real registers on 6502 are 8 bits.
class RealRegInfo: RegInfo< 8, 8, 8>;

// Only the program counter is 16 bits.
class PCRegInfo: RegInfo< 16, 16, 16 >;

// 8-bit General purpose register definition.
class MOSReg<bits<16> num,
             string name,
             list<Register> subregs = [],
             list<string> altNames = []
 >
  : RegisterWithSubRegs<name, subregs>
{
  field bits<16> Num = num;
  
  let HWEncoding = num;
  let Namespace = "MOS";
  let SubRegs = subregs;
  let AltNames = altNames;
  bit isVirtual = 0;
}
 
class MOSReg8Class< dag RegList, int priority = 32 >: 
  RegisterClass< "MOS", [i8], 8, RegList >
  {
    let AllocationPriority = priority;
  }

class MOSReg16Class< dag RegList, int priority = 33 >: 
  RegisterClass< "MOS", [i16], 16, RegList >
  {
    let AllocationPriority = priority;
  }

def A : MOSReg< 64, "A" >, DwarfRegNum<[64]>;
def AClass : MOSReg8Class<(add A), 0>;

def X : MOSReg< 65, "X" >, DwarfRegNum<[65]>;
def XClass : MOSReg8Class<(add X), 1>;

def Y : MOSReg< 66, "Y" >, DwarfRegNum<[66]>;
def YClass : MOSReg8Class<(add Y), 2>;

