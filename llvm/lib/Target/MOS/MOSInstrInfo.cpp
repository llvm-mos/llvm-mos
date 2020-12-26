//===-- MOSInstrInfo.cpp - MOS Instruction Information --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MOS implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "MOSInstrInfo.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCContext.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#include "MOS.h"
#include "MOSMachineFunctionInfo.h"
#include "MOSRegisterInfo.h"
#include "MOSTargetMachine.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"

#define GET_INSTRINFO_CTOR_DTOR
#include "MOSGenInstrInfo.inc"

namespace llvm {

MOSInstrInfo::MOSInstrInfo()
    : MOSGenInstrInfo(0, 0), RI(), ID() {}


} // end of namespace llvm

