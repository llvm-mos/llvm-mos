//===-- MOSRegisterInfo.cpp - MOS Register Information --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MOS implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "MOSRegisterInfo.h"

#include "llvm/ADT/BitVector.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/ErrorHandling.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSInstrInfo.h"
#include "MOSTargetMachine.h"

#define GET_REGINFO_TARGET_DESC
#include "MOSGenRegisterInfo.inc"

namespace llvm {

MOSRegisterInfo::MOSRegisterInfo() : MOSGenRegisterInfo(0) {}

void MOSRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator MI,
                                          int SPAdj, unsigned FIOperandNum,
                                          RegScavenger *RS /*= NULL*/) const {
  report_fatal_error("Not yet implemented.");
}

const uint16_t *
MOSRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF /*= 0*/) const {
  report_fatal_error("Not yet implemented.");
}

Register MOSRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  report_fatal_error("Not yet implemented.");
}

llvm::BitVector
MOSRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  report_fatal_error("Not yet implemented.");
}

} // end of namespace llvm
