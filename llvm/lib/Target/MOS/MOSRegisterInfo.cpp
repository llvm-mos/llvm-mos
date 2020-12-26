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
#include "llvm/IR/Function.h"
#include "llvm/CodeGen/TargetFrameLowering.h"

#include "MOS.h"
#include "MOSInstrInfo.h"
#include "MOSTargetMachine.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"

#define GET_REGINFO_TARGET_DESC
#include "MOSGenRegisterInfo.inc"

namespace llvm {

MOSRegisterInfo::MOSRegisterInfo() : MOSGenRegisterInfo(0) {}

void MOSRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator MI,
                                          int SPAdj, unsigned FIOperandNum,
                                          RegScavenger *RS /*= NULL*/) const {
  // todo
}

const uint16_t *
MOSRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF /*= 0*/) const {
  return nullptr;
}

const uint32_t *
MOSRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                      CallingConv::ID CC) const {
  return nullptr;
}

Register MOSRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  return 0;
}

const llvm::TargetRegisterClass *
MOSRegisterInfo::getLargestLegalSuperClass(const TargetRegisterClass *RC,
                                           const MachineFunction &MF) const {
  return nullptr;
}

const llvm::TargetRegisterClass *
MOSRegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                    unsigned Kind /*= 0*/) const {
  return nullptr;
}

llvm::BitVector
MOSRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  return BitVector(1);
}

bool MOSRegisterInfo::trackLivenessAfterRegAlloc(
    const MachineFunction &) const {
  return true;
}

} // end of namespace llvm
