//===-- MOSSubtarget.cpp - MOS Subtarget Information ----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the MOS specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "MOSSubtarget.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/Support/TargetRegistry.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSTargetMachine.h"

#define DEBUG_TYPE "mos-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "MOSGenSubtargetInfo.inc"

namespace llvm {

MOSSubtarget::MOSSubtarget(const Triple &TT, const std::string &CPU,
                           const std::string &FS, const MOSTargetMachine &TM)
    : MOSGenSubtargetInfo(TT, CPU, FS), InstrInfo(), FrameLowering(),
      TLInfo(TM, initializeSubtargetDependencies(CPU, FS, TM)), TSInfo(),

      // Subtarget features
      m_hasTinyEncoding(false), 

      m_Has6502Insns(false),
      m_Has6502BCDInsns(false),
      m_Has6502XInsns(false),
      m_Has65C02Insns(false),
      m_HasR65C02Insns(false),
      m_HasW65C02Insns(false),
      m_HasW65816Insns(false),
      m_Has65EL02Insns(false),
      m_Has65CE02Insns(false),
      m_HasSWEET16Insns(false),
      
      ELFArch(false),
      m_FeatureSetDummy(false) {
  // Parse features string.
  ParseSubtargetFeatures(CPU, FS);
}

const llvm::TargetFrameLowering *MOSSubtarget::getFrameLowering() const {
  return &FrameLowering;
}

const llvm::MOSInstrInfo *MOSSubtarget::getInstrInfo() const {
  return &InstrInfo;
}

const llvm::MOSRegisterInfo *MOSSubtarget::getRegisterInfo() const {
  return &InstrInfo.getRegisterInfo();
}

const llvm::MOSSelectionDAGInfo *MOSSubtarget::getSelectionDAGInfo() const {
  return &TSInfo;
}

const llvm::MOSTargetLowering *MOSSubtarget::getTargetLowering() const {
  return &TLInfo;
}

MOSSubtarget &
MOSSubtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                              const TargetMachine &TM) {
  // Parse features string.
  ParseSubtargetFeatures(CPU, FS);
  return *this;
}

} // namespace llvm
