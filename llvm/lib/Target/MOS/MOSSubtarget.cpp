//===-- MOSSubtarget.cpp - MOS Subtarget Information ----------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the MOS specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "MOSSubtarget.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/CodeGen/GlobalISel/CallLowering.h"
#include "llvm/Support/TargetRegistry.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSTargetMachine.h"

#define DEBUG_TYPE "mos-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "MOSGenSubtargetInfo.inc"

using namespace llvm;

MOSSubtarget::MOSSubtarget(const Triple &TT, const std::string &CPU,
                           const std::string &FS, const MOSTargetMachine &TM)
    : MOSGenSubtargetInfo(TT, CPU, /* TuneCPU */ CPU, FS), InstrInfo(),
      RegInfo(), FrameLowering(),
      TLInfo(TM, initializeSubtargetDependencies(CPU, FS, TM)),
      CallLoweringInfo(&TLInfo),

      // Subtarget features
      m_hasTinyEncoding(false),

      m_Has6502Insns(false), m_Has6502BCDInsns(false), m_Has6502XInsns(false),
      m_Has65C02Insns(false), m_HasR65C02Insns(false), m_HasW65C02Insns(false),
      m_HasW65816Insns(false), m_Has65EL02Insns(false), m_Has65CE02Insns(false),
      m_HasSWEET16Insns(false),

      m_LongRegisterNames(false),

      ELFArch(0), m_FeatureSetDummy(false) {
  // Parse features string.
  ParseSubtargetFeatures(CPU, /* TuneCPU */ CPU, FS);
}

const llvm::TargetFrameLowering *MOSSubtarget::getFrameLowering() const {
  return &FrameLowering;
}

const llvm::MOSInstrInfo *MOSSubtarget::getInstrInfo() const {
  return &InstrInfo;
}

const llvm::MOSRegisterInfo *MOSSubtarget::getRegisterInfo() const {
  return &RegInfo;
}

const llvm::MOSTargetLowering *MOSSubtarget::getTargetLowering() const {
  return &TLInfo;
}

const CallLowering *MOSSubtarget::getCallLowering() const {
  return &CallLoweringInfo;
}

MOSSubtarget &
MOSSubtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                              const TargetMachine &TM) {
  // Parse features string.
  ParseSubtargetFeatures(CPU, /* TuneCPU */ CPU, FS);
  return *this;
}
