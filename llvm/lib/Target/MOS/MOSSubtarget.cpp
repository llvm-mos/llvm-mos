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
#include "llvm/CodeGen/GlobalISel/InstructionSelector.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineScheduler.h"
#include "llvm/Support/TargetRegistry.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"
#include "MOSFrameLowering.h"
#include "MOSInstructionSelector.h"
#include "MOSLegalizerInfo.h"
#include "MOSTargetMachine.h"

#define DEBUG_TYPE "mos-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "MOSGenSubtargetInfo.inc"

using namespace llvm;

MOSSubtarget::MOSSubtarget(const Triple &TT, const std::string &CPU,
                           const std::string &FS, const MOSTargetMachine &TM)
    : MOSGenSubtargetInfo(TT, CPU, /* TuneCPU */ CPU, FS),
      ELFArch(0),
      InstrInfo(),
      RegInfo(), FrameLowering(),
      TLInfo(TM, initializeSubtargetDependencies(CPU, FS, TM)),
      CallLoweringInfo(&TLInfo),
      InstSelector(createMOSInstructionSelector(TM, *this, RegBankInfo)),
      InlineAsmLoweringInfo(&TLInfo),

      // Subtarget features
      m_hasTinyEncoding(false),

      m_Has6502Insns(false), m_Has6502BCDInsns(false), m_Has6502XInsns(false),
      m_Has65C02Insns(false), m_HasR65C02Insns(false), m_HasW65C02Insns(false),
      m_HasW65816Insns(false), m_Has65EL02Insns(false), m_Has65CE02Insns(false),
      m_HasSWEET16Insns(false),

      m_LongRegisterNames(false),

      m_FeatureSetDummy(false) {}

const MOSFrameLowering *MOSSubtarget::getFrameLowering() const {
  return &FrameLowering;
}

const MOSInstrInfo *MOSSubtarget::getInstrInfo() const {
  return &InstrInfo;
}

const MOSRegisterInfo *MOSSubtarget::getRegisterInfo() const {
  return &RegInfo;
}

const MOSTargetLowering *MOSSubtarget::getTargetLowering() const {
  return &TLInfo;
}

const CallLowering *MOSSubtarget::getCallLowering() const {
  return &CallLoweringInfo;
}

const LegalizerInfo *MOSSubtarget::getLegalizerInfo() const {
  return &Legalizer;
}

const RegisterBankInfo *MOSSubtarget::getRegBankInfo() const {
  return &RegBankInfo;
}

InstructionSelector *MOSSubtarget::getInstructionSelector() const {
  return InstSelector.get();
}

const InlineAsmLowering *MOSSubtarget::getInlineAsmLowering() const {
  return &InlineAsmLoweringInfo;
}

MOSSubtarget &
MOSSubtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                              const TargetMachine &TM) {
  // Parse features string.
  ParseSubtargetFeatures(CPU, /* TuneCPU */ CPU, FS);
  return *this;
}

void MOSSubtarget::overrideSchedPolicy(MachineSchedPolicy &Policy,
                                           unsigned NumRegionInstrs) const {
  // Force register pressure tracking; by default it's disabled for small
  // regions, but it's the only 6502 scheduling concern.
  Policy.ShouldTrackPressure = true;
  Policy.ShouldTrackLaneMasks = true;

  Policy.OnlyBottomUp = false;
  Policy.OnlyTopDown = false;
}
