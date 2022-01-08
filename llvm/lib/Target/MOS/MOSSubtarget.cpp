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
#include "llvm/MC/TargetRegistry.h"

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
    : MOSGenSubtargetInfo(TT, CPU, /* TuneCPU */ CPU, FS), InstrInfo(),
      RegInfo(), FrameLowering(),
      TLInfo(TM, initializeSubtargetDependencies(CPU, FS, TM)),
      CallLoweringInfo(&TLInfo), Legalizer(*this),
      InstSelector(createMOSInstructionSelector(TM, *this, RegBankInfo)),
      InlineAsmLoweringInfo(&TLInfo) {}

MOSSubtarget &
MOSSubtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                              const TargetMachine &TM) {
  // Parse features string.
  ParseSubtargetFeatures(CPU, /* TuneCPU */ CPU, FS);

  // Convert feature bits to e_flags
  EFlags = MOS_MC::makeEFlags(getFeatureBits());

  return *this;
}

void MOSSubtarget::overrideSchedPolicy(MachineSchedPolicy &Policy,
                                       unsigned NumRegionInstrs) const {
  // Force register pressure tracking; by default it's disabled for small
  // regions, but it's the only 6502 scheduling concern.
  Policy.ShouldTrackPressure = true;

  Policy.OnlyBottomUp = false;
  Policy.OnlyTopDown = false;
}
