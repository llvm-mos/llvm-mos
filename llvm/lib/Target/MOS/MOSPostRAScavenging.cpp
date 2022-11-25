//===-- MOSPostRAScavenging.cpp - MOS Post RA Scavenging ------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MOS post-register-allocation register scavenging pass.
//
// This pass runs immediately after post-RA pseudo expansion. These pseudos
// (including COPY) often require temporary registers on MOS; moreso than on
// other platforms. Accordingly, they emit virtual registers instead, and this
// pass performs register scavenging to assign them to physical registers,
// freeing them up via save and restore if neccesary. A very similar process is
// performed in prologue/epilogue insertion.
//
//===----------------------------------------------------------------------===//

#include "MOSPostRAScavenging.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/RegisterScavenging.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOS.h"

#define DEBUG_TYPE "mos-scavenging"

using namespace llvm;

namespace {

class MOSPostRAScavenging : public MachineFunctionPass {
public:
  static char ID;

  MOSPostRAScavenging() : MachineFunctionPass(ID) {
    llvm::initializeMOSPostRAScavengingPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

bool MOSPostRAScavenging::runOnMachineFunction(MachineFunction &MF) {
  if (MF.getProperties().hasProperty(
          MachineFunctionProperties::Property::NoVRegs))
    return false;

  RegScavenger RS;
  scavengeFrameVirtualRegs(MF, RS);

  return true;
}

} // namespace

char MOSPostRAScavenging::ID = 0;

INITIALIZE_PASS(MOSPostRAScavenging, DEBUG_TYPE,
                "Scavenge virtual registers emitted by post-RA pseudos", false,
                false)

MachineFunctionPass *llvm::createMOSPostRAScavengingPass() {
  return new MOSPostRAScavenging();
}
