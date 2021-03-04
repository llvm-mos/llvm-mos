//===-- MOSISelLowering.cpp - MOS DAG Lowering Implementation -------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that MOS uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#include "MOSISelLowering.h"

#include "llvm/ADT/StringSwitch.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/ErrorHandling.h"

#include "MOS.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"
#include "MOSTargetMachine.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"

using namespace llvm;

MOSTargetLowering::MOSTargetLowering(const MOSTargetMachine &TM,
                                     const MOSSubtarget &STI)
    : TargetLowering(TM), Subtarget(STI) {
  // This is only used for CallLowering to determine how to split large
  // primitive types for the calling convention. All need to be split to 8 bits,
  // so that's all that we report here. The register class is irrelevant.
  addRegisterClass(MVT::i8, &MOS::Anyi8RegClass);
  computeRegisterProperties(STI.getRegisterInfo());
}