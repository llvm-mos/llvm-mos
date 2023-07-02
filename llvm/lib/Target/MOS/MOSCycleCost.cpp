//===-- MOSCycleCost.cpp - MOS Cycle Cost structure -------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains additional helpers for the MOSCycleCost class.
//
//===----------------------------------------------------------------------===//

#include "MOSCycleCost.h"

#include "llvm/IR/Function.h"

using namespace llvm; 

namespace llvm {

int64_t MOSCycleCost::value(MOSCycleCostMode Mode) const {
  switch (Mode) {
  case MOSCycleCostMode::PreferBytes:
    return ((int64_t) Bytes << 32) + Cycles;
  case MOSCycleCostMode::PreferCycles:
    return ((int64_t) Cycles << 32) + Bytes;
  case MOSCycleCostMode::Average:
    return Bytes + Cycles;
  }
}

MOSCycleCostMode getMOSCycleCostModeFor(const MachineFunction &MF) {
  if (MF.getFunction().hasMinSize())
    return MOSCycleCostMode::PreferBytes;
  if (MF.getFunction().hasOptSize() || MF.getFunction().hasOptNone())
    return MOSCycleCostMode::Average;
  return MOSCycleCostMode::PreferCycles;
}

} // namespace llvm
