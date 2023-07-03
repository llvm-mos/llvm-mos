//===-- MOSInstrCost.cpp - MOS Instruction Cost structure -------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains additional helpers for the MOSInstrCost class.
//
//===----------------------------------------------------------------------===//

#include "MOSInstrCost.h"

#include "llvm/IR/Function.h"

using namespace llvm; 

namespace llvm {

int64_t MOSInstrCost::value(Mode Mode) const {
  switch (Mode) {
  case Mode::PreferBytes:
    return ((int64_t) Bytes << 32) + Cycles;
  case Mode::PreferCycles:
    return ((int64_t) Cycles << 32) + Bytes;
  case Mode::Average:
    return Bytes + Cycles;
  }
}

MOSInstrCost::Mode MOSInstrCost::getModeFor(const MachineFunction &MF) {
  if (MF.getFunction().hasMinSize())
    return Mode::PreferBytes;
  if (MF.getFunction().hasOptSize() || MF.getFunction().hasOptNone())
    return Mode::Average;
  return Mode::PreferCycles;
}

} // namespace llvm
