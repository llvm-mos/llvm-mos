//===-- MOSCycleCost.h - MOS Cycle Cost structure ---------------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the definition of the MOSCycleCost class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSCYCLECOST_H
#define LLVM_LIB_TARGET_MOS_MOSCYCLECOST_H

#include "llvm/CodeGen/MachineFunction.h"
#include <cstdint>

namespace llvm {

enum class MOSCycleCostMode {
  PreferBytes,
  PreferCycles,
  Average
};

class MOSCycleCost {
public:
  MOSCycleCost() : Bytes(0), Cycles(0) {}

  MOSCycleCost(int32_t Bytes, int32_t Cycles, int Multiplier = 256)
    : Bytes(Bytes * Multiplier), Cycles(Cycles * Multiplier) {}

  friend MOSCycleCost operator+(MOSCycleCost Left,
                                const MOSCycleCost& Right) {
    return MOSCycleCost(Left.Bytes + Right.Bytes,
                        Left.Cycles + Right.Cycles, 1);
  }

  MOSCycleCost& operator+=(const MOSCycleCost& Right) {
    this->Bytes += Right.Bytes;
    this->Cycles += Right.Cycles;
    return *this;
  }

  friend MOSCycleCost operator-(MOSCycleCost Left,
                                const MOSCycleCost& Right) {
    return MOSCycleCost(Left.Bytes - Right.Bytes,
                        Left.Cycles - Right.Cycles, 1);
  }

  MOSCycleCost& operator-=(const MOSCycleCost& Right) {
    this->Bytes -= Right.Bytes;
    this->Cycles -= Right.Cycles;
    return *this;
  }

  friend MOSCycleCost operator*(MOSCycleCost Left, int Right) {
    return MOSCycleCost(Left.Bytes * Right, Left.Cycles * Right, 1);
  }

  friend MOSCycleCost operator/(MOSCycleCost Left, int Right) {
    return MOSCycleCost(Left.Bytes / Right, Left.Cycles / Right, 1);
  }

  int64_t value(MOSCycleCostMode Mode = MOSCycleCostMode::Average) const;

private:
  int32_t Bytes, Cycles;
};

MOSCycleCostMode getMOSCycleCostModeFor(const MachineFunction &MF);

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSCYCLECOST_H
