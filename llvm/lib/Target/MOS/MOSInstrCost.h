//===-- MOSInstrCost.h - MOS Instruction Cost structure ---------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the definition of the MOSInstrCost class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSINSTRCOST_H
#define LLVM_LIB_TARGET_MOS_MOSINSTRCOST_H

#include "llvm/CodeGen/MachineFunction.h"
#include <cstdint>

namespace llvm {

class MOSInstrCost {
public:
  enum class Mode {
    PreferBytes,
    PreferCycles,
    Average
  };

  MOSInstrCost() : Bytes(0), Cycles(0) {}

  MOSInstrCost(int32_t Bytes, int32_t Cycles)
    : MOSInstrCost(Bytes, Cycles, 256) {}

  friend MOSInstrCost operator+(MOSInstrCost Left,
                                const MOSInstrCost& Right) {
    return MOSInstrCost(Left.Bytes + Right.Bytes,
                        Left.Cycles + Right.Cycles, 1);
  }

  MOSInstrCost& operator+=(const MOSInstrCost& Right) {
    this->Bytes += Right.Bytes;
    this->Cycles += Right.Cycles;
    return *this;
  }

  friend MOSInstrCost operator-(MOSInstrCost Left,
                                const MOSInstrCost& Right) {
    return MOSInstrCost(Left.Bytes - Right.Bytes,
                        Left.Cycles - Right.Cycles, 1);
  }

  MOSInstrCost& operator-=(const MOSInstrCost& Right) {
    this->Bytes -= Right.Bytes;
    this->Cycles -= Right.Cycles;
    return *this;
  }

  friend MOSInstrCost operator*(MOSInstrCost Left, int Right) {
    return MOSInstrCost(Left.Bytes * Right, Left.Cycles * Right, 1);
  }

  friend MOSInstrCost operator/(MOSInstrCost Left, int Right) {
    return MOSInstrCost(Left.Bytes / Right, Left.Cycles / Right, 1);
  }

  int64_t value(Mode Mode = Mode::Average) const;

  static Mode getModeFor(const MachineFunction &MF);

private:
  MOSInstrCost(int32_t Bytes, int32_t Cycles, int Multiplier)
    : Bytes(Bytes * Multiplier), Cycles(Cycles * Multiplier) {}

  int32_t Bytes, Cycles;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MOS_MOSCYCLECOST_H
