//===-- MOSMCInstLower.h - Lower MachineInstr to MCInst ---------*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MOS_MCINST_LOWER_H
#define LLVM_MOS_MCINST_LOWER_H

#include "llvm/Support/Compiler.h"

namespace llvm {

class AsmPrinter;
class MachineInstr;
class MachineOperand;
class MCContext;
class MCInst;
class MCOperand;
class MCSymbol;

/// Lowers `MachineInstr` objects into `MCInst` objects.
class MOSMCInstLower {
public:
  MOSMCInstLower(MCContext &, AsmPrinter &) {}
};

} // end namespace llvm

#endif // LLVM_MOS_MCINST_LOWER_H
