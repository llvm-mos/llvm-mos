//===-- MOSMCInstLower.cpp - Convert MOS MachineInstr to an MCInst --------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower MOS MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//

#include "MOSMCInstLower.h"

#include "MOSInstrInfo.h"
#include "MCTargetDesc/MOSMCExpr.h"

#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

} // end of namespace llvm

