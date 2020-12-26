//===-- MOSMCInstLower.cpp - Convert MOS MachineInstr to an MCInst --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
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

