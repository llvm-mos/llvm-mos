//===-- MOSAsmPrinter.cpp - MOS LLVM assembly writer ----------------------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a printer that converts from our internal representation
// of machine-dependent LLVM code to GAS-format MOS assembly language.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MOSInstPrinter.h"
#include "MOS.h"
#include "MOSMCInstLower.h"
#include "MOSSubtarget.h"

#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "mos-asm-printer"

namespace llvm {

/// An MOS assembly code printer.
class MOSAsmPrinter : public AsmPrinter {
public:
  MOSAsmPrinter(TargetMachine &TM, std::unique_ptr<MCStreamer> Streamer)
      : AsmPrinter(TM, std::move(Streamer)), MRI(*TM.getMCRegisterInfo()) {}

  StringRef getPassName() const override { return "MOS Assembly Printer"; }

private:
  const MCRegisterInfo &MRI;
};
} // end of namespace llvm

extern "C" void LLVM_EXTERNAL_VISIBILITY LLVMInitializeMOSAsmPrinter() {
  llvm::RegisterAsmPrinter<llvm::MOSAsmPrinter> X(llvm::getTheMOSTarget());
}
