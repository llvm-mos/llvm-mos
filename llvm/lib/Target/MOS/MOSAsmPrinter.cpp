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

#include "MOSMCInstLower.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"
#include "TargetInfo/MOSTargetInfo.h"
#include "llvm/ADT/StringSet.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Module.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "asm-printer"

namespace {

class MOSAsmPrinter : public AsmPrinter {
  MOSMCInstLower InstLowering;

public:
  explicit MOSAsmPrinter(TargetMachine &TM,
                             std::unique_ptr<MCStreamer> Streamer)
      : AsmPrinter(TM, std::move(Streamer)), InstLowering(OutContext, *this) {}

  void emitInstruction(const MachineInstr *MI) override;
};

void MOSAsmPrinter::emitInstruction(const MachineInstr *MI) {
  MCInst Inst;
  InstLowering.lower(MI, Inst);
  EmitToStreamer(*OutStreamer, Inst);
}

} // namespace

// Force static initialization.
extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeMOSAsmPrinter() {
  RegisterAsmPrinter<MOSAsmPrinter> X(getTheMOSTarget());
}
