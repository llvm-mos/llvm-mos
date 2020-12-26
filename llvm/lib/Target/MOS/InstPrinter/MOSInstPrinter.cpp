//===-- MOSInstPrinter.cpp - Convert MOS MCInst to assembly syntax --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This class prints an MOS MCInst to a .s file.
//
//===----------------------------------------------------------------------===//

#include "MOSInstPrinter.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"

#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"

#include <cstring>

#define DEBUG_TYPE "asm-printer"

namespace llvm {

void MOSInstPrinter::printInst(const MCInst *MI, raw_ostream &OS,
                               StringRef Annot, const MCSubtargetInfo &STI) {
  printInstruction(MI, OS);
}

void MOSInstPrinter::printOperand(const MCInst *MI, unsigned OpNo,
                                  raw_ostream &O) {
  const MCOperand &Op = MI->getOperand(OpNo);
  // const MCOperandInfo &MOI = this->MII.get(MI->getOpcode()).OpInfo[OpNo];

/*
  if (Op.isReg()) {
      O << getRegisterName(Op.getReg(), MOS::ptr);
    } else {
      O << getPrettyRegisterName(Op.getReg(), MRI);
    }
  } else 
  */
  if (Op.isImm()) {
    O << Op.getImm();
  } else {
    assert(Op.isExpr() && "Unknown operand kind in printOperand");
    O << *Op.getExpr();
  }
}


// Include the auto-generated portion of the assembly writer.
#define PRINT_ALIAS_INSTR
#include "MOSGenAsmWriter.inc"




} // end of namespace llvm

