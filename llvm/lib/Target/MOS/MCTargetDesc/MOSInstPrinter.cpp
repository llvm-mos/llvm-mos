//===-- MOSInstPrinter.cpp - Convert MOS MCInst to assembly syntax --------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This class prints an MOS MCInst to a .s file.
//
//===----------------------------------------------------------------------===//

#include "MOSInstPrinter.h"

#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOSMCExpr.h"

#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/raw_ostream.h"

#include <cstring>
#include <sstream>

#define DEBUG_TYPE "asm-printer"

namespace llvm {

void MOSInstPrinter::printInst(const MCInst *MI, uint64_t Address,
                               StringRef Annot, const MCSubtargetInfo &STI,
                               raw_ostream &OS) {
  std::string AiryOperands;
  raw_string_ostream AiryOperandStream(AiryOperands);
  assert(getMnemonic(MI).second && "Missing opcode for instruction.");
  printInstruction(MI, Address, AiryOperandStream);
  AiryOperands = AiryOperandStream.str();
  size_t SpacesSeen = 0;
  std::string CorrectOperands;
  for (const auto &Letter : AiryOperands) {
    if (isspace(Letter) != 0) {
      if (++SpacesSeen <= 2) {
        CorrectOperands += '\t';
      }
      continue;
    }
    CorrectOperands += Letter;
  }
  OS << CorrectOperands;
}

void MOSInstPrinter::printOperand(const MCInst *MI, unsigned OpNo,
                                  raw_ostream &O) {
  const MCOperand &Op = MI->getOperand(OpNo);

  if (Op.isReg()) {
    printRegName(O, Op.getReg());
  } else if (Op.isImm()) {
    O << formatImm(Op.getImm());
  } else {
    assert(Op.isExpr() && "Unknown operand kind in printOperand");
    // Format mos16 immediates using formatImm.
    if (const auto *MME = dyn_cast<MOSMCExpr>(Op.getExpr())) {
      int64_t Value = 0;
      if (MME->getKind() == MOSMCExpr::VK_MOS_IMM16 &&
          MME->getSubExpr()->evaluateAsAbsolute(Value)) {
        O << "mos16(" << formatImm(Value) << ')';
        return;
      }
    }
    Op.getExpr()->print(O, &MAI);
  }
}

void MOSInstPrinter::printBranchOperand(const MCInst *MI, uint64_t Address,
                                          unsigned OpNo,
                                          raw_ostream &O) {
  const MCOperand &Op = MI->getOperand(OpNo);
  if (!Op.isImm())
    return printOperand(MI, OpNo, O);
  uint64_t Target = Op.getImm();
  O << formatImm(PrintBranchImmAsAddress ? (int8_t)Target + Address + 2: Target);
}

void MOSInstPrinter::printRegName(raw_ostream &O, MCRegister Reg) const {
  O << getRegisterName(Reg);
}

format_object<int64_t> MOSInstPrinter::formatHex(int64_t Value) const {
  switch (PrintHexStyle) {
  case HexStyle::C:
    if (Value < 0) {
      return format("-$%" PRIx64, -Value);
    } else {
      return format("$%" PRIx64, Value);
    }
  case HexStyle::Asm:
    if (Value < 0) {
      return format("-$%" PRIx64, -Value);
    } else {
      return format("$%" PRIx64, Value);
    }
  }
  llvm_unreachable("unsupported print style");
}

format_object<uint64_t> MOSInstPrinter::formatHex(uint64_t Value) const {
  switch (PrintHexStyle) {
  case HexStyle::C:
    return format("$%" PRIx64, Value);
  case HexStyle::Asm:
    return format("$%" PRIx64, Value);
  }
  llvm_unreachable("unsupported print style");
}

// Include the auto-generated portion of the assembly writer.
#define PRINT_ALIAS_INSTR
#include "MOSGenAsmWriter.inc"

} // end of namespace llvm
