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

MCOperand MOSMCInstLower::lowerSymbolOperand(const MachineOperand &MO,
                                             MCSymbol *Sym) const {
  unsigned char TF = MO.getTargetFlags();
  const MCExpr *Expr = MCSymbolRefExpr::create(Sym, Ctx);

  bool IsNegated = false;
  if (TF & MOSII::MO_NEG) { IsNegated = true; }

  if (!MO.isJTI() && MO.getOffset()) {
    Expr = MCBinaryExpr::createAdd(
        Expr, MCConstantExpr::create(MO.getOffset(), Ctx), Ctx);
  }

  bool IsFunction = MO.isGlobal() && isa<Function>(MO.getGlobal());

  if (TF & MOSII::MO_LO) {
    if (IsFunction) {
      // N.B. Should we use _GS fixups here to cope with >128k progmem?
      Expr = MOSMCExpr::create(MOSMCExpr::VK_MOS_PM_LO8, Expr, IsNegated, Ctx);
    } else {
      Expr = MOSMCExpr::create(MOSMCExpr::VK_MOS_LO8, Expr, IsNegated, Ctx);
    }
  } else if (TF & MOSII::MO_HI) {
    if (IsFunction) {
      // N.B. Should we use _GS fixups here to cope with >128k progmem?
      Expr = MOSMCExpr::create(MOSMCExpr::VK_MOS_PM_HI8, Expr, IsNegated, Ctx);
    } else {
      Expr = MOSMCExpr::create(MOSMCExpr::VK_MOS_HI8, Expr, IsNegated, Ctx);
    }
  } else if (TF != 0) {
    llvm_unreachable("Unknown target flag on symbol operand");
  }

  return MCOperand::createExpr(Expr);
}

void MOSMCInstLower::lowerInstruction(const MachineInstr &MI, MCInst &OutMI) const {
  OutMI.setOpcode(MI.getOpcode());

  for (MachineOperand const &MO : MI.operands()) {
    MCOperand MCOp;

    switch (MO.getType()) {
    default:
      MI.print(errs());
      llvm_unreachable("unknown operand type");
    case MachineOperand::MO_Register:
      // Ignore all implicit register operands.
      if (MO.isImplicit())
        continue;
      MCOp = MCOperand::createReg(MO.getReg());
      break;
    case MachineOperand::MO_Immediate:
      MCOp = MCOperand::createImm(MO.getImm());
      break;
    case MachineOperand::MO_GlobalAddress:
      MCOp = lowerSymbolOperand(MO, Printer.getSymbol(MO.getGlobal()));
      break;
    case MachineOperand::MO_ExternalSymbol:
      MCOp = lowerSymbolOperand(
          MO, Printer.GetExternalSymbolSymbol(MO.getSymbolName()));
      break;
    case MachineOperand::MO_MachineBasicBlock:
      MCOp = MCOperand::createExpr(
          MCSymbolRefExpr::create(MO.getMBB()->getSymbol(), Ctx));
      break;
    case MachineOperand::MO_RegisterMask:
      continue;
    case MachineOperand::MO_BlockAddress:
      MCOp = lowerSymbolOperand(
          MO, Printer.GetBlockAddressSymbol(MO.getBlockAddress()));
      break;
    case MachineOperand::MO_JumpTableIndex:
      MCOp = lowerSymbolOperand(MO, Printer.GetJTISymbol(MO.getIndex()));
      break;
    case MachineOperand::MO_ConstantPoolIndex:
      MCOp = lowerSymbolOperand(MO, Printer.GetCPISymbol(MO.getIndex()));
      break;
    }

    OutMI.addOperand(MCOp);
  }
}

} // end of namespace llvm

