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
#include "MCTargetDesc/MOSMCExpr.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOSInstrInfo.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define DEBUG_TYPE "mos-mcinstlower"

void MOSMCInstLower::lower(const MachineInstr *MI, MCInst &OutMI) {
  switch (MI->getOpcode()) {
  default:
    OutMI.setOpcode(MI->getOpcode());
    break;
  case MOS::ASL:
    switch (MI->getOperand(0).getReg()) {
    default: {
      assert(MOS::ZPRegClass.contains(MI->getOperand(0).getReg()));
      OutMI.setOpcode(MOS::ASL_ZeroPage);
      MCOperand Addr;
      assert(lowerOperand(MI->getOperand(0), Addr));
      OutMI.addOperand(Addr);
      return;
    }
    case MOS::A:
      OutMI.setOpcode(MOS::ASL_Accumulator);
      return;
    }
  case MOS::LDimm:
    switch (MI->getOperand(0).getReg()) {
    default:
      llvm_unreachable("Unexpected LDimm destination.");
    case MOS::A:
      OutMI.setOpcode(MOS::LDA_Immediate);
      break;
    case MOS::X:
      OutMI.setOpcode(MOS::LDX_Immediate);
      break;
    case MOS::Y:
      OutMI.setOpcode(MOS::LDY_Immediate);
      break;
    }
    MCOperand Val;
    assert(lowerOperand(MI->getOperand(1), Val));
    OutMI.addOperand(Val);
    return;
  }

  for (const MachineOperand &MO : MI->operands()) {
    MCOperand MCOp;
    if (lowerOperand(MO, MCOp))
      OutMI.addOperand(MCOp);
  }
}

bool MOSMCInstLower::lowerOperand(const MachineOperand &MO, MCOperand &MCOp) {
  const MOSRegisterInfo &TRI =
      *MO.getParent()->getMF()->getSubtarget<MOSSubtarget>().getRegisterInfo();

  switch (MO.getType()) {
  default:
    LLVM_DEBUG(dbgs() << "Operand: " << MO << "\n");
    report_fatal_error("Operand type not implemented.");
  case MachineOperand::MO_RegisterMask:
    return false;
  case MachineOperand::MO_ExternalSymbol:
    MCOp = MCOperand::createExpr(MCSymbolRefExpr::create(
        Ctx.getOrCreateSymbol(MO.getSymbolName()), Ctx));
    break;
  case MachineOperand::MO_GlobalAddress: {
    const MCExpr *Expr =
        MCSymbolRefExpr::create(AP.getSymbol(MO.getGlobal()), Ctx);
    if (MO.getOffset() != 0)
      Expr = MCBinaryExpr::createAdd(
          Expr, MCConstantExpr::create(MO.getOffset(), Ctx), Ctx);
    Expr = applyTargetFlags(MO.getTargetFlags(), Expr);
    MCOp = MCOperand::createExpr(Expr);
    break;
  }
  case MachineOperand::MO_Immediate:
    MCOp = MCOperand::createImm(MO.getImm());
    break;
  case MachineOperand::MO_MachineBasicBlock:
    MCOp = MCOperand::createExpr(
        MCSymbolRefExpr::create(MO.getMBB()->getSymbol(), Ctx));
    break;
  case MachineOperand::MO_Register:
    // Ignore all implicit register operands.
    if (MO.isImplicit())
      return false;
    Register Reg = MO.getReg();
    if (MOS::ZP_PTRRegClass.contains(Reg) || MOS::ZPRegClass.contains(Reg))
      MCOp = MCOperand::createExpr(MCSymbolRefExpr::create(
          Ctx.getOrCreateSymbol(TRI.getZPSymbolName(Reg)), Ctx));
    else
      MCOp = MCOperand::createReg(MO.getReg());
    break;
  }
  return true;
}

const MCExpr *MOSMCInstLower::applyTargetFlags(unsigned Flags,
                                               const MCExpr *Expr) {
  switch (Flags) {
  default:
    llvm_unreachable("Invalid target operand flags.");
  case MOS::MO_NO_FLAGS:
    return Expr;
  case MOS::MO_LO:
    return MOSMCExpr::create(MOSMCExpr::VK_MOS_ADDR16_LO, Expr,
                             /*isNegated=*/false, Ctx);
  case MOS::MO_HI:
    return MOSMCExpr::create(MOSMCExpr::VK_MOS_ADDR16_HI, Expr,
                             /*isNegated=*/false, Ctx);
  }
}
