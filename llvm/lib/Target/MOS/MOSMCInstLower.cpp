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
  case MOS::ROL:
    switch (MI->getOperand(0).getReg()) {
    default: {
      assert(MOS::ZPRegClass.contains(MI->getOperand(0).getReg()));
      switch (MI->getOpcode()) {
      case MOS::ASL:
        OutMI.setOpcode(MOS::ASL_ZeroPage);
        break;
      case MOS::ROL:
        OutMI.setOpcode(MOS::ROL_ZeroPage);
        break;
      }
      MCOperand Addr;
      assert(lowerOperand(MI->getOperand(0), Addr));
      OutMI.addOperand(Addr);
      return;
    }
    case MOS::A:
      switch (MI->getOpcode()) {
      default:
        llvm_unreachable("Inconsistent opcode.");
      case MOS::ASL:
        OutMI.setOpcode(MOS::ASL_Accumulator);
        return;
      case MOS::ROL:
        OutMI.setOpcode(MOS::ROL_Accumulator);
        return;
      }
    }
  case MOS::BR: {
    switch (MI->getOperand(1).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MOS::C:
      OutMI.setOpcode(MI->getOperand(2).getImm() ? MOS::BCS_Relative
                                                 : MOS::BCC_Relative);
      break;
    case MOS::N:
      OutMI.setOpcode(MI->getOperand(2).getImm() ? MOS::BMI_Relative
                                                 : MOS::BPL_Relative);
      break;
    case MOS::Z:
      OutMI.setOpcode(MI->getOperand(2).getImm() ? MOS::BEQ_Relative
                                                 : MOS::BNE_Relative);
      break;
    }
    MCOperand Val;
    assert(lowerOperand(MI->getOperand(0), Val));
    OutMI.addOperand(Val);
    return;
  }
  case MOS::CMPimm:
  case MOS::LDimm:
  case MOS::LDabs:
  case MOS::LDzpr:
  case MOS::STabs: {
    switch (MI->getOperand(0).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MOS::A:
      switch (MI->getOpcode()) {
      case MOS::CMPimm:
        OutMI.setOpcode(MOS::CMP_Immediate);
        break;
      case MOS::LDimm:
        OutMI.setOpcode(MOS::LDA_Immediate);
        break;
      case MOS::LDabs:
        OutMI.setOpcode(MOS::LDA_Absolute);
        break;
      case MOS::LDzpr:
        OutMI.setOpcode(MOS::LDA_ZeroPage);
        break;
      case MOS::STabs:
        OutMI.setOpcode(MOS::STA_Absolute);
        break;
      }
      break;
    case MOS::X:
      switch (MI->getOpcode()) {
      case MOS::CMPimm:
        OutMI.setOpcode(MOS::CPX_Immediate);
        break;
      case MOS::LDimm:
        OutMI.setOpcode(MOS::LDX_Immediate);
        break;
      case MOS::LDabs:
        OutMI.setOpcode(MOS::LDX_Absolute);
        break;
      case MOS::LDzpr:
        OutMI.setOpcode(MOS::LDX_ZeroPage);
        break;
      case MOS::STabs:
        OutMI.setOpcode(MOS::STX_Absolute);
        break;
      }
      break;
    case MOS::Y:
      switch (MI->getOpcode()) {
      case MOS::CMPimm:
        OutMI.setOpcode(MOS::CPY_Immediate);
        break;
      case MOS::LDimm:
        OutMI.setOpcode(MOS::LDY_Immediate);
        break;
      case MOS::LDabs:
        OutMI.setOpcode(MOS::LDY_Absolute);
        break;
      case MOS::LDzpr:
        OutMI.setOpcode(MOS::LDY_ZeroPage);
        break;
      case MOS::STabs:
        OutMI.setOpcode(MOS::STY_Absolute);
        break;
      }
      break;
    }
    MCOperand Val;
    assert(lowerOperand(MI->getOperand(1), Val));
    OutMI.addOperand(Val);
    return;
  }
  case MOS::LDAidx: {
    switch (MI->getOperand(1).getReg()) {
    default:
      llvm_unreachable("Unexpected LDAidx register.");
    case MOS::X:
      OutMI.setOpcode(MOS::LDA_AbsoluteX);
      break;
    case MOS::Y:
      OutMI.setOpcode(MOS::LDA_AbsoluteY);
      break;
    }
    MCOperand Val;
    assert(lowerOperand(MI->getOperand(0), Val));
    OutMI.addOperand(Val);
    return;
  }
  case MOS::LDCimm: {
    switch (MI->getOperand(1).getImm()) {
    default:
      llvm_unreachable("Unexpected LDCimm immediate.");
    case 0:
      OutMI.setOpcode(MOS::CLC_Implied);
      return;
    case 1:
      OutMI.setOpcode(MOS::SEC_Implied);
      return;
    }
  }
  case MOS::IN_:
  case MOS::TA_:
  case MOS::T_A:
    switch (MI->getOperand(0).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MOS::X:
      switch (MI->getOpcode()) {
      default:
        llvm_unreachable("Inconsistent opcode.");
      case MOS::IN_:
        OutMI.setOpcode(MOS::INX_Implied);
        return;
      case MOS::TA_:
        OutMI.setOpcode(MOS::TAX_Implied);
        return;
      case MOS::T_A:
        OutMI.setOpcode(MOS::TXA_Implied);
        return;
      }
    case MOS::Y:
      switch (MI->getOpcode()) {
      default:
        llvm_unreachable("Inconsistent opcode.");
      case MOS::IN_:
        OutMI.setOpcode(MOS::INY_Implied);
        return;
      case MOS::TA_:
        OutMI.setOpcode(MOS::TAY_Implied);
        return;
      case MOS::T_A:
        OutMI.setOpcode(MOS::TYA_Implied);
        return;
      }
    }
  case MOS::STidx: {
    switch (MI->getOperand(2).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MOS::X:
      OutMI.setOpcode(MOS::STA_AbsoluteX);
      break;
    case MOS::Y:
      OutMI.setOpcode(MOS::STA_AbsoluteY);
      break;
    }
    MCOperand Val;
    assert(lowerOperand(MI->getOperand(1), Val));
    OutMI.addOperand(Val);
    return;
  }
  case MOS::STzpr: {
    switch (MI->getOperand(1).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MOS::A:
      OutMI.setOpcode(MOS::STA_ZeroPage);
      break;
    case MOS::X:
      OutMI.setOpcode(MOS::STX_ZeroPage);
      break;
    case MOS::Y:
      OutMI.setOpcode(MOS::STY_ZeroPage);
      break;
    }
    MCOperand Val;
    assert(lowerOperand(MI->getOperand(0), Val));
    OutMI.addOperand(Val);
    return;
  }
  }

  // Handle any real instructions that weren't generated from a pseudo.
  assert(!MI->isPseudo());
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
