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
#include "MCTargetDesc/MOSAsmBackend.h"
#include "MCTargetDesc/MOSMCExpr.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"
#include "MOSInstrInfo.h"
#include "MOSMachineFunctionInfo.h"
#include "MOSRegisterInfo.h"
#include "MOSSubtarget.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define DEBUG_TYPE "mos-mcinstlower"

static bool canUseZeroPageIdx(const MachineOperand &MO) {
  // Constants may extend past the zero page when added to the index, so they
  // cannot generally use zero page indexed addressing.
  if (!MO.isGlobal())
    return false;

  // Global values in the zero page must end before the zero page, which means
  // that pointers based on them can never overflow it.
  return MO.getGlobal()->getAliaseeObject()->getAddressSpace() == 1;
}

// Instructions with indexed addressing must have bases < 256 wrapped in mos16.
// Otherwise, if the assembly were later parsed, the zero page indexed
// addressing mode might be selected, which has different semantics when Base +
// Idx >= 256;
static MCOperand wrapAbsoluteIdxBase(MCOperand Op, MCContext &Ctx) {
  if (!Op.isImm() || Op.getImm() >= 256)
    return Op;

  // Using IMM16 here is a hack, but it seems to work. This really should be
  // ADDR16, but that doesn't exist, and it's unclear whether that would
  // conflict with IMM16, which is currently assigned to mos16().
  return MCOperand::createExpr(MOSMCExpr::create(
      MOSMCExpr::VK_MOS_IMM16, MCConstantExpr::create(Op.getImm(), Ctx),
      /*isNegated=*/false, Ctx));
}

void MOSMCInstLower::lower(const MachineInstr *MI, MCInst &OutMI) {
  switch (MI->getOpcode()) {
  default:
    OutMI.setOpcode(MI->getOpcode());
    break;
  case MOS::ADCAbsIdx:
  case MOS::SBCAbsIdx: {
    bool ZP = canUseZeroPageIdx(MI->getOperand(4));
    bool ImmConfusable = false;
    switch (MI->getOpcode()) {
    case MOS::ADCAbsIdx:
      switch (MI->getOperand(5).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MOS::X:
        OutMI.setOpcode(ZP ? MOS::ADC_ZeroPageX : MOS::ADC_AbsoluteX);
        ImmConfusable = !ZP;
        break;
      case MOS::Y:
        OutMI.setOpcode(MOS::ADC_AbsoluteY);
        break;
      }
      break;
    case MOS::SBCAbsIdx:
      switch (MI->getOperand(5).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MOS::X:
        OutMI.setOpcode(ZP ? MOS::SBC_ZeroPageX : MOS::SBC_AbsoluteX);
        ImmConfusable = !ZP;
        break;
      case MOS::Y:
        OutMI.setOpcode(MOS::SBC_AbsoluteY);
        break;
      }
      break;
    }
    MCOperand Addr;
    if (!lowerOperand(MI->getOperand(4), Addr))
      llvm_unreachable("Failed to lower operand");
    if (ImmConfusable)
      Addr = wrapAbsoluteIdxBase(Addr, Ctx);
    OutMI.addOperand(Addr);
    return;
  }
  case MOS::ANDAbsIdx:
  case MOS::EORAbsIdx:
  case MOS::ORAAbsIdx: {
    bool ZP = canUseZeroPageIdx(MI->getOperand(2));
    bool ImmConfusable = false;
    switch (MI->getOpcode()) {
    case MOS::ANDAbsIdx:
      switch (MI->getOperand(3).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MOS::X:
        OutMI.setOpcode(ZP ? MOS::AND_ZeroPageX : MOS::AND_AbsoluteX);
        break;
      case MOS::Y:
        OutMI.setOpcode(MOS::AND_AbsoluteY);
        break;
      }
      break;
    case MOS::EORAbsIdx:
      switch (MI->getOperand(3).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MOS::X:
        OutMI.setOpcode(ZP ? MOS::EOR_ZeroPageX : MOS::EOR_AbsoluteX);
        ImmConfusable = !ZP;
        break;
      case MOS::Y:
        OutMI.setOpcode(MOS::EOR_AbsoluteY);
        break;
      }
      break;
    case MOS::ORAAbsIdx:
      switch (MI->getOperand(3).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MOS::X:
        OutMI.setOpcode(ZP ? MOS::ORA_ZeroPageX : MOS::ORA_AbsoluteX);
        ImmConfusable = !ZP;
        break;
      case MOS::Y:
        OutMI.setOpcode(MOS::ORA_AbsoluteY);
        break;
      }
      break;
    }
    MCOperand Addr;
    if (!lowerOperand(MI->getOperand(2), Addr))
      llvm_unreachable("Failed to lower operand");
    if (ImmConfusable)
      Addr = wrapAbsoluteIdxBase(Addr, Ctx);
    OutMI.addOperand(Addr);
    return;
  }
  case MOS::ASL:
  case MOS::LSR:
  case MOS::ROL:
  case MOS::ROR:
    switch (MI->getOperand(0).getReg()) {
    default: {
      assert(MOS::Imag8RegClass.contains(MI->getOperand(0).getReg()));
      switch (MI->getOpcode()) {
      case MOS::ASL:
        OutMI.setOpcode(MOS::ASL_ZeroPage);
        break;
      case MOS::LSR:
        OutMI.setOpcode(MOS::LSR_ZeroPage);
        break;
      case MOS::ROL:
        OutMI.setOpcode(MOS::ROL_ZeroPage);
        break;
      case MOS::ROR:
        OutMI.setOpcode(MOS::ROR_ZeroPage);
        break;
      }
      MCOperand Addr;
      if (!lowerOperand(MI->getOperand(0), Addr))
        llvm_unreachable("Failed to lower operand");
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
      case MOS::LSR:
        OutMI.setOpcode(MOS::LSR_Accumulator);
        return;
      case MOS::ROL:
        OutMI.setOpcode(MOS::ROL_Accumulator);
        return;
      case MOS::ROR:
        OutMI.setOpcode(MOS::ROR_Accumulator);
        return;
      }
    }
  case MOS::ASLAbsIdx:
  case MOS::LSRAbsIdx:
  case MOS::ROLAbsIdx:
  case MOS::RORAbsIdx: {
    bool ZP = canUseZeroPageIdx(MI->getOperand(1));
    switch (MI->getOpcode()) {
    case MOS::ASLAbsIdx:
      OutMI.setOpcode(ZP ? MOS::ASL_ZeroPageX : MOS::ASL_AbsoluteX);
      break;
    case MOS::LSRAbsIdx:
      OutMI.setOpcode(ZP ? MOS::LSR_ZeroPageX : MOS::LSR_AbsoluteX);
      break;
    case MOS::ROLAbsIdx:
      OutMI.setOpcode(ZP ? MOS::ROL_ZeroPageX : MOS::ROL_AbsoluteX);
      break;
    case MOS::RORAbsIdx:
      OutMI.setOpcode(ZP ? MOS::ROR_ZeroPageX : MOS::ROR_AbsoluteX);
      break;
    }
    MCOperand Tgt;
    if (!lowerOperand(MI->getOperand(1), Tgt))
      llvm_unreachable("Failed to lower operand");
    if (!ZP)
      Tgt = wrapAbsoluteIdxBase(Tgt, Ctx);
    OutMI.addOperand(Tgt);
    return;
  }
  case MOS::BR: {
    Register Flag = MI->getOperand(1).getReg();
    int64_t Val = MI->getOperand(2).getImm();
    switch (Flag) {
    default:
      llvm_unreachable("Unexpected register.");
    case MOS::C:
      OutMI.setOpcode(Val ? MOS::BCS_Relative : MOS::BCC_Relative);
      break;
    case MOS::N:
      OutMI.setOpcode(Val ? MOS::BMI_Relative : MOS::BPL_Relative);
      break;
    case MOS::V:
      OutMI.setOpcode(Val ? MOS::BVS_Relative : MOS::BVC_Relative);
      break;
    case MOS::Z:
      OutMI.setOpcode(Val ? MOS::BEQ_Relative : MOS::BNE_Relative);
      break;
    }
    MCOperand Tgt;
    if (!lowerOperand(MI->getOperand(0), Tgt))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Tgt);
    return;
  }
  case MOS::CMPImm:
  case MOS::CMPImag8:
  case MOS::CMPAbs:
  case MOS::CMPAbsIdx: {
    bool ImmConfusable = false;
    switch (MI->getOpcode()) {
    case MOS::CMPImm:
      switch (MI->getOperand(1).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MOS::A:
        OutMI.setOpcode(MOS::CMP_Immediate);
        break;
      case MOS::X:
        OutMI.setOpcode(MOS::CPX_Immediate);
        break;
      case MOS::Y:
        OutMI.setOpcode(MOS::CPY_Immediate);
        break;
      }
      break;
    case MOS::CMPImag8:
    case MOS::CMPAbs:
      switch (MI->getOperand(1).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MOS::A:
        OutMI.setOpcode(MOS::CMP_ZeroPage);
        break;
      case MOS::X:
        OutMI.setOpcode(MOS::CPX_ZeroPage);
        break;
      case MOS::Y:
        OutMI.setOpcode(MOS::CPY_ZeroPage);
        break;
      }
      break;
    case MOS::CMPAbsIdx: {
      bool ZP = canUseZeroPageIdx(MI->getOperand(2));
      switch (MI->getOperand(3).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MOS::X:
        OutMI.setOpcode(ZP ? MOS::CMP_ZeroPageX : MOS::CMP_AbsoluteX);
        ImmConfusable = !ZP;
        break;
      case MOS::Y:
        OutMI.setOpcode(MOS::CMP_AbsoluteY);
        break;
      }
      break;
    }
    }
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(2), Val))
      llvm_unreachable("Failed to lower operand");
    if (ImmConfusable)
      Val = wrapAbsoluteIdxBase(Val, Ctx);
    OutMI.addOperand(Val);
    return;
  }
  case MOS::INCAbsIdx:
  case MOS::DECAbsIdx: {
    bool ZP = canUseZeroPageIdx(MI->getOperand(0));
    switch (MI->getOpcode()) {
    case MOS::INCAbsIdx:
      OutMI.setOpcode(ZP ? MOS::INC_ZeroPageX : MOS::INC_AbsoluteX);
      break;
    case MOS::DECAbsIdx:
      OutMI.setOpcode(ZP ? MOS::DEC_ZeroPageX : MOS::DEC_AbsoluteX);
    }
    MCOperand Tgt;
    if (!lowerOperand(MI->getOperand(0), Tgt))
      llvm_unreachable("Failed to lower operand");
    if (!ZP)
      Tgt = wrapAbsoluteIdxBase(Tgt, Ctx);
    OutMI.addOperand(Tgt);
    return;
  }
  case MOS::LDImm:
  case MOS::LDAbs:
  case MOS::LDImag8:
  case MOS::STAbs: {
    switch (MI->getOperand(0).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MOS::A:
      switch (MI->getOpcode()) {
      case MOS::LDImm:
        OutMI.setOpcode(MOS::LDA_Immediate);
        break;
      case MOS::LDAbs:
      case MOS::LDImag8:
        OutMI.setOpcode(MOS::LDA_ZeroPage);
        break;
      case MOS::STAbs:
        OutMI.setOpcode(MOS::STA_ZeroPage);
        break;
      }
      break;
    case MOS::X:
      switch (MI->getOpcode()) {
      case MOS::LDImm:
        OutMI.setOpcode(MOS::LDX_Immediate);
        break;
      case MOS::LDAbs:
      case MOS::LDImag8:
        OutMI.setOpcode(MOS::LDX_ZeroPage);
        break;
      case MOS::STAbs:
        OutMI.setOpcode(MOS::STX_ZeroPage);
        break;
      }
      break;
    case MOS::Y:
      switch (MI->getOpcode()) {
      case MOS::LDImm:
        OutMI.setOpcode(MOS::LDY_Immediate);
        break;
      case MOS::LDAbs:
      case MOS::LDImag8:
        OutMI.setOpcode(MOS::LDY_ZeroPage);
        break;
      case MOS::STAbs:
        OutMI.setOpcode(MOS::STY_ZeroPage);
        break;
      }
      break;
    }
    int64_t ImmIdx = MI->getOpcode() == MOS::CMPImm ? 2 : 1;
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(ImmIdx), Val))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Val);
    return;
  }
  case MOS::LDAAbsIdx: {
    bool ZP = canUseZeroPageIdx(MI->getOperand(1));
    bool ImmConfusable = false;
    switch (MI->getOperand(2).getReg()) {
    default:
      llvm_unreachable("Unexpected LDAAbsIdx register.");
    case MOS::X:
      OutMI.setOpcode(ZP ? MOS::LDA_ZeroPageX : MOS::LDA_AbsoluteX);
      ImmConfusable = !ZP;
      break;
    case MOS::Y:
      OutMI.setOpcode(MOS::LDA_AbsoluteY);
      break;
    }
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(1), Val))
      llvm_unreachable("Failed to lower operand");
    if (ImmConfusable)
      Val = wrapAbsoluteIdxBase(Val, Ctx);
    OutMI.addOperand(Val);
    return;
  }
  case MOS::LDXAbsIdx:
  case MOS::LDYAbsIdx: {
    bool ZP = canUseZeroPageIdx(MI->getOperand(1));
    switch (MI->getOpcode()) {
    default:
      llvm_unreachable("Unexpected LDAbsIdx register.");
    case MOS::LDXAbsIdx:
      OutMI.setOpcode(ZP ? MOS::LDX_ZeroPageY : MOS::LDX_AbsoluteY);
      break;
    case MOS::LDYAbsIdx:
      OutMI.setOpcode(ZP ? MOS::LDY_ZeroPageX : MOS::LDY_AbsoluteX);
      break;
    }
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(1), Val))
      llvm_unreachable("Failed to lower operand");
    if (!ZP)
      Val = wrapAbsoluteIdxBase(Val, Ctx);
    OutMI.addOperand(Val);
    return;
  }
  case MOS::LDCImm: {
    switch (MI->getOperand(1).getImm()) {
    default:
      llvm_unreachable("Unexpected LDCImm immediate.");
    case 0:
      OutMI.setOpcode(MOS::CLC_Implied);
      return;
    case -1:
      OutMI.setOpcode(MOS::SEC_Implied);
      return;
    }
  }
  case MOS::DE:
  case MOS::IN:
  case MOS::TA:
    switch (MI->getOperand(0).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MOS::X:
      switch (MI->getOpcode()) {
      default:
        llvm_unreachable("Inconsistent opcode.");
      case MOS::DE:
        OutMI.setOpcode(MOS::DEX_Implied);
        return;
      case MOS::IN:
        OutMI.setOpcode(MOS::INX_Implied);
        return;
      case MOS::TA:
        OutMI.setOpcode(MOS::TAX_Implied);
        return;
      }
    case MOS::Y:
      switch (MI->getOpcode()) {
      default:
        llvm_unreachable("Inconsistent opcode.");
      case MOS::DE:
        OutMI.setOpcode(MOS::DEY_Implied);
        return;
      case MOS::IN:
        OutMI.setOpcode(MOS::INY_Implied);
        return;
      case MOS::TA:
        OutMI.setOpcode(MOS::TAY_Implied);
        return;
      }
    }
  case MOS::PH:
  case MOS::PL: {
    switch (MI->getOperand(0).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MOS::A:
      OutMI.setOpcode(MI->getOpcode() == MOS::PH ? MOS::PHA_Implied
                                                 : MOS::PLA_Implied);
      return;
    case MOS::X:
      OutMI.setOpcode(MI->getOpcode() == MOS::PH ? MOS::PHX_Implied
                                                 : MOS::PLX_Implied);
      return;
    case MOS::Y:
      OutMI.setOpcode(MI->getOpcode() == MOS::PH ? MOS::PHY_Implied
                                                 : MOS::PLY_Implied);
      return;
    case MOS::P:
      OutMI.setOpcode(MI->getOpcode() == MOS::PH ? MOS::PHP_Implied
                                                 : MOS::PLP_Implied);
      return;
    }
  }
  case MOS::STAbsIdx: {
    bool ZP = canUseZeroPageIdx(MI->getOperand(0));
    switch (MI->getOperand(2).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MOS::X:
      OutMI.setOpcode(ZP ? MOS::STA_ZeroPageX : MOS::STA_AbsoluteX);
      break;
    case MOS::Y:
      OutMI.setOpcode(MOS::STA_AbsoluteY);
      break;
    }
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(1), Val))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Val);
    return;
  }
  case MOS::STImag8: {
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
    if (!lowerOperand(MI->getOperand(0), Val))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Val);
    return;
  }
  case MOS::STZAbsIdx: {
    bool ZP = canUseZeroPageIdx(MI->getOperand(0));
    OutMI.setOpcode(ZP ? MOS::STZ_ZeroPageX : MOS::STZ_AbsoluteX);
    MCOperand Tgt;
    if (!lowerOperand(MI->getOperand(0), Tgt))
      llvm_unreachable("Failed to lower operand");
    if (!ZP)
      Tgt = wrapAbsoluteIdxBase(Tgt, Ctx);
    OutMI.addOperand(Tgt);
    return;
  }
  case MOS::T_A:
    switch (MI->getOperand(1).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MOS::X:
      OutMI.setOpcode(MOS::TXA_Implied);
      return;
    case MOS::Y:
      OutMI.setOpcode(MOS::TYA_Implied);
      return;
    }
  }

  // Handle any real instructions that weren't generated from a pseudo.
#ifndef NDEBUG
  if (MI->isPseudo()) {
    dbgs() << *MI;
    llvm_unreachable("Pseudoinstruction was never lowered.");
  }
#endif
  for (const MachineOperand &MO : MI->operands()) {
    MCOperand MCOp;
    if (lowerOperand(MO, MCOp))
      OutMI.addOperand(MCOp);
  }
}

bool MOSMCInstLower::lowerOperand(const MachineOperand &MO, MCOperand &MCOp) {
  const auto &FuncInfo = *MO.getParent()->getMF()->getInfo<MOSFunctionInfo>();
  const MOSRegisterInfo &TRI =
      *MO.getParent()->getMF()->getSubtarget<MOSSubtarget>().getRegisterInfo();

  switch (MO.getType()) {
  default:
    LLVM_DEBUG(dbgs() << "Operand: " << MO << "\n");
    report_fatal_error("Operand type not implemented.");
  case MachineOperand::MO_RegisterMask:
    return false;
  case MachineOperand::MO_BlockAddress:
    MCOp =
        lowerSymbolOperand(MO, AP.GetBlockAddressSymbol(MO.getBlockAddress()));
    break;
  case MachineOperand::MO_ExternalSymbol:
    MCOp =
        lowerSymbolOperand(MO, AP.GetExternalSymbolSymbol(MO.getSymbolName()));
    break;
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MO.getGlobal();
    MCOp = lowerSymbolOperand(MO, AP.getSymbol(GV));

    // Don't add addr8 to expressions that have already been given a fixup type.
    if (auto *E = dyn_cast<MOSMCExpr>(MCOp.getExpr()))
      if (E->getKind() != MOSMCExpr::VK_MOS_NONE)
        break;

    // This is the last chance to catch values that are attributed a zero-page
    // section. It is the user's responsibility to ensure the linker will
    // locate the symbol completely within the zero-page.
    const auto *GVar = dyn_cast<GlobalVariable>(GV->getAliaseeObject());
    if (MOSAsmBackend::isZeroPageSectionName(GV->getSection()) ||
        (GVar && GVar->getAddressSpace() == 1)) {
      const MOSMCExpr *Expr =
          MOSMCExpr::create(MOSMCExpr::VK_MOS_ADDR8, MCOp.getExpr(),
                            /*isNegated=*/false, Ctx);
      MCOp = MCOperand::createExpr(Expr);
    }
    break;
  }
  case MachineOperand::MO_JumpTableIndex: {
    MCOp = lowerSymbolOperand(MO, AP.GetJTISymbol(MO.getIndex()));
    break;
  }
  case MachineOperand::MO_Immediate: {
    auto GetTotal = [&]() {
      size_t Idx = &MO - MO.getParent()->operands_begin();
      switch (MO.getParent()->getDesc().OpInfo[Idx].OperandType) {
      default:
        llvm_unreachable("Unexpected operand type.");
      case MOSOp::OPERAND_IMM8:
      case MOSOp::OPERAND_ADDR8:
        return 256;
        break;
      case MOSOp::OPERAND_ADDR16:
        return 65536;
        break;
      }
    };
    MCOp = MCOperand::createImm(MO.getImm() >= 0 ? MO.getImm()
                                                 : MO.getImm() + GetTotal());
    break;
  }
  case MachineOperand::MO_MachineBasicBlock:
    MCOp = MCOperand::createExpr(
        MCSymbolRefExpr::create(MO.getMBB()->getSymbol(), Ctx));
    break;
  case MachineOperand::MO_Register:
    // Ignore all implicit register operands.
    if (MO.isImplicit())
      return false;
    Register Reg = MO.getReg();

    // Some CSRs may have been "spilled" by silently renaming them to zero page
    // locations on the zero page stack. We want to maintain the illusion that
    // these are imaginary registers, so they are rewritten as late as possible.
    auto It = FuncInfo.CSRZPOffsets.find(Reg);
    if (It != FuncInfo.CSRZPOffsets.end()) {
      const MCExpr *Expr = MCSymbolRefExpr::create(
          AP.getSymbol(FuncInfo.ZeroPageStackValue), Ctx);
      size_t Offset = It->second;
      if (Offset)
        Expr = MCBinaryExpr::createAdd(
            Expr, MCConstantExpr::create(Offset, Ctx), Ctx);
      Expr = MOSMCExpr::create(MOSMCExpr::VK_MOS_ADDR8, Expr,
                               /*isNegated=*/false, Ctx);
      MCOp = MCOperand::createExpr(Expr);
      break;
    }

    if (MOS::Imag16RegClass.contains(Reg) || MOS::Imag8RegClass.contains(Reg)) {
      const MCExpr *Expr = MCSymbolRefExpr::create(
          Ctx.getOrCreateSymbol(TRI.getImag8SymbolName(Reg)), Ctx);
      Expr = MOSMCExpr::create(MOSMCExpr::VK_MOS_ADDR8, Expr,
                               /*isNegated=*/false, Ctx);
      MCOp = MCOperand::createExpr(Expr);
    } else
      MCOp = MCOperand::createReg(MO.getReg());
    break;
  }
  return true;
}

MCOperand MOSMCInstLower::lowerSymbolOperand(const MachineOperand &MO,
                                             const MCSymbol *Sym) {
  const MachineFrameInfo &MFI = MO.getParent()->getMF()->getFrameInfo();
  bool ZP;
  if (MO.isFI()) {
    ZP = MFI.getStackID(MO.getIndex()) == TargetStackID::MosZeroPage;
  } else if (MO.isGlobal()) {
    const auto *GV =
        dyn_cast<GlobalVariable>(MO.getGlobal()->getAliaseeObject());
    ZP = GV && GV->getAddressSpace() == 1;
  } else {
    ZP = false;
  }

  const MCExpr *Expr = MCSymbolRefExpr::create(Sym, Ctx);
  if (!MO.isJTI() && MO.getOffset() != 0)
    Expr = MCBinaryExpr::createAdd(
        Expr, MCConstantExpr::create(MO.getOffset(), Ctx), Ctx);
  switch (MO.getTargetFlags()) {
  default:
    llvm_unreachable("Invalid target operand flags.");
  case MOS::MO_NO_FLAGS:
    break;
  case MOS::MO_LO:
    if (!ZP) {
      Expr = MOSMCExpr::create(MOSMCExpr::VK_MOS_ADDR16_LO, Expr,
                               /*isNegated=*/false, Ctx);
    }
    break;
  case MOS::MO_HI:
    if (ZP) {
      Expr = MCConstantExpr::create(0, Ctx);
    } else {
      Expr = MOSMCExpr::create(MOSMCExpr::VK_MOS_ADDR16_HI, Expr,
                               /*isNegated=*/false, Ctx);
    }
    break;
  case MOS::MO_HI_JT: {
    // Jump tables are partitioned in two arrays: first all the low bytes,
    // then all the high bytes. This index referes to the high byte array, so
    // offset the appropriate amount into the overall array.
    assert(MO.isJTI());
    const MachineJumpTableInfo *JTI =
        MO.getParent()->getMF()->getJumpTableInfo();
    const auto &Table = JTI->getJumpTables()[MO.getIndex()];
    assert(Table.MBBs.size() < 256);
    Expr = MCBinaryExpr::createAdd(
        Expr, MCConstantExpr::create(Table.MBBs.size(), Ctx), Ctx);
    break;
  }
  }
  return MCOperand::createExpr(Expr);
}
