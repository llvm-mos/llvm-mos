//===-- MOSMCCodeEmitter.cpp - Convert MOS Code to Machine Code -----------===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the MOSMCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "MOSMCCodeEmitter.h"

#include "MCTargetDesc/MOSMCExpr.h"
#include "MCTargetDesc/MOSMCTargetDesc.h"

#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/EndianStream.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "mccodeemitter"

#define GET_INSTRMAP_INFO
#include "MOSGenInstrInfo.inc"
#undef GET_INSTRMAP_INFO

namespace llvm {

static void emitLittleEndian(uint64_t Val, unsigned Size,
                             SmallVectorImpl<char> &CB) {
  for (int64_t I = 0; I < Size; ++I) {
    CB.push_back((char)(Val & 0xff));
    Val = Val >> 8;
  }
}

void MOSMCCodeEmitter::encodeInstruction(const MCInst &MI,
                                         SmallVectorImpl<char> &CB,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
  // Get byte count of instruction
  unsigned Size = Desc.getSize();

  assert(Size > 0 && "Instruction size cannot be zero");

  uint64_t BinaryOpCode = getBinaryCodeForInstr(MI, Fixups, STI);
  emitLittleEndian(BinaryOpCode, Size, CB);
}

template <MOS::Fixups Fixup, unsigned Offset>
unsigned MOSMCCodeEmitter::encodeImm(const MCInst &MI, unsigned OpNo,
                                     SmallVectorImpl<MCFixup> &Fixups,
                                     const MCSubtargetInfo &STI) const {
  auto MO = MI.getOperand(OpNo);

  if (MO.isExpr()) {
    if (isa<MOSMCExpr>(MO.getExpr())) {
      // If the expression is already a MOSMCExpr,
      // we shouldn't perform any more fixups. Without this check, we would
      // instead create a fixup to the symbol named 'lo8(symbol)' which
      // is not correct.
      return getExprOpValue(MO.getExpr(), Fixups, STI, Offset);
    }

    MCFixupKind FixupKind = static_cast<MCFixupKind>(Fixup);
    Fixups.push_back(MCFixup::create(Offset, MO.getExpr(), FixupKind, MI.getLoc()));

    return 0;
  }

  assert(MO.isImm());
  return MO.getImm();
}

unsigned MOSMCCodeEmitter::getExprOpValue(const MCExpr *Expr,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI,
                                          unsigned int Offset ) const {

  MCExpr::ExprKind Kind = Expr->getKind();

  if (Kind == MCExpr::Binary) {
    Expr = static_cast<const MCBinaryExpr *>(Expr)->getLHS();
    Kind = Expr->getKind();
  }

  if (Kind == MOSMCExpr::Target) {
    MOSMCExpr const *MOSExpr = cast<MOSMCExpr>(Expr);
    int64_t Result;
    if (MOSExpr->evaluateAsConstant(Result)) {
      return Result;
    }

    MCFixupKind FixupKind = static_cast<MCFixupKind>(MOSExpr->getFixupKind());
    Fixups.push_back(MCFixup::create(Offset, MOSExpr, FixupKind));
    return 0;
  }

  assert(Kind == MCExpr::SymbolRef);
  return 0;
}


unsigned MOSMCCodeEmitter::getMachineOpValue(const MCInst &MI,
                                             const MCOperand &MO,
                                             SmallVectorImpl<MCFixup> &Fixups,
                                             const MCSubtargetInfo &STI) const {
  if (MO.isImm())
    return MO.getImm();

  assert(MO.isExpr());

  const MCExpr *Expr = MO.getExpr();
  if (isa<MCSymbolRefExpr>(Expr)) {
    Fixups.push_back(MCFixup::create(0, Expr, MCFixupKind::FK_Data_1));
    return 0;
  }

  int64_t Res;
  if (Expr->evaluateAsAbsolute(Res)) {
    return Res;
  }

  llvm_unreachable("Unhandled expression!");
  return 0;
}

MCCodeEmitter *createMOSMCCodeEmitter(const MCInstrInfo &MCII, MCContext &Ctx) {
  return new MOSMCCodeEmitter(MCII, Ctx);
}

} // end of namespace llvm

#define ENABLE_INSTR_PREDICATE_VERIFIER
#include "MOSGenMCCodeEmitter.inc"
