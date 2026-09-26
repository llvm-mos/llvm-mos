//===-- MOSCFIExprHelpers.h - MOS CFI expression-form helpers ---*- C++ -*-===//
//
// Part of LLVM-MOS, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// MCCFIInstruction has no first-class constructors for DW_CFA_expression,
// DW_CFA_val_expression, or DW_CFA_def_cfa_expression.  Targets that need
// them assemble the raw escape byte sequence and hand it to
// MCCFIInstruction::createEscape.
//
// These helpers do exactly that, with signatures that match what
// first-class constructors would take.  If upstream ever grows real
// constructors (see the discussion on llvm-mos/llvm-mos#568), migration
// is a body swap on these three functions with no changes at the call
// sites.
//
// Encoding (DWARF 5, section 6.4.2):
//   DW_CFA_def_cfa_expression  opcode <ULEB128 len> <expr bytes>
//   DW_CFA_expression          opcode <ULEB128 reg> <ULEB128 len> <expr bytes>
//   DW_CFA_val_expression      opcode <ULEB128 reg> <ULEB128 len> <expr bytes>
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MOS_MOSCFIEXPRHELPERS_H
#define LLVM_LIB_TARGET_MOS_MOSCFIEXPRHELPERS_H

#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/BinaryFormat/Dwarf.h"
#include "llvm/MC/MCDwarf.h"
#include "llvm/Support/LEB128.h"
#include "llvm/Support/raw_ostream.h"

namespace llvm {
namespace mos {

inline MCCFIInstruction createDefCfaExpression(StringRef Expr) {
  SmallString<32> Data;
  raw_svector_ostream OS(Data);
  OS << uint8_t(dwarf::DW_CFA_def_cfa_expression);
  encodeULEB128(Expr.size(), OS);
  OS << Expr;
  return MCCFIInstruction::createEscape(nullptr, OS.str());
}

inline MCCFIInstruction createRegExpression(uint8_t CFAOpcode,
                                            unsigned DwarfReg,
                                            StringRef Expr) {
  SmallString<32> Data;
  raw_svector_ostream OS(Data);
  OS << CFAOpcode;
  encodeULEB128(DwarfReg, OS);
  encodeULEB128(Expr.size(), OS);
  OS << Expr;
  return MCCFIInstruction::createEscape(nullptr, OS.str());
}

inline MCCFIInstruction createExpression(unsigned DwarfReg, StringRef Expr) {
  return createRegExpression(dwarf::DW_CFA_expression, DwarfReg, Expr);
}

inline MCCFIInstruction createValExpression(unsigned DwarfReg, StringRef Expr) {
  return createRegExpression(dwarf::DW_CFA_val_expression, DwarfReg, Expr);
}

} // namespace mos
} // namespace llvm

#endif // LLVM_LIB_TARGET_MOS_MOSCFIEXPRHELPERS_H
