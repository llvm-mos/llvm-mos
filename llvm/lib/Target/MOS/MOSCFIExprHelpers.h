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

/// Emit a DWARF expression that computes the value of a register plus a
/// constant offset: DW_OP_breg<N> for the low register numbers, DW_OP_bregx
/// otherwise (the imaginary registers have large DWARF numbers).
inline void emitRegPlusOffsetExpr(SmallVectorImpl<char> &Expr,
                                  unsigned DwarfReg, int64_t Offset) {
  raw_svector_ostream OS(Expr);
  if (DwarfReg < 32) {
    OS << uint8_t(dwarf::DW_OP_breg0 + DwarfReg);
  } else {
    OS << uint8_t(dwarf::DW_OP_bregx);
    encodeULEB128(DwarfReg, OS);
  }
  encodeSLEB128(Offset, OS);
}

/// Emit a DWARF expression that computes a normalized hardware stack address.
/// The 6502 hardware stack is at 0x0100-0x01FF. The S register is physically
/// 8-bit, but some debuggers report it as 16-bit (e.g., MAME reports 0x01FE).
/// This expression normalizes to always produce addresses in 0x0100-0x01FF:
///   result = ((S + Offset) & 0xFF) | 0x0100
inline void emitNormalizedHardwareStackExpr(SmallVectorImpl<char> &Expr,
                                            unsigned DwarfS, int64_t Offset) {
  // S + offset
  emitRegPlusOffsetExpr(Expr, DwarfS, Offset);

  // DW_OP_const1u 0xFF
  Expr.push_back(static_cast<char>(dwarf::DW_OP_const1u));
  Expr.push_back(static_cast<char>(0xFF));

  // DW_OP_and - keep low byte only
  Expr.push_back(static_cast<char>(dwarf::DW_OP_and));

  // DW_OP_const2u 0x0100 (little-endian: 0x00, 0x01)
  Expr.push_back(static_cast<char>(dwarf::DW_OP_const2u));
  Expr.push_back(static_cast<char>(0x00));
  Expr.push_back(static_cast<char>(0x01));

  // DW_OP_or - force into hardware stack page
  Expr.push_back(static_cast<char>(dwarf::DW_OP_or));
}

/// Emit a DWARF expression that yields the *value* of the caller's PC, for use
/// with DW_CFA_val_expression.
///
/// The return address is pushed high byte first, so the low byte is at
/// S + Offset and the high byte at S + Offset + 1. The two bytes are read
/// independently because they are physically non-consecutive when S wraps
/// within the stack page.
///
/// JSR pushes (return_PC - 1), so for an ordinary call the result adds 1 to
/// recover the true return address. An interrupt pushes the address RTI
/// returns to exactly, so for an ISR (\p IsISR) nothing is added.
inline void emitReturnAddressValueExpr(SmallVectorImpl<char> &Expr,
                                       unsigned DwarfS, int64_t Offset,
                                       bool IsISR = false) {
  // Low byte
  emitNormalizedHardwareStackExpr(Expr, DwarfS, Offset);
  Expr.push_back(static_cast<char>(dwarf::DW_OP_deref_size));
  Expr.push_back(static_cast<char>(1));

  // High byte
  emitNormalizedHardwareStackExpr(Expr, DwarfS, Offset + 1);
  Expr.push_back(static_cast<char>(dwarf::DW_OP_deref_size));
  Expr.push_back(static_cast<char>(1));

  // low | (high << 8)
  Expr.push_back(static_cast<char>(dwarf::DW_OP_const1u));
  Expr.push_back(static_cast<char>(8));
  Expr.push_back(static_cast<char>(dwarf::DW_OP_shl));
  Expr.push_back(static_cast<char>(dwarf::DW_OP_or));

  if (IsISR)
    return;

  // + 1 - JSR pushes PC-1
  Expr.push_back(static_cast<char>(dwarf::DW_OP_plus_uconst));
  Expr.push_back(static_cast<char>(1)); // ULEB128 encoding of 1
}

} // namespace mos
} // namespace llvm

#endif // LLVM_LIB_TARGET_MOS_MOSCFIEXPRHELPERS_H
