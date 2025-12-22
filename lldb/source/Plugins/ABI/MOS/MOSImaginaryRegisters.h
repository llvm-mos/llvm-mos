//===-- MOSImaginaryRegisters.h ---------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// MOS "imaginary registers" are zero-page memory locations that the compiler
// treats as registers. This class manages the mapping from DWARF register
// numbers to zero-page addresses, scanned from ELF symbols (__rc0, __rc1, etc).
//
// The DWARF register numbering comes from MOSRegisterInfo.td via MCRegisterInfo
// - no magic constants are hardcoded here.
//
//===----------------------------------------------------------------------===//

#ifndef LLDB_SOURCE_PLUGINS_ABI_MOS_MOSIMAGINARYREGISTERS_H
#define LLDB_SOURCE_PLUGINS_ABI_MOS_MOSIMAGINARYREGISTERS_H

#include "lldb/Target/DynamicRegisterInfo.h"
#include "lldb/lldb-types.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/MC/MCRegisterInfo.h"

#include <cstdint>
#include <optional>

namespace lldb_private {
class Module;
} // namespace lldb_private

namespace lldb_private {

class MOSImaginaryRegisters {
public:
  /// Default constructor - needed for StringMap storage.
  MOSImaginaryRegisters() = default;

  /// Get or create the imaginary register map for a module.
  /// Cached by module UUID (or file path as fallback).
  static MOSImaginaryRegisters &GetOrCreate(Module &module,
                                             llvm::MCRegisterInfo &mc_info);

  /// Clear the cache (e.g., when plugin terminates).
  static void ClearCache();

  /// Check if a DWARF register number is an imaginary register (RC or RS).
  bool IsImaginary(uint32_t dwarf_num) const;

  /// Check if a DWARF register number is an 8-bit RC register.
  bool IsRCRegister(uint32_t dwarf_num) const;

  /// Check if a DWARF register number is a 16-bit RS register.
  bool IsRSRegister(uint32_t dwarf_num) const;

  /// Get the zero-page address for an imaginary register.
  /// For RC registers, returns the single byte address.
  /// For RS registers, returns the address of the low byte.
  std::optional<lldb::addr_t> GetAddress(uint32_t dwarf_num) const;

  /// Get the number of RC registers present in this module.
  uint32_t GetNumRCRegisters() const { return num_rc_regs_; }

  /// Get the number of RS registers present in this module.
  uint32_t GetNumRSRegisters() const { return num_rs_regs_; }

  /// Get the base DWARF number for RC registers.
  uint32_t GetRCBaseDwarf() const { return rc_base_dwarf_; }

  /// Get the base DWARF number for RS registers.
  uint32_t GetRSBaseDwarf() const { return rs_base_dwarf_; }

  /// Add imaginary registers to the register list for AugmentRegisterInfo.
  void AddToRegisterList(std::vector<DynamicRegisterInfo::Register> &regs,
                         llvm::MCRegisterInfo &mc_info) const;

  /// Check if this module has any imaginary registers.
  bool HasImaginaryRegisters() const { return num_rc_regs_ > 0; }

private:
  void Initialize(Module &module, llvm::MCRegisterInfo &mc_info);
  void ScanForSymbols(Module &module);

  /// Get the RC index (0-255) from a DWARF register number.
  uint32_t GetRCIndex(uint32_t dwarf_num) const {
    return dwarf_num - rc_base_dwarf_;
  }

  /// Get the RS index (0-127) from a DWARF register number.
  uint32_t GetRSIndex(uint32_t dwarf_num) const {
    return dwarf_num - rs_base_dwarf_;
  }

  // Cache keyed by module UUID or file path
  static llvm::StringMap<MOSImaginaryRegisters> s_cache_;

  // Zero-page addresses for RC registers, indexed by register number (0-255)
  llvm::SmallVector<lldb::addr_t, 32> rc_addresses_;

  // Base DWARF numbers - discovered from MCRegisterInfo at init time
  uint32_t rc_base_dwarf_ = 0;
  uint32_t rs_base_dwarf_ = 0;

  // How many registers are present in this module
  uint32_t num_rc_regs_ = 0;
  uint32_t num_rs_regs_ = 0;

  // Whether initialization has been done
  bool initialized_ = false;
};

} // namespace lldb_private

#endif // LLDB_SOURCE_PLUGINS_ABI_MOS_MOSIMAGINARYREGISTERS_H
