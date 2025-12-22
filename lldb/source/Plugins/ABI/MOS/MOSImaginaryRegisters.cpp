//===-- MOSImaginaryRegisters.cpp -------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOSImaginaryRegisters.h"

#include "lldb/Core/Module.h"
#include "lldb/Symbol/Symbol.h"
#include "lldb/Symbol/Symtab.h"
#include "lldb/Utility/ConstString.h"
#include "lldb/Utility/LLDBLog.h"
#include "lldb/Utility/Log.h"

using namespace lldb;
using namespace lldb_private;

// Static cache
llvm::StringMap<MOSImaginaryRegisters> MOSImaginaryRegisters::s_cache_;

//------------------------------------------------------------------
// Helper to look up DWARF register number by name via MCRegisterInfo.
// This is DRY - the actual DWARF numbers come from MOSRegisterInfo.td
// through the MCRegisterInfo generated at LLVM build time.
//------------------------------------------------------------------
static int GetDwarfRegNumByName(llvm::MCRegisterInfo &mc_info,
                                llvm::StringRef name) {
  // MCRegisterInfo stores names in uppercase
  std::string mc_name = name.upper();

  for (unsigned reg = 0; reg < mc_info.getNumRegs(); ++reg) {
    if (mc_info.getName(reg) == mc_name) {
      return mc_info.getDwarfRegNum(reg, /*isEH=*/false);
    }
  }
  return -1;
}

MOSImaginaryRegisters &
MOSImaginaryRegisters::GetOrCreate(Module &module,
                                   llvm::MCRegisterInfo &mc_info) {
  // Use UUID as cache key, fall back to file path
  std::string key = module.GetUUID().GetAsString();
  if (key.empty()) {
    key = module.GetFileSpec().GetPath();
  }

  auto it = s_cache_.find(key);
  if (it != s_cache_.end() && it->second.initialized_) {
    return it->second;
  }

  auto &regs = s_cache_[key];
  regs.Initialize(module, mc_info);
  return regs;
}

void MOSImaginaryRegisters::ClearCache() { s_cache_.clear(); }

void MOSImaginaryRegisters::Initialize(Module &module,
                                       llvm::MCRegisterInfo &mc_info) {
  if (initialized_)
    return;

  // Query MCRegisterInfo for the base DWARF numbers by name.
  // These come from MOSRegisterInfo.td - single source of truth.
  // We look up "RC0" and "RS0" by name to get their DWARF numbers.
  int rc0_dwarf = GetDwarfRegNumByName(mc_info, "RC0");
  int rs0_dwarf = GetDwarfRegNumByName(mc_info, "RS0");

  if (rc0_dwarf >= 0) {
    rc_base_dwarf_ = static_cast<uint32_t>(rc0_dwarf);
  } else {
    LLDB_LOG(GetLog(LLDBLog::Expressions),
             "MOSImaginaryRegisters: Could not find DWARF number for RC0");
  }

  if (rs0_dwarf >= 0) {
    rs_base_dwarf_ = static_cast<uint32_t>(rs0_dwarf);
  } else {
    LLDB_LOG(GetLog(LLDBLog::Expressions),
             "MOSImaginaryRegisters: Could not find DWARF number for RS0");
  }

  LLDB_LOG(GetLog(LLDBLog::Expressions),
           "MOSImaginaryRegisters: RC base DWARF={0}, RS base DWARF={1}",
           rc_base_dwarf_, rs_base_dwarf_);

  ScanForSymbols(module);
  initialized_ = true;
}

void MOSImaginaryRegisters::ScanForSymbols(Module &module) {
  Symtab *symtab = module.GetSymtab();
  if (!symtab)
    return;

  // Scan for __rc0, __rc1, ..., __rcN symbols
  // These are absolute symbols containing the zero-page address
  for (uint32_t i = 0; i < 256; ++i) {
    std::string symbol_name = "__rc" + std::to_string(i);
    Symbol *symbol = symtab->FindFirstSymbolWithNameAndType(
        ConstString(symbol_name), eSymbolTypeAbsolute, Symtab::eDebugAny,
        Symtab::eVisibilityAny);

    if (!symbol)
      continue;

    // GetRawValue() is the correct way to get absolute symbol values
    addr_t addr = symbol->GetRawValue();

    // Extend the vector if needed
    if (i >= rc_addresses_.size()) {
      rc_addresses_.resize(i + 1, LLDB_INVALID_ADDRESS);
    }
    rc_addresses_[i] = addr;
    num_rc_regs_ = std::max(num_rc_regs_, i + 1);

    LLDB_LOG(GetLog(LLDBLog::Expressions),
             "MOSImaginaryRegisters: {0} -> 0x{1:x}", symbol_name, addr);
  }

  // RS registers are pairs of RC registers: RS[n] = {RC[2n], RC[2n+1]}
  // So num_rs_regs = num_rc_regs / 2
  num_rs_regs_ = num_rc_regs_ / 2;

  LLDB_LOG(GetLog(LLDBLog::Expressions),
           "MOSImaginaryRegisters: Found {0} RC registers, {1} RS registers",
           num_rc_regs_, num_rs_regs_);
}

bool MOSImaginaryRegisters::IsImaginary(uint32_t dwarf_num) const {
  return IsRCRegister(dwarf_num) || IsRSRegister(dwarf_num);
}

bool MOSImaginaryRegisters::IsRCRegister(uint32_t dwarf_num) const {
  return dwarf_num >= rc_base_dwarf_ &&
         dwarf_num < rc_base_dwarf_ + num_rc_regs_;
}

bool MOSImaginaryRegisters::IsRSRegister(uint32_t dwarf_num) const {
  return dwarf_num >= rs_base_dwarf_ &&
         dwarf_num < rs_base_dwarf_ + num_rs_regs_;
}

std::optional<addr_t>
MOSImaginaryRegisters::GetAddress(uint32_t dwarf_num) const {
  if (IsRCRegister(dwarf_num)) {
    uint32_t index = GetRCIndex(dwarf_num);
    if (index < rc_addresses_.size() &&
        rc_addresses_[index] != LLDB_INVALID_ADDRESS) {
      return rc_addresses_[index];
    }
  } else if (IsRSRegister(dwarf_num)) {
    // RS[n] is at the address of RC[2*n] (low byte)
    uint32_t rs_index = GetRSIndex(dwarf_num);
    uint32_t rc_index = rs_index * 2;
    if (rc_index < rc_addresses_.size() &&
        rc_addresses_[rc_index] != LLDB_INVALID_ADDRESS) {
      return rc_addresses_[rc_index];
    }
  }
  return std::nullopt;
}

void MOSImaginaryRegisters::AddToRegisterList(
    std::vector<DynamicRegisterInfo::Register> &regs,
    llvm::MCRegisterInfo &mc_info) const {

  if (!HasImaginaryRegisters())
    return;

  ConstString empty_alt_name;
  ConstString reg_set{"imaginary"};

  // Find the next available offset and regnum
  uint32_t next_offset = 0;
  uint32_t next_regnum = 0;
  for (const auto &reg : regs) {
    if (reg.byte_offset != LLDB_INVALID_INDEX32)
      next_offset = std::max(next_offset, reg.byte_offset + reg.byte_size);
    if (reg.regnum_remote != LLDB_INVALID_REGNUM)
      next_regnum = std::max(next_regnum, reg.regnum_remote + 1);
  }

  // Add RC registers (8-bit)
  for (uint32_t i = 0; i < num_rc_regs_; ++i) {
    uint32_t dwarf_num = rc_base_dwarf_ + i;
    std::string name = "rc" + std::to_string(i);

    DynamicRegisterInfo::Register reg{
        ConstString(name),
        empty_alt_name,
        reg_set,
        1, // byte_size
        next_offset,
        eEncodingUint,
        eFormatHex,
        dwarf_num,           // regnum_ehframe
        dwarf_num,           // regnum_dwarf
        LLDB_INVALID_REGNUM, // regnum_generic
        LLDB_INVALID_REGNUM, // regnum_remote (will be set below)
        {},                  // value_regs
        {}                   // invalidate_regs
    };
    reg.regnum_remote = next_regnum++;
    next_offset += 1;
    regs.push_back(reg);
  }

  // Add RS registers (16-bit, pairs of RC registers)
  for (uint32_t i = 0; i < num_rs_regs_; ++i) {
    uint32_t dwarf_num = rs_base_dwarf_ + i;
    std::string name = "rs" + std::to_string(i);

    DynamicRegisterInfo::Register reg{
        ConstString(name),
        empty_alt_name,
        reg_set,
        2, // byte_size
        next_offset,
        eEncodingUint,
        eFormatHex,
        dwarf_num, // regnum_ehframe
        dwarf_num, // regnum_dwarf
        (i == 0) ? LLDB_REGNUM_GENERIC_FP
                 : LLDB_INVALID_REGNUM, // RS0 is soft stack pointer
        LLDB_INVALID_REGNUM,            // regnum_remote
        {},                             // value_regs
        {}                              // invalidate_regs
    };
    reg.regnum_remote = next_regnum++;
    next_offset += 2;
    regs.push_back(reg);
  }
}
