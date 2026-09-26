//===-- MOSCallingConvention.cpp --------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOSCallingConvention.h"

#include "llvm/ADT/StringRef.h"

#include <string>

using namespace lldb_private;

namespace {
uint32_t LookupDwarfByName(llvm::MCRegisterInfo &mc_info,
                           llvm::StringRef name) {
  std::string mc_name = name.upper();
  for (unsigned reg = 0; reg < mc_info.getNumRegs(); ++reg) {
    if (mc_info.getName(reg) == mc_name) {
      int n = mc_info.getDwarfRegNum(reg, /*isEH=*/false);
      if (n >= 0)
        return static_cast<uint32_t>(n);
      break;
    }
  }
  return LLDB_INVALID_REGNUM;
}
} // namespace

void MOSCallingConvention::Init(llvm::MCRegisterInfo &mc_info) {
  if (initialized_)
    return;
  rc_base_dwarf_ = LookupDwarfByName(mc_info, "RC0");
  rs_base_dwarf_ = LookupDwarfByName(mc_info, "RS0");
  a_dwarf_ = LookupDwarfByName(mc_info, "A");
  x_dwarf_ = LookupDwarfByName(mc_info, "X");
  y_dwarf_ = LookupDwarfByName(mc_info, "Y");
  p_dwarf_ = LookupDwarfByName(mc_info, "P");
  initialized_ = true;
}

bool MOSCallingConvention::IsVolatile(uint32_t dwarf_num) const {
  if (!initialized_ || dwarf_num == LLDB_INVALID_REGNUM)
    return false;

  // Hardware A/X/Y and the P flag register are call-clobbered.
  if (dwarf_num == a_dwarf_ || dwarf_num == x_dwarf_ ||
      dwarf_num == y_dwarf_ || dwarf_num == p_dwarf_)
    return true;

  // Imaginary RC bank (rc0..rc255). Per MOSCallingConv.td, only rc20..rc31
  // (MOS_CSR) are callee-saved. Everything else in the RC bank is either
  // reserved runtime scratch (rc0, rc1) or caller-saved (rc2..rc19 for args,
  // rc32..rc255 unused by the CC and therefore not preserved).
  if (rc_base_dwarf_ != LLDB_INVALID_REGNUM &&
      dwarf_num >= rc_base_dwarf_ && dwarf_num - rc_base_dwarf_ <= 255) {
    uint32_t idx = dwarf_num - rc_base_dwarf_;
    return !(idx >= 20 && idx <= 31);
  }

  // Imaginary RS bank (rs0..rs127). rs0 is the soft stack pointer
  // (callee-saved). rs10..rs15 are callee-saved (they alias the callee-saved
  // rc20..rc31 pairs). rs1..rs9 and rs16.. are call-clobbered.
  if (rs_base_dwarf_ != LLDB_INVALID_REGNUM &&
      dwarf_num >= rs_base_dwarf_ && dwarf_num - rs_base_dwarf_ <= 127) {
    uint32_t idx = dwarf_num - rs_base_dwarf_;
    return !(idx == 0 || (idx >= 10 && idx <= 15));
  }

  // S, PC, and anything else: preserved.
  return false;
}
