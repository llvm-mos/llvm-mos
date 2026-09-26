//===-- MOSCallingConvention.h ----------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Classifies MOS registers by their default calling-convention behavior:
// call-clobbered ("volatile" in LLDB terminology) vs preserved across calls.
//
// SOURCE OF TRUTH: llvm/lib/Target/MOS/MOSCallingConv.td (the leading comment
// block enumerates caller-saved vs callee-saved registers; MOS_CSR names the
// callee-saved imaginary RC set). If the calling convention changes there,
// update the classification here.
//
//===----------------------------------------------------------------------===//

#ifndef LLDB_SOURCE_PLUGINS_ABI_MOS_MOSCALLINGCONVENTION_H
#define LLDB_SOURCE_PLUGINS_ABI_MOS_MOSCALLINGCONVENTION_H

#include "lldb/lldb-defines.h"
#include "llvm/MC/MCRegisterInfo.h"

#include <cstdint>

namespace lldb_private {

class MOSCallingConvention {
public:
  MOSCallingConvention() = default;

  /// Discover the base DWARF numbers of the register banks by name via
  /// MCRegisterInfo. Safe to call more than once; second call is a no-op.
  void Init(llvm::MCRegisterInfo &mc_info);

  /// True if the register is call-clobbered per the default calling
  /// convention. False for callee-saved / preserved registers and for
  /// anything unrecognized (conservative).
  bool IsVolatile(uint32_t dwarf_num) const;

private:
  uint32_t rc_base_dwarf_ = LLDB_INVALID_REGNUM;
  uint32_t rs_base_dwarf_ = LLDB_INVALID_REGNUM;
  uint32_t a_dwarf_ = LLDB_INVALID_REGNUM;
  uint32_t x_dwarf_ = LLDB_INVALID_REGNUM;
  uint32_t y_dwarf_ = LLDB_INVALID_REGNUM;
  uint32_t p_dwarf_ = LLDB_INVALID_REGNUM;
  bool initialized_ = false;
};

} // namespace lldb_private

#endif // LLDB_SOURCE_PLUGINS_ABI_MOS_MOSCALLINGCONVENTION_H
