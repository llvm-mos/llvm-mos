//===-- MOSRegisterContext.h ------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Custom register context for MOS that handles imaginary registers.
// Imaginary registers are zero-page memory locations that the compiler
// treats as registers. This context intercepts reads/writes to these
// registers and translates them to memory accesses.
//
//===----------------------------------------------------------------------===//

#ifndef LLDB_SOURCE_PLUGINS_ABI_MOS_MOSREGISTERCONTEXT_H
#define LLDB_SOURCE_PLUGINS_ABI_MOS_MOSREGISTERCONTEXT_H

#include "Plugins/Process/gdb-remote/GDBRemoteRegisterContext.h"

namespace lldb_private {
class MOSImaginaryRegisters;
} // namespace lldb_private

class MOSRegisterContext
    : public lldb_private::process_gdb_remote::GDBRemoteRegisterContext {
public:
  MOSRegisterContext(
      lldb_private::process_gdb_remote::ThreadGDBRemote &thread,
      uint32_t concrete_frame_idx,
      lldb_private::process_gdb_remote::GDBRemoteDynamicRegisterInfoSP
          reg_info_sp,
      bool read_all_registers_at_once, bool write_all_registers_at_once,
      const lldb_private::MOSImaginaryRegisters &imag_regs);

  bool ReadRegister(const lldb_private::RegisterInfo *reg_info,
                    lldb_private::RegisterValue &value) override;

  bool WriteRegister(const lldb_private::RegisterInfo *reg_info,
                     const lldb_private::RegisterValue &value) override;

private:
  /// Reference to the imaginary registers helper.
  /// This provides DWARF-number-based lookup for imaginary register addresses.
  const lldb_private::MOSImaginaryRegisters &m_imag_regs;

  /// Read an imaginary register by reading from zero-page memory.
  /// @param dwarf_num The DWARF register number.
  /// @param byte_size 1 for RC registers, 2 for RS registers.
  /// @param value Output register value.
  /// @return true on success.
  bool ReadImaginaryRegister(uint32_t dwarf_num, uint32_t byte_size,
                             lldb_private::RegisterValue &value);

  /// Write an imaginary register by writing to zero-page memory.
  /// @param dwarf_num The DWARF register number.
  /// @param byte_size 1 for RC registers, 2 for RS registers.
  /// @param value The value to write.
  /// @return true on success.
  bool WriteImaginaryRegister(uint32_t dwarf_num, uint32_t byte_size,
                              const lldb_private::RegisterValue &value);
};

#endif // LLDB_SOURCE_PLUGINS_ABI_MOS_MOSREGISTERCONTEXT_H
