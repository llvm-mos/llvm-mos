//===-- MOSRegisterContext.cpp ----------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOSRegisterContext.h"
#include "MOSImaginaryRegisters.h"

#include "lldb/Target/Process.h"
#include "lldb/Target/Thread.h"
#include "lldb/Utility/LLDBLog.h"
#include "lldb/Utility/Log.h"
#include "lldb/Utility/RegisterValue.h"
#include "lldb/Utility/Status.h"

using namespace lldb;
using namespace lldb_private;

MOSRegisterContext::MOSRegisterContext(
    lldb_private::process_gdb_remote::ThreadGDBRemote &thread,
    uint32_t concrete_frame_idx,
    lldb_private::process_gdb_remote::GDBRemoteDynamicRegisterInfoSP
        reg_info_sp,
    bool read_all_registers_at_once, bool write_all_registers_at_once,
    const MOSImaginaryRegisters &imag_regs)
    : GDBRemoteRegisterContext(thread, concrete_frame_idx, reg_info_sp,
                               read_all_registers_at_once,
                               write_all_registers_at_once),
      m_imag_regs(imag_regs) {
  LLDB_LOG(GetLog(LLDBLog::Expressions),
           "MOSRegisterContext: Created with {0} RC and {1} RS registers",
           imag_regs.GetNumRCRegisters(), imag_regs.GetNumRSRegisters());
}

bool MOSRegisterContext::ReadRegister(const RegisterInfo *reg_info,
                                      RegisterValue &value) {
  if (!reg_info)
    return false;

  // Check if this is an imaginary register by its DWARF number
  uint32_t dwarf_num = reg_info->kinds[eRegisterKindDWARF];
  if (dwarf_num != LLDB_INVALID_REGNUM && m_imag_regs.IsImaginary(dwarf_num)) {
    return ReadImaginaryRegister(dwarf_num, reg_info->byte_size, value);
  }

  // Fallback to base class for hardware registers
  return GDBRemoteRegisterContext::ReadRegister(reg_info, value);
}

bool MOSRegisterContext::WriteRegister(const RegisterInfo *reg_info,
                                       const RegisterValue &value) {
  if (!reg_info)
    return false;

  // Check if this is an imaginary register by its DWARF number
  uint32_t dwarf_num = reg_info->kinds[eRegisterKindDWARF];
  if (dwarf_num != LLDB_INVALID_REGNUM && m_imag_regs.IsImaginary(dwarf_num)) {
    return WriteImaginaryRegister(dwarf_num, reg_info->byte_size, value);
  }

  // Fallback to base class for hardware registers
  return GDBRemoteRegisterContext::WriteRegister(reg_info, value);
}

bool MOSRegisterContext::ReadImaginaryRegister(uint32_t dwarf_num,
                                               uint32_t byte_size,
                                               RegisterValue &value) {
  auto addr_opt = m_imag_regs.GetAddress(dwarf_num);
  if (!addr_opt) {
    LLDB_LOG(GetLog(LLDBLog::Expressions),
             "MOSRegisterContext: No address for DWARF reg {0}", dwarf_num);
    return false;
  }

  addr_t addr = *addr_opt;
  ProcessSP process_sp = m_thread.GetProcess();
  if (!process_sp)
    return false;

  Status error;

  if (byte_size == 1) {
    // RC register - single byte
    uint8_t byte = 0;
    size_t bytes_read = process_sp->ReadMemory(addr, &byte, 1, error);
    if (bytes_read == 1 && error.Success()) {
      value.SetUInt8(byte);
      LLDB_LOG(GetLog(LLDBLog::Expressions),
               "MOSRegisterContext: Read RC DWARF={0} from 0x{1:x} = 0x{2:x}",
               dwarf_num, addr, byte);
      return true;
    }
  } else if (byte_size == 2) {
    // RS register - two bytes (little-endian)
    // RS[n] is stored at the address of RC[2*n] (low byte) and RC[2*n+1] (high)
    // The GetAddress for RS returns the low byte address
    uint8_t lo = 0, hi = 0;
    if (process_sp->ReadMemory(addr, &lo, 1, error) != 1 || !error.Success())
      return false;
    if (process_sp->ReadMemory(addr + 1, &hi, 1, error) != 1 ||
        !error.Success())
      return false;

    uint16_t val = lo | (hi << 8);
    value.SetUInt16(val);
    LLDB_LOG(GetLog(LLDBLog::Expressions),
             "MOSRegisterContext: Read RS DWARF={0} from 0x{1:x} = 0x{2:x}",
             dwarf_num, addr, val);
    return true;
  }

  LLDB_LOG(GetLog(LLDBLog::Expressions),
           "MOSRegisterContext: Failed to read DWARF reg {0} at 0x{1:x}",
           dwarf_num, addr);
  return false;
}

bool MOSRegisterContext::WriteImaginaryRegister(uint32_t dwarf_num,
                                                uint32_t byte_size,
                                                const RegisterValue &value) {
  auto addr_opt = m_imag_regs.GetAddress(dwarf_num);
  if (!addr_opt) {
    LLDB_LOG(GetLog(LLDBLog::Expressions),
             "MOSRegisterContext: No address for DWARF reg {0}", dwarf_num);
    return false;
  }

  addr_t addr = *addr_opt;
  ProcessSP process_sp = m_thread.GetProcess();
  if (!process_sp)
    return false;

  Status error;

  if (byte_size == 1) {
    // RC register - single byte
    uint8_t byte = value.GetAsUInt8();
    size_t bytes_written = process_sp->WriteMemory(addr, &byte, 1, error);
    if (bytes_written == 1 && error.Success()) {
      LLDB_LOG(GetLog(LLDBLog::Expressions),
               "MOSRegisterContext: Wrote RC DWARF={0} to 0x{1:x} = 0x{2:x}",
               dwarf_num, addr, byte);
      return true;
    }
  } else if (byte_size == 2) {
    // RS register - two bytes (little-endian)
    uint16_t val = value.GetAsUInt16();
    uint8_t lo = val & 0xFF;
    uint8_t hi = (val >> 8) & 0xFF;

    if (process_sp->WriteMemory(addr, &lo, 1, error) != 1 || !error.Success())
      return false;
    if (process_sp->WriteMemory(addr + 1, &hi, 1, error) != 1 ||
        !error.Success())
      return false;

    LLDB_LOG(GetLog(LLDBLog::Expressions),
             "MOSRegisterContext: Wrote RS DWARF={0} to 0x{1:x} = 0x{2:x}",
             dwarf_num, addr, val);
    return true;
  }

  LLDB_LOG(GetLog(LLDBLog::Expressions),
           "MOSRegisterContext: Failed to write DWARF reg {0} at 0x{1:x}",
           dwarf_num, addr);
  return false;
}
