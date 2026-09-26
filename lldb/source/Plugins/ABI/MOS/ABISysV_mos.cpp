//===-- ABISysV_mos.cpp -----------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "ABISysV_mos.h"
#include "MOSImaginaryRegisters.h"
#include "MOSRegisterContext.h"

#include "lldb/Core/PluginManager.h"
#include "lldb/Core/Value.h"
#include "lldb/Symbol/UnwindPlan.h"
#include "lldb/Target/Process.h"
#include "lldb/Target/Thread.h"
#include "lldb/Utility/LLDBLog.h"
#include "lldb/Utility/Log.h"

#include "Plugins/Process/gdb-remote/ThreadGDBRemote.h"

#include "llvm/BinaryFormat/Dwarf.h"
#include "llvm/TargetParser/Triple.h"

using namespace lldb;
using namespace lldb_private;

LLDB_PLUGIN_DEFINE_ADV(ABISysV_mos, ArchitectureMOS)

//------------------------------------------------------------------
// Static Functions
//------------------------------------------------------------------

void ABISysV_mos::Initialize() {
  PluginManager::RegisterPlugin(GetPluginNameStatic(),
                                "System V ABI for MOS targets", CreateInstance);
}

void ABISysV_mos::Terminate() {
  MOSImaginaryRegisters::ClearCache();
  PluginManager::UnregisterPlugin(CreateInstance);
}

ABISP ABISysV_mos::CreateInstance(lldb::ProcessSP process_sp,
                                  const ArchSpec &arch) {
  if (arch.GetTriple().getArch() == llvm::Triple::mos) {
    return ABISP(
        new ABISysV_mos(std::move(process_sp), MakeMCRegisterInfo(arch)));
  }
  return ABISP();
}

//------------------------------------------------------------------
// MCBasedABI interface
//------------------------------------------------------------------

uint32_t ABISysV_mos::GetGenericNum(llvm::StringRef name) {
  // Map register names to generic register numbers
  if (name.equals_insensitive("pc"))
    return LLDB_REGNUM_GENERIC_PC;
  if (name.equals_insensitive("sp") || name.equals_insensitive("s"))
    return LLDB_REGNUM_GENERIC_SP;
  if (name.equals_insensitive("p"))
    return LLDB_REGNUM_GENERIC_FLAGS;
  // RS0 is the soft stack pointer (frame pointer)
  if (name.equals_insensitive("rs0"))
    return LLDB_REGNUM_GENERIC_FP;

  return LLDB_INVALID_REGNUM;
}

void ABISysV_mos::AugmentRegisterInfo(
    std::vector<DynamicRegisterInfo::Register> &regs) {
  // First, call base class to handle standard DWARF/EH frame numbers
  MCBasedABI::AugmentRegisterInfo(regs);

  // Add imaginary registers if we have a main module
  ModuleSP main_module = GetMainModule();
  if (main_module && m_mc_register_info_up) {
    MOSImaginaryRegisters &imag_regs = MOSImaginaryRegisters::GetOrCreate(
        *main_module, *m_mc_register_info_up);
    if (imag_regs.HasImaginaryRegisters()) {
      imag_regs.AddToRegisterList(regs, *m_mc_register_info_up);

      LLDB_LOG(GetLog(LLDBLog::Expressions),
               "ABISysV_mos: Added {0} RC and {1} RS imaginary registers",
               imag_regs.GetNumRCRegisters(), imag_regs.GetNumRSRegisters());
    }
  }
}

//------------------------------------------------------------------
// ABI interface - trivial call support (not implemented for 6502)
//------------------------------------------------------------------

bool ABISysV_mos::PrepareTrivialCall(Thread &thread, lldb::addr_t sp,
                                     lldb::addr_t func_addr,
                                     lldb::addr_t return_addr,
                                     llvm::ArrayRef<addr_t> args) const {
  // 6502 doesn't support complex calling conventions in the traditional sense
  return false;
}

bool ABISysV_mos::GetArgumentValues(Thread &thread, ValueList &values) const {
  // 6502 argument passing is very architecture-specific
  return false;
}

Status ABISysV_mos::SetReturnValueObject(lldb::StackFrameSP &frame_sp,
                                         lldb::ValueObjectSP &new_value) {
  return Status::FromErrorString(
      "Setting return values not implemented for MOS");
}

ValueObjectSP ABISysV_mos::GetReturnValueObjectImpl(Thread &thread,
                                                    CompilerType &type) const {
  return ValueObjectSP();
}

bool ABISysV_mos::RegisterIsVolatile(const RegisterInfo *reg_info) {
  // "Volatile" here means "call-clobbered" — used by LLDB to decide whether a
  // value in the younger frame's register can be inherited by the caller.
  // Classification lives in MOSCallingConvention, sourced from MOSCallingConv.td.
  if (!reg_info)
    return false;
  return m_calling_conv.IsVolatile(reg_info->kinds[eRegisterKindDWARF]);
}

//------------------------------------------------------------------
// Unwind Plans
//------------------------------------------------------------------

/// Build a DWARF expression that computes a normalized hardware stack address.
/// The 6502 hardware stack is at 0x0100-0x01FF. The S register is physically
/// 8-bit, but some debuggers report it as 16-bit (e.g., MAME reports 0x01FE).
/// This expression normalizes to always produce addresses in 0x0100-0x01FF:
///   result = ((S + Offset) & 0xFF) | 0x0100
static void buildNormalizedHardwareStackExpr(std::vector<uint8_t> &expr,
                                             uint8_t dwarf_s, int8_t offset) {
  // DW_OP_breg<S> <offset> - S + offset
  expr.push_back(llvm::dwarf::DW_OP_breg0 + dwarf_s);
  expr.push_back(static_cast<uint8_t>(offset));

  // DW_OP_const1u 0xFF
  expr.push_back(llvm::dwarf::DW_OP_const1u);
  expr.push_back(0xFF);

  // DW_OP_and - keep low byte only
  expr.push_back(llvm::dwarf::DW_OP_and);

  // DW_OP_const2u 0x0100 (little-endian: 0x00, 0x01)
  expr.push_back(llvm::dwarf::DW_OP_const2u);
  expr.push_back(0x00);
  expr.push_back(0x01);

  // DW_OP_or - force into hardware stack page
  expr.push_back(llvm::dwarf::DW_OP_or);
}

/// Build a DWARF expression that yields the *value* of the caller PC.
///
/// JSR pushes (return_PC - 1) as two independent bytes: high byte first at
/// the current S, then low byte at S-1, decrementing S each time. On entry
/// to the callee the low byte is at S+1 and the high byte is at S+2. When S
/// wraps within the stack page these two addresses are physically
/// non-consecutive (e.g. S=0xFE puts the low byte at 0x01FF and the high
/// byte at 0x0100), so we cannot use a single 2-byte dereference. Instead
/// we dereference each byte independently, reassemble the 16-bit value, and
/// add 1 to recover the true return address.
///
///   result = deref1(normalized(S+1)) | (deref1(normalized(S+2)) << 8) + 1
static void buildReturnAddressValueExpr(std::vector<uint8_t> &expr,
                                        uint8_t dwarf_s) {
  // Low byte of (return_PC - 1)
  buildNormalizedHardwareStackExpr(expr, dwarf_s, 1);
  expr.push_back(llvm::dwarf::DW_OP_deref_size);
  expr.push_back(1);

  // High byte of (return_PC - 1)
  buildNormalizedHardwareStackExpr(expr, dwarf_s, 2);
  expr.push_back(llvm::dwarf::DW_OP_deref_size);
  expr.push_back(1);

  // (high << 8)
  expr.push_back(llvm::dwarf::DW_OP_const1u);
  expr.push_back(8);
  expr.push_back(llvm::dwarf::DW_OP_shl);

  // low | (high << 8)
  expr.push_back(llvm::dwarf::DW_OP_or);

  // + 1 — JSR pushes PC-1, so add 1 to recover the true return address.
  expr.push_back(llvm::dwarf::DW_OP_plus_uconst);
  expr.push_back(1); // ULEB128 encoding of 1
}

UnwindPlanSP ABISysV_mos::CreateFunctionEntryUnwindPlan() {
  // DWARF register numbers for S and PC come from MOSRegisterInfo.td via the
  // MCRegisterInfo generated at LLVM build time. MCBasedABI::GetEHAndDWARFNums
  // performs the lookup for us.
  if (!m_mc_register_info_up)
    return nullptr;

  uint32_t dwarf_s = GetEHAndDWARFNums("S").second;
  uint32_t dwarf_pc = GetEHAndDWARFNums("PC").second;
  if (dwarf_s == LLDB_INVALID_REGNUM || dwarf_pc == LLDB_INVALID_REGNUM) {
    LLDB_LOG(GetLog(LLDBLog::Expressions),
             "ABISysV_mos: Could not find DWARF numbers for S or PC");
    return nullptr;
  }

  // 6502 calling convention:
  // - JSR pushes (return_address - 1) onto the hardware stack, high byte
  //   first, decrementing S each time. On entry:
  //     [S+1] = low byte of (return_address - 1)
  //     [S+2] = high byte of (return_address - 1)
  //
  // We use normalized expressions to handle both 8-bit and 16-bit S values,
  // and to handle the case where S wraps within the stack page (the two
  // return-address bytes may be at physically non-consecutive addresses).
  // Each hardware stack byte's address is computed independently as
  // ((S + offset) & 0xFF) | 0x0100.

  // dwarf_s is a compile-time invariant of the MOS target; the expression
  // byte-vectors depend only on it, so build them exactly once. C++11
  // guarantees thread-safe static-local init.
  struct CachedExprs {
    std::vector<uint8_t> cfa;
    std::vector<uint8_t> pc;
    std::vector<uint8_t> s;
    uint32_t dwarf_s;
  };
  static const CachedExprs E = [this]() {
    uint32_t ds = GetEHAndDWARFNums("S").second;
    CachedExprs r;
    r.dwarf_s = ds;
    buildNormalizedHardwareStackExpr(r.cfa, ds, 3);
    buildReturnAddressValueExpr(r.pc, ds);
    buildNormalizedHardwareStackExpr(r.s, ds, 2);
    return r;
  }();

  assert(E.dwarf_s == dwarf_s &&
         "S DWARF number changed between calls; MOS MCRegisterInfo unstable?");

  auto plan_sp = std::make_shared<UnwindPlan>(eRegisterKindDWARF);
  plan_sp->SetSourceName("mos function-entry unwind plan");
  plan_sp->SetSourcedFromCompiler(eLazyBoolNo);
  plan_sp->SetUnwindPlanValidAtAllInstructions(eLazyBoolNo);
  plan_sp->SetUnwindPlanForSignalTrap(eLazyBoolNo);

  UnwindPlan::Row row;

  // LLDB asserts in GetFullUnwindPlanForFrame() if the first row does not
  // explicitly mark unmentioned registers as undefined.
  row.SetUnspecifiedRegistersAreUndefined(true);

  // CFA = normalized(S + 3) = ((S + 3) & 0xFF) | 0x0100
  row.GetCFAValue().SetIsDWARFExpression(E.cfa.data(), E.cfa.size());

  // PC value comes from reassembling the two return-address bytes and
  // adding 1 (JSR pushes PC-1). IsDWARFExpression means: the expression
  // result IS the register value.
  UnwindPlan::Row::AbstractRegisterLocation pc_loc;
  pc_loc.SetIsDWARFExpression(E.pc.data(), E.pc.size());
  row.SetRegisterInfo(dwarf_pc, pc_loc);

  // S_prev = normalized(S + 2) - direct expression, NOT CFA - 1.
  UnwindPlan::Row::AbstractRegisterLocation s_loc;
  s_loc.SetIsDWARFExpression(E.s.data(), E.s.size());
  row.SetRegisterInfo(dwarf_s, s_loc);

  plan_sp->AppendRow(std::move(row));
  plan_sp->SetReturnAddressRegister(dwarf_pc);

  return plan_sp;
}

UnwindPlanSP ABISysV_mos::CreateDefaultUnwindPlan() {
  // Heuristic fallback used when CFI is unavailable. The 6502 compiler freely
  // emits PHA/PLA pairs to spill values to the hardware stack mid-function; in
  // between such a pair, S no longer points at the return-address slot and the
  // function-entry plan yields garbage. This is the best available guess when
  // no CFI is present — accurate at function entry and in blocks with balanced
  // PHA/PLA usage, unreliable otherwise.
  return CreateFunctionEntryUnwindPlan();
}

//------------------------------------------------------------------
// Custom Register Context for imaginary registers
//------------------------------------------------------------------

lldb::RegisterContextSP
ABISysV_mos::CreateGDBRemoteRegisterContextForThread(
    lldb_private::Thread &thread, uint32_t concrete_frame_idx) const {

  // Downcast to ThreadGDBRemote - this is safe in the GDB remote context.
  // We use static_cast because ThreadGDBRemote doesn't have LLVM RTTI
  // (classof).
  auto *gdb_thread =
      static_cast<lldb_private::process_gdb_remote::ThreadGDBRemote *>(&thread);

  // Get the register info from the thread
  auto reg_info_sp = gdb_thread->GetRegisterInfoSP();
  if (!reg_info_sp) {
    LLDB_LOG(GetLog(LLDBLog::Expressions),
             "ABISysV_mos: No register info available");
    return nullptr;
  }

  // Get the imaginary registers helper for the main module
  ModuleSP main_module = GetMainModule();
  if (!main_module || !m_mc_register_info_up) {
    LLDB_LOG(GetLog(LLDBLog::Expressions),
             "ABISysV_mos: No main module or MCRegisterInfo, using base "
             "register context");
    return nullptr;
  }

  MOSImaginaryRegisters &imag_regs =
      MOSImaginaryRegisters::GetOrCreate(*main_module, *m_mc_register_info_up);

  return std::make_shared<MOSRegisterContext>(
      *gdb_thread, concrete_frame_idx, reg_info_sp,
      /*read_all_registers_at_once=*/false,
      /*write_all_registers_at_once=*/false, imag_regs);
}

//------------------------------------------------------------------
// Helper functions
//------------------------------------------------------------------

ModuleSP ABISysV_mos::GetMainModule() const {
  ProcessSP process_sp = GetProcessSP();
  if (!process_sp)
    return nullptr;

  TargetSP target_sp = process_sp->CalculateTarget();
  if (!target_sp)
    return nullptr;

  return target_sp->GetExecutableModule();
}
