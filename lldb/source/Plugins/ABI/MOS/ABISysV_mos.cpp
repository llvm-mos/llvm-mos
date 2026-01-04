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
// Helper to look up DWARF register number by name via MCRegisterInfo.
// This is DRY - the actual DWARF numbers come from MOSRegisterInfo.td
// through the MCRegisterInfo generated at LLVM build time.
//------------------------------------------------------------------
static int GetDwarfRegNumByName(llvm::MCRegisterInfo *mc_info,
                                llvm::StringRef name) {
  if (!mc_info)
    return -1;

  // MCRegisterInfo stores names in uppercase
  std::string mc_name = name.upper();

  for (unsigned reg = 0; reg < mc_info->getNumRegs(); ++reg) {
    if (mc_info->getName(reg) == mc_name) {
      return mc_info->getDwarfRegNum(reg, /*isEH=*/false);
    }
  }
  return -1;
}

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
  // Nothing ever happens behind your back on MOS, so no volatile registers
  return false;
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

UnwindPlanSP ABISysV_mos::CreateFunctionEntryUnwindPlan() {
  // Get the DWARF register numbers by name via MCRegisterInfo.
  // These are looked up at runtime - no hardcoded constants.
  // The actual DWARF numbers come from MOSRegisterInfo.td.
  if (!m_mc_register_info_up)
    return nullptr;

  int dwarf_s = GetDwarfRegNumByName(m_mc_register_info_up.get(), "S");
  int dwarf_pc = GetDwarfRegNumByName(m_mc_register_info_up.get(), "PC");

  if (dwarf_s < 0 || dwarf_pc < 0) {
    LLDB_LOG(GetLog(LLDBLog::Expressions),
             "ABISysV_mos: Could not find DWARF numbers for S or PC");
    return nullptr;
  }

  // 6502 calling convention:
  // - JSR pushes (return_address - 1) onto hardware stack
  //   - First pushes high byte to [SP], then decrements SP
  //   - Then pushes low byte to [SP], then decrements SP
  // - After JSR, SP points to next free slot (two below pushed address)
  // - RTS increments SP, reads low byte, increments SP, reads high byte, adds 1
  //
  // Stack layout after JSR (S = current value):
  //   [S+1] = low byte of (return_address - 1)
  //   [S+2] = high byte of (return_address - 1)
  //
  // We use normalized expressions to handle both 8-bit and 16-bit S values.
  // Each hardware stack address is computed independently (not CFA-relative)
  // to avoid subtraction underflow when S wraps within the stack page.
  //
  // The expression computes: ((S + offset) & 0xFF) | 0x0100
  // This ensures the result is always in the hardware stack range
  // 0x0100-0x01FF.

  // IMPORTANT: These expression vectors must be static because UnwindPlan
  // stores raw pointers to the data, not copies. They must persist for the
  // lifetime of the UnwindPlan.
  static std::vector<uint8_t> cfa_expr;
  static std::vector<uint8_t> pc_expr;
  static std::vector<uint8_t> s_expr;
  static bool initialized = false;
  static int cached_dwarf_s = -1;

  // Rebuild if not initialized or if DWARF number changed (shouldn't happen)
  if (!initialized || cached_dwarf_s != dwarf_s) {
    cfa_expr.clear();
    pc_expr.clear();
    s_expr.clear();

    buildNormalizedHardwareStackExpr(cfa_expr, dwarf_s, 3); // CFA = S + 3
    buildNormalizedHardwareStackExpr(pc_expr, dwarf_s, 1);  // PC at S + 1
    buildNormalizedHardwareStackExpr(s_expr, dwarf_s, 2);   // S_prev = S + 2

    cached_dwarf_s = dwarf_s;
    initialized = true;
  }

  auto plan_sp = std::make_shared<UnwindPlan>(eRegisterKindDWARF);
  plan_sp->SetSourceName("mos function-entry unwind plan");
  plan_sp->SetSourcedFromCompiler(eLazyBoolNo);
  plan_sp->SetUnwindPlanValidAtAllInstructions(eLazyBoolNo);
  plan_sp->SetUnwindPlanForSignalTrap(eLazyBoolNo);

  UnwindPlan::Row row;

  // CFA = normalized(S + 3) = ((S + 3) & 0xFF) | 0x0100
  row.GetCFAValue().SetIsDWARFExpression(cfa_expr.data(), cfa_expr.size());

  // PC = [normalized(S + 1)] - direct expression, NOT [CFA - 2]
  // atDWARFExpression means: dereference the address computed by the
  // expression.
  UnwindPlan::Row::AbstractRegisterLocation pc_loc;
  pc_loc.SetAtDWARFExpression(pc_expr.data(), pc_expr.size());
  row.SetRegisterInfo(dwarf_pc, pc_loc);

  // S_prev = normalized(S + 2) - direct expression, NOT CFA - 1
  // isDWARFExpression means: the expression result IS the register value.
  UnwindPlan::Row::AbstractRegisterLocation s_loc;
  s_loc.SetIsDWARFExpression(s_expr.data(), s_expr.size());
  row.SetRegisterInfo(dwarf_s, s_loc);

  plan_sp->AppendRow(std::move(row));
  plan_sp->SetReturnAddressRegister(dwarf_pc);

  return plan_sp;
}

UnwindPlanSP ABISysV_mos::CreateDefaultUnwindPlan() {
  // The default unwind plan is used when we're in the middle of a function.
  // For 6502, the return address is still on the hardware stack at S+1/S+2.
  return CreateFunctionEntryUnwindPlan();
}

//------------------------------------------------------------------
// Custom Register Context for imaginary registers
//------------------------------------------------------------------

lldb::RegisterContextSP
ABISysV_mos::CreateRegisterContextForThread(lldb_private::Thread &thread,
                                            uint32_t concrete_frame_idx) const {

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
