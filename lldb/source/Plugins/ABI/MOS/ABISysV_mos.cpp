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
#include "lldb/Target/Target.h"
#include "lldb/Target/Thread.h"
#include "lldb/Utility/LLDBLog.h"
#include "lldb/Utility/Log.h"

#include "Plugins/Process/gdb-remote/ThreadGDBRemote.h"

#include "llvm/BinaryFormat/Dwarf.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCDwarf.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCTargetOptions.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/LEB128.h"
#include "llvm/TargetParser/Triple.h"

#include <optional>

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

std::string ABISysV_mos::GetMCName(std::string reg) {
  // Remote stubs such as MAME's gdbstub call the hardware stack pointer "sp",
  // but LLVM names it S. Without this mapping, sp gets no DWARF number and
  // every CFI rule that reads the hardware stack (DW_OP_breg4) fails.
  if (llvm::StringRef(reg).equals_insensitive("sp"))
    return "s";
  return reg;
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

namespace {
/// One register rule from the compiler's CIE initial instructions.
struct InitialFrameRule {
  enum Kind { CFAIsExpr, RegIsExpr, RegAtExpr, RegIsSame } kind;
  uint32_t dwarf_reg = LLDB_INVALID_REGNUM;
  std::vector<uint8_t> expr;
};
} // namespace

/// Decode the CIE initial instructions that the MOS backend registers on its
/// MCAsmInfo (see createMOSMCAsmInfo in MOSMCTargetDesc.cpp). The 6502 stack
/// rules are written down once, in the compiler; this only understands the
/// DWARF framing around them. Returns std::nullopt on anything unexpected.
static std::optional<std::vector<InitialFrameRule>>
DecodeInitialFrameState(const llvm::MCAsmInfo &mai) {
  std::vector<InitialFrameRule> rules;
  for (const llvm::MCCFIInstruction &inst : mai.getInitialFrameState()) {
    if (inst.getOperation() == llvm::MCCFIInstruction::OpSameValue) {
      rules.push_back({InitialFrameRule::RegIsSame, inst.getRegister(), {}});
      continue;
    }
    if (inst.getOperation() != llvm::MCCFIInstruction::OpEscape)
      return std::nullopt;

    // MOSCFIExprHelpers.h encodes each expression-form rule as one escape:
    //   opcode [ULEB128 reg] ULEB128 len <len expression bytes>
    llvm::StringRef bytes = inst.getValues();
    const uint8_t *p = bytes.bytes_begin();
    const uint8_t *end = bytes.bytes_end();
    auto read_uleb = [&](uint64_t &value) {
      unsigned n = 0;
      const char *error = nullptr;
      value = llvm::decodeULEB128(p, &n, end, &error);
      p += n;
      return error == nullptr;
    };

    if (p == end)
      return std::nullopt;
    uint8_t opcode = *p++;
    InitialFrameRule rule;
    uint64_t reg = LLDB_INVALID_REGNUM;
    switch (opcode) {
    case llvm::dwarf::DW_CFA_def_cfa_expression:
      rule.kind = InitialFrameRule::CFAIsExpr;
      break;
    case llvm::dwarf::DW_CFA_val_expression:
      rule.kind = InitialFrameRule::RegIsExpr;
      if (!read_uleb(reg))
        return std::nullopt;
      break;
    case llvm::dwarf::DW_CFA_expression:
      rule.kind = InitialFrameRule::RegAtExpr;
      if (!read_uleb(reg))
        return std::nullopt;
      break;
    default:
      return std::nullopt;
    }

    uint64_t len = 0;
    if (!read_uleb(len) || len != static_cast<uint64_t>(end - p))
      return std::nullopt;
    rule.dwarf_reg = static_cast<uint32_t>(reg);
    rule.expr.assign(p, end);
    rules.push_back(std::move(rule));
  }
  return rules;
}

UnwindPlanSP ABISysV_mos::CreateCIEUnwindPlan(bool at_function_entry) {
  if (!m_mc_register_info_up)
    return nullptr;
  const llvm::MCRegisterInfo &mri = *m_mc_register_info_up;
  Log *log = GetLog(LLDBLog::Unwind);

  // The rules are a property of the MOS target, not of any one process, so
  // decode them once. They are copied into static storage because UnwindPlan
  // rows hold raw pointers to expression bytes, and cached plans can outlive
  // both this ABI object and the MCAsmInfo used to produce them. C++11
  // guarantees thread-safe static-local init.
  static const std::optional<std::vector<InitialFrameRule>> rules =
      [&]() -> std::optional<std::vector<InitialFrameRule>> {
    llvm::Triple triple("mos");
    if (ProcessSP process_sp = GetProcessSP())
      triple = process_sp->GetTarget().GetArchitecture().GetTriple();
    std::string error;
    const llvm::Target *target =
        llvm::TargetRegistry::lookupTarget(triple, error);
    if (!target) {
      LLDB_LOG(log, "ABISysV_mos: no LLVM target for {0}: {1}", triple.str(),
               error);
      return std::nullopt;
    }
    std::unique_ptr<llvm::MCAsmInfo> mai(
        target->createMCAsmInfo(mri, triple, llvm::MCTargetOptions()));
    if (!mai)
      return std::nullopt;
    auto decoded = DecodeInitialFrameState(*mai);
    if (!decoded)
      LLDB_LOG(log, "ABISysV_mos: could not decode the MOS CIE initial "
                    "instructions; no function-entry unwind plan");
    return decoded;
  }();
  if (!rules)
    return nullptr;

  uint32_t dwarf_pc = mri.getDwarfRegNum(mri.getRARegister(), false);

  auto plan_sp = std::make_shared<UnwindPlan>(eRegisterKindDWARF);
  plan_sp->SetSourceName(at_function_entry ? "mos function-entry unwind plan"
                                           : "mos default unwind plan");
  plan_sp->SetSourcedFromCompiler(eLazyBoolNo);
  plan_sp->SetUnwindPlanValidAtAllInstructions(eLazyBoolNo);
  plan_sp->SetUnwindPlanForSignalTrap(eLazyBoolNo);

  UnwindPlan::Row row;

  // LLDB asserts in GetFullUnwindPlanForFrame() if the first row does not
  // explicitly mark unmentioned registers as undefined.
  row.SetUnspecifiedRegistersAreUndefined(true);

  bool have_cfa = false;
  bool have_pc = false;
  for (const InitialFrameRule &rule : *rules) {
    UnwindPlan::Row::AbstractRegisterLocation loc;
    switch (rule.kind) {
    case InitialFrameRule::CFAIsExpr:
      row.GetCFAValue().SetIsDWARFExpression(rule.expr.data(),
                                             rule.expr.size());
      have_cfa = true;
      continue;
    case InitialFrameRule::RegIsSame:
      // Only true before the function has run; see below.
      if (at_function_entry)
        row.SetRegisterLocationToSame(rule.dwarf_reg, /*must_replace=*/false);
      continue;
    case InitialFrameRule::RegIsExpr:
      loc.SetIsDWARFExpression(rule.expr.data(), rule.expr.size());
      break;
    case InitialFrameRule::RegAtExpr:
      loc.SetAtDWARFExpression(rule.expr.data(), rule.expr.size());
      break;
    }
    row.SetRegisterInfo(rule.dwarf_reg, loc);
    have_pc |= rule.dwarf_reg == dwarf_pc;
  }
  if (!have_cfa || !have_pc) {
    LLDB_LOG(log, "ABISysV_mos: the MOS CIE has no CFA or PC rule");
    return nullptr;
  }

  // At entry the callee has not run, so the registers it must preserve still
  // hold the caller's values. Past entry, a function without CFI may already
  // have moved RS0 or saved and reused a callee-saved register, so the
  // default plan leaves them undefined rather than claim values it cannot
  // know.
  if (at_function_entry)
    for (uint32_t dwarf : m_calling_conv.GetPreservedImaginaryRegisters())
      row.SetRegisterLocationToSame(dwarf, /*must_replace=*/false);

  plan_sp->AppendRow(std::move(row));
  plan_sp->SetReturnAddressRegister(dwarf_pc);

  return plan_sp;
}

UnwindPlanSP ABISysV_mos::CreateFunctionEntryUnwindPlan() {
  return CreateCIEUnwindPlan(/*at_function_entry=*/true);
}

UnwindPlanSP ABISysV_mos::CreateDefaultUnwindPlan() {
  // Heuristic fallback used when CFI is unavailable. The 6502 compiler freely
  // emits PHA/PLA pairs to spill values to the hardware stack mid-function; in
  // between such a pair, S no longer points at the return-address slot and the
  // function-entry plan yields garbage. This is the best available guess when
  // no CFI is present — accurate at function entry and in blocks with balanced
  // PHA/PLA usage, unreliable otherwise.
  return CreateCIEUnwindPlan(/*at_function_entry=*/false);
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
