//===-- ABISysV_mos.h -------------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLDB_SOURCE_PLUGINS_ABI_MOS_ABISYSV_MOS_H
#define LLDB_SOURCE_PLUGINS_ABI_MOS_ABISYSV_MOS_H

#include "lldb/Target/ABI.h"
#include "lldb/lldb-forward.h"

class ABISysV_mos : public lldb_private::MCBasedABI {
public:
  ~ABISysV_mos() override = default;

  // Plugin interface
  static void Initialize();
  static void Terminate();
  static lldb::ABISP CreateInstance(lldb::ProcessSP process_sp,
                                    const lldb_private::ArchSpec &arch);
  static llvm::StringRef GetPluginNameStatic() { return "sysv-mos"; }
  llvm::StringRef GetPluginName() override { return GetPluginNameStatic(); }

  // ABI interface
  size_t GetRedZoneSize() const override { return 0; }
  uint64_t GetStackFrameSize() override {
    return 256;
  } // 6502 stack is 256 bytes

  bool PrepareTrivialCall(lldb_private::Thread &thread, lldb::addr_t sp,
                          lldb::addr_t func_addr, lldb::addr_t return_addr,
                          llvm::ArrayRef<lldb::addr_t> args) const override;

  bool GetArgumentValues(lldb_private::Thread &thread,
                         lldb_private::ValueList &values) const override;

  lldb_private::Status
  SetReturnValueObject(lldb::StackFrameSP &frame_sp,
                       lldb::ValueObjectSP &new_value) override;

  bool CallFrameAddressIsValid(lldb::addr_t cfa) override {
    // MOS uses 16-bit addresses, soft stack can be anywhere
    return cfa <= 0xFFFF;
  }

  bool CodeAddressIsValid(lldb::addr_t pc) override {
    // MOS uses 16-bit addresses
    return pc <= 0xFFFF;
  }

  bool RegisterIsVolatile(const lldb_private::RegisterInfo *reg_info) override;

  lldb::UnwindPlanSP CreateDefaultUnwindPlan() override;
  lldb::UnwindPlanSP CreateFunctionEntryUnwindPlan() override;

  // MCBasedABI interface
  uint32_t GetGenericNum(llvm::StringRef name) override;

  // Register augmentation - adds imaginary registers
  void AugmentRegisterInfo(
      std::vector<lldb_private::DynamicRegisterInfo::Register> &regs) override;

  // Custom register context for imaginary register handling
  lldb::RegisterContextSP
  CreateRegisterContextForThread(lldb_private::Thread &thread,
                                 uint32_t concrete_frame_idx) const override;

protected:
  lldb::ValueObjectSP
  GetReturnValueObjectImpl(lldb_private::Thread &thread,
                           lldb_private::CompilerType &type) const override;

private:
  ABISysV_mos(lldb::ProcessSP process_sp,
              std::unique_ptr<llvm::MCRegisterInfo> info_up)
      : MCBasedABI(std::move(process_sp), std::move(info_up)) {}

  // Helper to get the main executable module
  lldb::ModuleSP GetMainModule() const;
};

#endif // LLDB_SOURCE_PLUGINS_ABI_MOS_ABISYSV_MOS_H
