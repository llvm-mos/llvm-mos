//===-- ABIMOSTest.cpp ----------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Plugins/ABI/MOS/ABISysV_mos.h"
#include "lldb/Symbol/UnwindPlan.h"
#include "lldb/Utility/ArchSpec.h"
#include "llvm/Support/ManagedStatic.h"
#include "llvm/Support/TargetSelect.h"
#include "gtest/gtest.h"

using namespace lldb_private;
using namespace lldb;

namespace {
// DWARF register numbers from MOSRegisterInfo.td.
constexpr uint32_t dwarf_s = 4;
constexpr uint32_t dwarf_pc = 5;
constexpr uint32_t dwarf_rc0 = 0x20000;
constexpr uint32_t dwarf_rs0 = 0x30000;

class ABIMOSTest : public testing::Test {
public:
  static void SetUpTestCase() {
    LLVMInitializeMOSTargetInfo();
    LLVMInitializeMOSTargetMC();
    ABISysV_mos::Initialize();
  }
  static void TearDownTestCase() {
    ABISysV_mos::Terminate();
    llvm::llvm_shutdown();
  }
};

using RegLoc = UnwindPlan::Row::AbstractRegisterLocation;

const UnwindPlan::Row &GetOnlyRow(const UnwindPlanSP &plan_sp) {
  EXPECT_EQ(plan_sp->GetRowCount(), 1);
  return *plan_sp->GetRowAtIndex(0);
}

// The plans mark unmentioned registers undefined.
bool IsUndefined(const UnwindPlan::Row &row, uint32_t reg) {
  RegLoc loc;
  return row.GetRegisterInfo(reg, loc) && loc.IsUndefined();
}
} // namespace

// The function-entry plan is decoded from the CIE that the MOS backend
// registers on its MCAsmInfo. If the backend changes the CIE in a way the
// decoder does not understand, the plan disappears; this catches that.
TEST_F(ABIMOSTest, FunctionEntryPlanFromCIE) {
  ABISP abi_sp = ABI::FindPlugin(ProcessSP(), ArchSpec("mos"));
  ASSERT_TRUE(abi_sp);
  UnwindPlanSP plan_sp = abi_sp->CreateFunctionEntryUnwindPlan();
  ASSERT_TRUE(plan_sp);
  EXPECT_EQ(plan_sp->GetReturnAddressRegister(), dwarf_pc);
  const UnwindPlan::Row &row = GetOnlyRow(plan_sp);

  EXPECT_TRUE(row.GetCFAValue().IsDWARFExpression());
  RegLoc loc;
  ASSERT_TRUE(row.GetRegisterInfo(dwarf_pc, loc));
  EXPECT_TRUE(loc.IsDWARFExpression());
  ASSERT_TRUE(row.GetRegisterInfo(dwarf_s, loc));
  EXPECT_TRUE(loc.IsDWARFExpression());

  // At entry, RS0 and the callee-saved registers (MOS_CSR) still hold the
  // caller's values.
  for (uint32_t reg : {dwarf_rs0, dwarf_rs0 + 10, dwarf_rs0 + 15,
                       dwarf_rc0 + 20, dwarf_rc0 + 31}) {
    ASSERT_TRUE(row.GetRegisterInfo(reg, loc)) << reg;
    EXPECT_TRUE(loc.IsSame()) << reg;
  }
  // Call-clobbered registers are undefined.
  for (uint32_t reg :
       {dwarf_rs0 + 1, dwarf_rc0 + 2, dwarf_rc0 + 19, dwarf_rc0 + 32})
    EXPECT_TRUE(IsUndefined(row, reg)) << reg;
}

// The default plan stands in for functions without CFI at any point in their
// body, where RS0 and the callee-saved registers may already have changed.
TEST_F(ABIMOSTest, DefaultPlanClaimsNoSameRegisters) {
  ABISP abi_sp = ABI::FindPlugin(ProcessSP(), ArchSpec("mos"));
  ASSERT_TRUE(abi_sp);
  UnwindPlanSP plan_sp = abi_sp->CreateDefaultUnwindPlan();
  ASSERT_TRUE(plan_sp);
  const UnwindPlan::Row &row = GetOnlyRow(plan_sp);

  EXPECT_TRUE(row.GetCFAValue().IsDWARFExpression());
  RegLoc loc;
  ASSERT_TRUE(row.GetRegisterInfo(dwarf_pc, loc));
  EXPECT_TRUE(loc.IsDWARFExpression());
  ASSERT_TRUE(row.GetRegisterInfo(dwarf_s, loc));
  EXPECT_TRUE(loc.IsDWARFExpression());

  for (uint32_t reg : {dwarf_rs0, dwarf_rs0 + 10, dwarf_rc0 + 20})
    EXPECT_TRUE(IsUndefined(row, reg)) << reg;
}
