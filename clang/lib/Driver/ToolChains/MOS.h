//===-- MOS.h - MOS ToolChain -----------------------------------*- C++ -*-===//
//
// Part of the LLVM-MOS Project, under Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef CLANG_LIB_DRIVER_TOOLCHAINS_MOS_H_
#define CLANG_LIB_DRIVER_TOOLCHAINS_MOS_H_

#include "clang/Driver/Tool.h"
#include "clang/Driver/ToolChain.h"

#include "llvm/Support/Compiler.h"

namespace clang {
namespace driver {

namespace toolchains {

class LLVM_LIBRARY_VISIBILITY MOS : public ToolChain {
public:
  MOS(const Driver &D, const llvm::Triple &Triple,
      const llvm::opt::ArgList &Args);

protected:
  Tool *buildLinker() const override;

public:
  bool isPICDefault() const override { return false; }
  bool isPIEDefault(const llvm::opt::ArgList &Args) const override {
    return false;
  }
  bool isPICDefaultForced() const override { return true; }

  bool HasNativeLLVMSupport() const override { return true; }
  bool IsIntegratedAssemblerDefault() const override { return true; }

  const char *getDefaultLinker() const override { return "ld.lld"; }
  RuntimeLibType GetDefaultRuntimeLibType() const override {
    return ToolChain::RLT_CompilerRT;
  }

  bool SupportsProfiling() const override { return false; }

  void
  AddClangSystemIncludeArgs(const llvm::opt::ArgList &DriverArgs,
                            llvm::opt::ArgStringList &CC1Args) const override;
  void
  addClangTargetOptions(const llvm::opt::ArgList &DriverArgs,
                        llvm::opt::ArgStringList &CC1Args,
                        Action::OffloadKind DeviceOffloadKind) const override;
};

} // namespace toolchains

namespace tools {
namespace mos {

class LLVM_LIBRARY_VISIBILITY Linker : public Tool {
public:
  Linker(const ToolChain &TC) : Tool("mos::Linker", "ld.lld", TC) {}
  bool isLinkJob() const override { return true; }
  bool hasIntegratedCPP() const override { return false; }
  void ConstructJob(Compilation &C, const JobAction &JA,
                    const InputInfo &Output, const InputInfoList &Inputs,
                    const llvm::opt::ArgList &TCArgs,
                    const char *LinkingOutput) const override;

private:
  void AddLTOOptions(const toolchains::MOS &TC, const llvm::opt::ArgList &Args,
                     const InputInfo &Output, const InputInfoList &Inputs,
                     llvm::opt::ArgStringList &CmdArgs) const;
};

} // namespace mos
} // namespace tools

} // namespace driver
} // namespace clang

#endif // not CLANG_LIB_DRIVER_TOOLCHAINS_MOS_H_
