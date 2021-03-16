//===-- MOS.h - MOS ToolChain -----------------------------------*- C++ -*-===//
//
// Part of the LLVM-MOS Project, under Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef CLANG_LIB_DRIVER_TOOLCHAINS_MOS_H_
#define CLANG_LIB_DRIVER_TOOLCHAINS_MOS_H_

#include "clang/Driver/ToolChain.h"

#include "llvm/Support/Compiler.h"

namespace clang {
namespace driver {

namespace toolchains {

class LLVM_LIBRARY_VISIBILITY MOS : public ToolChain {
public:
  MOS(const Driver &D, const llvm::Triple &Triple,
      const llvm::opt::ArgList &Args);

  bool isPICDefault() const override { return false; }
  bool isPIEDefault() const override { return false; }
  bool isPICDefaultForced() const override { return true; }

  bool HasNativeLLVMSupport() const override { return true; }
  bool IsIntegratedAssemblerDefault() const override { return true; }

  const char *getDefaultLinker() const override { return "ld.lld"; }
  RuntimeLibType GetDefaultRuntimeLibType() const override {
    return ToolChain::RLT_CompilerRT;
  }

  bool SupportsProfiling() const override { return false; }
};

} // namespace toolchains

} // namespace driver
} // namespace clang

#endif // not CLANG_LIB_DRIVER_TOOLCHAINS_MOS_H_
