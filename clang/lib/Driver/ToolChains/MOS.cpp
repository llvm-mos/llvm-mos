//===-- MOS.cpp - MOS ToolChain -------------------------------------------===//
//
// Part of the LLVM-MOS Project, under Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MOS.h"

#include "CommonArgs.h"
#include "InputInfo.h"

#include "clang/Driver/Compilation.h"
#include "clang/Driver/Options.h"
#include "llvm/Support/Path.h"

using namespace llvm::opt;
using namespace clang::driver;
using namespace clang::driver::tools;
using namespace clang::driver::toolchains;

MOS::MOS(const Driver &D, const llvm::Triple &Triple,
         const llvm::opt::ArgList &Args)
    : ToolChain(D, Triple, Args) {
  // Look for binaries in both the installation and driver directory.
  getProgramPaths().push_back(getDriver().getInstalledDir());
  if (getDriver().getInstalledDir() != getDriver().Dir)
    getProgramPaths().push_back(getDriver().Dir);
}

Tool *MOS::buildLinker() const { return new tools::mos::Linker(*this); }

void mos::Linker::ConstructJob(Compilation &C, const JobAction &JA,
                               const InputInfo &Output,
                               const InputInfoList &Inputs, const ArgList &Args,
                               const char *LinkingOutput) const {
  ArgStringList CmdArgs;

  auto &TC = static_cast<const toolchains::MOS &>(getToolChain());
  auto &D = TC.getDriver();

  AddLinkerInputs(TC, Inputs, Args, CmdArgs, JA);

  CmdArgs.push_back("-Bstatic");

  if (!D.SysRoot.empty())
    CmdArgs.push_back(Args.MakeArgString("--sysroot=" + D.SysRoot));

  TC.AddFilePathLibArgs(Args, CmdArgs);
  Args.AddAllArgs(CmdArgs, {options::OPT_L, options::OPT_T_Group,
                            options::OPT_e, options::OPT_s, options::OPT_t,
                            options::OPT_Z_Flag, options::OPT_r});
  // Set default linker script.
  if (!Args.hasArg(options::OPT_T)) {
    SmallString<128> LinkerScript;
    // Vendor
    if (!TC.getPlatform().empty()) {
      llvm::sys::path::append(LinkerScript, TC.getPlatform());
    }

    // Note: The clang cross-compilation docs refer to this as "sys", which is
    // closer to how we're using it. Instead of the modern
    // one-platform-many-OSes of today, here we have many plaftorms, each with
    // basically one in-ROM OS. There's different ways of loading code into a
    // given OS, but those are closer to different ABIs than different OSes
    // themselves. Very high-level OS-es (Lunix, FreeRTOS, SpartaDOS, etc.)
    // enough to get their own OS definition here if an when they're officially
    // supported.
    if (!TC.getOS().empty()) {
      llvm::sys::path::append(LinkerScript, TC.getOS());
    }
    if (!TC.getTriple().getEnvironmentName().empty()) {
      llvm::sys::path::append(LinkerScript,
                              TC.getTriple().getEnvironmentName());
    }
    llvm::sys::path::append(LinkerScript, "include", "link.ld");
    CmdArgs.push_back(
        Args.MakeArgString(Twine("-T") + D.GetFilePath(LinkerScript, TC)));
  }

  if (TC.ShouldLinkCXXStdlib(Args))
    TC.AddCXXStdlibLibArgs(Args, CmdArgs);

  CmdArgs.push_back("-o");
  CmdArgs.push_back(Output.getFilename());

  C.addCommand(std::make_unique<Command>(JA, *this, ResponseFileSupport::None(),
                                         Args.MakeArgString(TC.GetLinkerPath()),
                                         CmdArgs, Inputs, Output));
}
