# The LLVM-MOS Project

This directory and its sub-directories contain source code for LLVM-MOS,
a project to port the LLVM toolkit to the MOS 6502.

The README briefly describes how to get started with LLVM-MOS.

## Getting Started

Modified from https://llvm.org/docs/GettingStarted.html.

## Overview

Of the myriad LLVM projects, only Clang, LLD, and LLVM proper have received
attention. The MOS target is still considered experimental (and unfinished!),
so you need to pass `-DLLVM_EXPERIMENTAL_TARGETS_TO_BUILD="MOS"` to cmake in
order to actually build the new target. If you like, you can speed up the
build by disabling the other targets: `-DLLVM_TARGETS_TO_BUILD="".`

It's early days for Clang and the LLVM code generator, but the assembler and
linker (LLD) are both MVP complete. Both use ELF as their object format; the
target-specific ELF definitions were extended to accomodate the 6502.

No libraries or platform support are included with this repository to keep it
a clean fork of LLVM. These are contained in the related
[llvm-mos-sdk](http://github.com/llvm-mos/llvm-mos-sdk). The default mos
target will only use compiler built-in include and library paths (e.g.,
stdint.h), so the compiler can be used without the SDK. However, the various
platform subtargets (e.g. apple2e, c64, etc) used by the compiler driver will
require the SDK to be present. See the SDK for more information as it develops.

### Getting the Source Code and Building

The LLVM Getting Started documentation may be out of date.  The [Clang
Getting Started](http://clang.llvm.org/get_started.html) page might have more
accurate information.

This is an example work-flow and configuration to get and build the LLVM source:

1. Checkout LLVM (including related sub-projects like Clang):

     * ``git clone https://github.com/llvm-mos/llvm-mos.git``

     * Or, on windows, ``git clone --config core.autocrlf=false
    https://github.com/llvm-mos/llvm-mos.git``

2. Configure and build LLVM and Clang:

     * ``cd llvm-project``

     * ``cmake -S llvm -B build -G <generator> -DLLVM_EXPERIMENTAL_TARGETS_TO_BUILD="MOS" [options]``

        Some common build system generators are:

        * ``Ninja`` --- for generating [Ninja](https://ninja-build.org)
          build files. Most llvm developers use Ninja.
        * ``Unix Makefiles`` --- for generating make-compatible parallel makefiles.
        * ``Visual Studio`` --- for generating Visual Studio projects and
          solutions.
        * ``Xcode`` --- for generating Xcode projects.

        Some Common options:

        * ``-DLLVM_ENABLE_PROJECTS='...'`` --- semicolon-separated list of the LLVM
          sub-projects you'd like to additionally build. Can include any of: clang,
          clang-tools-extra, libcxx, libcxxabi, libunwind, lldb, compiler-rt, lld,
          polly, or debuginfo-tests.

          For example, to build LLVM, Clang, libcxx, and libcxxabi, use
          ``-DLLVM_ENABLE_PROJECTS="clang;libcxx;libcxxabi"``.

        * ``-DCMAKE_INSTALL_PREFIX=directory`` --- Specify for *directory* the full
          path name of where you want the LLVM tools and libraries to be installed
          (default ``/usr/local``).

        * ``-DCMAKE_BUILD_TYPE=type`` --- Valid options for *type* are Debug,
          Release, RelWithDebInfo, and MinSizeRel. Default is Debug.

        * ``-DLLVM_ENABLE_ASSERTIONS=On`` --- Compile with assertion checks enabled
          (default is Yes for Debug builds, No for all other build types).

      * ``cmake --build build [-- [options] <target>]`` or your build system specified above
        directly.

        * The default target (i.e. ``ninja`` or ``make``) will build all of LLVM.

        * The ``check-all`` target (i.e. ``ninja check-all``) will run the
          regression tests to ensure everything is in working order.

        * CMake will generate targets for each tool and library, and most
          LLVM sub-projects generate their own ``check-<project>`` target.

        * Running a serial build will be **slow**.  To improve speed, try running a
          parallel build.  That's done by default in Ninja; for ``make``, use the option
          ``-j NNN``, where ``NNN`` is the number of parallel jobs, e.g. the number of
          CPUs you have.

      * For more information see [CMake](https://llvm.org/docs/CMake.html)

Consult the
[Getting Started with LLVM](https://llvm.org/docs/GettingStarted.html#getting-started-with-llvm)
page for detailed information on configuring and compiling LLVM. You can visit
[Directory Layout](https://llvm.org/docs/GettingStarted.html#directory-layout)
to learn about the layout of the source code tree.
