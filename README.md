# LLVM-MOS

This directory and its sub-directories contain source code for LLVM-MOS,
an experiment to support the MOS 65xx series of microprocessors as first-class
targets within the LLVM architecture.

**WARNING!** This compiler and toolchain has not been audited for C99
compatibility. Very little attention has been given to the quality of
code it generates beyond making sure that it's possible to clean up
later. Because of this, please don't publicly review, compare, or
benchmark this compiler against other compilers at this time. You'll
be surprised how much better the code will get once we turn our
full attention to it.

To keep this project a clean fork of LLVM, no target-specific source code or
libraries are part of this project. These are contained in the related
[llvm-mos-sdk](http://github.com/llvm-mos/llvm-mos-sdk). The default mos
target will only use compiler built-in include and library paths (e.g.,
stdint.h), so the compiler can technically be used without the SDK; however, 
this means that you will have to provide your own libc and your own
run-time initialization.  If you don't understand what this means, then you
should use llvm-mos in conjunction with the llvm-mos-sdk.

For more information about this project, please see
[llvm-mos.org](https://www.llvm-mos.org). 

For information about the current status of this project, please see 
[Current status](https://llvm-mos.org/wiki/Current_status).

To learn why this project exists, please see
[Rationale](https://llvm-mos.org/wiki/Rationale).

# Getting started

## Download the LLVM-MOS tools

If you want to play with the current state of the LLVM-MOS toolchain, you
may not have to build LLVM-MOS from source code yourself.  Instead, just download
the most recent binaries for your platform:

- [MacOS](https://github.com/llvm-mos/llvm-mos/releases/tag/llvm-mos-darwin-main)
- [Linux](https://github.com/llvm-mos/llvm-mos/releases/tag/llvm-mos-linux-main)
- [Windows](https://github.com/llvm-mos/llvm-mos/releases/tag/llvm-mos-windows-main)

These binaries are built from the main branch of the LLVM-MOS project,
using [Github's actions functionality](https://github.com/features/actions).

## Or, build the LLVM-MOS tools 

However, if you're allergic to precompiled binaries, or your platform is not
listed above, then you'll need to compile LLVM-MOS for your own platform.

Generally, compiling LLVM-MOS follows the same convention as compiling LLVM.
First, please review the [hardware and software requirements](https://llvm.org/docs/GettingStarted.html#requirements)
for building LLVM.

Once you meet those requirements, you may use the following formula within your 
build environment:

### Clone the LLVM-MOS repository

On Linux and MacOS:

```
git clone https://github.com/llvm-mos/llvm-mos.git
```

On Windows:

```
git clone --config core.autocrlf=false https://github.com/llvm-mos/llvm-mos.git
```

If you fail to use the --config flag as above, then verification tests will fail
on Windows.

### Configure the LLVM-MOS project

```
cd llvm-mos
cmake -C clang/cmake/caches/MOS.cmake [-G <generator>] -S llvm -B build [...]
```

This configuration command seeds the CMake cache with values from MOS.cmake.
Feel free to review and adjust these values for your environment.

Additional options can be added to the cmake command, which override the
values provided in MOS.cmake.  A handful are listed below.  For a complete list
of options, see [Building LLVM with CMake](https://llvm.org/docs/CMake.html).

- `-G <generator>` --- Lets you choose the CMake generator for your build 
environment.  CMake will try to automatically detect your build tools and
use them; however, it's recommended to install [Ninja](https://ninja-build.org/) 
and pass Ninja as the parameter to the -G command.

- ``-DLLVM_ENABLE_PROJECTS=...`` --- semicolon-separated list of the LLVM
sub-projects you'd like to additionally build. Can include any of: clang,
clang-tools-extra, libcxx, libcxxabi, libunwind, lldb, compiler-rt, lld,
polly, or debuginfo-tests.

- ``-DCMAKE_INSTALL_PREFIX=directory`` --- Specify for *directory* the full
path name of where you want the LLVM tools and libraries to be installed
(default ``/usr/local``).

- ``-DCMAKE_BUILD_TYPE=type`` --- Valid options for *type* are Debug,
Release, RelWithDebInfo, and MinSizeRel. Default is MinSizeRel, if you
are using the MOS.cmake cache file.

- ``-DLLVM_ENABLE_ASSERTIONS=On`` --- Compile with assertion checks enabled
(default is Yes for Debug builds, No for all other build types).

### Build the LLVM-MOS project

```
cmake --build build [-- [options] <target>]
```

The default target will build all of LLVM.  The `check-all` target will run the
regression tests.  The `distribution` target will build a collection of 
all the LLVM-MOS tools, suitable for redistribution.

CMake will generate targets for each tool and library, and most
LLVM sub-projects generate their own ``check-<project>`` target.

Running a serial build will be **slow**.  To improve speed, try running a
parallel build.  That's done by default in Ninja; for ``make``, use the option
``-j NNN``, where ``NNN`` is the number of parallel jobs, e.g. the number of
CPUs you have.

# Help us out

**We need your help!**  Please review the issue tracker, please review the 
current state of the code, and jump in and help us with pull requests for
bug fixes.

All LLVM-MOS code is expected to *strictly* observe the
[LLVM coding standards](https://llvm.org/docs/CodingStandards.html).  That means
your code must have been run through clang-format with the --style set to LLVM,
and clang-tidy with the LLVM coding conventions with the llvm-\*, modernize-\*,
and cppcore-\* checks enabled.  If your code does not observe these standards,
there's a good chance we'll reject it, unless you have a *good reason* for not
observing these rules.

If you add new functionality or an optimization pass to LLVM-MOS, we're 
not going to accept it unless you have modified the associated test suite to
exercise your new functionality.  Drive-by feature pulls will probably not be
accepted, unless their new functionality is too trivial to be tested.
GlobalISel gives you no excuses *not* to write a full test suite for your 
codegen pass or your new functionality.

You can submit well-written, carefully researched issue requests via the
[issue tracker](https://github.com/llvm-mos/llvm-mos/issues).  Please note, we
don't have the bandwidth yet to handle "why dosent my pogrem compil" type
requests.

Additionally, the current state of our documentation at
https://llvm-mos.org can always use improvements and clarifications.

