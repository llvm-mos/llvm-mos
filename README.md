# LLVM-MOS

LLVM-MOS is a LLVM fork supporting the MOS 65xx series of microprocessors.

For more information about this project, please see
[llvm-mos.org](https://www.llvm-mos.org).

## Notice

The llvm-mos project is not officially affiliated with or endorsed by the LLVM
Foundation or LLVM project. Our project is a fork of LLVM that provides a new
backend/target; our project is based on LLVM, not a part of LLVM. Our use of
LLVM or other related trademarks does not imply affiliation or endorsement.

This repository only contains the core llvm-mos utilities, and it doesn't form a
complete toolchain. Accordingly, there are no official binary releases for this
repository; it's for internal development only.

Please [see our SDK](https://github.com/llvm-mos/llvm-mos-sdk#getting-started)
to get started.

# Building LLVM-MOS

If you wish to modify the compiler, then you'll need to compile LLVM-MOS from
source.

Generally, compiling LLVM-MOS follows the same convention as compiling LLVM.
First, please review the
[hardware and software requirements](https://llvm.org/docs/GettingStarted.html#requirements)
for building LLVM.

Once you meet those requirements, you may use the following formula within your
build environment:

## Clone the LLVM-MOS repository

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

## Configure the LLVM-MOS project

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
  clang-tools-extra, lldb, or lld.

- ``-DCMAKE_INSTALL_PREFIX=directory`` --- Specify for *directory* the full
path name of where you want the LLVM tools and libraries to be installed
(default ``/usr/local``).

- ``-DCMAKE_BUILD_TYPE=type`` --- Valid options for *type* are Debug,
Release, RelWithDebInfo, and MinSizeRel. Default is MinSizeRel, if you
are using the MOS.cmake cache file.

- ``-DLLVM_ENABLE_ASSERTIONS=On`` --- Compile with assertion checks enabled
(default is Yes for Debug builds, No for all other build types).

## Build the LLVM-MOS project

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

All LLVM-MOS code should observe the
[LLVM coding standards](https://llvm.org/docs/CodingStandards.html).
`clang-format` and `clang-tidy` are aids for this; this repo contains appropriate
configuration files for them.

Code should be appropriately documented and well tested. We're not quite as picky
as the upstream LLVM project, but a compiler is too complex a project to thrive
without a high bar for code quality.

You submit issue requests via the
[issue tracker](https://github.com/llvm-mos/llvm-mos/issues).  Please note, we
don't have the bandwidth yet to handle "why dosent my pogrem compil" type
requests; it helps to do at least some legwork to figure out what's going on first.
Small reproducers are tremendously helpful, and for more finnicky issues, they're
essentially required.

Additionally, the current state of our documentation at
https://llvm-mos.org can always use improvements and clarifications.

