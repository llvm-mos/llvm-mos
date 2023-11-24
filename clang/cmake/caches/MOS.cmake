# MOS.cmake
# Configure llvm-mos for building a distribution
# Usage for configuring:
#   cmake -C [path-to-this-file] ...

set(LLVM_TARGETS_TO_BUILD "" CACHE STRING "")
set(LLVM_EXPERIMENTAL_TARGETS_TO_BUILD "MOS" CACHE STRING "")
set(LLVM_ENABLE_PROJECTS clang;clang-tools-extra;lld CACHE STRING "")
set(LLVM_ENABLE_LIBXML2 "OFF" CACHE STRING "")
set(LLVM_ENABLE_ZLIB "OFF" CACHE STRING "")
set(LLVM_ENABLE_ZSTD "OFF" CACHE STRING "")

set(LLVM_ENABLE_RUNTIMES compiler-rt CACHE STRING "")

set(LLVM_BUILTIN_TARGETS mos-unknown-unknown CACHE STRING "")
set(LLVM_RUNTIME_TARGETS mos-unknown-unknown CACHE STRING "")
set(BUILTINS_mos-unknown-unknown_COMPILER_RT_BAREMETAL_BUILD ON CACHE BOOL "")
set(BUILTINS_mos-unknown-unknown_COMPILER_RT_BUILTINS_ENABLE_PIC OFF CACHE BOOL "")
set(BUILTINS_mos-unknown-unknown_CMAKE_BUILD_TYPE MinSizeRel CACHE BOOL "")
set(BUILTINS_mos-unknown-unknown_CMAKE_SYSTEM_NAME Generic CACHE STRING "")

set(LLVM_DEFAULT_TARGET_TRIPLE "mos-unknown-unknown" CACHE STRING "")

# The following option is principally to reduce space on Github action runner
# builds. They make smaller, and possibly slower, releases; but the releases are
# already over 1GB without them on most platforms, and the compilers don't seem
# to be slow on MOS-sized projects.  If you have more disk space, you may not
# need them.
set(CMAKE_BUILD_TYPE MinSizeRel CACHE STRING "CMake build type")

# disable lldb testing until the lldb tests stabilize
set(LLDB_INCLUDE_TESTS OFF CACHE BOOL "Include lldb tests")

# Ship the release with these tools
set(LLVM_INSTALL_TOOLCHAIN_ONLY ON CACHE BOOL "")
set(LLVM_TOOLCHAIN_TOOLS
  llvm-addr2line
  llvm-ar
  llvm-cxxfilt
  llvm-dwarfdump
  llvm-mc
  llvm-mlb
  llvm-nm
  llvm-objcopy
  llvm-objdump
  llvm-ranlib
  llvm-readelf
  llvm-readobj
  llvm-size
  llvm-strings
  llvm-strip
  llvm-symbolizer CACHE STRING "")

set(LLVM_DISTRIBUTION_COMPONENTS
  builtins
  clang
  lld
  clang-apply-replacements
  clang-format
  clang-resource-headers
  clang-include-fixer
  clang-refactor
  clang-scan-deps
  clang-tidy
  clangd
  find-all-symbols
  ${LLVM_TOOLCHAIN_TOOLS}
  CACHE STRING "")

# Add clang symlinks prefixed with mos-* to allow distinguishing a llvm-mos
# directory from a system clang directory.
set(CLANG_LINKS_TO_CREATE
  clang++
  clang-cpp
  mos-clang
  mos-clang++
  mos-clang-cpp)
set(CLANG_LINKS_TO_CREATE ${CLANG_LINKS_TO_CREATE}
  CACHE STRING "Clang symlinks to create during install.")

# Disable bindings since they can be problematic for the install pattern used along with llvm-mos-sdk.
set(LLVM_ENABLE_BINDINGS OFF CACHE BOOL "Build bindings.")
