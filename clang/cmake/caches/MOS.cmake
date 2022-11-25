# MOS.cmake
# Configure llvm-mos for building a distribution
# Usage for configuring:
#   cmake -C [path-to-this-file] ...

set(LLVM_TARGETS_TO_BUILD ""
    CACHE STRING "LLVM targets to build")
set(LLVM_EXPERIMENTAL_TARGETS_TO_BUILD "MOS"
    CACHE STRING "LLVM experimental targets to build")
set(LLVM_ENABLE_PROJECTS clang;clang-tools-extra;lld
    CACHE STRING "LLVM projects to enable")
set(LLVM_ENABLE_LIBXML2 "OFF" CACHE STRING "")
set(LLVM_ENABLE_ZSTD "OFF" CACHE STRING "")
set(LLVM_INSTALL_TOOLCHAIN_ONLY ON
    CACHE BOOL "LLVM install toolchain only")

set(LLVM_DEFAULT_TARGET_TRIPLE "mos-unknown-unknown" CACHE STRING "")

# The following options are principally to reduce space on Github action
# runner builds. They make smaller, and possibly slower, releases; but the
# releases are already over 1GB without them on most platforms, and the
# compilers don't seem to be slow on MOS-sized projects.  If you have
# more disk space, you may not need them.

set(CMAKE_BUILD_TYPE MinSizeRel
    CACHE STRING "CMake build type")
if(LLVM_MOS_BUILD_DYNAMIC_LIBRARIES)
    set(LLVM_BUILD_LLVM_DYLIB ON
        CACHE BOOL "Build LLVM dynamic libraries")
    set(LLVM_LINK_LLVM_DYLIB ON
        CACHE BOOL "Link LLVM dynamic libraries")
endif(LLVM_MOS_BUILD_DYNAMIC_LIBRARIES)

# disable lldb testing until the lldb tests stabilize
set(LLDB_INCLUDE_TESTS OFF
    CACHE BOOL "Include lldb tests")

# sccache or ccache may be used to speed up builds, if we can find it
# on the path
if(LLVM_MOS_USE_COMPILER_CACHE)
find_program(LLVM_MOS_CCACHE
    NAMES sccache ccache
    DOC "Path to compiler caching program")
if (NOT LLVM_MOS_CCACHE STREQUAL "LLVM_MOS_CCACHE-NOTFOUND")
    set(CMAKE_C_COMPILER_LAUNCHER ${LLVM_MOS_CCACHE}
        CACHE PATH "Path to C compiler caching program")
    set(CMAKE_CXX_COMPILER_LAUNCHER ${LLVM_MOS_CCACHE}
        CACHE PATH "Path to C++ compiler caching program")
    message(DEBUG "Compiler caching program found at \
        ${CMAKE_C_COMPILER_LAUNCHER}")
else()
    message(DEBUG "Compiler caching program not found")
endif()
endif() # LLVM_MOS_USE_COMPILER_CACHE

# Ship the release with these tools
set(LLVM_MOS_TOOLCHAIN_TOOLS
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
    llvm-symbolizer
)
set(LLVM_TOOLCHAIN_TOOLS ${LLVM_MOS_TOOLCHAIN_TOOLS}
    CACHE STRING "LLVM toolchain tools")

# Add clang symlinks prefixed with mos-* to allow distinguishing a llvm-mos
# directory from a system clang directory.
set(CLANG_LINKS_TO_CREATE
    clang++
    clang-cpp
    mos-clang
    mos-clang++
    mos-clang-cpp
)
set(CLANG_LINKS_TO_CREATE ${CLANG_LINKS_TO_CREATE}
    CACHE STRING "Clang symlinks to create during install.")

# Disable bindings since they can be problematic for the install pattern used along with llvm-mos-sdk.
set(LLVM_ENABLE_BINDINGS OFF CACHE BOOL "Build bindings.")
