if ("AArch64" IN_LIST LLVM_TARGETS_TO_BUILD)
  add_subdirectory(AArch64)
endif()

if ("RISCV" IN_LIST LLVM_TARGETS_TO_BUILD)
  add_subdirectory(RISCV)
endif()

if ("MOS" IN_LIST LLVM_TARGETS_TO_BUILD)
  add_subdirectory(MOS)
endif()
