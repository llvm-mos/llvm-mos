add_llvm_library(LLVMMOSDesc
  MOSAsmBackend.cpp
  MOSELFObjectWriter.cpp
  MOSELFStreamer.cpp
  MOSInstPrinter.cpp
  MOSMCAsmInfo.cpp
  MOSMCCodeEmitter.cpp
  MOSMCELFStreamer.cpp
  MOSMCExpr.cpp
  MOSMCTargetDesc.cpp
  MOSTargetStreamer.cpp
)

add_dependencies(LLVMMOSDesc MOSCommonTableGen)

