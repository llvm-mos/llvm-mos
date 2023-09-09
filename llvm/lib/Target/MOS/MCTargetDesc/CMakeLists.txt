add_llvm_component_library(LLVMMOSDesc
  MOSAsmBackend.cpp
  MOSELFObjectWriter.cpp
  MOSFixupKinds.cpp
  MOSInstPrinter.cpp
  MOSMCAsmInfo.cpp
  MOSMCCodeEmitter.cpp
  MOSMCELFStreamer.cpp
  MOSMCExpr.cpp
  MOSMCInstrAnalysis.cpp
  MOSMCTargetDesc.cpp
  MOSTargetStreamer.cpp

  LINK_COMPONENTS
  MC
  MOSInfo
  Support

  ADD_TO_COMPONENT
  MOS
)

add_dependencies(LLVMMOSDesc MOSCommonTableGen)
