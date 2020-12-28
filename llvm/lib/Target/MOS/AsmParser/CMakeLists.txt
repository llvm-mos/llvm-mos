add_llvm_component_library(LLVMMOSAsmParser
  MOSAsmParser.cpp

  LINK_COMPONENTS
  MC
  MCParser
  MOSDesc
  MOSInfo
  Support

  ADD_TO_COMPONENT
  MOS
)
