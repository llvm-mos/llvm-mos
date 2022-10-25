add_llvm_component_library(LLVMMOSDisassembler
  MOSDisassembler.cpp

  LINK_COMPONENTS
  MC
  MCDisassembler
  MOSDesc
  MOSInfo
  Support

  ADD_TO_COMPONENT
  MOS
)


