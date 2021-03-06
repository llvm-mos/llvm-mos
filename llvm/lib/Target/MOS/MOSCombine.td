def MOSCombinerHelper: GICombinerHelper<
  "MOSGenCombinerHelper", [all_combines]> {
  let DisableRuleOption = "moscombiner-disable-rule";
  let StateClass = "MOSCombinerHelperState";
  let AdditionalArguments = [];
}
