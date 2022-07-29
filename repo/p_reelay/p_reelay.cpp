#include "p_reelay.h"

std::vector<reelay::monitor<reelay::json, reelay::json> > monitors;
std::vector<reelay::json> monitor_states;

PRT_VALUE* P_InitDiscreteReelayMonitor_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  char* spec = PrtPrimGetString(*argRefs[0]);
  auto options = reelay::discrete_timed<time_t>::monitor<reelay::json, reelay::json>::options().disable_condensing();
  reelay::monitor<reelay::json, reelay::json> monitor = reelay::make_monitor(spec, options);
  monitors.push_back(monitor);
  monitor_states.push_back(NULL);
  return PrtMkIntValue((PRT_UINT32)(monitors.size() - 1));
}

PRT_VALUE* P_InitDenseReelayMonitor_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  char* spec = PrtPrimGetString(*argRefs[0]);
  auto options = reelay::dense_timed<time_t>::monitor<reelay::json, reelay::json>::options();
  reelay::monitor<reelay::json, reelay::json> monitor = reelay::make_monitor(spec, options);
  monitors.push_back(monitor);
  monitor_states.push_back(NULL);
  return PrtMkIntValue((PRT_UINT32)(monitors.size() - 1));
}

PRT_VALUE* P_UpdateReelayMonitor_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  int index = PrtPrimGetInt(*argRefs[0]);
  char* message_str = PrtPrimGetString(*argRefs[1]);
  reelay::json message = reelay::json::parse(message_str);
  monitor_states[index] = monitors[index].update(message);
  bool temp = monitor_states[index]["value"];
  return PrtMkBoolValue((PRT_BOOLEAN)temp);
}

PRT_VALUE* P_CheckReelayMonitor_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  int index = PrtPrimGetInt(*argRefs[0]);
  if (monitor_states[index] == NULL) {
    return PrtMkBoolValue((PRT_BOOLEAN)false);
  }
  bool temp = monitor_states[index]["value"];
  return PrtMkBoolValue((PRT_BOOLEAN)temp);
}
