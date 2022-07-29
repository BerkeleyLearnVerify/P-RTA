#include <vector>
#include "Prt.h"
#include "reelay/monitors.hpp"

extern "C" PRT_VALUE* P_InitDiscreteReelayMonitor_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_InitDenseReelayMonitor_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_UpdateReelayMonitor_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_CheckReelayMonitor_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
