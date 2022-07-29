#include "slave.h"

extern "C" PRT_VALUE* P_Init_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_Rotate_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_MoveForward_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_GetDistanceSensorReadings_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
