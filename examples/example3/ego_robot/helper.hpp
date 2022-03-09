#include "ego_robot.h"

extern "C" PRT_VALUE* P_Init_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_MoveBackward_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_Stop_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_GetIsBumperReleasedLeft_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_GetIsBumperReleasedCenter_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_GetIsBumperReleasedRight_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_GetIsButtonPressedAndReleasedB0_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_GetIsCliffLeft_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_GetIsCliffCenter_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_GetIsCliffRight_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_TurnLeft_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_TurnRight_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_CheckIfReached_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_RotateTowardsLocation_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_MoveForward_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_Stay_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_RotateLeft_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_RotateRight_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_GetBatteryLevel_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_GetTrajectoryDeviation_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_RandomController_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_GetRobotPosition_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_GetChargerPosition_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_StepPID_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_GetOMPLMotionPlanAC_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);
extern "C" PRT_VALUE* P_GetOMPLMotionPlanSC_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_RegisterPotentialAvoidLocation_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_RegisterGeoFenceLocation_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_GetIsGeoFenceViolated_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_SetLed_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_IsTherePotentialAvoidLocation_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_FirstTourIsCompleted_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_ResetOdometry_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_IsThereAvoidLocationInSegment_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_InitMonitorGlobalLowerBound_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_UpdateMonitor_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_CheckMonitor_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);

extern "C" PRT_VALUE* P_NotifyController_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs);


