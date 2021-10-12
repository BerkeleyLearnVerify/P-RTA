#include "helper.h"

#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdlib.h>

// time in [ms] of a simulation step
#define TIME_STEP 16  
#define MAX_SPEED 6.28

WbDeviceTag right_motor, left_motor, ds0, ds1;

PRT_VALUE* P_Init_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {

  wb_robot_init();

  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(right_motor, INFINITY);

  left_motor = wb_robot_get_device("left wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  
  ds0 = wb_robot_get_device("ds0");
  wb_distance_sensor_enable(ds0, TIME_STEP);
  ds1 = wb_robot_get_device("ds1");
  wb_distance_sensor_enable(ds1, TIME_STEP);

  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  wb_robot_step(TIME_STEP);

  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_MoveForward_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
  wb_robot_step(TIME_STEP);
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_Rotate_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
  wb_robot_step(TIME_STEP); 
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_GetDistanceSensorReadings_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  PRT_VALUE* value = (PRT_VALUE*)PrtMalloc(sizeof(PRT_VALUE));
  PRT_TYPE* seqType = PrtMkSeqType(PrtMkPrimitiveType(PRT_KIND_FLOAT));
  value = PrtMkDefaultValue(seqType);
  double ds0_value = wb_distance_sensor_get_value(ds0);
  double ds1_value = wb_distance_sensor_get_value(ds1);
  PrtSeqInsert(value, PrtMkIntValue(0), PrtMkFloatValue(ds0_value));
  PrtSeqInsert(value, PrtMkIntValue(1), PrtMkFloatValue(ds1_value));
  return value;
}
