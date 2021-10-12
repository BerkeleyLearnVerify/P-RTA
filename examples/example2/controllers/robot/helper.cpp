#include "helper.h"

#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/pen.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

// time in [ms] of a simulation step
#define TIME_STEP 16  
#define MAX_SPEED 6.28
#define R 0.025

WbDeviceTag right_motor, left_motor, gps, compass, ds0, ds1, pen;

double x_position = 0.0;
double y_position = 0.0;
double z_position = 0.0;
double orientation = 0.0;

double x_start = 0.0;
double y_start = 0.0;
double z_start = 0.0;

const int n_lines = 8;
const char* screen[n_lines] = {"Robot: ", "Motion Planner: ", "Motion Primitives: ", "Battery: ", "Geo-Fence1/Geo-Fence2: ", "Obstacle Avoidance: ", "Position: ", "Orientation: "};
char lines[n_lines][100];

PRT_VALUE* P_Init_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {

  srand(16);

  wb_robot_init();

  pen = wb_robot_get_device("pen");
  wb_pen_set_ink_color(pen, 0x000000, 0.0);

  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);

  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(right_motor, INFINITY);

  left_motor = wb_robot_get_device("left wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  
  ds0 = wb_robot_get_device("ds0");
  wb_distance_sensor_enable(ds0, TIME_STEP);
  ds1 = wb_robot_get_device("ds1");
  wb_distance_sensor_enable(ds1, TIME_STEP);

  wb_robot_battery_sensor_enable(TIME_STEP);

  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  wb_robot_step(TIME_STEP);

  const double *gps_values = wb_gps_get_values(gps);
  x_position = gps_values[0];
  y_position = gps_values[1];
  z_position = gps_values[2];

  const double *north = wb_compass_get_values(compass);
  orientation = atan2(-north[0], -north[2]);
  orientation = orientation < 0 ? orientation + 2*M_PI : orientation;

  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_CheckIfReached_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float x = PrtPrimGetFloat(*argRefs[0]);
  float z = PrtPrimGetFloat(*argRefs[1]);
  float speedMultiplier = PrtPrimGetFloat(*argRefs[2]);
  bool returnValue = (fabs(x - x_position) + fabs(z - z_position)) < speedMultiplier * 0.05;
  if (returnValue) {
    const double *north = wb_compass_get_values(compass);
    orientation = atan2(-north[0], -north[2]);
    orientation = orientation < 0 ? orientation + 2*M_PI : orientation;
    const double *gps_values = wb_gps_get_values(gps);
    x_position = gps_values[0];
    y_position = gps_values[1];
    z_position = gps_values[2];
    x_start = x_position;
    y_start = y_position;
    z_start = z_position;
  }
  return PrtMkBoolValue((PRT_BOOLEAN)returnValue);
}

PRT_VALUE* P_RotateTowardsLocation_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float x = PrtPrimGetFloat(*argRefs[0]);
  float z = PrtPrimGetFloat(*argRefs[1]);
  float rotationSpeedMultiplier = PrtPrimGetFloat(*argRefs[2]);
  bool isLowBattery = PrtPrimGetBool(*argRefs[3]);
  //if (isLowBattery) {
  //  wb_pen_set_ink_color(pen, 0xFFAA00, 1.0);
  //} else {
  //  wb_pen_set_ink_color(pen, 0xFF0000, 1.0);
  //}
  double angle2Goal = atan2((z_position - z), (x_position - x)) - M_PI;
  angle2Goal = angle2Goal < 0 ? angle2Goal + 2*M_PI : angle2Goal;
  double angleDifference = angle2Goal - orientation;
  while (angleDifference > M_PI) {
    angleDifference -= 2*M_PI;
  }
  while (angleDifference < -M_PI) {
    angleDifference += 2*M_PI;
  }
  bool direction = angleDifference > 0;
  if (fabs(angle2Goal - orientation) > 0.1 * rotationSpeedMultiplier) {
    if (direction) {
      wb_motor_set_velocity(left_motor, rotationSpeedMultiplier * MAX_SPEED);
      wb_motor_set_velocity(right_motor, -rotationSpeedMultiplier * MAX_SPEED);
      orientation += rotationSpeedMultiplier / 2 * MAX_SPEED * TIME_STEP / 1000.0;
    } else {
      wb_motor_set_velocity(left_motor, -rotationSpeedMultiplier * MAX_SPEED);
      wb_motor_set_velocity(right_motor, rotationSpeedMultiplier * MAX_SPEED);
      orientation -= rotationSpeedMultiplier / 2 * MAX_SPEED * TIME_STEP / 1000.0;
    }
    wb_robot_step(TIME_STEP);
    orientation = orientation < 0 ? orientation + 2*M_PI : orientation;
    orientation = orientation > 2*M_PI ? orientation - 2*M_PI : orientation;
    return PrtMkBoolValue((PRT_BOOLEAN)true);//Rotated
  }
  return PrtMkBoolValue((PRT_BOOLEAN)false);//No need to rotate
}

PRT_VALUE* P_MoveForward_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float speedMultiplier = PrtPrimGetFloat(*argRefs[0]);
  bool isLowBattery = PrtPrimGetBool(*argRefs[1]);
  bool isCollisionAvoidanceController = PrtPrimGetBool(*argRefs[2]);
  //if (isLowBattery) {
  //  if (isCollisionAvoidanceController) {
  //    wb_pen_set_ink_color(pen, 0xFFFFFF, 1.0);
  //  } else {
  //    wb_pen_set_ink_color(pen, 0xFFAA00, 1.0);
  //  }
  //} else {
  //  if (isCollisionAvoidanceController) {
  //    wb_pen_set_ink_color(pen, 0x222222, 1.0);
  //  } else {
  //    wb_pen_set_ink_color(pen, 0xFF0000, 1.0);
  //  }
  //}
  wb_motor_set_velocity(left_motor, speedMultiplier * MAX_SPEED);
  wb_motor_set_velocity(right_motor, speedMultiplier * MAX_SPEED);
  wb_robot_step(TIME_STEP);
  x_position += ((R * MAX_SPEED * speedMultiplier * TIME_STEP) / 1000.0)*cos(orientation);
  z_position += ((R * MAX_SPEED * speedMultiplier * TIME_STEP) / 1000.0)*sin(orientation);
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_Stay_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
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

PRT_VALUE* P_RotateLeft_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float rotationSpeedMultiplier = PrtPrimGetFloat(*argRefs[0]);
  bool isLowBattery = PrtPrimGetBool(*argRefs[1]);
  //if (isLowBattery) {
  //  wb_pen_set_ink_color(pen, 0xFFFFFF, 1.0);
  //} else {
  //  wb_pen_set_ink_color(pen, 0x222222, 1.0);
  //}
  wb_motor_set_velocity(left_motor, -rotationSpeedMultiplier * MAX_SPEED);
  wb_motor_set_velocity(right_motor, rotationSpeedMultiplier * MAX_SPEED);
  wb_robot_step(TIME_STEP); 
  orientation -= rotationSpeedMultiplier / 2 * MAX_SPEED * TIME_STEP / 1000.0;
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_RotateRight_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float rotationSpeedMultiplier = PrtPrimGetFloat(*argRefs[0]);
  bool isLowBattery = PrtPrimGetBool(*argRefs[1]);
  //if (isLowBattery) {
  //  wb_pen_set_ink_color(pen, 0xFFFFFF, 1.0);
  //} else {
  //  wb_pen_set_ink_color(pen, 0x222222, 1.0);
  //}
  wb_motor_set_velocity(left_motor, rotationSpeedMultiplier * MAX_SPEED);
  wb_motor_set_velocity(right_motor, -rotationSpeedMultiplier * MAX_SPEED);
  wb_robot_step(TIME_STEP); 
  orientation += rotationSpeedMultiplier / 2 * MAX_SPEED * TIME_STEP / 1000.0;
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_GetBatteryLevel_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  return PrtMkFloatValue((PRT_FLOAT)wb_robot_battery_sensor_get_value());
}

PRT_VALUE* P_GetEstimatedPositionDeviation_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  const double *gps_values = wb_gps_get_values(gps);
  float x_deviation = fabs(gps_values[0] - x_position);
  float z_deviation = fabs(gps_values[2] - z_position);
  float deviation = sqrt(pow(x_deviation, 2) + pow(z_deviation, 2));
  return PrtMkFloatValue((PRT_FLOAT)deviation);
}

PRT_VALUE* P_UpdateEstimatedPositionWithGPS_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  const double *gps_values = wb_gps_get_values(gps);
  x_position = gps_values[0];
  y_position = gps_values[1];
  z_position = gps_values[2];
  return PrtMkFloatValue((PRT_FLOAT)wb_robot_battery_sensor_get_value());
}

PRT_VALUE* P_GetEstimatedOrientationDeviation_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  const double *north = wb_compass_get_values(compass);
  double trueOrientation = atan2(-north[0], -north[2]);
  trueOrientation = trueOrientation < 0 ? trueOrientation + 2*M_PI : trueOrientation;
  float orientationDeviation = fabs(trueOrientation - orientation);
  return PrtMkFloatValue((PRT_FLOAT)orientationDeviation);
}

PRT_VALUE* P_UpdateEstimatedOrientationWithCompass_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  const double *north = wb_compass_get_values(compass);
  orientation = atan2(-north[0], -north[2]);
  orientation = orientation < 0 ? orientation + 2*M_PI : orientation;
  return PrtMkFloatValue((PRT_FLOAT)wb_robot_battery_sensor_get_value());
}

PRT_VALUE* P_IsInTrajectory_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float x_0 = PrtPrimGetFloat(*argRefs[0]);
  float z_0 = PrtPrimGetFloat(*argRefs[1]);
  float x_1 = PrtPrimGetFloat(*argRefs[2]);
  float z_1 = PrtPrimGetFloat(*argRefs[3]);
  float trajectoryDeviationThreshold = PrtPrimGetFloat(*argRefs[4]);
  double angle2Goal = atan2((z_position - z_1), (x_position - x_1)) - M_PI;
  angle2Goal = angle2Goal < 0 ? angle2Goal + 2*M_PI : angle2Goal;
  double theta = angle2Goal - orientation;
  while (theta > M_PI) {
    theta -= 2*M_PI;
  }
  while (theta < -M_PI) {
    theta += 2*M_PI;
  }
  theta = fabs(theta);
  if (theta > M_PI/2) {
    return PrtMkBoolValue((PRT_BOOLEAN)false);
  }
  float trajectoryDeviationDistance = sqrt(pow(x_position - x_0, 2) + pow(z_position - z_0, 2));
  float trajectoryDeviation = fabs(trajectoryDeviationDistance * sin(theta));
  if (trajectoryDeviation >= trajectoryDeviationThreshold) {
    return PrtMkBoolValue((PRT_BOOLEAN)false);
  }
  return PrtMkBoolValue((PRT_BOOLEAN)true);
}

PRT_VALUE* P_RandomController_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  bool isLowBattery = PrtPrimGetBool(*argRefs[0]);
  //if (isLowBattery) {
  //  wb_pen_set_ink_color(pen, 0x8300FF, 1.0);
  //} else {
  //  wb_pen_set_ink_color(pen, 0x0000FF, 1.0);
  //}
  double speedMultiplier = 1.0;
  double random_selector = (double)rand() / (double)RAND_MAX;
  if (random_selector < 0.2) {
    wb_motor_set_velocity(left_motor, -speedMultiplier * MAX_SPEED);
    wb_motor_set_velocity(right_motor, speedMultiplier * MAX_SPEED);
    wb_robot_step(TIME_STEP); 
    orientation -= speedMultiplier / 2 * MAX_SPEED * TIME_STEP / 1000.0;
  } else {
    wb_motor_set_velocity(left_motor, speedMultiplier * MAX_SPEED);
    wb_motor_set_velocity(right_motor, speedMultiplier * MAX_SPEED);
    wb_robot_step(TIME_STEP);
    x_position += ((R * MAX_SPEED * speedMultiplier * TIME_STEP) / 1000.0)*cos(orientation);
    z_position += ((R * MAX_SPEED * speedMultiplier * TIME_STEP) / 1000.0)*sin(orientation);
  }
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_Print_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  char* str = PrtPrimGetString(*argRefs[0]);
  int line = PrtPrimGetInt(*argRefs[1]);
  strncpy(lines[line], str, 100);
  std::cout << "\x1b[2J";
  for (int i = 0; i < n_lines; i++) {
    if (lines[i][0] == 85) {
      std::cout << screen[i] << "\x1b[32m" << lines[i] << "\x1b[0m" << std::endl;
    } else {
      std::cout << screen[i] << "\x1b[31m" << lines[i] << "\x1b[0m" << std::endl;
    }
  }
  return PrtMkIntValue((PRT_UINT32)1);
}

