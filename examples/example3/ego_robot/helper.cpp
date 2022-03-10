/**                                                                           !
 * @file src/demos/simple_loop.cpp
 *
 * @brief Example program with a simple control loop.
 *
 * Controls the kobuki around a dead-reckoned square with sides of
 * length 0.25m.
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
 **/

/*****************************************************************************
 * Includes
 ****************************************************************************/

#include "helper.hpp"

#include "monitor.hpp"

#include <time.h>

#include <limits>
#include <string>

#include <csignal>
#include <ecl/geometry.hpp>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/command_line.hpp>
#include "kobuki_core/kobuki.hpp"
#include "kobuki_core/event_manager.hpp"

#include <cmath>
#include <set>

#include <fstream>
#include <iostream>
#include <vector>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/config.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/Cost.h>

#include<memory>

#include<utility>

#define R ((float) 0.2)

namespace ob = ompl::base;
namespace og = ompl::geometric;

// OMPL constants
static const double GRIDLOW = -1.5;
static const double GRIDHIGH = 1.5;

double x_position = 0.0;
double y_position = 0.0;
double z_position = 0.0;

double orientation = 0.0;

double x_start = 0.0;
double y_start = 0.0;
double z_start = 0.0;

double charger_x = 0.0;
double charger_y = 0.0;
double charger_z = 0.0;

int currentControllers[4] = {0, 0, 0, 0};

float prev_position_difference = 0.0;
float prev_angle_difference = 0.0;

float position_integral;
float angle_integral;

std::shared_ptr<ob::SE2StateSpace> space(std::make_shared<ob::SE2StateSpace>());
og::SimpleSetup ss(space);
og::PathGeometric waypoints(ss.getSpaceInformation());

std::set<std::pair<float, float> > avoidLocations;
std::set<std::pair<float, float> > geoFenceLocations;
std::set<std::pair<float, float> > potentialAvoidLocations;

float K_ap;
float K_av;
float K_ai;
float K_tp;
float K_tv;
float K_ti;
float BASE_SPEED;
float SPEED_SCALE;

/*****************************************************************************
** Classes
*****************************************************************************/

float getDistanceBetweenPairs(float p1X, float p1Z, float p2X, float p2Z) {
  return sqrt(pow(p1X - p2X, 2) + pow(p1Z - p2Z, 2));
}

bool isStateOutOfGeoFence(float x, float z) {
    for (auto it = geoFenceLocations.begin(); it != geoFenceLocations.end(); it++) {
      if (getDistanceBetweenPairs(x, z, (*it).first, (*it).second) <= 2 * R) {
        return false;
      }
    }
    return true;
}

class KobukiManager {
public:
  KobukiManager(
      const std::string & device,
      const double &length,
      const bool &disable_smoothing
  ) :
    dx(0.0), dth(0.0),
    length(length),
    is_button_pressed_b0(false),
    is_button_pressed_and_released_b0(false),
    slot_stream_data(&KobukiManager::processStreamData, *this),
    slot_bumper_event(&KobukiManager::processBumperEvent, *this),
    slot_button_event(&KobukiManager::processButtonEvent, *this),
    slot_cliff_event(&KobukiManager::processCliffEvent, *this),
    is_bumper_released_left(true), is_bumper_released_center(true), is_bumper_released_right(true), is_geofence_violated(false),
    is_cliff_left(false), is_cliff_center(false), is_cliff_right(false)
  {

    //openConnection();
    kobuki::Parameters parameters;
    parameters.sigslots_namespace = "/kobuki";
    parameters.device_port = device;
    parameters.enable_acceleration_limiter = !disable_smoothing;

    pose[0] = 0;
    pose[1] = 0;
    pose[2] = 0;

    kobuki.init(parameters);
    kobuki.enable();
    slot_stream_data.connect("/kobuki/stream_data");
    slot_bumper_event.connect("/kobuki/bumper_event");
    slot_button_event.connect("/kobuki/button_event");
    slot_cliff_event.connect("/kobuki/cliff_event");

    std::ifstream f;
    f.open ("pid_parameters.txt");
    f >> K_ap;
    f >> K_av;
    f >> K_ai;
    f >> K_tp;
    f >> K_tv;
    f >> K_ti;
    f >> BASE_SPEED;
    f >> SPEED_SCALE;
    f.close();

    kobuki.resetOdometry();

  }

  ~KobukiManager() {
    kobuki.setBaseControl(0,0); // linear_velocity, angular_velocity in (m/s), (rad/s)
    kobuki.disable();
  }

  void processStreamData() {
    // std::cout << "Before: " << x_position << " | " << z_position << " | " << orientation << std::endl;
    ecl::linear_algebra::Vector3d pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    kobuki.updateOdometry(pose_update, pose_update_rates);
    float c = std::cos(pose[2]);
    float s = std::sin(pose[2]);
    float tempX = pose[0] + c*pose_update[0] - s*pose_update[1];
    float tempZ = pose[1] + s*pose_update[0] + c*pose_update[1];
    float tempTheta = pose[2] + pose_update[2];
    while (tempTheta > 2*M_PI) {
      tempTheta -= 2*M_PI;
    }
    while (tempTheta < 0) {
      tempTheta += 2*M_PI;
    }
    pose[0] = tempX;
    pose[1] = tempZ;
    pose[2] = tempTheta;
    x_position = pose[0];
    z_position = pose[1];
    orientation = pose[2];
    if (isStateOutOfGeoFence(x_position, z_position)) {
      is_geofence_violated = false;
    } else {
      is_geofence_violated = true;
    }
  }

  const ecl::linear_algebra::Vector3d& getPose() {
    return pose;
  }

  void resetOdometry() {
    pose[0] = 0.0;
    pose[1] = 0.0;
    pose[2] = 0.0;
    kobuki.resetOdometry();
  }

  void processBumperEvent(const kobuki::BumperEvent &event) {
    if (event.state == kobuki::BumperEvent::Released) {
      switch (event.bumper) {
        case kobuki::BumperEvent::Left  : is_bumper_released_left   = true; break;
        case kobuki::BumperEvent::Center: is_bumper_released_center = true; break;
        case kobuki::BumperEvent::Right : is_bumper_released_right  = true; break;
      }
    } else if (event.state == kobuki::BumperEvent::Pressed) {
      kobuki.setBaseControl(-1.0, 0);
      switch (event.bumper) {
        case kobuki::BumperEvent::Left  : is_bumper_released_left   = false; break;
        case kobuki::BumperEvent::Center: is_bumper_released_center = false; break;
        case kobuki::BumperEvent::Right : is_bumper_released_right  = false; break;
      }
    }
    // std::cout << is_bumper_released_left << " | " << is_bumper_released_center << " | " << is_bumper_released_right << std::endl;
  }

  void processButtonEvent(const kobuki::ButtonEvent &event) {
    if (event.button == kobuki::ButtonEvent::Button0) {
      if (event.state == kobuki::ButtonEvent::Pressed) {
        is_button_pressed_b0 = true;
      } else if (event.state == kobuki::ButtonEvent::Released) {
        if (is_button_pressed_b0) {
          is_button_pressed_and_released_b0 = true;
        }
        is_button_pressed_b0 = false;
      }
    }
  }

  void processCliffEvent(const kobuki::CliffEvent &event) {
    if (event.state == kobuki::CliffEvent::Cliff) {
      switch (event.sensor) {
        case kobuki::CliffEvent::Left  : is_cliff_left   = true; break;
        case kobuki::CliffEvent::Center: is_cliff_center = true; break;
        case kobuki::CliffEvent::Right : is_cliff_right  = true; break;
      }
    } else if (event.state == kobuki::CliffEvent::Floor) {
      switch (event.sensor) {
        case kobuki::CliffEvent::Left  : is_cliff_left   = false; break;
        case kobuki::CliffEvent::Center: is_cliff_center = false; break;
        case kobuki::CliffEvent::Right : is_cliff_right  = false; break;
      }
    }
  }

  bool get_is_button_pressed_and_released_b0() {
    if (is_button_pressed_and_released_b0) {
      is_button_pressed_and_released_b0 = false;
      return true;
    }
    return false;
  }

  void setBaseControl(float positionalSpeed, float rotationalSpeed) {
    if (positionalSpeed < 0) {
      positionalSpeed = 0.0;
    } else if (positionalSpeed > 1.0) {
      positionalSpeed = 1.0;
    }
    if (rotationalSpeed < -2.0) {
      rotationalSpeed = -2.0;
    } else if (rotationalSpeed > 2.0) {
      rotationalSpeed = 2.0;
    }
    //std::cout << "AC" << std::endl;
    kobuki.setBaseControl(positionalSpeed, rotationalSpeed);
  }

  void moveForward(float speed) {
    if (speed < 0) {
      kobuki.setBaseControl(0.0, 0.0);
    } else if (speed > 1.0) {
      kobuki.setBaseControl(1.0, 0.0);
    } else {
      kobuki.setBaseControl(speed, 0.0);
    }
  }

  void moveBackward(float speed) {
    if (speed < 0) {
      kobuki.setBaseControl(0.0, 0.0);
    } else if (speed > 1.0) {
      kobuki.setBaseControl(-1.0, 0.0);
    } else {
      kobuki.setBaseControl(-speed, 0.0);
    }
  }

  void turnLeft(float speed) {
    if (speed < 0) {
      kobuki.setBaseControl(0.0, 0.0);
    } else if (speed > 1.0) {
      kobuki.setBaseControl(0.0, 1.0);
    } else {
      kobuki.setBaseControl(0.0, speed);
    }
  }

  void turnRight(float speed) {
    if (speed < 0) {
      kobuki.setBaseControl(0.0, 0.0);
    } else if (speed > 1.0) {
      kobuki.setBaseControl(0.0, -1.0);
    } else {
      kobuki.setBaseControl(0.0, -speed);
    }
  }

  void stop() {
    kobuki.setBaseControl(0.0, 0.0);
  }

  bool get_is_bumper_released_left() {
    return is_bumper_released_left;
  }

  bool get_is_bumper_released_center() {
    return is_bumper_released_center;
  }

  bool get_is_bumper_released_right() {
    return is_bumper_released_right;
  }

  bool get_is_geofence_violated() {
    return is_geofence_violated;
  }

  bool get_is_cliff_left() {
    return is_cliff_left;
  }

  bool get_is_cliff_center() {
    return is_cliff_center;
  }

  bool get_is_cliff_right() {
    return is_cliff_right;
  }

  // color:
  // 0 -> black
  // 1 -> red
  // 2 -> green
  // 3 -> orange
  void setLed(int led_num, int color) { 
    kobuki::LedNumber LED;
    kobuki::LedColour COLOR;
    switch (led_num) {
      case 0: LED = kobuki::LedNumber::Led1; break;
      case 1: LED = kobuki::LedNumber::Led2; break;
    }
    switch (color) {
      case 0: COLOR = kobuki::LedColour::Black;  break;
      case 1: COLOR = kobuki::LedColour::Red;    break;
      case 2: COLOR = kobuki::LedColour::Green;  break;
      case 3: COLOR = kobuki::LedColour::Orange; break;
    }
    kobuki.setLed(LED, COLOR);
  }
  
private:
  double dx, dth;
  const double length;
  ecl::linear_algebra::Vector3d pose;  // x, y, heading
  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;
  ecl::Slot<const kobuki::BumperEvent&> slot_bumper_event;
  ecl::Slot<const kobuki::ButtonEvent&> slot_button_event;
  ecl::Slot<const kobuki::CliffEvent&> slot_cliff_event;
  bool is_bumper_released_left;
  bool is_bumper_released_center;
  bool is_bumper_released_right;
  bool is_geofence_violated;
  bool is_button_pressed_b0;
  bool is_button_pressed_and_released_b0;
  bool is_cliff_left;
  bool is_cliff_center;
  bool is_cliff_right;
};

/*****************************************************************************
** Signal Handler
*****************************************************************************/

float getDistance(const ob::State *state, float tempX, float tempY) {
  const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
    
  // extract the first component of the state and cast it to what we expect
  const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

  // extract the second component of the state and cast it to what we expect
  /* const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1); */

  // check validity of state defined by pos & rot
  float x = pos->values[0];
  float y = pos->values[1];

  return sqrt(pow(x - tempX, 2) + pow(y - tempY, 2));

}

bool isStateValid(const ob::State *state) {
    for (auto it = avoidLocations.begin(); it != avoidLocations.end(); it++) {
      if (getDistance(state, (*it).first, (*it).second) <= R) {
        return false;
      }
    }
    for (auto it = geoFenceLocations.begin(); it != geoFenceLocations.end(); it++) {
      if (getDistance(state, (*it).first, (*it).second) <= 2.2 * R) {
        return false;
      }
    }
    return true;
}


bool shutdown_req = false;
void signalHandler(int /* signum */) {
  shutdown_req = true;
}

ecl::ValueArg<std::string> device_port("p", "port", "Path to device file of serial port to open", false, "/dev/kobuki", "string");
ecl::ValueArg<double> length("l", "length", "traverse square with sides of this size in length (m)", false, 0.25, "double");
ecl::SwitchArg disable_smoothing("d", "disable_smoothing", "Disable the acceleration limiter (smoothens velocity)", false);

KobukiManager kobuki_manager(device_port.getValue(), length.getValue(), disable_smoothing.getValue());

PRT_VALUE* P_Init_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  ecl::CmdLine cmd_line("Uses a simple control loop to move Kobuki around a dead-reckoned square with sides of length 1.0m", ' ', "0.2");
  cmd_line.add(device_port);
  cmd_line.add(length);
  cmd_line.add(disable_smoothing);
  signal(SIGINT, signalHandler);

  ob::RealVectorBounds bounds(2);

  bounds.setLow(GRIDLOW);
  bounds.setHigh(GRIDHIGH);
  space->setBounds(bounds);
  ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_MoveForward_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float positionalSpeed = PrtPrimGetFloat(*argRefs[0]);
  float rotationalSpeed = PrtPrimGetFloat(*argRefs[1]);
  kobuki_manager.setBaseControl(positionalSpeed, rotationalSpeed);
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_MoveBackward_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float speed = PrtPrimGetFloat(*argRefs[0]);
  kobuki_manager.moveBackward(speed);
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_RotateLeft_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float speed = PrtPrimGetFloat(*argRefs[0]);
  // std::cout << "RotateLeft" << std::endl;
  kobuki_manager.turnLeft(speed);
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_RotateRight_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float speed = PrtPrimGetFloat(*argRefs[0]);
  // std::cout << "RotateRight" << std::endl;
  kobuki_manager.turnRight(speed);
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_GetIsBumperReleasedLeft_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  bool returnValue = kobuki_manager.get_is_bumper_released_left();
  return PrtMkBoolValue((PRT_BOOLEAN)returnValue);
}

PRT_VALUE* P_GetIsBumperReleasedCenter_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  bool returnValue = kobuki_manager.get_is_bumper_released_center();
  return PrtMkBoolValue((PRT_BOOLEAN)returnValue);
}

PRT_VALUE* P_GetIsBumperReleasedRight_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  bool returnValue = kobuki_manager.get_is_bumper_released_right();
  return PrtMkBoolValue((PRT_BOOLEAN)returnValue);
}

PRT_VALUE* P_GetIsGeoFenceViolated_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  bool returnValue = kobuki_manager.get_is_geofence_violated();
  return PrtMkBoolValue((PRT_BOOLEAN)returnValue);
}

PRT_VALUE* P_GetIsButtonPressedAndReleasedB0_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  bool returnValue = kobuki_manager.get_is_button_pressed_and_released_b0();
  return PrtMkBoolValue((PRT_BOOLEAN)returnValue);
}

PRT_VALUE* P_GetIsCliffLeft_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  bool returnValue = kobuki_manager.get_is_cliff_left();
  return PrtMkBoolValue((PRT_BOOLEAN)returnValue);
}

PRT_VALUE* P_GetIsCliffCenter_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  bool returnValue = kobuki_manager.get_is_cliff_center();
  return PrtMkBoolValue((PRT_BOOLEAN)returnValue);
}

PRT_VALUE* P_GetIsCliffRight_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  bool returnValue = kobuki_manager.get_is_cliff_right();
  return PrtMkBoolValue((PRT_BOOLEAN)returnValue);
}

PRT_VALUE* P_GetRobotPosition_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  // std::cout << "Get Robot Position" << std::endl;
  PRT_VALUE* value = (PRT_VALUE*)PrtMalloc(sizeof(PRT_VALUE));
  PRT_TYPE* seqType = PrtMkSeqType(PrtMkPrimitiveType(PRT_KIND_FLOAT));
  value = PrtMkDefaultValue(seqType);
  PrtSeqInsert(value, PrtMkIntValue(0), PrtMkFloatValue(x_position));
  PrtSeqInsert(value, PrtMkIntValue(1), PrtMkFloatValue(z_position));
  return value;
}

PRT_VALUE* P_GetChargerPosition_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  PRT_VALUE* value = (PRT_VALUE*)PrtMalloc(sizeof(PRT_VALUE));
  PRT_TYPE* seqType = PrtMkSeqType(PrtMkPrimitiveType(PRT_KIND_FLOAT));
  value = PrtMkDefaultValue(seqType);
  PrtSeqInsert(value, PrtMkIntValue(0), PrtMkFloatValue(charger_x));
  PrtSeqInsert(value, PrtMkIntValue(1), PrtMkFloatValue(charger_z));
  return value;
}

PRT_VALUE* P_CheckIfReached_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float x = PrtPrimGetFloat(*argRefs[0]);
  float z = PrtPrimGetFloat(*argRefs[1]);
  float speedMultiplier = PrtPrimGetFloat(*argRefs[2]);
  bool returnValue = (fabs(x - x_position) + fabs(z - z_position)) < speedMultiplier * 0.025;
  //std::cout << x_position << " " << z_position << " " << x << " " << z << std::endl;
  if (returnValue) {
    x_start = x_position;
    y_start = y_position;
    z_start = z_position;
    position_integral = 0.0;
    angle_integral = 0.0;
  }
  return PrtMkBoolValue((PRT_BOOLEAN)returnValue);
}

PRT_VALUE* P_RotateTowardsLocation_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float x = PrtPrimGetFloat(*argRefs[0]);
  float z = PrtPrimGetFloat(*argRefs[1]);
  float speed = PrtPrimGetFloat(*argRefs[2]);
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
  //std::cout << angle2Goal << " " << orientation << " " << angleDifference << std::endl;
  if (fabs(angleDifference) > 0.05 * speed) {
    if (direction) {
      kobuki_manager.turnLeft(speed);
    } else {
      kobuki_manager.turnRight(speed);
    }
    return PrtMkBoolValue((PRT_BOOLEAN)true);//Rotated
  }
  return PrtMkBoolValue((PRT_BOOLEAN)false);//No need to rotate
}

PRT_VALUE* P_Stay_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  kobuki_manager.stop();
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_RegisterPotentialAvoidLocation_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  // Compute the avoidLocation
  float x_goal = PrtPrimGetFloat(*argRefs[0]);
  float z_goal = PrtPrimGetFloat(*argRefs[1]);
  float x, z;
  float angle;
  if (!kobuki_manager.get_is_bumper_released_left()) {
    angle = orientation + M_PI / 4;
  } else if (!kobuki_manager.get_is_bumper_released_right()) {
    angle = orientation - M_PI / 4;
  } else {
    angle = orientation;
  }
  x = x_position + cos(angle) * R;
  z = z_position + sin(angle) * R;
  std::pair<float, float> potentialAvoidLocation(x, z);
  // std::cout << "potentialAvoidLocation: " << potentialAvoidLocation.first << " " << potentialAvoidLocation.second << std::endl;
  for (auto it = potentialAvoidLocations.begin(); it != potentialAvoidLocations.end(); it++) {
    if (getDistanceBetweenPairs(potentialAvoidLocation.first, potentialAvoidLocation.second, (*it).first, (*it).second) <= 2 * R) {
      avoidLocations.insert(*it);
      avoidLocations.insert(potentialAvoidLocation);
    }
  }
  potentialAvoidLocations.insert(potentialAvoidLocation);
  for (auto it = avoidLocations.begin(); it != avoidLocations.end(); it++) {
    // std::cout << "avoidLocation: " << (*it).first << " " << (*it).second << std::endl;
    if (getDistanceBetweenPairs(x_goal, z_goal, (*it).first, (*it).second) <= 2 * R) {
      return PrtMkBoolValue((PRT_BOOLEAN)false);
    }
  }
  return PrtMkBoolValue((PRT_BOOLEAN)true);
}

PRT_VALUE* P_RegisterGeoFenceLocation_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  // Compute the avoidLocation
  float x_goal = PrtPrimGetFloat(*argRefs[0]);
  float z_goal = PrtPrimGetFloat(*argRefs[1]);
  std::pair<float, float> geoFenceLocation(x_goal, z_goal);
  geoFenceLocations.insert(geoFenceLocation);
  return PrtMkBoolValue((PRT_BOOLEAN)true);
}

PRT_VALUE* P_GetTrajectoryDeviation_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float x_goal = PrtPrimGetFloat(*argRefs[0]);
  float z_goal = PrtPrimGetFloat(*argRefs[1]);
  float trajectoryDeviation = fabs((x_goal - x_start) * (z_start - z_position) - (x_start - x_position) * (z_goal - z_start)) / sqrt(pow(x_goal - x_start, 2) + pow(z_goal - z_start, 2));
  return  PrtMkFloatValue((PRT_FLOAT)trajectoryDeviation);;
}

PRT_VALUE* P_RandomController_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float forwardSpeed = PrtPrimGetFloat(*argRefs[0]);
  float rotationSpeed = PrtPrimGetFloat(*argRefs[1]);
  double random_selector = (double)rand() / (double)RAND_MAX;
  if (random_selector < 0.2) {
    kobuki_manager.turnLeft(rotationSpeed);
  } else {
    kobuki_manager.moveForward(forwardSpeed);
  }
  return PrtMkIntValue((PRT_UINT32)1);
}

float normalizeAngle(float a1) {
    if (a1 > M_PI) {
      return a1 - 2*M_PI;
    } else if (a1 < -M_PI) {
      return a1 + 2*M_PI;
    } else {
      return a1;
    }
}

float angleDiff(float a1, float a2) {
    return normalizeAngle(normalizeAngle(a1) - normalizeAngle(a2));
}

PRT_VALUE* P_StepPID_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
    // One PID controller for deviation from the line between two points (to control torque)
    // Another PID controller for distance to the next goal (to control acceleration)

    float nextLocationX = PrtPrimGetFloat(*argRefs[0]);
    float nextLocationZ = PrtPrimGetFloat(*argRefs[1]);
    
    //std::cout << "Current position:" << x_position << " " << z_position << std::endl;
    
    //std::cout << "Current orientation:" << orientation << std::endl;
    
    //std::cout << "Goal position:" << nextLocationX << " " << nextLocationZ << std::endl;
    //std::cout << "Current position:" << x_position << " " << z_position << std::endl;
    
    float x_diff = nextLocationX - x_position;
    float z_diff = nextLocationZ - z_position;

    float target_orientation = -atan2(z_diff, x_diff);

    //std::cout << "Current orientation:" << orientation << std::endl;
    //std::cout << "Target orientation:" << target_orientation << std::endl;

    target_orientation = normalizeAngle(target_orientation);
    double angle2Goal = atan2((z_position - nextLocationZ), (x_position - nextLocationX)) - M_PI;
    angle2Goal = angle2Goal < 0 ? angle2Goal + 2*M_PI : angle2Goal;
    double angle_difference = angle2Goal - orientation;
    while (angle_difference > M_PI) {
      angle_difference -= 2*M_PI;
    }
    while (angle_difference < -M_PI) {
      angle_difference += 2*M_PI;
    }

    float position_difference = sqrt(pow(x_diff,2) + pow(z_diff,2));

    //std::cout << "Position Diff = " << position_difference << std::endl;
    //std::cout << "Angle Diff = " << angle_difference << std::endl;

    float position_velocity = position_difference - prev_position_difference;
    float angle_velocity = angle_difference - prev_angle_difference;

    position_integral += position_difference;
    angle_integral += angle_difference;

    float pos_out = BASE_SPEED + K_ap * position_difference + K_av * position_velocity + K_ai * position_integral;
    float angl_out = K_tp * angle_difference + K_tv * angle_velocity + K_ti * angle_integral;


    //std::cout << "BASE_SPEED: " << BASE_SPEED << " | position_difference: " << position_difference << " | position_velocity: " << position_velocity << " | position_integral: " << position_integral << " | pos_out = " << pos_out << std::endl;
    //std::cout << "SPEED_SCALE: " << SPEED_SCALE << " | angle_difference: " << angle_difference << " | angle_velocity: " << angle_velocity << " | angle_integral: " << angle_integral << " | angl_out = " << angl_out << std::endl;

    kobuki_manager.setBaseControl(pos_out * SPEED_SCALE, angl_out * SPEED_SCALE);

    prev_position_difference = position_difference;
    prev_angle_difference = angle_difference;
    return PrtMkIntValue((PRT_UINT32)1);
}

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

PRT_VALUE* P_GetOMPLMotionPlanAC_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {

    // std::cout << "GetOMPLMotionPlanAC" << std::endl;
    float currentLocationX = PrtPrimGetFloat(*argRefs[0]);
    float currentLocationZ = PrtPrimGetFloat(*argRefs[1]);
    float goalLocationX = PrtPrimGetFloat(*argRefs[2]);
    float goalLocationZ = PrtPrimGetFloat(*argRefs[3]);

    // set start state
    ob::ScopedState<> start(space);
    start->as<ob::SE2StateSpace::StateType>()->setX(currentLocationX);
    start->as<ob::SE2StateSpace::StateType>()->setY(currentLocationZ);
  
    // set goal state
    ob::ScopedState<> goal(space);
    goal->as<ob::SE2StateSpace::StateType>()->setX(goalLocationX);
    goal->as<ob::SE2StateSpace::StateType>()->setY(goalLocationZ);
  
    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);
    ss.setOptimizationObjective(getPathLengthObjective(ss.getSpaceInformation()));
    ob::PlannerPtr optimizingPlanner(new og::RRTstar(ss.getSpaceInformation()));
    ss.setPlanner(optimizingPlanner);
    // std::cout << "Planning towards: " << goalLocation << std::endl;

    // this call is optional, but we put it in to get more output information
    /* ss.setup(); */
    /* ss.print(); */
  
    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve();
    PRT_VALUE* value = (PRT_VALUE*)PrtMalloc(sizeof(PRT_VALUE));
    PRT_TYPE* seqType = PrtMkSeqType(PrtMkPrimitiveType(PRT_KIND_FLOAT));
    value = PrtMkDefaultValue(seqType);
    if (solved) {
        //std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        //ss.getSolutionPath().print(std::cout);
        waypoints = ss.getSolutionPath();
        std::vector<ob::State*> waypointsVector = waypoints.getStates();
        if (waypointsVector[waypointsVector.size() - 1]->as<ob::SE2StateSpace::StateType>()->getX() == goalLocationX &&
            waypointsVector[waypointsVector.size() - 1]->as<ob::SE2StateSpace::StateType>()->getY() == goalLocationZ) {
          for (unsigned int i = 1; i < waypointsVector.size(); i++) {
            PRT_TUPVALUE* tupPtr = (PRT_TUPVALUE*) PrtMalloc(sizeof(PRT_TUPVALUE));
            PRT_VALUE* value2 = (PRT_VALUE*)PrtMalloc(sizeof(PRT_VALUE));
            value2->discriminator = PRT_VALUE_KIND_TUPLE;
            value2->valueUnion.tuple = tupPtr;
            tupPtr->size = 2;
            tupPtr->values = (PRT_VALUE**)PrtCalloc(2, sizeof(PRT_VALUE));
            tupPtr->values[0] = PrtMkFloatValue(waypointsVector[i]->as<ob::SE2StateSpace::StateType>()->getX());
            tupPtr->values[1] = PrtMkFloatValue(waypointsVector[i]->as<ob::SE2StateSpace::StateType>()->getY());
            PrtSeqInsert(value, PrtMkIntValue(i - 1), value2);
          }
          ss.clear();
          return value; 
        }
    }
    ss.clear();
    std::cout << "No solution found" << std::endl;
    PRT_TUPVALUE* tupPtr = (PRT_TUPVALUE*) PrtMalloc(sizeof(PRT_TUPVALUE));
    PRT_VALUE* value2 = (PRT_VALUE*)PrtMalloc(sizeof(PRT_VALUE));
    value2->discriminator = PRT_VALUE_KIND_TUPLE;
    value2->valueUnion.tuple = tupPtr;
    tupPtr->size = 2;
    tupPtr->values = (PRT_VALUE**)PrtCalloc(2, sizeof(PRT_VALUE));
    tupPtr->values[0] = PrtMkFloatValue(goalLocationX);
    tupPtr->values[1] = PrtMkFloatValue(goalLocationZ);
    PrtSeqInsert(value, PrtMkIntValue(0), value2);
    value = PrtMkDefaultValue(seqType);
    return value;
 }

PRT_VALUE* P_SetLed_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
    int led_num = PrtPrimGetInt(*argRefs[0]);
    int led_color = PrtPrimGetInt(*argRefs[1]);
    kobuki_manager.setLed(led_num, led_color);
    return PrtMkIntValue((PRT_UINT32)1);
}

class ClearanceObjective : public ob::StateCostIntegralObjective {
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
    }
 
    ob::Cost stateCost(const ob::State* state) const
    {
      float minDistance = std::numeric_limits<float>::max();
      for (auto it = avoidLocations.begin(); it != avoidLocations.end(); it++) {
        minDistance = std::min(minDistance, getDistance(state, (*it).first, (*it).second));
      }
      for (auto it = geoFenceLocations.begin(); it != geoFenceLocations.end(); it++) {
        minDistance = std::min(minDistance, getDistance(state, (*it).first, (*it).second) - R);
      }
      return ob::Cost(1 / minDistance);
    }
};

ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));
 
    return lengthObj + clearObj;
}

PRT_VALUE* P_GetOMPLMotionPlanSC_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {

     // std::cout<< "GetOMPLMotionPlanSC" << std::endl;
    float currentLocationX = PrtPrimGetFloat(*argRefs[0]);
    float currentLocationZ = PrtPrimGetFloat(*argRefs[1]);
    float goalLocationX = PrtPrimGetFloat(*argRefs[2]);
    float goalLocationZ = PrtPrimGetFloat(*argRefs[3]);

    // set start state
    ob::ScopedState<> start(space);
    start->as<ob::SE2StateSpace::StateType>()->setX(currentLocationX);
    start->as<ob::SE2StateSpace::StateType>()->setY(currentLocationZ);
  
    // set goal state
    ob::ScopedState<> goal(space);
    goal->as<ob::SE2StateSpace::StateType>()->setX(goalLocationX);
    goal->as<ob::SE2StateSpace::StateType>()->setY(goalLocationZ);
  
    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);
    ss.setOptimizationObjective(getBalancedObjective(ss.getSpaceInformation()));
    ob::PlannerPtr optimizingPlanner(new og::RRTstar(ss.getSpaceInformation()));
    ss.setPlanner(optimizingPlanner);
    //std::cout << "Planning towards: " << goalLocation << std::endl;

    // this call is optional, but we put it in to get more output information
    /* ss.setup(); */
    /* ss.print(); */
  
    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(1.0);
    PRT_VALUE* value = (PRT_VALUE*)PrtMalloc(sizeof(PRT_VALUE));
    PRT_TYPE* seqType = PrtMkSeqType(PrtMkPrimitiveType(PRT_KIND_FLOAT));
    value = PrtMkDefaultValue(seqType);
    if (solved) {
        //std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        //ss.getSolutionPath().print(std::cout);
        waypoints = ss.getSolutionPath();
        std::vector<ob::State*> waypointsVector = waypoints.getStates();
        for (unsigned int i = 1; i < waypointsVector.size(); i++) {
          PRT_TUPVALUE* tupPtr = (PRT_TUPVALUE*) PrtMalloc(sizeof(PRT_TUPVALUE));
          PRT_VALUE* value2 = (PRT_VALUE*)PrtMalloc(sizeof(PRT_VALUE));
          value2->discriminator = PRT_VALUE_KIND_TUPLE;
          value2->valueUnion.tuple = tupPtr;            
          tupPtr->size = 2;
          tupPtr->values = (PRT_VALUE**)PrtCalloc(2, sizeof(PRT_VALUE));
          tupPtr->values[0] = PrtMkFloatValue(waypointsVector[i]->as<ob::SE2StateSpace::StateType>()->getX());
          tupPtr->values[1] = PrtMkFloatValue(waypointsVector[i]->as<ob::SE2StateSpace::StateType>()->getY());
          PrtSeqInsert(value, PrtMkIntValue(i - 1), value2);
        }
        ss.clear();
        return value;
    }
    ss.clear();
    std::cout << "No solution found" << std::endl;
    PRT_TUPVALUE* tupPtr = (PRT_TUPVALUE*) PrtMalloc(sizeof(PRT_TUPVALUE));
    PRT_VALUE* value2 = (PRT_VALUE*)PrtMalloc(sizeof(PRT_VALUE));
    value2->discriminator = PRT_VALUE_KIND_TUPLE;
    value2->valueUnion.tuple = tupPtr;
    tupPtr->size = 2;
    tupPtr->values = (PRT_VALUE**)PrtCalloc(2, sizeof(PRT_VALUE));
    tupPtr->values[0] = PrtMkFloatValue(goalLocationX);
    tupPtr->values[1] = PrtMkFloatValue(goalLocationZ);
    PrtSeqInsert(value, PrtMkIntValue(0), value2);
    value = PrtMkDefaultValue(seqType);
    return value;
 }



PRT_VALUE* P_IsTherePotentialAvoidLocation_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float minDistance = std::numeric_limits<float>::max();
  for (auto it = potentialAvoidLocations.begin(); it != potentialAvoidLocations.end(); it++) {
    minDistance = std::min(minDistance, getDistanceBetweenPairs(x_position, z_position, (*it).first, (*it).second));
  }
  bool returnValue = minDistance <= 2 * R;
  return PrtMkBoolValue((PRT_BOOLEAN)returnValue);
}


PRT_VALUE* P_ResetOdometry_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  x_position = 0.0;
  y_position = 0.0;
  z_position = 0.0;
  orientation = 0.0;
  kobuki_manager.resetOdometry();
  return PrtMkIntValue((PRT_UINT32)1);
}

float getDistanceToLine(float x0, float z0, float x1, float z1, float x, float z) {
  return fabs((x1 - x0) * (z0 - z) - (x0 - x) * (z1 - z0)) / sqrt(pow(x1 - x0, 2) + pow(z1 - z0, 2));
}


PRT_VALUE* P_IsThereAvoidLocationInSegment_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  float x0 = PrtPrimGetFloat(*argRefs[0]);
  float z0 = PrtPrimGetFloat(*argRefs[1]);
  float x1 = PrtPrimGetFloat(*argRefs[2]);
  float z1 = PrtPrimGetFloat(*argRefs[3]);
  float minDistance = std::numeric_limits<float>::max();
  for (auto it = avoidLocations.begin(); it != avoidLocations.end(); it++) {
    minDistance = std::min(minDistance, getDistanceToLine(x0, z0, x1, z1, (*it).first, (*it).second));
  }
  for (auto it = geoFenceLocations.begin(); it != geoFenceLocations.end(); it++) {
    minDistance = std::min(minDistance, getDistanceToLine(x0, z0, x1, z1, (*it).first, (*it).second) - R);
  }
  bool returnValue = minDistance <= 1.2 * R;
  return PrtMkBoolValue((PRT_BOOLEAN)returnValue);
}


PRT_VALUE* P_InitMonitorGlobalLowerBound_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  int len = PrtPrimGetInt(*argRefs[0]);
  float threshold = PrtPrimGetFloat(*argRefs[1]);
  int returnValue = initMonitorLowerBound(len, threshold);
  return PrtMkIntValue((PRT_UINT32)returnValue);
}

PRT_VALUE* P_UpdateMonitor_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  int id = PrtPrimGetInt(*argRefs[0]);
  float value = PrtPrimGetFloat(*argRefs[1]);
  updateMonitor(id, value);
  // std::cout << "Logged to the monitor: " << value << std::endl;
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_CheckMonitor_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  int id = PrtPrimGetInt(*argRefs[0]);
  bool returnValue = checkMonitor(id);
  // std::cout << "Monitor returned: " << returnValue << std::endl;
  return PrtMkBoolValue((PRT_BOOLEAN)returnValue);
}

PRT_VALUE* P_PrintControllerExecution_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  int id = PrtPrimGetInt(*argRefs[0]);
  if (id == 0) {
    std::cout << "TC," << x_position << "," << z_position << std::endl;
  } else {
    std::cout << "UT," << x_position << "," << z_position << std::endl;
  }
  // std::cout << "Monitor returned: " << returnValue << std::endl;
  return PrtMkBoolValue((PRT_BOOLEAN)true);
}

PRT_VALUE* P_PrintTime_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  std::cout << "Time: " << std::time(0) << std::endl;
  // std::cout << "Monitor returned: " << returnValue << std::endl;
  return PrtMkBoolValue((PRT_BOOLEAN)true);
}

