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

#include <string>

#include <csignal>
#include <ecl/geometry.hpp>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/command_line.hpp>
#include "kobuki_core/kobuki.hpp"
#include "kobuki_core/event_manager.hpp"

/*****************************************************************************
** Classes
*****************************************************************************/

class KobukiManager {
public:
  KobukiManager(
      const std::string & device,
      const double &length,
      const bool &disable_smoothing
  ) :
    dx(0.0), dth(0.0),
    length(length),
    slot_stream_data(&KobukiManager::processStreamData, *this),
    slot_bumper_event(&KobukiManager::processBumperEvent, *this),
    is_bumper_released_left(true), is_bumper_released_right(true)
  {
    kobuki::Parameters parameters;
    parameters.sigslots_namespace = "/kobuki";
    parameters.device_port = device;
    parameters.enable_acceleration_limiter = !disable_smoothing;

    kobuki.init(parameters);
    kobuki.enable();
    slot_stream_data.connect("/kobuki/stream_data");
    slot_bumper_event.connect("/kobuki/bumper_event");
  }

  ~KobukiManager() {
    kobuki.setBaseControl(0,0); // linear_velocity, angular_velocity in (m/s), (rad/s)
    kobuki.disable();
  }

  void processStreamData() {
    ecl::linear_algebra::Vector3d pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    kobuki.updateOdometry(pose_update, pose_update_rates);
    ecl::concatenate_poses(pose, pose_update);
    dx += pose_update[0];   // x
    dth += pose_update[2];  // heading
  }

  const ecl::linear_algebra::Vector3d& getPose() {
    return pose;
  }

  void processBumperEvent(const kobuki::BumperEvent &event) {
    if (event.state == kobuki::BumperEvent::Released ) {
      is_bumper_released_left = true;
      is_bumper_released_right = true;
    } else if (event.state == kobuki::BumperEvent::Pressed) {
      if (event.bumper == kobuki::BumperEvent::Left) {
        is_bumper_released_left = false;
      } else {
        is_bumper_released_right = false;
      }
    }
  }

  void moveForward() {
    kobuki.setBaseControl(0.2, 0.0);
  }

  void moveBackward() {
    kobuki.setBaseControl(-0.2, 0.0);
  }

  void turnLeft() {
    kobuki.setBaseControl(0.0, 1.0);
  }

  void turnRight() {
    kobuki.setBaseControl(0.0, -1.0);
  }

  void stop() {
    kobuki.setBaseControl(0.0, 0.0);
  }

  bool get_is_bumper_released_left() {
    return is_bumper_released_left;
  }

  bool get_is_bumper_released_right() {
    return is_bumper_released_right;
  }

private:
  double dx, dth;
  const double length;
  ecl::linear_algebra::Vector3d pose;  // x, y, heading
  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;
  ecl::Slot<const kobuki::BumperEvent&> slot_bumper_event;
  bool is_bumper_released_left;
  bool is_bumper_released_right;
};

/*****************************************************************************
** Signal Handler
*****************************************************************************/

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


  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_MoveForward_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  kobuki_manager.moveForward();
  //ecl::Sleep sleep(1);
  //sleep();
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_MoveBackward_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  kobuki_manager.moveBackward();
  //ecl::Sleep sleep(1);
  //sleep();
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_Stop_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  kobuki_manager.stop();
  //ecl::Sleep sleep(1);
  //sleep();
  return PrtMkIntValue((PRT_UINT32)1);
}

PRT_VALUE* P_GetIsBumperReleasedLeft_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  bool returnValue = kobuki_manager.get_is_bumper_released_left();
  //ecl::Sleep sleep(1);
  //sleep();
  return PrtMkBoolValue((PRT_BOOLEAN)returnValue);
}

PRT_VALUE* P_GetIsBumperReleasedRight_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  bool returnValue = kobuki_manager.get_is_bumper_released_right();
  //ecl::Sleep sleep(1);
  //sleep();
  return PrtMkBoolValue((PRT_BOOLEAN)returnValue);
}

PRT_VALUE* P_TurnLeft_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  kobuki_manager.turnLeft();
  //ecl::Sleep sleep(1);
  //sleep();
  return PrtMkIntValue((PRT_UINT32)1);
}


PRT_VALUE* P_TurnRight_IMPL(PRT_MACHINEINST* context, PRT_VALUE*** argRefs) {
  kobuki_manager.turnRight();
  //ecl::Sleep sleep(1);
  //sleep();
  return PrtMkIntValue((PRT_UINT32)1);
}






