fun Init(): int;
fun CheckIfReached(x: float, z:float, speed:float): bool;
fun RotateTowardsLocation(x: float, z:float, speed:float): bool;
fun MoveForward(positionalSpeed: float, rotationalSpeed: float): int;
fun MoveBackward(speed: float): int;
fun RotateLeft(speed: float): int;
fun RotateRight(speed: float): int;
fun Stay(): int;
fun GetTrajectoryDeviation(x_goal:float, z_goal:float): float;
fun RandomController(forwardSpeed: float, rotationSpeed: float): int;
fun GetRobotPosition(): seq[float];
fun GetChargerPosition(): seq[float];
fun GetIsBumperReleasedLeft(): bool;
fun GetIsBumperReleasedCenter(): bool;
fun GetIsBumperReleasedRight(): bool;
fun GetIsGeoFenceViolated(): bool;
fun GetBatteryLevel(): float;
fun GetIsButtonPressedAndReleasedB0(): bool;
fun StepPID(x: float, z:float): int;
fun GetOMPLMotionPlanAC(currentLocationX: float, currentLocationZ: float, goalLocationX: float, goalLocationZ: float): seq[(float, float)];
fun GetOMPLMotionPlanSC(currentLocationX: float, currentLocationZ: float, goalLocationX: float, goalLocationZ: float): seq[(float, float)];
fun RegisterPotentialAvoidLocation(x_goal: float, z_goal: float): bool;
fun RegisterGeoFenceLocation(x_goal: float, z_goal: float): bool;
fun SetLed(led_num: int, led_color: int): int;
fun IsTherePotentialAvoidLocation(): bool;
fun FirstTourIsCompleted(): int;
fun ResetOdometry(): int;
fun IsThereAvoidLocationInSegment(x_start: float, z_start: float, x_goal: float, z_goal: float): bool;
fun InitMonitorGlobalLowerBound(len: int, threshold: float): int;
fun UpdateMonitor(id: int, value: float): int;
fun CheckMonitor(id: int): bool;
fun NotifyController(machineId: int, controllerId: int): int;

type locationType = (float, float);

event eMotionRequest: (machine, locationType, locationType, bool);
event eMotionRequestX priority 10: (machine, locationType, locationType, bool);
event eMotion: locationType;
event eMotionX priority 10: locationType;
event eBatteryLow priority 10: machine;
event eBatteryRecovered priority 10: machine;
event eCurrentLocation priority 10: locationType;
event eCurrentGoal priority 10: locationType;

machine EgoRobot {
    var motionPlanner: machine;
    var motionPrimitives: machine;
    var battery: machine;
    var currentLocation: locationType;
    var goals: seq[locationType];
    var currentGoalIndex: int;
    var temp: seq[float];
    var chargerLocation: locationType;
    var geoFencedLocations: seq[locationType];

    fun DM(): string {
        if (currentLocation == goals[currentGoalIndex]) {
            if (currentGoalIndex + 1 < sizeof(goals)) {
                currentGoalIndex = currentGoalIndex + 1;
            } else {
                currentGoalIndex = 0;
            }
            NotifyController(0, 0);
            return "SC";
        }
        NotifyController(0, 1);
        return "AC";
    }

    fun AC() {
        ;
    }

    fun SC() {
        send motionPlanner, eMotionRequest, (motionPrimitives, currentLocation, goals[currentGoalIndex], false);
    }

    fun handler(payload: locationType) {
        currentLocation = payload;
    }

    start state Init {
        entry {
            Init();
            temp = GetRobotPosition();
            currentLocation = (temp[0], temp[1]);
            temp = GetChargerPosition();
            chargerLocation = (temp[0], temp[1]);
            geoFencedLocations += (sizeof(geoFencedLocations), (1.0, 0.0));
            motionPlanner = new MotionPlanner(geoFencedLocations);
            motionPrimitives = new MotionPrimitives(this, motionPlanner, currentLocation, 0.2, 300.0);
            battery = new Battery(motionPrimitives, motionPlanner, chargerLocation, 50.0, 100.0);
            goals += (sizeof(goals), (1.0, 1.0));
            goals += (sizeof(goals), (-1.0, 1.0));
            goals += (sizeof(goals), (-1.0, -1.0));
            goals += (sizeof(goals), (1.0, -1.0));
            currentGoalIndex = 0;
            goto WaitB0Press;
        }
    }

    state WaitB0Press {
        entry {
            var isButtonB0PressedAndReleased: bool;
            isButtonB0PressedAndReleased = GetIsButtonPressedAndReleasedB0();
            if (isButtonB0PressedAndReleased) {
                send motionPlanner, eMotionRequest, (motionPrimitives, currentLocation, goals[currentGoalIndex], false);
                goto Run;
            } else {
                goto WaitB0Press;
            }
        }
    }

    state Run {
        rtamodule {
            controller AC;
            controller SC;
            decisionmodule DM @ {AC: 1, SC: 1};
            on eCurrentLocation with handler;
        }
    }
}






