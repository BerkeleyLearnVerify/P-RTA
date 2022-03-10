machine MotionPrimitives {
    var motions: seq[locationType];
    var currentMotionIndex: int;
    var highPriorityMotions: seq[locationType];
    var currentHighPriorityMotionsIndex: int;
    var trajectoryDeviationThreshold: float;
    var trajectoryDeviation: float;
    var currentLocation: locationType;
    var currentMotion: locationType;
    var lastGoal: locationType;
    var advancedMotionControllerCount: int;
    var safeMotionControllerCount: int;
    var robot: machine;
    var rotateCount: int;
    var isBatteryLow: bool;
    var bumpCountPerGoalLocation: int;
    var bumpCountThreshold: int;
    var isPreviouslyBumpedLeft: bool; // RESET WHEN BATTERY IS LOW!!!!
    var isPreviouslyBumpedRight: bool; // RESET WHEN BATTERY IS LOW!!!!
    var isPreviouslyBumpedCenter: bool; // RESET WHEN BATTERY IS LOW!!!!
    var motionPlanner: machine;
    var isAvoidLocationSent: bool;
    var backwardCount: int;
    var tempLocation: seq[float];
    var tourCount: int;
    var monitorId: int;

    fun DM(): string {
        var temp: bool;
        var isBumperReleasedLeft: bool;
        var isBumperReleasedCenter: bool;
        var isBumperReleasedRight: bool;
        var isGeoFenceViolated: bool;
        isBumperReleasedLeft = GetIsBumperReleasedLeft();
        isBumperReleasedCenter = GetIsBumperReleasedCenter();
        isBumperReleasedRight = GetIsBumperReleasedRight();
        isGeoFenceViolated = GetIsGeoFenceViolated();
        isAvoidLocationSent = false;
        SetLed(0, 0); /* set led0 to black */
        SetLed(1, 0); /* set led1 to black */
        temp = CheckMonitor(monitorId);
        //if (tourCount > 1 && !temp && !IsTherePotentialAvoidLocation()) {
        if (!temp) {
            SetLed(0, 2); /* set led0 to green */
            return "AdvancedMotionController";
        }
        SetLed(0, 1); /* set led0 to red */
        if (isBatteryLow) {
            SetLed(0, 3); /* set led0 to orange */
        }
        return "SafeMotionController";
    }

    fun SafeMotionController() {
        var forwardSpeed: float;
        var rotationSpeed: float;
        forwardSpeed = 0.1;
        rotationSpeed = 0.2;
        PrintControllerExecution(0);
        if (currentHighPriorityMotionsIndex < sizeof(highPriorityMotions)) {
            currentMotion = highPriorityMotions[currentHighPriorityMotionsIndex];
            if (!RotateTowardsLocation(currentMotion.0, currentMotion.1, rotationSpeed)) {
                MoveForward(forwardSpeed, 0.0);
            }
            if (CheckIfReached(currentMotion.0, currentMotion.1, forwardSpeed)) {
                if (currentMotion.0 == 1.0 && currentMotion.1 == 1.0) {
                    tourCount = tourCount + 1;
                }
                tempLocation = GetRobotPosition();
                currentLocation = (tempLocation[0], tempLocation[1]);
                currentHighPriorityMotionsIndex = currentHighPriorityMotionsIndex + 1;
                bumpCountPerGoalLocation = 0;
                isPreviouslyBumpedLeft = false;
                isPreviouslyBumpedCenter = false;
                isPreviouslyBumpedRight = false;
                send robot, eCurrentLocation, currentMotion;
            }
            safeMotionControllerCount = safeMotionControllerCount + 1;
        } else if (!isBatteryLow && currentMotionIndex < sizeof(motions)) {
            currentMotion = motions[currentMotionIndex];
            if (!RotateTowardsLocation(currentMotion.0, currentMotion.1, rotationSpeed)) {
                MoveForward(forwardSpeed, 0.0);
            }
            if (CheckIfReached(currentMotion.0, currentMotion.1, forwardSpeed)) {
                if (currentMotion.0 == 1.0 && currentMotion.1 == 1.0) {
                    tourCount = tourCount + 1;
                }
                tempLocation = GetRobotPosition();
                currentLocation = (tempLocation[0], tempLocation[1]);
                currentMotionIndex = currentMotionIndex + 1;
                bumpCountPerGoalLocation = 0;
                isPreviouslyBumpedLeft = false;
                isPreviouslyBumpedCenter = false;
                isPreviouslyBumpedRight = false;
                send robot, eCurrentLocation, currentMotion;
            }
            safeMotionControllerCount = safeMotionControllerCount + 1;
        } else {
            Stay();
        }
        trajectoryDeviation = GetTrajectoryDeviation(currentMotion.0, currentMotion.1);
        UpdateMonitor(monitorId, trajectoryDeviation);
    }

    fun AdvancedMotionController() {
        var speedMultiplier: float;
        speedMultiplier = 2.0;
        PrintControllerExecution(1);
        if (currentHighPriorityMotionsIndex < sizeof(highPriorityMotions)) {
            currentMotion = highPriorityMotions[currentHighPriorityMotionsIndex];
            StepPID(currentMotion.0, currentMotion.1);
            if (CheckIfReached(currentMotion.0, currentMotion.1, speedMultiplier)) {
                if (currentMotion.0 == 1.0 && currentMotion.1 == 1.0) {
                    tourCount = tourCount + 1;
                }
                tempLocation = GetRobotPosition();
                currentLocation = (tempLocation[0], tempLocation[1]);
                currentHighPriorityMotionsIndex = currentHighPriorityMotionsIndex + 1;
                bumpCountPerGoalLocation = 0;
                isPreviouslyBumpedLeft = false;
                isPreviouslyBumpedCenter = false;
                isPreviouslyBumpedRight = false;
                send robot, eCurrentLocation, currentMotion;
            }
            advancedMotionControllerCount = advancedMotionControllerCount + 1;
        } else if (!isBatteryLow && currentMotionIndex < sizeof(motions)) {
            currentMotion = motions[currentMotionIndex];
            StepPID(currentMotion.0, currentMotion.1);
            if (CheckIfReached(currentMotion.0, currentMotion.1, speedMultiplier)) {
                if (currentMotion.0 == 1.0 && currentMotion.1 == 1.0) {
                    tourCount = tourCount + 1;
                }
                tempLocation = GetRobotPosition();
                currentLocation = (tempLocation[0], tempLocation[1]);
                currentMotionIndex = currentMotionIndex + 1;
                bumpCountPerGoalLocation = 0;
                isPreviouslyBumpedLeft = false;
                isPreviouslyBumpedCenter = false;
                isPreviouslyBumpedRight = false;
                send robot, eCurrentLocation, currentMotion;
            }
            advancedMotionControllerCount = advancedMotionControllerCount + 1;
        } else {
            Stay();
        }
        trajectoryDeviation = GetTrajectoryDeviation(currentMotion.0, currentMotion.1);
        UpdateMonitor(monitorId, trajectoryDeviation);
    }

    start state Init {
        entry (payload: (machine, machine, locationType, float, float)) {
            currentMotionIndex = 0;
            currentMotion.0 = 0.0;
            currentMotion.1 = 0.0;
            currentHighPriorityMotionsIndex = 0;
            robot = payload.0;
            motionPlanner = payload.1;
            currentLocation = payload.2;
            trajectoryDeviationThreshold = payload.3;
            advancedMotionControllerCount = 0;
            safeMotionControllerCount = 0;
            rotateCount = 0;
            isBatteryLow = false;
            bumpCountPerGoalLocation = 0;
            bumpCountThreshold = 2;
            isPreviouslyBumpedLeft = false;
            isPreviouslyBumpedCenter = false;
            isPreviouslyBumpedRight = false;
            isAvoidLocationSent = false;
            tourCount = 0;
            monitorId = InitMonitorGlobalLowerBound(10, trajectoryDeviationThreshold);
            goto Run;
        }
    }

    state Run {
        rtamodule {
            controller SafeMotionController period 50 ms;
            controller AdvancedMotionController period 10 ms;
            decisionmodule DM @ {SafeMotionController: 1,
                                 AdvancedMotionController: 1};
        }
        on eMotion do (payload: locationType) {
            motions += (sizeof(motions), payload);
        }
        on eMotionX do (payload: locationType) {
            highPriorityMotions += (sizeof(highPriorityMotions), payload);
        }
    }
}