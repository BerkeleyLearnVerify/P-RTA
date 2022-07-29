fun Init(): int;
fun CheckIfReached(x: float, z:float, speedMultiplier:float): bool;
fun RotateTowardsLocation(x: float, z:float, speedMultiplier:float, isLowBattery: bool): bool;
fun MoveForward(speed: float, isLowBattery: bool): int;
fun Stay(): int;
fun GetDistanceSensorReadings(): seq[float];
fun RotateLeft(speedMultiplier: float): int;
fun RotateRight(speedMultiplier: float): int;
fun GetBatteryLevel(): float;
fun GetEstimatedPositionDeviation(): float;
fun UpdateEstimatedPositionWithGPS(): int;
fun GetEstimatedOrientationDeviation(): float;
fun UpdateEstimatedOrientationWithCompass(): int;
fun Print(str:string, line:int): int;

fun InitDiscreteReelayMonitor(str: string): int;
fun InitDenseReelayMonitor(str: string): int;
fun UpdateReelayMonitor(id: int, str: string): bool;
fun CheckReelayMonitor(id: int): bool;
fun IsInTrajectory(x0: float, z0: float, x1:float, z1:float, trajectoryDeviationThreshold: float): bool;
fun RandomController(isLowBattery: bool): int;

fun GetRobotPosition(): seq[float];
fun GetChargerPosition(): seq[float];

type locationType = (float, float);

event eMotionPlan: (machine, locationType, locationType);
event eMotionPlanX priority 10: (machine, locationType, locationType);
event eMotion: locationType;
event eMotionX priority 10: locationType;
event eBatteryLow priority 10: machine;
event eBatteryRecovered priority 10: machine;
event eCurrentLocation priority 10: locationType;
event eCurrentGoal priority 10: locationType;
event eLocation: locationType;


machine Robot {
    var motionPrimitives: machine;
    var motionPlanner: machine;
    var battery: machine;
    var goals: seq[locationType];
    var currentGoalIndex: int;
    var currentLocation: locationType;
    var chargerLocation: locationType;
    var temp: seq[float];

    fun DM(): string {
        if (currentLocation == goals[currentGoalIndex]) {
            if (currentGoalIndex + 1 < sizeof(goals)) {
                currentGoalIndex = currentGoalIndex + 1;
            } else {
                currentGoalIndex = 0;
            }
            return "SC";
        }
        return "AC";
    }

    fun AC() {
        Print("Untrusted Controller", 0);
    }

    fun SC() {
        Print("Trusted Controller", 0);
        send motionPlanner, eMotionPlan, (motionPrimitives, currentLocation, goals[currentGoalIndex]);
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
            motionPlanner = new MotionPlanner();
            motionPrimitives = new MotionPrimitives(this, currentLocation, 0.1, 300.0);
            battery = new Battery(motionPrimitives, motionPlanner, chargerLocation, 50.0, 200.0);
            goals += (sizeof(goals), (0.5, 0.5));
            goals += (sizeof(goals), (-0.5, 0.5));
            goals += (sizeof(goals), (-0.5, -0.5));
            goals += (sizeof(goals), (0.5, -0.5));
            currentGoalIndex = 0;
            send motionPlanner, eMotionPlan, (motionPrimitives, currentLocation, goals[currentGoalIndex]);
            Print("Untrusted Controller", 0);
            goto Run;
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

machine MotionPlanner {
    var destination: machine;
    var currentLocation: locationType;
    var goalLocation: locationType;

    fun DM(): string {
        if ((currentLocation.0 < 0.5 && currentLocation.1 < 0.5 &&
            currentLocation.0 > -0.5 && currentLocation.1 > -0.5) || 
           (goalLocation.0 < 0.5 && goalLocation.1 < 0.5 &&
            goalLocation.0 > -0.5 && goalLocation.1 > -0.5)) {
            return "AC";
        }
        return "SC";
    }

    fun AC() {
        Print("Untrusted Controller", 1);
        send destination, eMotion, goalLocation;
    }

    fun SC() {
        var step: float;
        Print("Trusted Controller", 1);
        step = 0.25;
        while (currentLocation.0 != goalLocation.0 ||
               currentLocation.1 != goalLocation.1) {
            if (currentLocation.0 > goalLocation.0) {
                currentLocation.0 = currentLocation.0 - step;
                if (currentLocation.0 < goalLocation.0) {
                    currentLocation.0 = goalLocation.0;
                }
            } else if (currentLocation.0 < goalLocation.0) {
                currentLocation.0 = currentLocation.0 + step;
                if (currentLocation.0 > goalLocation.0) {
                    currentLocation.0 = goalLocation.0;
                }
            }
            if (currentLocation.1 > goalLocation.1) {
                currentLocation.1 = currentLocation.1 - step;
                if (currentLocation.1 < goalLocation.1) {
                    currentLocation.1 = goalLocation.1;
                }
            } else if (currentLocation.1 < goalLocation.1) {
                currentLocation.1 = currentLocation.1 + step;
                if (currentLocation.1 > goalLocation.1) {
                    currentLocation.1 = goalLocation.1;
                }
            }
            send destination, eMotion, currentLocation;
        }
    }

    fun handler(payload: (machine, locationType, locationType)) {
        destination = payload.0;
        currentLocation = payload.1;
        goalLocation = payload.2;
    }

    start state Init {
        entry {
            goto Run;
        }
    }

    state Run {
        rtamodule {
            controller AC;
            controller SC;
            decisionmodule DM @ {AC: 1, SC: 1};
            on eMotionPlan, eMotionPlanX with handler;
        }
    }
}

machine MotionPrimitives {
    var motions: seq[locationType];
    var currentMotionIndex: int;
    var highPriorityMotions: seq[locationType];
    var currentHighPriorityMotionsIndex: int;
    var trajectoryDeviationMonitorId: int;
    var trajectoryDeviationThreshold: float;
    var trajectoryDeviation: float;
    var currentLocation: locationType;
    var currentMotion: locationType;
    var lastGoal: locationType;
    var advancedMotionControllerCount: int;
    var safeMotionControllerCount: int;
    var advancedMotionControllerLowBatteryCount: int;
    var safeMotionControllerLowBatteryCount: int;
    var robot: machine;

    fun printStats() {
        print(format("Run: {0} {1}\n", advancedMotionControllerCount, safeMotionControllerCount));
        print(format("LowBatteryRun: {0} {1}\n", advancedMotionControllerLowBatteryCount, safeMotionControllerLowBatteryCount));
    }

    fun updateMonitors() {
        var temp: bool;
        temp = IsInTrajectory(currentLocation.0, currentLocation.1, currentMotion.0, currentMotion.1, trajectoryDeviationThreshold);
        UpdateReelayMonitor(trajectoryDeviationMonitorId, format("{{\"isInTrajectory\": {0}}}", temp));
    }

    fun updateMonitorsLowBattery() {
        var temp: bool;
        temp = IsInTrajectory(currentLocation.0, currentLocation.1, currentMotion.0, currentMotion.1, trajectoryDeviationThreshold/2.0);
        UpdateReelayMonitor(trajectoryDeviationMonitorId, format("{{\"isInTrajectory\": {0}}}", temp));
    }

    fun DM(): string {
        var temp: bool;
        temp = CheckReelayMonitor(trajectoryDeviationMonitorId);
        if (temp) {
            return "AdvancedMotionController";
        }
        return "SafeMotionController";

    }

    fun AdvancedMotionController() {
        Print("Untrusted Controller Normal Mode", 2);
        if (currentHighPriorityMotionsIndex < sizeof(highPriorityMotions)) {
            currentMotion = highPriorityMotions[currentHighPriorityMotionsIndex];
            RandomController(false);
            if (CheckIfReached(currentMotion.0, currentMotion.1, 1.0)) {
                currentLocation = currentMotion;
                currentHighPriorityMotionsIndex = currentHighPriorityMotionsIndex + 1;
                send robot, eCurrentLocation, currentLocation;
                printStats();
            }
            advancedMotionControllerCount = advancedMotionControllerCount + 1;
        } else if (currentMotionIndex < sizeof(motions)) {
            currentMotion = motions[currentMotionIndex];
            RandomController(false);
            if (CheckIfReached(currentMotion.0, currentMotion.1, 1.0)) {
                currentLocation = currentMotion;
                currentMotionIndex = currentMotionIndex + 1;
                send robot, eCurrentLocation, currentLocation;
                printStats();
            }
            advancedMotionControllerCount = advancedMotionControllerCount + 1;
        } else {
            Stay();
        }
        updateMonitors();
    }

    fun SafeMotionController() {
        var speedMultiplier: float;
        var rotationSpeedMultiplier: float;
        Print("Trusted Controller Normal Mode", 2);
        speedMultiplier = 0.6;
        rotationSpeedMultiplier = speedMultiplier / 2.0;
        if (currentHighPriorityMotionsIndex < sizeof(highPriorityMotions)) {
            currentMotion = highPriorityMotions[currentHighPriorityMotionsIndex];
            if (!RotateTowardsLocation(currentMotion.0, currentMotion.1, rotationSpeedMultiplier, false)) {
                MoveForward(speedMultiplier, false);
            }
            if (CheckIfReached(currentMotion.0, currentMotion.1, speedMultiplier)) {
                currentLocation = currentMotion;
                currentHighPriorityMotionsIndex = currentHighPriorityMotionsIndex + 1;
                send robot, eCurrentLocation, currentLocation;
                printStats();
            }
            safeMotionControllerCount = safeMotionControllerCount + 1;
        } else if (currentMotionIndex < sizeof(motions)) {
            currentMotion = motions[currentMotionIndex];
            if (!RotateTowardsLocation(currentMotion.0, currentMotion.1, rotationSpeedMultiplier, false)) {
                MoveForward(speedMultiplier, false);
            }
            if (CheckIfReached(currentMotion.0, currentMotion.1, speedMultiplier)) {
                currentLocation = currentMotion;
                currentMotionIndex = currentMotionIndex + 1;
                send robot, eCurrentLocation, currentLocation;
                printStats();
            }
            safeMotionControllerCount = safeMotionControllerCount + 1;
        } else {
            Stay();
        }
        updateMonitors();
    }

    fun DMLowBattery(): string {
        var temp: bool;
        temp = CheckReelayMonitor(trajectoryDeviationMonitorId);
        if (temp) {
            return "AdvancedMotionControllerLowBattery";
        }
        return "SafeMotionControllerLowBattery";

    }

    fun AdvancedMotionControllerLowBattery() {
        Print("Untrusted Controller Low Battery Mode", 2);
        if (currentHighPriorityMotionsIndex < sizeof(highPriorityMotions)) {
            currentMotion = highPriorityMotions[currentHighPriorityMotionsIndex];
            RandomController(true);
            if (CheckIfReached(currentMotion.0, currentMotion.1, 1.0)) {
                currentLocation = currentMotion;
                currentHighPriorityMotionsIndex = currentHighPriorityMotionsIndex + 1;
                send robot, eCurrentLocation, currentLocation;
                printStats();
            }
            advancedMotionControllerLowBatteryCount = advancedMotionControllerLowBatteryCount + 1;
        } else {
            Stay();
        }
        updateMonitorsLowBattery();
    }

    fun SafeMotionControllerLowBattery() {
        var speedMultiplier: float;
        var rotationSpeedMultiplier: float;
        Print("Trusted Controller Low Battery Mode", 2);
        speedMultiplier = 1.0;
        rotationSpeedMultiplier = speedMultiplier / 2.0;
        if (currentHighPriorityMotionsIndex < sizeof(highPriorityMotions)) {
            currentMotion = highPriorityMotions[currentHighPriorityMotionsIndex];
            if (!RotateTowardsLocation(currentMotion.0, currentMotion.1, rotationSpeedMultiplier, true)) {
                MoveForward(speedMultiplier, true);
            }
            if (CheckIfReached(currentMotion.0, currentMotion.1, speedMultiplier)) {
                currentLocation = currentMotion;
                currentHighPriorityMotionsIndex = currentHighPriorityMotionsIndex + 1;
                send robot, eCurrentLocation, currentLocation;
                printStats();
            }
            safeMotionControllerLowBatteryCount = safeMotionControllerLowBatteryCount + 1;
        } else {
            Stay();
        }
        updateMonitorsLowBattery();
    }

    start state Init {
        entry (payload: (machine, locationType, float, float)) {
            currentMotionIndex = 0;
            currentHighPriorityMotionsIndex = 0;
            robot = payload.0;
            currentLocation = payload.1;
            trajectoryDeviationThreshold = payload.2;
            trajectoryDeviationMonitorId = InitDiscreteReelayMonitor("historically[:10]{{isInTrajectory}}");
            advancedMotionControllerCount = 0;
            safeMotionControllerCount = 0;
            advancedMotionControllerLowBatteryCount = 0;
            safeMotionControllerLowBatteryCount = 0;
            goto Run;
        }
    }

    state Run {
        rtamodule {
            controller SafeMotionController period 16 ms;
            controller AdvancedMotionController period 16 ms;
            decisionmodule DM @ {SafeMotionController: 50, AdvancedMotionController: 10};
        }
        on eMotion do (payload: locationType) {
            motions += (sizeof(motions), payload);
        }
        on eMotionX do (payload: locationType) {
            highPriorityMotions += (sizeof(highPriorityMotions), payload);
        }
        on eBatteryLow goto LowBatteryRun with (payload: machine) {
            send payload, eCurrentLocation, currentLocation;
            lastGoal = currentMotion;
        }
    }

    state LowBatteryRun {
        defer eMotion;
        rtamodule {
            controller SafeMotionControllerLowBattery period 16 ms;
            controller AdvancedMotionControllerLowBattery period 16 ms;
            decisionmodule DMLowBattery @ {SafeMotionControllerLowBattery: 100, AdvancedMotionControllerLowBattery: 10};
        }
        on eMotionX do (payload: locationType) {
            highPriorityMotions += (sizeof(highPriorityMotions), payload);
        }
        on eBatteryRecovered goto Run with (payload: machine) {
            send payload, eCurrentGoal, lastGoal;
        }
    }

}

machine Battery {
    var destination: machine;
    var motionPlanner: machine;
    var chargerLocation: locationType;
    var currentBatteryLevel: float;
    var batteryLevelLowerBound: float;
    var batteryLevelUpperBound: float;
    var monitor1Id: int;
    var monitor2Id: int;
    var isBatteryLow: bool;

    fun DM(): string {
        var monitorStatus: bool;
        monitorStatus = CheckReelayMonitor(monitor1Id);
        if (!isBatteryLow && monitorStatus) {
            return "HandleLowBattery";
        }
        monitorStatus = CheckReelayMonitor(monitor2Id);
        if (isBatteryLow && monitorStatus) {
            return "NotifyRecovery";
        }
        return "Idle";
    }

    fun updateMonitors() {
        currentBatteryLevel = GetBatteryLevel();
        UpdateReelayMonitor(monitor1Id, format("{{\"batteryLevel\": {0}}}", currentBatteryLevel));
        UpdateReelayMonitor(monitor2Id, format("{{\"batteryLevel\": {0}}}", currentBatteryLevel));
    }

    fun Idle() {
        Print("Untrusted Controller", 3);
        updateMonitors();
    }

    fun HandleLowBattery() {
        Print("Trusted Controller Low Battery", 3);
        isBatteryLow = true;
        send destination, eBatteryLow, this;
        updateMonitors();
    }

    fun NotifyRecovery() {
        Print("Trusted Controller Battery Recovered", 3);
        isBatteryLow = false;
        send destination, eBatteryRecovered, this;
        updateMonitors();
    }

    start state Init {
        entry (payload: (machine, machine, locationType, float, float)) {
            destination = payload.0;
            motionPlanner = payload.1;
            chargerLocation = payload.2;
            batteryLevelLowerBound= payload.3;
            batteryLevelUpperBound= payload.4;
            isBatteryLow = false;
            monitor1Id = InitDiscreteReelayMonitor(format("pre{{batteryLevel > {0}}} and {{batteryLevel <= {0}}}", batteryLevelLowerBound));
            monitor2Id = InitDiscreteReelayMonitor(format("pre{{batteryLevel <= {0}}} and {{batteryLevel > {0}}}", batteryLevelUpperBound));
            goto Run;
        }
    }

    state Run {
        rtamodule {
            controller Idle period 1 s;
            controller HandleLowBattery period 1 s;
            controller NotifyRecovery period 1 s;
            decisionmodule DM @ {Idle: 1, HandleLowBattery: 1, NotifyRecovery: 1};
        }
        on eCurrentLocation do (payload: locationType) {
            send motionPlanner, eMotionPlanX, (this, payload, chargerLocation);
        }
        on eMotion do (payload: locationType) {
            send destination, eMotionX, payload;
        }
        on eCurrentGoal do (payload: locationType) {
            send motionPlanner, eMotionPlanX, (this, chargerLocation, payload);
        }
    }
}

