fun Init(): int;
fun CheckIfReached(x: float, z:float, speedMultiplier:float): bool;
fun RotateTowardsLocation(x: float, z:float, speedMultiplier:float, isLowBattery: bool): bool;
fun MoveForward(speed: float, isLowBattery: bool, isCollisionAvoidanceController: bool): int;
fun Stay(): int;
fun GetDistanceSensorReadings(): seq[float];
fun RotateLeft(speedMultiplier: float, isLowBattery: bool): int;
fun RotateRight(speedMultiplier: float, isLowBattery: bool): int;
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
fun GetBoxPositions(): seq[float];

type locationType = (float, float);

event eMotionRequest: (machine, locationType, locationType, bool);
event eMotionRequestX priority 10: (machine, locationType, locationType, bool);
event eMotionPlan: (machine, locationType, locationType, seq[locationType], bool);
event eMotionPlanX priority 10: (machine, locationType, locationType, seq[locationType], bool);
event eMotion: locationType;
event eMotionX priority 10: locationType;
event eBatteryLow priority 10: machine;
event eBatteryRecovered priority 10: machine;
event eCurrentLocation priority 10: locationType;
event eCurrentGoal priority 10: locationType;
event eLocation: locationType;
event ePosition priority 20;
event eOrientation priority 20;


machine Robot {
    var motionPlanner: machine;
    var planExecutor: machine;
    var geoFencing1: machine;
    var geoFencing2: machine;
    var battery: machine;
    var position: machine;
    var orientation: machine;
    var obstacleAvoidance: machine;
    var currentLocation: locationType;
    var goals: seq[locationType];
    var currentGoalIndex: int;
    var step: float;
    var temp: seq[float];
    var chargerLocation: locationType;
    var obstacles: seq[locationType];

    fun DM(): string {
        if (currentLocation == goals[currentGoalIndex] && currentGoalIndex + 1 < sizeof(goals)) {
            currentGoalIndex = currentGoalIndex + 1;
            return "SC";
        }
        return "AC";
    }

    fun AC() {
        Print("Untrusted Controller", 0);
    }

    fun SC() {
        Print("Trusted Controller", 0);
        send motionPlanner, eMotionRequest, (planExecutor, currentLocation, goals[currentGoalIndex], false);
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
            temp = GetBoxPositions();
            obstacles += (sizeof(obstacles), (temp[0], temp[1]));
            obstacles += (sizeof(obstacles), (temp[2], temp[3]));
            obstacles += (sizeof(obstacles), (temp[4], temp[5]));
            obstacles += (sizeof(obstacles), (temp[6], temp[7]));
            obstacles += (sizeof(obstacles), (temp[8], temp[9]));
            obstacles += (sizeof(obstacles), (temp[10], temp[11]));
            step = 0.25;
            planExecutor = new PlanExecutor(this, currentLocation, 0.1, 300.0);
            obstacleAvoidance = new ObstacleAvoidance(obstacles, 0.125);//!!!!Ensure that battery also uses this
            geoFencing2 = new GeoFencing(obstacleAvoidance, step, -0.25, 0.25, -1.25, -0.75);
            geoFencing1 = new GeoFencing(geoFencing2, step, -0.25, 0.25, 0.75, 1.25);
            motionPlanner = new MotionPlanner(geoFencing1, step);
            position = new Position(planExecutor, 0.01);
            orientation = new Orientation(planExecutor, 0.01);
            battery = new Battery(planExecutor, motionPlanner, chargerLocation, 100.0, 200.0);
            goals += (sizeof(goals), (1.0, 1.0));
            goals += (sizeof(goals), (-1.0, 1.0));
            goals += (sizeof(goals), (-1.0, -1.0));
            goals += (sizeof(goals), (1.0, -1.0));
            goals += (sizeof(goals), (1.0, 1.0));
            currentGoalIndex = 0;
            send motionPlanner, eMotionRequest, (planExecutor, currentLocation, goals[currentGoalIndex], false);
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
    var step: float;
    var destinationOfRequest: machine;
    var currentLocation: locationType;
    var goalLocation: locationType;
    var isHighPriorityMotionRequest: bool;

    fun DM(): string {
        if ((currentLocation.0 < 0.2 && currentLocation.1 < 0.2 &&
            currentLocation.0 > -0.2 && currentLocation.1 > -0.2) && 
           (goalLocation.0 < 0.2 && goalLocation.1 < 0.2 &&
            goalLocation.0 > -0.2 && goalLocation.1 > -0.2)) {
            return "AC";
        }
        return "SC";
    }

    fun AC() {
        var motionPlan: seq[locationType];
        Print("Untrusted Controller", 1);
        motionPlan += (sizeof(motionPlan), goalLocation);
        if (isHighPriorityMotionRequest) {
            send destination, eMotionPlanX, (destinationOfRequest, currentLocation, goalLocation, motionPlan, true);
        } else {
            send destination, eMotionPlan, (destinationOfRequest, currentLocation, goalLocation, motionPlan, false);
        }
    }

    fun SC() {
        var waypoint: locationType;
        var motionPlan: seq[locationType];
        Print("Trusted Controller", 1);
        waypoint = currentLocation;
        while (waypoint.0 != goalLocation.0 ||
               waypoint.1 != goalLocation.1) {
            if (waypoint.0 > goalLocation.0) {
                waypoint.0 = waypoint.0 - step;
                if (waypoint.0 < goalLocation.0) {
                    waypoint.0 = goalLocation.0;
                }
            } else if (waypoint.0 < goalLocation.0) {
                waypoint.0 = waypoint.0 + step;
                if (waypoint.0 > goalLocation.0) {
                    waypoint.0 = goalLocation.0;
                }
            }
            if (waypoint.1 > goalLocation.1) {
                waypoint.1 = waypoint.1 - step;
                if (waypoint.1 < goalLocation.1) {
                    waypoint.1 = goalLocation.1;
                }
            } else if (waypoint.1 < goalLocation.1) {
                waypoint.1 = waypoint.1 + step;
                if (waypoint.1 > goalLocation.1) {
                    waypoint.1 = goalLocation.1;
                }
            }
            motionPlan += (sizeof(motionPlan), waypoint);
        }
        if (isHighPriorityMotionRequest) {
            send destination, eMotionPlanX, (destinationOfRequest, currentLocation, goalLocation, motionPlan, true);
        } else {
            send destination, eMotionPlan, (destinationOfRequest, currentLocation, goalLocation, motionPlan, false);
        }
    }

    fun handler(payload: (machine, locationType, locationType, bool)) {
        destinationOfRequest = payload.0;
        currentLocation = payload.1;
        goalLocation = payload.2;
        isHighPriorityMotionRequest = payload.3;
    }

    start state Init {
        entry (payload: (machine, float)) {
            destination = payload.0;
            step = payload.1;
            goto Run;
        }
    }

    state Run {
        rtamodule {
            controller AC;
            controller SC;
            decisionmodule DM @ {AC: 1, SC: 1};
            on eMotionRequest, eMotionRequestX with handler;
        }
    }
}

machine ObstacleAvoidance {
    var obstacles: seq[locationType];
    var epsilon: float;
    var destinationOfRequest: machine;
    var currentLocation: locationType;
    var goalLocation: locationType;
    var isHighPriorityMotionRequest: bool;
    var motionPlan: seq[locationType];

    fun DM(): string {
        if ((currentLocation.0 < 0.2 && currentLocation.1 < 0.2 &&
            currentLocation.0 > -0.2 && currentLocation.1 > -0.2) && 
           (goalLocation.0 < 0.2 && goalLocation.1 < 0.2 &&
            goalLocation.0 > -0.2 && goalLocation.1 > -0.2)) {
            return "AC";
        }
        return "SC";
    }

    fun AC() {
        var index: int;
        Print("Untrusted Controller", 5);
        index = 0;
        while (index < sizeof(motionPlan)) {
            if (isHighPriorityMotionRequest) {
                send destinationOfRequest, eMotionX, motionPlan[index];
            } else {
                send destinationOfRequest, eMotion, motionPlan[index];
            }
            index = index + 1;
        }
    }

    fun SC() {
        var i: int;
        var j: int;
        var flag: bool;
        Print("Trusted Controller", 5);
        i = 0;
        while (i < sizeof(motionPlan)) {
            j = 0;
            flag = true;
            while (j < sizeof(obstacles)) {
                if (motionPlan[i].0 >= obstacles[j].0 - 0.2 &&
                    motionPlan[i].0 <= obstacles[j].0 + 0.2 &&
                    motionPlan[i].1 >= obstacles[j].1 - 0.2 &&
                    motionPlan[i].1 <= obstacles[j].1 + 0.2) {
                    flag = false;
                }
                j = j + 1;
            }
            if (flag) {
                if (isHighPriorityMotionRequest) {
                    send destinationOfRequest, eMotionX, motionPlan[i];
                } else {
                    send destinationOfRequest, eMotion, motionPlan[i];
                }
            }
            i = i + 1;
        }
    }

    fun handler(payload: (machine, locationType, locationType, seq[locationType], bool)) {
        destinationOfRequest = payload.0;
        currentLocation = payload.1;
        goalLocation = payload.2;
        motionPlan = payload.3;
        isHighPriorityMotionRequest = payload.4;
    }

    start state Init {
        entry (payload: (seq[locationType], float)) {
            obstacles = payload.0;
            epsilon = payload.1;
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

machine GeoFencing {
    var destination: machine;
    var step: float;
    var geoFenceX1: float;
    var geoFenceX2: float;
    var geoFenceZ1: float;
    var geoFenceZ2: float;
    var destinationOfRequest: machine;
    var currentLocation: locationType;
    var goalLocation: locationType;
    var isHighPriorityMotionRequest: bool;
    var motionPlan: seq[locationType];

    fun DM(): string {
        if ((currentLocation.0 < 0.2 && currentLocation.1 < 0.2 &&
            currentLocation.0 > -0.2 && currentLocation.1 > -0.2) && 
           (goalLocation.0 < 0.2 && goalLocation.1 < 0.2 &&
            goalLocation.0 > -0.2 && goalLocation.1 > -0.2)) {
            return "AC";
        }
        return "SC";
    }

    fun AC() {
        Print("Untrusted Controller", 4);
        if (isHighPriorityMotionRequest) {
            send destination, eMotionPlanX, (destinationOfRequest, currentLocation, goalLocation, motionPlan, true);
        } else {
            send destination, eMotionPlan, (destinationOfRequest, currentLocation, goalLocation, motionPlan, false);
        }
    }

    fun SC() {
        var index: int;
        var waypoint: locationType;
        var geoFencedMotionPlan: seq[locationType];
        var isGeoFenceHandled: bool;
        var temp: float;
        Print("Trusted Controller", 4);
        index = 0;
        waypoint = currentLocation;
        isGeoFenceHandled = false;
        while (index < sizeof(motionPlan)) {
            if (motionPlan[index].0 >= geoFenceX1 &&
                motionPlan[index].0 <= geoFenceX2 &&
                motionPlan[index].1 >= geoFenceZ1 &&
                motionPlan[index].1 <= geoFenceZ2) {
                waypoint = motionPlan[index];
                temp = geoFenceZ2 - geoFenceZ1;
                if (geoFenceZ1 < 0.0) {
                    waypoint.1 = waypoint.1 + temp;
                } else {
                    waypoint.1 = waypoint.1 - temp;
                }
                if (waypoint.0 == geoFenceX1) {
                    waypoint.0 = waypoint.0 - step;
                } else if (waypoint.0 == geoFenceX2) {
                    waypoint.0 = waypoint.0 + step;
                }
                geoFencedMotionPlan += (sizeof(geoFencedMotionPlan), waypoint);
            } else {
                waypoint = motionPlan[index];
                geoFencedMotionPlan += (sizeof(geoFencedMotionPlan), waypoint);
            }
            index = index + 1;
        }
        if (isHighPriorityMotionRequest) {
            send destination, eMotionPlanX, (destinationOfRequest, currentLocation, goalLocation, geoFencedMotionPlan, true);
        } else {
            send destination, eMotionPlan, (destinationOfRequest, currentLocation, goalLocation, geoFencedMotionPlan, false);
        }
    }

    fun handler(payload: (machine, locationType, locationType, seq[locationType], bool)) {
        destinationOfRequest = payload.0;
        currentLocation = payload.1;
        goalLocation = payload.2;
        motionPlan = payload.3;
        isHighPriorityMotionRequest = payload.4;
    }

    start state Init {
        entry (payload: (machine, float, float, float, float, float)) {
            destination = payload.0;
            step = payload.1;
            geoFenceX1 = payload.2;
            geoFenceX2 = payload.3;
            geoFenceZ1 = payload.4;
            geoFenceZ2 = payload.5;
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

machine PlanExecutor {
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
    var collisionAvoidanceControllerCount: int;
    var collisionAvoidanceControllerLowBatteryCount: int;
    var collisionAvoidanceDistanceThreshold: float;
    var collisionAvoidanceMonitorId: int;
    var right: bool;
    var left: bool;
    var ds: seq[float];
    var robot: machine;

    fun printStats() {
        print(format("Run: {0} {1} {2}\n", advancedMotionControllerCount, safeMotionControllerCount, collisionAvoidanceControllerCount));
        print(format("LowBatteryRun: {0} {1} {2}\n", advancedMotionControllerLowBatteryCount, safeMotionControllerLowBatteryCount, collisionAvoidanceControllerLowBatteryCount));
    }

    fun updateMonitors() {
        var temp: bool;
        temp = IsInTrajectory(currentLocation.0, currentLocation.1, currentMotion.0, currentMotion.1, trajectoryDeviationThreshold);
        UpdateReelayMonitor(trajectoryDeviationMonitorId, format("{{\"isInTrajectory\": {0}}}", temp));
        ds = GetDistanceSensorReadings();
        UpdateReelayMonitor(collisionAvoidanceMonitorId, format("{{\"ds0\": {0}, \"ds1\": {1}}}", ds[0], ds[1]));
    }

    fun updateMonitorsLowBattery() {
        var temp: bool;
        temp = IsInTrajectory(currentLocation.0, currentLocation.1, currentMotion.0, currentMotion.1, trajectoryDeviationThreshold/2.0);
        UpdateReelayMonitor(trajectoryDeviationMonitorId, format("{{\"isInTrajectory\": {0}}}", temp));
        ds = GetDistanceSensorReadings();
        UpdateReelayMonitor(collisionAvoidanceMonitorId, format("{{\"ds0\": {0}, \"ds1\": {1}}}", ds[0], ds[1]));
    }

    fun DM(): string {
        var temp: bool;
        temp = CheckReelayMonitor(collisionAvoidanceMonitorId);
        if (temp) {
            return "CollisionAvoidanceController";
        }
        temp = CheckReelayMonitor(trajectoryDeviationMonitorId);
        if (temp) {
            return "AdvancedMotionController";
        }
        return "SafeMotionController";

    }

    fun CollisionAvoidanceController() {
        var speedMultiplier: float;
        var rotationSpeedMultiplier: float;
        Print("Trusted Controller Collision Avoidance Normal Mode", 2);
        speedMultiplier = 0.5;
        rotationSpeedMultiplier = speedMultiplier;
        if (!left && ds[0] >= collisionAvoidanceDistanceThreshold && (ds[0] >= ds[1] || right)) {
            right = true;
            RotateRight(rotationSpeedMultiplier, false);
        } else if (!right && ds[1] >= collisionAvoidanceDistanceThreshold && (ds[0] < ds[1] || left)) {
            left = true;
            RotateLeft(rotationSpeedMultiplier, false);
        } else {
            right = false;
            left = false;
            MoveForward(speedMultiplier, false, true);
        }
        collisionAvoidanceControllerCount = collisionAvoidanceControllerCount + 1;
        updateMonitors();
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
                MoveForward(speedMultiplier, false, false);
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
                MoveForward(speedMultiplier, false, false);
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
        temp = CheckReelayMonitor(collisionAvoidanceMonitorId);
        if (temp) {
            return "CollisionAvoidanceControllerLowBattery";
        }
        temp = CheckReelayMonitor(trajectoryDeviationMonitorId);
        if (temp) {
            return "AdvancedMotionControllerLowBattery";
        }
        return "SafeMotionControllerLowBattery";

    }

    fun CollisionAvoidanceControllerLowBattery() {
        var speedMultiplier: float;
        var rotationSpeedMultiplier: float;
        Print("Trusted Controller Collision Avoidance Low Battery Mode", 2);
        speedMultiplier = 0.5;
        rotationSpeedMultiplier = speedMultiplier;
        if (!left && ds[0] >= collisionAvoidanceDistanceThreshold && (ds[0] >= ds[1] || right)) {
            right = true;
            RotateRight(rotationSpeedMultiplier, true);
        } else if (!right && ds[1] >= collisionAvoidanceDistanceThreshold && (ds[0] < ds[1] || left)) {
            left = true;
            RotateLeft(rotationSpeedMultiplier, true);
        } else {
            right = false;
            left = false;
            MoveForward(speedMultiplier, true, true);
        }
        collisionAvoidanceControllerLowBatteryCount = collisionAvoidanceControllerLowBatteryCount + 1;
        updateMonitors();
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
                MoveForward(speedMultiplier, true, false);
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
            robot = payload.0;
            currentMotionIndex = 0;
            currentHighPriorityMotionsIndex = 0;
            currentLocation = payload.1;
            trajectoryDeviationThreshold = payload.2;
            collisionAvoidanceDistanceThreshold = payload.3;
            trajectoryDeviationMonitorId = InitDiscreteReelayMonitor("historically[:10]{{isInTrajectory}}");
            collisionAvoidanceMonitorId = InitDiscreteReelayMonitor(format("historically[:10]{{ds0 >= {0}}} or historically[:10]{{ds1 >= {0}}}", collisionAvoidanceDistanceThreshold));
            advancedMotionControllerCount = 0;
            safeMotionControllerCount = 0;
            advancedMotionControllerLowBatteryCount = 0;
            safeMotionControllerLowBatteryCount = 0;
            collisionAvoidanceControllerCount = 0;
            collisionAvoidanceControllerLowBatteryCount = 0;
            right = false;
            left = false;
            goto Run;
        }
    }

    state Run {
        rtamodule {
            controller SafeMotionController period 16 ms;
            controller AdvancedMotionController period 16 ms;
            controller CollisionAvoidanceController period 16 ms;
            decisionmodule DM @ {SafeMotionController: 10, AdvancedMotionController: 1, CollisionAvoidanceController: 100};
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
        on ePosition do {
            UpdateEstimatedPositionWithGPS();
        }
        on eOrientation do {
            UpdateEstimatedOrientationWithCompass();
        }
    }

    state LowBatteryRun {
        defer eMotion;
        rtamodule {
            controller SafeMotionControllerLowBattery period 16 ms;
            controller AdvancedMotionControllerLowBattery period 16 ms;
            controller CollisionAvoidanceControllerLowBattery period 16 ms;
            decisionmodule DMLowBattery @ {SafeMotionControllerLowBattery: 10, AdvancedMotionControllerLowBattery: 1, CollisionAvoidanceControllerLowBattery: 100};
        }
        on eMotionX do (payload: locationType) {
            highPriorityMotions += (sizeof(highPriorityMotions), payload);
        }
        on eBatteryRecovered goto Run with (payload: machine) {
            send payload, eCurrentGoal, lastGoal;
        }
        on ePosition do {
            UpdateEstimatedPositionWithGPS();
        }
        on eOrientation do {
            UpdateEstimatedOrientationWithCompass();
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
            send motionPlanner, eMotionRequestX, (destination, payload, chargerLocation, true);
        }
        on eCurrentGoal do (payload: locationType) {
            send motionPlanner, eMotionRequestX, (destination, chargerLocation, payload, true);
        }
    }
}



machine Position {
    var threshold: float;
    var positionDeviation: float;
    var positionDeviationMonitorId: int;
    var destination: machine;

    fun DM(): string {
        var temp: bool;
        temp = CheckReelayMonitor(positionDeviationMonitorId);
        if (temp) {
            return "SC";
        }
        return "AC";
    }

    fun AC() {
        Print("Untrusted Controller", 6);
        positionDeviation = GetEstimatedPositionDeviation();
        UpdateReelayMonitor(positionDeviationMonitorId, format("{{\"pd\": {0}}}", positionDeviation));
    }

    fun SC() {
        Print("Trusted Controller", 6);
        send destination, ePosition;
        positionDeviation = GetEstimatedPositionDeviation();
        UpdateReelayMonitor(positionDeviationMonitorId, format("{{\"pd\": {0}}}", positionDeviation));
    }

    start state Init {
        entry (payload: (machine, float)) {
            destination = payload.0;
            threshold = payload.1;
            positionDeviationMonitorId = InitDiscreteReelayMonitor(format("historically[:2]{{pd >= {0}}}", threshold));
            goto Run;
        }
    }

    state Run {
        rtamodule {
            controller AC period 1 s;
            controller SC period 1 s;
            decisionmodule DM @ {AC: 1, SC: 1};
        }
    }
}


machine Orientation {
    var threshold: float;
    var orientationDeviation: float;
    var orientationDeviationMonitorId: int;
    var destination: machine;

    fun DM(): string {
        var temp: bool;
        temp = CheckReelayMonitor(orientationDeviationMonitorId);
        if (temp) {
            return "SC";
        }
        return "AC";
    }

    fun AC() {
        Print("Untrusted Controller", 7);
        orientationDeviation = GetEstimatedOrientationDeviation();
        UpdateReelayMonitor(orientationDeviationMonitorId, format("{{\"od\": {0}}}", orientationDeviation));
    }

    fun SC() {
        Print("Trusted Controller", 7);
        send destination, eOrientation;
        orientationDeviation = GetEstimatedOrientationDeviation();
        UpdateReelayMonitor(orientationDeviationMonitorId, format("{{\"od\": {0}}}", orientationDeviation));
    }

    start state Init {
        entry (payload: (machine, float)) {
            destination = payload.0;
            threshold = payload.1;
            orientationDeviationMonitorId = InitDiscreteReelayMonitor(format("historically[:2]{{od >= {0}}}", threshold));
            goto Run;
        }
    }

    state Run {
        rtamodule {
            controller AC period 1 s;
            controller SC period 1 s;
            decisionmodule DM @ {AC: 1, SC: 1};
        }
    }
}