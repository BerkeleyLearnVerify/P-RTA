machine MotionPlanner {
    var destination: machine;
    var destinationOfRequest: machine;
    var currentLocation: locationType;
    var goalLocation: locationType;
    var isHighPriorityMotionRequest: bool;
    var geoFencedLocations: seq[locationType];

    fun DM(): string {
        var temp: bool;
        temp = IsThereAvoidLocationInSegment(currentLocation.0, currentLocation.1, goalLocation.0, goalLocation.1);
        if (!temp) {
            NotifyController(2, 1);
            return "AC";
        }
        NotifyController(2, 0);
        return "SC";
    }

    fun AC() {
        var motionPlan: seq[locationType];
        var index: int;
        motionPlan = GetOMPLMotionPlanAC(currentLocation.0, currentLocation.1, goalLocation.0, goalLocation.1);
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
        var motionPlan: seq[locationType];
        var index: int;
        motionPlan = GetOMPLMotionPlanSC(currentLocation.0, currentLocation.1, goalLocation.0, goalLocation.1);
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

    fun handler(payload: (machine, locationType, locationType, bool)) {
        destinationOfRequest = payload.0;
        currentLocation = payload.1;
        goalLocation = payload.2;
        isHighPriorityMotionRequest = payload.3;
    }

    start state Init {
        entry (payload: seq[locationType]) {
            var index: int;
            index = 0;
            geoFencedLocations = payload;
            while (index < sizeof(geoFencedLocations)) {
                RegisterGeoFenceLocation(geoFencedLocations[index].0, geoFencedLocations[index].1);
                index = index + 1;
            }
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