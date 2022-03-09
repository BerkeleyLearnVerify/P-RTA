machine Battery {
    var destination: machine;
    var motionPlanner: machine;
    var chargerLocation: locationType;
    var previousBatteryLevel: float;
    var currentBatteryLevel: float;
    var batteryLevelLowerBound: float;
    var batteryLevelUpperBound: float;
    var isBatteryLow: bool;

    fun DM(): string {
        var isButtonB0PressedAndReleased: bool;
        isButtonB0PressedAndReleased = GetIsButtonPressedAndReleasedB0();
        previousBatteryLevel = currentBatteryLevel;
        currentBatteryLevel = GetBatteryLevel();
        if (!isBatteryLow && currentBatteryLevel < batteryLevelLowerBound && previousBatteryLevel >= batteryLevelLowerBound) {
            NotifyController(1, 0);
            return "HandleLowBattery";
        }
        if (isBatteryLow && isButtonB0PressedAndReleased) {
            NotifyController(1, 0);
            return "NotifyRecovery";
        }
        NotifyController(1, 1);
        return "Idle";
    }

    fun Idle() {
        ;
    }

    fun HandleLowBattery() {
        isBatteryLow = true;
        send destination, eBatteryLow, this;
    }

    fun NotifyRecovery() {
        isBatteryLow = false;
        send destination, eBatteryRecovered, this;
    }

    start state Init {
        entry (payload: (machine, machine, locationType, float, float)) {
            destination = payload.0;
            motionPlanner = payload.1;
            chargerLocation = payload.2;
            batteryLevelLowerBound= payload.3;
            batteryLevelUpperBound= payload.4;
            isBatteryLow = false;
            previousBatteryLevel = 100.0;
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