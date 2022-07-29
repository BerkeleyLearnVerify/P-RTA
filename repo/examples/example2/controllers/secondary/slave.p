fun Init(): int;
fun Rotate(): int;
fun MoveForward(): int;
fun GetDistanceSensorReadings(): seq[float];

machine Robot {
    var ds: seq[float];

    fun DM(): string {
        ds = GetDistanceSensorReadings();
        if (ds[0] >= 300.0 ||
            ds[1] >= 300.0) {
            return "SC";
        }
        return "AC";

    }

    fun AC() {
        MoveForward();
    }

    fun SC() {
        Rotate();
    }

    start state Init {
        entry {
            Init();
            goto Run;
        }
    }

    state Run {
        rtamodule {
            controller AC period 16 ms;
            controller SC period 16 ms;
            decisionmodule DM @ {AC: 1, SC:1};
        }
    }
}
