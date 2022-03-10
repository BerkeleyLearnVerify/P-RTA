fun Init(): int;
fun MoveForward(): int;
fun MoveBackward(): int;
fun Stop(): int;
fun TurnRight(): int;
fun TurnLeft(): int;
fun GetIsBumperReleasedLeft(): bool;
fun GetIsBumperReleasedRight(): bool;

machine RandomWalker {

    fun DM(): string {
    	var isBumperReleasedLeft: bool;
    	var isBumperReleasedRight: bool;
        isBumperReleasedLeft = GetIsBumperReleasedLeft();
        isBumperReleasedRight = GetIsBumperReleasedRight();
        if (!isBumperReleasedLeft) {
        	return "SC1";//Trun Right
        }
        if (!isBumperReleasedRight) {
        	return "SC2";//Trun Left
        }
        return "AC";//Move forward

    }

    fun AC() {
        MoveForward();
    }

    fun SC1() {
    	var isBumperReleasedLeft: bool;
        isBumperReleasedLeft = GetIsBumperReleasedLeft();
        if (isBumperReleasedLeft) {
        	TurnRight();
        } else {
        	MoveBackward();
        }
    }

    fun SC2() {
    	var isBumperReleasedRight: bool;
        isBumperReleasedRight = GetIsBumperReleasedRight();
        if (isBumperReleasedRight) {
        	TurnLeft();
        } else {
        	MoveBackward();
        }
    }

    start state Init {
        entry {
            Init();
            goto Run;
        }
    }

    state Run {
        rtamodule {
            controller AC period 50 ms;
            controller SC1 period 200 ms;
            controller SC2 period 200 ms;
            decisionmodule DM @ {AC: 1, SC1:10, SC2:10};
        }
    }
}
