#include "random_walker.h"
#include "Prt.h"
#include <stdio.h>
#include <cstring>

PRT_PROCESS* MAIN_P_PROCESS;
static PRT_BOOLEAN cooperative = PRT_TRUE;
static const char* parg = NULL;

void ErrorHandler(PRT_STATUS status, PRT_MACHINEINST* ptr) {
	if (status == PRT_STATUS_ASSERT) {
		//fprintf_s(stdout, "exiting with PRT_STATUS_ASSERT (assertion failure)\n");
		exit(1);
	} else if (status == PRT_STATUS_EVENT_OVERFLOW) {
		//fprintf_s(stdout, "exiting with PRT_STATUS_EVENT_OVERFLOW\n");
		exit(1);
	} else if (status == PRT_STATUS_EVENT_UNHANDLED) {
		//fprintf_s(stdout, "exiting with PRT_STATUS_EVENT_UNHANDLED\n");
		exit(1);
	} else if (status == PRT_STATUS_QUEUE_OVERFLOW) {
		//fprintf_s(stdout, "exiting with PRT_STATUS_QUEUE_OVERFLOW \n");
		exit(1);
	} else if (status == PRT_STATUS_ILLEGAL_SEND) {
		//fprintf_s(stdout, "exiting with PRT_STATUS_ILLEGAL_SEND \n");
		exit(1);
	} else {
		fprintf_s(stdout, "unexpected PRT_STATUS in ErrorHandler: %d\n", status);
		exit(2);
	}
}

void Log(PRT_STEP step, PRT_MACHINESTATE *senderState, PRT_MACHINEINST *receiver, PRT_VALUE* event, PRT_VALUE* payload) {
	PrtPrintStep(step, senderState, receiver, event, payload);
}

void PRT_CALL_CONV MyAssert(PRT_INT32 condition, PRT_CSTRING message) {
    if (condition != 0) {
        return;
    } else if (message == NULL) {
        //fprintf_s(stderr, "ASSERT");
    } else {
        fprintf_s(stderr, "ASSERT: %s", message);
    }
    exit(1);
}

// Initializes our ROS program as well the P program used to execute the case study.
int main(int argc, char *argv[]) {
    PRT_DBG_START_MEM_BALANCED_REGION
    {
        PRT_GUID processGuid;
        PRT_VALUE *payload;

        // PrtInitialize(&P_GEND_PROGRAM);
        processGuid.data1 = 1;
        processGuid.data2 = 0;
        processGuid.data3 = 0;
        processGuid.data4 = 0;
        MAIN_P_PROCESS = PrtStartProcess(processGuid, &P_GEND_IMPL_DefaultImpl, ErrorHandler, Log);
        
        if (cooperative) {
            PrtSetSchedulingPolicy(MAIN_P_PROCESS, PRT_SCHEDULINGPOLICY_COOPERATIVE);
        }
        if (parg == NULL) {
            payload = PrtMkNullValue();
        } else {
            int i = atoi(parg);
            payload = PrtMkIntValue(i);
        }

        PrtUpdateAssertFn(MyAssert);
        PRT_UINT32 machineId;
        PRT_BOOLEAN foundMainMachine = PrtLookupMachineByName("RandomWalker", &machineId); // Sets up the P program, but starting at the Task Planner.
        
        if (foundMainMachine == PRT_FALSE) {
            printf("%s\n", "FAILED TO FIND DroneMachine");
            exit(1);
        }
       
        PrtMkMachine(MAIN_P_PROCESS, machineId, 1, &payload);

        PrtRunProcess(MAIN_P_PROCESS);
        PrtFreeValue(payload);
        PrtStopProcess(MAIN_P_PROCESS);
    }
    PRT_DBG_END_MEM_BALANCED_REGION
}
