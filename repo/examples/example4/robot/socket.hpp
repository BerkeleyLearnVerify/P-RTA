struct kobuki_t {
    float x;
    float y;
    float angle;
};

struct obstacle_t {
    float x;
    float y;
};

enum controller {
    TRUSTED,
    UNTRUSTED,
    AVOIDANCE
};

struct machine_t {
    controller robot;
    controller motion_planner;
    controller motion_primitives;
    controller repair;
};

bool openConnection();
void sendKobukiPosition(kobuki_t pos);
void sendObstaclePosition(obstacle_t pos);
void sendMachine(machine_t machine);

