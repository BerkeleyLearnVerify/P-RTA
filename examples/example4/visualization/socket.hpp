struct kobuki_t {
    float x;
    float y;
    float angle;
};

struct obstacle_t {
    float x;
    float y;
};

bool openConnection();
void sendKobukiPosition(kobuki_t pos);
void sendObstaclePosition(obstacle_t pos);

