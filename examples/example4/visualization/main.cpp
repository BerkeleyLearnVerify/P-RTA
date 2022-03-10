#include "socket.hpp"
#include <unistd.h>

int main() {
    openConnection();
    sendKobukiPosition({1.0, 1.0, 0.0});
    sleep(5);
    sendKobukiPosition({-1.0, 1.0, 3.14});
    sendObstaclePosition({1.0, -1.0});
    return 0;
}
