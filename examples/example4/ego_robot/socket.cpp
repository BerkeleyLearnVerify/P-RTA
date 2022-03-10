#include "socket.hpp"
#include <iostream>
#include <cstdio>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#define IP "192.168.1.133"
#define PORT 8080

enum PacketType {
    KOBUKI,
    OBSTACLE,
    MACHINE,
};

struct packet_t {
    PacketType packet_type;
    union {
        kobuki_t kobuki_pos;
        obstacle_t obstacle_pos;
        machine_t machine;
    } packet_data;
};

int sockfd;
struct sockaddr_in serv_addr;

// Returns true if connection successful, false if not
bool openConnection() {
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        std::cout << "Error opening socket" << std::endl;
        return false;
    }
    std::cout << "Socket opened" << std::endl;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    inet_pton(AF_INET, IP, &(serv_addr.sin_addr.s_addr));
    std::cout << "Attempting to connect to server" << std::endl;
    int success = connect(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    if (success < 0) {
        std::cout << "Error connecting to server" << std::endl;
        return false;
    } else {
        std::cout << "Connected to server" << std::endl;
        return true;
    }
}

void sendKobukiPosition(kobuki_t pos) {
    packet_t packet;
    packet.packet_type = KOBUKI;
    packet.packet_data.kobuki_pos = pos;
    send(sockfd, &packet, sizeof(packet), 0);
}

void sendObstaclePosition(obstacle_t pos) {
    packet_t packet;
    packet.packet_type = OBSTACLE;
    packet.packet_data.obstacle_pos = pos;
    send(sockfd, &packet, sizeof(packet), 0);
}

void sendMachine(machine_t machine) {
    packet_t packet;
    packet.packet_type = MACHINE;
    packet.packet_data.machine = machine;
    send(sockfd, &packet, sizeof(packet), 0);
}
