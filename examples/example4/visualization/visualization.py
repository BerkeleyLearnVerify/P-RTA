#!/usr/bin/env python

import sys
import math
import socket
import ctypes
import struct
import turtle

IP, PORT = "192.168.1.133", 8080

class Kobuki(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float),
                ("y", ctypes.c_float),
                ("angle", ctypes.c_float)]

class Obstacle(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float),
                ("y", ctypes.c_float)]

class Machine(ctypes.Structure):
    _fields_ = [("robot", ctypes.c_int),
                ("motion_planner", ctypes.c_int),
                ("motion_primitives", ctypes.c_int),
                ("repair", ctypes.c_int)]

class PacketData(ctypes.Union):
    _fields_ = [("kobuki", Kobuki),
                ("obstacle", Obstacle),
                ("machine", Machine)]

class Packet(ctypes.Structure):
    _fields_ = [("packet_type", ctypes.c_int),
                ("packet_data", PacketData)]
    KOBUKI = 0
    OBSTACLE = 1
    MACHINE = 2

machine_text = ["Trusted Controller", "Untrusted Controller", "Trusted Obstacle Avoidance Controller"]
machine_color = ["red", "green", "orange"]

obstacle_turtle = turtle.Turtle()
obstacle_turtle.hideturtle()
obstacle_turtle.penup()
obstacle_turtle.speed(0)

obstacle_turtle.goto(120, 50)
obstacle_turtle.dot(48, "orange")

obstacle_turtle.goto(0, 50)
obstacle_turtle.dot(48, "green")

obstacle_turtle.goto(120, 170)
obstacle_turtle.dot(12, "blue")
obstacle_turtle.goto(-120, 170)
obstacle_turtle.dot(12, "blue")
obstacle_turtle.goto(-120, -70)
obstacle_turtle.dot(12, "blue")
obstacle_turtle.goto(120, -70)
obstacle_turtle.dot(12, "blue")

text_turtle = turtle.Turtle()
text_turtle.hideturtle()
text_turtle.penup()
text_turtle.speed(0)

text_turtle.color("black")
text_turtle.goto(-160, -280)
text_turtle.write("Robot:", False, "right", font=("Arial", 16, "normal"))
text_turtle.goto(-160, -300)
text_turtle.write("Motion Planner:", False, "right", font=("Arial", 16, "normal"))
text_turtle.goto(-160, -320)
text_turtle.write("Motion Primitives:", False, "right", font=("Arial", 16, "normal"))

kobuki_turtle = turtle.Turtle()
kobuki_turtle.shape("turtle")
kobuki_turtle.penup()
kobuki_turtle.speed(0)
kobuki_turtle.goto(0, 50)
kobuki_turtle.seth(0)
kobuki_turtle.speed(1)

turtle.setup(600, 700)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((IP, PORT))
    s.listen()
    print("Listening for connections...")
    conn, addr = s.accept()
    with conn:
        print("Connected with", addr)
        while True:
            data = conn.recv(ctypes.sizeof(Packet))
            if len(data) == 0:
                print("Disconnected")
                break
            packet = Packet()
            ctypes.memmove(ctypes.pointer(packet), data, ctypes.sizeof(packet))
            if packet.packet_type == Packet.KOBUKI:
                kobuki_turtle.seth(packet.packet_data.kobuki.angle / math.pi * 180)
                kobuki_turtle.goto(120 * packet.packet_data.kobuki.x, 120 * packet.packet_data.kobuki.y + 50)
            elif packet.packet_type == Packet.OBSTACLE:
                obstacle_turtle.goto(120 * packet.packet_data.obstacle.x, 120 * packet.packet_data.obstacle.y + 50)
                obstacle_turtle.dot(48, "red")
            elif packet.packet_type == Packet.MACHINE:
                text_turtle.fillcolor("white")
                text_turtle.begin_fill()
                text_turtle.goto(-150, -350)
                text_turtle.goto(300, -350)
                text_turtle.goto(300, -250)
                text_turtle.goto(-150, -250)
                text_turtle.end_fill()
                text_turtle.goto(-140, -280)
                text_turtle.color(machine_color[packet.packet_data.machine.robot])
                text_turtle.write(machine_text[packet.packet_data.machine.robot], False, "left", font=("Arial", 16, "normal"))
                text_turtle.goto(-140, -300)
                text_turtle.color(machine_color[packet.packet_data.machine.motion_planner])
                text_turtle.write(machine_text[packet.packet_data.machine.motion_planner], False, "left", font=("Arial", 16, "normal"))
                text_turtle.goto(-140, -320)
                text_turtle.color(machine_color[packet.packet_data.machine.motion_primitives])
                text_turtle.write(machine_text[packet.packet_data.machine.motion_primitives], False, "left", font=("Arial", 16, "normal"))
            # print(packet.packet_type)
            # print(packet.packet_data.obstacle.x)
            # print(packet.packet_data.obstacle.y)
            # print(packet.packet_data.kobuki.angle)
    while True:
        pass
