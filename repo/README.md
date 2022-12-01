# ULGEN: A Runtime Assurance Framework for Programming Safe Cyber-Physical Systems

ULGEN is a framework for programming safe cyber-physical systems (CPS). This repository contains a toolbox for ULGEN. The toolbox comes with three major components, i.e., a small library for the Reelay monitoring tool, an extended P compiler for compilation to executable C, and an extended C runtime backend for executing the compiled programs. It also contains four example case studies and their detailed explanations for demonstration.

## Installation

### 1. Build extended P compiler and backend

First, install [.Net SDK 3.1](https://dotnet.microsoft.com/download/dotnet/3.1) and Java Runtime. Then build the P compiler as follows. After a successful build, add printed aliases to your `.bash_profile`.

```
cd P/Bld
./build.sh
```

Build backend as follows.


```
./build-c-runtime.sh
```

### 2. Build Reelay (Optional)

If you want to use Reelay, build the Reelay C++ library as instructed in this [link](https://github.com/doganulus/reelay/blob/master/docs/install.md).

## Running Case Studies

To run the case studies, you need to complete all steps listed above and install the [Webots](https://cyberbotics.com/) simulator. Additionally, depending on your local machine, you may need to change some lines in the `CMakeLists.txt` files in controller directories. Further instructions are given in the `CMakeLists.txt` files.

### 1. Case Study: A Robot Surveillance System

Compile the ULGEN program for the robot.
```
cd examples/example1/controllers/robot
pc -generate:C robot.p
cmake CMakeLists.txt
make
```

Then open the world file `examples/example1/example1.wbt` in the Webots simulator.

### 2. Case Study: A More Complex Robot Surveillance System

Compile the ULGEN program for secondary robots.

```
cd examples/example2/controllers/secondary
pc -generate:C secondary.p
cmake CMakeLists.txt
make
```


Compile the ULGEN program for the ego robot.

```
cd examples/example2/controllers/robot
pc -generate:C robot.p
cmake CMakeLists.txt
make
```

Then open the world file `examples/example2/example2.wbt` in the Webots simulator.

### 3. Case Study: A Robot Surveillance System with Kobuki

This case study is implemented for and tested on the iClebo Kobuki robot base controlled with a Raspberry Pi 3 Model B+ running a 64-bit Ubuntu Server 20.04 LTS. Follow the instructions at this [link](https://kobuki.readthedocs.io/en/devel/software.html) to set up the standalone Kobuki core on your Raspberry Pi. Then compile the ULGEN program to executable C on your development machine as follows.

```
cd examples/example3/robot
pc -generate:C robot.p motion_planner.p motion_primitives.p
```

For fast development and testing, you can place the generated `robot.c` and `robot.h` files in the `src/demos` directory of the Kobuki standalone core, and after modifying the `CMakeLists.txt` files, you can simply build the Kobuki core just like presented in this [link](https://kobuki.readthedocs.io/en/devel/software.html). This will generate an executable for the case study among other demos of the Kobuki core.

As a better development practice, you can also directly compile the executable C code of the case study on your Raspberry Pi just by linking the Kobuki standalone core libraries. Notice that after compiling the ULGEN program to executable C on your development machine, you can treat this C program just like any other C program; therefore, you are free to compile it in any way that is suitable to your setup.

To run the case study, make sure that have the same IP addresses in the files `robot/socket.cpp` and `visualization/visualization.py`. Then run the `visualization.py` script on your server, and then run the compiled executable of the case study on the Kobuki robot.

### 4. Case Study: A Robot Exploration System with Kobuki

This case study is implemented for and tested on the iClebo Kobuki robot base controlled with a Raspberry Pi 3 Model B+ running a 64-bit Ubuntu Server 20.04 LTS. Follow the instructions at this [link](https://kobuki.readthedocs.io/en/devel/software.html) to set up the standalone Kobuki core on your Raspberry Pi. Then compile the ULGEN program to executable C on your development machine as follows.

```
cd examples/example3/robot
pc -generate:C robot.p motion_planner.p motion_primitives.p
```

For fast development and testing, you can place the generated `robot.c` and `robot.h` files in the `src/demos` directory of the Kobuki standalone core, and after modifying the `CMakeLists.txt` files, you can simply build the Kobuki core just like presented in this [link](https://kobuki.readthedocs.io/en/devel/software.html). This will generate an executable for the case study among other demos of the Kobuki core.

As a better development practice, you can also directly compile the executable C code of the case study on your Raspberry Pi just by linking the Kobuki standalone core libraries. Notice that after compiling the ULGEN program to executable C on your development machine, you can treat this C program just like any other C program; therefore, you are free to compile it in any way that is suitable to your setup.

To run the case study, make sure that have the same IP addresses in the files `robot/socket.cpp` and `visualization/visualization.py`. Then run the `visualization.py` script on your server, and then run the compiled executable of the case study on the Kobuki robot.

## Details of Case Studies

Below, we present the details of the machines implemented in the case studies.

### 1. Case Study: A Robot Surveillance System

#### Robot
This machine ensures that the robot visits given waypoints in the correct order. It has two states: `Init` and `Run`. `Init` initializes local variables. Then a transition is taken to `Run` where an event-driven RTA module sends the next waypoint if the robot reaches the previous one. The RTA module is triggered when motion primitives sends the current location. The decision module checks this location to see if it is the previous waypoint. If so, the RTA module executes its trusted controller which sends the next waypoint to the motion planner. If not, i.e., it is an intermediate waypoint computed by the motion planner, the RTA module executes the untrusted controller which does not do anything.

#### Motion Planner
This machine computes safe motion plans from one location to another. It also has two states: `Init` and `Run`. `Run` implements an event-driven RTA module for computing motion plans. The decision module selects the trusted controller if the robot is near the walls; otherwise, it selects the untrusted controller. The trusted controller computes detailed motion plans whereas the untrusted controller computes less detailed motion plans. Computed plans are sent to the machine specified in the event requesting the motion plan. Notice that in a more complicated environment, a similar design can be applied, e.g., a drone can use a trusted controller near a lake and can use an untrusted controller in open fields.

#### Motion Primitives (Plan Executor)
This machine guarantees that the robot closely follows a trajectory between waypoints. It has three states: `Init`, `Run`, and `LowBatteryRun`. `Run` and `LowBatteryRun` implement time-driven RTA modules with an untrusted and a trusted controller. In both states, upon reaching a waypoint, the current location is sent to the robot. For evaluation, untrusted controllers are implemented as random controllers, i.e., the controller turns left with a probability of 20% or moves forward with a probability of 80%. Since we increment the virtual time of the simulator with 16 ms steps, controllers of time-driven RTA modules have 16 ms periods.

After `Init`, the machine transitions to `Run` where it receives motion plans and stores them in an array so that the next time the RTA module runs a controller, they can be read from this array. When it is time for the execution of the RTA module, the decision module selects the proper controller by checking the deviation from the ideal trajectory, which is performed by a Reelay monitor. If the deviation is historically more than the threshold in the last ten steps, the trusted controller executes; otherwise, the untrusted controller executes. The decision frequencies are $50$ and $10$ for trusted and untrusted controllers, respectively. The difference between decision frequencies gives an advantage to the trusted controller for recovery.

In `Run`, if the machine receives an event indicating a low battery, it responds with the current location, saves its goal, and transitions to `LowBatteryRun` where it only accepts high-priority events. The Reelay monitor of this RTA module also checks the trajectory deviation, but with a lower threshold. For trusted and untrusted controllers, decision frequencies are 100 and 10, respectively, to emphasize that the robot is in an emergency mode and it is more important to reach the charger than to execute the untrusted controller. If the machine receives an event indicating the recovered battery, it responds with its previous goal so that a high priority plan can be computed and transitions to `Run` where all plans are served.

#### Battery (Battery Observer)
This machine keeps the battery at a safe level. It has two states: `Init` and `Run`. `Run` implements a time-driven RTA module of three controllers with 1 s periods. The decision module uses two Reelay monitors. The first monitor raises a flag when the battery level drops below 20%. Then the first trusted controller sends a high priority event to Motion Primitives announcing the low battery. Motion Primitives responds with the current location. Upon receiving the current location, Battery requests a motion plan from this location to the charger and forwards the response to Motion Primitives with high priority events. The second monitor raises a flag when the battery level exceeds 200. Then the second trusted controller sends a high-priority event to Motion Primitives announcing the recovered battery, to which Motion Primitives responds with its previous goal. Battery requests a plan to this location and forwards it to Motion Primitives with a high priority event.% The untrusted controller is an empty controller; however, in a more complex setting, it can be designed to lead the robot to the charger using a heuristic method. Notice that implementing this procedure with high-priority events ensures that the Motion Primitives machine will reach its previous goal.

##### Remark
Implementing trusted and untrusted controllers is not in the scope of this paper. Therefore, we implement untrusted controllers either with an empty controller, i.e., a controller not doing anything, or a random controller and we implement trusted controllers ourselves without proving their safety. Notice that using empty or random controllers as untrusted controllers and still guaranteeing safety demonstrates the fault-tolerant behavior of our framework, which we present next via rigorous systematic testing.

### 2. Case Study: A More Complex Robot Surveillance System

#### Secondary Robot
The task of the secondary robots is to avoid collisions. These robots are implemented with one machine and instantiated. The machine has two states: `Init` and `Run`. `Run` implements a time-driven RTA module of two controllers with 16 ms periods (since we increment the simulator time with 16 ms). The untrusted controller moves the robot forward, and the trusted controller rotates the robot. The decision module selects between the two using the distance sensor. If the sensor readings are greater than a threshold, the trusted controller executes; otherwise, the untrusted controller executes.

#### Robot
This machine is identical to the implementation presented in the previous case study.

#### Motion Planner
This machine is also the same as the implementation given in the previous case study. However, for this task, it sends computed motion plans to one of the Geo-Fence machines along with the destination of the plan. Moreover, it can receive high priority motion plan requests which are served to the destination with high priority motion plan events. We made this modification to demonstrate a different design paradigm where high-priority requests are served with high-priority responses.

#### Geo-Fence
The Geo-Fence machine has `Init` and `Run` states. `Run` implements an event-driven RTA module triggered by events carrying motion plans. If the received motion plan intersects with the protected region, then the trusted controller executes and replaces waypoints in the received motion plan with new waypoints that are going around the protected region and sends this new motion plan. Otherwise, it executes the untrusted controller which forwards the received motion plan. We implement a generic machine protecting a region in 2D space. Since there are two geo-fenced regions in the case study, the robot creates one Geo-Fence machine per region.


#### Obstacle Avoidance
This machine also has `Init` and `Run` states. `Run` implements an event-driven RTA module triggered by events carrying motion plans. If the received motion plan has waypoints intersecting with an obstacle, the trusted controller removes these waypoints from the plan and sends the rest of the waypoints to the collision-aware motion primitives machine. Otherwise, the untrusted controller directly forwards the motion plan. robot initializes this machine with obstacle coordinates.

#### Collision Aware Motion Primitives (Collision Aware Plan Executor)
It is similar to the motion primitives machine presented in the previous case study. However, instead of having two controllers in the RTA modules of `Run` and `LowBatteryRun`, it has an additional trusted controller that executes when a collision threat is present. To detect a threat, it has a second Reelay monitor which raises a flag if the values of distance sensor readings are historically more than a threshold in the last 10 time steps. Such a temporal specification for the online monitor is useful to distinguish between actual collision threats and momentary sensor errors that are not actually collision threats. If the flag is raised, then the collision avoidance controller rotates the robot to the opposite direction of the threat and moves forward. In both RTA modules, the decision frequency of the collision avoidance controller is 100 to give enough execution time to the controller for recovery. We decrease the decision frequencies of other controllers to 1 (for the untrusted controllers) and 10 (for the trusted controllers) so that the collision avoidance controller can execute when needed.


#### Battery (Battery Observer)
This machine is the same as the implementation given in the previous case study. The only difference is that instead of receiving motion plans from the motion planner and forwarding them, it instructs the motion planner to send computed motion plans to the collision-aware motion primitives machine with a high priority event. The reason for this modification is to present an alternative design emphasizing the reactive nature of ULGEN machines.

#### Position/Orientation Correction
Position correction and orientation correction machines implement similar logics, where one does the correction for position and the other does it for orientation. Both have `Init` and `Run` states. `Run` states implement time-driven RTA modules of two controllers with 1 s periods. Their decision modules use Reelay monitors to decide which controller to run. If the position/orientation estimation is deviated from the GPS/compass reading more than the given threshold in the last two time steps, the monitor raises a flag and the trusted controller sends the correct position/orientation value to collision-aware motion primitives for correction via a high priority event. Otherwise, the untrusted controller is selected which does not do anything.

### 3. Case Study: A Robot Surveillance System with Kobuki

#### Robot
The robot machine includes a trusted controller and an untrusted controller. The decision module selects the trusted controller if the robot reaches a major waypoint and selects the untrusted controller if the robot reaches an intermediate waypoint. The trusted controller sends the next major waypoint to the motion planner after acknowledging that the previous waypoint is visited and the untrusted controller does not do anything. Notice that one can implement a different untrusted controller that optimizes some metrics (instead of having an empty controller); however, for our case study, this was not necessary.


#### Motion Planner
The motion planner machine uses the Open Motion Planning Library (OMPL) to plan the route. The untrusted controller computes the shortest path from the current location to the next waypoint if there are no potential obstacles in between, whereas the trusted controller considers the clearance of the path. The path is considered clear if there are no potential obstacles within a certain distance from the shortest trajectory (a straight line). Formally, the trusted controller is executed if 

<img src="https://render.githubusercontent.com/render/math?math=\text{min}_{L_{ob} \in \emph{PotObs}}\ \emph{DistFromTrajectory}(L_{\emph{curr}}, L_{\emph{goal}}, L_{\emph{ob}}) < 2R">

, where <img src="https://render.githubusercontent.com/render/math?math=PotObs"> is the set of potential obstacles previously sensed by the robot, <img src="https://render.githubusercontent.com/render/math?math=L_{\emph{curr}}"> and <img src="https://render.githubusercontent.com/render/math?math=L_{\emph{goal}}"> are the current location and the next waypoint, <img src="https://render.githubusercontent.com/render/math?math=R"> is the radius of the robot, and <img src="https://render.githubusercontent.com/render/math?math=DistFromTrajectory(L_0, L_1, L)"> computes the distance from <img src="https://render.githubusercontent.com/render/math?math=L"> to the line connecting <img src="https://render.githubusercontent.com/render/math?math=L)0"> and <img src="https://render.githubusercontent.com/render/math?math=L_1">. Plans generated by the untrusted controller are optimized with respect to path length, while those generated by the trusted controller are optimized with respect to both path length and path clearance (inversely proportional to the minimum distance from potential obstacles to the ideal trajectory). 

#### Motion Primitives (Plan Executor)
Given the current location and the next waypoint, the motion primitives machine controls the movement of the robot. We implemented three controllers for the machine: (1) a trusted motion controller, (2) an untrusted motion controller, and (3) a trusted obstacle avoidance controller. 


The trusted motion controller is executed when the robot deviates from the planned trajectory by more than 20 cm when the robot is not heading towards the next waypoint, and the robot is close to a potential obstacle (potential obstacles are defined below). It controls the robot to correct its orientation to face the next waypoint and stay on the trajectory. If both conditions are satisfied, the untrusted motion controller, which is implemented as a PID controller, is executed. 

The PID controller works to maintain a consistent orientation and speed towards the next waypoint. The proportional component of the PID controller encourages the Kobuki to rotate faster when its orientation is far off from its target and increase its speed when the next waypoint is distant. The derivative component of the PID controller prevents the Kobuki from accelerating or rotating too quickly so that its movements don't overshoot their intended target. The integral component of the PID controller accumulates the error over time, allowing the Kobuki to slowly increase its speed if it is not making sufficient progress towards its next waypoint or slowly increase its turning speed if it is struggling to reach its desired orientation.

The trusted obstacle avoidance controller is executed when the robot senses a bump or when the robot is entering geofence regions. When bumping into an unknown obstacle, we register it as a potential obstacle and record its location, since it might be a dynamic obstacle and will not appear at the same location in the next round. Upon bumping into an unknown obstacle that is near to a potential obstacle location, we registered both coordinates as avoiding locations.

### 4. Case Study: A Robot Exploration System with Kobuki

#### Robot
This machine is identical to the machine presented in the previous case study.


#### Motion Planner
This machine is identical to the machine presented in the previous case study except that it uses its trusted controllers in unknown regions of the environment.


#### Motion Primitives (Plan Executor)
This machine is identical to the machine presented in the previous case study except that it uses its trusted controllers in unknown regions of the environment.


## Writing and Executing ULGEN Programs

ULGEN builts on top of the P syntax. For details of the P syntax, see this [link](https://p-org.github.io/P/). The ULGEN syntax is given in the paper (under submission). For a quick review, you can check the case studies. ULGEN programs use the same file extension as the P programs, i.e., `.p`. To compile ULGEN programs, we recommend using `CMakeLists.txt` files similar to the ones given in the case studies. To run ULGEN programs, you need to provide the main function that starts the program with several worker threads, for details see `Main.cpp` files in the case studies.

## Syntax Highlighting in Sublime

In Sublime, click on `Preferences > Browse pages` and then copy `ulgen.tmLanguage` file to the `Packages` folder. After this, you can select ULGEN from the syntax highlighting menu of Sublime.
