# P-RTA

P-RTA is a framework for programming safe cyber-physical systems (CPS). This repository contains a toolbox for P-RTA. The toolbox comes with three major components, i.e., a small library for the Reelay monitoring tool, an extended P compiler for P to C compilation, and an extended C runtime backend for executing P-RTA programs. It also contains two example case studies for demonstration.

## Installation

### 1. Clone the repo

```
git clone https://github.com/BerkeleyLearnVerify/P-RTA.git
cd P-RTA
git submodule update --init --recursive
```

### 2. Build extended P compiler

Install [.Net SDK 3.1.](https://dotnet.microsoft.com/download/dotnet/3.1)

```
cd P/Bld
./build.sh
```

After a successful build, add printed lines to your `bash_profile`.

### 3. Build extended P backend

```
./build-c-runtime.sh
```

### 4. Build Reelay

Build Reelay C++ library as instructed in this [link](https://github.com/doganulus/reelay/blob/master/docs/install.md).

## Case Studies

To run the case studies, you need to complete all steps listed above and install the [Webots](https://cyberbotics.com/) simulator. Additionally, you may need to change some lines in the `CMakeLists.txt` files in controller directories. Further instructions are given in the `CMakeLists.txt` files.

### 1. Case Study: Robot Surveillance Task

Compile the P-RTA program for the robot.
```
cd examples/example1/controllers/robot
pc -generate:C robot.p
cmake CMakeLists.txt
make
```

Then open the world file `examples/example1/example1.wbt` in the Webots simulator.

Video available at [https://www.youtube.com/watch?v=-GzR69ljTU0](https://www.youtube.com/watch?v=-GzR69ljTU0)

### 2. Case Study: A More Complex Robot Surveillance Task

Compile the P-RTA program for slave robots.

```
cd examples/example2/controllers/slave
pc -generate:C slave.p
cmake CMakeLists.txt
make
```


Compile the P-RTA program for the ego robot.

```
cd examples/example2/controllers/robot
pc -generate:C robot.p
cmake CMakeLists.txt
make
```

Then open the world file `examples/example2/example2.wbt` in the Webots simulator.

Video available at [https://www.youtube.com/watch?v=B5AZ5Web-wc](https://www.youtube.com/watch?v=B5AZ5Web-wc)
