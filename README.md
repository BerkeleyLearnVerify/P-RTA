# P-RTA

P-RTA is a framework for programming safe cyber-physical systems (CPS). This repository contains a toolbox for P-RTA. The toolbox comes with three major components, i.e., a small library for the Reelay monitoring tool, an extended P compiler for P to C compilation, and an extended C runtime backend for executing P-RTA programs. It also contains two example case studies for demonstration.

## Installation

### 1. Clone the repository

```
git clone https://github.com/BerkeleyLearnVerify/P-RTA.git
cd P-RTA
git submodule update --init --recursive
```

### 2. Build extended P compiler and backend

First, install [.Net SDK 3.1](https://dotnet.microsoft.com/download/dotnet/3.1) and Java Runtime. Then build P compiler as follows.

```
cd P/Bld
./build.sh
```

After a successful build, add printed aliases to your `bash_profile`. Build backend as follows.


```
./build-c-runtime.sh
```

### 3. Build Reelay

Build Reelay C++ library as instructed in this [link](https://github.com/doganulus/reelay/blob/master/docs/install.md).

## Case Studies

To run the case studies, you need to complete all steps listed above and install the [Webots](https://cyberbotics.com/) simulator. Additionally, you may need to change some lines in the `CMakeLists.txt` files in controller directories. Further instructions are given in the `CMakeLists.txt` files.

### 1. Case Study: Robot Surveillance System

Compile the P-RTA program for the robot.
```
cd examples/example1/controllers/robot
pc -generate:C robot.p
cmake CMakeLists.txt
make
```

Then open the world file `examples/example1/example1.wbt` in the Webots simulator.

Video available at [https://www.youtube.com/watch?v=-GzR69ljTU0](https://www.youtube.com/watch?v=-GzR69ljTU0)

### 2. Case Study: Complex Robot Surveillance System

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

## Writing and Executing P-RTA Programs

P-RTA builts on top of the P syntax. For details of the P syntax, see this [link](https://p-org.github.io/P/). The P-RTA syntax is given in the paper (under submission). For a quick review, you can check the case studies. P-RTA programs uses the same file extension as the P programs, i.e., `.p`. To compile P-RTA programs, we recommend using `CMakeLists.txt` files similar to ones given in the case studies. To run P-RTA programs, you need to provide a main function that starts the program with a number of worker threads, for details see `Main.cpp` files in the case studies.

## Syntax Highlighting in Sublime

In Sublime, click on `Preferences > Browse pages` and then copy `p-rta.tmLanguage` file the `Packages` folder. After this, you can select P-RTA from the syntax highlighting menu of Sublime.