# parameter_estimate
    Estimating robot chassis parameters


### Authors: Zheng Jie

### 1 Installation

#### 1.1 Required Dependencies

- Before you install Ceres-solver, You need to install some dependencies: 

> sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.2 libgflags-dev libgoogle-glog-dev libgtest-dev

- Install Ceres

> git clone : https://github.com/ceres-solver/ceres-solver

> cd ceres-solver

> mkdir build 

> cd build 

> cmake ..

> make

> sudo make install


#### 1.2 Build

> cd catkin_ws/src

> git clone https://github.com/Happenzj/parameter_estimate

> cd ..

> catkin_make 

### 2 usage

> roscore

> rosbag play " your bagname"

> rosrun parameter_estimate parameter_estimate


When the message is received, Run "Ctrl+c",The result will be printed in the terminal

    a--Right wheel circumference

    b--Left wheel circumference

    c--wheelbase

### 2 experiment

> roscore

> cd catkin_ws/src/parameter_estimate/dataset

> rosbag play 2019-06-17-16-47-35.bag

> cd catkin_ws

> rosrun parameter_estimate parameter_estimate