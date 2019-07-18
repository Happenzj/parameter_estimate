# parameter_estimate
    Estimating robot chassis parameters


### Authors: Zheng Jie

### 1 Installation

#### 1.1 Required Dependencies

- Ceres :
  1:Before you install Ceres-solver, You need to install some dependencies: 
       sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3.1.2 libgflags-dev libgoogle-glog-dev libgtest-dev
  2:Install Ceres

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

#### 2 Usage

> roscore

> rosbag play " your bagname"

> rosrun parameter_estimate parameter_estimate


When the message is received, Run "Ctrl+c",The result will be printed in the terminal

a--Right wheel circumference

b--Left wheel circumference

c--wheelbase