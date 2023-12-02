# Walking_Turtle

## Prerequisites
```
sudo apt install cppcheck cpplint doxygen ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg \
prefix turtlebot3_gazebo \
`/share/turtlebot3_gazebo/models/
```

## Installing Package
```
cd <workspace>/src
git clone https://github.com/1412kauti/Walking_Turtle.git
cd ..
colcon build
```

## Running Package
```
ros2 launch turtlebot_walker walking_turtle.launch.py record_bag:=true
```

## Bag Info
```
ros2 bag info custom_bag
```

