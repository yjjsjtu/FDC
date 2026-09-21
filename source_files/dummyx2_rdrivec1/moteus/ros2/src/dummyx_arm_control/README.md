**install**
```shell
    cd dummyx/ros2/dummyx_ws
    colcon build
    source install/setup.bash
    sudo apt-get install ros-humble-controller-manager
    sudo apt-get install ros-humble-joint-state-broadcaster
    sudo apt-get install ros-humble-ros2-control ros-humble-ros2-controllers
```

**open terminal**
```shell
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 txqueuelen 5000
sudo ip link set can0 up
ros2 launch dummyx-moveit-config demo.launch.py
```

**open another terminal**
```shell
ros2 run dummyx_usb2can usb2can_node
```

