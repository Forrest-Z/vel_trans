## Dependencies
```
  sudo apt-get install libusb-0.1-4
  sudo cp src/dt/core/driver/can_bus/lib/libusb.so /usr/lib
  sudo cp src/dt/core/driver/can_bus/lib/libusb-1.0.so /usr/lib
  sudo cp src/dt/core/driver/can_bus/lib/libECanVci.so.1 /usr/lib
  cd /usr/lib
  sudo ln -sv /usr/lib/libECanVci.so.1 /usr/lib/libECanVci.so
```



--topic
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
