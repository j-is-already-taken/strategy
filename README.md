# teamJ_software

## Usage
### install depends
```
sudo apt install build-essential cmake git libcamera-dev libopencv-dev libdrm-dev libboost-dev libboost-program-options-dev
```

### Camera Setting
```
v4l2-ctl -v width=640
v4l2-ctl -v height=480
```

### clone
```bash
git clone https://github.com/ros-perception/image_common.git -b humble
git clone https://github.com/ros-perception/vision_opencv.git -b humble
git clone https://github.com/j-is-already-taken/teamJ_software.git
git clone https://github.com/Dansato1203/picamera_ros2.git -b fix-streamroles-removal
git clone git@github.com:j-is-already-taken/MotorControl.git
```

### build
```
colcon build --symlink-install
```

### Run Program
1. Start Camera
   ```
   ros2 run picamera_ros2 picamera_pub_exec
   ```
3. Start Detection
   ```
   ros2 run teamJ_software detect_yellow
   ```
5. Start Action Server
   ```
   sudo pigpiod
   ros2 run omni_wheel omni_wheel_action_server
   ```
7. Start Action Client
   ```
   ros2 run omni_wheel omni_wheel_action_client
   ```
