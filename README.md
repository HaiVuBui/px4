# px4_playground
simple setup, code, and run of px4

## Add px4_msgs 

Tested with px4_msgs @ 18cbcce

```
git clone https://github.com/PX4/px4_msgs.git
```

## Launch PX4 - ROS in docker

Use this with mac (arm-based) - choose the correct one for diff CPU

```
docker run -p 6080:80 --security-opt seccomp=unconfined -v ${PWD}:/home/ubuntu/air_ws/src/ --name air-dev --shm-size=1g droneblocks/dexi-px4-sitl-arm64:1.15
```

for server

```
docker run -p 6080:80 --security-opt seccomp=unconfined -v ${PWD}:/home/ubuntu/air_ws/src/ --name air-dev --shm-size=1g droneblocks/dexi-px4-sitl-amd64:1.15
```

## Access over browser

http://localhost:6080

## Run PX4 SITL

```
cd /px4/PX4-Autopilot
make px4_sitl gz_x500
```

Other drones:gz_omnicopter, or gz_standard_vtol


## Turn on dds communication

```
MicroXRCEAgent udp4 --port 8888
```

## Build the package and launch it

```
cd ~/air_ws
colcon build
source install/setup.bash
```
## launch commands:
```bash
ros2 launch px4_takeoff takeoff.launch.py # take off
ros2 launch px4_takeoff mission.launch.py takeoff_height:=5.0 goal_x:=5.0 goal_y:=0.0 goal_z:=-5.0 goal_yaw:=0.0 # go to destinated point
```
