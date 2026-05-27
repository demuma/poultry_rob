# How to use the DIL/HAW Hamburg poultry detection and navigation system

## Docker container for Socket to ROS2 communication
1. Set network interface for ROS2 in *entrypoint.sh* or comment out when there is only one:

2. Build container from *src* with:

	> cd src
	> docker build -f ./poultry_rob_bridge/Dockerfile -t dil-ros2-humble:latest

3. Run docker container:

	> docker run --rm -it --network host -v /tmp:/tmp --name dil dil-ros2-humble:latest

## Run navigation planner

1. Clone repository:

	> git clone https://github.com/demuma/poultry_rob

2. Install missing dependencies

	> cd poultry_rob
	> rosdep install --from-paths src -y --ignore-src

3. Build the packages:

	> colcon build

4. Run navigation planner:

	> ros2 launch high_level_mission_planner mission_executor.launch.py
