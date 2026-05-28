# How to use the DIL/HAW Hamburg interface and navigation system

## Run navigation planner
1. Clone repository:

	```bash
	git clone https://github.com/demuma/poultry_rob
 	```

2. Install missing dependencies:

	```bash
	cd poultry_rob
	rosdep install --from-paths src -y --ignore-src
 	```

3. Build the packages:

	```bash
	colcon build
 	```

4. Run navigation planner:

	```bash
	ros2 launch high_level_mission_planner mission_executor.launch.py
 	```

## Docker container for Socket to ROS2 communication
1. Set network interface for ROS2 in *entrypoint.sh* at line 15 or comment out when there is only one interface:

	```bash
 	cd poultry_rob/src/poultry_rob_bridge
 	```

3. Build container from *src* with:

	```bash
	cd src
	docker build -f ./poultry_rob_bridge/Dockerfile -t dil-ros2-humble:latest
 	```

4. Run docker container:

	```bash
	docker run --rm -it --network host -v /tmp:/tmp --name dil dil-ros2-humble:latest
 	```
