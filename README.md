
ssh -L 9091:localhost:9091 -E /dev/null nvidia@192.168.90.231

For SLAM - 


Run ssh and then split terminals by using ./run.sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091

ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/nvidia/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml

ros2 launch f1tenth_stack bringup_launch.py

launch foxglove 9091

ros2 run nav2_map_server map_saver_cli -f "map" 

Change map location to inside
sudo mv map* /home/nvidia/f1tenth_ws/src/particle_filter/maps/
Run ./rem_particle.sh

Once, you have the map, then we need the centerline

To run waypoint logger

ros2 launch f1tenth_stack bringup_launch.py
ros2 run pure_pursuit waypoint_logger.py


Now, that path file should be in /home/nvidia/

now run 

cd Desktop
./f1.sh

Now, you have the checkpoints

To run individual nodes

ros2 launch f1tenth_stack bringup_launch.py
ros2 launch particle_filter localize_launch.py
ros2 run pure_pursuit pure_pursuit_node



