Docker ssh :

run ./docker_jazzy.sh on the laptop 


teleop :

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.21 -p turn:=0.43


nav2 :

ros2 launch interface anibot_full.launch.py map:=$HOME/maps/my_map.yaml


slam :

ros2 launch interface slam_full.launch.py



map_saver : 

ros2 run nav2_map_server map_saver_cli -f ~/maps/kitch --ros-args -p save_map_timeout:=10.0



This is a ROS2 diff drive navigation robot using raspberry pi 5

