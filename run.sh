cd ~/RTKLIB/app/consapp/str2str/gcc && ./str2str -in ntrip://rtk2go.com:2101/KANAME-ANX-RTCM3 -out serial://ttyACM0:230400
roslaunch ublox_gps ublox_device.launch node_name:=ublox param_file_name:=zed_f9p_rover
rosrun satfix_to_pose_py gps_sub.py
rosrun urg_node urg_node _serial_port:="/dev/ttyACM1"
rosrun laser_to_points laser_to_points 
rosrun limit_pc limit_pc 
rosrun my_pcl calc_area 
rviz
