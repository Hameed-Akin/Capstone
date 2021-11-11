# Capstone

Playing back a bag
`rosbag play -r 0.1 --clock wool.bag`

RUN CAMERA_NBG
rosrun camera_nbg camera_nbg_node _file_path:=/home/hameed/Capstone/20210831_dataset/table.ply _topic:=/table _frame:=camera_color_optical_frame _rate:=1.0

RUN extract_hands 
rosrun openpose_parser extract_hands

Rviz to visualize
