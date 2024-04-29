
cd ~/uss_vis 

catkin_make

roscore &

sleep 5

cd ~/uss_vis

source devel/setup.bash

rosrun uss_vis_package audio.py
