#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
ros2 launch wafer_bringup benchmark.launch.py &
ROS_PID=$!
sleep 20
ffmpeg -nostdin -f x11grab -video_size 1920x1080 -framerate 15 -i :99 -t 15 -y /home/vansh/.gemini/antigravity-ide/brain/329e6daa-9c10-4968-a126-9fea9f6eb764/waferflow_demo.mp4
kill $ROS_PID
