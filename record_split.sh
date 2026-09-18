#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

echo "Starting simulation..."
ros2 launch wafer_bringup benchmark.launch.py &
ROS_PID=$!

echo "Waiting 35 seconds for initialization and robot movement to begin..."
sleep 35

echo "Tiling windows using xdotool..."
# Resize RViz to right half
for id in $(xdotool search --name "RViz" || true); do
  xdotool windowsize $id 960 1080
  xdotool windowmove $id 960 0
done

# Try various possible names for the Gazebo window, resize to left half
for id in $(xdotool search --name "Gazebo" || true); do
  xdotool windowsize $id 960 1080
  xdotool windowmove $id 0 0
done

for id in $(xdotool search --class "ign-gazebo" || true); do
  xdotool windowsize $id 960 1080
  xdotool windowmove $id 0 0
done

echo "Recording 15 seconds..."
ffmpeg -nostdin -f x11grab -video_size 1920x1080 -framerate 15 -i :99 -t 15 -y /home/vansh/.gemini/antigravity-ide/brain/329e6daa-9c10-4968-a126-9fea9f6eb764/waferflow_demo.mp4

echo "Taking screenshot..."
ffmpeg -nostdin -i /home/vansh/.gemini/antigravity-ide/brain/329e6daa-9c10-4968-a126-9fea9f6eb764/waferflow_demo.mp4 -ss 00:00:05 -vframes 1 /home/vansh/.gemini/antigravity-ide/brain/329e6daa-9c10-4968-a126-9fea9f6eb764/simulation_screenshot.png -y

echo "Done!"
