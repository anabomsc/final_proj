#!/bin/bash
# exp1.sh

TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")

set -e # stops execution in case of errors

cleanup() {

    echo ""
    echo "Cleaning up..."

    if [ -n "$BAG_PID" ]; then
        echo "Stopping rosbag..."

        kill -INT "$BAG_PID" 2>/dev/null || true

        wait "$BAG_PID" 2>/dev/null || true
    fi

    if [ -n "$SIM_PID" ]; then
        echo "Stopping simulation..."

        kill -TERM -- -"$SIM_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT

# Setup paths
WORKSPACE="$HOME/mono_ws"
PKG_DIR="$WORKSPACE/src/final_proj"

RESULTS_ROOT="$WORKSPACE/results"
RESULTS_DIR="$RESULTS_ROOT/$TIMESTAMP"

LOG_FILE="${RESULTS_ROOT}/${TIMESTAMP}/exp1.log"

# Create results directory
mkdir -p "$RESULTS_DIR"

# Logging setup
exec > >(tee -i "$LOG_FILE") 2>&1
echo "Logging to $LOG_FILE"

# ROS2 and workspace source
source /opt/ros/humble/setup.bash
source $WORKSPACE/install/setup.bash    

# 1. Starting Gazebo/ROS 2 simulation
echo ""
echo "Starting multi robot simulation $TIMESTAMP..."

setsid ros2 launch final_proj sim_multi_robot.launch.py \
    lidar_noise_mean:=0.0 lidar_noise_stddev:=0.0 & # simulation runs in background

SIM_PID=$! # stores background simulation process pid
echo "Simulation PID: $SIM_PID"

# 2. Waits 15s for the simulation to finish setup
echo "Waiting for Gazebo and ROS to finish setting up..."
sleep 15

# 3. Starts DiffDrive vs OdomPublisher pose translation
echo ""
echo "Starting OdomPublisher translation to DiffDrive reference (relative OdomPub)..."

ros2 run final_proj relative_odom_publisher.py & # pose translation runs in background

POSE_TRANSL_PID=$! # stores pose translation process pid
echo "Relative OdomPublisher PID: $POSE_TRANSL_PID"

# 4. Waits 4s for node to start publishing
echo "Waiting pose plugin translation node to finish setting up..."
sleep 4

# 5. Starting ros2bag recording
echo ""
echo "Starting rosbag recording..."

rm -rf "$RESULTS_DIR/bag" # removes existing rosbag

ros2 bag record \
    /artbot1/odom /artbot2/odom \
    /artbot1/gt_pose_rel /artbot2/gt_pose_rel \
    /tf /tf_static \
    -o "$RESULTS_DIR/bag" & # rosbag runs in background

BAG_PID=$! # stores background rosbag process pid
echo "Rosbag PID: $BAG_PID"

# 6. Waits 2s for rosbag to start recording
echo "Waiting for rosbag to start recording..."
sleep 2

# 7. Starting multi robot trajectory node exp1
echo ""
echo "Running experiment..."

ros2 run final_proj multi_robot_exp1.py # script waits for it to finish

echo "Experiment finished."

# 8. Stopping rosbag recording
echo ""
echo "Stopping rosbag..."

kill -INT $BAG_PID # kill the backgroung rosbag process

wait $BAG_PID # script waits for it to die

# 9. Waits 2s before result generation
echo "Waiting rosbag to stop before result generation..."
sleep 2

# 10. Generating individual/parcial maps from each robot 
echo ""
echo "Generating maps..."

ros2 service call artbot1/slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '${RESULTS_DIR}/artbot1_map_save'}}"
ros2 service call artbot2/slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '${RESULTS_DIR}/artbot2_map_save'}}"

# 11. Generating CSV and plots
echo ""
echo "Generating CSV..."

python3 $PKG_DIR/experiments/extract_pose_data.py "$RESULTS_DIR"

echo ""
echo "Generating plot..."

python3 $PKG_DIR/experiments/plot_pose_data.py "$RESULTS_DIR"

echo ""
echo "Finished."

echo "Trajectory image: $RESULTS_DIR/trajectory.png"
echo "Trajectory csv: $RESULTS_DIR/trajectory.csv"