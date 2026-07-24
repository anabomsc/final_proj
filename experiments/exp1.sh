#!/bin/bash
# exp1.sh

set -e # stops execution in case of errors

LIDAR_NOISE_MEAN=${1:-0.0}
LIDAR_NOISE_STDDEV=${2:-0.0}
SESSION_DIR=${3:-"$HOME/mono_ws/results"}

TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")

stop_process_group(){
    local pid=$1
    local name=$2

    [ -z "$pid" ] && return

    echo "Stopping $name..."

    kill -INT -- -"$pid" 2>/dev/null || true

    for i in {1..20}; do
        if ! kill -0 "$pid" 2>/dev/null; then
            return
        fi
        sleep 0.5
    done

    echo "$name did not stop, forcing..."

    kill -TERM -- -"$pid" 2>/dev/null || true
    sleep 2
    kill -KILL -- -"$pid" 2>/dev/null || true
}

cleanup() {

    echo ""
    echo "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
    echo "Experiment Cleanup"
    echo ""

    stop_process_group "$MAP_FUSE" "map fusion"
    stop_process_group "$MAP_ALIGN" "map alignment"
    stop_process_group "$POSE_TRANSL_PID" "pose translation"
    stop_process_group "$SIM_PID" "simulation"

    echo "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
    echo ""
}

trap cleanup EXIT

# Setup paths
WORKSPACE="$HOME/mono_ws"
PKG_DIR="$WORKSPACE/src/final_proj"

# Create results directory
EXPERIMENTS_DIR="$SESSION_DIR/$TIMESTAMP"
mkdir -p "$EXPERIMENTS_DIR"

# Logging and session index
# LOG_FILE="${SESSION_DIR}/exp1.log"
# exec > >(tee -i "$LOG_FILE") 2>&1

# ROS2 and workspace source
source /opt/ros/humble/setup.bash
source $WORKSPACE/install/setup.bash    

## Session information: lidar noise and dir
echo ""
echo "-------------- EXPERIMENT INFO ----------------"
echo "Timestamp: $TIMESTAMP"
echo "Lidar noise mean: $LIDAR_NOISE_MEAN"
echo "Lidar noise std dev: $LIDAR_NOISE_STDDEV"
echo "Experiment results directory: $EXPERIMENTS_DIR"
echo "-----------------------------------------------"

## Starting Gazebo/ROS 2 simulation
echo ""
echo "Starting multi robot simulation $TIMESTAMP..."

setsid ros2 launch final_proj sim_multi_robot.launch.py \
    lidar_noise_mean:=${LIDAR_NOISE_MEAN} \
    lidar_noise_stddev:=${LIDAR_NOISE_STDDEV} & # simulation runs in background

SIM_PID=$! # stores background simulation process pid
echo "Simulation PID: $SIM_PID"

## Waits 15s for the simulation to finish setup
echo "Waiting for Gazebo and ROS to finish setting up..."
sleep 15

## Starts DiffDrive vs OdomPublisher pose translation
echo ""
echo "Starting OdomPublisher translation to DiffDrive reference..."

setsid ros2 run final_proj plugin_reference_translation.py & # pose translation runs in background

POSE_TRANSL_PID=$! # stores pose translation process pid
echo "Relative OdomPublisher PID: $POSE_TRANSL_PID"

## Waits 4s for node to start publishing
echo "Waiting pose plugin translation node to finish setting up..."
sleep 4

## Starting ros2bag recording
echo ""
echo "Starting rosbag recording..."

rm -rf "$EXPERIMENTS_DIR/bag" # removes existing rosbag

ros2 bag record \
    /artbot1/odom /artbot2/odom \
    /artbot1/gt_pose_rel /artbot2/gt_pose_rel \
    /tf /tf_static \
    -o "$EXPERIMENTS_DIR/bag" & # rosbag runs in background

BAG_PID=$! # stores background rosbag process pid
echo "Rosbag PID: $BAG_PID"

## Waits 2s for rosbag to start recording
echo "Waiting for rosbag to start recording..."
sleep 2

## Starting multi robot trajectory node exp1
echo ""
echo "Running experiment..."

ros2 run final_proj multi_robot_exp1.py # script waits for it to finish

echo "Experiment finished."

## Stopping rosbag recording
echo ""
echo "Stopping rosbag..."

kill -INT $BAG_PID # kill the backgroung rosbag process

wait $BAG_PID # script waits for it to die
unset BAG_PID

## Waits 2s before result generation
echo "Waiting rosbag to stop before result generation..."
sleep 2

## Generating individual/partial maps from each robot 
echo ""
echo "Saving local maps..."

ros2 service call artbot1/slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '${EXPERIMENTS_DIR}/artbot1_map_save'}}"
ros2 service call artbot2/slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '${EXPERIMENTS_DIR}/artbot2_map_save'}}"

## Generate aligned maps
echo ""
echo "Generating Aligned Maps..."

ros2 run final_proj map_alignment.py --ros-args -p use_sim_time:=true &

MAP_ALIGN=$! # stores map alignment process pid
echo "Map Alignment PID: $MAP_ALIGN"

## Waits 4s for node to start publishing
echo "Waiting map_alignment node to finish setting up..."
sleep 4

## Generate merged map
echo ""
echo "Generating Merged Map..."

ros2 run final_proj map_fusion.py --ros-args -p use_sim_time:=true &

MAP_FUSE=$! # stores map alignment process pid
echo "Map Fusion PID: $MAP_FUSE"

## Waits 4s for node to start publishing
echo "Waiting map_fusion node to finish setting up..."
sleep 4

## Generating CSV and plots
echo ""
echo "Generating CSV..."

python3 $PKG_DIR/experiments/extract_pose_data.py "$EXPERIMENTS_DIR"

echo ""
echo "Generating Plot Data..."

python3 $PKG_DIR/experiments/plot_pose_data.py "$EXPERIMENTS_DIR"

## Generating global merged map
echo ""
echo "Saving merged map..."

ros2 run nav2_map_server map_saver_cli -f ${EXPERIMENTS_DIR}/merged_map_save -t /merged_map

echo ""
echo "Experiment $TIMESTAMP Pipeline Completed"

echo "Trajectory image file saved to: $EXPERIMENTS_DIR/trajectory.png"
echo "Trajectory csv file saved to: $EXPERIMENTS_DIR/trajectory.csv"
echo "Relative pose file saved to: $EXPERIMENTS_DIR/relative_poses.json"
echo "---------------------------------------------"
echo ""