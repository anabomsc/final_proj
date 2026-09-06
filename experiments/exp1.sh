#!/bin/bash
# exp1.sh

set -e # stops execution in case of errors

LIDAR_NOISE_STDDEV=${1:-0.0}
ODOM_NOISE_STDDEV=${2:-0.0}
DIST_NOISE_STDDEV=${3:-0.0}
THETA_NOISE_STDDEV=${4:-0.0}
SESSION_DIR=${5:-"$HOME/mono_ws/results"}

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

    stop_process_group "$MAP_MERGE" "Map Merging"
    stop_process_group "$POSE_TRANSL_PID" "Pose Plugin Translation"
    stop_process_group "$SLAM_POSE_PUB_PID" "SLAM Pose Pub"
    stop_process_group "$SIM_PID" "Simulation"

    echo "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
    echo ""
}

trap cleanup EXIT

save_map_retry() {
    local cmd="$1"
    local map_base="$2"
    local description="$3"
    local max_attempts="${4:-10}"
    local wait_time="${5:-5}"

    local attempt=1

    while [ $attempt -le $max_attempts ]; do
        echo "$description (attempt $attempt/$max_attempts)..."

        rm -f "${map_base}.yaml" "${map_base}.pgm"

        if eval "$cmd"; then
            if [ -f "${map_base}.yaml" ] && [ -f "${map_base}.pgm" ]; then
                echo "$description saved successfully."
                return 0
            fi
        fi

        echo "$description failed."

        if [ $attempt -lt $max_attempts ]; then
            echo "Retrying in ${wait_time}s..."
            sleep "$wait_time"
        fi

        attempt=$((attempt + 1))
    done

    echo "ERROR: Could not save $description after $max_attempts attempts."
    return 1
}

# Setup paths
WORKSPACE="$HOME/mono_ws"
PKG_DIR="$WORKSPACE/src/final_proj"

# Create results directory
EXPERIMENTS_DIR="$SESSION_DIR/EXP01_${TIMESTAMP}"
mkdir -p "$EXPERIMENTS_DIR"

# ROS2 and workspace source
source /opt/ros/humble/setup.bash
source $WORKSPACE/install/setup.bash    

## Session information: lidar/alignment noise and dir
echo ""
echo "------------ EXPERIMENT 01 INFO ---------------"
echo "Timestamp: $TIMESTAMP"
echo "Lidar noise std dev: $LIDAR_NOISE_STDDEV"
echo "Odom noise std dev: $ODOM_NOISE_STDDEV"
echo "Alignment distance noise std dev: $DIST_NOISE_STDDEV"
echo "Alignment theta noise std dev: $THETA_NOISE_STDDEV"
echo "Experiment results directory: $EXPERIMENTS_DIR"
echo "-----------------------------------------------"

## Starts DiffDrive vs OdomPublisher pose translation
echo ""
echo "Starting OdomPublisher translation to DiffDrive reference..."

setsid ros2 run final_proj plugin_reference_translation.py & # pose translation runs in background

POSE_TRANSL_PID=$! # stores pose translation process pid
echo "Relative OdomPublisher PID: $POSE_TRANSL_PID"

## Waits 4s for node to start publishing
echo "Waiting pose plugin translation node to finish setting up..."
sleep 4

## Starting Gazebo/ROS 2 simulation
echo ""
echo "Starting multi robot simulation $TIMESTAMP..."

setsid ros2 launch final_proj sim_multi_robot.launch.py \
    lidar_noise_stddev:=${LIDAR_NOISE_STDDEV} \
    odom_noise_stddev:=${ODOM_NOISE_STDDEV} \
    environment:="ambiente_0.launch.py"\
    robots_positions:="(-4.0,-2.0,3.1384),(13.5,1.0,3.1384)" & # simulation runs in background

SIM_PID=$! # stores background simulation process pid
echo "Simulation PID: $SIM_PID"

## Waits for the simulation to finish setup
echo "Waiting for Gazebo and ROS to finish setting up..."
sleep 10

## Starts SLAM pose publisher
echo ""
echo "Starting SLAM pose publisher..."

setsid ros2 run final_proj slam_pose_publisher.py & # runs in background

SLAM_POSE_PUB_PID=$! # stores process pid
echo "SLAM Pose Pub PID: $SLAM_POSE_PUB_PID"

## Waits for node to start publishing
echo "Waiting slam pose pub node to finish setting up..."
sleep 4

## Starting ros2bag recording
echo ""
echo "Starting rosbag recording..."

rm -rf "$EXPERIMENTS_DIR/bag" # removes existing rosbag

ros2 bag record \
    /artbot1/odom /artbot2/odom \
    /artbot1/gt_odom /artbot2/gt_odom \
    /artbot1/slam_pose /artbot2/slam_pose \
    -o "$EXPERIMENTS_DIR/bag" & # rosbag runs in background

BAG_PID=$! # stores background rosbag process pid
echo "Rosbag PID: $BAG_PID"

## Waits for rosbag to start recording
echo "Waiting for rosbag to start recording..."
sleep 4

## Starting multi robot trajectory node exp1
echo ""
echo "Running robot trajectories..."

ros2 run final_proj multi_robot_exp1.py # script waits for it to finish

echo "Trajectories finished."

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

save_map_retry \
"ros2 service call artbot1/slam_toolbox/save_map slam_toolbox/srv/SaveMap \
\"{name: {data: '${EXPERIMENTS_DIR}/artbot1_map_save'}}\"" \
"${EXPERIMENTS_DIR}/artbot1_map_save" \
"Artbot1 Local Map"

save_map_retry \
"ros2 service call artbot2/slam_toolbox/save_map slam_toolbox/srv/SaveMap \
\"{name: {data: '${EXPERIMENTS_DIR}/artbot2_map_save'}}\"" \
"${EXPERIMENTS_DIR}/artbot2_map_save" \
"Artbot2 Local Map"

## Generate aligned maps and merged map
echo ""
echo "Generating aligned and merged maps..."

setsid ros2 run final_proj map_merging.py \
    --ros-args -p use_sim_time:=true \
    -p alignment_output_file:=${EXPERIMENTS_DIR}/alignment_data.json \
    -p gaussian_noise:=[${DIST_NOISE_STDDEV},${THETA_NOISE_STDDEV}] &

MAP_MERGE=$! # stores map alignment process pid
echo "Map Merging PID: $MAP_MERGE"

## Waits for node to start publishing
echo "Waiting map_merge node to finish setting up..."
sleep 10

## Generating local aligned maps
echo ""
echo "Saving local maps after alignment..."

save_map_retry \
"ros2 run nav2_map_server map_saver_cli \
-f ${EXPERIMENTS_DIR}/artbot1_map_aligned_save \
-t /artbot1/map_aligned" \
"${EXPERIMENTS_DIR}/artbot1_map_aligned_save" \
"Local Aligned Map artbot1"

save_map_retry \
"ros2 run nav2_map_server map_saver_cli \
-f ${EXPERIMENTS_DIR}/artbot2_map_aligned_save \
-t /artbot2/map_aligned" \
"${EXPERIMENTS_DIR}/artbot2_map_aligned_save" \
"Local Aligned Map artbot2"

## Generating global merged map
echo ""
echo "Saving merged map..."

save_map_retry \
"ros2 run nav2_map_server map_saver_cli \
-f ${EXPERIMENTS_DIR}/merged_map_save \
-t /merged_map" \
"${EXPERIMENTS_DIR}/merged_map_save" \
"Merged Map"

## Generating CSV and plots
echo ""
echo "Generating CSV..."

python3 $PKG_DIR/experiments/extract_pose_data.py "$EXPERIMENTS_DIR"

echo ""
echo "Generating pose data plots..."

python3 $PKG_DIR/experiments/plot_pose_data.py "$EXPERIMENTS_DIR"

echo ""
echo "Generating occupancy grid maps plots..."

python3 $PKG_DIR/experiments/plot_occup_grid_map.py "$EXPERIMENTS_DIR"

echo ""
echo "Generating aligned maps plots..."

python3 $PKG_DIR/experiments/plot_aligned_maps.py "$EXPERIMENTS_DIR"

echo ""
echo "Generating merged map plots..."

python3 $PKG_DIR/experiments/plot_merged_map.py "$EXPERIMENTS_DIR"

echo ""
echo "Removing bag recording files..."

rm -rf ${EXPERIMENTS_DIR}/bag

echo ""
echo "Experiment $TIMESTAMP Pipeline Completed"

echo "Trajectory csv files saved to: $EXPERIMENTS_DIR/artbotX_{gt,odom,slam}.csv"
echo "Alignment data file saved to: $EXPERIMENTS_DIR/alignment_data.json"
echo "Transformation data file saved to: $EXPERIMENTS_DIR/transformation_data.json"
echo "---------------------------------------------"
echo ""