#!/bin/bash
# run_lidar_noise_session.sh

set -e # stops execution in case of errors

SESSION_TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")

# Setup paths
WORKSPACE="$HOME/mono_ws"
PKG_DIR="$WORKSPACE/src/final_proj"
RESULTS_ROOT="$WORKSPACE/results"

# Create session directory
SESSION_DIR="$RESULTS_ROOT/session_$SESSION_TIMESTAMP"
mkdir -p "$SESSION_DIR"

# Logging
LOG_FILE="$SESSION_DIR/noise_experiments.log"
exec > >(tee -i "$LOG_FILE") 2>&1

# Session index yaml
INDEX_FILE="$SESSION_DIR/session_index.yaml"
cat > "$INDEX_FILE" <<EOF
session_timestamp: "$SESSION_TIMESTAMP"
experiments:
EOF

echo "==================== SESSION INFO ===================="
echo "Timestamp: $SESSION_TIMESTAMP"
echo "Directory: $SESSION_DIR"
echo "Index file: $INDEX_FILE"
echo "Log file: $LOG_FILE"
echo "======================================================"

# Noise configurations
LIDAR_STDDEVS=(
0.05
0.10
0.30
0.50
)

V_ODOM_STDDEVS=(
0.05
0.10
0.30
0.50
)

W_ODOM_STDDEVS=(
0.0174532925 #1°
0.0872664626 #5°
0.174532925 #10°
0.34906585 #20°
)

DIST_STDDEVS=(
0.05
0.10
0.30
0.50
)

THETA_STDDEVS=(
0.174532925 #10°
0.34906585 #20°
0.41887902 #24°
0.523598776 #30°
)

SESSION_TYPE="zero-tr-noise"
SESSION_TYPE_DIR="$SESSION_DIR/$SESSION_TYPE"
for ((i=0; i<${#LIDAR_STDDEVS[@]}; i++)); do
    lidar_stddev="${LIDAR_STDDEVS[i]}"
    v_odom_stddev="${V_ODOM_STDDEVS[i]}"
    w_odom_stddev="${W_ODOM_STDDEVS[i]}"
    $PKG_DIR/experiments/exp1.sh "$lidar_stddev" "$v_odom_stddev" "$w_odom_stddev" "0.0" "0.0" "$SESSION_TYPE_DIR"

    # Diretório do experimento recém-criado
    EXPERIMENT_DIR=$(ls -1dt "$SESSION_TYPE_DIR"/*/ | head -n1)
    EXPERIMENT_NAME=$(basename "$EXPERIMENT_DIR")

        cat >> "$INDEX_FILE" <<EOF
  - directory: "./$SESSION_TYPE/$EXPERIMENT_NAME"
    lidar_noise_stddev: $lidar_stddev
    v_odom_noise_stddev: $v_odom_stddev
    w_odom_noise_stddev: $w_odom_stddev
    dist_noise_stddev: 0.0
    theta_noise_stddev: 0.0
EOF

    sleep 10

done

for ((i=0; i<${#LIDAR_STDDEVS[@]}; i++)); do
    lidar_stddev="${LIDAR_STDDEVS[i]}"
    v_odom_stddev="${V_ODOM_STDDEVS[i]}"
    w_odom_stddev="${W_ODOM_STDDEVS[i]}"
    $PKG_DIR/experiments/exp2.sh "$lidar_stddev" "$v_odom_stddev" "$w_odom_stddev" "0.0" "0.0" "$SESSION_TYPE_DIR"

    # Diretório do experimento recém-criado
    EXPERIMENT_DIR=$(ls -1dt "$SESSION_TYPE_DIR"/*/ | head -n1)
    EXPERIMENT_NAME=$(basename "$EXPERIMENT_DIR")

        cat >> "$INDEX_FILE" <<EOF
  - directory: "./$SESSION_TYPE/$EXPERIMENT_NAME"
    lidar_noise_stddev: $lidar_stddev
    v_odom_noise_stddev: $v_odom_stddev
    w_odom_noise_stddev: $w_odom_stddev
    dist_noise_stddev: 0.0
    theta_noise_stddev: 0.0
EOF

    sleep 10

done

SESSION_TYPE="tr-noise"
SESSION_TYPE_DIR="$SESSION_DIR/$SESSION_TYPE"
for ((i=0; i<${#LIDAR_STDDEVS[@]}; i++)); do
    lidar_stddev="${LIDAR_STDDEVS[i]}"
    v_odom_stddev="${V_ODOM_STDDEVS[i]}"
    w_odom_stddev="${W_ODOM_STDDEVS[i]}"
    dist_stddev="${DIST_STDDEVS[i]}"
    theta_stddev="${THETA_STDDEVS[i]}"

    $PKG_DIR/experiments/exp1.sh "$lidar_stddev" "$v_odom_stddev" "$w_odom_stddev" "$dist_stddev" "$theta_stddev" "$SESSION_TYPE_DIR"

    # Diretório do experimento recém-criado
    EXPERIMENT_DIR=$(ls -1dt "$SESSION_TYPE_DIR"/*/ | head -n1)
    EXPERIMENT_NAME=$(basename "$EXPERIMENT_DIR")

                cat >> "$INDEX_FILE" <<EOF
  - directory: "./$SESSION_TYPE/$EXPERIMENT_NAME"
    lidar_noise_stddev: $lidar_stddev
    v_odom_noise_stddev: $v_odom_stddev
    w_odom_noise_stddev: $w_odom_stddev
    dist_noise_stddev: $dist_stddev
    theta_noise_stddev: $theta_stddev
EOF

    sleep 10
done

for ((i=0; i<${#LIDAR_STDDEVS[@]}; i++)); do
    lidar_stddev="${LIDAR_STDDEVS[i]}"
    v_odom_stddev="${V_ODOM_STDDEVS[i]}"
    w_odom_stddev="${W_ODOM_STDDEVS[i]}"
    dist_stddev="${DIST_STDDEVS[i]}"
    theta_stddev="${THETA_STDDEVS[i]}"

    $PKG_DIR/experiments/exp2.sh "$lidar_stddev" "$v_odom_stddev" "$w_odom_stddev" "$dist_stddev" "$theta_stddev" "$SESSION_TYPE_DIR"

    # Diretório do experimento recém-criado
    EXPERIMENT_DIR=$(ls -1dt "$SESSION_TYPE_DIR"/*/ | head -n1)
    EXPERIMENT_NAME=$(basename "$EXPERIMENT_DIR")

                cat >> "$INDEX_FILE" <<EOF
  - directory: "./$SESSION_TYPE/$EXPERIMENT_NAME"
    lidar_noise_stddev: $lidar_stddev
    v_odom_noise_stddev: $v_odom_stddev
    w_odom_noise_stddev: $w_odom_stddev
    dist_noise_stddev: $dist_stddev
    theta_noise_stddev: $theta_stddev
EOF

    sleep 10
done

echo ""
echo "All Experiments Completed."
echo "Session Directory: $SESSION_DIR"
echo "Session Experiments Index: $INDEX_FILE"
echo "======================================================"