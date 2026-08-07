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
0.00
0.05
)

DIST_STDDEVS=(
0.00
)

THETA_STDDEVS=(
0.00
)

for lidar_stddev in "${LIDAR_STDDEVS[@]}"; do
    for dist_stddev in "${DIST_STDDEVS[@]}"; do
        for theta_stddev in "${THETA_STDDEVS[@]}"; do

            $PKG_DIR/experiments/exp1.sh "$lidar_stddev" "$dist_stddev" "$theta_stddev" "$SESSION_DIR"

            # Diretório do experimento recém-criado
            EXPERIMENT_DIR=$(ls -1dt "$SESSION_DIR"/*/ | head -n1)
            EXPERIMENT_NAME=$(basename "$EXPERIMENT_DIR")

            cat >> "$INDEX_FILE" <<EOF
  - directory: "$EXPERIMENT_NAME"
    lidar_noise_stddev: $lidar_stddev
    dist_noise_stddev: $dist_stddev
    theta_noise_stddev: $theta_stddev
EOF

            sleep 10
    
        done
    done
done

echo ""
echo "All Experiments Completed."
echo "Session Directory: $SESSION_DIR"
echo "Session Experiments Index: $INDEX_FILE"
echo "======================================================"