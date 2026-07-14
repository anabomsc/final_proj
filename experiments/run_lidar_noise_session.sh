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
LOG_FILE="$SESSION_DIR/lidar_noise_experiments.log"
exec > >(tee -i "$LOG_FILE") 2>&1

# Session index yaml
INDEX_FILE="$SESSION_DIR/experiment_index.yaml"
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
MEANS=(
0.0
)

STDDEVS=(
0.00
# 0.01
# 0.02
# 0.05
# 0.10
0.20
)

for mean in "${MEANS[@]}"; do
    for stddev in "${STDDEVS[@]}"; do

        $PKG_DIR/experiments/exp1.sh "$mean" "$stddev" "$SESSION_DIR"

        # Diretório do experimento recém-criado
        EXPERIMENT_DIR=$(ls -1dt "$SESSION_DIR"/*/ | head -n1)
        EXPERIMENT_NAME=$(basename "$EXPERIMENT_DIR")

        cat >> "$INDEX_FILE" <<EOF
  - directory: "$EXPERIMENT_NAME"
    lidar_noise_mean: $mean
    lidar_noise_stddev: $stddev
EOF

        sleep 10

    done
done

echo ""
echo "All Experiments Completed."
echo "Session Directory: $SESSION_DIR"
echo "Session Experiments Index: $INDEX_FILE"
echo "======================================================"