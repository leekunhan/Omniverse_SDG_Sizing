#!/bin/bash

source /isaac-sim/sdg_sizing/robotic_sizing/config.sh

echo "Starting Data Generation"  

cd $ISAAC_SIM_PATH

echo $PWD

# 1080p resolution
# ./python.sh $SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_1080P \
# --asset-count $ASSET_COUNT \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_1080P \
# --subframes $SUBFRAMES

# 2k resolution
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_2K \
# --asset-count $ASSET_COUNT \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_2K \
# --subframes $SUBFRAMES

# 4k resolution
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_4K \
# --asset-count $ASSET_COUNT \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_4K \
# --subframes $SUBFRAMES