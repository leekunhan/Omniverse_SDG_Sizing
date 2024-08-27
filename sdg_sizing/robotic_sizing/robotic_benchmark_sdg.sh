#!/bin/bash

source /isaac-sim/sdg_sizing/robotic_sizing/config.sh

echo "Starting Data Generation"  

cd $ISAAC_SIM_PATH

echo $PWD

# 1080p resolution
./python.sh $SCRIPT_PATH \
--allow-root \
--num-amrs $NUM_AMR \
--num-robotic-arms $NUM_ROBOTIC_ARM \
--num-gpus $NUM_GPUS \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--annotators $ANNOTATORS \
--backend-type $BACKEND_TYPE \
--disable-viewport-rendering \
--resolution $RESOLUTION_1080P \
--output-dir $OUTPUT_DIR_1080P \
--subframes $SUBFRAMES \
--benchmark-name $BENCHMARK_NAME_1080p \
--print-results \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_1080P


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