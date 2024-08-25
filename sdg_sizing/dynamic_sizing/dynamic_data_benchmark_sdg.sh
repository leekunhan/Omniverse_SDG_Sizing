#!/bin/bash

source /isaac-sim/sdg_sizing/dynamic_sizing/config.sh

echo "Starting Data Generation"  

cd $ISAAC_SIM_PATH

echo $PWD

# 1080p resolution
./python.sh $SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_1080P \
--benchmark-name $BENCHMARK_NAME_1080p \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_1080P \
--subframes $SUBFRAMES \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_1080P

# 2k resolution
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_2K \
--benchmark-name $BENCHMARK_NAME_2K \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_2K \
--subframes $SUBFRAMES \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_2K

# 4k resolution
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_4K \
--benchmark-name $BENCHMARK_NAME_4K \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_4K \
--subframes $SUBFRAMES \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K