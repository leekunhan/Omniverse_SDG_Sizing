#!/bin/bash

source /isaac-sim/sdg_sizing/statistic_sizing/config.sh

echo "Starting Data Generation"  

cd $ISAAC_SIM_PATH

echo $PWD

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
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_4K \
--subframes $SUBFRAMES \
--headless \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

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
--headless \
--disable-viewport-rendering \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_4K$NAME1 \
--subframes 1 \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

# 4k resolution
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_4K \
--benchmark-name $BENCHMARK_NAME_4K \
--headless \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_4K$NAME32 \
--subframes 32 \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

# 4k resolution
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_4K \
--benchmark-name $BENCHMARK_NAME_4K \
--asset-count $ASSET_COUNT \
--headless \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_4K$NAME64 \
--subframes 64 \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

# 4k resolution
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_4K \
--benchmark-name $BENCHMARK_NAME_4K \
--asset-count $ASSET_COUNT \
--headless \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_4K$NAME128 \
--subframes 128 \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K