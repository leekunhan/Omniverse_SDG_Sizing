#!/bin/bash

source /isaac-sim/sdg_sizing/statistic_sizing/config.sh

echo "Starting Data Generation"  

cd $ISAAC_SIM_PATH

echo $PWD

# 1080p resolution 128 subframes
./python.sh $SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_1080P \
--benchmark-name $BENCHMARK_NAME_1080p$128FRAME \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_1080P \
--subframes $SUBFRAMES \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_1080P

# # 1080p resolution 128 subframes 1 annotator
# ./python.sh $SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_1080P \
# --benchmark-name $BENCHMARK_NAME_1080p$128FRAME$1ANNOTATOR \
# --asset-count $ASSET_COUNT \
# --annotators rgb \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_1080P \
# --subframes $SUBFRAMES \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_1080P

# # 1080p resolution subframes 64
# ./python.sh $SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_1080P \
# --benchmark-name $BENCHMARK_NAME_1080p \
# --asset-count $ASSET_COUNT \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_1080P \
# --subframes 64 \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_1080P

# # 1080p resolution subframes 32
# ./python.sh $SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_1080P \
# --benchmark-name $BENCHMARK_NAME_1080p \
# --asset-count $ASSET_COUNT \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_1080P \
# --subframes 32 \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_1080P

# # 1080p resolution subframes 1
# ./python.sh $SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_1080P \
# --benchmark-name $BENCHMARK_NAME_1080p \
# --asset-count $ASSET_COUNT \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_1080P \
# --subframes 1 \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_1080P

# # 1080p resolution 128 subframes 1 time asset count
# ./python.sh $SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_1080P \
# --benchmark-name $BENCHMARK_NAME_1080p \
# --asset-count 1 \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_1080P \
# --subframes $SUBFRAMES \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_1080P

# # 1080p resolution 128 subframes 2 time asset count
# ./python.sh $SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_1080P \
# --benchmark-name $BENCHMARK_NAME_1080p \
# --asset-count 2 \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_1080P \
# --subframes $SUBFRAMES \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_1080P

# # 2k resolution 128 subframes
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_2K \
# --benchmark-name $BENCHMARK_NAME_2K \
# --asset-count $ASSET_COUNT \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_2K \
# --subframes $SUBFRAMES \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_2K

# # 2k resolution 64 subframes
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_2K \
# --benchmark-name $BENCHMARK_NAME_2K \
# --asset-count $ASSET_COUNT \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_2K \
# --subframes 64 \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_2K

# # 2k resolution 32 subframes
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_2K \
# --benchmark-name $BENCHMARK_NAME_2K \
# --asset-count $ASSET_COUNT \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_2K \
# --subframes 32 \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_2K

# # 2k resolution 1 subframes
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_2K \
# --benchmark-name $BENCHMARK_NAME_2K \
# --asset-count $ASSET_COUNT \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_2K \
# --subframes 1 \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_2K

# # 2k resolution 128 subframes 1 time asset count
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_2K \
# --benchmark-name $BENCHMARK_NAME_2K \
# --asset-count 1 \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_2K \
# --subframes $SUBFRAMES \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_2K

# # 2k resolution 128 subframes 2 time asset count
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_2K \
# --benchmark-name $BENCHMARK_NAME_2K \
# --asset-count 2 \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_2K \
# --subframes $SUBFRAMES \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_2K

# # 2k resolution 128 subframes 1 annotator
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_2K \
# --benchmark-name $BENCHMARK_NAME_2K \
# --asset-count $ASSET_COUNT \
# --annotators rgb \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_2K \
# --subframes $SUBFRAMES \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_2K

# # 4k resolution 128 subframes
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_4K \
# --benchmark-name $BENCHMARK_NAME_4K \
# --asset-count $ASSET_COUNT \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_4K \
# --subframes $SUBFRAMES \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

# # 4k resolution 64 subframes
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_4K \
# --benchmark-name $BENCHMARK_NAME_4K \
# --asset-count $ASSET_COUNT \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_4K \
# --subframes 64 \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

# # 4k resolution 32 subframes
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_4K \
# --benchmark-name $BENCHMARK_NAME_4K \
# --asset-count $ASSET_COUNT \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_4K \
# --subframes 32 \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

# # 4k resolution 1 subframes
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_4K \
# --benchmark-name $BENCHMARK_NAME_4K \
# --asset-count $ASSET_COUNT \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_4K \
# --subframes 1 \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

# # 4k resolution 128 subframes 1 time asset count
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_4K \
# --benchmark-name $BENCHMARK_NAME_4K \
# --asset-count 1 \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_4K \
# --subframes $SUBFRAMES \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

# # 4k resolution 128 subframes 2 time asset count
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_4K \
# --benchmark-name $BENCHMARK_NAME_4K \
# --asset-count 2 \
# --annotators $ANNOTATORS \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_4K \
# --subframes $SUBFRAMES \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

# # 4k resolution 128 subframes 1 annotator
# ./python.sh $SIMULATION_PATH$SCRIPT_PATH \
# --allow-root \
# --num-frames $NUM_FRAMES \
# --num-cameras $NUM_CAMERAS \
# --num-gpus $NUM_GPUS \
# --resolution $RESOLUTION_4K \
# --benchmark-name $BENCHMARK_NAME_4K \
# --asset-count $ASSET_COUNT \
# --annotators rgb \
# --disable-viewport-rendering \
# --headless \
# --delete-data-when-done \
# --print-results \
# --backend-type $BACKEND_TYPE \
# --output-dir $OUTPUT_DIR_4K \
# --subframes $SUBFRAMES \
# --/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K