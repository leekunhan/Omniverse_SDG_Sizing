#!/bin/bash

source /isaac-sim/sdg_sizing/statistic_sizing/config.sh

echo "Starting Data Generation"  

cd $ISAAC_SIM_PATH

echo $PWD

# 1. 1080p resolution 128 subframes
./python.sh $SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_1080P \
--benchmark-name $BENCHMARK_NAME_1080p$FRAME128 \
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

# 2. 1080p resolution 128 subframes 1 annotator
./python.sh $SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_1080P \
--benchmark-name $BENCHMARK_NAME_1080p$FRAME128$ANNOTATOR1 \
--asset-count $ASSET_COUNT \
--annotators rgb \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_1080P \
--subframes $SUBFRAMES \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_1080P

# 3. 1080p resolution subframes 64
./python.sh $SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_1080P \
--benchmark-name $BENCHMARK_NAME_1080p$FRAME64 \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_1080P \
--subframes 64 \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_1080P

# 4. 1080p resolution subframes 32
./python.sh $SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_1080P \
--benchmark-name $BENCHMARK_NAME_1080p$FRAME32 \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_1080P \
--subframes 32 \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_1080P

# 5. 1080p resolution subframes 1
./python.sh $SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_1080P \
--benchmark-name $BENCHMARK_NAME_1080p$FRAME1 \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_1080P \
--subframes 1 \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_1080P

# 6. 1080p resolution 128 subframes 1 time asset count
./python.sh $SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_1080P \
--benchmark-name $BENCHMARK_NAME_1080p$ASSSET1 \
--asset-count 1 \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_1080P \
--subframes $SUBFRAMES \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_1080P

# 7. 1080p resolution 128 subframes 2 time asset count
./python.sh $SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_1080P \
--benchmark-name $BENCHMARK_NAME_1080p$ASSETS2 \
--asset-count 2 \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_1080P \
--subframes $SUBFRAMES \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_1080P

# 1. 2k resolution 128 subframes
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_2K \
--benchmark-name $BENCHMARK_NAME_2K$FRAME128 \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_2K \
--subframes $SUBFRAMES \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_2K

# 2. 2k resolution 64 subframes
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_2K \
--benchmark-name $BENCHMARK_NAME_2K$FRAME64 \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_2K \
--subframes 64 \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_2K

# 3. 2k resolution 32 subframes
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_2K \
--benchmark-name $BENCHMARK_NAME_2K$FRAME32 \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_2K \
--subframes 32 \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_2K

# 4. 2k resolution 1 subframes
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_2K \
--benchmark-name $BENCHMARK_NAME_2K$FRAME1 \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_2K \
--subframes 1 \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_2K

# 5. 2k resolution 128 subframes 1 time asset count
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_2K \
--benchmark-name $BENCHMARK_NAME_2K$ASSSET1 \
--asset-count 1 \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_2K \
--subframes $SUBFRAMES \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_2K

# 6. 2k resolution 128 subframes 2 time asset count
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_2K \
--benchmark-name $BENCHMARK_NAME_2K$ASSETS2 \
--asset-count 2 \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_2K \
--subframes $SUBFRAMES \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_2K

# 7. 2k resolution 128 subframes 1 annotator
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_2K \
--benchmark-name $BENCHMARK_NAME_2K$ANNOTATOR1 \
--asset-count $ASSET_COUNT \
--annotators rgb \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_2K \
--subframes $SUBFRAMES \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_2K

# 1. 4k resolution 128 subframes
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_4K \
--benchmark-name $BENCHMARK_NAME_4K$FRAME128 \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_4K \
--subframes $SUBFRAMES \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

# 2. 4k resolution 64 subframes
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_4K \
--benchmark-name $BENCHMARK_NAME_4K$FRAME64 \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_4K \
--subframes 64 \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

# 3. 4k resolution 32 subframes
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_4K \
--benchmark-name $BENCHMARK_NAME_4K$FRAME32 \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_4K \
--subframes 32 \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

# 4. 4k resolution 1 subframes
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_4K \
--benchmark-name $BENCHMARK_NAME_4K$FRAME1 \
--asset-count $ASSET_COUNT \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_4K \
--subframes 1 \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

# 5. 4k resolution 128 subframes 1 time asset count
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_4K \
--benchmark-name $BENCHMARK_NAME_4K$ASSSET1 \
--asset-count 1 \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_4K \
--subframes $SUBFRAMES \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

# 6. 4k resolution 128 subframes 2 time asset count
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_4K \
--benchmark-name $BENCHMARK_NAME_4K$ASSSET2 \
--asset-count 2 \
--annotators $ANNOTATORS \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_4K \
--subframes $SUBFRAMES \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K

# 7. 4k resolution 128 subframes 1 annotator
./python.sh $SIMULATION_PATH$SCRIPT_PATH \
--allow-root \
--num-frames $NUM_FRAMES \
--num-cameras $NUM_CAMERAS \
--num-gpus $NUM_GPUS \
--resolution $RESOLUTION_4K \
--benchmark-name $BENCHMARK_NAME_4K$ANNOTATOR1 \
--asset-count $ASSET_COUNT \
--annotators rgb \
--disable-viewport-rendering \
--headless \
--delete-data-when-done \
--print-results \
--backend-type $BACKEND_TYPE \
--output-dir $OUTPUT_DIR_4K \
--subframes $SUBFRAMES \
--/exts/omni.isaac.benchmark.services/metrics/metrics_output_folder=$MATRICS_OUTPUT_FOLDER_4K