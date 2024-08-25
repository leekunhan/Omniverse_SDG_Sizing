# config.sh

# Path where Isaac Sim is installed which contains the python.sh script
ISAAC_SIM_PATH='/isaac-sim'
SCRIPT_PATH='./sdg_sizing/dynamic_sizing/dynamic_data_benchmark_sdg.py'
OUTPUT_BASE_DIR='/replicator_data/dynamic_sizing'

# Parameters:
NUM_FRAMES=1000
NUM_CAMERAS=2
NUM_GPUS=1
ASSET_COUNT=3
SUBFRAMES=128
ANNOTATORS='all' 
# VALID_ANNOTATORS = {"rgb",
#     "bounding_box_2d_tight",
#     "bounding_box_2d_loose",
#     "semantic_segmentation",
#     "instance_id_segmentation",
#     "instance_segmentation",
#     "distance_to_camera",
#     "distance_to_image_plane",
#     "bounding_box_3d",
#     "occlusion",
#     "normals",
#     "motion_vectors",
#     "camera_params",
#     "pointcloud",
#     "skeleton_data",
# }
BACKEND_TYPE='JSONFileMetrics'

# 1080p resolution
RESOLUTION_1080P="1080 720"
OUTPUT_DIR_1080P="$ISAAC_SIM_PATH$OUTPUT_BASE_DIR/1080p"
MATRICS_OUTPUT_FOLDER_1080P='/isaac-sim/replicator_data/dynamic_sizing/'
BENCHMARK_NAME_1080p="Dynamic_Data_Benchmark_SDG_1080p"

# 2k resolution
RESOLUTION_2K="2560 1440"
OUTPUT_DIR_2K="$ISAAC_SIM_PATH$OUTPUT_BASE_DIR/2k"
MATRICS_OUTPUT_FOLDER_2K='/isaac-sim/replicator_data/dynamic_sizing/'
BENCHMARK_NAME_2K="Dynamic_Data_Benchmark_SDG_2k"

# 4k resolution
RESOLUTION_4K="3840 2160"
OUTPUT_DIR_4K="$ISAAC_SIM_PATH$OUTPUT_BASE_DIR/4k"
MATRICS_OUTPUT_FOLDER_4K='/isaac-sim/replicator_data/dynamic_sizing/'
BENCHMARK_NAME_4K="Dynamic_Data_Benchmark_SDG_4k"