# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import argparse
import os

VALID_ANNOTATORS = {
    "rgb",
    "bounding_box_2d_tight",
    "bounding_box_2d_loose",
    "semantic_segmentation",
    "instance_id_segmentation",
    "instance_segmentation",
    "distance_to_camera",
    "distance_to_image_plane",
    "bounding_box_3d",
    "occlusion",
    "normals",
    "motion_vectors",
    "camera_params",
    "pointcloud",
    "skeleton_data",
}

ENV_URL = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"

REPLICATOR_GLOBAL_SEED = 11

parser = argparse.ArgumentParser()
parser.add_argument("--num-amrs", type=int, default=1, help="Number of robots")
parser.add_argument("--num-gpus", type=int, default=1, help="Number of GPUs on machine.")
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to run benchmark for")
parser.add_argument("--num-cameras", type=int, default=1, help="Number of cameras")
parser.add_argument("--headless", action="store_true", help="Run in headless mode")
parser.add_argument(
    "--annotators",
    nargs="+",
    default=["rgb"],
    choices=list(VALID_ANNOTATORS) + ["all"],
    help="List of annotators to enable, separated by space. Use 'all' to select all available.",
)
parser.add_argument(
    "--backend-type",
    default="OsmoKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile"],
    help="Benchmarking backend, defaults",
)
parser.add_argument("--disable-viewport-rendering", action="store_true", help="Disable viewport rendering")
parser.add_argument("--delete-data-when-done", action="store_true", help="Delete local data after benchmarking")
parser.add_argument("--resolution", nargs=2, type=int, default=[1280, 720], help="Camera resolution")
parser.add_argument("--output-dir", type=str, default=os.getcwd() + "/_robotic_data", help="Location where data will be output")
parser.add_argument("--subframes", type=int, default=512, help="Number of subframes to run the benchmark for")
parser.add_argument("--benchmark-name", type=str, default="Robotic_SDG", help="Name of the benchmark")
parser.add_argument("--print-results", action="store_true", help="Print results in terminal")
parser.add_argument("--env-url", default=ENV_URL, help="Path to the environment url, default None")

args, unknown = parser.parse_known_args()


n_amrs = args.num_amrs
n_gpu = args.num_gpus
n_frames = args.num_frames
num_cameras = args.num_cameras
headless = args.headless
disable_viewport_rendering = args.disable_viewport_rendering
delete_data_when_done = args.delete_data_when_done
width, height = args.resolution[0], args.resolution[1]
output_dir = args.output_dir
subframes = args.subframes
benchmark_name = args.benchmark_name
print_results = args.print_results
env_url = args.env_url

if "all" in args.annotators:
    annotators_kwargs = {annotator: True for annotator in VALID_ANNOTATORS}
else:
    annotators_kwargs = {annotator: True for annotator in args.annotators if annotator in VALID_ANNOTATORS}

print(f"[SDG Benchmark] Running SDG Benchmark with:")
print(f"\tNumber of AMRs: {n_amrs}")
print(f"\tNumber of GPUs: {n_gpu}")
print(f"\tNumber of Frames: {n_frames}")
print(f"\tNumber of Cameras: {num_cameras}")
print(f"\tHeadless: {headless}")
print(f"\tDisable Viewport Rendering: {disable_viewport_rendering}")
print(f"\tDelete Data When Done: {delete_data_when_done}")
print(f"\tResolution: {width}x{height}")
print(f"\tOutput Directory: {output_dir}")
print(f"\tSubframes: {subframes}")
print(f"\tBenchmark Name: {benchmark_name}")
print(f"\tPrint Results: {print_results}")
print(f"\tEnvironment URL: {env_url}")
print(f"\tAnnotators: {annotators_kwargs.keys()}")

import shutil
import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": headless, "max_gpu_count": n_gpu})

import omni
import omni.replicator.core as rep
import omni.kit.test
from omni.isaac.core import PhysicsContext
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.wheeled_robots.robots import WheeledRobot

enable_extension("omni.isaac.benchmark.services")
from omni.isaac.benchmark.services import BaseIsaacBenchmark

# Create the benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name=benchmark_name,
    workflow_metadata={
        "metadata": [
            {"name": "num_robots", "data": n_amrs},
            {"name": "num_gpus", "data": n_gpu},
            {"name": "num_frames", "data": n_frames},
            {"name": "num_cameras", "data": num_cameras},
            {"name": "headless", "data": headless},
            {"name": "disable_viewport_rendering", "data": disable_viewport_rendering},
            {"name": "delete_data_when_done", "data": delete_data_when_done},
            {"name": "resolution", "data": f"{width}x{height}"},
            {"name": "output_dir", "data": output_dir},
            {"name": "subframes", "data": subframes},
            {"name": "benchmark_name", "data": benchmark_name},
            {"name": "print_results", "data": print_results},
            {"name": "env_url", "data": env_url},
            {"name": "annotators", "data": annotators_kwargs.keys()},
        ]
    },
    backend_type=args.backend_type,
)
benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

amr_path = "/Isaac/Robots/Carter/nova_carter_sensors.usd"

scene_path = env_url
benchmark.fully_load_stage(benchmark.assets_root_path + scene_path)
stage = omni.usd.get_context().get_stage()
PhysicsContext(physics_dt=1.0 / 60.0)
set_camera_view(eye=[-6, -15.5, 6.5], target=[-6, 10.5, -1], camera_prim_path="/OmniverseKit_Persp")

# Render Camera
cameras = []
for i in range(num_cameras):
    cameras.append(rep.create.camera(name=f"cam_{i}"))

render_products = []
for i, cam in enumerate(cameras):
    render_products.append(rep.create.render_product(cam, (width, height), name=f"rp_{i}"))

# Create Groups
cameras = rep.create.group(cameras)

robots = []
for i in range(n_amrs):
    robot_prim_path = "/Robots/Robot_" + str(i)
    robot_usd_path = benchmark.assets_root_path + amr_path
    # position the robot
    MAX_IN_LINE = 10
    robot_position = np.array([-2 * (i % MAX_IN_LINE), -2 * np.floor(i / MAX_IN_LINE), 0])
    current_robot = WheeledRobot(
        prim_path=robot_prim_path,
        wheel_dof_names=["joint_wheel_left", "joint_wheel_right"],
        create_robot=True,
        usd_path=robot_usd_path,
        position=robot_position,
    )

    omni.kit.app.get_app().update()
    omni.kit.app.get_app().update()

    robots.append(current_robot)

timeline = omni.timeline.get_timeline_interface()
timeline.play()
omni.kit.app.get_app().update()

for robot in robots:
    robot.initialize()
    # start the robot rotating in place so not to run into each
    robot.apply_wheel_actions(
        ArticulationAction(joint_positions=None, joint_efforts=None, joint_velocities=10 * np.array([0, 1]))
    )

omni.kit.app.get_app().update()
omni.kit.app.get_app().update()

benchmark.store_measurements()
# perform benchmark
benchmark.set_phase("benchmark")

for _ in range(1, n_frames):
    omni.kit.app.get_app().update()

benchmark.store_measurements()
benchmark.stop()

timeline.stop()
simulation_app.close()
