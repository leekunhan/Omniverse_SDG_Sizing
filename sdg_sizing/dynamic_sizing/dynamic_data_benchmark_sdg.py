import os
import argparse

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

ENV_URL = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"

parser = argparse.ArgumentParser()
parser.add_argument("--num-frames", type=int, default=600, help="Number of frames to capture")
parser.add_argument("--num-cameras", type=int, default=1, help="Number of cameras")
parser.add_argument("--num-gpus", type=int, default=1, help="Number of GPUs on machine.")
parser.add_argument("--resolution", nargs=2, type=int, default=[1280, 720], help="Camera resolution")
parser.add_argument(
    "--asset-count", type=int, default=1, help="Number of assets of each type (cube, cone, cylinder, sphere, torus)"
)
parser.add_argument(
    "--annotators",
    nargs="+",
    default=["rgb"],
    choices=list(VALID_ANNOTATORS) + ["all"],
    help="List of annotators to enable, separated by space. Use 'all' to select all available.",
)
parser.add_argument("--disable-viewport-rendering", action="store_true", help="Disable viewport rendering")
parser.add_argument("--delete-data-when-done", action="store_true", help="Delete local data after benchmarking")
parser.add_argument("--print-results", action="store_true", help="Print results in terminal")
parser.add_argument("--headless", action="store_true", help="Run in headless mode")
parser.add_argument(
    "--backend-type",
    default="OsmoKPIFile",
    choices=["LocalLogMetrics", "JSONFileMetrics", "OsmoKPIFile"],
    help="Benchmarking backend, defaults",
)
parser.add_argument("--skip-write", action="store_true", help="Skip writing annotator data to disk")
parser.add_argument("--env-url", default=ENV_URL, help="Path to the environment url, default None")
parser.add_argument("--output-dir", default=None, help="Output directory for the benchmark data")
parser.add_argument("--subframes", type=int, default=512, help="Number of subframes to run the benchmark for")
parser.add_argument("--benchmark-name", type=str, default="SDG", help="Name of the benchmark")

args, unknown = parser.parse_known_args()

num_frames = args.num_frames
num_cameras = args.num_cameras
width, height = args.resolution[0], args.resolution[1]
asset_count = args.asset_count
disable_viewport_rendering = args.disable_viewport_rendering
delete_data_when_done = args.delete_data_when_done
print_results = args.print_results
headless = args.headless
n_gpu = args.num_gpus
skip_write = args.skip_write
env_url = args.env_url
output_dir = args.output_dir
subframes = args.subframes
benchmark_name = args.benchmark_name

if "all" in args.annotators:
    annotators_kwargs = {annotator: True for annotator in VALID_ANNOTATORS}
else:
    annotators_kwargs = {annotator: True for annotator in args.annotators if annotator in VALID_ANNOTATORS}

print(f"[SDG Benchmark] Running SDG Benchmark with:")
print(f"\tnum_frames: {num_frames}")
print(f"\tsubframes: {subframes}")
print(f"\tnum_cameras: {num_cameras}")
print(f"\tresolution: {width}x{height}")
print(f"\tasset_count: {asset_count}")
print(f"\tannotators: {annotators_kwargs.keys()}")
print(f"\tdisable_viewport_rendering: {disable_viewport_rendering}")
print(f"\tdelete_data_when_done: {delete_data_when_done}")
print(f"\tprint_results: {print_results}")
print(f"\theadless: {headless}")
print(f"\tskip_write: {skip_write}")
print(f"\tenv_url: {env_url}")
print(f"\toutput_dir: {output_dir}")
print(f"\tbackend_type: {args.backend_type}")

import os
import shutil
import time

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": headless, "max_gpu_count": n_gpu})

REPLICATOR_GLOBAL_SEED = 11

from pxr import Semantics
import omni.kit.app
import omni.replicator.core as rep
import omni.usd
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.nucleus import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from omni.isaac.core.utils.stage import get_current_stage

enable_extension("omni.isaac.benchmark.services")
from omni.isaac.benchmark.services import BaseIsaacBenchmark

# Create the benchmark
benchmark = BaseIsaacBenchmark(
    benchmark_name=benchmark_name,
    workflow_metadata={
        "metadata": [
            {"name": "num_frames", "data": num_frames},
            {"name": "num_cameras", "data": num_cameras},
            {"name": "width", "data": width},
            {"name": "height", "data": height},
            {"name": "asset_count", "data": asset_count},
            {"name": "annotators", "data": args.annotators},
            {"name": "num_gpus", "data": n_gpu},
        ]
    },
    backend_type=args.backend_type,
)

benchmark.set_phase("loading", start_recording_frametime=False, start_recording_runtime=True)

if env_url is not None:
    env_path = env_url if env_url.startswith("omniverse://") else get_assets_root_path() + env_url
    print(f"[SDG Benchmark] Loading stage from path: {env_path}")
    omni.usd.get_context().open_stage(env_path)
else:
    print(f"[SDG Benchmark] Loading a new empty stage..")
    omni.usd.get_context().new_stage()

if disable_viewport_rendering:
    print(f"[SDG Benchmark] Disabling viewport rendering..")
    get_active_viewport().updates_enabled = False

rep.set_global_seed(REPLICATOR_GLOBAL_SEED)

# Start Replicator workflow

PALLETJACKS = ["http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Equipment/Pallet_Trucks/Scale_A/PalletTruckScale_A01_PR_NVD_01.usd",
            "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Equipment/Pallet_Trucks/Heavy_Duty_A/HeavyDutyPalletTruck_A01_PR_NVD_01.usd",
            "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Equipment/Pallet_Trucks/Low_Profile_A/LowProfilePalletTruck_A01_PR_NVD_01.usd"]

DISTRACTORS_WAREHOUSE =    ["/Isaac/Environments/Simple_Warehouse/Props/S_TrafficCone.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/S_WetFloorSign.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_BarelPlastic_A_01.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_BarelPlastic_A_02.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_BarelPlastic_A_03.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_BarelPlastic_B_01.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_BarelPlastic_B_01.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_BarelPlastic_B_03.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_BarelPlastic_C_02.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_BottlePlasticA_02.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_BottlePlasticB_01.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_BottlePlasticA_02.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_BottlePlasticA_02.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_BottlePlasticD_01.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_BottlePlasticE_01.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_BucketPlastic_B.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxB_01_1262.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxB_01_1268.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxB_01_1482.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxB_01_1683.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxB_01_291.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_01_1454.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_01_1513.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_CratePlastic_A_04.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_CratePlastic_B_03.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_CratePlastic_B_05.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_CratePlastic_C_02.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_CratePlastic_E_02.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_PushcartA_02.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_RackPile_04.usd",
                            "/Isaac/Environments/Simple_Warehouse/Props/SM_RackPile_03.usd"]

TEXTURES = ["/Isaac/Materials/Textures/Patterns/nv_asphalt_yellow_weathered.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_tile_hexagonal_green_white.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_rubber_woven_charcoal.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_granite_tile.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_tile_square_green.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_marble.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_brick_reclaimed.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_concrete_aged_with_lines.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_wooden_wall.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_stone_painted_grey.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_wood_shingles_brown.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_tile_hexagonal_various.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_carpet_abstract_pattern.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_wood_siding_weathered_green.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_animalfur_pattern_greys.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_artificialgrass_green.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_bamboo_desktop.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_brick_reclaimed.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_brick_red_stacked.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_fireplace_wall.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_fabric_square_grid.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_granite_tile.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_marble.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_gravel_grey_leaves.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_plastic_blue.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_stone_red_hatch.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_stucco_red_painted.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_rubber_woven_charcoal.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_stucco_smooth_blue.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_wood_shingles_brown.jpg",
            "/Isaac/Materials/Textures/Patterns/nv_wooden_wall.jpg"]

def prefix_with_isaac_asset_server(relative_path):
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        raise Exception("Nucleus server not found, could not access Isaac Sim assets folder")
    return assets_root_path + relative_path

def update_semantics(stage, keep_semantics=[]):
    """ Remove semantics from the stage except for keep_semantic classes"""
    for prim in stage.Traverse():
        if prim.HasAPI(Semantics.SemanticsAPI):
            processed_instances = set()
            for property in prim.GetProperties():
                is_semantic = Semantics.SemanticsAPI.IsSemanticsAPIPath(property.GetPath())
                if is_semantic:
                    instance_name = property.SplitName()[1]
                    if instance_name in processed_instances:
                        # Skip repeated instance, instances are iterated twice due to their two semantic properties (class, data)
                        continue
                    
                    processed_instances.add(instance_name)
                    sem = Semantics.SemanticsAPI.Get(prim, instance_name)
                    type_attr = sem.GetSemanticTypeAttr()
                    data_attr = sem.GetSemanticDataAttr()


                    for semantic_class in keep_semantics:
                    # Check for our data classes needed for the model
                        if data_attr.Get() == semantic_class:
                            continue
                        else:
                            # remove semantics of all other prims
                            prim.RemoveProperty(type_attr.GetName())
                            prim.RemoveProperty(data_attr.GetName())
                            prim.RemoveAPI(Semantics.SemanticsAPI, instance_name)

# Init products
# Create palletjacks
rep_obj_list = [rep.create.from_usd(palletjack_path, semantics=[("class", "palletjack")], count = asset_count) for palletjack_path in PALLETJACKS]

# Create textures
textures = []
for texture in TEXTURES:
    textures.append(prefix_with_isaac_asset_server(texture))

# Create distractors
full_distractors = []
for distractor in DISTRACTORS_WAREHOUSE:
    full_distractors.append(prefix_with_isaac_asset_server(distractor))
distractors = [rep.create.from_usd(distractor_path, count = asset_count) for distractor_path in full_distractors]

# Render Camera
cameras = []
for i in range(num_cameras):
    cameras.append(rep.create.camera(name=f"cam_{i}"))

render_products = []
for i, cam in enumerate(cameras):
    render_products.append(rep.create.render_product(cam, (width, height), name=f"rp_{i}"))

# Create Groups
cameras = rep.create.group(cameras)
rep_palletjack_group = rep.create.group(rep_obj_list)
rep_distractor_group = rep.create.group(distractors)

# Create Trigger
writer_trigger = rep.trigger.on_frame(max_execs = num_frames, rt_subframes = subframes ,interval = 120)
frame_trigger = rep.trigger.on_time(max_execs = num_frames, rt_subframes = subframes, interval = 4)

# We only need labels for the palletjack objects
stage = get_current_stage()
update_semantics(stage=stage, keep_semantics=["palletjack"])

with frame_trigger:
    # Randomize the position, rotation and scale of the cameras
    with cameras:
        rep.modify.pose(
            position=rep.distribution.uniform((-9.2, -11.8, 0.4), (7.2, 15.8, 4)),
            look_at=(0, 0, 0),
        )

    # Randomize the position, rotation and scale of the palletjack
    with rep_palletjack_group:
        rep.modify.pose(
            position=rep.distribution.uniform((-6, -6, 2), (6, 12, 3)),
            rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360)),
            scale=rep.distribution.uniform((0.01, 0.01, 0.01), (0.01, 0.01, 0.01))
        )
        rep.physics.rigid_body()

    # Get the Palletjack body mesh and modify its color
    with rep.get.prims(path_pattern="SteerAxles"):
        rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))

    # Randomize the lighting of the scene
    with rep.get.prims(path_pattern="RectLight"):
        rep.modify.attribute("color", rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
        rep.modify.attribute("intensity", rep.distribution.normal(100000.0, 400000.0))
        rep.modify.visibility(rep.distribution.choice([True, False, False, False]))

    with rep_distractor_group:
        rep.modify.pose(position=rep.distribution.uniform((-6, -6, 2), (6, 12, 3)),
                            rotation=rep.distribution.uniform((0, 0, 0), (0, 360, 360)),
                            scale=rep.distribution.uniform(1, 1.5))
        rep.physics.rigid_body()
        
    # select floor material
    random_mat_floor = rep.create.material_omnipbr(
        diffuse_texture=rep.distribution.choice(textures),
        roughness=rep.distribution.uniform(0, 1),
        metallic=rep.distribution.choice([0, 1]),
        emissive_texture=rep.distribution.choice(textures),
        emissive_intensity=rep.distribution.uniform(0, 1000)
    )
    with rep.get.prims(path_pattern="SM_Floor"):
        rep.randomizer.materials(random_mat_floor)
        rep.physics.collider()

    # select random wall material
    random_mat_wall = rep.create.material_omnipbr(
        diffuse_texture=rep.distribution.choice(textures),
        roughness=rep.distribution.uniform(0, 1),
        metallic=rep.distribution.choice([0, 1]),
        emissive_texture=rep.distribution.choice(textures),
        emissive_intensity=rep.distribution.uniform(0, 1000)
    )
    with rep.get.prims(path_pattern="SM_Wall"):
        rep.randomizer.materials(random_mat_wall)

rep.orchestrator.preview()

# End of Replicator workflow

# Run for a few frames to ensure everything is loaded
for _ in range(10):
    omni.kit.app.get_app().update()
benchmark.store_measurements()

# Writer
if skip_write:
    print("[SDG Benchmark] Skipping writing to disk, attaching annotators to render products..")
    for annot_type, enabled in annotators_kwargs.items():
        if enabled:
            annot = rep.AnnotatorRegistry.get_annotator(annot_type)
            for rp in render_products:
                annot.attach(rp)

elif output_dir is not None:
    writer = rep.writers.get("BasicWriter")
    output_directory = output_dir
    print(f"[SDG Benchmark] Output directory: {output_directory}")
    writer.initialize(output_dir=output_directory, **annotators_kwargs)
    writer.attach(render_products, trigger = writer_trigger )

else:
    writer = rep.writers.get("BasicWriter")
    output_directory = (
        os.getcwd()
        + "/replicator_data"
        + f"/_out_sdg_benchmark_{num_frames}_frames_{num_cameras}_cameras_{asset_count}_asset_count_{len(annotators_kwargs)}_annotators"
    )
    print(f"[SDG Benchmark] Output directory: {output_directory}")
    writer.initialize(output_dir=output_directory, **annotators_kwargs)
    writer.attach(render_products, trigger = writer_trigger)

print("[SDG Benchmark] Starting SDG..")
benchmark.set_phase("benchmark")
start_time = time.time()
rep.orchestrator.run_until_complete()
end_time = time.time()
benchmark.store_measurements()
omni.kit.app.get_app().update()

duration = end_time - start_time
avg_frametime = duration / num_frames
if delete_data_when_done and not skip_write:
    print(f"[SDG Benchmark] Deleting data: {output_directory}")
    shutil.rmtree(output_directory)
if print_results:
    print(f"[SDG Benchmark] duration: {duration} seconds")
    print(f"[SDG Benchmark] avg frametime: {avg_frametime:.4f} seconds")
    print(f"[SDG Benchmark] avg FPS: {1 / avg_frametime:.2f}")
    results_csv = f"{num_frames}, {num_cameras}, {width}, {height}, {asset_count}, {duration:.4f}, {avg_frametime:.4f}, {1 / avg_frametime:.2f}"
    print(f"num_frames, num_cameras, width, height, asset_count, duration, avg_frametime, avg_fps\n{results_csv}\n")

benchmark.stop()

simulation_app.close()