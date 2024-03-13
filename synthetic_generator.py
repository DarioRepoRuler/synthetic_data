from omni.isaac.kit import SimulationApp
# Create a simulation application context
simulation_app = SimulationApp(launch_config={"headless": False, "renderer": "RayTracedLighting"})

from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
# Create a world within the application context
world = World()
world.reset()

# Randomization
import omni.replicator.core as rep
import omni.usd

# For math and OS
import os
import math
import random
import numpy as np
from pxr import Gf, PhysxSchema, Sdf, UsdPhysics

# Import the URDF extension interface
from omni.importer.urdf import _urdf
from omni.isaac.core.articulations import Articulation

# Camera specific
from scipy.spatial.transform import Rotation as R
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.sensor import Camera
import matplotlib.pyplot as plt

# For the flying distractors
from flying_distractors.collision_box import CollisionBox
from flying_distractors.dynamic_object_set import DynamicObjectSet
from flying_distractors.dynamic_shape_set import DynamicShapeSet

from pxr import Usd, UsdGeom
from data_formatter import create_json_file

import numpy as np
from scipy.spatial.transform import Rotation as R

def camera_pose():
    """
    Generate random camera position and orientation.

    Returns:
        position (numpy.ndarray): Array of shape (3,) representing the camera position.
        euler_angles (numpy.ndarray): Array of shape (3,) representing the camera orientation in Euler angles.
    """
    # Generate a random position
    posx = np.random.uniform(2, 5)
    posy = np.random.uniform(-5, 5)
    posz = np.random.uniform(0, 5)
    position = np.array([posx, posy, posz])
    # Calculate the direction vector to the origin
    direction = -position

    # Normalize the direction vector
    direction /= np.linalg.norm(direction)

    # Calculate the camera's current forward vector
    forward = np.array([1, 0, 0]) 

    # Calculate the rotation axis (cross product of initial and desired directions)
    rotation_axis = np.cross(forward, direction)
    rotation_axis /= np.linalg.norm(rotation_axis)

    # Calculate the rotation angle (angle between initial and desired directions)
    rotation_angle = np.arccos(np.dot(forward, direction))

    # Create a rotation matrix using the axis-angle representation
    rotation_matrix = R.from_rotvec(rotation_angle * rotation_axis).as_matrix()

    # Extract Euler angles from the rotation matrix
    euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz')
    euler_angles[0] = 0.0
    return position, euler_angles
    

def initialise_cameras(num=1):
    """
    Initialize and return a list of camera objects.

    Args:
        num (int): The number of cameras to initialize. Default is 1.

    Returns:
        list: A list of Camera objects.
    """
    cameras = []
    for i in range(num):
        pose = camera_pose()
        camera = Camera(
            prim_path=f"/World/camera_{i}",
            position=pose[0],
            frequency=20,
            resolution=(1080, 720),
            orientation=rot_utils.euler_angles_to_quats(np.degrees(pose[1]), degrees=True),
        )
        cameras.append(camera)
    return cameras

def setup_collision_box(rotation_quat, position = np.array([0.0, 0.0, 0.0]), collision_box_path = "/World/collision_box",  collision_box_name = "collision_box"):      
    """
    Set up a collision box with the given rotation quaternion, position, collision box path, and collision box name.
    
    Parameters:
        rotation_quat (numpy.ndarray): The rotation quaternion of the collision box.
        position (numpy.ndarray, optional): The position of the collision box. Defaults to [0.0, 0.0, 0.0].
        collision_box_path (str, optional): The path of the collision box. Defaults to "/World/collision_box".
        collision_box_name (str, optional): The name of the collision box. Defaults to "collision_box".
    
    Returns:
        CollisionBox: The created collision box object.
    """
    return CollisionBox( 
        collision_box_path,
        collision_box_name,
        position=position,
        orientation=rotation_quat,
        width=1.0,
        height=1.0
    )

def setup_distractors(collision_box, num_distractors=5, i=0):
    """
    Sets up distractor objects for the synthetic generator.

    Args:
        collision_box (str): The collision box for the distractors.
        num_distractors (int, optional): The number of distractors to set up. Defaults to 5.
        i (int, optional): The index of the distractor set. Defaults to 0.
    """
    distractor_mesh_filenames = ['002_master_chef_can', '004_sugar_box', '005_tomato_soup_can', '006_mustard_bottle', '007_tuna_fish_can', '008_pudding_box', '009_gelatin_box', '010_potted_meat_can', '011_banana', '019_pitcher_base', '021_bleach_cleanser', '024_bowl', '025_mug', '036_wood_block', '037_scissors', '040_large_marker', '051_large_clamp', '052_extra_large_clamp', '061_foam_brick']
    #distractor_mesh_filenames = [ '011_banana', '037_scissors', '051_large_clamp', '052_extra_large_clamp']
    assets_root_path = get_assets_root_path()
    asset_path = assets_root_path + "/Isaac/Props/YCB/Axis_Aligned/"
    ycb_asset_path = assets_root_path + "/Isaac/Props/YCB/Axis_Aligned/"

    usd_path_list = [
        f"{ycb_asset_path}{usd_filename_prefix}.usd" for usd_filename_prefix in distractor_mesh_filenames
    ]
    mesh_list = [f"_{usd_filename_prefix[1:]}" for usd_filename_prefix in distractor_mesh_filenames]

        
    # Distractors for the MESH dataset
    mesh_shape_set = DynamicShapeSet(
        f"/World/mesh_shape_set_{i}",
        f"mesh_shape_set_{i}",
        f"mesh_shape_{i}",
        f"mesh_shape_{i}",
        num_distractors, # number of mesh distractors
        collision_box,
        scale=np.array([0.05, 0.05, 0.05]),
        mass=1,
        fraction_glass=0.15
    )

    mesh_object_set = DynamicObjectSet(
        f"/World/mesh_object_set_{i}",
        f"mesh_object_set_{i}",
        usd_path_list,
        mesh_list,
        f"mesh_object_{i}",
        f"mesh_object_ {i}",
        num_distractors, # number of distractors
        collision_box,
        scale=np.array([0.5, 0.5 ,0.5]),
        mass=1,
        fraction_glass=0.15
    )
        
def randomize_domelight(texture_paths):
    """
    Randomizes the dome light in the scene.

    Args:
        texture_paths (list): List of texture paths to choose from.

    Returns:
        Node: The node representing the randomized dome light.
    """
    lights = rep.create.light(
        light_type="Dome",
        rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
        texture=rep.distribution.choice(texture_paths)
    )
    return lights.node
        
        

# Acquire the URDF extension interface
urdf_interface = _urdf.acquire_urdf_interface()
# Set the settings in the import config
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.fix_base = True
import_config.make_default_prim = True
import_config.self_collision = False
import_config.create_physics_scene = False
import_config.import_inertia_tensor = False
import_config.default_drive_strength = 1000.0
import_config.default_position_drive_damping = 50.0
import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
import_config.distance_scale = 1
import_config.density = 0.0

# Get the urdf file path
root_path = os.path.join(os.getcwd(), "abb_common")
file_name = "urdf/irb120.urdf"

# Import the robot
result, prim_path = omni.kit.commands.execute( 
                                    "URDFParseAndImportFile", 
                                    urdf_path=os.path.join(root_path, file_name),
                                    import_config=import_config, 
                                    get_articulation_root=True,
                                    )


# Get stage handle
stage = omni.usd.get_context().get_stage()

# Enable physics
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
# Set gravity
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(0.0) # we don't want gravity to affect the robot, because dynamics are irrelevant for this task
# Set solver settings
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
physxSceneAPI.CreateEnableCCDAttr(True)
physxSceneAPI.CreateEnableStabilizationAttr(True)
physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
physxSceneAPI.CreateSolverTypeAttr("TGS")

# # Add ground plane
# omni.kit.commands.execute(
#     "AddGroundPlaneCommand",
#     stage=stage,
#     planePath="/groundPlane",
#     axis="Z",
#     size=2.0,
#     position=Gf.Vec3f(0, 0, -0.5),
#     color=Gf.Vec3f(0.5),
# )

# Gather all background images from the nucleus server
assets_root_path = get_assets_root_path()
dome_texture_path = assets_root_path + "/NVIDIA/Assets/Skies/"
images_path = "/home/dario/Downloads/VOCdevkit/VOC2012/JPEGImages/"
dome_tex =['Clear/evening_road_01_4k', 'Clear/kloppenheim_02_4k', 'Clear/mealie_road_4k', 'Clear/noon_grass_4k', 'Clear/qwantani_4k', 'Clear/signal_hill_sunrise_4k', 'Clear/sunflowers_4k', 'Clear/syferfontein_18d_clear_4k', 'Clear/venice_sunset_4k', 'Clear/white_cliff_top_4k', 'Cloudy/abandoned_parking_4k', 'Cloudy/champagne_castle_1_4k', 'Cloudy/evening_road_01_4k', 'Cloudy/kloofendal_48d_partly_cloudy_4k', 'Cloudy/lakeside_4k', 'Cloudy/sunflowers_4k', 'Cloudy/table_mountain_1_4k', 'Evening/evening_road_01_4k', 'Indoor/adams_place_bridge_4k', 'Indoor/autoshop_01_4k', 'Indoor/bathroom_4k', 'Indoor/carpentry_shop_01_4k', 'Indoor/en_suite_4k', 'Indoor/entrance_hall_4k', 'Indoor/hospital_room_4k', 'Indoor/hotel_room_4k', 'Indoor/lebombo_4k', 'Indoor/old_bus_depot_4k', 'Indoor/small_empty_house_4k', 'Indoor/studio_small_04_4k', 'Indoor/surgery_4k', 'Indoor/vulture_hide_4k', 'Indoor/wooden_lounge_4k', 'Night/kloppenheim_02_4k', 'Night/moonlit_golf_4k', 'Storm/approaching_storm_4k']
images_paths = []
for i, image in enumerate(os.listdir(images_path)):
    images_paths.append(os.path.join(images_path, image))
dome_texture_paths = [dome_texture_path + dome_texture + ".hdr" for dome_texture in dome_tex]

# Add camera
# Initialize the camera
pose = camera_pose()
cameras = initialise_cameras(2)

# Add lights
distance_light = rep.create.light(rotation=(315, 0, 0), intensity=3000, light_type="distant")
distance_light_1 = rep.create.light(rotation=(315, 0, 0), intensity=3000, light_type="distant")

# Start simulation
omni.timeline.get_timeline_interface().play()
# perform one simulation step so physics is loaded and dynamic control works.
simulation_app.update()

prim = Articulation(prim_path=prim_path, name="abbyArm")
prim.initialize()

# Setup distractors for each camera
for i,camera in enumerate(cameras):
    camera.initialize()
    print(f"Camera {i} position: {camera.get_default_state().position} and Camera {i} orientation: {camera.get_default_state().orientation}")
    
    box = setup_collision_box(camera.get_default_state().orientation,camera.get_default_state().position*0.8, collision_box_path = f"/World/collision_box_{i}",  collision_box_name = f"collision_box_{i}")
    setup_distractors(box, num_distractors=15, i=i)
    
# Setup the randomization events    
with rep.trigger.on_custom_event(event_name="randomize_light"):
    with distance_light:
        # Calculate random spherical coordinates
        theta = random.uniform(0, math.pi)  # Azimuthal angle
        phi = random.uniform(0, math.pi/4)       # Polar angle
        r = random.uniform(5, 10)
        # Convert spherical coordinates to Cartesian coordinates
        x = r* math.sin(phi) * math.cos(theta)
        y = r* math.sin(phi) * math.sin(theta)
        z = r* math.cos(phi)

        # Set the light's position
        rep.modify.pose(position=(x, y, z),  look_at=(0, 0, 0))
        rep.randomizer.rotation()
        rep.modify.attribute("color", rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
        rep.modify.attribute("intensity", random.uniform(2000, 5000))

with rep.trigger.on_custom_event(event_name="randomize_light_1"):
    with distance_light_1:
        # Calculate random spherical coordinates
        theta = random.uniform(-math.pi, 0)  # Azimuthal angle
        phi = random.uniform(0, math.pi/4)       # Polar angle
        r = random.uniform(5, 10)
        # Convert spherical coordinates to Cartesian coordinates
        x = r* math.sin(phi) * math.cos(theta)
        y = r* math.sin(phi) * math.sin(theta)
        z = r* math.cos(phi)

        # Set the light's position
        rep.modify.pose(position=(x, y, z),  look_at=(0, 0, 0))
        rep.randomizer.rotation()
        rep.modify.attribute("color", rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
        rep.modify.attribute("intensity", random.uniform(2000, 5000))
    
rep.randomizer.register(randomize_domelight, override=True)

with rep.trigger.on_custom_event(event_name="randomize_domelight"):
    rep.randomizer.randomize_domelight(dome_texture_paths)#images_paths
    
# Set the boundaries for joint angles    
dof_lower = prim.dof_properties['lower'] + 0.1
dof_upper = prim.dof_properties['upper'] - 0.1
link_paths = ["/abbyArm/link_1", "/abbyArm/link_2", "/abbyArm/link_3", "/abbyArm/link_4", "/abbyArm/link_5", "/abbyArm/link_6"]
link_prims = [stage.GetPrimAtPath(link_path) for link_path in link_paths]

data_dir = os.path.join(os.getcwd(), "synthetic_data")
if not os.path.exists(data_dir):
    print(f"Create data directory at {data_dir}")
    os.mkdir(os.path.join(os.getcwd(), "synthetic_data"))

# Break up condition for failure detection
aborted = False

# Execute the simulation for a number of steps
for i in range(10):
    
    # Sample random joint angles
    random_dof_values = np.random.uniform(dof_lower, dof_upper)    
    state = prim.get_joints_state()
    # Catch invalid states (happens sometimes due to physics simulation instability)
    if aborted == True:
        break
    if np.isnan(state.positions).any():
        prim.set_joint_positions(np.zeros(6))
        break
    
    prim.set_joint_positions(random_dof_values)
    rep.utils.send_og_event(event_name="randomize_light")
    rep.utils.send_og_event(event_name="randomize_light_1")
    rep.utils.send_og_event(event_name="randomize_domelight")
    
    rep.orchestrator.step(rt_subframes=8)
    
    # Capture the data
    for j, camera in enumerate(cameras):
        camera_data = {
            "location_worldframe": camera.get_default_state().position,
            "quaternion_xyzw_worldframe": camera.get_default_state().orientation
        }
        if aborted == True:
            break
        # Save image of the camera
        print(f"Capture frame to {os.path.join(data_dir, f'{i}_{j}.png')} for camera {j}")
        image_path = os.path.join(data_dir, f'{i}_{j}.png')
        plt.imsave(image_path, camera.get_rgba()[:, :, :3])
        # Get the 3D keypoints and the 2D projection on the camera frame
        keypoints_data = []
        for link_prim in link_prims:
            xform = UsdGeom.Xformable(link_prim)
            transform = xform.GetLocalTransformation()
            translation = Gf.Vec3d(transform.ExtractTranslation())
            keypoint3D = np.array([translation[0], translation[1], translation[2]])
            # If the robot arm happens to fall out the window the keypoints get very large...
            if np.linalg.norm(keypoint3D) > 2.0:
                print(f"Aborted due to invalid robot state")
                aborted= True
                break
            keypoint2D = camera.get_image_coords_from_world_points(np.expand_dims(keypoint3D, axis=0))
            #print(f"3D Keypoint: {keypoint3D}, 2D Keypoint {keypoint2D} of link {link_prim.GetPath()}")
            keypoints_data.append({"name": str(link_prim.GetPath()), "location": keypoint3D, "projected_location": keypoint2D})            
        print(f"Writing captured data for camera {j} to {os.path.join(data_dir, f'{i}_{j}.json')}")
        create_json_file(os.path.join(data_dir, f'{i}_{j}.json'), camera_data, prim.get_joints_state().positions, keypoints_data)
    simulation_app.update()
    
rep.orchestrator.wait_until_complete()
simulation_app.close() # close Isaac Sim