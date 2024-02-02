from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(launch_config={"headless": False})

from omni.isaac.core import World

world = World()
world.reset()

import os
import omni.replicator.core as rep
import omni.usd

import math
import random

from pxr import Gf, PhysxSchema, Sdf, UsdPhysics

# Import the URDF extension interface
from omni.importer.urdf import _urdf
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.articulations import Articulation, ArticulationView

import numpy as np

# Camera specific
from scipy.spatial.transform import Rotation as R
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.sensor import Camera
import matplotlib.pyplot as plt

from omni.isaac.core.utils.nucleus import get_assets_root_path
from flying_distractors.collision_box import CollisionBox
from flying_distractors.dynamic_object_set import DynamicObjectSet
from flying_distractors.dynamic_shape_set import DynamicShapeSet


def camera_pose():
    # Generate a random position
    posx = np.random.uniform(0, 10)
    posy = np.random.uniform(-10, 10)
    posz = np.random.uniform(0, 10)
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
    

def initialise_cameras(num= 1):    
    cameras = []
    for i in range(num):
        pose = camera_pose()
        camera = Camera(
                prim_path=f"/World/camera_{i}",
                position=pose[0],
                frequency=20,
                resolution=(1080,  720),
                orientation=rot_utils.euler_angles_to_quats(np.degrees(pose[1]), degrees=True),
            )
        cameras.append(camera)
    return cameras

def setup_collision_box(rotation_quat, position = np.array([0.0, 0.0, 0.0]) ):
        # Create a collision box in view of the camera, allowing distractors placed in the box to be within
        # [MIN_DISTANCE, MAX_DISTANCE] of the camera. The collision box will be placed in front of the camera,
        # regardless of CAMERA_ROTATION or CAMERA_RIG_ROTATION.
        collision_box_path = "/World/collision_box"
        collision_box_name = "collision_box"

        # Collision box has no rotation with respect to the camera.
        # collision_box_rotation_from_camera = np.array([0, 0, 0])
        # collision_box_orientation_from_camera = euler_angles_to_quat(collision_box_rotation_from_camera, degrees=True)

        # Get the desired pose of the collision box from a pose defined locally with respect to the camera.
        # camera_prim = world.stage.GetPrimAtPath(self.camera_path)
        # collision_box_center, collision_box_orientation = get_world_pose_from_relative(
        #     camera_prim, collision_box_translation_from_camera, collision_box_orientation_from_camera
        # )
        
        
        return CollisionBox( 
            collision_box_path,
            collision_box_name,
            position=position,
            orientation=rotation_quat,
            width=1.0,
            height=1.0
        )

def setup_distractors(collision_box):
        # List of distractor objects should not contain objects that are being used for training
        #train_objects = ['003_cracker_box', '003_cracker_box']
        distractor_mesh_filenames = ['002_master_chef_can', '004_sugar_box', '005_tomato_soup_can', '006_mustard_bottle', '007_tuna_fish_can', '008_pudding_box', '009_gelatin_box', '010_potted_meat_can', '011_banana', '019_pitcher_base', '021_bleach_cleanser', '024_bowl', '025_mug', '036_wood_block', '037_scissors', '040_large_marker', '051_large_clamp', '052_extra_large_clamp', '061_foam_brick']
        
        assets_root_path = get_assets_root_path()
        asset_path = assets_root_path + "/Isaac/Props/YCB/Axis_Aligned/"
        ycb_asset_path = assets_root_path + "/Isaac/Props/YCB/Axis_Aligned/"

        usd_path_list = [
            f"{ycb_asset_path}{usd_filename_prefix}.usd" for usd_filename_prefix in distractor_mesh_filenames
        ]
        mesh_list = [f"_{usd_filename_prefix[1:]}" for usd_filename_prefix in distractor_mesh_filenames]

        
        # Distractors for the MESH dataset
        mesh_shape_set = DynamicShapeSet(
            "/World/mesh_shape_set",
            "mesh_shape_set",
            "mesh_shape",
            "mesh_shape",
            10, # number of mesh distractors
            collision_box,
            scale=np.array([0.05, 0.05, 0.05]),
            mass=1,
            fraction_glass=0.15
        )
            #self.mesh_distractors.add(mesh_shape_set)

        mesh_object_set = DynamicObjectSet(
            "/World/mesh_object_set",
            "mesh_object_set",
            usd_path_list,
            mesh_list,
            "mesh_object",
            "mesh_object",
            10, # number of distractors
            collision_box,
            scale=np.array([0.5, 0.5 ,0.5]),
            mass=1,
            fraction_glass=0.15
        )
        #self.mesh_distractors.add(mesh_object_set)
        # Set the current distractors to the mesh dataset type
        #self.current_distractors = self.mesh_distractors



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
root_path = "/home/dario/Documents/AUT_Proj/data_gen/abb_common"
file_name = "urdf/irb120.urdf"

# Import the robot
result, prim_path = omni.kit.commands.execute( "URDFParseAndImportFile", urdf_path=os.path.join(root_path, file_name),
                                                import_config=import_config, get_articulation_root=True,)

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

assets_root_path = get_assets_root_path()
dome_texture_path = assets_root_path + "/NVIDIA/Assets/Skies/"
# Add ground plane
# result, ground_path = omni.kit.commands.execute(
#     "AddGroundPlaneCommand",
#     stage=stage, 
#     planePath="/groundPlane",
#     axis="Z",
#     size=1500.0,
#     position=Gf.Vec3f(0, 0, -2),
#     color=Gf.Vec3f(0.5),
# )
# print(f"Ground plane path: {ground_path}")

images_path = "/home/dario/Downloads/VOCdevkit/VOC2012/JPEGImages/"
dome_tex =['Clear/evening_road_01_4k', 'Clear/kloppenheim_02_4k', 'Clear/mealie_road_4k', 'Clear/noon_grass_4k', 'Clear/qwantani_4k', 'Clear/signal_hill_sunrise_4k', 'Clear/sunflowers_4k', 'Clear/syferfontein_18d_clear_4k', 'Clear/venice_sunset_4k', 'Clear/white_cliff_top_4k', 'Cloudy/abandoned_parking_4k', 'Cloudy/champagne_castle_1_4k', 'Cloudy/evening_road_01_4k', 'Cloudy/kloofendal_48d_partly_cloudy_4k', 'Cloudy/lakeside_4k', 'Cloudy/sunflowers_4k', 'Cloudy/table_mountain_1_4k', 'Evening/evening_road_01_4k', 'Indoor/adams_place_bridge_4k', 'Indoor/autoshop_01_4k', 'Indoor/bathroom_4k', 'Indoor/carpentry_shop_01_4k', 'Indoor/en_suite_4k', 'Indoor/entrance_hall_4k', 'Indoor/hospital_room_4k', 'Indoor/hotel_room_4k', 'Indoor/lebombo_4k', 'Indoor/old_bus_depot_4k', 'Indoor/small_empty_house_4k', 'Indoor/studio_small_04_4k', 'Indoor/surgery_4k', 'Indoor/vulture_hide_4k', 'Indoor/wooden_lounge_4k', 'Night/kloppenheim_02_4k', 'Night/moonlit_golf_4k', 'Storm/approaching_storm_4k']
images_paths = []
for i, image in enumerate(os.listdir(images_path)):
    images_paths.append(os.path.join(images_path, image))
    #print(os.path.join(images_path, image))
dome_texture_paths = [
            dome_texture_path + dome_texture + ".hdr" for dome_texture in dome_tex
        ]


# Add camera
# Initialize the camera
pose = camera_pose()
cameras = initialise_cameras(2)


# Add lights
distance_light = rep.create.light(rotation=(315, 0, 0), intensity=3000, light_type="distant")
distance_light_1 = rep.create.light(rotation=(315, 0, 0), intensity=3000, light_type="distant")
#sphere_light = rep.create.light(rotation=(315, 0, 0), intensity=4000, color=(1.0, 1.0, 1.0), light_type="sphere")

# Start simulation
omni.timeline.get_timeline_interface().play()
# perform one simulation step so physics is loaded and dynamic control works.
simulation_app.update()

prim = Articulation(prim_path=prim_path, name="abbyArm")
prim.initialize()
for camera in cameras:
    camera.initialize()

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

# with rep.trigger.on_custom_event(event_name="randomize_sphere_light"):
#     with sphere_light:
        # Calculate random cooridnates
        # Calculate random spherical coordinates
        # theta = random.uniform(-math.pi, 0)  # Azimuthal angle
        # phi = random.uniform(0, math.pi/2)       # Polar angle
        # r = random.uniform(5, 10)
        # # Convert spherical coordinates to Cartesian coordinates
        # x = r* math.sin(phi) * math.cos(theta)
        # y = r* math.sin(phi) * math.sin(theta)
        # z = r* math.cos(phi)

        # # Set the light's position
        # rep.modify.pose(position=(x, y, z),  look_at=(0, 0, 0))
        # rep.randomizer.rotation()
#with rep.new_layer():
# def get_shape():
#         shape = rep.get.prim_at_path("/abbyArm/groundPlane")
#         print(f"Ground plane shape: {shape}")
        
#         with shape:
#             rep.randomizer.texture(textures=
#                     images_paths
#                     )
#         return shape.node
# rep.randomizer.register(get_shape)

def randomize_domelight(texture_paths):
    lights = rep.create.light(
        light_type="Dome",
        rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
        texture=rep.distribution.choice(texture_paths)
    )
    return lights.node

rep.randomizer.register(randomize_domelight, override=True)



with rep.trigger.on_custom_event(event_name="randomize_domelight"):
    rep.randomizer.randomize_domelight(dome_texture_paths)#images_paths
        

print(f"Arm dof upper: {prim.dof_properties['upper']}")
print(f"Arm dof lower: {prim.dof_properties['lower']}")
dof_lower = prim.dof_properties['lower'] + 0.1
dof_upper = prim.dof_properties['upper'] - 0.1

box = setup_collision_box(camera.get_default_state().orientation, np.array([2.0 ,2.0 ,0.0]))
setup_distractors(box)

for i in range(10):
    random_dof_values = np.random.uniform(dof_lower, dof_upper)
    print(f"Random dof values: {random_dof_values}")
    
    state = prim.get_joints_state()
    if np.isnan(state.positions).any():
        #prim = Articulation(prim_path=prim_path, name="abbyArm")
        #prim.initialize()
        prim.set_joint_positions(np.zeros(6))
        #break
    print(f"Joint states: {state.positions}")
    prim.set_joint_positions(random_dof_values)
    rep.utils.send_og_event(event_name="randomize_light")
    rep.orchestrator.step(rt_subframes=1)
    rep.utils.send_og_event(event_name="randomize_light_1")
    rep.utils.send_og_event(event_name="randomize_domelight")
    #rep.utils.send_og_event(event_name="randomize_sphere_light")
    
    rep.orchestrator.step(rt_subframes=1)
    print(f"Camera Position: {camera.get_default_state().position} and Camera Orientation: {camera.get_default_state().orientation}")    
    #print(f"World position: {camera.get_world_pose()[0]} and World orientation: {camera.get_world_pose()[1]}") # this is just to double check the camera position

    for j, camera in enumerate(cameras):
        plt.imsave(f"/home/dario/Documents/AUT_Proj/data_gen/Data/rgba_image_{i}_{j}.png", camera.get_rgba()[:, :, :3])
    simulation_app.update()
    
rep.orchestrator.wait_until_complete()
simulation_app.close() # close Isaac Sim
