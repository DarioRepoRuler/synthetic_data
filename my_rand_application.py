from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(launch_config={"headless": False})

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
scene.CreateGravityMagnitudeAttr().Set(0.0) # we don't want gravity to affect the robot
# Set solver settings
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
physxSceneAPI.CreateEnableCCDAttr(True)
physxSceneAPI.CreateEnableStabilizationAttr(True)
physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
physxSceneAPI.CreateSolverTypeAttr("TGS")

# Add ground plane
omni.kit.commands.execute(
    "AddGroundPlaneCommand",
    stage=stage,
    planePath="/groundPlane",
    axis="Z",
    size=1500.0,
    position=Gf.Vec3f(0, 0, -2),
    color=Gf.Vec3f(0.5),
)
# Add camera
# Initialize the camera
pose = camera_pose()

cameras = initialise_cameras(1)


# Add lights
distance_light = rep.create.light(rotation=(315, 0, 0), intensity=2000, light_type="distant")
distance_light_1 = rep.create.light(rotation=(315, 0, 0), intensity=2000, light_type="distant")
sphere_light = rep.create.light(rotation=(315, 0, 0), intensity=2000, color=(1.0, 1.0, 1.0), light_type="sphere")

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

with rep.trigger.on_custom_event(event_name="randomize_sphere_light"):
    with sphere_light:
        # Calculate random cooridnates
        x,y = np.random.uniform(-10, 10,2)
        z = np.random.uniform(0, 10)
        
        # Set the light's position
        rep.modify.pose(position=(x, y, z),  look_at=(0, 0, 0))
        rep.modify.attribute("color", (random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)))
        


print(f"Arm dof upper: {prim.dof_properties['upper']}")
print(f"Arm dof lower: {prim.dof_properties['lower']}")
dof_lower = prim.dof_properties['lower']
dof_upper = prim.dof_properties['upper']



for i in range(10):
    random_dof_values = np.random.uniform(dof_lower, dof_upper)
    print(f"Random dof values: {random_dof_values}")
    
    state = prim.get_joints_state()
    if np.isnan(state.positions).any():
        break
    print(f"Joint states: {state.positions}")
    prim.set_joint_positions(random_dof_values)
    rep.utils.send_og_event(event_name="randomize_light")
    rep.utils.send_og_event(event_name="randomize_light_1")
    #rep.orchestrator.step(rt_subframes=1)
    rep.utils.send_og_event(event_name="randomize_sphere_light")
    
    rep.orchestrator.step(rt_subframes=10)
    print(f"Camera Position: {camera.get_default_state().position} and Camera Orientation: {camera.get_default_state().orientation}")    
    #print(f"World position: {camera.get_world_pose()[0]} and World orientation: {camera.get_world_pose()[1]}") # this is just to double check the camera position

    for j, camera in enumerate(cameras):
        plt.imsave(f"/home/dario/Documents/AUT_Proj/data_gen/Data/rgba_image_{i}_{j}.png", camera.get_rgba()[:, :, :3])
    simulation_app.update()
    
rep.orchestrator.wait_until_complete()
simulation_app.close() # close Isaac Sim
