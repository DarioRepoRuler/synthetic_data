#launch Isaac Sim before any other imports
#launch Isaac Sim before any other imports
#default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.


from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

# from Hello Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
import carb
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.utils.types import ArticulationAction



# for urdf
from omni.importer.urdf import _urdf
from omni.isaac.core.utils.extensions import get_extension_path_from_name
import omni.kit.commands
import omni.usd
import os
from omni.isaac.core.articulations import Articulation, ArticulationView
from pxr import Gf, PhysxSchema, Sdf, UsdLux, UsdPhysics


# for capturing pictures
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file

from omni.isaac.sensor import Camera
import omni.isaac.core.utils.numpy.rotations as rot_utils
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


# For creating a light
import omni.isaac.core.utils.prims as prim_utils





world = World()
world.scene.add_default_ground_plane(z_position = -0.1)


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
import_config.default_drive_strength = 1047.19751
import_config.default_position_drive_damping = 52.35988
import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
import_config.distance_scale = 1
import_config.density = 0.0
# Get the urdf file path
extension_path = get_extension_path_from_name("omni.importer.urdf")
print(f"Extension path: {extension_path}")
root_path = extension_path + "/data/urdf/robots/franka_description/robots"
root_path = "/home/dario/Documents/AUT_Proj/data_gen/abb_common"
file_name = "urdf/irb120.urdf"

#print(f"Path loaded: {os.path.join(root_path, file_name)}")
# Finally import the robot
result, prim_path = omni.kit.commands.execute( "URDFParseAndImportFile", urdf_path=os.path.join(root_path, file_name),
                                                import_config=import_config, get_articulation_root=True,)

print(f"Prim path {prim_path}")
prim = Articulation(prim_path=prim_path, name="abbyArm")
#print(f"DOF names: {prim.dof_names}")
#print(f"Properies: {prim.dof_properties}")
action = ArticulationAction(joint_positions=np.array([ 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0]))
prim_v = ArticulationView(prim_paths_expr= prim_path, name="abbyArm_view")



# Generate a random position
position = np.random.uniform(0, 10, 3)
#position = np.array([-5.0, -5.0, 0.0])
print(f"Position: {position}")
# Calculate the direction vector to the origin
direction = -position

# Normalize the direction vector
direction /= np.linalg.norm(direction)
print(f"Direction: {direction}")

# Calculate the camera's current forward vector
forward = np.array([1, 0, 0]) 

# Calculate the rotation vector
rotation_vector = np.cross(forward, direction)
print(f"Rotation vector: {rotation_vector}")
# Calculate the angle of rotation
angle = -np.arccos(np.dot(forward, direction))
print(f"Angle: {angle}")


# Calculate the rotation axis (cross product of initial and desired directions)
rotation_axis = np.cross(forward, direction)
rotation_axis /= np.linalg.norm(rotation_axis)
print(f"Rotation axis: {rotation_axis}")

# Calculate the rotation angle (angle between initial and desired directions)
rotation_angle = np.arccos(np.dot(forward, direction))

# Create a rotation matrix using the axis-angle representation
rotation_matrix = R.from_rotvec(rotation_angle * rotation_axis).as_matrix()

# Extract Euler angles from the rotation matrix
euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz')
euler_angles[0] = 0.0
print(f"Rotation Angles (degrees):", np.degrees(euler_angles))

# Initialize the camera
camera = Camera(
    prim_path="/World/camera",
    position=position,
    frequency=20,
    resolution=(1080,  720),
    orientation=rot_utils.euler_angles_to_quats(np.degrees(euler_angles), degrees=True),
)


light_1 = prim_utils.create_prim(
    "/World/Light_1",
    "SphereLight",
    position=np.array([1.0, 1.0, 1.0]),
    attributes={
        "inputs:radius": 0.01,
        "inputs:intensity": 5e3,
        "inputs:color": (1.0, 0.0, 1.0)
    }
)

# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly
world.reset()
camera.initialize()
prim.initialize()
light_1.initialize()
#camera.add_motion_vectors_to_frame()

print(f"Prim: {prim}")
print(f"Limits: {prim_v.get_dof_limits()}")
vp_api = get_active_viewport()
print(f"VP API: {vp_api.camera_path}")

for i in range(30):
    state = prim.get_joints_state()
    print(f"Joint states: {state.positions}")
    #prim.apply_action(action)
    prim.set_joint_positions(np.array([ 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0]))
    capture_viewport_to_file(vp_api, f"/home/dario/Documents/AUT_Proj/data_gen/Data/picture_{i}")
    world.step(render=True) # execute one physics step and one rendering step
    camera.get_current_frame()
    if i == 29:
        print(f"Limits: {prim_v.get_dof_limits()}")
        print(f"Camera Position: {camera.get_default_state().position} and Camera Orientation: {camera.get_default_state().orientation}")
        print(f"World position: {camera.get_world_pose()[0]} and World orientation: {camera.get_world_pose()[1]}")
        plt.imsave(f"/home/dario/Documents/AUT_Proj/data_gen/Data/rgba_image_{i}.png", camera.get_rgba()[:, :, :3])
simulation_app.close() # close Isaac Sim
