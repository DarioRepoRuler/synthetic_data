from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(launch_config={"headless": False})

from omni.isaac.core import World

world = World()


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



# Calculates forward kinematics using DH-Konvention
def DH(q):
    q_temp = np.zeros(6)
    q_temp[:np.size(q)] = q
    # Robot-Params
    l02 = 290
    l24 = 340
    l36 = 302
    a2 = 270
    a3= 70
    l6=72
    pi = np.pi
    H = np.eye(4)
    # DH-Params
    theta_i =   np.array([q_temp[0] ,q_temp[1]-pi/2     ,q_temp[2]  ,q_temp[3]+pi   ,q_temp[4]  ,q_temp[5]])
    d_i =       np.array([l02       ,0                  ,0          ,l36            ,0          ,l6  ])
    a_i =       np.array([0         ,a2                ,a3         ,0              ,0          ,0   ])
    alpha_1 =   np.array([-pi/2     ,0                  ,-pi/2      ,-pi/2          ,pi/2       ,0   ]) 
    
    for i in range(np.size(q)):
        Hs = DH_Formula(a_i[i], alpha_1[i], d_i[i], theta_i[i])
        H = np.matmul(H,Hs)
    return H

def DH_Formula(a_i, alpha_i, d_i, theta_i):
    cos = math.cos
    sin = math.sin
    H = np.matrix([[cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), a_i*cos(theta_i)],
                    [sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), a_i*sin(theta_i)],
                    [0, sin(alpha_i), cos(alpha_i), d_i],
                    [0, 0, 0, 1]])
    return H

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



# Add lights
distance_light = rep.create.light(rotation=(315, 0, 0), intensity=3000, light_type="distant")
distance_light_1 = rep.create.light(rotation=(315, 0, 0), intensity=3000, light_type="distant")

# Start simulation
omni.timeline.get_timeline_interface().play()
# perform one simulation step so physics is loaded and dynamic control works.
simulation_app.update()

art = Articulation(prim_path=prim_path, name="abbyArm")
print(f"Articulation: {art.articulation_handle}")
print(f"Prim name :{ art.name}")
print(f"Prim entity: {art.prim}")
print(f"Prim path: {art.prim_path}")
art.initialize()

print(f"Arm dof upper: {art.dof_properties['upper']}")
print(f"Arm dof lower: {art.dof_properties['lower']}")
#print(f"Arm View: {art_v.dof_names}")
import omni.isaac.core.utils.stage as stage_utils
# from omni.isaac.core.robots import Robot
# robot = Robot(prim_path=prim_path, name="ABB IRB 120")
print(f"Stage: {stage_utils.get_current_stage()}")
from pxr import Usd, UsdGeom





for i in range(10):
    #art.set_joint_positions()
    des_q = np.array([0.0 , np.pi/10 , -np.pi/10 , 0.0 , 0.0 , 0.0])
    art.set_joint_positions(des_q)
    state = art.get_joints_state()
    print(f"Joint states: {state.positions}")
    
    # Get the transform of the link you are interested in (replace "link_name" with the actual name of your link)
    link_path = "/abbyArm/link_6"  # Update with the correct path
    link_prim = stage.GetPrimAtPath(link_path)
    
    if link_prim:
        xform = UsdGeom.Xformable(link_prim)
        transform = xform.GetLocalTransformation()
        print(f"Transform of link 'link': {transform}")
        # Extract translation from the matrix
        translation = Gf.Vec3d(transform.ExtractTranslation())
        print(f"Translation: {translation}")
        #print(f"Rotation: {transform.ExtractRotation()}")
        #rotation = Gf.Rotation(transform.ExtractRotation())
        
        #print(f"Transform of link 'link_name': Translation {translation}, Rotation {rotation}")
    else:
        print(f"Link 'link_name' not found in the USD stage")
    
    print(f"Joint states: {state.positions}")
    simulation_app.update()
    
simulation_app.close() # close Isaac Sim
