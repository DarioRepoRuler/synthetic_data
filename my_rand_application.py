from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(launch_config={"headless": False})
import os
import omni.replicator.core as rep
import omni.usd

import math
import random


from omni.isaac.core import World
from pxr import Gf, PhysxSchema, Sdf, UsdLux, UsdPhysics

# Import the URDF extension interface
from omni.importer.urdf import _urdf
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.core.articulations import Articulation, ArticulationView

import numpy as np

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


stage = omni.usd.get_context().get_stage()
# Enable physics
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
# Set gravity
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(9.81)
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
    position=Gf.Vec3f(0, 0, -0.1),
    color=Gf.Vec3f(0.5),
)

# Add lights
distance_light = rep.create.light(rotation=(315, 0, 0), intensity=4000, light_type="distant")
sphere_light = rep.create.light(rotation=(315, 0, 0), intensity=4000, color=(1.0, 1.0, 1.0), light_type="sphere")

# Start simulation
omni.timeline.get_timeline_interface().play()
# perform one simulation step so physics is loaded and dynamic control works.
simulation_app.update()

prim = Articulation(prim_path=prim_path, name="abbyArm")
prim.initialize()

# Need more cameras!
cam = rep.create.camera(position=(0, 0, 5), look_at=(0, 0, 0))
rp = rep.create.render_product(cam, (512, 512)) #"/OmniverseKit_Persp"

writer = rep.WriterRegistry.get("BasicWriter")
out_dir = os.getcwd() + "/_out_custom_event"
print(f"Writing data to {out_dir}")
writer.initialize(output_dir=out_dir, rgb=True)
writer.attach(rp)
        
with rep.trigger.on_custom_event(event_name="randomize_light"):
    with distance_light:
        # Calculate random spherical coordinates
        theta = random.uniform(0, 2 * math.pi)  # Azimuthal angle
        phi = random.uniform(0, math.pi)       # Polar angle

        # Convert spherical coordinates to Cartesian coordinates
        x = math.sin(phi) * math.cos(theta)
        y = math.sin(phi) * math.sin(theta)
        z = math.cos(phi)

        # Set the light's position
        rep.modify.pose(position=(x, y, z),  look_at=(0, 0, 0))
        rep.randomizer.rotation()
        # You can also add randomization for light intensity here if needed

with rep.trigger.on_custom_event(event_name="randomize_sphere_light"):
    with sphere_light:
        # Calculate random spherical coordinates
        theta = random.uniform(0, 2 * math.pi)
        phi = random.uniform(0, math.pi)
        
        # Convert spherical coordinates to Cartesian coordinates
        x = math.sin(phi) * math.cos(theta)
        y = math.sin(phi) * math.sin(theta)
        z = math.cos(phi)
        
        # Set the light's position
        rep.modify.pose(position=(x, y, z),  look_at=(0, 0, 0))
        rep.modify.attribute("color", (random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)))

  


for i in range(30):
    state = prim.get_joints_state()
    print(f"Joint states: {state.positions}")
    #prim.apply_action(action)
    prim.set_joint_positions(np.array([ 0.0 , np.pi/7, -np.pi/5, 0.0, 0.0, 0.0]))
    
    rep.utils.send_og_event(event_name="randomize_light")
    print("Capturing frame")
    rep.orchestrator.step(rt_subframes=8)
    rep.utils.send_og_event(event_name="randomize_sphere_light")
    print("Capturing frame")
    rep.orchestrator.step(rt_subframes=8)
    #rep.orchestrator.step()
    simulation_app.update()
    
rep.orchestrator.wait_until_complete()

simulation_app.close() # close Isaac Sim
