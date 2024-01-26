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


world = World()
world.scene.add_default_ground_plane()
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find nucleus server with /Isaac folder")
asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
#add_reference_to_stage(usd_path=asset_path, prim_path="/World/Fancy_Robot")
jetbot = world.scene.add(WheeledRobot(prim_path="/World/Fancy_Robot", name="fancy_robot",
                                      wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                                      create_robot=True, usd_path=jetbot_asset_path,)
                         )

print("Num of degrees of freedom before first reset: " + str(jetbot.num_dof))
# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly
world.reset()
for i in range(500):
    jetbot.apply_wheel_actions(ArticulationAction(joint_positions=None,

                                                            joint_efforts=None,

                                                            joint_velocities=5 * np.array([1.0 , 1.0]) ))
    
    
    position, orientation = jetbot.get_world_pose()
    linear_velocity = jetbot.get_linear_velocity()
    # will be shown on terminal
    print("Jetbot position is : " + str(position))
    print("Jetbot orientation is : " + str(orientation))
    print("Jetbot linear velocity is : " + str(linear_velocity))
    # we have control over stepping physics and rendering in this workflow
    # things run in sync
    world.step(render=True) # execute one physics step and one rendering step
simulation_app.close() # close Isaac Sim