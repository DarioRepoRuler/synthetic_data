from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp(launch_config={"headless": False})
import os
import omni.replicator.core as rep
import omni.usd

import math
import random

omni.usd.get_context().new_stage()
distance_light = rep.create.light(rotation=(315, 0, 0), intensity=4000, light_type="distant")
sphere_light = rep.create.light(rotation=(0, 0, 0), intensity=5e6, color=(0.0, 0.0, 1.0), light_type="sphere")

large_cube = rep.create.cube(scale=1.25, position=(1, 1, 0))
small_cube = rep.create.cube(scale=0.75, position=(-1, -1, 0))
large_cube_prim = large_cube.get_output_prims()["prims"][0]
small_cube_prim = small_cube.get_output_prims()["prims"][0]

cam = rep.create.camera(position=(0, 0, 5), look_at=(0, 0, 0))
rp = rep.create.render_product("/OmniverseKit_Persp", (512, 512))

writer = rep.WriterRegistry.get("BasicWriter")
out_dir = os.getcwd() + "/_out_custom_event"
print(f"Writing data to {out_dir}")
writer.initialize(output_dir=out_dir, rgb=True)
writer.attach(rp)

with rep.trigger.on_custom_event(event_name="randomize_large_cube"):
    with large_cube:
        rep.randomizer.rotation()
with rep.trigger.on_custom_event(event_name="randomize_small_cube"):
    with small_cube:
        rep.randomizer.rotation()
        
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

        
def run_example():
    print(f"Randomizing small cube")
    rep.utils.send_og_event(event_name="randomize_small_cube")
    print("Capturing frame")
    rep.orchestrator.step(rt_subframes=8)
    print("Moving small cube")
    small_cube_prim.GetAttribute("xformOp:translate").Set((-2, -2, 0))
    print("Capturing frame")
    rep.orchestrator.step(rt_subframes=8)
    print(f"Randomizing large cube")
    rep.utils.send_og_event(event_name="randomize_large_cube")
    print("Capturing frame")
    rep.orchestrator.step(rt_subframes=8)
    print("Moving large cube")
    large_cube_prim.GetAttribute("xformOp:translate").Set((2, 2, 0))
    print("Capturing frame")
    rep.orchestrator.step(rt_subframes=8)
    rep.utils.send_og_event(event_name="randomize_light")
    print("Capturing frame")
    rep.orchestrator.step(rt_subframes=8)
    rep.utils.send_og_event(event_name="randomize_sphere_light")
    print("Capturing frame")
    rep.orchestrator.step(rt_subframes=8)
    rep.utils.send_og_event(event_name="randomize_sphere_light")
    print("Capturing frame")
    rep.orchestrator.step(rt_subframes=8)
    rep.utils.send_og_event(event_name="randomize_sphere_light")
    print("Capturing frame")
    rep.orchestrator.step(rt_subframes=8)
    # Wait until all the data is saved to disk
    rep.orchestrator.wait_until_complete()

run_example()

simulation_app.close()
