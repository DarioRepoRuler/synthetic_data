import math

# Simulation parameters
max_time_steps = 10
angle_offset = 0.1
activate_flying_distractors = True
activate_ground_plane = False

# Camera parameters and position boundaries
focal_length = 5.0
camera_resolution = (1080, 720)
posx = (2, 5)
posy = (-5, 5)
posz = (0, 5)

# Light parameters
light_intensity_lower = 2000
light_intensity_upper = 5000
color_lower = (0, 0, 0)
color_upper = (1, 1, 1)
azimuthal_boundary = math.pi
polar_boundary = math.pi/4
radius_lower = 5
radius_upper = 10

# Background images
use_custom_background_images = False