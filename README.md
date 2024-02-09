<h1 align="center">
Synthetic data generation for robopose
</h1>
This repo provides a synthetic data generation tool utilizing isaac sim. It is specifically designed to provide trainig data for robopose. Robopose estimates a robot's state, which includes the 6D pose of an anchor together with the joint positions using only a RGB image. In order to train the robopose network many images are needed with various conditions which can be achieved more easily by synthtic data. However the ground truth needed for this network are the camera to robot transformation, the joint positions, 3d keypoints(of every base) and the 2D projected keypoints.


<br />
This repo takes a robot (ABB IRB120 per default) randomizes its joint positions within the limits and captures the ground truth data for the robopose network. 
To achieve a better synthetic to real world performence of the network various randomizations are implemented in the data such as domain randomization of the background, distractors, randomizations of light sources(position and color).

![synthetic_data](/doc_img/synthetic_new.gif)
## Setup
This repo is not wrapped in a docker container so the setup specifications are important. This repo was designed on `Ubuntu 22.0.4` with a `Nvidia Geforce RTX 3070` GPU and a `AMD® Ryzen 9 3900x 12-core` CPU.
The nvidia GPU was operated with `525.147.05` Nvidia driver. The used isaac sim version is `2023.1.1`.

## Installation
The base installation procedure is explained in [this](https://www.youtube.com/watch?v=ZUX9SrPGrbk&t=302s) youtube tutorial. 

For the sake of completness the procedure is sketched here as well:
- First register, download and install NVIDIA's omniverse from the [official website](https://www.nvidia.com/de-de/omniverse/). We have used the `free` option. 
- Within the omniverse launcher install `isaac sim 2023.1.1`, `omniverse cache 2023.1.0` and the `omniverse nucleus navigator 3.3.3`. 
- After that add a `local host` connection to the nucleus navigator this is important since it also gives you access to some isaac specific entities. 

## Generating data
There are different [workflows](https://docs.omniverse.nvidia.com/isaacsim/latest/introductory_tutorials/tutorial_intro_workflows.html#isaac-sim-app-tutorial-intro-workflows) within isaac sim, we are using the `python standalone` workflow for this project since we can easily controll the robot and the time stepping. 

- To execute such a standalone application you first have to find the `python.sh` file which comes with the omniverse. It is usually located somewhere like: `/home/user_name/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh`. This shell script is important because it executes not only python but also sets important paths accordingly. 
- Place this repo as `user_examples` folder into the `examples` folder of isaac sim. This folder should be located somewhere like `/home/user_name/.local/share/ov/pkg/isaac_sim-2023.1.1/exts/omni.isaac.examples/omni/isaac/examples/`. The folder structure should then look like:
- Import the `flying distractors` from the nvidia code folder into your `user_examples` folder. The distractors folder should be somewhere like `/home/user_name/.local/share/ov/pkg/isaac_sim-2023.1.1/standalone_examples/replicator/offline_pose_generation`. 

Your folder structure should then look like this:
```
├── USER_EXAMPLES
│   ├── init_.py
│   ├── hello_world_extension.py
│   ├── hello_world.py
│   ├── my_application_1.py
│   ├── my_application.py
│   ├── my_rand_application_2.py
│   ├── my_rand_application.py
│   ├── my_random.py
│   ├──synthetic_generator.py
│   ├── (playground.py)
│   └── README.md
├── flying_distractors
│   ├── __init__.py
│   ├── collision_box.py
│   ├── dynamic_asset_set.py
│   ├── dynamic_object_set.py
│   ├── dynamic_object.py
│   ├── dynamic_shape_set.py
│   └──flying_distractors.py
└──abb_common
    ├── meshes
    │   ├── ...
    ├── urdf
    │   ├── ...
    ├── urdf_old
    │   ├── ...
    ...
```

Once is achieved and the `python.sh` shell script is found you can simply execute the written code via:
```
/home/dario/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh /home/dario/.local/share/ov/pkg/isaac_sim-2023.1.1/exts/omni.isaac.examples/omni/isaac/examples/user_examples/synthetic_generator.py
```
This script will now simulate random positions under random lightning conditions with random initilaised camera posiions. The synthetic data will then be stored in a new folder `synthetic_data`. 
The folder should then contain captured images in the format `{image_number}_{camera_number}.png` and the recorded joint positions, 6d pose of the base and the 3D as well as the projected keypoints sampled in a json file in the format `{image_number}_{camera_number}.json`. 

## Ressources for Isaac Sim
- [First steps in isaac sim](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html#isaac-sim-app-tutorial-core-hello-world)
- [Introduction of differen workflows](https://docs.omniverse.nvidia.com/isaacsim/latest/introductory_tutorials/tutorial_intro_workflows.html#isaac-sim-app-tutorial-intro-workflows)
- [How to install packages in isaac sims python](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html)
- [Python Core API documentation](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html)
- [Python standalone example](https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_camera.html#python-example)
- [Python replicator documentation](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.replicator.isaac/docs/index.html)
- [Python replicator API](https://docs.omniverse.nvidia.com/py/replicator/1.10.10/source/extensions/omni.replicator.core/docs/API.html#writers)
- [Replicator tutorials](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/tutorial_replicator_training_pose_estimation_model.html)
- [Domain Randomization](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/tutorial_replicator_offline_pose_estimation.html)
- [Some more python snippets](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/tutorial_replicator_isaac_snippets.html)




