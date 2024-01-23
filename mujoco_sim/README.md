# Mujuco simulation

## Installation
- create a new conda environment with `python=3.7`.
- install [mujoco 2.1](https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz) and [mujoco_py](https://github.com/openai/mujoco-py).
read the `readme` in [mujoco_py](https://github.com/openai/mujoco-py) for details.

### Solutions of some bugs 
- 'distutils.errors.CompileError: command 'gcc' failed with exit status 1' 
 ```
 git clone https://github.com/openai/mujoco-py
cd mujoco-py
pip install -e . --no-cache (run it in a conda env)
```
 - `gcc` error,

`sudo apt-get install -y libglew-dev`

add following in .bashrc
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/xiao/.mujoco/mujoco210/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so
``` 
if `No such file or directory: 'patchelf' `,  run
`sudo apt-get install patchelf`.

if run one demo in the `examples` in `mujoco_py` package, It should show a GUI of the simulation environment.

## inhand Exploration
- `inhand_exploration*.py`, receives data (allegro joints, palm pose) from Matlab `main_exploration.m` by ROS, and adds transformation. Both `inhand_exploration*.py` and `main_exploration.m`
should be run at th same time.
- The pose of the hand base is directly set. Fingers are impedance-controlled.
- Use `r.exploration_contact` to get all contact points and contact forces.
- Set `obj_name` and `scalingFactor` in `inhand_exploration.py` for different objects.
- run `inhand_exploration_replay.py` for replaying the results in MuJoCo in pure kinematics.


## object model preprocess
Since Mujoco simulator only detects collision between convex bodies, 
we need to  represent the non-convex objecs as a union of convex objects, 
by defining multiple convex geoms within the same mujoco body.
(otherwise Mujoco will use the convex hull of the body.)
Please do it as the following steps:
- if `scalingFactor != 1`, open [MeshLab](https://www.meshlab.net/), import the `.ply` file. Click `Filter - Normals, Cur...  - Transfer: Scale`,
input the `scalingFactor` (the same as `main_exploration.m`). Then save the new body as `.ply`.
- transfer `.ply` file to `.obj` by an online [website](https://products.aspose.app/3d/conversion/ply-to-stl).
- cut the nonconvex `.obj` model into convex geometries by [v-hacd](https://github.com/kmammou/v-hacd) (follow its readme to install the package).
Run `./TestVHACD  apc_red_bowlx2.obj -v 800 -e 0.02 -p true -r 3000000 -s true` and get `decomp.obj`. You can use Meshlab to have a look.
- Download `pymesh2-0.3-cp37-cp37m-linux_x86_64.whl` at this [link](https://github.com/PyMesh/PyMesh/releases),
and install it in the conda env by `pip install pymesh2...whl`.
- Modify the `name` and `model` in `obj2xml.ipynb` and run all cells. This will cut the `decomp.obj` into
separate parts of `.obj` files and then transfer them into `.stl` files by `pymesh`. Finally, two xml files are 
generated for Mujoco.
- Integrate the two `xml` files as what I did in `inhand_exploration.xml`

### Usage of V-HACD
` TestVHACD <wavefront.obj> (options)`
`./TestVHACD apc_2.obj -v 1024 -e 0.001 -p true -r 3000000 -s true -h 256`

-h <n>                  : Maximum number of output convex hulls. Default is 32\
-r <voxelresolution>    : Total number of voxels to use. Default is 100,000 \
-e <volumeErrorPercent> : Volume error allowed as a percentage. Default is 1% \
-d <maxRecursionDepth>  : Maximum recursion depth. Default value is 12.\
-s <true/false>         : Whether or not to shrinkwrap output to source mesh. Default is true.\
-f <fillMode>           : Fill mode. Default is 'flood', also 'surface' and 'raycast' are valid.\
-v <maxHullVertCount>   : Maximum number of vertices in the output convex hull. Default value is 64\
-a <true/false>         : Whether or not to run asynchronously. Default is 'true'\
-l <minEdgeLength>      : Minimum size of a voxel edge. Default value is 2 voxels.\
-p <true/false>         : If false, splits hulls in the middle. If true, tries to find optimal split plane location. False by default.
