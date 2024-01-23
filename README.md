# Object Shape Exploration
Codes for our paper [Online active and dynamic object shape exploration with a multi-fingered robotic hand](https://www.sciencedirect.com/science/article/pii/S0921889023001008)
## Installation
- Go to the package folder `inhand_exploration` and run the script `environment_configuration.m` to configure the path.

## Quick Start
### To perform an analysis of the Allegro hand model
- Go to the folder `hand_sampling`;
- Run script `main.m` to analyze and visualize the intersection region between hand's reachability space and the target object's surface in Cartesian space. This intersection region could be used to plan the next exploration position of the finger tip.

## Data format
- Object models are saved as point cloud data under `database`.
For `csv` file, the point cloud is an `(N,6)` matrix. `N` is the number of data samples; columns save the `[x,y,z,u,v,w]` coordinates data. `[x,y,z]` are coordinates, and `[u,v,w]` represent the surface normals.  

## Third-party packages
- `SynGrasp2.3` package is used in MATLAB to create the hand model. It has been replaced by customized package `mySynGrasp` in the current version.

## Signal estimation strategy
The key challenge is to estimate the rotation angle of the explored object. Currently, the estimation considers three inputs:
- rotation command sent to robot;
- similarity between sampled data in this trial and the entire collected dataset; 
- estimation based on sensor signal (surface norm etc);

The robot cmd is used as the main information source. The other two inputs are used as correction terms.