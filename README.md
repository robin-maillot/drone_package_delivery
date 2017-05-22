# Multi-drone-delivery-system

#### Goal
Implement a decentrialized controller for a multi-drone delivery system. Waypoints for the package are predefined and the goal is to follow them as closely as possible. The environment is composed of static buildings, dynamic objects and unknown wind forces.


#### Implementation

The implementation is done in **C#** through **Unity**.
All files modified are contained in [Assets](https://github.com/robin-maillot/drone_package_delivery/tree/master/Assets).

test_problem.json can be used as a reference for how maps are defined.
test_problem_newdiag.json is the latest map.

| Author              		 | GitHub                                            |
|:---------------------------|:--------------------------------------------------|
| Robin Maillot   			 | [robin-maillot](https://github.com/robin-maillot) |
| Kevin Holdcroft			 | [Xtracheese23](https://github.com/Xtracheese23) |

#### Algorithm

The algorithm is behaviour based. Each behaviour is transformed into a vector and each agent is driven by the sum of these vectors. The components choosen as well as a brief descriptions of the calculation steps are described below:

- Avoiding buildings:
	- Find closest building point
	- Calculate distance and orientation
- Getting to the goal:
	- Calculate distance to goal (Naive using GPS coordinates)
- Keep in formation:
	- ???
Each vector is then multiplied by a certain weight (found empirically).
Wind is added (using a 2 dimensional gaussian model) as an unknown disturbance and the weight of each drone and the package is also added.

#### Advancement
- [X] 3D vizualization in Unity (dynamic camera also added)
- [X] 3D collision detection (already implemented in **Unity**)

#### TODO

- [ ] Implement path planning
	- [X] Set waypoints
	- [ ] Possibility to go above buildings
	- [X] Deviate from initial formation (a new formation can be set at each waypoint)
	- [ ] Convert PID and behavior based controller to 3D
- [X] Add disturbances (ie wind and weights)
- [X] Add moving obstacles
