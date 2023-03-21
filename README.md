# u0941152-Project2
## How to run:
There are two driving scripts for this project, run_rrt.py and run_prm.py. Modifying parameters within each of these files will run the desired algorithm within the given configuration. There is a vscode.launch configuration created within the root directory, with RUN_RRT and RUN_PRM launch configurations. 


## Configuration Description
RRT
- `problem` String corresponding to a file within the `Project 2` workspace. This must be a txt file, or `"vrep"` if using Coppelia sim
- `connect` Boolean determining if build_rrt_connect should be run (if `True`, bidirection should be false)
- `bidirection` Boolean determining if build_bidirectional_rrt_connect should be run (if `True`, connect should be false)
- If both `connect` and `bidirection` are false, `build_rrt` will be run. 
- `num_samples` number of samples that will be taken in rrt builds. 
- `_CONNECT_PROB` probability in each iteration that the goal state is sampled. 
- `_STEP_LENGTH_POLYGON` step length used in extend function for polygon environment. 
    - Only applies to polygon environments, not to vrep problems
- `_STEP_LENGTH_VREP` Step length used for Coppelia sim

Depending on the IDE or python instance used, a breakpoint may need to be added *after* line to correcltly output the visual from `environment.draw_plan`. This wasn't an issue, but occassionally popped up. 

PRM 
- `num_samples` Number of nodes that a PRM should contain. Higher number means a more sophisticated map, but also high computation time. 
- `problem` Similar to RRT. This should be a filename string of the desired environment to test.
- `_STEP_LENGTH_POLYGON` float describing step lenght in polygon environment. Essentially anything not run through coppelia sim
- `_STEP_LENGTH_VREP` float wth step length for vrep wrapper. n
- `_RADIUS` Radius which PRM should use in sampling possible points. A higher radius will lead to more paths, but higher computation time.

### RRT specific
There are 2 environments, env0 and env1. There are additional 