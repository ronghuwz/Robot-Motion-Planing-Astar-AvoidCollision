# Robot-Motion-Planing-Astar-AvoidCollision

### Both matlab & C++ code available.

For the C++ program, you'll need OpenCV for visualization. For macOS, install it using:
```sh
brew install opencv
```


C++ compilation and linking using g++ with OpenCV support: 
```sh
g++ -std=c++17 -o astar_robot main.cpp `pkg-config --cflags --libs opencv4`
```
Run the program with:
```sh
./astar_robot
```


### How it works:
Two robot working on the same work space, from starting point to goal and return to the starting point. Avoid collision while moving.

First,  a grid map was defined where 0 represents free path (black blocks), 1 represents obstacles (grey blocks), and 2 represents a collision region (green blocks).

Then, the starting and goal points were defined for two robots, as well as the locations of two items. Then initialize a figure for animation and plot the grid, the robots' initial positions, and  the collision region.

The A* algorithm was used to find the paths for both robots from their starting points to their respective goal points. We also find the paths for the robots to return from their goal points back to their starting points. The paths are combined to form round trips for each robot.  


The collision region was handled by calling a function named ‘handle_collision’, which ensures that only one robot can be present in the collision region at a time. This function modifies the paths to include necessary waiting steps, thereby preventing collisions between the robots in the collision region. The function updates the paths to ensure safe and efficient navigation through the grid.


In the collision handling approach, it aimed to ensure safe and efficient navigation for both robots in the presence of a defined collision region.  
Before collision handling, the optimal paths for each robot are already determined using the A* algorithm. The task is to identify the necessary waiting steps for each robot to avoid collisions. We then incorporate these additional waiting steps into their respective paths, ensuring both robots can navigate safely and efficiently through the grid without colliding. 


The `handle_collision` function takes the optimum paths( found by A*) of both robots, the grid map, and the plot handles as input. It iteratively checks the positions of each robot and determines if they are within the collision region. 


If neither robot is in the collision region, both robots move simultaneously along their respective paths. However, if one robot is in the collision region and the other is not, the robot inside the collision region continues moving towards its goal. The robot outside the region will move forward if its next step is not within the collision region. If the next step is within the collision region, the outside robot waits at the boundary, adding its current position to the final path, until the robot inside the collision region exits. Once the inside robot exits the collision region, the waiting robot can then proceed. 


If both robots are about to enter the collision region at the same time, priority is given to robot 2. This ensures that the robots take turns and avoid collisions in all cases.  


The final paths for each robot are then constructed based on these conditions, and by adding the necessary waiting steps to their path.  By implementing this collision handling strategy, we guarantee that the robots navigate safely and efficiently, even in the presence of potential collision scenarios. 
