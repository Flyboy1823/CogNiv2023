# CogNiv2023
#
#The idea of the following files is to create a code for two robots that will be the "waiters" at a restaurant.  They will be recieving orders to be delivered to the #tables.  They must use logical motion planning to create an optimal path to their targets, and once there, use the arm to hand the costumers their orders.  In order #to asses the opitmal paths for the two robots, an A* algorthem on a PRM plan was used.  With that, the code uses many functions to show the process of the motion #planning to the user.  The following is an explenation of each file:

#Robotic_waiter.py:
#This is the main file.  Here, the coder chooses which tables in a 4x4 tabled restaurant the robots must deliver the orders too and plan a path towards.  #Additionaly, random obsitcles are added within map of the restaurant because we wouldn't want to make things too easy.  This file is where the motion planning is #executed, and where the outputs are created.  The outputs are the plots of the stages of both algorithms and the final resulted trajectories.

#Nav_MotionPlanner.py:
#This is the code that creates the roadmaps for the planned trajectory of the two robots. It takes into account the moving obsicles on the map, aswell as the goals #assigned to each robot.  Here, the A* algorithem is implmented to create the trajectory and the PRM solver is implemented to create the plan for the robots.

#Nav_Plotter.py:
#This file is used to house all the functions related to the plotting throughout the runtime of the execution.  The plotting of the table, moving obsticles, goals, #robots' initial states, PRM plan, A* algorithm, and planned trajectories are all taken into account in making several different plots.  The plots show a simplified #TRF graph, a final trajectory plan of the robots, and an assesment of the nodes used within the calculations to get to these trajectories.

#Nav_TaskPlanner.py:
#Here, the functions to solve a pddl problem are housed.  The problem itself is given an action count to stay optomized, and the goals and obsticles inputded in #Ignorance are used to create a pddl solution.

#New_Geometry.py
#This is a file used to house the functions that bring the goals, obsitcles and robots into the 2D world.  It houses functions that create the limitations of the map #and each of these components as well as functions that deal with checkign interactions in between them.  

#Arm Motion Planning
To start off, our waiter robot needs to pick up a dish from the restaurant counter using its 4 revolute joint  arm. The goal is to reach the destination efficiently while avoiding any obstacles in the way, and then transfer it onto the tray of the robot.. We decided to use RRT and RRT* algorithms to create a path for the robot arm within a known working area.

#Rapidly Exploring Random Trees (RRT)
The RRT algorithm is a popular motion planning algorithm that generates a tree-like structure in the configuration space of the robot. RRT works by randomly generating new nodes in the configuration space and extending the search tree towards those nodes until the goal is reached. However, this process may lead to suboptimal or inefficient paths, especially in large or complex search spaces.

#RRT
The RRT algorithm randomly samples configurations in the configuration space (C-Space) and expands the tree towards the nearest existing node. Each new node in the tree represents a new configuration of the robot arm, including 4 random joint degrees. The forward kinematics of the arm are used to detect collisions with the environment, and nodes resulting in collisions are rejected. Nodes without collisions are added to the tree with the nearest node as their parent, and this process continues until a configuration is found that allows the robot arm to reach the goal. The trajectory to the goal is obtained by following the path in the tree from the initial node to the goal node. During the process we accumulate the cost of node with respect to his parent, which is measured by the Euclidean distance the robot end-effector goes through the process.  

#RRT*
The RRT* algorithm works in a similar way to RRT, with the added step of finding a better parent for each new node added to the tree. After a new node is added to the tree, RRT* searches for nearby nodes that could serve as a better parent for the new node. This is done by checking the cost of reaching the new node through each of its neighboring nodes. If a better path to the new node is found, the parent of the new node is reassigned to the neighbor with the lower cost. In addition to finding better parents, RRT* also performs a rewiring process. This involves searching for nodes in the tree that are farther away from the root than the new node and checking if the new node can provide a better path to those nodes. If a better path is found, the parent of the previously existing node is reassigned to the new node.

#Extend
This function extends the tree by creating a new node that is a step size away from the nearest node towards a randomly sampled configuration. It then performs forward kinematics to get the position of the gripper for both the new node and the nearest node, calculates the cost as the Euclidean distance between the two positions, and creates a new tree node with the vectorized joint states of the new node. If the new node does not already exist in the tree, the new node and its cost are returned. If the new node already exists in the tree, an error message is printed and nothing is returned.

#Nearest Neigbours
This function takes as input the new node, an iteration number, and a constant d.The function iterates through all the nodes in the tree and adds their indices to the list if they are within the radius and not the goal node, and it returns a list of indices corresponding to the nearest neighbors of the node within a certain radius in the tree.

#Choosing Best Parent
This function chooses the parent node for a newly added node based on the cost of reaching the parent node and the cost of reaching the new node from the parent node. It takes in the newly added node, a list of indices for the nearest nodes, the cost of reaching the new node from its original parent, and the index of the nearest node to the new node.
For each nearest node, it extends the path from the nearest node to the new node and calculates the cost of the path. If the path is collision-free and has a lower cost than the current minimum cost, the nearest node is set as the new node's parent and its cost is updated. The function then returns the node with the lowest cost as the parent of the new node. If no path is found, the function returns False.

#Rewire
The rewire function is used to update the cost of the nodes in the RRT tree. It takes in two parameters: update_new_node, which is the newly added node that will be used to rewire the tree, and near_inds, which is a list of indices of nearby nodes in the tree.
For each node in near_inds, the function calculates the cost of the edge between that node and update_new_node. If the cost is less than the current cost of the node, then the function updates the node's cost and predecessor to update_new_node, and recursively updates the cost of its successors in the tree.Overall, this function helps improve the efficiency of the RRT algorithm by rewiring the tree to find better paths towards the goal.

#Optimization Constraint
As we set out to solve the problem of having the robot arm navigate to a desired, we chose to use the RRT* algorithm due to its effectiveness in finding a feasible path in high-dimensional state spaces.
To represent the state of the robot, we chose to use a 4D vector to represent the radians values for the robot's revolute joints. However, we recognized that optimizing solely for the joint angles themselves may not necessarily lead to a smooth and efficient motion towards the goal.
Therefore, we chose to optimize for the end-effector position instead, which is what ultimately matters for achieving the desired task. In each iteration of the algorithm, we generate 4 random radians values for the joints, resulting in a 4D vector, and compare the Euclidean distance between the end-effector positions of neighboring nodes. This ensures that the robot moves in a smooth and continuous manner towards the goal, rather than making sudden jumps in the joint space.

