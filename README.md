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
