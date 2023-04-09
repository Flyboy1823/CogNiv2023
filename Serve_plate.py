import numpy as np
from Arm_Modules.Arm_Robot import JointState, Robot
from Arm_Modules.Arm_Plotter import Plotter
from Arm_Modules.Arm_Environment import Environment, distance
from Arm_Modules.Arm_RRT_Planner import RRTPlanner
from Nav_Modules.Nav_Geometry import ObstacleSq

def Serve_plate(output_path=None, save_img=False, save_gif=False):
    """
    Runs the rover arm simulation
    :param output_path: path of the output folder
    :param save_img: True if we want to save our images (.png)
    :param save_gif: True if we want to save our animations (.gif)
    """
    robot = Robot(4)
    RightLeg = ObstacleSq(np.array([6.5, -6]), 1.5, 11)
    LeftLeg = ObstacleSq(np.array([16, -6]), 1.5, 11)
    Plate = ObstacleSq(np.array([5, 3]), 14, 3)
    Table = [RightLeg, LeftLeg, Plate]
    target = np.array([11, 6])
    environment = Environment(robot, Table, [target], epsilon=1, is_wall = False, is_floor = True)
    plotter = Plotter(environment, output_path=output_path, save_img=save_img, save_gif=save_gif)




    """ To check initial/final state uncomment """
    # init_states2 = [0.25*np.pi, 0.5*np.pi, 0.38*np.pi, 0.33*np.pi]
    # fin_states = [1.609, -1.044, 0.063, -1.736]
    # haha = plotter.EEF_kin(fin_states)
    # plotter.plot_state(init_states2)

    # costs = []
    # sons = []
    # i = 2
    # while len(costs) < 10:
    #     # np.random.seed(0)
    #     for j in range(2):
    #         rrt_planner = RRTPlanner(environment, 0.1* i, max_iterations=3000)
    #         init_states = [JointState(0.25 * np.pi), JointState(0.5 * np.pi), JointState(0.38 * np.pi), JointState(0.33 * np.pi)]
    #         tree = rrt_planner.rrt_star(init_states)
    #         if tree is None:
    #             continue
    #         paths, cost, son = rrt_planner.generate_paths()
    #         costs.append(cost)
    #         sons.append(son)
    #         print(len(costs))
    #     i += 1
    # print("Avg of costs is:", sum(costs)/10, sum(sons)/10)

    np.random.seed(0)
    rrt_planner = RRTPlanner(environment, 0.5, max_iterations=3000)
    init_states = [JointState(0.25 * np.pi), JointState(0.5 * np.pi), JointState(0.38 * np.pi), JointState(0.33 * np.pi)]
    tree = rrt_planner.rrt_star(init_states)
    paths = rrt_planner.generate_paths()
    plotter.rrt_plot(paths, tree.nodes, rrt_planner.random_samples, rrt_planner.nearest_nodes)
    nodes = paths[0]
    states1 = [nodes[i].joint_values for i in range(len(nodes))]
    states2 = [nodes[i].joint_values for i in range(len(nodes))]
    states2.reverse()
    states = states1 + states2
    plotter.generate_trajectory(states, output_path, n_frames=3)

    return

if __name__ == '__main__':
    Serve_plate()