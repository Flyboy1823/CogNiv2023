import os, shutil
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.animation import PillowWriter
from Arm_Modules.Arm_Robot import JointState
from Nav_Modules.Nav_Geometry import Rectangle, Circle
from Nav_Modules.Nav_Plotter import Create_GIF
import matplotlib.patches as patches


class Plotter:
    def __init__(self, environment, output_path=None, save_img=False, save_gif=False, show_anim=False):
        self.environment = environment
        self.output_path = output_path
        self.background = plt.imread("restaurant2.jpg")
        self.save_img = save_img
        self.save_gif = save_gif
        self.show_anim = show_anim
        self.robot = environment.robot
        self.obstacles = environment.obstacles
        self.targets = environment.targets
        self.gripper_positions = []
        self.frames = []
        self.collision_in_frame = []
        self.goal_in_frame = []

    def plot_obstacles(self):
        for obs in self.obstacles:
            obs.plot_rectangle(self.ax)
        return

    def plot_background(self):
        self.ax.imshow(self.background, extent=[-30, 30, -13, 27])
        return

    def plot_boundaries(self):
        if self.environment.is_floor:
            xy = np.array([-0.5, 0]) * self.robot.n_dof * self.robot.link_length + np.array([0, -5])
            rect = Rectangle(xy, self.robot.n_dof * self.robot.link_length, 5)
            rect.plot_rectangle(self.ax)
        if self.environment.is_wall:
            xy = np.array([-5, 0])
            rect = Rectangle(xy, 5, self.robot.n_dof * self.robot.link_length)
            rect.plot_rectangle(self.ax)
        return

    def plot_rover(self):
        xy = np.array([-10, -5])
        Rectangle(xy, 10, 5).plot_rectangle(self.ax)

        wheel_r = 1.5;
        wheel_color = 'black'
        Circle(wheel_r, np.array([-10, -5]), wheel_color).plot_circle(self.ax)
        Circle(wheel_r, np.array([0, -5]), wheel_color).plot_circle(self.ax)
        wheel_r = 1.2;
        wheel_color = 'grey'
        Circle(wheel_r, np.array([-10, -5]), wheel_color).plot_circle(self.ax)
        Circle(wheel_r, np.array([0, -5]), wheel_color).plot_circle(self.ax)
        return

    def plot_targets(self):
        for target in self.targets:
            Xorg = target[0]
            Yorg = target[1]
            x = [Xorg - 2, Xorg + 2, Xorg + 4, Xorg - 4]
            y = [Yorg - 1, Yorg - 1, Yorg + 0.5, Yorg + 0.5]
            self.ax.add_patch(patches.Polygon(xy=list(zip(x, y)), fill=True, color='white'))
        return

    def EEF_kin(self, node):
        x = 0
        y = 0
        cur_thet = 0
        for i in range(self.environment.robot.n_dof):
            cur_thet += node[i]
            x += np.cos(cur_thet) * self.environment.robot.link_length
            y += np.sin(cur_thet) * self.environment.robot.link_length
        return np.array([x, y])

    def connect_points(self, array, point1, point2):
        xs = [point1[0], point2[0]]
        ys = [point1[1], point2[1]]
        array.append(xs)
        array.append(ys)
        return

    def rrt_plot(self, path, nodes, sample_nodes, nearest_nodes):
        print("Animating the RRT* algorithm...")
        t_delay = 0.01
        self.fig, self.ax = plt.subplots()
        plt.grid()
        self.ax.set_ylim(-8, 24)
        self.ax.set_xlim(-15, 15)
        self.plot_obstacles()

        root_node = nodes.pop(0)
        root_vals = root_node.vectorized_values()
        plt.plot(root_vals[0], root_vals[1], 0.05, color='grey')
        plt.title("RRT* Algorithm in the EEF space")
        plt.xlabel("x [dm]")
        plt.ylabel("y [dm]")
        # if self.save_gif:
        #     print("Producing RRT animation .gif...")
        #     img_list = ['Output/Temp_Images/1.png']
        #     plt.savefig('Output/Temp_Images/1.png')

        for (node, sample_node, nearest_node) in zip(nodes, sample_nodes, nearest_nodes):
            goal_cost = np.inf
            color_set = ['lime', 'gold', 'coral']
            i = 0
            if node.goal == True:
                for idx_n, node in enumerate(path[i]):
                    if idx_n > 0:
                        self.connect_parent_and_child(node, color=color_set[i])

                    xlabel_string_new = xlabel_string + ', Final Path Nodes: ' + str(idx_n + 1) + ' / ' + str(len(path))
                    self.ax.set_xlabel(xlabel_string_new)
                    plt.pause(t_delay/10)
                    # if self.save_gif:
                    #     new_idx = idx + idx_n
                    #     img_list.append('Output/Temp_Images/' + str(new_idx) + '.png')
                    #     plt.savefig('Output/Temp_Images/' + str(new_idx) + '.png')
                    # idx_n += 1
                i += 1
                continue

            idx = nodes.index(node) + 1
            start_vec = self.EEF_kin(path[0][0].vectorized_values())
            goal_vec = self.EEF_kin(path[0][-1].vectorized_values())
            circle1 = plt.Circle((start_vec[0], start_vec[1]), 0.5, color='blue')
            circle2 = plt.Circle((goal_vec[0], goal_vec[1]), 0.5, color='green')
            self.ax.add_patch(circle1)
            self.ax.add_patch(circle2)

            xlabel_string = 'x [mm]   Nodes: ' + str(idx) + ' / ' + str(len(nodes))
            self.ax.set_xlabel(xlabel_string)

            sample_vec = self.EEF_kin(sample_node.vectorized_values())
            node_vec = self.EEF_kin(node.vectorized_values())
            near_vec = self.EEF_kin(nearest_node.vectorized_values())

            circle1 = plt.Circle((sample_vec[0], sample_vec[1]), 0.25, color='blue')
            circle2 = plt.Circle((near_vec[0], near_vec[1]), 0.25, color='orange')
            circle3 = plt.Circle((node_vec[0], node_vec[1]), 0.25, color='grey')
            self.ax.add_patch(circle1)
            plt.pause(t_delay)
            self.ax.add_patch(circle2)
            plt.pause(t_delay)
            self.ax.add_patch(circle3)
            plt.pause(t_delay)
            self.connect_parent_and_child(node)
            plt.pause(t_delay)
            circle1.remove()
            circle2.remove()
            circle3.remove()

            # if self.save_gif:
            #     img_list.append('Output/Temp_Images/' + str(idx) + '.png')
            #     plt.savefig('Output/Temp_Images/' + str(idx) + '.png')

        xlabel_string_final = xlabel_string_new + ', Plotting Complete!'
        self.ax.set_xlabel(xlabel_string_final)

        # if self.save_gif:
        #     for i in range(1, 10):
        #         img_list.append('Output/Temp_Images/' + str(new_idx + i) + '.png')
        #         shutil.copyfile('Output/Temp_Images/' + str(new_idx) + '.png',
        #                         'Output/Temp_Images/' + str(new_idx + i) + '.png')
        #     Create_GIF(img_list, "RRT*_Animation", self.output_path)
        #     print("\t...Done")
        return

    def connect_parent_and_child(self, child_node, color='grey', markersize=3):
        joint_values_parent = self.EEF_kin(child_node.predecessor.vectorized_values())
        joint_values_child = self.EEF_kin(child_node.vectorized_values())
        xs, ys = [joint_values_parent[0], joint_values_child[0]], [joint_values_parent[1], joint_values_child[1]]
        plt.plot(xs, ys, marker='o', color=color, markersize=markersize, linewidth=markersize / 2)
        return

    def generate_cartesian_points(self):
        xs = [joint_pos[0] for joint_pos in self.robot.joint_positions]
        ys = [joint_pos[1] for joint_pos in self.robot.joint_positions]
        xs.append(self.robot.gripper_position[0])
        ys.append(self.robot.gripper_position[1])
        points = []
        for i in range(len(xs) - 1):
            self.connect_points(points, [xs[i], ys[i]], [xs[i + 1], ys[i + 1]])
        return points

    def generate_config_waypoints(self, initial_joint_states, final_joint_states, n_frames, traj_type):
        waypoints = np.array(initial_joint_states)
        a_i = np.array([])
        a_f = np.array([])
        for joint_state_i, joint_state_f in zip(initial_joint_states, final_joint_states):
            a_i = np.append(a_i, joint_state_i.value)
            a_f = np.append(a_f, joint_state_f.value)

        if traj_type == 'linear':
            for i in range(1, n_frames):
                joint_states = np.array([])

                a_t = a_i + ((a_f - a_i) / (n_frames - 1)) * i

                for j in range(self.robot.n_dof):
                    joint_states = np.append(joint_states, JointState(a_t[j]))
                waypoints = np.vstack([waypoints, joint_states])
        return waypoints

    def generate_trajectory(self, path, output_name, framerate=15, n_frames=75, traj_type='linear'):
        print("Animating the Robotic waiter...")
        for i in range(len(path) - 1):
            waypoints = self.generate_config_waypoints(path[i], path[i + 1], n_frames, traj_type)
            for i in range(len(waypoints)):
                vals = []
                for joint_state in waypoints[i]:
                    vals.append(joint_state.value)
                self.robot.set_joint_values(vals)
                self.robot.forward_kinematics()
                self.gripper_positions.append(self.robot.gripper_position)
                self.frames.append(self.generate_cartesian_points())
                self.collision_in_frame.append(self.environment.query_robot_collision())
                self.goal_in_frame.append(self.environment.query_robot_at_goal())

        self.fig, axis = plt.subplots()
        self.ax = axis
        halfway = len(self.frames)
        for i in range(int(halfway)):
            if i % 2 == 0:
                self.animate(i)
                plt.pause(0.001)
            else:
                continue
        plt.show()
        print("\t...Done")

        if self.save_gif:
            halfway = len(self.frames)
            print("Producing animation .gif...")
            ani = FuncAnimation(self.fig, self.animate, frames=len(self.frames), interval=1000 / framerate,
                                repeat=False)

            ani.save("Arm_Animation.gif", dpi=100, writer=PillowWriter(fps=framerate))
            print("\t...Done")
        return

    def plot_state(self, states):

        self.robot.set_joint_values(states)
        self.robot.forward_kinematics()
        self.gripper_positions.append(self.robot.gripper_position)
        self.frames.append(self.generate_cartesian_points())
        self.collision_in_frame.append(self.environment.query_robot_collision())
        self.goal_in_frame.append(self.environment.query_robot_at_goal())
        _, axis = plt.subplots()
        self.ax = axis
        self.animate(0)
        plt.show()
        return

    def animate(self, i):
        halfway = len(self.frames) / 2
        serve = True
        if i >= int(halfway):
            serve = False
        self.ax.clear()
        self.ax.set_xlim(-25, 25)
        self.ax.set_ylim(-13, 22)
        self.plot_background()
        self.plot_obstacles()
        # self.plot_targets()
        self.plot_rover()
        if self.collision_in_frame[i] is True:
            self.ax.plot(*self.frames[i], marker='o', color='r', lw=4)
        elif self.goal_in_frame[i] is True:
            self.ax.plot(*self.frames[i], marker='o', color='lime', lw=4)
        else:
            self.ax.plot(*self.frames[i], marker='o', color='#cccccc', lw=4)

        self.ax.set_title("RoboWaiter Arm Motion")
        self.ax.set_xlabel("x [dm]")
        self.ax.set_ylabel("y [dm]")
        if serve:
            Xorg = self.gripper_positions[i][0]
            Yorg = self.gripper_positions[i][1]
            x = [Xorg - 2, Xorg + 2, Xorg + 4, Xorg - 4]
            y = [Yorg, Yorg, Yorg + 1.5, Yorg + 1.5]
            self.ax.add_patch(patches.Polygon(xy=list(zip(x, y)), fill=True, color='white'))
        else:
            Xorg = self.targets[0][0]
            Yorg = self.targets[0][1]
            x = [Xorg - 2, Xorg + 2, Xorg + 4, Xorg - 4]
            y = [Yorg, Yorg, Yorg + 1.5, Yorg + 1.5]
            self.ax.add_patch(patches.Polygon(xy=list(zip(x, y)), fill=True, color='white'))
        self.ax.plot(self.gripper_positions[i][0], self.gripper_positions[i][1], marker='o', color='purple',
                     markersize=10)
        return