import os, imageio, shutil
import matplotlib.pyplot as plt
import networkx as nx
from Nav_Modules.Nav_Geometry import *
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
import matplotlib.lines as mlines

class Plotter:
    def __init__(self, map, obstacles, moving_obs, goals, start, samples, roadmap, PRM_graph, new_graph, trajectories1, trajectories2, final_traj, output_path=None, axis=None, save_img=False, save_gif=False):
        self.map = map
        self.obstacles = obstacles
        self.moving_obs = moving_obs
        self.goals = list(goals[0]) + list(goals[1])
        self.start = start
        self.samples = samples
        self.roadmap = roadmap
        self.PRM_graph = PRM_graph
        self.new_graph =  new_graph
        self.trajectories1 = trajectories1
        self.trajectories2 = trajectories2
        self.final_traj = final_traj
        self.ax = axis
        self.save_img = save_img
        self.save_gif = save_gif
        self.output_path = output_path
    
    def plot_init(self, axis=None):
        """
        Plots the initial map, including obstacles, goals, and the starting point
        :param axis: axis of the subplot to add the plot to
        """
        if axis is None:
            _, axis = plt.subplots()
        axis.set_title('C-space')
        axis.set_ylabel('y [m]')
        axis.set_xlabel('x [m]')

        # plotting the map, obstacles, and goal zones
        self.map.plot_rectangle(axis)
        for o in self.obstacles:
            o.plot_circle(axis)
        for g in self.goals:
            g.plot_circle(axis)
        self.start[0].plot_node(axis)
        self.start[1].plot_node(axis)

        # add a legend
        obstacle_patch = mpatches.Patch(color='orange', label='Obstacle')
        goal_patch = mpatches.Patch(color='green', label='Goal')
        start_patch = mpatches.Patch(color='blue', label='Starting point')
        plt.legend(handles=[obstacle_patch, goal_patch, start_patch])

        plt.show()
        if self.save_img:
            plt.savefig(self.output_path+'C_space.png')
        return

    def plot_knn(self):
        """
        Plots the mapand shows the k-Nearest Neighbors of each node after the initial roadmap sampling
        """
        _, axis = plt.subplots()
        axis.set_title('kNN Neighbors')
        axis.set_ylabel('y [m]')
        axis.set_xlabel('x [m]')
        
        # plotting the map, obstacles, and goal zones
        self.map.plot_rectangle(axis)
        for o in self.obstacles:
            o.plot_circle(axis)
        for o in self.moving_obs:
            o.plot_circle(axis)
        for g in self.goals:
            g.plot_circle(axis)

        # plotting all the nodes (including the start and goal nodes)
        for s in self.samples:
            s.plot_node(axis)

        if self.save_gif:
            print("Producing kNN animation .gif...")
            img_list = ['Output/Temp_Images/1.png']
            plt.savefig('Output/Temp_Images/1.png')

        # plotting neighbors
        i = 0
        for idx_k, key in enumerate(self.PRM_graph.graph):
            key.plot_node(axis, color='yellow')
            xlabel_string = 'kNN Check: ' + str(idx_k+1) + ' / ' + str(len(self.PRM_graph.graph))
            axis.set_xlabel(xlabel_string)
            i += 1
            if self.save_gif:
                img_list.append('Output/Temp_Images/' + str(i) + '.png')
                plt.savefig('Output/Temp_Images/' + str(i) + '.png')

            plt.pause(0.001)
            for n in self.PRM_graph.graph[key]:
                n.plot_node(axis, color='orange')
            i += 1
            if self.save_gif:
                img_list.append('Output/Temp_Images/' + str(i) + '.png')
                plt.savefig('Output/Temp_Images/' + str(i) + '.png')

            plt.pause(0.001)


        # create legend patches
        yellow_patch = mpatches.Patch(color='yellow', label='kNN node')
        orange_patch = mpatches.Patch(color='orange', label='kNN neighbor')

        # add legend to the plot
        plt.legend(handles=[yellow_patch, orange_patch])

        plt.show()
        if self.save_gif:
            for j in range(0,10):
                img_list.append('Output/Temp_Images/' + str(i+j+1) + '.png')
                shutil.copyfile('Output/Temp_Images/' + str(i) + '.png', 'Output/Temp_Images/' + str(i+j+1) + '.png')
            Create_GIF(img_list, "kNN_Animation", self.output_path)
            print("\t...Done")
        return

    def plot_PRM(self, axis=None):
        """
        Plots the mapafter the PRM is generated
        :param axis: axis of the subplot to add the plot to
        """
        if axis is None:
            _, axis = plt.subplots()
        axis.set_title('PRM')
        axis.set_ylabel('y [m]')
        axis.set_xlabel('x [m]')

        # plotting the map, obstacles, and goal zones
        self.map.plot_rectangle(axis)
        for o in self.obstacles:
            o.plot_circle(axis)
        for o in self.moving_obs:
            o.plot_circle(axis)
        for g in self.goals:
            g.plot_circle(axis)

        # Add legend
        legend_elements = [Line2D([0], [0], marker='o', color='w', label='Sample Node', markerfacecolor='g', markersize=10),
                        Line2D([0], [0], marker='o', color='w', label='Goal Node', markerfacecolor='c', markersize=10),
                        Line2D([0], [0], marker='o', color='w', label='Obstacle', markerfacecolor='orange', markersize=10),
                        Line2D([0], [0], marker='o', color='w', label='Moving Obstacle', markerfacecolor='k', markersize=10)]
        axis.legend(handles=legend_elements, loc='upper right')


        if self.save_gif:
            print("Producing PRM animation .gif...")
            img_list = ['Output/Temp_Images/1.png']
            plt.savefig('Output/Temp_Images/1.png')

        # plotting all the nodes (including the start and goal nodes)
        for idx_s, s in enumerate(self.samples):
            s.plot_node(axis)
            xlabel_string = 'x [m]       Nodes: ' + str(idx_s+1) + ' / ' + str(len(self.samples))
            axis.set_xlabel(xlabel_string)
            if len(self.samples) < 100:
                if self.save_gif:
                    img_list.append('Output/Temp_Images/' + str(1+idx_s) + '.png')
                    plt.savefig('Output/Temp_Images/' + str(1+idx_s) + '.png')

                plt.pause(0.001)
            elif len(self.samples) >= 100:
                if idx_s % 10 == 0 or idx_s == len(self.samples) - 1:
                    if self.save_gif:
                        img_list.append('Output/Temp_Images/' + str(1+idx_s) + '.png')
                        plt.savefig('Output/Temp_Images/' + str(1+idx_s) + '.png')
                    plt.pause(0.001)

        if self.save_gif:
            for i in range(1,10):
                img_list.append('Output/Temp_Images/' + str(1+idx_s+i) + '.png')
                shutil.copyfile('Output/Temp_Images/' + str(1+idx_s) + '.png', 'Output/Temp_Images/' + str(1+idx_s+i) + '.png')

        # plotting all the edges
        for idx_e, e in enumerate(self.roadmap):
            e.plot_edge(axis)
            e.Highlight_Nodes(axis)
            xlabel_string_new = xlabel_string + ', Edges: ' + str(idx_e+1) + ' / ' + str(len(self.roadmap))
            axis.set_xlabel(xlabel_string_new)
            if len(self.roadmap) < 200:
                if self.save_gif:
                    img_list.append('Output/Temp_Images/' + str(10+idx_s+idx_e) + '.png')
                    plt.savefig('Output/Temp_Images/' + str(10+idx_s+idx_e) + '.png')

                plt.pause(0.001)
            elif len(self.roadmap) >= 200 and len(self.roadmap) < 1000:
                if idx_e == len(self.roadmap) - 1:
                    axis.set_xlabel(xlabel_string_new + ', Plotting Complete!')   
                if idx_e % 20 == 0 or idx_e == len(self.roadmap) - 1:
                    if self.save_gif:
                        img_list.append('Output/Temp_Images/' + str(10+idx_s+idx_e) + '.png')
                        plt.savefig('Output/Temp_Images/' + str(10+idx_s+idx_e) + '.png')
                    plt.pause(0.001)
            elif len(self.roadmap) >= 1000:
                if idx_e == len(self.roadmap) - 1:
                    axis.set_xlabel(xlabel_string_new )#+ ', Plotting Complete!')   
                if idx_e % 200 == 0 or idx_e == len(self.roadmap) - 1:
                    if self.save_gif:
                        img_list.append('Output/Temp_Images/' + str(10+idx_s+idx_e) + '.png')
                        plt.savefig('Output/Temp_Images/' + str(10+idx_s+idx_e) + '.png')

                    plt.pause(0.001)

        plt.show()
        if self.save_img:
            plt.savefig(self.output_path+'/PRM.png')
        if self.save_gif:
            for j in range(1,15):
                img_list.append('Output/Temp_Images/' + str(10+idx_s+idx_e+j) + '.png')
                shutil.copyfile('Output/Temp_Images/' + str(10+idx_s+idx_e) + '.png', 'Output/Temp_Images/' + str(10+idx_s+idx_e+j) + '.png')
            Create_GIF(img_list, "PRM_Animation", self.output_path)
            print("\t...Done")
        return

    def plot_Astar(self, axis=None, clean=False):
        """
        Plots the mapshowing the paths obtained by the A* algorithm, including those paths, the PRM (optinal), and the initial map contents
        :param axis: axis of the subplot to add the plot to
        :param clean: if True, then only the Dijkstra paths are shown, otherwise the whole PRM is shown as well
        """
        if axis is None:
            _, axis = plt.subplots()
        if clean:
            axis.set_title('A* Paths between Critical Nodes')
        else:
            axis.set_title('PRM with A* Paths')
        axis.set_ylabel('y [m]')
        axis.set_xlabel('x [m]')

        # plotting the map, obstacles, and goal zones
        self.map.plot_rectangle(axis)
        for o in self.obstacles:
            o.plot_circle(axis)
        for o in self.moving_obs:
            o.plot_circle(axis)
        for g in self.goals:
            g.plot_circle(axis)
        if not clean: # show the original PRM, else show only the shortest paths
            for s in self.samples:
                s.plot_node(axis)
            for e in self.roadmap:
                e.plot_edge(axis)

        if self.save_gif:
            print("Producing A* animation .gif...")
            img_list = ['Output/Temp_Images/1.png']
            plt.savefig('Output/Temp_Images/1.png')

        count = 0
        # plotting the solution trajectories (including the start and goal nodes)
        for idx_t, traj in enumerate(self.trajectories1):
            xlabel_string = 'x [m]       Trajectories: ' + str(idx_t+1) + ' / ' + str(len(self.trajectories1))
            if idx_t == len(self.trajectories1) - 1:
                axis.set_xlabel(xlabel_string)# + ', Plotting Complete!') 
            else:
                axis.set_xlabel(xlabel_string)
            for o in traj:
                if isinstance(o,Node):
                    if not isinstance(o,Start_Node) and not isinstance(o,Goal_Node):
                        o.plot_node(axis, color='yellow')
                    else:
                        o.plot_node(axis)
                elif isinstance(o,Edge):
                    o.plot_edge(axis, color='yellow')
                if self.save_gif:
                    count += 1
                    img_list.append('Output/Temp_Images/' + str(1+count) + '.png')
                    plt.savefig('Output/Temp_Images/' + str(1+count) + '.png')
                plt.pause(0.001)
        
        if self.save_gif:
            for j in range(1,15):
                img_list.append('Output/Temp_Images/' + str(1+count+j) + '.png')
                shutil.copyfile('Output/Temp_Images/' + str(1+count) + '.png', 'Output/Temp_Images/' + str(1+count+j) + '.png')

        if self.save_img:
            plt.savefig(self.output_path+'Astar.png')

        plt.show()
        if self.save_gif:
            Create_GIF(img_list, "Astar_Animation", self.output_path)
            print("\t...Done")
        
        count = 0
        # plotting the solution trajectories (including the start and goal nodes)
        for idx_t, traj in enumerate(self.trajectories2):
            xlabel_string = 'x [m]       Trajectories: ' + str(idx_t+1) + ' / ' + str(len(self.trajectories2))
            if idx_t == len(self.trajectories2) - 1:
                axis.set_xlabel(xlabel_string)# + ', Plotting Complete!') 
            else:
                axis.set_xlabel(xlabel_string)
            for o in traj:
                if isinstance(o,Node):
                    if not isinstance(o,Start_Node) and not isinstance(o,Goal_Node):
                        o.plot_node(axis, color='yellow')
                    else:
                        o.plot_node(axis)
                elif isinstance(o,Edge):
                    o.plot_edge(axis, color='yellow')
                if self.save_gif:
                    count += 1
                    img_list.append('Output/Temp_Images/' + str(1+count) + '.png')
                    plt.savefig('Output/Temp_Images/' + str(1+count) + '.png')

                plt.pause(0.001)
        
        # add legend to the plot
        yellow_line = mlines.Line2D([], [], color='yellow', label='Shortest path')
        blue_line = mlines.Line2D([], [], color='blue', label='Alternative path')
        axis.legend(handles=[yellow_line, blue_line])

        if self.save_gif:
            for j in range(1,15):
                img_list.append('Output/Temp_Images/' + str(1+count+j) + '.png')
                shutil.copyfile('Output/Temp_Images/' + str(1+count) + '.png', 'Output/Temp_Images/' + str(1+count+j) + '.png')

        if self.save_img:
            plt.savefig(self.output_path+'Astar.png')
        plt.show()
        if self.save_gif:
            Create_GIF(img_list, "Astar_Animation", self.output_path)
            print("\t...Done")

        return

    def Visualize_Final_Graph(self, axis=None):
        """
        Plots a simplified graph showing the connections between the critical nodes as well as their weights
        :param new_graph: dictionary which contains the distances between pairs of critical nodes as well as the best path on the PRM that connects them
        """
        new_graph = self.new_graph
        if axis is None:
            _, axis = plt.subplots()
    
        G = nx.Graph()
        for n in new_graph:
            G.add_node(n.name, pos=(n.center[0],n.center[1]), color=n.color)
            for n2 in new_graph[n]:
                cost = new_graph[n][n2]['Value']
                cost = f"{cost:.3f}"
                G.add_edge(n.name, n2.name, weight=cost, color='yellow')
        pos = nx.get_node_attributes(G,'pos')
        nx.draw(G,pos,ax=axis,with_labels=True)
        labels = nx.get_edge_attributes(G,'weight')
        nx.draw_networkx_edge_labels(G,pos,ax=axis,edge_labels=labels)

        axis.set_title('Simplified Graph')
        yellow_line = mlines.Line2D([], [], color='yellow', label='Path weight')
        plt.legend(handles=[yellow_line])

        plt.show()
        if self.save_img:
            plt.savefig(self.output_path+'Simplified_Graph.png')
        return

    def plot4(self):
        """
        Plots the 4 first important graphs: initial map, PRM map, Astar map, and simplified TSP graph
        """
        fig1, ax1 = plt.subplots()
        self.plot_init(axis=ax1)
        ax1.set_title('Initial Map')

        fig2, ax2 = plt.subplots()
        self.plot_PRM(axis=ax2)
        ax2.set_title('PRM Map')

        fig3, ax3 = plt.subplots()
        self.plot_Astar(axis=ax3, clean=True)
        ax3.set_title('Astar Map')

        fig4, ax4 = plt.subplots()
        self.Visualize_Final_Graph(axis=ax4)
        ax4.set_title('Simplified TSP Graph')

        plt.show()
        return

    def plot_final(self):
        """
        Plots the final trajectory, including the initial map contents
        """
        _, axis = plt.subplots()
        axis.set_title('Final C-Space Trajectory')
        axis.set_ylabel('y [m]')
        axis.set_xlabel('x [m]')

        # plotting the map, obstacles, and goal zones
        self.map.plot_rectangle(axis)
        for o in self.obstacles:
            o.plot_circle(axis)
        for g in self.goals:
            g.plot_circle(axis)

        if self.save_gif:
            print("Producing final trajectory animation .gif...")
            img_list = ['Output/Temp_Images/1.png']
            plt.savefig('Output/Temp_Images/1.png')

        traj_count = 0
        node_count = 0
        check = 1
        # plotting the solution trajectories (including the start and goal nodes)
        for idx_t, traj in enumerate(self.final_traj):
            xlabel_string = 'x [m]       Trajectories: ' + str(idx_t+1) + ' / ' + str(len(self.final_traj))
            axis.set_xlabel(xlabel_string)
            traj_count += 1
            
            # determine the color for the trajectory
            if self.start[0] in traj:
                color_fwd = 'red'
                color_bwd = 'pink'
            elif self.start[1] in traj:
                color_fwd = 'gray'
                color_bwd = 'purple'
                
            # plot the start nodes in the appropriate color
            self.start[0].plot_node(axis, color=color_fwd)
            self.start[1].plot_node(axis, color=color_fwd)
            t = 0
            for idx_o, o in enumerate(traj):
                node_count += 1
                if isinstance(o,Node):
                    if not isinstance(o,Start_Node) and not isinstance(o,Goal_Node):
                        # plot the node in the appropriate color based on direction
                        if idx_t == len(self.final_traj) - 1:
                            o.plot_node(axis, color=color_fwd)
                        else:
                            o.plot_node(axis, color=color_bwd)
                    else:
                        o.plot_node(axis)
                    
                    if check == 3:
                        check = 1
                        if t >= 2:
                            self.moving_obs[t-2].remove_circle(axis)
                            self.moving_obs[t-1].remove_circle(axis)
                            self.moving_obs[t].remove_circle(axis)
                        self.moving_obs[t].plot_circle(axis)
                        self.moving_obs[t+1].plot_circle(axis)
                        self.moving_obs[t+2].plot_circle(axis)
                        if (t+3) < len(self.moving_obs):
                            t = t+3
                    check = check + 1

                elif isinstance(o,Edge):
                    o.plot_edge(axis, color=((255-(30*idx_t))/255, (255-(30*idx_t))/255, 0))
                if (idx_t == len(self.final_traj) - 1) and (idx_o == len(traj) - 1):
                    axis.set_xlabel(xlabel_string)# + ', Plotting Complete!') 
                if self.save_gif:
                    img_list.append('Output/Temp_Images/' + str(node_count) + '.png')
                    plt.savefig('Output/Temp_Images/' + str(node_count) + '.png')
                plt.pause(0.1)
            plt.pause(0.01)

        if self.save_img:
            plt.savefig(self.output_path+'Final_Map.png')
        if self.save_gif:
            for j in range(1,15):
                img_list.append('Output/Temp_Images/' + str(node_count+j) + '.png')
                shutil.copyfile('Output/Temp_Images/' + str(node_count) + '.png', 'Output/Temp_Images/' + str(node_count+j) + '.png')
            Create_GIF(img_list, "Final_Trajectory_Animation", self.output_path)
            print("\t...Done\n")
        
        legend_elements = [Line2D([0], [0], marker='o', color='w', label='Start Node',
                          markerfacecolor='blue', markersize=10),
                   Line2D([0], [0], marker='o', color='w', label='Goal Node',
                          markerfacecolor='green', markersize=10),
                   Line2D([0], [0], marker='o', color='red', label='Forward Trajectory',
                          markerfacecolor='red', markersize=10),
                   Line2D([0], [0], marker='o', color='purple', label='Backward Trajectory',
                          markerfacecolor='purple', markersize=10)]
        axis.legend(handles=legend_elements, loc='upper right')


        plt.show()
        return
    
def Create_GIF(img_list, name, output_path):
    """
    Creates a .gif animation from a list of image files (.png)
    :param img_list: List of the paths of the images to concatenate
    :param name: Name for the .gif
    :param output_path: Path for the output folder
    """
    with imageio.get_writer(output_path + name + ".gif", mode='I') as writer:
        for filename in img_list:
            image = imageio.imread(filename)
            writer.append_data(image)
    # Remove the temporary image files
    for filename in set(img_list):
        os.remove(filename)
    return