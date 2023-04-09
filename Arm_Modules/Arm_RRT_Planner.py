import numpy as np
from Arm_Modules.Arm_DataStructures import *
from Arm_Modules.Arm_Robot import JointState

class RRTPlanner:
    def __init__(self, environment, step, max_iterations=10000):
        self.environment = environment
        self.step = step
        self.max_iterations = max_iterations
        self.tree = Tree()
        self.goal_nodes = []
        self.gripper_values = []
        self.random_samples = []
        self.nearest_nodes = []

    def sample_random_state(self):
        node_in_tree = True
        while node_in_tree:
            rand_angles = np.random.uniform(self.environment.robot.min_joint_limit, self.environment.robot.max_joint_limit, self.environment.robot.n_dof) # check if uniform works, try also np.random.rand
            joint_values_node = TreeNode([JointState(rand_angles[i]) for i in range(len(rand_angles))])
            node_in_tree = self.tree.query_node_in_graph(joint_values_node)
        return joint_values_node

    def query_state_cause_collision(self, angles):
        self.environment.robot.set_joint_values(angles)
        self.environment.robot.forward_kinematics()
        return self.environment.query_robot_collision()

    def query_state_reach_target(self, angles):
        self.environment.robot.set_joint_values(angles)
        self.environment.robot.forward_kinematics()
        return self.environment.query_robot_at_goal()

    def forward_kinematics_for_extend(self, angles):
        self.environment.robot.set_joint_values(angles)
        self.environment.robot.forward_kinematics()

    def calculate_cost_rewire(self, new_node, near_node):
        new_node_vector = new_node.vectorized_values()
        near_node_vector = near_node.vectorized_values()
        self.forward_kinematics_for_extend(new_node_vector)
        node_1 = self.environment.robot.gripper_position
        self.forward_kinematics_for_extend(near_node_vector)
        node_2 = self.environment.robot.gripper_position
        cost = np.linalg.norm(node_1 - node_2)
        return cost

    def extend(self, rand_node, near_node):
        rand_node_vector = rand_node.vectorized_values()
        near_node_vector = near_node.vectorized_values()
        new_node_vector = self.step*(rand_node_vector - near_node_vector)/np.linalg.norm(rand_node_vector - near_node_vector) + near_node_vector
        self.forward_kinematics_for_extend(new_node_vector)
        node_1= self.environment.robot.gripper_position
        self.forward_kinematics_for_extend(near_node_vector)
        node_2 = self.environment.robot.gripper_position
        cost = np.linalg.norm(node_1 - node_2)
        '''if near_node.num_of_predecessor>=10:
            print("Test")'''
        new_node = TreeNode([JointState(new_node_vector[i]) for i in range(len(new_node_vector))])
        if not self.tree.query_node_in_graph(new_node):
            return new_node,cost
        else:
            print("ERROR: Node already exists within tree...")
            return

    def update_successor_cost(self, near_node):
        if np.any(near_node.successors):
            # self.update_successor_cost(near_node)
            # print("update successor cost")
            for successor in near_node.successors:
                # print("successor total cost:", successor.cost)
                successor.cost = 0
                # print("successor.last update cost:", successor.last_update_cost)
                successor.cal_cost(successor.last_update_cost)
                # print("successor new cost:", successor.cost)
                self.update_successor_cost(successor)
        return



    def rrt(self, initial_joint_values):
        self.tree.add_node(TreeNode(initial_joint_values))
        print("Running the RRT Algorithm...")
        for i in range(self.max_iterations):
            # if i%10==0:
                # print("Num of interation:", i)
            if len(self.goal_nodes) > 0:
                print(f"Finished running the RRT Algorithm, after {i} iterations.\n")
                return self.tree
            random_node = self.sample_random_state()
            nearest_node_ind = self.tree.nearest_neighbour(random_node)
            new_node,cost = self.extend(random_node, self.tree.nodes[nearest_node_ind])

            if not self.query_state_cause_collision(new_node.vectorized_values()) and not self.tree.query_node_in_graph(new_node):
                self.nearest_nodes.append(self.tree.nodes[nearest_node_ind])
                self.random_samples.append(random_node)

                self.gripper_values.append(self.environment.robot.gripper_position)
                if self.query_state_reach_target(new_node.vectorized_values()):
                    new_node.set_goal_true()
                    self.goal_nodes.append(new_node)
                new_node.set_predecessor(self.tree.nodes[nearest_node_ind])
                self.tree.nodes[nearest_node_ind].add_successor(new_node)
                new_node.cal_cost(cost)
                self.tree.add_node(new_node)

            if i == (self.max_iterations - 2):
                a=self.max_iterations - 1
                print("max_iterations")
        print(f"Finished running the RRT Algorithm, after {i} iterations.\n")
        if i == self.max_iterations-1:
            print("The RoboWaiter couldnt find a path :(")
            return None
        return self.tree

    def rrt_star(self, initial_joint_values):
        self.goal_is_found = False
        print("Running the RRT* Algorithm...")
        self.tree.add_node(TreeNode(initial_joint_values))
        for i in range(self.max_iterations):
            # if i % 100 == 0:
            #     print("Num of interation:", i)
            if self.goal_is_found:
                return self.tree
            random_node = self.sample_random_state()
            self.random_samples.append(random_node)
            nearest_node_ind = self.tree.nearest_neighbour(random_node)

            new_node,cost = self.extend(random_node, self.tree.nodes[nearest_node_ind])

            if not self.query_state_cause_collision(new_node.vectorized_values()) and not self.tree.query_node_in_graph(new_node):
                self.nearest_nodes.append(self.tree.nodes[nearest_node_ind])
                self.gripper_values.append(self.environment.robot.gripper_position)
                new_node_cost=self.tree.nodes[nearest_node_ind].cost+cost

                #The following lines add the rewiring capabilities of rrt*:
                near_inds= self.tree.neareast_neighbours(new_node, i)
                #removing the near node that was found initially from the near nodes index list(near_inds)
                update_new_node = False
                # sending the new node that was found initially, the updated near nodes indext list, and the cost between the new node and the nearset node
                if len(near_inds)>1:
                    update_new_node, update_near_node, update_cost, min_near_node_ind = self.choose_parent( new_node, near_inds,new_node_cost,nearest_node_ind)
                if update_new_node:
                    if len(near_inds)>0:
                        near_inds.remove(min_near_node_ind)
                    update_new_node.set_predecessor(update_near_node)
                    update_near_node.add_successor(update_new_node)
                    update_new_node.cal_cost(update_cost)
                    self.gripper_values.append(self.environment.robot.gripper_position)
                    self.rewire(update_new_node, near_inds)

                    if self.query_state_reach_target(update_new_node.vectorized_values()):
                        # print("\n\nGoal node was found\n\n")
                        self.goal_nodes.append(update_new_node)
                        update_new_node.set_goal_true()
                        self.goal_is_found = True
                    else:
                        self.goal_is_found = False
                    self.tree.add_node(update_new_node)
                else:
                    if len(near_inds)>1:
                        near_inds.remove(min_near_node_ind)
                    new_node.set_predecessor(self.tree.nodes[nearest_node_ind])
                    self.tree.nodes[nearest_node_ind].add_successor(new_node)
                    new_node.cal_cost(cost)
                    self.gripper_values.append(self.environment.robot.gripper_position)
                    self.rewire(new_node, near_inds)
                    if self.query_state_reach_target(new_node.vectorized_values()):
                        # print("\n\nGoal node was found\n\n")
                        self.goal_nodes.append(new_node)
                        new_node.set_goal_true()
                        self.goal_is_found = True
                    else:
                        self.goal_is_found = False
                    self.tree.add_node(new_node)
            if i == (self.max_iterations - 2):
                a = self.max_iterations - 1
                print("max_iterations")
        print(f"Finished running the RRT* Algorithm, after {i} iterations.\n")
        if i == self.max_iterations-1:
            print("The RoboWaiter couldnt find a path :(")
            return None
        return self.tree

    def choose_parent(self, new_node, near_inds,new_node_cost,nearest_node_ind):
        if not near_inds:
            return None
        costs = []
        for node in near_inds:
            near_node = self.tree.nodes[node]
            t_node,cost = self.extend(new_node,near_node)
            if not self.query_state_cause_collision(t_node.vectorized_values()) and not self.tree.query_node_in_graph(t_node):
                t_node.set_predecessor(near_node)
                t_node.cal_cost(t_node.cost)
                costs.append(t_node.cost)
            else:
                costs.append(float("inf"))  # the cost of collision node

        min_cost = min(costs)

        if min_cost == float("inf") or new_node_cost<=min_cost:
            # print("There is no good path.(min_cost is inf)")
            return False, self.tree.nodes[nearest_node_ind],new_node_cost, 0
        # else:
            # print("Better path was found...")
        min_ind = near_inds[costs.index(min_cost)]
        new_node, cost = self.extend(new_node, self.tree.nodes[min_ind])

        return new_node, self.tree.nodes[min_ind] ,cost, min_ind



    def rewire(self, update_new_node, near_inds):
        for i in near_inds: #TODO remove the node the that already connected to the updated new node, form choose_parent funciton
            near_node = self.tree.nodes[i]
            edge_cost = self.calculate_cost_rewire(near_node, update_new_node)
            cost= update_new_node.cost+edge_cost
            improved_cost = near_node.cost > cost


            if improved_cost:
                # print("rewiring...")
                near_node.set_predecessor(update_new_node)
                update_new_node.add_successor(near_node)
                near_node.cal_cost(edge_cost)

                self.update_successor_cost(near_node)

    def passed_collision_test(self, node_i, node_j, steps =5):
        vectors = np.linspace(node_i.vectorized_values(), node_j.vectorized_values, steps)
        for vector in vectors:
            if self.query_state_cause_collision(vector):
                return False
        return True
        
    def generate_paths(self):
        paths = []
        path_ind = 0
        goal_node_cost = []
        min_goal_node = self.goal_nodes[0]
        for goal_node in self.goal_nodes:
            if goal_node.cost < min_goal_node.cost:
                min_goal_node = goal_node
                path_ind = self.goal_nodes.index(goal_node)

        path = []
        current_node = min_goal_node
        while current_node.predecessor is not None:
            path.append(current_node)
            current_node = current_node.predecessor
        path.append(current_node) # adds initial state
        path.reverse()
        paths.append(path)
        # print("number of paths that were found:", len(self.goal_nodes))
        # print("The path index of the minimal one:", path_ind)
        print("Total ecalidian distance(cost) for the minimal path:", min_goal_node.cost)
        print("Total predecessors:", min_goal_node.num_of_predecessor, "\n")
        return paths #, min_goal_node.cost, min_goal_node.num_of_predecessor