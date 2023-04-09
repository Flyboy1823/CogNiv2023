import warnings
warnings.simplefilter("ignore", UserWarning)
from unified_planning.shortcuts import *
from unified_planning.model.metrics import *
from unified_planning.io import PDDLWriter
from Nav_Modules.Nav_Geometry import *

class TaskPlanner:
    def __init__(self, new_graph, output_path=None):
        self.new_graph = new_graph
        # self.save_pddl = save_pddl
        self.output_path = output_path
        self.goals1 = []
        self.starts = []
        for node in new_graph:
            if isinstance(node, Start_Node):
                self.starts.append(node)
            elif isinstance(node, Goal_Node):
                self.goals1.append(node)
        
    def BuildProblem(self):
        ## Declaring types
        Rob_waiter = UserType("Rob_waiter")
        Location = UserType("Location")
        Start = UserType("Start", father=Location)
        Goal = UserType("Goal", father=Location)

        ## Creating predicates (fluents)
        Rob_waiter_At = Fluent("Rob_waiter_At", BoolType(), Rob_waiter=Rob_waiter, location=Location)
        Visited = Fluent("Visited", BoolType(), Rob_waiter=Rob_waiter, location=Location)

        ## Declaring objects
        rob_waiter1 = Object("rob_waiter1", Rob_waiter)
        rob_waiter2 = Object("rob_waiter2", Rob_waiter)
        starts = [Object(start.name, Start) for start in self.starts]
        goals1 = [Object(f'G{i}', Goal) for i in range(1, 10)]
        goals2 = [Object(f'G{i}', Goal) for i in range(1, 10)]

        ## Creating action costs
        costs = {}
        for n1 in self.new_graph:
            if not (n1.name in costs):
                costs[n1.name] = {}
            for n2 in self.new_graph:
                if not (n2.name in costs):
                    costs[n2.name] = {}
                if n2 in self.new_graph[n1]:
                    costs[n1.name][n2.name] = Int(int(self.new_graph[n1][n2]['Value']))
                    costs[n2.name][n1.name] = Int(int(self.new_graph[n1][n2]['Value']))
        for start in self.starts:
            if start.name not in costs:
                print("Start node is not connected to the other nodes, please try again...")
                return None
        
        # Creating actions
        moves = []
        mac = {}
        action_names = set()

        for start_node, start_obj in zip(self.starts, starts):
            moves = []
            mac = {}
            action_names = set()
            if start_node == self.starts[0]:
                # move to first goal Rob1
                move_to_first_goal_name = f"move_Rob1_to_first_goal_{start_node.name}"
                move_to_first_goal = InstantaneousAction(move_to_first_goal_name, r=rob_waiter1.type, l_from=start_obj.type, l_to=goals1[0].type)
                r = move_to_first_goal.parameter("r")
                l_from = move_to_first_goal.parameter("l_from")
                l_to = move_to_first_goal.parameter("l_to")
                move_to_first_goal.add_precondition(Rob_waiter_At(r, l_from))
                move_to_first_goal.add_effect(Rob_waiter_At(r, l_from), False)
                move_to_first_goal.add_effect(Rob_waiter_At(r, l_to), True)
                move_to_first_goal.add_effect(Visited(r, l_to), True)
                moves.append(move_to_first_goal)
                mac[move_to_first_goal] = costs[start_obj.name][goals1[0].name]
                action_names.add(move_to_first_goal_name)

                # move Rob1 to other goals1
                for i in range(1, len(goals1)):
                    move_name = f"move_Rob1_from_{start_node.name}_to_{goals1[i].name}"
                    move = InstantaneousAction(move_name, r=rob_waiter1.type, l_from=start_obj.type, l_to=goals1[i].type)
                    r = move.parameter("r")
                    l_from = move.parameter("l_from")
                    l_to = move.parameter("l_to")
                    move.add_precondition(Rob_waiter_At(r, l_from))
                    move.add_effect(Rob_waiter_At(r, l_from), False)
                    move.add_effect(Rob_waiter_At(r, l_to), True)
                    move.add_effect(Visited(r, l_to), True)
                    moves.append(move)
                    mac[move] = costs[start_obj.name][goals1[i-1].name]
                    action_names.add(move_name)

                    # move ROb1 to start
                    move_to_start_name = f"move_Rob1_to_start_from_{goals1[i-1].name}"
                    move_to_start = InstantaneousAction(move_to_start_name, r=rob_waiter1.type, l_from=goals1[i-1].type, l_to=start_obj.type)
                    r = move_to_start.parameter("r")
                    l_from = move_to_start.parameter("l_from")
                    l_to = move_to_start.parameter("l_to")
                    move_to_start.add_precondition(Rob_waiter_At(r, l_from))
                    move_to_start.add_effect(Rob_waiter_At(r, l_from), False)
                    move_to_start.add_effect(Rob_waiter_At(r, l_to), True)
                    move_to_start.add_effect(Visited(r, l_to), True)
                    moves.append(move_to_start)
                    mac[move_to_start] = costs[goals1[i-1].name][start_obj.name]
                    action_names.add(move_to_start_name)
            else:
                # move to first goal Rob2
                move_to_first_goal_name = f"move_Rob2_to_first_goal_{start_node.name}"
                move_to_first_goal = InstantaneousAction(move_to_first_goal_name, r=rob_waiter2.type, l_from=start_obj.type, l_to=goals2[0].type)
                r = move_to_first_goal.parameter("r")
                l_from = move_to_first_goal.parameter("l_from")
                l_to = move_to_first_goal.parameter("l_to")
                move_to_first_goal.add_precondition(Rob_waiter_At(r, l_from))
                move_to_first_goal.add_effect(Rob_waiter_At(r, l_from), False)
                move_to_first_goal.add_effect(Rob_waiter_At(r, l_to), True)
                move_to_first_goal.add_effect(Visited(r, l_to), True)
                moves.append(move_to_first_goal)
                mac[move_to_first_goal] = costs[start_obj.name][goals2[0].name]
                action_names.add(move_to_first_goal_name)

                # move Rob2 to other goals2
                for i in range(1, len(goals2)):
                    move_name = f"move_Rob2_from_{start_node.name}_to_{goals2[i].name}"
                    move = InstantaneousAction(move_name, r=rob_waiter2.type, l_from=start_obj.type, l_to=goals2[i].type)
                    r = move.parameter("r")
                    l_from = move.parameter("l_from")
                    l_to = move.parameter("l_to")
                    move.add_precondition(Rob_waiter_At(r, l_from))
                    move.add_effect(Rob_waiter_At(r, l_from), False)
                    move.add_effect(Rob_waiter_At(r, l_to), True)
                    move.add_effect(Visited(r, l_to), True)
                    moves.append(move)
                    mac[move] = costs[start_obj.name][goals2[i-1].name]
                    action_names.add(move_name)

                    # move Rob2 to start
                    move_to_start_name = f"move_Rob2_to_start_from_{goals2[i-1].name}"
                    move_to_start = InstantaneousAction(move_to_start_name, r=rob_waiter2.type, l_from=goals2[i-1].type, l_to=start_obj.type)
                    r = move_to_start.parameter("r")
                    l_from = move_to_start.parameter("l_from")
                    l_to = move_to_start.parameter("l_to")
                    move_to_start.add_precondition(Rob_waiter_At(r, l_from))
                    move_to_start.add_effect(Rob_waiter_At(r, l_from), False)
                    move_to_start.add_effect(Rob_waiter_At(r, l_to), True)
                    move_to_start.add_effect(Visited(r, l_to), True)
                    moves.append(move_to_start)
                    mac[move_to_start] = costs[goals2[i-1].name][start_obj.name]
                    action_names.add(move_to_start_name)

            ## Populating the problem with fluents and actions
            problem = Problem("restaurant")
            problem.add_fluent(Rob_waiter_At, default_initial_value=False)
            problem.add_fluent(Visited, default_initial_value=False)
            problem.add_actions(moves)

            ## Adding objects
            problem.add_object(rob_waiter1)
            problem.add_object(start_obj)
            problem.add_objects(goals1)

            ## Setting the initial state
            problem.set_initial_value(Rob_waiter_At(rob_waiter1,start_obj), True)
            problem.set_initial_value(Visited(rob_waiter1,start_obj), True)
        
        ## Setting the goal state and metric
        for g in goals1:
            problem.add_goal(Visited(rob_waiter1,g))
        problem.add_goal(Rob_waiter_At(rob_waiter1,start_obj))
        problem.add_quality_metric(MinimizeActionCosts(mac))

        ## Save the .pddl files if you wanty
        # if self.save_pddl:
        w = PDDLWriter(problem)
        w.write_domain(self.output_path+'domain_restaurant.pddl')
        w.write_problem(self.output_path+'problem_restaurant.pddl')
        return problem

    def SolveProblem(self):
        problem = self.BuildProblem()
        if problem == None:
            return None
        up.shortcuts.get_env().credits_stream = None # this just hides the credits of the planner
        with OneshotPlanner(name='fast-downward-opt') as planner:
            result = planner.solve(problem)
            plan = result.plan
            if plan is not None:
                for i in range(len(plan.actions)):
                    robot = plan.actions[i].actual_parameters[0]  # get robot name from action argument
                    action = plan.actions[i].actual_parameters[1:]
                    print('\tRobot {} will perform action {}'.format(robot, action))
            else:
                print("No plan was found")
        return plan

    def GetFinalTrajectory(self):
        plan = self.SolveProblem()
        if plan == None:
            return None
        new_graph = self.new_graph
        
        ## Getting the actions from the plan
        actions = []
        for i in range(len(plan.actions)):
            l1 = plan.actions[i].actual_parameters[1]
            l2 = plan.actions[i].actual_parameters[2]
            actions.append([l1, l2])

        ## Turning the list of actions into a trajectory
        final_trajectory = []; total_distance = 0
        for j in range(len(actions)):
            start_node = str(actions[j][0])
            end_node = str(actions[j][1])
            for n1 in new_graph:
                for n2 in new_graph:
                    if n2 in new_graph[n1]:
                        if n1.name == start_node and n2.name == end_node:
                            final_trajectory.append(new_graph[n1][n2]['Trajectory'])
                            total_distance += new_graph[n1][n2]['Value']         
                        elif n2.name == start_node and n1.name == end_node:  
                            final_trajectory.append(new_graph[n1][n2]['Trajectory'][::-1])
                            total_distance += new_graph[n1][n2]['Value'] 
        print(f"Total cost of the proposed path: {total_distance:0.3f}\n")
        return final_trajectory