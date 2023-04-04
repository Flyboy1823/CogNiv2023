import warnings, time
warnings.simplefilter("ignore", UserWarning)
from unified_planning.shortcuts import *
from unified_planning.model.metrics import *
from unified_planning.io import PDDLWriter
from Nav_Modules.Nav_Geometry import *

def add_obstacles(obs):
    # This function updates the obstacles with new positions
    r_person = 3
    #while True:
    for i in range(3):
       obs.append(Obstacle(r_person, [random.randint(10, 90), random.randint(10, 90)]))
       time.sleep(1)  # Wait for 10ms before updating obstacles again
    for i in range(3):
        obs.pop()
    time.sleep(1)  # Wait for 10ms before updating obstacles again

class TaskPlanner:
    def __init__(self, new_graph, obs, save_pddl=False, output_path=None):
        self.new_graph = new_graph
        self.save_pddl = save_pddl
        self.output_path = output_path
        self.goals = []
        self.starts = []
        self.obs = obs or []
        for node in new_graph:
            if isinstance(node, Start_Node):
                self.starts.append(node)
            elif isinstance(node, Goal_Node):
                self.goals.append(node)
        if self.obs:
            add_obstacles(self.obs)

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
        goals = [Object(f'G{i}', Goal) for i in range(1, len(self.goals) + 1)]
        count = 0
        for goal in self.goals:
            if count < 10:
                count = count + 1
            else:
                count = count + 1
                goal.name = f"{goal.name}_1"

        rob1_loc = self.starts[0]
        rob2_loc = self.starts[1]
        rob1_x, rob1_y = rob1_loc.get_center()
        rob2_x, rob2_y = rob2_loc.get_center()

        # Calculate the distance of each goal object to each robot
        goal_distances = []
        for goal in self.goals:
            x, y = goal.get_center()
            distance_to_robot1 = ((x - rob1_x) ** 2 + (y - rob1_y) ** 2) ** 0.5
            distance_to_robot2 = ((x - rob2_x) ** 2 + (y - rob2_y) ** 2) ** 0.5
            goal_distances.append((goal, distance_to_robot1, distance_to_robot2))

        # Sort the goal objects by the distance to the closest robot
        goal_distances.sort(key=lambda x: min(x[1], x[2]))

        # Split the goal objects into two lists based on the closest robot
        robot1_goals = [Object(goal[0].name, Goal) for goal in goal_distances if goal[1] <= goal[2]]
        robot2_goals = [Object(goal[0].name, Goal) for goal in goal_distances if goal[1] > goal[2]]

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

        for start_node, start_obj in zip(self.starts, starts):
            moves = []
            mac = {}
            '''
            # move robot 1 to first goal
            move_to_first_goal_name = f"move_robot1_to_first_goal_{start_node.name}"
            move_to_first_goal = InstantaneousAction(move_to_first_goal_name, r=rob_waiter1.type, l_from=start_obj.type, l_to=robot1_goals[0].type)
            r = move_to_first_goal.parameter("r")
            l_from = move_to_first_goal.parameter("l_from")
            l_to = move_to_first_goal.parameter("l_to")
            move_to_first_goal.add_precondition(Rob_waiter_At(r, l_from))
            move_to_first_goal.add_effect(Rob_waiter_At(r, l_from), False)
            move_to_first_goal.add_effect(Rob_waiter_At(r, l_to), True)
            move_to_first_goal.add_effect(Visited(r, l_to), True)
            moves.append(move_to_first_goal)
            mac[move_to_first_goal] = costs[start_obj.name][robot1_goals[0].name]

            # move robot 2 to first goal
            move_to_first_goal_name = f"move_robot2_to_first_goal_{start_node.name}"
            move_to_first_goal = InstantaneousAction(move_to_first_goal_name, r=rob_waiter2.type, l_from=start_obj.type, l_to=robot2_goals[0].type)
            r = move_to_first_goal.parameter("r")
            l_from = move_to_first_goal.parameter("l_from")
            l_to = move_to_first_goal.parameter("l_to")
            move_to_first_goal.add_precondition(Rob_waiter_At(r, l_from))
            move_to_first_goal.add_effect(Rob_waiter_At(r, l_from), False)
            move_to_first_goal.add_effect(Rob_waiter_At(r, l_to), True)
            move_to_first_goal.add_effect(Visited(r, l_to), True)
            moves.append(move_to_first_goal)
            mac[move_to_first_goal] = costs[start_obj.name][robot2_goals[0].name]
            '''
            # move robots to other goals
            for i in range(1, len(robot1_goals)-1):
                move_name = f"move_robot1_from_{start_node.name}_to_{robot1_goals[i-1]}"
                move = InstantaneousAction(move_name, r=rob_waiter1.type, l_from=start_obj.type, l_to=robot1_goals[i-1].type)
                r = move.parameter("r")
                l_from = move.parameter("l_from")
                l_to = move.parameter("l_to")
                move.add_precondition(Rob_waiter_At(r, l_from))
                move.add_effect(Rob_waiter_At(r, l_from), False)
                move.add_effect(Rob_waiter_At(r, l_to), True)
                move.add_effect(Visited(r, l_to), True)
                moves.append(move)
                mac[move] = costs[start_obj.name][robot1_goals[i-1].name]

                # move to start robot 1
                move_to_start_name = f"move_robot1_to_start_from_{robot1_goals[i-1]}"
                move_to_start = InstantaneousAction(move_to_start_name, r=rob_waiter1.type, l_from=robot1_goals[i-1].type, l_to=start_obj.type)
                r = move_to_start.parameter("r")
                l_from = move_to_start.parameter("l_from")
                l_to = move_to_start.parameter("l_to")
                move_to_start.add_precondition(Rob_waiter_At(r, l_from))
                move_to_start.add_effect(Rob_waiter_At(r, l_from), False)
                move_to_start.add_effect(Rob_waiter_At(r, l_to), True)
                move_to_start.add_effect(Visited(r, l_to), True)
                moves.append(move_to_start)
                mac[move_to_start] = costs[robot1_goals[i-1].name][start_obj.name]

            for i in range(1, len(robot2_goals)-3):
                move_name = f"move_robot2_from_{start_node.name}_to_{robot2_goals[i-1]}"
                move = InstantaneousAction(move_name, r=rob_waiter2.type, l_from=start_obj.type, l_to=robot2_goals[i-1].type)
                r = move.parameter("r")
                l_from = move.parameter("l_from")
                l_to = move.parameter("l_to")
                move.add_precondition(Rob_waiter_At(r, l_from))
                move.add_effect(Rob_waiter_At(r, l_from), False)
                move.add_effect(Rob_waiter_At(r, l_to), True)
                move.add_effect(Visited(r, l_to), True)
                moves.append(move)
                mac[move] = costs[start_obj.name][robot2_goals[i-1].name]
                
                # move to start robot 2
                move_to_start_name = f"move_robot2_to_start_from_{robot2_goals[i-1]}"
                move_to_start = InstantaneousAction(move_to_start_name, r=rob_waiter2.type, l_from=robot2_goals[i-1].type, l_to=start_obj.type)
                r = move_to_start.parameter("r")
                l_from = move_to_start.parameter("l_from")
                l_to = move_to_start.parameter("l_to")
                move_to_start.add_precondition(Rob_waiter_At(r, l_from))
                move_to_start.add_effect(Rob_waiter_At(r, l_from), False)
                move_to_start.add_effect(Rob_waiter_At(r, l_to), True)
                move_to_start.add_effect(Visited(r, l_to), True)
                moves.append(move_to_start)
                mac[move_to_start] = costs[robot2_goals[i-1].name][start_obj.name]

            ## Populating the problem with fluents and actions
            problem = Problem("restaurant")
            problem.add_fluent(Rob_waiter_At, default_initial_value=False)
            problem.add_fluent(Visited, default_initial_value=False)
            problem.add_actions(moves)

            ## Adding objects
            problem.add_object(rob_waiter1)
            problem.add_object(rob_waiter2)
            problem.add_object(start_obj)
            problem.add_objects(goals)

            ## Setting the initial state
            problem.set_initial_value(Rob_waiter_At(rob_waiter1,start_obj), True)
            problem.set_initial_value(Visited(rob_waiter1,start_obj), True)
            problem.set_initial_value(Rob_waiter_At(rob_waiter2,start_obj), True)
            problem.set_initial_value(Visited(rob_waiter2,start_obj), True)
        
        ## Setting the goal state and metric
        for g in robot1_goals:
            problem.add_goal(Visited(rob_waiter1,g))
        problem.add_goal(Rob_waiter_At(rob_waiter1,start_obj))
        for g in robot2_goals:
            problem.add_goal(Visited(rob_waiter2,g))
        problem.add_goal(Rob_waiter_At(rob_waiter2,start_obj))
        problem.add_quality_metric(MinimizeActionCosts(mac))

        ## Save the .pddl files if you want
        if self.save_pddl:
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
            if plan is not None and isinstance(plan, list):
                print("Optimal Fast Downward Planner returned the following plan:")
                for action in plan:
                    robot = action.args[0].name # get robot name from action argument
                    print('\tRobot {} will perform action {}'.format(robot, action.name))
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