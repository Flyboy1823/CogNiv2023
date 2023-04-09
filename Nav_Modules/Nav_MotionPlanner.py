import random, sys, heapq
from scipy.spatial import KDTree
from Nav_Modules.Nav_Geometry import *

def Create_Goal_Nodes(goals):
    return [g.Create_Goal_Node(name='G'+str(idx+1)) for idx, g in enumerate(goals)]

def Create_Samples(map, start_node1, start_node2, obstacles, goals1, goals2, N_samples=100, N_knn=3):
    samples = []
    goals = [goals1, goals2]
    while len(samples) <= N_samples:
        tx = random.random() * map.width
        ty = random.random() * map.height

        obs_and_goals = obstacles + list(goals[0]) + list(goals[1])
        obs_and_goals_kd_tree = KDTree([o.center for o in obs_and_goals])

        dist, idx = obs_and_goals_kd_tree.query([tx, ty], k=N_knn)        
        no_col = True
        for i in range(N_knn):
            if dist[i] <= obs_and_goals[idx[i]].radius:
                no_col = False
                break
        if no_col:
            samples.append(Node([tx,ty]))

    samples.append(start_node1)
    samples.append(start_node2)
    goal_nodes1 = Create_Goal_Nodes(goals[0])
    goal_nodes2 = Create_Goal_Nodes(goals[1])
    return samples + goal_nodes1 + goal_nodes2, goal_nodes1, goal_nodes2

def Create_Roadmap(samples, obstacles, goals1, goals2, N_knn=3):
    """
    Creates the roadmap
    :param samples: list of XY positions of sampled points [m]
    :param obstacles: list of obstacles in the C-Space
    :param obstacle_kd_tree: KDTree object of obstacles
    :returns: the roadmap
    """
    samples_kd_tree = KDTree([s.center for s in samples])
    N_sample = len(samples)
    roadmap = []
    moving_obs = []
    r = 2
    PRM_graph = Graph(samples)

    # random.seed(4)
    moving_obs.append(Obstacle(r, [50, 80]))
    moving_obs.append(Obstacle(r, [20, 50]))
    moving_obs.append(Obstacle(r, [50, 20]))
    count = 0
    for i in range(50):
        for t in range(count, count + 3):
            # print("moving_obs len:", len(moving_obs))
            # print("t:", t)
            if t < len(moving_obs):
                x, y = moving_obs[t].get_center()
                no_col = True
                x_temp = x + random.randint(-5, 5)
                if x_temp > 100 : x_temp = 100
                if x_temp < 0 : x_temp = 0
                y_temp = y + random.randint(-5, 5)
                if y_temp > 100 : y_temp = 100
                if y_temp < 0 : y_temp = 0
                obs_check = Obstacle(r, [x_temp, y_temp])
                for o in obstacles:
                    no_col = no_col and Check_Circles_No_Collision(o, obs_check)
                    if no_col:
                        for g in goals1:
                            no_col = no_col and Check_Circles_No_Collision(g, obs_check)
                        for g in goals2:
                            no_col = no_col and Check_Circles_No_Collision(g, obs_check)
                        if no_col:
                            moving_obs.append(obs_check)
                    else:
                        while(no_col != True):
                            no_col = True
                            temp = Obstacle(r, [x + random.randint(-2, 2), y + random.randint(-2, 2)])
                            no_col = no_col and Check_Circles_No_Collision(o, temp)
                            for g in goals1:
                                no_col = no_col and Check_Circles_No_Collision(g, temp)
                            for g in goals2:
                                no_col = no_col and Check_Circles_No_Collision(g, temp)
                        moving_obs.append(temp)
                    break
        count = count + 3 

    for s in samples:
        edges = []; PRM_graph.graph[s] = {}
        dists, idx = samples_kd_tree.query(s.center, k=N_sample)       
        for i in range(1,N_sample):
            neighbor = samples[idx[i]]
            no_col = True
            edge = Edge(s, neighbor, cost=dists[i])
            for e in roadmap:
                if edge == e:
                    continue
            for o in obstacles:
                if edge.Check_Edge_Obstacle_Collision(o):
                    no_col = False
            for o in moving_obs:
                if edge.Check_Edge_Obstacle_Collision(o):
                    no_col = False
            if no_col:
                PRM_graph.graph[s][neighbor] = edge.cost
                roadmap.append(edge)
                edges.append(edge)
            if len(edges) >= N_knn:
                break
        # if len(PRM_graph.graph[s]) < 1:
            # print('Not all goal/start nodes could be connected to the roadmap.')
    return roadmap, PRM_graph, moving_obs



def Astar_Algorithm(PRM_graph, start_node, goal_node, edges):
    """
    Runs A* algorithm on the roadmap
    """
    unvisited_nodes = [(0, start_node)]
    shortest_path = {}
    previous_nodes = {}
    # We'll use max_value to initialize the "infinity" value of the unvisited nodes   
    max_value = sys.maxsize
    for node in PRM_graph.nodes:
        shortest_path[node] = max_value
    # However, we initialize the starting node's value with 0   
    shortest_path[start_node] = 0
    while unvisited_nodes:
        # Get the node with the lowest f score
        current_min_node = heapq.heappop(unvisited_nodes)[1]
        if current_min_node == goal_node:
            break

        # The code block below retrieves the current node's neighbors and updates their distances
        neighbors = PRM_graph.Get_Outgoing_Edges(current_min_node)
        for neighbor in neighbors:
            tentative_value = shortest_path[current_min_node] + PRM_graph.Get_Edge_Value(current_min_node, neighbor)
            if tentative_value < shortest_path[neighbor]:
                shortest_path[neighbor] = tentative_value
                previous_nodes[neighbor] = current_min_node
                f_score = tentative_value + PRM_graph.Heuristic(neighbor, goal_node)
                heapq.heappush(unvisited_nodes, (f_score, neighbor))

    path = []
    node = goal_node

    if node not in previous_nodes:
        print("\tPath could not be obtained...")
        return None, None, None

    while node != start_node:
        path.append(node)
        node = previous_nodes[node]

    # Add the start node manually
    path.append(start_node)

    path_value = shortest_path[goal_node]
    best_path = list(reversed(path))

    best_edges = []
    for i in range(len(path) - 1):
        current_edge = PRM_graph.Get_Edge_Value(best_path[i], best_path[i + 1])
        best_edges.append(current_edge)

    # print(f"\tA* search completed, shortest path has value {path_value:.3f}")
    return path_value, best_path, best_edges

def PRM_Solve(start_nodes, goals1, goals2, samples, goal1_nodes, goal2_nodes, roadmap, PRM_graph):
    """
    Runs PRM planning algorithm
    """
    N_goals = len(goals1) + len(goals2)
    N_traj_expected = int(N_goals*(N_goals+1)/2)
    print("Running Astar's algorithm, with " + str(len(samples)) + " nodes left to explore and " + str(N_traj_expected) + " trajectories expected...")

    new_graph = {}
    for start_n in start_nodes:
        new_graph[start_n] = {}
        for g in goal1_nodes:
            new_graph[g] = {}
        for g in goal2_nodes:
            new_graph[g] = {}
        trajectories1 = []
        trajectories2 = []
        for g in goal1_nodes:
            path_value, best_path, best_edges = Astar_Algorithm(PRM_graph, start_n, g, roadmap)
            if path_value==None or best_path==None or best_edges==None:
                pass
            else:
                new_graph[start_n][g] = {'Value' : path_value, 'Trajectory' : []}
                for i in range(len(best_edges)):
                    new_graph[start_n][g]['Trajectory'].append(best_path[i])
                    new_graph[start_n][g]['Trajectory'].append(best_edges[i])
                new_graph[start_n][g]['Trajectory'].append(best_path[-1])
                trajectories1.append(new_graph[start_n][g]['Trajectory'])
            for g2 in goal1_nodes:
                if not g == g2 and not g in new_graph[g2]:
                    path_value, best_path, best_edges = Astar_Algorithm(PRM_graph, g, g2, roadmap)
                    if path_value==None or best_path==None or best_edges==None:
                        pass
                    else:
                        new_graph[g][g2] = {'Value' : path_value, 'Trajectory' : []}
                        for j in range(len(best_edges)):
                            new_graph[g][g2]['Trajectory'].append(best_path[j])
                            new_graph[g][g2]['Trajectory'].append(best_edges[j])
                        new_graph[g][g2]['Trajectory'].append(best_path[-1])
                        trajectories1.append(new_graph[g][g2]['Trajectory'])
            for g2 in goal2_nodes:
                if not g == g2 and not g in new_graph[g2]:
                    path_value, best_path, best_edges = Astar_Algorithm(PRM_graph, g, g2, roadmap)
                    if path_value==None or best_path==None or best_edges==None:
                        pass
                    else:
                        new_graph[g][g2] = {'Value' : path_value, 'Trajectory' : []}
                        for j in range(len(best_edges)):
                            new_graph[g][g2]['Trajectory'].append(best_path[j])
                            new_graph[g][g2]['Trajectory'].append(best_edges[j])
                        new_graph[g][g2]['Trajectory'].append(best_path[-1])
                        trajectories1.append(new_graph[g][g2]['Trajectory'])
        
        for g in goal2_nodes:
                path_value, best_path, best_edges = Astar_Algorithm(PRM_graph, start_n, g, roadmap)
                if path_value==None or best_path==None or best_edges==None:
                    pass
                else:
                    new_graph[start_n][g] = {'Value' : path_value, 'Trajectory' : []}
                    for i in range(len(best_edges)):
                        new_graph[start_n][g]['Trajectory'].append(best_path[i])
                        new_graph[start_n][g]['Trajectory'].append(best_edges[i])
                    new_graph[start_n][g]['Trajectory'].append(best_path[-1])
                    trajectories2.append(new_graph[start_n][g]['Trajectory'])
                for g2 in goal1_nodes:
                    if not g == g2 and not g in new_graph[g2]:
                        path_value, best_path, best_edges = Astar_Algorithm(PRM_graph, g, g2, roadmap)
                        if path_value==None or best_path==None or best_edges==None:
                            pass
                        else:
                            new_graph[g][g2] = {'Value' : path_value, 'Trajectory' : []}
                            for j in range(len(best_edges)):
                                new_graph[g][g2]['Trajectory'].append(best_path[j])
                                new_graph[g][g2]['Trajectory'].append(best_edges[j])
                            new_graph[g][g2]['Trajectory'].append(best_path[-1])
                            trajectories2.append(new_graph[g][g2]['Trajectory'])
                for g2 in goal2_nodes:
                    if not g == g2 and not g in new_graph[g2]:
                        path_value, best_path, best_edges = Astar_Algorithm(PRM_graph, g, g2, roadmap)
                        if path_value==None or best_path==None or best_edges==None:
                            pass
                        else:
                            new_graph[g][g2] = {'Value' : path_value, 'Trajectory' : []}
                            for j in range(len(best_edges)):
                                new_graph[g][g2]['Trajectory'].append(best_path[j])
                                new_graph[g][g2]['Trajectory'].append(best_edges[j])
                            new_graph[g][g2]['Trajectory'].append(best_path[-1])
                            trajectories2.append(new_graph[g][g2]['Trajectory'])
            
    
    print("Finished running Astar's Algorithm on the PRM, with " + str(len(trajectories1)) + str(len(trajectories2)) + " trajectories found.\n")
    return new_graph, trajectories1, trajectories2