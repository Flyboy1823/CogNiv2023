from Nav_Modules.Nav_Geometry import *
from Nav_Modules.Nav_MotionPlanner import *
from Nav_Modules.Nav_Plotter import *
from Nav_Modules.Nav_TaskPlanner import *
from Serve_plate import Serve_plate


def Create_Obstacles(N_obs, N_goals):
    """
    Creates the obstacles for our problem
    :param N_obs: the number of obstacles to instantiate
    :returns: list of the obstacle instances
    """
    # print the main tables
    # print side tables that aren't in obsitcles
    r_table = 6
    r_chair = 2
    r_person = 2.5
    obs = []
    for i in range(N_obs):
        for k in range(N_obs):
            obs.append(Obstacle(r_table, [22 * i + 19, 22 * k + 19]))
    for i in range(N_obs):
        for k in range(N_obs):
            if (i * N_obs + k) in N_goals:
                continue
            else:
                obs.append(Obstacle(r_chair, [22 * i + 24, 22 * k + 22]))
                obs.append(Obstacle(r_chair, [22 * i + 24, 22 * k + 16]))
                obs.append(Obstacle(r_chair, [22 * i + 14, 22 * k + 22]))
                obs.append(Obstacle(r_chair, [22 * i + 14, 22 * k + 16]))
    obs.append(Obstacle(r_person, [5, 50]))
    obs.append(Obstacle(r_person, [95, 50]))
    obs.append(Obstacle(r_person, [28, 28]))
    obs.append(Obstacle(r_person, [28, 74]))
    obs.append(Obstacle(r_person, [52, 50]))
    obs.append(Obstacle(r_person, [52, 72]))
    obs.append(Obstacle(r_person, [74, 52]))
    obs.append(Obstacle(r_person, [74, 28]))

    return obs

def Create_Goal_Zones(N_goals, obs, N_obs):
    """
    Creates the goal zones for our problem
    :param N_goals: the number of goal zones to instantiate
    :param obs: list of the obstacle instances in our problem
    :returns: list of the goal zone instances
    """
    goals = []
    r_chair = 1
    for i in N_goals:
        x = 22 * math.floor(i / N_obs) + 25
        y = 22 * (i % N_obs) + 25
        g = Goal(r_chair, [x, y])
        no_col = True
        for o in obs:
            no_col = no_col and Check_Circles_No_Collision(g, o)
        for p in goals:
            no_col = no_col and Check_Circles_No_Collision(g, p)
        if no_col:
            goals.append(Goal(r_chair, [x, y]))

        x = 22 * math.floor(i / N_obs) + 25
        y = 22 * (i % N_obs) + 13
        g = Goal(r_chair, [x, y])
        no_col = True
        for o in obs:
            no_col = no_col and Check_Circles_No_Collision(g, o)
        for p in goals:
            no_col = no_col and Check_Circles_No_Collision(g, p)
        if no_col:
            goals.append(Goal(r_chair, [x, y]))

        x = 22 * math.floor(i / N_obs) + 13
        y = 22 * (i % N_obs) + 25
        g = Goal(r_chair, [x, y])
        no_col = True
        for o in obs:
            no_col = no_col and Check_Circles_No_Collision(g, o)
        for p in goals:
            no_col = no_col and Check_Circles_No_Collision(g, p)
        if no_col:
            goals.append(Goal(r_chair, [x, y]))

        x = 22 * math.floor(i / N_obs) + 13
        y = 22 * (i % N_obs) + 13
        g = Goal(r_chair, [x, y])
        no_col = True
        for o in obs:
            no_col = no_col and Check_Circles_No_Collision(g, o)
        for p in goals:
            no_col = no_col and Check_Circles_No_Collision(g, p)
        if no_col:
            goals.append(Goal(r_chair, [x, y]))
    return goals

def Create_Start_Node(obs, goals1, goals2):
    """
    Creates the start node for our problem
    :param obs: list of the obstacle instances in our problem
    :param goals: list of the goal zone instances
    :returns: the start node instance
    """
    start_not_found = True
    while start_not_found:
        start_not_found = False
        x = 0
        y = 0
        x2 = 99
        y2 = 99
        start = Start_Node([x,y])
        start2 = Start_Node([x2,y2])
        for o in obs:
            if not Check_Point_Not_In_Circle(start,o) and not Check_Point_Not_In_Circle(start2,o):
                start_not_found = True
                continue
        for g in goals1:
            if not Check_Point_Not_In_Circle(start,g):
                start_not_found = True
                continue
        for g in goals2:
            if not Check_Point_Not_In_Circle(start2,g):
                start_not_found = True
                continue
    return start, start2

def CreateOutputFolder():
    """
    Creates the necessary output folders (if they don't already exist)
    :param save_imgs: True if we want to save our images (.png)
    :param save_gifs: True if we want to save our animations (.gif)
    :returns: path of the output folder (called "Output")
    """
    if not os.path.exists("Output"):
        os.mkdir("Output")

    temp_folder = "Output/Temp_Images"
    if not os.path.exists(temp_folder):
        os.mkdir(temp_folder)
    elif os.path.exists(temp_folder):
        for filename in os.listdir(temp_folder):
            file_path = os.path.join(temp_folder, filename)
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
        
    i = 0
    while os.path.exists("Output/%i/" % i):
        i += 1
    path = "Output/%i/" % i
    os.mkdir(path)
    return path

def main():
    path = None
    show_nav, show_arm, save_gifs, save_imgs = get_options()
    if save_gifs == 'y':
        save_gifs = True
    else:
        save_gifs = False
    if save_imgs == 'y':
        save_imgs = True
    else:
        save_imgs = False
    path = CreateOutputFolder()

    print("---- Starting the robotic waiter simulation ----\n")
    if show_nav == 'y':
        print("Navigating the robotic waiter to the tables...")
        map = Map()
        N_obs = 4
        N_goals = [2, 7, 9, 12,15]
        N_samples = 100
        obs = Create_Obstacles(N_obs, N_goals)
        goals = Create_Goal_Zones(N_goals, obs, N_obs)
        goals1 = list(goals[:int(len(goals) / 2)])
        goals2 = list(goals[int(len(goals) / 2):])
        start1, start2 = Create_Start_Node(obs, goals1, goals2)
        start = [start1, start2]
        samples, goal1_nodes, goal2_nodes = Create_Samples(map, start1, start2, obs, goals1, goals2,
                                                           N_samples=N_samples, N_knn=5)
        roadmap, PRM_graph, updated_obs = Create_Roadmap(samples, obs, goals1, goals2, N_knn=5)
        new_graph, trajectories1, trajectories2 = PRM_Solve(start, goals1, goals2, samples, goal1_nodes, goal2_nodes,
                                                            roadmap, PRM_graph)
        final_trajectory = TaskPlanner(new_graph, output_path=path).GetFinalTrajectory()
        if final_trajectory == None:
            print("No solution found!")
            return

        goals = [goals1, goals2]
        plotter = Plotter(map, obs, updated_obs, goals, start, samples,
                          roadmap, PRM_graph, new_graph, trajectories1,
                          trajectories2, final_trajectory, output_path=path,
                          save_img=save_imgs, save_gif=save_gifs)
        plotter.plot_init()
        plotter.plot_PRM()
        plotter.plot_knn()
        plotter.plot_Astar()
        plotter.Visualize_Final_Graph()
        plotter.plot4()
        plotter.plot_final()

    if show_arm == 'y':
        Serve_plate(output_path=path, save_gif=save_gifs, save_img=save_imgs)
    print("---- Thank you! ----\n"
          "The simulation is over!")
    return


def get_options():
    show_nav = None; show_arm = None; save_gifs = None; save_imgs = None
    print("                             (    (    (\n"
          "                         \    )    )    )   / \n"   
          "                          \~~~~~~~~~~~~~~~~/\n"
          "                           \              /\n"
          "                            \____________/")
    print("                  Welcome to Robotic waiter simulation!  \n")
    print("This simulation demonstrate the operation of robotic waiters in a restaurant.\n"
          "the simulation is devided into 2 parts: the navigation and the arm manipulation.\n")

    while show_nav != 'y' and show_nav != 'n':
        show_nav = str(input("Show robotic waiter navigation? (y/n):\n"))
    while show_arm != 'y' and show_arm != 'n':
        show_arm = str(input("Show robotic waiter arm manipulation? (y/n):\n"))
    while save_gifs != 'y' and save_gifs != 'n':
        save_gifs = str(input("Save gifs? (y/n):\n"))
    while save_imgs != 'y' and save_imgs != 'n':
        save_imgs = str(input("Save images? (y/n):\n"))
    return show_nav, show_arm, save_gifs, save_imgs

if __name__ == '__main__':
    main()