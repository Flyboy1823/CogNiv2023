import argparse
from Nav_Modules.Nav_Geometry import *
from Nav_Modules.Nav_MotionPlanner import *
from Nav_Modules.Nav_Plotter import *
from Nav_Modules.Nav_TaskPlanner import *
from Arm_Run import Arm_Run


def Create_Obstacles(N_obs, N_goals):
    """
    Creates the obstacles for our problem
    :param N_obs: the number of obstacles to instantiate
    :returns: list of the obstacle instances
    """
    #print the main tables
    #print side tables that aren't in obsitcles
    r_table = 6
    r_chair=2
    r_person=2.5
    obs = []
    for i in range(N_obs):
        for k in range(N_obs):
            obs.append(Obstacle(r_table,[22*i+19,22*k+19]))
    for i in range(N_obs):
        for k in range(N_obs):
            if (i*N_obs+k) in N_goals:
                continue
            else:            
                obs.append(Obstacle(r_chair,[22*i+24,22*k+22]))
                obs.append(Obstacle(r_chair,[22*i+24,22*k+16]))
                obs.append(Obstacle(r_chair,[22*i+14,22*k+22]))
                obs.append(Obstacle(r_chair,[22*i+14,22*k+16]))
    obs.append(Obstacle(r_person,[5,50]))
    obs.append(Obstacle(r_person,[95,50]))
    obs.append(Obstacle(r_person,[28,28]))
    obs.append(Obstacle(r_person,[28,74]))
    obs.append(Obstacle(r_person,[52,50]))
    obs.append(Obstacle(r_person,[52,72]))
    obs.append(Obstacle(r_person,[74,52]))
    obs.append(Obstacle(r_person,[74,28]))
    
    return obs

def Create_Goal_Zones(N_goals, obs, N_obs):
    """
    Creates the goal zones for our problem
    :param N_goals: the number of goal zones to instantiate
    :param obs: list of the obstacle instances in our problem
    :returns: list of the goal zone instances
    """
    goals = []
    r_chair=1  
    for i in N_goals:
        x = 22*math.floor(i/N_obs)+25
        y = 22*(i%N_obs)+25
        g = Goal(r_chair,[x,y])
        no_col = True
        for o in obs:
            no_col = no_col and Check_Circles_No_Collision(g,o)
        for p in goals:
            no_col = no_col and Check_Circles_No_Collision(g,p)
        if no_col:
            goals.append(Goal(r_chair,[x,y]))
            
        x = 22*math.floor(i/N_obs)+25
        y = 22*(i%N_obs)+13
        g = Goal(r_chair,[x,y])
        no_col = True
        for o in obs:
            no_col = no_col and Check_Circles_No_Collision(g,o)
        for p in goals:
            no_col = no_col and Check_Circles_No_Collision(g,p)
        if no_col:
            goals.append(Goal(r_chair,[x,y]))
            
            
        x = 22*math.floor(i/N_obs)+13
        y = 22*(i%N_obs)+25
        g = Goal(r_chair,[x,y])
        no_col = True
        for o in obs:
            no_col = no_col and Check_Circles_No_Collision(g,o)
        for p in goals:
            no_col = no_col and Check_Circles_No_Collision(g,p)
        if no_col:
            goals.append(Goal(r_chair,[x,y]))
            
            
        x = 22*math.floor(i/N_obs)+13
        y = 22*(i%N_obs)+13
        g = Goal(r_chair,[x,y])
        no_col = True
        for o in obs:
            no_col = no_col and Check_Circles_No_Collision(g,o)
        for p in goals:
            no_col = no_col and Check_Circles_No_Collision(g,p)
        if no_col:
            goals.append(Goal(r_chair,[x,y]))
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

def CreateOutputFolder(save_imgs,save_gifs,save_pddl):
    """
    Creates the necessary output folders (if they don't already exist)
    :param save_imgs: True if we want to save our images (.png)
    :param save_gifs: True if we want to save our animations (.gif)
    :param save_pddl: True if we want to save our PDDL files (.pddl)
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
        
    path = None
    if save_imgs or save_gifs or save_pddl:
        i = 0
        while os.path.exists("Output/%i/" % i):
            i += 1
        path = "Output/%i/" % i
        os.mkdir(path)
    return path

def main():
    argus = options()
    N_obs = argus.o
    N_goals = argus.n; 
    N_samples = argus.s; 
    show_knn = eval(argus.k); 
    show_animation = eval(argus.a)
    save_pddl = eval(argus.p); 
    save_gifs = eval(argus.g); 
    save_imgs = eval(argus.i); 
    show_arm = eval(argus.arm); 
    show_nav = eval(argus.nav)
    path = CreateOutputFolder(save_imgs,save_gifs,save_pddl)

    print("** Starting simulation **\n")
    if show_nav:
        map = Map()
        obs = Create_Obstacles(N_obs, N_goals)
        goals = Create_Goal_Zones(N_goals, obs, N_obs)
        goals1 = list(goals[:int(len(goals)/2)])
        goals2 = list(goals[int(len(goals)/2):])
        start1, start2 = Create_Start_Node(obs, goals1, goals2)
        start = [start1, start2]
        samples, goal1_nodes, goal2_nodes = Create_Samples(map, start1, start2, obs, goals1, goals2, N_samples=N_samples, N_knn=5)
        roadmap, PRM_graph, updated_obs = Create_Roadmap(samples, obs, goals1, goals2, N_knn=5)
        new_graph, trajectories1, trajectories2= PRM_Solve(start, goals1, goals2, samples, goal1_nodes, goal2_nodes, roadmap, PRM_graph)
        final_trajectory = TaskPlanner(new_graph, save_pddl=save_pddl, output_path=path).GetFinalTrajectory()
        if final_trajectory == None:
            return

        goals = [goals1, goals2]
        plotter = Plotter(map, obs, updated_obs, goals, start, samples, roadmap, PRM_graph, new_graph, trajectories1, trajectories2, final_trajectory, output_path=path, save_img=save_imgs, save_gif=save_gifs, show_anim=show_animation)
        if show_animation or show_knn or save_gifs: 
            plotter.plot_init()
            plotter.plot_PRM()
            if show_knn:
                plotter.plot_knn()
            plotter.plot_Astar()
            plotter.Visualize_Final_Graph()
        else:
            plotter.plot4()
        plotter.plot_final()

    if show_arm:
        Arm_Run(output_path=path, save_img=save_imgs, save_gif=save_gifs, show_anim=show_animation)
    print("** Simulation complete! **")
    return

def options():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-nav", dest='nav', required=False, default="True", help="Run the navigation simulation")
    parser.add_argument("-arm", dest='arm', required=False, default="True", help="Run the arm simulation")
    parser.add_argument("-n", dest='n', required=False, default=[2, 7, 9, 12,15], help="Table Numbers that are goals to create")
    parser.add_argument("-o", dest='o', required=False, default = 4, help="Number of obstacles to create")
    parser.add_argument("-s", dest='s', required=False, default = 100, help="Number of samples for PRM")
    parser.add_argument("-a", dest='a', required=False, default="False", help="Show animations")
    parser.add_argument("-k", dest='k', required=False, default="False", help="Show the k-nearest neighbors animation")
    parser.add_argument("-p", dest='p', required=False, default="False", help="Save .pddl files")
    parser.add_argument("-i", dest='i', required=False, default="False", help="Save .png image files")
    parser.add_argument("-g", dest='g', required=False, default="False", help="Save .gif animation files")
    return parser.parse_args()

if __name__ == '__main__':
    main()