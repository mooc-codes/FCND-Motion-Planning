import numpy as np
from planning_utils import a_star_graph, heuristic, create_grid, a_star_grid
from sampling import Sampler, create_graph
import json

def get_path():
        
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        north_offset = -316
        east_offset = -445
        sampler = Sampler(data)
        print("AAA")
        polygons = sampler._polygons
        print("BBB")
        nodes = sampler.sample(3000)
        # (339, 380) (441, 519)
        start = (0, 0, 6)
        goal = (331, 116, 19)
        nodes += [start, goal]
        print(nodes)
        g = create_graph(nodes, k=10, polygons=polygons)
        path, _ = a_star_graph(g,heuristic,start, goal)
        waypoints = [[int(p[0]) , int(p[1]) ,int(p[2]), 0] for p in path]
        path_dict = {'path' : waypoints}
    
        print(waypoints)
        with open('path.json', 'w') as json_file:
            json.dump(path_dict, json_file)

        with open('path.json', 'r') as json_file2:
            dta = json.load(json_file2)  

        print(dta['path']) 
        return waypoints     


if __name__ == '__main__':
    get_path()