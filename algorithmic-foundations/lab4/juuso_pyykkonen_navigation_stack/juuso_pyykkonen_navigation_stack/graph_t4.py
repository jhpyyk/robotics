import numpy as np
import matplotlib.pyplot as plt
import networkx

def graph():

    waypoints ={0:[0,0], 1:[3, 0.4], 2:[3.4,-2.6], 3:[5.8, -2.5], 4:[6, 7], 5:[7.8,7],
                    6:[8,-2.7], 7:[7.5,-5], 8:[3, -5.3], 9:[3,-9.5], 10:[6,-8], 11:[0,-8],
                    12:[0,-5], 13:[-2,-4], 14:[-3,-1], 15:[-5,2], 16:[-1.8,2.5], 17:[-1.4,4.6],
                    18:[0.5,4.5], 19:[0.6,2.3], 20:[3,2.3], 21:[3,7], 22:[-1.3,7],
                    23:[-6.7, 0], 24:[-6,-3], 25:[-4,-6], 26:[-8,-7]}

    neighbor_pairs = [(0,1), (1,2), (2,3), (3,4), (3,6),
                        (4,5), (4,21), (21,22), (5,6), (6,7),
                        (7,8), (8,9), (9,10), (9,11), (11,12), (11,25), (11,26),
                        (12,13), (13,14), (14,15), (15, 23), (23, 24),
                        (24, 25), (24, 26), (25, 26), (15,16), (16,17),
                        (17,18), (18,19), (19,20), (20,21)]

    map_graph = networkx.Graph()
    map_graph.add_edges_from(neighbor_pairs)
    positions = networkx.spring_layout(map_graph)
    networkx.draw(map_graph, pos=positions, with_labels=True, node_color='white')


    adjacency_matrix = np.zeros([len(waypoints), len(neighbor_pairs)])
    for i in neighbor_pairs:
        adjacency_matrix[i[0], i[1]] = 1
        adjacency_matrix[i[1], i[0]] = 1

    weight_matrix = np.zeros([len(waypoints), len(neighbor_pairs)])
    for i in neighbor_pairs:
        point = [waypoints[i[0]][0] - waypoints[i[1]][0], waypoints[i[0]][1] - waypoints[i[1]][1]]
        distance = np.sqrt((np.power(point[0], 2) + np.power(point[1], 2)))
        weight_matrix[i[0], i[1]] = distance
        weight_matrix[i[1], i[0]] = distance
    
    return [adjacency_matrix, weight_matrix, waypoints]

