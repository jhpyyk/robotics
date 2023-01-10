import numpy as np
import matplotlib.pyplot as plt
import networkx

def graph(waypoints, neighbor_pairs):

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
    
    return [adjacency_matrix, weight_matrix]

