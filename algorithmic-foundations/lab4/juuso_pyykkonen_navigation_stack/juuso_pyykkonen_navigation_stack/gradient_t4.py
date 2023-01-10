import numpy as np

KP_POS = 0.1
KP_NEG = 0.000003

def polar_to_cartesian_coordinate(scan, angle_offset):
    
    ranges = scan.ranges
    angle_min = scan.angle_min
    angle_increment = scan.angle_increment

    angle = angle_min + angle_offset           # start angle  
    tf_points = []         
    for range in ranges:
        x = range * np.cos(angle)
        y = range * np.sin(angle)

        # current angle is last angle add angle_increment
        angle += angle_increment
        if (range < 3.45):     
            tf_points.append([x,y])

    # transform the transformed points to numpy array
    # points_np = np.array(tf_points)
    return tf_points


def gradient_attraction(pos, goal):
    x = KP_POS * (pos[0] - goal[0])
    y = KP_POS * (pos[1] - goal[1])
    return [x,y]

def gradient_repulsion(pos, obstacle):
    x_p = pos[0]
    x_g = obstacle[0]
    y_p = pos[1]
    y_g = obstacle[1]

    x = KP_NEG * (2 * (x_p - x_g))/(((x_g - x_p)**2 + (y_g - y_p)**2)**2)
    y = KP_NEG * (2 * (y_p - y_g))/(((x_g - x_p)**2 + (y_g - y_p)**2)**2)

    return [x,y]

def gradient(scan, pos, goal, yaw):

    obstacles = polar_to_cartesian_coordinate(scan, yaw)

    gradient_arr = []
    gradient_arr.append(gradient_attraction(pos, goal))
    
    for obst in obstacles:
        gradient_arr.append(gradient_repulsion(pos, obst))

    x = 0.0
    y = 0.0
    for grad in gradient_arr:
        x += grad[0]
        y += grad[1]

    return [x,y]