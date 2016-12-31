import particle_filt_help as help
import numpy as np
import matplotlib.pyplot as plt

DEG = np.pi / 180 
FT = 0.3048 

WALL_SEGS = [
        [(-0.5 * 3 * FT, -0.5 * 3 * FT), (3* FT * -0.5,  5.5*FT * 3 )],
        [(-0.5 * 3 * FT, -0.5 * 3 * FT), (3* FT *  3.5, -0.5*FT * 3 )],
        [(-0.5 * 3 * FT,  5.5 * 3 * FT), (3* FT *  3.5,  5.5*FT * 3 )],
        [( 3.5 * 3 * FT, -0.5 * 3 * FT), (3* FT *  3.5,  5.5*FT * 3 )],
        [( 0.5 * 3 * FT, -0.5 * 3 * FT), (3* FT *  0.5,  1.5*FT * 3 )],
        [( 0.5 * 3 * FT,  3.5 * 3 * FT), (3* FT *  0.5,  5.5*FT * 3 )],
        [( 1.5 * 3 * FT,  3.5 * 3 * FT), (3* FT *  1.5,  4.5*FT * 3 )],
        [( 0.5 * 3 * FT,  0.5 * 3 * FT), (3* FT *  2.5,  0.5*FT * 3 )],
        [( 1.5 * 3 * FT,  1.5 * 3 * FT), (3* FT *  3.5,  1.5*FT * 3 )],
        [(-0.5 * 3 * FT,  2.5 * 3 * FT), (3* FT *  1.5,  2.5*FT * 3 )],
        [( 2.5 * 3 * FT,  2.5 * 3 * FT), (3* FT *  3.5,  2.5*FT * 3 )],
        [( 1.5 * 3 * FT,  3.5 * 3 * FT), (3* FT *  3.5,  3.5*FT * 3 )],
        [( 1.5 * 3 * FT,  4.5 * 3 * FT), (3* FT *  2.5,  4.5*FT * 3 )]
   ]


def find_xsection_bn_seg_and_ray(ray, wall_segment):
    # ok so we're about to redo this
    # pi = p0 + alpha * d
    # pi = p1 + beta * (p2 - p1)
    p0 = ray.origin
    p1 = wall_segment[0]
    p2 = wall_segment[1]

    starting_pt = help.vec2(p1[0], p1[1])
    ending_pt = help.vec2(p2[0], p2[1])

    k = ending_pt - starting_pt
    k_vec = np.array([k.getx(), k.gety()])
    d_vec = np.array([ray.direction.getx(), ray.direction.gety()])
    matrix = np.array([d_vec, -k_vec])
    matrix = matrix.transpose()

    temp = starting_pt - p0
    p_vec = np.array([temp.getx(), temp.gety()])

    a_b_vec = np.linalg.solve(matrix, p_vec)
    alpha = a_b_vec[0]
    beta = a_b_vec[1]

    if beta < 0 or beta > 1:
        return None
    elif alpha > 0 :
        return np.abs(alpha) # this is the distance to our hit point

def expected_ranges(kinetic_pose, angles, walls):
    # angles - np.array(angles) relative to forward or some directional theta; walls - our walls; kp - x, y, theta of our robot
    # return for each angle, some distance before it hits (in theory!) this is meant to be our expected values

    ray = help.Ray()
    robot_center = help.vec2(kinetic_pose.x, kinetic_pose.y)
    robot_angle = kinetic_pose.theta

    #####################################
    ##########    TESTING   #############
    #####################################
    #robot_angle = 90*DEG
    #robot_center = help.vec2(9*FT,3*FT)
    ray.update_origin(robot_center) # this should set a POINT to be the robots center

    distance_exp = []
    for angle in angles:
        direction = help.vec2(np.cos(angle + robot_angle),np.sin(angle + robot_angle))
        ray.update_direction(direction)
        min_dist = np.Inf
        for wall_seg in walls:
            distance = find_xsection_bn_seg_and_ray(ray, wall_seg)
            if distance == None:
                continue
            if distance < min_dist:
                used_wall = wall_seg
                min_dist = distance
            
        #print("This is the used wall: {}".format(used_wall))
        #print("This is the actual distance: {}".format( min_dist))
        distance_exp.append(min_dist)
    return (distance_exp, used_wall)

def query_user():
    print("Specify the following on a grid dimension. For example, (0,1) or (1,3) etc.")
    x_position = raw_input("What is the x pos of your robot? ")
    x_pos = 3 * FT * int(x_position)
    y_position = raw_input("What is the y pos of your robot? ")
    y_pos = 3 * FT * int(y_position)
    theta_rob =  raw_input("What is the theta of your robot? (specify in degrees) ")
    theta = int(theta_rob) * DEG
    return (x_pos, y_pos, theta)

if __name__ == '__main__':

    (x_pos, y_pos, theta) = query_user()
    curr_pose = help.CurrPose(x_pos, y_pos, theta)
    width_of_scan = 10
    angles_to_use = np.linspace(-width_of_scan*DEG, width_of_scan*DEG, 10)
    angles_to_use = angles_to_use.tolist()

    (expected_distances, wall) = expected_ranges(curr_pose, angles_to_use, WALL_SEGS)
    
    (wall_start, wall_end) = wall
    plt.plot([wall_start[0], wall_end[0]], [wall_start[1],wall_end[1]], color ='#ff69b4')

    for angle in angles_to_use:
        pt_on_line = help.vec2(np.cos(angle + curr_pose.theta),np.sin(angle + curr_pose.theta))
        pt_on_line = pt_on_line + help.vec2(curr_pose.x, curr_pose.y)
        # print("This is our point on our line: ({}, {})".format(pt_on_line.getx(), pt_on_line.gety()))
        plt.plot([curr_pose.x, pt_on_line.getx()], [curr_pose.y, pt_on_line.gety()],'g')

    # These are the measured distances using the scan data measured from robot origin
    print('These are our expected distances: {}'.format(expected_distances))

    viz.plot_robot(curr_pose.x, curr_pose.y, curr_pose.theta)
        
    plt.axis('equal') # squares should be square
    plt.show()
    

