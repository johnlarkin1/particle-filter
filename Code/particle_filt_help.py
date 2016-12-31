import numpy as np


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


class CurrPose:
	def __init__(self, x = None, y = None, theta = None):
		self.x = x
		self.y = y
		self.theta = theta

class Ray:
    def __init__(self, origin = None, direction = None):
        self.origin = origin
        self.direction = direction
    def update_origin(self, new_origin):
        self.origin = new_origin
    def update_direction(self, new_dir):
        self.direction = new_dir

# This should interact with the map somehow
class Wall_Segment:
    def __init__(self, starting_pt = None, ending_pt = None):
        self.starting_pt = starting_pt
        self.ending_pt = ending_pt
        self.segment_vec = ending_pt - starting_pt

class vec2:
    # Initialization
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def getx(self):
        return self.x
    def gety(self):
        return self.y
    # Addition with vector
    def __add__(self, other):
        return vec2(self.x + other.x, self.y + other.y)
    # Subtraction with vector
    def __sub__(self, other):
        return vec2(self.x - other.x, self.y - other.y)
    # Multiplication with *scalar*
    def __mul__(self, other):
        return vec2(self.x * other, self.y * other)
    # Division with *scalar*
    def __div__(self, other):
        return vec2(self.x / other, self.y / other)
    # Multiplication either way
    __rmul__ = __mul__
    # Lin alg! Dot product
    def dot(self, other):
        return (self.x * other.x) + (self.y * other.y)
    # Length
    def length(self):
        return (self.x**2 + self.y**2)**(1.0/2.0)
    # Normalize - this does NOT return a new vector
    def normalize(self):
        length = self.length()
        self.x = self.x / length
        self.y = self.y / length
    # Normal - this DOES return a new vector
    def normal(self):
        length = self.length()
        return vec2(self.x/length, self.y/length)

class vec3:
        def __init__(self, x, y, z):
                # Can set multiple things on one line; see this link http://openbookproject.net/thinkcs/python/english3e/tuples.html
                (self.x, self.y, self.z) = (x, y, z)
        # Adding with vectors
        def __add__(self, other):
                return vec3(self.x + other.x, self.y + other.y, self.z + other.z)
        # Subtracting with vectors
        def __sub__(self, other):
                return vec3(self.x - other.x, self.y - other.y, self.z - other.z )
        # Division with a scalar
        def __div__(self, other):
                return vec3(self.x / other, self.y / other, self.z / other)
        # Multiplication with a scalar
        def __mul__(self, other):
                return vec3(self.x * other, self.y * other, self.z * other)
        # let's make sure we can have multiplication either way
        __rmul__ = __mul__
        # Dot product 
        def dot(self, other):
                # this should return a scalar
                return (self.x * other.x) + (self.y * other.y) + (self.z * other.z)
        def length(self):
        # Length or magnitude of the vector
                return (self.x**2 + self.y**2 + self.z**2)**(1.0/2.0)
        # Cross product. Tells the orthogonal vector. 
        def cross(self, other):
                # This is just going to be the formula for 3 dimensions; x product returns a normal that is pointing in the opposite direction
                # this is also going to be a x b
                return vec3(self.y*other.z - self.z * other.y, self.z * other.x - self.x * other.y, self.x * other.y - self.y * other.x)
        def normalize(self):
                length = self.length()
                self.x /= length
                self.y /= length
                self.z /= length
        def normal(self):
                length = self.length()
                return vec3(self.x/length, self.y / length, self.z / length)
        def componenets(self):
                return (self.x, self.y, self.z)

def expected_ranges(kinect_pose, angles, walls):
    ray = Ray()
    robot_center = vec2(kinect_pose[0], kinect_pose[1])
    
    robot_angle = kinect_pose[2]

    ray.update_origin(robot_center) 

    distance_exp = []
    for angle in angles:
        direction = vec2(np.cos(angle+robot_angle),np.sin(angle + robot_angle))
        ray.update_direction(direction)
        min_dist = np.Inf 
        for wall_seg in walls:
            distance = find_xsection_bn_seg_and_ray(ray, wall_seg)
            if distance == None:
                continue
            if distance < min_dist:
                min_dist = distance 
        distance_exp.append(min_dist)
    
    return distance_exp
    

def find_xsection_bn_seg_and_ray(ray, wall_segment):
    p0 = ray.origin
    p1 = wall_segment[0]
    p2 = wall_segment[1]
    
    starting_pt = vec2(p1[0], p1[1])
    ending_pt = vec2(p2[0], p2[1]) 

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
    elif alpha > 0:
        return alpha # this is the distance to our hit point


def find_xsection_bn_seg_and_ray_old(ray, wall_segment):
    start_pt = wall_segment[0]
    end_pt = wall_segment[1]
    starting_pt = vec2(start_pt[0], start_pt[1])
    ending_pt = vec2(end_pt[0], end_pt[1]) 

    v1 = vec3(ray.origin.getx(), ray.origin.gety(), 0) - vec3(starting_pt.getx(), starting_pt.gety(), 0)
    v2 = vec3(ending_pt.getx(), ending_pt.gety(), 0) - \
            vec3(starting_pt.getx(), starting_pt.gety(), 0)
    v3 = vec3(-ray.direction.gety(), ray.direction.getx(), 0)
    
    if v2.dot(v3) == 0:
        return None # there is no intersection; they are parallel
    crossed = v2.cross(v1)
    time = crossed.length() / (v2.dot(v3))
    return time 