import particle_filt_help as help
import numpy as np
import matplotlib.pyplot as plt

# Global Variables - bad coding practice
# Measurement conversion
DEG = np.pi / 180
FT = 0.3048

# Number of objects
NUM_OF_RAYS = 8
NUM_OF_PARTICLES = 24

# How wide should we look?
DEG_SPAN = 30*DEG

# Errors
SIGMA_XY = 0.25
SIGMA_THETA = 0.01
SIGMA_Z = 0.25

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

ROBOT_PATH = [
	(0, 0, 90*DEG),
	(0, 1, 90*DEG), 
	(0, 2, 90*DEG), 
	(0, 2, 0), 
	(1, 2, 0),
	(1, 2, -90*DEG),
	(1, 1, -90*DEG), 
	(1, 1, 0), 
	(2, 1, 0), 
	(3, 1, 0),
	(3, 1, -90*DEG), 
	(3, 0, -90*DEG)
]

SHORT_ROBOT_PATH = [
	(0, 0, 90*DEG),
	(0, 1, 90*DEG), 
	(0, 2, 90*DEG), 
	(0, 2, 0), 
	(1, 2, 0),
	(1, 2, -90*DEG),
	(1, 1, -90*DEG), 
	(1, 1, 0), 
	(2, 1, 0), 
	(3, 1, 0),
]

PATH_COMMANDS = [
	'forward',
	'forward',
	'turnright',
	'forward',
	'turnright',
	'forward',
	'turnleft',
	'forward',
	'forward',
	'turnright', 
	'forward'
]

SHORT_PATH_COMMANDS = [
	'turnleft',
	'forward',
	'forward',
	'turnright',
	'forward',
	'turnright',
	'forward',
	'turnleft',
	'forward',
	'forward',
]

# This just plots a line segment
def plot_segment(p0, p1):

    x0, y0 = p0
    x1, y1 = p1
    
    plt.plot([x0, x1], [y0, y1], 'b')

# This plots our actual robot as a circle
def plot_robot(x, y, theta):

    dx = 0.2*np.cos(theta)
    dy = 0.2*np.sin(theta)

    plt.gca().add_artist(plt.Circle((x, y), 0.15, color='g'))
    plt.plot([x, x+dx], [y, y+dy], color='k')

# This plots a ray
def plot_ray(x, y, angle, distance):

    dx = distance*np.cos(angle)
    dy = distance*np.sin(angle)

    plt.plot([x, x+dx], [y, y+dy], color='g')

# This initializes our class
def pseudo_initialization():
	robot_pose = help.CurrPose(0, 0, 0)
	kinect_pose = robot_pose

	angles_to_use = np.linspace(-DEG_SPAN, DEG_SPAN, NUM_OF_RAYS)
	angles_to_use = angles_to_use.tolist()

	particles = np.zeros((3, NUM_OF_PARTICLES))

	width = 4
	height = 6

	# Scattering our points
	x_coords = np.linspace(0, (width-1.5), NUM_OF_PARTICLES/height)
	x_coords = np.append(x_coords, [x_coords]*(height-1))

	y_coords = np.linspace(0, (height-1.5), NUM_OF_PARTICLES/width)
	y_coords = np.repeat(y_coords, width)
	theta_coords = np.linspace(-180*DEG, 180*DEG, NUM_OF_PARTICLES)

	particles = np.array(\
		[x_coords, \
		 y_coords, \
		 theta_coords] )

	# need this guy to latch on
	particles[:,0] = np.array([0,0,0])

	weights = np.ones((NUM_OF_PARTICLES)) 

	return (angles_to_use, particles, weights)

# This is our motion update.
def motion_update(particles, control, sigma_xy, sigma_theta):
	nparticles = particles.shape[1]
	noisex = np.random.normal(scale = sigma_xy, size = nparticles)
	noisey = np.random.normal(scale = sigma_xy, size = nparticles)
	thetanoise = np.random.normal(scale = sigma_theta, size = nparticles)
	if control == "forward":
		particles[0,:] += 3 * FT * np.cos(particles[2,:]) + noisex
		particles[1,:] += 3 * FT * np.sin(particles[2,:]) + noisey
		particles[2,:] += thetanoise
	elif control == "backward":
		particles[0,:] -= 3 * FT * np.cos(particles[2,:]) + noisex
		particles[1,:] -= 3 * FT * np.sin(particles[2,:]) + noisey
		particles[2,:] += thetanoise
	elif control == "turnleft":
		particles[0,:] += noisex
		particles[1,:] += noisey
		particles[2,:] += 90*DEG + thetanoise
	elif control == "turnright":
		particles[0,:] += noisex
		particles[1,:] += noisey
		particles[2,:] -= 90*DEG + thetanoise
	return particles

# This reads our data in
def read_measurements(input_file):
	line = input_file.readline()
	line = line.split()
	measurement = map(float, line)
	return measurement

# This is our measurement update
def measurement_update(particles, measured_distances):
	# read in our measured distances, just entering this in manually
	z = measured_distances

	# This is so we can alter how many rays we actually want to use
	z = z[::len(z)/NUM_OF_RAYS]
	nan_location = np.isnan(z)

	weights = np.ones(NUM_OF_PARTICLES)
	nparticles = particles.shape[1]

	for particle_index in range(nparticles):
		expected_distances = help.expected_ranges(particles[:,particle_index], angles_to_use, WALL_SEGS)
		expected_dist = np.array(expected_distances)

		for ray_index, val in enumerate(nan_location):
			if val:
				weights[particle_index] = weights[particle_index]
			else:
				weights[particle_index] = weights[particle_index] * np.exp(-( ( (z[ray_index]-expected_dist[ray_index])**2 ) / (2*SIGMA_Z**2) ) )

	# Need to normalize our weights
	weights = weights / (np.sum(weights))

	particles_to_use = np.linspace(0, nparticles - 1, num = nparticles)
	indices = np.random.choice(particles_to_use, size = nparticles, p = weights)
	indices = indices.astype(int)

	particles = particles[:,indices]

	return particles

def read_particle_locations(input_file):
	x_positions = input_file.readline()
	y_positions = input_file.readline()
	theta_positions = input_file.readline()

	x_positions = x_positions.split()
	y_positions = y_positions.split()
	theta_positions = theta_positions.split()

	x_positions = map(float, x_positions)
	y_positions = map(float, y_positions)
	theta_positions = map(float, theta_positions)

	curr_postions = np.array([ \
		x_positions,
		y_positions, 
		theta_positions])
	return curr_postions

def plot_particles(particle_matrix):
	for idx in range(particle_matrix.shape[1]):
		x = particle_matrix[0,idx]
		y = particle_matrix[1,idx]
		theta = particle_matrix[2, idx]

		# print("This is x {} this is y {} this is theta {}".format(x, y, theta))

		dx = 0.05*np.cos(theta)
		dy = 0.05*np.sin(theta)

		plt.gca().add_artist(plt.Circle((x, y), 0.05, color='#ff1493'))
		plt.plot([x, x+dx], [y, y+dy], color='#8a2be2')

def plot_robot_path(robot_paths, i):
	position = robot_paths[i]
	plot_robot(position[0], position[1], position[2])

if __name__ == "__main__":

	use_data = True

	ystep = 0.5

	if use_data:
		particle_position_file = open('robot_output.txt')
		measure_file = open('measured_distances.txt')
		angles_to_use, particles, weights = pseudo_initialization()

		y = np.zeros_like(particles)

		
		for seg in WALL_SEGS:
			p0, p1 = seg
			plot_segment(p0, p1)

		plot_particles(particles)
		plot_robot(0, 0, 0)
		plt.xlabel('X Coordinates')
		plt.ylabel('Y Coordinates')
		plt.title('Original Starting Position')
		plt.axis('equal')
		plt.show()

		for i, path in enumerate(SHORT_PATH_COMMANDS):
			for seg in WALL_SEGS:
				p0, p1 = seg
				plot_segment(p0, p1)
			
			# Motion Update
			particles = motion_update(particles, path, SIGMA_XY, SIGMA_THETA)

			plot_particles(particles)

			plot_robot_path(SHORT_ROBOT_PATH, i)

			plt.xlabel('X Coordinates')
			plt.ylabel('Y Coordinates')
			plt.title('Iteration {}: After Motion Update'.format(i))
			plt.axis('equal')
			plt.show()

			# Measurement Update
			measured_distances = read_measurements(measure_file)

			for seg in WALL_SEGS:
				p0, p1 = seg
				plot_segment(p0, p1)
			
			if i != 9:
				particles = measurement_update(particles, measured_distances)

			plot_particles(particles)

			plot_robot_path(SHORT_ROBOT_PATH, i)
			
			plt.xlabel('X Coordinates')
			plt.ylabel('Y Coordinates')
			plt.title('Iteration {}: After Measurement Update'.format(i))
			plt.axis('equal')
			plt.show()



	else:
		particle_position_file = open('robot_output.txt')
		plt.figure("Particle Filter with Leela")
		plt.title("Visualization of Particle Filter")
		plt.xlabel("X position in world (m)")
		plt.ylabel("Y position in world (m)")
		
		(angles_to_use, particles, weights) = pseudo_initialization()
		for seg in WALL_SEGS:
			p0, p1 = seg
			plot_segment(p0, p1)
		plot_particles(particles)
		plot_robot(0, 0, 0)
		plt.axis('equal')
		plt.show()

		for i in range(10):
			plt.figure("Particle Filter with Leela")
			plt.title("Visualization of Particle Filter")
			plt.xlabel("X position in world (m)")
			plt.ylabel("Y position in world (m)")
			for seg in WALL_SEGS:
			    p0, p1 = seg
			    plot_segment(p0, p1)
			curr_particles = read_particle_locations(particle_position_file)
			plot_particles(curr_particles)
			plot_robot_path(SHORT_ROBOT_PATH, i)
			plt.axis('equal')
			plt.show()

		
		particle_position_file.close()






