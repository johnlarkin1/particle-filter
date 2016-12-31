import numpy as np
import matplotlib.pyplot as plt

FT = 0.3048
DEG = np.pi / 180

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

def plot_segment(p0, p1):

    x0, y0 = p0
    x1, y1 = p1
    
    plt.plot([x0, x1], [y0, y1], 'b')

def plot_robot(x, y, theta):

    dx = 0.2*np.cos(theta)
    dy = 0.2*np.sin(theta)

    plt.gca().add_artist(plt.Circle((x, y), 0.15, color='r'))
    plt.plot([x, x+dx], [y, y+dy], color='k')

def plot_ray(x, y, angle, distance):

    dx = distance*np.cos(angle)
    dy = distance*np.sin(angle)

    plt.plot([x, x+dx], [y, y+dy], color='g')
    

for seg in WALL_SEGS:
    p0, p1 = seg
    plot_segment(p0, p1)




