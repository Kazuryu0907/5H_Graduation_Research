import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import vonmises
#-----CONST------
POTENTIAL_MAX = 10
POTENTIAL_MIN = -10
WEIGHT_OBST = 1
WEIGHT_GOAL = 1
dt = 0.01
d_star = 0.5
speed = 0.2
Nobs = 10
#----Params------
# obs_poss = [[5,6]]
# obs_vels = [[0,0]]
obs_poss = [[6.5,6.5]]
obs_vels = [[2.,0.]]
# obs_poss = [[5.,10.],[5.,5.],[8.,2.],[2.,6.],]
# obs_poss = np.array([[0,2],[12,4],[0,6],[12,8],[0,10]],dtype=np.float64)
# obs_vels = np.array([[0.5,-.0],[-0.5,-0.],[.5,0],[-.5,-0],[.5,0]])
# obs_poss = np.array([[i*0.5+2.5,5] for i in range(20)],dtype=np.float64)
# obs_vels = np.array([[0,0.] for _ in range(20)])
# goal_poss = [[10.,10.],[0.,0]]
goal_poss = [[10.,10.]]
# robs_poss = [[0.,0.],[10.,10.]]
robs_poss = [[0.,0.]]

robs_routes = [[i] for i in robs_poss]
obs_routes = [obs_poss[-1].copy()]
def calc_pot(x,y,obs_poss):
    out_pot = 0
    d = lambda x,y:np.sqrt(np.power(x[0]-y[0],2) + np.power(x[1]-y[1],2))
    for i,obs_pos in enumerate(obs_poss):
        vel = obs_vels[i]
        vel_dot = np.sqrt(vel[0]**2+vel[1]**2)
        obs_pot = 0
        theta = np.arctan2(vel[1],vel[0])
        ang = np.arctan2(y-obs_pos[1],x-obs_pos[0])
        von = 3*vonmises.pdf(theta-ang,vel_dot+1e-10)
        # von = 1
        obs_pot = von*np.exp(-d(obs_pos,[x,y])/2.)/(2*np.pi)
        out_pot += WEIGHT_OBST * obs_pot

    return out_pot

getcolum = lambda x,i:list(map(lambda y:y[i],x))
pot = lambda x,y:calc_pot(x,y,obs_poss)

#---contour------
X,Y = np.meshgrid(np.linspace(-2,15,100),np.linspace(15,-2,100))
Z = calc_pot(X,Y,obs_poss)
Z = np.vectorize(lambda x:POTENTIAL_MAX if x > POTENTIAL_MAX else x)(Z)
Z = np.vectorize(lambda x:POTENTIAL_MIN if x < POTENTIAL_MIN else x)(Z)

# fig,ax = plt.subplots(1,1)
fig = plt.figure(figsize = (8, 8))
ax = fig.add_subplot(111, projection='3d')

# cont_pot = ax.contour(X,Y,Z)
# ax.plot_surface(X,Y,Z,cmap='viridis')
ax.contour(X,Y,Z)
ax.set_xlim(-2,15)
ax.set_ylim(-2,15)
# ax.set_aspect("equal")
print(vonmises.pdf(np.pi/2,3.99))
plt.show()

