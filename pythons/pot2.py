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
# obs_poss = [[8,6],[5,5]]
# obs_vels = [[-0.,-0.],[-0.7,-0.7]]
# obs_poss = [[5.,10.],[5.,5.],[8.,2.],[2.,6.],]
# obs_poss = [[0,2],[12,4],[0,6],[12,8],[0,10]]
# obs_vels = [[0.7,-.0],[-0.7,-0.],[.7,0],[-.7,-0],[.7,-0]]
obs_poss = np.array([[5,i*0.5+2.5] for i in range(Nobs-1)],dtype=np.float64)
obs_vels = np.array([[0,0] for _ in range(Nobs-1)],dtype=np.float64)
obs_poss = np.concatenate([obs_poss,[[5,5]]]).tolist()
obs_vels = np.concatenate([obs_vels,[[-2,-2]]]).tolist()

# goal_poss = [[10.,10.],[0.,0]]
goal_poss = [[10.,10.]]
# robs_poss = [[0.,0.],[10.,10.]]
robs_poss = [[0.,0.]]

robs_routes = [[i] for i in robs_poss]
obs_routes = [obs_poss[-1].copy()]
def calc_pot(x,y,obs_poss,goal_poss):
    out_pot = 0
    d = lambda x,y:np.sqrt(np.power(x[0]-y[0],2) + np.power(x[1]-y[1],2))

    for i,obs_pos in enumerate(obs_poss):
        vel = obs_vels[i]
        vel_dot = np.sqrt(vel[0]**2+vel[1]**2)*0.1
        theta = np.arctan2(vel[1],vel[0])
        ang = np.arctan2(y-obs_pos[1],x-obs_pos[0])
        von = 3*vonmises.pdf(theta-ang,vel_dot+1e-10) if vel_dot != 0 else 15
        von = 10
        # print(von,vel_dot)
        obs_pot = von*np.exp(-d(obs_pos,[x,y])/2.)/(2*np.pi)
        # print(f"obs:{obs_pot}")
        out_pot += WEIGHT_OBST * obs_pot
    for goal_pos in goal_poss:
        goal_pot = 0
        #Chap4-Potential-Field_howie.pdf
        goal_pot = np.where(d([x,y],goal_pos) < d_star,0.5*d([x,y],goal_pos)**2,d_star*d([x,y],goal_pos)-0.5*d_star**2)
        # print(f"gol:{goal_pot}")
        # goal_pot = -1. / np.sqrt(np.power(x-goal_pos[0],2) + np.power(y-goal_pos[1],2) + 10**-8)
        out_pot += WEIGHT_GOAL * goal_pot

    return out_pot

getcolum = lambda x,i:list(map(lambda y:y[i],x))
pot = lambda x,y:calc_pot(x,y,obs_poss,goal_poss)

#---contour------
X,Y = np.meshgrid(np.linspace(-2,15,100),np.linspace(15,-2,100))
Z = calc_pot(X,Y,obs_poss,goal_poss)

fig,ax = plt.subplots(1,1)
# fig = plt.figure(figsize=(10,6))
# ax = fig.add_subplot(111,projection="3d")
lines_obs = ax.scatter(getcolum(obs_poss,0),getcolum(obs_poss,1),color="red")
lines_goal = ax.scatter(getcolum(goal_poss,0),getcolum(goal_poss,1),color="blue")
cont_pot = ax.contour(X,Y,Z)
# ax.plot_wireframe(X,Y,Z,color="darkblue")
ax.set_xlim(-2,15)
ax.set_ylim(-2,15)

lines_rob = ax.scatter(getcolum(robs_poss,0),getcolum(robs_poss,1),color="green")
count = 0
while 1:
    lines_obs.remove()
    for i,obs_vel in enumerate(obs_vels):
        for k in range(2):
            obs_poss[i][k] += obs_vel[k] * 0.1
    # obs_poss[0][0] += 0.05;obs_poss[0][1] += 0.005
    # obs_poss[1][0] -= 0.1;obs_poss[1][1] -= 0.1
    # obs_poss[2][0] -= 0.1
    # obs_poss[3][0] += 0.1
    obs_routes.append(obs_poss[-1].copy())
    # Z = calc_pot(X,Y,np.concatenate([obs_poss,robs_poss]),goal_poss)
    Z = calc_pot(X,Y,obs_poss,goal_poss)

    plt.cla()
    ax.set_xlim(-2,15)
    ax.set_ylim(-2,15)
    ax.set_aspect("equal")

    ax.scatter(getcolum(goal_poss,0),getcolum(goal_poss,1),color="blue")
    # ax.plot_wireframe(X,Y,Z)
    ax.contour(X,Y,Z)

    lines_obs = ax.scatter(getcolum(obs_poss,0),getcolum(obs_poss,1),color="red")
    alldiff = []
    for index,(x,y) in enumerate(robs_poss):
        mask = np.ones(len(robs_poss),dtype=bool)
        mask[index] = False
        _obs_poss = np.concatenate([obs_poss,np.array(robs_poss)[mask]])

        vx = -(calc_pot(x+dt,y,_obs_poss,[goal_poss[index]]) - calc_pot(x-dt,y,_obs_poss,[goal_poss[index]]))/dt
        vy = -(calc_pot(x,y+dt,_obs_poss,[goal_poss[index]]) - calc_pot(x,y-dt,_obs_poss,[goal_poss[index]]))/dt

        v = math.sqrt(vx**2+vy**2)
        vx /= v/speed
        vy /= v/speed
        
        x += vx
        y += vy
        robs_poss[index][0] = x
        robs_poss[index][1] = y
        robs_routes[index].append([x,y])
        diff = math.sqrt((goal_poss[index][0]-x)**2+(goal_poss[index][1]-y)**2)
        alldiff.append(diff)
    
    
    # print(max(alldiff))
    
    ax.scatter(getcolum(robs_poss,0),getcolum(robs_poss,1),color="green")
    ax.scatter(getcolum(obs_routes,0),getcolum(obs_routes,1),color="red")
    for robs_route in robs_routes:
        ax.scatter(getcolum(robs_route,0),getcolum(robs_route,1),color="lime")
    plt.draw()
    plt.pause(0.01)
    # plt.savefig(f"imgs/{count}.png")
    count += 1
    if max(alldiff) < 0.2:
        break
