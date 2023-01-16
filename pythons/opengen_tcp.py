import opengen as og
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.stats import vonmises

def visual_vector(loc, vector, color = "red"):
    ax.quiver(loc[0], loc[1],
              vector[0], vector[1], color = color,
              angles = 'xy', scale_units = 'xy', scale = 1)
def dynamics_ct(_x,_u):
    return [_u[0] * np.cos(_x[2]),
            _u[0] * np.sin(_x[2]),
            _u[1]]

N = 10 #予測ホライゾン
NX = 3 #状態ベクトルの次元
NU = 2 #入力ベクトルの次元
Nobs = 10 #obsの数
Nlines = 1#linesの数
Nnears = 2#nearsの数

d_star = 0.5
L = 0.5
sampling_time = 0.1
MAXCLOSE = 1.



mng = og.opengen.tcp.OptimizerTcpManager("optimization_engine/mpc_controller")
mng.start()

#TODO npかlistか統一する 
x_state_0 = np.array([0,0,0],dtype=np.float64)
x_goal = np.array([12,12,0],dtype=np.float64)
#TODO obsに変数変える
# _obs = np.array([[0,2],[12,4],[0,6],[12,8],[0,10]],dtype=np.float64)
# obs_vel = np.array([[2,-.0],[-2,-0.],[2,0],[-2,-0],[2,-0]],dtype=np.float64)
_obs = np.array([[5,i*0.5+2.5] for i in range(Nobs-1)],dtype=np.float64)
# _obs = np.array([[100,100] for _ in range(Nobs-1)],dtype=np.float64)
obs_vel = np.array([[0,0] for _ in range(Nobs-1)],dtype=np.float64)

# _obs = np.concatenate([_obs,np.array([[100,100] for _ in range(Nobs-5)],dtype=np.float64)])
# obs_vel = np.concatenate([obs_vel,np.array([[0,0] for _ in range(Nobs-5)],dtype=np.float64)])

# _obs = np.random.randint(-100,100,(Nobs,2))*0.1
# obs_vel = np.random.randint(-10,10,(Nobs,2))*0.1
# _obs = np.concatenate([_obs,[[8,6],[5,5]]])
# obs_vel = np.concatenate([obs_vel,[[0,0],[-0.7,-0.7]]])
_obs = np.concatenate([_obs,[[5,5]]])
obs_vel = np.concatenate([obs_vel,[[-2,-2]]])
_obs = np.apply_along_axis(lambda x:np.concatenate([x,[0]]),1,_obs).flatten()
obs_vel = np.apply_along_axis(lambda x:np.concatenate([x,[0]]),1,obs_vel).flatten()

obs_line = np.array([0,8,8,8],dtype=np.float64)

state_sequence = x_state_0
input_sequence = []
rob_routes = [[100,100]]
obs_routes = [[100,100]]
x = x_state_0
end_i = 0
cm_obs = plt.get_cmap("Reds")
cm_obs = [cm_obs(1.)]
cm_rob = plt.get_cmap("Greens")
cm_rob = [cm_rob(1.)]
print(cm_obs)
getcolum = lambda x,i:list(map(lambda y:y[i],x))

fig,ax = plt.subplots(1,1)
times = []

while 1:
    #*-----------visualize------------
    ax.scatter(x_goal[0],x_goal[1],color="blue")
    for i in range(Nobs):
        __obs = _obs[i*3:i*3+3]
        ax.scatter(__obs[0],__obs[1],color="red")
        ax.add_patch(patches.Circle((__obs[0],__obs[1]),radius=MAXCLOSE,fill=False))
    ax.scatter(x[0],x[1],color="green")
    ax.scatter(getcolum(rob_routes,0),getcolum(rob_routes,1),c="lime")
    ax.scatter(getcolum(obs_routes,0),getcolum(obs_routes,1),c="red")
    visual_vector(x[:2],[np.cos(x[2]),np.sin(x[2])])
    # visual_vector(obs_line[:2],obs_line[2:]-obs_line[:2])
    ax.set_xlim(-2,15)
    ax.set_ylim(-2,15)
    ax.set_aspect("equal")
    plt.draw()
    # plt.savefig(f"imgs/{end_i}.png")
    plt.pause(10**-8)
    plt.cla()
    #* ----------process data------------
    _obs_array = _obs.reshape([Nobs,3])
    obs_vel_array = obs_vel.reshape([Nobs,3])
    
    diff = np.apply_along_axis(lambda o:(o[0]-x[0])**2+(o[1]-x[1])**2,1,_obs_array)
    index_near = np.argsort(diff)[:2]

    # index_near = get_near_obs(x,_obs_array,2)
    obs_nears = np.array(_obs_array[index_near],dtype=np.float64)
    # ax.scatter(getcolum(obs_nears,0),getcolum(obs_nears,1),color="yellow")
    obs_nears = obs_nears.flatten()
    obs_nears_vel = np.array(obs_vel_array[index_near],dtype=np.float64).flatten()
    vel_dot = np.apply_along_axis(lambda o:np.sqrt(o[0]**2+o[1]**2),1,obs_vel_array)
    obs_rob_rad = np.apply_along_axis(lambda o:np.arctan2(x[1]-o[1],x[0]-o[0]),1,_obs_array)
    vel_theta = np.apply_along_axis(lambda o:np.arctan2(o[1],o[0]),1,obs_vel_array)
    vel_von_fnc = np.frompyfunc(lambda d,t,r:50*vonmises.pdf(t-r,d) if d != 0 else 1.,3,1)
    vel_von = vel_von_fnc(vel_dot,vel_theta,obs_rob_rad)
    #http://www.robot.t.u-tokyo.ac.jp/~yamashita/paper/E/E318Final.pdf
    # print(vel_von)
    # vel_von = np.array([1 for _ in range(Nobs)])
    #* -----------solver------------
    print(",".join(map(str,np.concatenate([x,x_goal,_obs,obs_vel,obs_nears,obs_nears_vel]))))
    solver_status = mng.call(np.concatenate([x,x_goal,_obs,obs_vel,obs_nears,obs_nears_vel,vel_von]))#,obs_line
    calc_wall = 0
    if solver_status.is_ok():
        solution_data = solver_status.get()
        solver_time = solution_data.solve_time_ms
        if solver_time <= 300:
            times.append(solver_time)
        print(solver_time,solver_status["solution"][-1])
    us = solver_status["solution"]
    u = us[:2] #first u

    #* ----------calc predict------------
    x_pre = []
    _x = x
    for i in range(N):
        _u = us[i*2:i*2+2]
        _dx = dynamics_ct(_x,_u)
        _x = [_x[j] + sampling_time * _dx[j] for j in range(NX)]
        x_pre.append(_x)
    ax.scatter(getcolum(x_pre,0),getcolum(x_pre,1),marker="x")
    #* ----------calc dynamics-----------
    dx = dynamics_ct(x,u)
    x_next = [x[j] + sampling_time * dx[j] for j in range(NX)]

    diff = np.sqrt(np.power(x_next[0]-x_goal[0],2) + np.power(x_next[1]-x_goal[1],2))
    state_sequence += x_next
    input_sequence += [u]
    rob_routes.append(x_next)
    obs_routes.append(_obs.reshape([Nobs,3])[-1].tolist())
    cm_obs_interval = [i / (len(obs_routes)-1) for i in range(len(obs_routes))]
    cm_rob_interval = [i / (len(rob_routes)-1) for i in range(len(rob_routes))]
    cm_obs = plt.get_cmap("Reds")(cm_obs_interval)
    cm_rob = plt.get_cmap("Greens")(cm_rob_interval)
    x = x_next

    #* ------------move obs----------------
    for i in range(Nobs):
        _obs[i*3:i*3+3] += obs_vel[i*3:i*3+3] * sampling_time
    #*------------------------------------
    end_i += 1
    # print(diff)
    if diff <= 0.13:
        break
mng.kill()


time = np.arange(0,sampling_time*end_i,sampling_time)

getcolum = lambda x,i:list(map(lambda y:y[i],x))
print(sum(times)/len(times))
plt.subplot(211)
plt.plot(time, getcolum(input_sequence,0), '-o')
plt.ylabel('v')
plt.subplot(212)
plt.plot(time, getcolum(input_sequence,1), '-o')
plt.ylabel('w')
plt.xlabel('Time')
plt.show()

# ave 31.18743455882351 ms
# params 147

# debug
# 15.906279850746268 ms
# params 39

# release
# 4.334221739130435 ms
# params 39