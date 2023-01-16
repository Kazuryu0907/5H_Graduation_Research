import casadi.casadi as cs
import opengen as og
#https://56.gigafile.nu/0111-b2f50d090dcb1e1141261d3059ca6f789
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


Q = cs.DM.eye(NX) * [1.0, 1.0, 0.1]
R = cs.DM.eye(NU) * [0.1,1.]
QN = cs.DM.eye(NX) * [20.,20.,2]


def dynamics_ct(_x,_u):
    return [_u[0] * cs.cos(_x[2]),
            _u[0] * cs.sin(_x[2]),
            _u[1]]

def dynamics_dt(x,u):
    dx = cs.vcat(dynamics_ct(x,u))
    return cs.vcat([x[i] + sampling_time * dx[i] for i in range(NX)])


def stage_cost(_x,_u,_x_ref=None,_u_ref=None,obs=None,_pri_u=None,nears=None):
    if _x_ref is None:
        _x_ref = cs.DM.zeros(_x.shape)
    if _u_ref is None:
        _u_ref = cs.DM.zeros(_u.shape)
    if obs is None:
        obs = cs.DM.zeros(_x.shape)
    dx = _x - _x_ref
    # du = _u - _u_ref
    du = _u
    dudt = _u - _pri_u
    cost = 0

    d = cs.sqrt(cs.mtimes([dx.T,cs.DM.eye(NX)*[1,1,0],dx]))

    f = cs.if_else(d<=d_star,0.5*d**2,d_star*d-0.5*d_star**2)

    # VecA = obs_line[:2];VecB = obs_line[2:];VecP = _x[:2]
    # Veca = VecB - VecA
    # Vecb = VecP - VecA
    # r_wall = cs.dot(Veca,Vecb)/cs.dot(Veca,Veca)
    # nearest_wall = cs.if_else(r_wall <= 0,VecA,cs.if_else(r_wall >= 1,VecB,VecA + r_wall*Veca))
    # d_wall = cs.mtimes([(_x[:2]-nearest_wall).T,cs.DM.eye(2),(_x[:2]-nearest_wall)])
    # wall = 100*cs.exp(-d_wall/2.)
    for i in range(Nobs):
        _obs = obs[i*3:i*3+3]
        #http://www.robot.t.u-tokyo.ac.jp/~yamashita/paper/E/E318Final.pdf
        dx_obs = _x - _obs
        #* 運動方向に伸ばす
        # R = (1+cs.fabs(_obs_vel[0]))*cs.power(dx_obs[0],2) + (1+cs.fabs(_obs_vel[1]))*cs.power(dx_obs[1],2)
        # Rcost = cs.DM.eye(3)*[1,1,0]*_obs_abs_vel
        # Up = cs.if_else(cs.mtimes([_obs.T,cs.DM.eye(NX)*[1,1,0],_obs]) == 0,0,100)
        cost += 100*cs.exp(-cs.mtimes([dx_obs.T,cs.DM.eye(NX)*[1,1,0],dx_obs])/2.)*obs_von[i]
        # cost += 100./cs.sqrt(cs.mtimes([dx_obs.T,cs.DM.eye(NX)*[1,1,0],dx_obs]))
    return 10*cs.mtimes([dx.T,Q,dx]) + 10*cs.mtimes([dudt.T,cs.DM.eye(NU),dudt]) + cost + cs.mtimes([du.T,R,du]) + f # + wall+ 10*cs.mtimes([dudt.T,cs.DM.eye(NU),dudt])
    # return 10*cs.mtimes([dx.T,Q,dx]) #+ wall#

#終端コスト
def terminal_cost(_x,x_0,_x_ref=None):
    if _x_ref is None:
        _x_ref = cs.DM.zeros(_x.shape)
    dx = _x - _x_ref
    return cs.mtimes([dx.T,QN,dx])#+ 1./cs.sqrt(cs.norm_2(dxx0))
    
x_0 = cs.MX.sym("x_0",NX)
x_ref = cs.MX.sym("x_ref",NX)
u_k = [cs.MX.sym("u_"+str(i),NU) for i in range(N)]
obs = cs.MX.sym("obs",NX*Nobs)
obs_vel = cs.MX.sym("obs_vec",NX*Nobs)
obs_line = cs.MX.sym("obs_line",4*Nlines) # x,y,x',y'
f_pot = cs.MX.sym("f_pot",1)
obs_nears = cs.MX.sym("obs_nears",Nnears*NX)
obs_nears_vel = cs.MX.sym("obs_nears_vel",Nnears*NX)
obs_von = cs.MX.sym("obs_von",Nobs)
s = cs.MX.sym("s",1)
#コスト関数作成
x_t = x_0
total_cost = 0
pri_u = cs.DM.zeros(u_k[0].shape)
obs_next = cs.MX(NX*Nobs)
obs_next = obs[:]
obs_nears_next = cs.MX(NX*Nnears)
obs_nears_next = obs_nears[:]
c = []
for t in range(N-1):
    total_cost += stage_cost(x_t,u_k[t],x_ref,obs=obs_next,_pri_u=pri_u)
    x_t = dynamics_dt(x_t,u_k[t])
    pri_u = u_k[t]
    f = cs.MX(1e+3)
    for i in range(Nobs):
        obs_next[i*3:i*3+3] = obs_next[i*3:i*3+3] + obs_vel[i*3:i*3+3]*sampling_time
        # f = cs.fmin(f,cs.mtimes([(obs_next[i*3:i*3+3]-x_t).T,cs.DM.eye(NX)*[1,1,0],obs_next[i*3:i*3+3]-x_t]))
    for i in range(Nnears):
        obs_nears_next[i*3:i*3+3] = obs_nears_next[i*3:i*3+3] + obs_nears_vel[i*3:i*3+3]*sampling_time
        # f = cs.mtimes([(obs_nears_next[i*3:i*3+3]-x_t).T,cs.DM.eye(NX)*[1,1,0],obs_nears_next[i*3:i*3+3]-x_t])
        # c.append(cs.fmax(0,-f+MAXCLOSE**2))
        f = cs.fmin(f,cs.mtimes([(obs_nears_next[i*3:i*3+3]-x_t).T,cs.DM.eye(NX)*[1,1,0],obs_nears_next[i*3:i*3+3]-x_t]))
    
    c.append(cs.fmax(0,-f+MAXCLOSE**2))
# print(c)


total_cost += terminal_cost(x_t,x_0,x_ref)

optimization_variables = []
optimization_parameters = []

optimization_variables += u_k
# optimization_variables += [s]
# optimization_variables += [f_pot]

optimization_parameters += [x_0]
optimization_parameters += [x_ref]
optimization_parameters += [obs]
optimization_parameters += [obs_vel]
optimization_parameters += [obs_nears]
optimization_parameters += [obs_nears_vel]
optimization_parameters += [obs_von]

#? 使ってない制約条件
# bigM = 1e+5
# eps = 1e-5

# d = cs.mtimes([(x_ref-x_t).T,cs.DM.eye(NX)*[1,1,0],x_ref-x_t])
# gx = d - d_star**2
# bx = d_star*cs.sqrt(d)-0.5*d_star**2
# c = cs.vertcat(cs.fmax(0.0,-bigM*(1-z)-gx),cs.fmax(0.0,eps - bigM*z + gx),
#                cs.fmax(0.0,-bigM*(1-z)-(f_pot-0.5*d)),cs.fmax(0.0,f_pot-0.5*d-bigM*(1-z)),
#                cs.fmax(0.0,-bigM*z-(f_pot-bx)),cs.fmax(0.0,f_pot-bx-bigM*z))
#?---------------------

# c = []

# f = cs.MX(1e+10)

# for i in range(Nobs):
#     _obs = obs[i*3:i*3+3]
#     f = cs.fmin(f,cs.mtimes([(_obs-x_t).T,cs.DM.eye(NX)*[1,1,0],_obs-x_t]))
#     # c.append(cs.fmax(0.,-cs.mtimes([(_obs-x_t).T,cs.DM.eye(NX)*[1,1,0],_obs-x_t])+MAXCLOSE**2))
# # c = cs.vertcat(*c)
# c = cs.fmax(0,-f+MAXCLOSE**2)
c = cs.vertcat(*c)

optimization_variables = cs.vertcat(*optimization_variables)
optimization_parameters = cs.vertcat(*optimization_parameters) 
#https://70.gigafile.nu/0116-ccf854bdb7267fe3362bad02c4876a6b3
umin = [-2,-1]*N
umax = [2,1]*N
bounds = og.opengen.constraints.Rectangle(umin,umax)

# binary = og.opengen.constraints.FiniteSet([[0],[1]])
# free = og.opengen.constraints.Rectangle([0],[0.01])
# bounds = og.opengen.constraints.CartesianProduct([2*N-1,2*N],[rect,free])
problem = og.opengen.builder.Problem(optimization_variables,
                                    optimization_parameters,
                                    total_cost)\
                            .with_constraints(bounds)\
                            .with_penalty_constraints(c)

                            
ros_config = og.opengen.config.RosConfiguration()\
    .with_package_name("open_mpc_controller")\
    .with_node_name("open_mpc_controller_node")\
    .with_rate((int)(1./sampling_time))\
    .with_description("Cool ROS node.")
build_config = og.opengen.config.BuildConfiguration()\
    .with_build_directory("optimization_engine")\
    .with_build_mode("release")\
    .with_ros(ros_config)\
    # .with_tcp_interface_config()
    # .with_ros(ros_config)\
    # .with_tcp_interface_config()
    # .with_build_c_bindings()\
    # .with_ros(ros_config)\
    # .with_tcp_interface_config()
    # .with_build_python_bindings()\
    


meta = og.opengen.config.OptimizerMeta()\
    .with_optimizer_name("mpc_controller")
solver_config = og.opengen.config.SolverConfiguration()\
    .with_tolerance(1e-5)\
    .with_initial_penalty(1.0)\
    .with_max_duration_micros(500000)
builder = og.opengen.builder.OpEnOptimizerBuilder(problem,
                                                meta,
                                                build_config,
                                                solver_config)
builder.build()
