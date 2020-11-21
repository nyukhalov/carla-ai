from casadi import interpolant, vertcat, MX, types, tanh, cos, sin, Function
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosOcpCost, AcadosOcpConstraints, AcadosOcpOptions
import scipy.sparse.linalg
import time


class Config(object):
    def __init__(self):
        self.pred_horizon = 1.0  # prediction horizon [sec]
        self.pred_num_steps = 50  # number of discretization steps

        self.nsbx = 1  # ???
        self.nh = 5  # size of h
        self.nsh = self.nh
        self.ns = self.nsh + self.nsbx

        self.nx = 6  # number of states
        self.nu = 2  # number of control inputs

        self.ny = self.nx + self.nu
        self.ny_e = self.nx


def make_state(s: float, d: float, alpha: float, v: float, throttle: float, delta: float):
    return np.array([s, d, alpha, v, throttle, delta])


# have know idea what is kappa
# always return 0 for now
def kapparef_s(s: float) -> float:
    return 0


# define acados ODE
def make_model():
    # state
    s = MX.sym("s")  # longitudinal distance
    n = MX.sym("n")  # lateral distance
    alpha = MX.sym("alpha")  # heading ???
    v = MX.sym("v")  # velocity
    D = MX.sym("D")  # acceleration ???
    delta = MX.sym("delta")  # steering angle
    x = vertcat(s, n, alpha, v, D, delta)

    # xdot
    sdot = MX.sym("sdot")
    ndot = MX.sym("ndot")
    alphadot = MX.sym("alphadot")
    vdot = MX.sym("vdot")
    Ddot = MX.sym("Ddot")
    deltadot = MX.sym("deltadot")
    xdot = vertcat(sdot, ndot, alphadot, vdot, Ddot, deltadot)

    # controls
    derD = MX.sym("derD")
    derDelta = MX.sym("derDelta")
    u = vertcat(derD, derDelta)

    # algebraic variables
    z = vertcat([])

    # parameters
    p = vertcat([])

    ## Race car parameters
    m = 0.043
    C1 = 0.5
    C2 = 15.5
    Cm1 = 0.28
    Cm2 = 0.05
    Cr0 = 0.011
    Cr2 = 0.006

    # dynamics
    sdota = (v * cos(alpha + C1 * delta)) / (1 - kapparef_s(s) * n)
    Fxd = (Cm1 - Cm2 * v) * D - Cr2 * v * v - Cr0 * tanh(5 * v)
    f_expl = vertcat(
        sdota,  # s_dot
        v * sin(alpha + C1 * delta),  # n_dot
        v * C2 * delta - kapparef_s(s) * sdota,  # alpha_dot
        Fxd / m * cos(C1 * delta),  # v_dot, aka accel
        derD,  # D_dot
        derDelta,  # delta_dot
    )

    # constraint on forces
    a_lat = C2 * v * v * delta + Fxd * sin(C1 * delta) / m
    a_long = Fxd / m
    con_h_expr = vertcat(a_long, a_lat, n, D, delta)

    assert x.size()[0] == config.nx
    assert xdot.size()[0] == config.nx
    assert u.size()[0] == config.nu

    # https://docs.acados.org/interfaces/#acados_template.acados_model.AcadosModel
    model_ac = AcadosModel()
    model_ac.name = "BicycleModel"
    model_ac.x = x  # state of the system
    model_ac.xdot = xdot  # derivative of the state wrt time
    model_ac.u = u  # input of the system
    model_ac.p = p  # parameters of the DAE
    model_ac.z = z  # algebraic variables of the DAE
    model_ac.f_impl_expr = xdot - f_expl  # expression for the implicit dynamics F(xdot, x, u ,z) = 0
    model_ac.f_expl_expr = f_expl  # expression for the explicit dynamics xdot = f(x, u)
    model_ac.con_h_expr = con_h_expr  # expression for the constraint h
    return model_ac


def make_ocp_cost(config: Config) -> AcadosOcpCost:
    unscale = config.pred_num_steps / config.pred_horizon

    Q = np.diag([ 1e-1, 1e-8, 1e-8, 1e-8, 1e-3, 5e-3 ])
    Qe = np.diag([ 5e0, 1e1, 1e-8, 1e-8, 5e-3, 2e-3 ])
    R = np.eye(config.nu)
    R[0, 0] = 1e-3
    R[1, 1] = 5e-3

    Vx = np.zeros((config.ny, config.nx))
    Vx[:config.nx, :config.nx] = np.eye(config.nx)

    Vu = np.zeros((config.ny, config.nu))
    Vu[6, 0] = 1.0
    Vu[7, 1] = 1.0

    Vx_e = np.zeros((config.ny_e, config.nx))
    Vx_e[:config.nx, :config.nx] = np.eye(config.nx)

    # https://docs.acados.org/interfaces/#acados_template.acados_ocp.AcadosOcpCost
    cost = AcadosOcpCost()

    # linear least square
    cost.cost_type = "LINEAR_LS"
    cost.cost_type_e = "LINEAR_LS"

    cost.W = unscale * scipy.linalg.block_diag(Q, R)
    cost.W_e = Qe / unscale

    cost.Vx = Vx
    cost.Vu = Vu
    cost.Vx_e = Vx_e

    cost.zl = 100 * np.ones((config.ns,))
    cost.zu = 100 * np.ones((config.ns,))
    cost.Zl = 1 * np.ones((config.ns,))
    cost.Zu = 1 * np.ones((config.ns,))

    # set intial references
    cost.yref = np.array([1, 0, 0, 0, 0, 0, 0, 0])
    cost.yref_e = np.array([0, 0, 0, 0, 0, 0])

    return cost

def make_ocp_constraints(config: Config) -> AcadosOcpConstraints:
    # Model bounds
    n_min = -0.12  # width of the track [m]
    n_max = 0.12  # width of the track [m]

    # state bounds
    throttle_min = -1.0
    throttle_max = 1.0

    delta_min = -0.40  # minimum steering angle [rad]
    delta_max = 0.40  # maximum steering angle [rad]

    # input bounds
    ddelta_min = -2.0  # minimum change rate of stering angle [rad/s]
    ddelta_max = 2.0  # maximum change rate of steering angle [rad/s]
    dthrottle_min = -10  # -10.0  # minimum throttle change rate
    dthrottle_max = 10  # 10.0  # maximum throttle change rate

    # nonlinear constraint
    alat_min = -4  # maximum lateral force [m/s^2]
    alat_max = 4  # maximum lateral force [m/s^2]
    along_min = -4  # maximum longitudinal force [m/s^2]
    along_max = 4  # maximum longitudinal force [m/s^2]

    # input bounds
    ddelta_min = -2.0  # minimum change rate of stering angle [rad/s]
    ddelta_max = 2.0  # maximum change rate of steering angle [rad/s]
    dthrottle_min = -10  # -10.0  # minimum throttle change rate
    dthrottle_max = 10  # 10.0  # maximum throttle change rate

    # initial state
    x0 = make_state(
        s = 0,
        d = 0,
        alpha = 0,
        v = 0,
        throttle = 0,
        delta = 0
    )

    # https://docs.acados.org/interfaces/#acados_template.acados_ocp.AcadosOcpConstraints
    constraints = AcadosOcpConstraints()

    # lower/upper bounds on state
    constraints.lbx = np.array([-12])
    constraints.ubx = np.array([12])
    constraints.idxbx = np.array([1])

    # lower/upper boundas on controls
    constraints.lbu = np.array([dthrottle_min, ddelta_min])
    constraints.ubu = np.array([dthrottle_max, ddelta_max])
    constraints.idxbu = np.array([0, 1])

    constraints.lsbx = np.zeros([config.nsbx])  # lower bounds on slacks corresponding to soft lower bounds on x
    constraints.usbx = np.zeros([config.nsbx])  # upper bounds on slacks corresponding to soft upper bounds on x
    constraints.idxsbx = np.array(range(config.nsbx))  # indexes of soft bounds on x within the indices of bounds on x

    # lower/upper bounds for nonlinear inequalities h
    constraints.lh = np.array([along_min, alat_min, n_min, throttle_min, delta_min])
    constraints.uh = np.array([along_max, alat_max, n_max, throttle_max, delta_max])

    # lower/upper bounds on slacks corresponding to soft lower/upper bounds for nonlinear constraints
    constraints.lsh = np.zeros(config.nsh)
    constraints.ush = np.zeros(config.nsh)
    constraints.idxsh = np.array(range(config.nsh))

    # initial state
    constraints.x0 = x0

    return constraints

def make_ocp_options(config: Config) -> AcadosOcpOptions:
    # https://docs.acados.org/interfaces/#acados_template.acados_ocp.AcadosOcpOptions
    options = AcadosOcpOptions()

    # set QP solver and integration
    options.tf = config.pred_horizon  # prediction horizon
    options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    options.nlp_solver_type = "SQP"
    options.hessian_approx = "GAUSS_NEWTON"
    options.integrator_type = "ERK"
    options.sim_method_num_stages = 4
    options.sim_method_num_steps = 3
    # options.nlp_solver_step_length = 0.05
    options.nlp_solver_max_iter = 200
    options.tol = 1e-4
    # options.nlp_solver_tol_comp = 1e-1

    return options


def make_ocp(config: Config) -> AcadosOcp:
    ocp = AcadosOcp()  # OCP - Optimal Control Problem ?
    ocp.model = make_model()
    ocp.dims.N = config.pred_num_steps
    ocp.cost = make_ocp_cost(config)
    ocp.constraints = make_ocp_constraints(config)
    ocp.solver_options = make_ocp_options(config)
    return ocp


print("Initializing solver")

# create solver
config = Config()
ocp = make_ocp(config)
acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")

print("Running simulation")

T = 10.00  # maximum simulation time [sec]
Nsim = int(T * config.pred_num_steps / config.pred_horizon)

# initialize data structs
simX = np.ndarray((Nsim, config.nx))
simU = np.ndarray((Nsim, config.nu))
s0 = 0
tcomp_sum = 0
tcomp_max = 0
sref_N = 3  # reference for final reference progress

for i in range(100):
    # update reference
    sref = s0 + sref_N
    for step_no in range(config.pred_num_steps):
        progress = step_no / config.pred_num_steps
        yref_s = s0 + sref_N * progress
        yref_n = 0
        yref_alpha = 0
        yref_v = 0
        yref_D = 0
        yref_delta = 0
        yref_Ddot = 0
        yref_deltadot = 0
        yref = np.array([
            yref_s,
            yref_n,
            yref_alpha,
            yref_v,
            yref_D,
            yref_delta,
            yref_Ddot,
            yref_deltadot
        ])
        acados_solver.set(step_no, "yref", yref)

    yref_N = np.array([s0 + sref_N, 0, 0, 0, 0, 0])
    acados_solver.set(config.pred_num_steps, "yref", yref_N)

    # solve ocp
    t = time.time()
    status = acados_solver.solve()
    elapsed = time.time() - t

    if status != 0:
        print("acados returned status {} in closed loop iteration {}.".format(status, i))

    # manage timings
    tcomp_sum += elapsed
    if elapsed > tcomp_max:
        tcomp_max = elapsed

    # get solution
    x0 = acados_solver.get(0, "x")
    u0 = acados_solver.get(0, "u")
    for j in range(config.nx):
        simX[i, j] = x0[j]
    for j in range(config.nu):
        simU[i, j] = u0[j]

    # update initial condition
    x0 = acados_solver.get(1, "x")
    acados_solver.set(0, "lbx", x0)
    acados_solver.set(0, "ubx", x0)
    s0 = x0[0]

    # check if one lap is done and break and remove entries beyond
    if False and s0 > Sref[-1] + 0.1:
        # find where vehicle first crosses start line
        N0 = np.where(np.diff(np.sign(simX[:, 0])))[0][0]
        Nsim = i - N0  # correct to final number of simulation steps for plotting
        simX = simX[N0:i, :]
        simU = simU[N0:i, :]
        break

# Plot Results
#t = np.linspace(0.0, Nsim * config.pred_horizon / config.pred_num_steps, Nsim)
#plotRes(simX, simU, t)
#plotTrackProj(simX, track)
#plotalat(simX, simU, constraint, t)
#plt.show()

# Print some stats
print("Average computation time: {}".format(tcomp_sum / Nsim))
print("Maximum computation time: {}".format(tcomp_max))
print("Average speed:{} m/s".format(np.average(simX[:, 3])))
print("Lap time: {}s".format(config.pred_horizon * Nsim / config.pred_num_steps))
