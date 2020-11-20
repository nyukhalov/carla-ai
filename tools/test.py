from casadi import interpolant, vertcat, MX, types, tanh, cos, sin, Function
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosOcpCost, AcadosOcpConstraints, AcadosOcpOptions
import scipy.sparse.linalg

class Config(object):
    def __init__(self):
        self.Tf = 1.0  # prediction horizon
        self.N = 50  # number of discretization steps
        self.T = 10.00  # maximum simulation time[s]
        self.Nsim = int(T * N / Tf)

        self.nsbx = 1  # ???
        self.nh = 5  # size of h
        self.nsh = self.nh
        self.ns = self.nsh + self.nsbx

        self.nx = 6  # number of states
        self.nu = 2  # number of control inputs

        self.ny = self.nx + self.nu
        self.ny_e = self.nx



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
    Fxd = (Cm1 - Cm2 * v) * D - Cr2 * v * v - Cr0 * tanh(5 * v)
    sdota = (v * cos(alpha + C1 * delta)) / (1 - kapparef_s(s) * n)
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
    unscale = config.N / config.Tf

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
    # https://docs.acados.org/interfaces/#acados_template.acados_ocp.AcadosOcpConstraints
    constraints = AcadosOcpConstraints()

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
    x0 = np.array([-2, 0, 0, 0, 0, 0])

    constraints.lbx = np.array([-12])  # lower bounds on x
    constraints.ubx = np.array([12])  # upper bounds on x
    constraints.idxbx = np.array([1])  # indexes of bounds on x (defines Jbx)

    constraints.lbu = np.array([dthrottle_min, ddelta_min])  # lower bounds on u
    constraints.ubu = np.array([dthrottle_max, ddelta_max])  # upper bounds on u
    constraints.idxbu = np.array([0, 1])  # indexes of bounds on u (defines Jbu)

    constraints.lsbx = np.zeros([config.nsbx])  # lower bounds on slacks corresponding to soft lower bounds on x
    constraints.usbx = np.zeros([config.nsbx])  # upper bounds on slacks corresponding to soft upper bounds on x
    constraints.idxsbx = np.array(range(config.nsbx))  # indexes of soft bounds on x within the indices of bounds on x

    # lower bound for nonlinear inequalities h
    constraints.lh = np.array(
        [
            along_min,
            alat_min,
            n_min,
            throttle_min,
            delta_min,
        ]
    )
    # upper bound for nonlinear inequalities h
    constraints.uh = np.array(
        [
            along_max,
            alat_max,
            n_max,
            throttle_max,
            delta_max,
        ]
    )
    # lower/upper bounds on slacks corresponding to soft lower/upper bounds for nonlinear constraints
    constraints.lsh = np.zeros(config.nsh)
    constraints.ush = np.zeros(config.nsh)
    # bounds on slacks corresponding to soft lower bounds for nonlinear constraints
    constraints.idxsh = np.array(range(config.nsh))

    # initial state
    constraints.x0 = x0

    return constraints

def make_ocp_options(config: Config) -> AcadosOcpOptions:
    # https://docs.acados.org/interfaces/#acados_template.acados_ocp.AcadosOcpOptions
    options = AcadosOcpOptions()

    # set QP solver and integration
    options.tf = config.Tf
    # options.qp_solver = 'FULL_CONDENSING_QPOASES'
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
    ocp.dims.N = config.N  # discretization
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

# initialize data structs
simX = np.ndarray((config.Nsim, config.nx))
simU = np.ndarray((config.Nsim, config.nu))
s0 = model.x0[0]
tcomp_sum = 0
tcomp_max = 0

for i in range(Nsim):


print("Done")
